#!/usr/bin/env python3
"""Read-only Airy/FAST-LIO/MAVROS/PX4 readiness monitor."""

import math
import threading
from collections import deque

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import EstimatorStatus, State, StatusText, TimesyncStatus
from nav_msgs.msg import Odometry


class Stream:
    def __init__(self, window=10.0):
        self.window = float(window)
        self.receipts = deque(maxlen=1000)
        self.last_message = None

    def add(self, message, now):
        now_sec = now.to_sec()
        self.receipts.append(now_sec)
        self.last_message = message
        self.trim(now_sec)

    def trim(self, now_sec):
        while self.receipts and now_sec - self.receipts[0] > self.window:
            self.receipts.popleft()

    def rate(self, now_sec):
        self.trim(now_sec)
        if len(self.receipts) < 2:
            return 0.0
        span = self.receipts[-1] - self.receipts[0]
        return 0.0 if span <= 0.0 else (len(self.receipts) - 1) / span

    def age(self, now_sec):
        return float("inf") if not self.receipts else now_sec - self.receipts[-1]


class AiryPx4Monitor:
    def __init__(self):
        self.lock = threading.RLock()
        self.uav_name = rospy.get_param("~uav_name", "liu").strip("/")
        prefix = "/{}/mavros".format(self.uav_name)

        self.source_topic = rospy.get_param("~source_topic", "/Odometry")
        self.vision_topic = rospy.get_param(
            "~vision_topic", prefix + "/vision_pose/pose"
        )
        self.state_topic = rospy.get_param("~state_topic", prefix + "/state")
        self.estimator_topic = rospy.get_param(
            "~estimator_topic", prefix + "/estimator_status"
        )
        self.timesync_topic = rospy.get_param(
            "~timesync_topic", prefix + "/timesync_status"
        )
        self.local_pose_topic = rospy.get_param(
            "~local_pose_topic", prefix + "/local_position/pose"
        )
        self.statustext_topic = rospy.get_param(
            "~statustext_topic", prefix + "/statustext/recv"
        )
        self.bridge_diagnostics_topic = rospy.get_param(
            "~bridge_diagnostics_topic", "/airy_px4/bridge/diagnostics"
        )
        self.output_topic = rospy.get_param(
            "~diagnostics_topic", "/airy_px4/monitor/diagnostics"
        )

        self.source_min_rate_hz = float(rospy.get_param("~source_min_rate_hz", 8.0))
        self.flight_min_pose_rate_hz = float(
            rospy.get_param("~flight_min_pose_rate_hz", 30.0)
        )
        self.max_data_age_s = float(rospy.get_param("~max_data_age_s", 0.5))
        self.max_timesync_age_s = float(rospy.get_param("~max_timesync_age_s", 2.0))
        self.max_timesync_rtt_ms = float(
            rospy.get_param("~max_timesync_rtt_ms", 20.0)
        )
        self.max_timesync_offset_std_ms = float(
            rospy.get_param("~max_timesync_offset_std_ms", 5.0)
        )
        self.direction_test_confirmed = bool(
            rospy.get_param("~direction_test_confirmed", False)
        )

        self.source = Stream()
        self.vision = Stream()
        self.state = Stream()
        self.estimator = Stream()
        self.timesync = Stream()
        self.local_pose = Stream()
        self.bridge = Stream()
        self.timesync_offsets_ns = deque(maxlen=50)
        self.recent_fcu_fault = ""
        self.recent_fcu_fault_time = None
        self.last_summary = ""

        self.publisher = rospy.Publisher(
            self.output_topic, DiagnosticArray, queue_size=10, latch=True
        )
        rospy.Subscriber(self.source_topic, Odometry, self._source_cb, queue_size=30)
        rospy.Subscriber(self.vision_topic, PoseStamped, self._vision_cb, queue_size=30)
        rospy.Subscriber(self.state_topic, State, self._state_cb, queue_size=20)
        rospy.Subscriber(
            self.estimator_topic, EstimatorStatus, self._estimator_cb, queue_size=20
        )
        rospy.Subscriber(
            self.timesync_topic, TimesyncStatus, self._timesync_cb, queue_size=20
        )
        rospy.Subscriber(
            self.local_pose_topic, PoseStamped, self._local_pose_cb, queue_size=20
        )
        rospy.Subscriber(
            self.statustext_topic, StatusText, self._statustext_cb, queue_size=50
        )
        rospy.Subscriber(
            self.bridge_diagnostics_topic,
            DiagnosticArray,
            self._bridge_cb,
            queue_size=20,
        )
        self.timer = rospy.Timer(rospy.Duration(1.0), self._timer_cb)
        rospy.logwarn(
            "Airy/PX4 monitor is read-only; it never changes mode, arming, or parameters"
        )

    def _add(self, stream, message):
        with self.lock:
            stream.add(message, rospy.Time.now())

    def _source_cb(self, message):
        self._add(self.source, message)

    def _vision_cb(self, message):
        self._add(self.vision, message)

    def _state_cb(self, message):
        self._add(self.state, message)

    def _estimator_cb(self, message):
        self._add(self.estimator, message)

    def _timesync_cb(self, message):
        with self.lock:
            self.timesync.add(message, rospy.Time.now())
            self.timesync_offsets_ns.append(float(message.estimated_offset_ns))

    def _local_pose_cb(self, message):
        self._add(self.local_pose, message)

    def _bridge_cb(self, message):
        self._add(self.bridge, message)

    def _statustext_cb(self, message):
        text = message.text.strip()
        lower = text.lower()
        keywords = (
            "preflight fail",
            "ekf",
            "vision timeout",
            "vision data",
            "position estimate",
            "sensor failure",
        )
        if message.severity <= StatusText.WARNING and any(word in lower for word in keywords):
            with self.lock:
                self.recent_fcu_fault = text
                self.recent_fcu_fault_time = rospy.Time.now()
                rospy.logwarn("PX4 status text: %s", text)

    @staticmethod
    def _bridge_status(message):
        if message is None:
            return None
        for status in message.status:
            if status.name == "airy_px4/fastlio_to_mavros_bridge":
                return status
        return None

    @staticmethod
    def _pose_is_finite(message):
        if message is None:
            return False
        pose = message.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            return False
        quaternion_norm = math.sqrt(sum(float(value) ** 2 for value in values[3:]))
        return 0.90 <= quaternion_norm <= 1.10

    def _evaluate(self, now):
        now_sec = now.to_sec()
        source_rate = self.source.rate(now_sec)
        vision_rate = self.vision.rate(now_sec)
        state = self.state.last_message
        estimator = self.estimator.last_message
        timesync = self.timesync.last_message
        bridge = self._bridge_status(self.bridge.last_message)

        connected = bool(
            state and self.state.age(now_sec) < 2.0 and state.connected
        )
        source_fresh = (
            self.source.age(now_sec) <= self.max_data_age_s
            and source_rate >= self.source_min_rate_hz
        )
        vision_fresh = (
            self.vision.age(now_sec) <= self.max_data_age_s
            and vision_rate >= self.source_min_rate_hz
            and self._pose_is_finite(self.vision.last_message)
        )
        flight_rate_ok = vision_rate >= self.flight_min_pose_rate_hz
        local_pose_ok = (
            self.local_pose.age(now_sec) <= self.max_data_age_s
            and self._pose_is_finite(self.local_pose.last_message)
        )
        bridge_ok = bool(
            bridge
            and bridge.level == DiagnosticStatus.OK
            and bridge.message == "PUBLISHING_NATIVE_RATE"
            and self.bridge.age(now_sec) <= 2.0
        )

        estimator_ok = False
        if estimator is not None and self.estimator.age(now_sec) <= 2.0:
            estimator_ok = bool(
                estimator.attitude_status_flag
                and estimator.velocity_horiz_status_flag
                and estimator.velocity_vert_status_flag
                and (
                    estimator.pos_horiz_rel_status_flag
                    or estimator.pos_horiz_abs_status_flag
                )
                and (
                    estimator.pos_vert_abs_status_flag
                    or estimator.pos_vert_agl_status_flag
                )
                and not estimator.const_pos_mode_status_flag
                and not estimator.gps_glitch_status_flag
                and not estimator.accel_error_status_flag
            )

        rtt_ms = float("inf")
        offset_std_ms = float("inf")
        timesync_ok = False
        if timesync is not None and self.timesync.age(now_sec) <= self.max_timesync_age_s:
            rtt_ms = float(timesync.round_trip_time_ms)
            if len(self.timesync_offsets_ns) >= 5:
                offset_std_ms = float(np.std(self.timesync_offsets_ns)) / 1.0e6
            timesync_ok = bool(
                math.isfinite(rtt_ms)
                and rtt_ms <= self.max_timesync_rtt_ms
                and math.isfinite(offset_std_ms)
                and offset_std_ms <= self.max_timesync_offset_std_ms
            )

        fcu_fault_active = bool(
            self.recent_fcu_fault_time
            and (now - self.recent_fcu_fault_time).to_sec() < 30.0
        )
        technical_ready = bool(
            connected
            and source_fresh
            and vision_fresh
            and flight_rate_ok
            and local_pose_ok
            and bridge_ok
            and estimator_ok
            and timesync_ok
            and not fcu_fault_active
        )
        position_ready = technical_ready and self.direction_test_confirmed

        if position_ready:
            level, summary = DiagnosticStatus.OK, "POSITION_DATA_READY"
        elif technical_ready:
            level, summary = DiagnosticStatus.WARN, "DIRECTION_TEST_NOT_CONFIRMED"
        elif vision_fresh and bridge_ok and (not estimator_ok or not local_pose_ok):
            level, summary = DiagnosticStatus.ERROR, "PX4_NOT_ACCEPTING_VISION"
        elif vision_fresh and bridge_ok and not timesync_ok:
            level, summary = DiagnosticStatus.ERROR, "TIMESYNC_UNHEALTHY"
        elif vision_fresh and bridge_ok and fcu_fault_active:
            level, summary = DiagnosticStatus.ERROR, "PX4_STATUS_FAULT"
        elif vision_fresh and bridge_ok and not flight_rate_ok:
            level, summary = DiagnosticStatus.WARN, "BENCH_ONLY_POSE_RATE_TOO_LOW"
        elif connected and source_fresh:
            level, summary = DiagnosticStatus.WARN, "DIAGNOSTIC_LINK_OK_NOT_POSITION_READY"
        else:
            level, summary = DiagnosticStatus.ERROR, "NOT_READY"

        values = {
            "position_data_ready": position_ready,
            "direction_test_confirmed": self.direction_test_confirmed,
            "fcu_connected": connected,
            "fcu_armed": bool(state and state.armed),
            "fcu_mode": state.mode if state else "unknown",
            "fastlio_fresh": source_fresh,
            "fastlio_rate_hz": "{:.2f}".format(source_rate),
            "vision_fresh": vision_fresh,
            "vision_rate_hz": "{:.2f}".format(vision_rate),
            "required_flight_pose_rate_hz": "{:.1f}".format(
                self.flight_min_pose_rate_hz
            ),
            "bridge_ok": bridge_ok,
            "bridge_state": bridge.message if bridge else "missing",
            "px4_estimator_ready": estimator_ok,
            "px4_local_pose_fresh": local_pose_ok,
            "timesync_ok": timesync_ok,
            "timesync_rtt_ms": (
                "unknown" if not math.isfinite(rtt_ms) else "{:.3f}".format(rtt_ms)
            ),
            "timesync_offset_std_ms": (
                "unknown"
                if not math.isfinite(offset_std_ms)
                else "{:.3f}".format(offset_std_ms)
            ),
            "recent_px4_fault": self.recent_fcu_fault if fcu_fault_active else "none",
        }
        return level, summary, values

    def _timer_cb(self, _event):
        now = rospy.Time.now()
        with self.lock:
            level, summary, values = self._evaluate(now)
            status = DiagnosticStatus()
            status.level = level
            status.name = "airy_px4/readiness"
            status.message = summary
            status.hardware_id = "robosense_airy_px4"
            status.values = [
                KeyValue(key=str(key), value=str(value))
                for key, value in sorted(values.items())
            ]
            message = DiagnosticArray()
            message.header.stamp = now
            message.status = [status]
            self.publisher.publish(message)
            compact = "{} | FCU={} armed={} mode={} | LIO={}Hz vision={}Hz | bridge={}".format(
                summary,
                values["fcu_connected"],
                values["fcu_armed"],
                values["fcu_mode"],
                values["fastlio_rate_hz"],
                values["vision_rate_hz"],
                values["bridge_state"],
            )
            if compact != self.last_summary:
                if level == DiagnosticStatus.OK:
                    rospy.loginfo(compact)
                elif level == DiagnosticStatus.WARN:
                    rospy.logwarn(compact)
                else:
                    rospy.logerr(compact)
                self.last_summary = compact


def main():
    rospy.init_node("airy_px4_readiness_monitor", anonymous=False)
    AiryPx4Monitor()
    rospy.spin()


if __name__ == "__main__":
    main()
