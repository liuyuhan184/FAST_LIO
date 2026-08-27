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

    def count(self, now_sec):
        self.trim(now_sec)
        return len(self.receipts)


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
        self.local_pose_min_rate_hz = float(
            rospy.get_param("~local_pose_min_rate_hz", 2.0)
        )
        self.min_local_pose_samples = max(
            2, int(rospy.get_param("~min_local_pose_samples", 5))
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
        self.position_mode_accepted_once = False
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
        with self.lock:
            self.state.add(message, rospy.Time.now())
            if message.mode == "POSCTL":
                self.position_mode_accepted_once = True

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
        local_pose_rate = self.local_pose.rate(now_sec)
        local_pose_samples = self.local_pose.count(now_sec)
        source_age = self.source.age(now_sec)
        vision_age = self.vision.age(now_sec)
        local_pose_age = self.local_pose.age(now_sec)
        estimator_age = self.estimator.age(now_sec)
        bridge_age = self.bridge.age(now_sec)
        state = self.state.last_message
        estimator = self.estimator.last_message
        timesync = self.timesync.last_message
        bridge = self._bridge_status(self.bridge.last_message)

        connected = bool(
            state and self.state.age(now_sec) < 2.0 and state.connected
        )
        source_fresh = (
            source_age <= self.max_data_age_s
            and source_rate >= self.source_min_rate_hz
        )
        vision_fresh = (
            vision_age <= self.max_data_age_s
            and vision_rate >= self.source_min_rate_hz
            and self._pose_is_finite(self.vision.last_message)
        )
        flight_rate_ok = vision_rate >= self.flight_min_pose_rate_hz
        local_pose_fresh = (
            local_pose_age <= self.max_data_age_s
            and self._pose_is_finite(self.local_pose.last_message)
        )
        local_pose_continuous = bool(
            local_pose_fresh
            and local_pose_rate >= self.local_pose_min_rate_hz
            and local_pose_samples >= self.min_local_pose_samples
        )
        bridge_diagnostics_fresh = bool(bridge and bridge_age <= 2.0)
        bridge_state = bridge.message if bridge else "missing"
        bridge_ok = bool(
            bridge_diagnostics_fresh
            and bridge.level == DiagnosticStatus.OK
            and bridge_state == "PUBLISHING_NATIVE_RATE"
        )
        bridge_gate_blocked = bool(
            bridge_diagnostics_fresh and bridge_state == "BLOCKED_BY_STARTUP_GATE"
        )
        bridge_waiting = bool(
            bridge_diagnostics_fresh
            and bridge_state in ("WAITING_FASTLIO", "WAITING_STATIONARY_ALIGNMENT")
        )
        bridge_fault = bool(
            bridge_diagnostics_fresh
            and not bridge_ok
            and not bridge_gate_blocked
            and not bridge_waiting
        )

        estimator_fresh = bool(estimator is not None and estimator_age <= 2.0)
        solution_flags_ok = False
        if estimator_fresh:
            solution_flags_ok = bool(
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

        # MAVLink ESTIMATOR_STATUS exposes source-agnostic solution-validity
        # flags and does not prove that external vision is fused.  PX4's
        # reported POSCTL mode does directly prove that Commander accepted
        # Position at least once during this monitor session, so record that
        # narrower fact without promoting it to EV fusion evidence.
        ev_fusion_verified = False
        position_mode_acceptance_verified = bool(self.position_mode_accepted_once)
        position_mode_acceptance_unverified = not position_mode_acceptance_verified
        observable_pipeline_ok = bool(
            connected
            and source_fresh
            and vision_fresh
            and flight_rate_ok
            and local_pose_continuous
            and bridge_ok
            and solution_flags_ok
            and timesync_ok
            and not fcu_fault_active
        )
        position_ready = bool(
            observable_pipeline_ok
            and self.direction_test_confirmed
            and ev_fusion_verified
            and not position_mode_acceptance_unverified
        )

        blocking_reasons = []

        def block(reason):
            if reason not in blocking_reasons:
                blocking_reasons.append(reason)

        if not connected:
            block("FCU_DISCONNECTED")
        if not source_fresh:
            block("FASTLIO_NOT_READY")
        if fcu_fault_active:
            block("PX4_STATUS_FAULT")
        if not timesync_ok:
            block("TIMESYNC_UNHEALTHY")

        if bridge is None:
            block("BRIDGE_DIAGNOSTICS_MISSING")
        elif not bridge_diagnostics_fresh:
            block("BRIDGE_DIAGNOSTICS_STALE")
        elif bridge_gate_blocked:
            block("DIAGNOSTIC_GATE_BLOCKED")
        elif bridge_waiting:
            block("BRIDGE_WAITING_{}".format(bridge_state))
        elif bridge_fault:
            block("BRIDGE_FAULT_{}".format(bridge_state))

        # Downstream failures are meaningful once the bridge says it is
        # publishing.  Add all of them instead of choosing one mutually
        # exclusive cause, so a 10 Hz warning cannot hide an EKF/local-pose
        # failure.
        if bridge_ok:
            if not vision_fresh:
                block("VISION_STREAM_NOT_READY")
            if not flight_rate_ok:
                block("POSE_RATE_BELOW_FLIGHT_MIN")
            if not local_pose_continuous:
                block("PX4_LOCAL_POSE_NOT_CONTINUOUS")
            if not solution_flags_ok:
                block("PX4_SOLUTION_FLAGS_NOT_OK")

        if not self.direction_test_confirmed:
            block("DIRECTION_TEST_NOT_CONFIRMED")
        if not ev_fusion_verified:
            block("EV_FUSION_UNVERIFIED")
        if position_mode_acceptance_unverified:
            block("POSITION_MODE_ACCEPTANCE_UNVERIFIED")

        if bridge is None:
            level, summary = DiagnosticStatus.ERROR, "BRIDGE_DIAGNOSTICS_MISSING"
        elif not bridge_diagnostics_fresh:
            level, summary = DiagnosticStatus.ERROR, "BRIDGE_DIAGNOSTICS_STALE"
        elif bridge_fault:
            level, summary = DiagnosticStatus.ERROR, "BRIDGE_FAULT"
        elif not connected:
            level, summary = DiagnosticStatus.ERROR, "FCU_DISCONNECTED"
        elif not source_fresh:
            level, summary = DiagnosticStatus.ERROR, "FASTLIO_NOT_READY"
        elif fcu_fault_active:
            level, summary = DiagnosticStatus.ERROR, "PX4_STATUS_FAULT"
        elif not timesync_ok:
            level, summary = DiagnosticStatus.ERROR, "TIMESYNC_UNHEALTHY"
        elif bridge_gate_blocked:
            level, summary = DiagnosticStatus.WARN, "DIAGNOSTIC_GATE_BLOCKED"
        elif bridge_waiting:
            level, summary = DiagnosticStatus.WARN, "BRIDGE_WAITING"
        elif not vision_fresh:
            level, summary = DiagnosticStatus.ERROR, "VISION_STREAM_NOT_READY"
        elif not solution_flags_ok:
            level, summary = DiagnosticStatus.ERROR, "PX4_SOLUTION_FLAGS_NOT_OK"
        elif not local_pose_continuous:
            level, summary = DiagnosticStatus.ERROR, "PX4_LOCAL_POSE_NOT_CONTINUOUS"
        elif not flight_rate_ok:
            level, summary = DiagnosticStatus.WARN, "BENCH_ONLY_POSE_RATE_TOO_LOW"
        elif not self.direction_test_confirmed:
            level, summary = DiagnosticStatus.WARN, "DIRECTION_TEST_NOT_CONFIRMED"
        else:
            level, summary = DiagnosticStatus.WARN, "EV_FUSION_UNVERIFIED"

        values = {
            "position_data_ready": position_ready,
            "observable_pipeline_ok": observable_pipeline_ok,
            "ev_fusion_verified": ev_fusion_verified,
            "position_mode_acceptance_verified": position_mode_acceptance_verified,
            "position_mode_acceptance_unverified": position_mode_acceptance_unverified,
            "blocking_reason_count": len(blocking_reasons),
            "blocking_reasons": ",".join(blocking_reasons),
            "direction_test_confirmed": self.direction_test_confirmed,
            "fcu_connected": connected,
            "fcu_armed": bool(state and state.armed),
            "fcu_mode": state.mode if state else "unknown",
            "fastlio_fresh": source_fresh,
            "fastlio_rate_hz": "{:.2f}".format(source_rate),
            "fastlio_age_s": self._format_age(source_age),
            "vision_fresh": vision_fresh,
            "vision_rate_hz": "{:.2f}".format(vision_rate),
            "vision_age_s": self._format_age(vision_age),
            "required_flight_pose_rate_hz": "{:.1f}".format(
                self.flight_min_pose_rate_hz
            ),
            "bridge_ok": bridge_ok,
            "bridge_diagnostics_fresh": bridge_diagnostics_fresh,
            "bridge_fault": bridge_fault,
            "bridge_gate_blocked": bridge_gate_blocked,
            "bridge_waiting": bridge_waiting,
            "bridge_state": bridge_state,
            "px4_estimator_status_fresh": estimator_fresh,
            "px4_solution_flags_ok": solution_flags_ok,
            "px4_solution_flags_source_agnostic": True,
            "px4_solution_flag_attitude": self._solution_flag(
                estimator, estimator_fresh, "attitude_status_flag"
            ),
            "px4_solution_flag_velocity_horiz": self._solution_flag(
                estimator, estimator_fresh, "velocity_horiz_status_flag"
            ),
            "px4_solution_flag_velocity_vert": self._solution_flag(
                estimator, estimator_fresh, "velocity_vert_status_flag"
            ),
            "px4_solution_flag_pos_horiz_rel": self._solution_flag(
                estimator, estimator_fresh, "pos_horiz_rel_status_flag"
            ),
            "px4_solution_flag_pos_horiz_abs": self._solution_flag(
                estimator, estimator_fresh, "pos_horiz_abs_status_flag"
            ),
            "px4_solution_flag_pos_vert_abs": self._solution_flag(
                estimator, estimator_fresh, "pos_vert_abs_status_flag"
            ),
            "px4_solution_flag_pos_vert_agl": self._solution_flag(
                estimator, estimator_fresh, "pos_vert_agl_status_flag"
            ),
            "px4_solution_flag_const_pos_mode": self._solution_flag(
                estimator, estimator_fresh, "const_pos_mode_status_flag"
            ),
            "px4_solution_flag_pred_pos_horiz_rel": self._solution_flag(
                estimator, estimator_fresh, "pred_pos_horiz_rel_status_flag"
            ),
            "px4_solution_flag_pred_pos_horiz_abs": self._solution_flag(
                estimator, estimator_fresh, "pred_pos_horiz_abs_status_flag"
            ),
            "px4_solution_flag_gps_glitch": self._solution_flag(
                estimator, estimator_fresh, "gps_glitch_status_flag"
            ),
            "px4_solution_flag_accel_error": self._solution_flag(
                estimator, estimator_fresh, "accel_error_status_flag"
            ),
            "px4_local_pose_fresh": local_pose_fresh,
            "px4_local_pose_continuous": local_pose_continuous,
            "px4_local_pose_rate_hz": "{:.2f}".format(local_pose_rate),
            "px4_local_pose_sample_count": local_pose_samples,
            "px4_local_pose_age_s": self._format_age(local_pose_age),
            "required_local_pose_rate_hz": "{:.1f}".format(
                self.local_pose_min_rate_hz
            ),
            "required_local_pose_samples": self.min_local_pose_samples,
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

    @staticmethod
    def _format_age(age):
        return "missing" if not math.isfinite(age) else "{:.3f}".format(age)

    @staticmethod
    def _solution_flag(estimator, estimator_fresh, attribute):
        if not estimator_fresh:
            return "missing"
        return bool(getattr(estimator, attribute))

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
