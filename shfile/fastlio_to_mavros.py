#!/usr/bin/env python3
"""Safely transform FAST-LIO Airy odometry into a MAVROS vision pose.

The bridge is deliberately fail-closed.  It never changes PX4 parameters,
flight mode, or arming state, and it publishes nothing until the startup script
has confirmed all external gates and an Airy-IMU-to-aircraft transform has been
explicitly marked as calibrated.
"""

import math
import sys
import threading
from collections import deque

import numpy as np
import rospy
import tf.transformations as tft
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, TimesyncStatus
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def _finite(values):
    return all(math.isfinite(float(value)) for value in values)


def _unit(vector):
    vector = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(vector))
    if not math.isfinite(norm) or norm < 1.0e-9:
        raise ValueError("zero or non-finite vector")
    return vector / norm


def _normalise_quaternion(quaternion):
    quaternion = np.asarray(quaternion, dtype=float)
    if quaternion.shape != (4,) or not _finite(quaternion):
        raise ValueError("quaternion contains non-finite values")
    norm = float(np.linalg.norm(quaternion))
    if norm < 1.0e-9 or abs(norm - 1.0) > 0.10:
        raise ValueError("quaternion norm is outside [0.90, 1.10]")
    return quaternion / norm


def _rotation_from_quaternion(quaternion):
    return tft.quaternion_matrix(_normalise_quaternion(quaternion))[:3, :3]


def _quaternion_from_rotation(rotation):
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = rotation
    quaternion = tft.quaternion_from_matrix(matrix)
    return _normalise_quaternion(quaternion)


def _rotation_angle_deg(left, right):
    relative = np.asarray(left).T.dot(np.asarray(right))
    cosine = max(-1.0, min(1.0, (float(np.trace(relative)) - 1.0) * 0.5))
    return math.degrees(math.acos(cosine))


def _vector_angle_deg(left, right):
    left_u = _unit(left)
    right_u = _unit(right)
    cosine = max(-1.0, min(1.0, float(left_u.dot(right_u))))
    return math.degrees(math.acos(cosine))


def _as_vector_param(name, default, length):
    value = rospy.get_param(name, default)
    if isinstance(value, str):
        value = [item.strip() for item in value.strip("[] ").split(",") if item.strip()]
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError("{} must contain {} numbers".format(name, length))
    result = np.asarray([float(item) for item in value], dtype=float)
    if not _finite(result):
        raise ValueError("{} contains non-finite values".format(name))
    return result


def compute_alignment(rotation_world_sensor, rotation_enu_body,
                      rotation_body_sensor, position_world_sensor,
                      sensor_position_in_body):
    """Return R_EW and t_EW for an initially zero aircraft-body position.

    R_BI rotates vectors from the Airy IMU frame I into aircraft base_link FLU
    frame B.  sensor_position_in_body is the position of the Airy IMU origin
    relative to the desired aircraft origin, expressed in B.
    """
    rotation_sensor_body = rotation_body_sensor.T
    rotation_enu_world = (
        rotation_enu_body.dot(rotation_body_sensor).dot(rotation_world_sensor.T)
    )
    translation_enu_world = (
        rotation_enu_body.dot(sensor_position_in_body)
        - rotation_enu_world.dot(position_world_sensor)
    )
    # Keep the unused expression here explicit: it documents the output chain.
    assert rotation_sensor_body.shape == (3, 3)
    return rotation_enu_world, translation_enu_world


def transform_body_pose(rotation_enu_world, translation_enu_world,
                        rotation_world_sensor, position_world_sensor,
                        rotation_body_sensor, sensor_position_in_body):
    rotation_enu_body = (
        rotation_enu_world.dot(rotation_world_sensor).dot(rotation_body_sensor.T)
    )
    position_enu_sensor = (
        rotation_enu_world.dot(position_world_sensor) + translation_enu_world
    )
    position_enu_body = (
        position_enu_sensor - rotation_enu_body.dot(sensor_position_in_body)
    )
    return rotation_enu_body, position_enu_body


class FastlioToMavrosBridge:
    def __init__(self):
        self.lock = threading.RLock()

        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.sensor_imu_topic = rospy.get_param("~sensor_imu_topic", "/rslidar_imu_data")
        self.fcu_imu_topic = rospy.get_param("~fcu_imu_topic", "/liu/mavros/imu/data")
        self.state_topic = rospy.get_param("~state_topic", "/liu/mavros/state")
        self.timesync_topic = rospy.get_param(
            "~timesync_topic", "/liu/mavros/timesync_status"
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/liu/mavros/vision_pose/pose"
        )
        self.diagnostics_topic = rospy.get_param(
            "~diagnostics_topic", "/airy_px4/bridge/diagnostics"
        )

        self.publish_enabled = bool(rospy.get_param("~publish_enabled", False))
        self.mount_confirmed = bool(rospy.get_param("~mount_confirmed", False))
        self.expected_frame = rospy.get_param("~expected_frame", "camera_init")
        self.expected_child_frame = rospy.get_param("~expected_child_frame", "body")
        self.output_frame = rospy.get_param("~output_frame", "odom")
        self.stable_seconds = float(rospy.get_param("~stable_seconds", 5.0))
        self.min_source_rate_hz = float(rospy.get_param("~min_source_rate_hz", 8.0))
        self.max_source_age_s = float(rospy.get_param("~max_source_age_s", 0.5))
        self.max_imu_age_s = float(rospy.get_param("~max_imu_age_s", 0.25))
        self.max_state_age_s = float(rospy.get_param("~max_state_age_s", 2.0))
        self.max_timesync_age_s = float(rospy.get_param("~max_timesync_age_s", 2.0))
        self.max_timesync_rtt_ms = float(
            rospy.get_param("~max_timesync_rtt_ms", 20.0)
        )
        self.max_timesync_offset_std_ms = float(
            rospy.get_param("~max_timesync_offset_std_ms", 5.0)
        )
        self.max_source_gap_s = float(rospy.get_param("~max_source_gap_s", 0.30))
        self.max_receipt_stall_s = float(
            rospy.get_param("~max_receipt_stall_s", self.max_source_age_s)
        )
        self.max_position_jump_m = float(rospy.get_param("~max_position_jump_m", 1.0))
        self.max_orientation_jump_deg = float(
            rospy.get_param("~max_orientation_jump_deg", 45.0)
        )
        self.max_stationary_angular_rate = float(
            rospy.get_param("~max_stationary_angular_rate_rad_s", 0.15)
        )
        self.max_gravity_mismatch_deg = float(
            rospy.get_param("~max_gravity_mismatch_deg", 10.0)
        )
        self.max_alignment_change_deg = float(
            rospy.get_param("~max_alignment_change_deg", 3.0)
        )

        quaternion_body_sensor = _as_vector_param(
            "~sensor_to_body_quaternion_xyzw", [0.0, 0.0, 0.0, 1.0], 4
        )
        self.rotation_body_sensor = _rotation_from_quaternion(quaternion_body_sensor)
        self.sensor_position_in_body = _as_vector_param(
            "~sensor_position_in_body_xyz_m", [0.0, 0.0, 0.0], 3
        )

        self.state = None
        self.state_receive_time = None
        self.timesync = None
        self.timesync_receive_time = None
        self.timesync_offsets_ns = deque(maxlen=50)
        self.sensor_imu = None
        self.sensor_imu_receive_time = None
        self.fcu_imu = None
        self.fcu_imu_receive_time = None
        self.latest_odom = None
        self.latest_odom_receive_time = None
        self.last_source_stamp = None
        self.last_source_position = None
        self.last_source_rotation = None
        self.source_receipts = deque(maxlen=200)
        self.last_valid_source_receipt = None
        self.last_source_receipt_gap_s = 0.0
        self.max_observed_source_receipt_gap_s = 0.0
        self.last_source_gap_s = 0.0
        self.max_observed_source_gap_s = 0.0
        self.output_receipts = deque(maxlen=200)
        self.stable_since = None
        self.alignment_candidate = None
        self.rotation_enu_world = None
        self.translation_enu_world = None
        self.aligned = False
        self.latched_fault = ""
        self.last_rejection = "waiting for input"
        self.gravity_mismatch_deg = float("nan")

        self.pose_publisher = rospy.Publisher(
            self.output_topic, PoseStamped, queue_size=10
        )
        self.diagnostic_publisher = rospy.Publisher(
            self.diagnostics_topic, DiagnosticArray, queue_size=10, latch=True
        )
        rospy.Subscriber(self.state_topic, State, self._state_callback, queue_size=10)
        rospy.Subscriber(
            self.timesync_topic,
            TimesyncStatus,
            self._timesync_callback,
            queue_size=20,
        )
        rospy.Subscriber(self.sensor_imu_topic, Imu, self._sensor_imu_callback, queue_size=50)
        rospy.Subscriber(self.fcu_imu_topic, Imu, self._fcu_imu_callback, queue_size=50)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(0.5), self._timer_callback)

        rospy.logwarn(
            "FAST-LIO->MAVROS bridge started fail-closed: publish_enabled=%s, "
            "mount_confirmed=%s",
            self.publish_enabled,
            self.mount_confirmed,
        )

    @staticmethod
    def _message_quaternion(message):
        return np.array(
            [message.x, message.y, message.z, message.w], dtype=float
        )

    @staticmethod
    def _message_vector(message):
        return np.array([message.x, message.y, message.z], dtype=float)

    @staticmethod
    def _rate(receipts):
        if len(receipts) < 2:
            return 0.0
        span = receipts[-1] - receipts[0]
        return 0.0 if span <= 0.0 else float(len(receipts) - 1) / span

    def _trim_receipts(self, receipts, now_sec, window=5.0):
        while receipts and now_sec - receipts[0] > window:
            receipts.popleft()

    def _state_callback(self, message):
        with self.lock:
            self.state = message
            self.state_receive_time = rospy.Time.now()

    def _timesync_callback(self, message):
        with self.lock:
            self.timesync = message
            self.timesync_receive_time = rospy.Time.now()
            self.timesync_offsets_ns.append(float(message.estimated_offset_ns))

    def _sensor_imu_callback(self, message):
        with self.lock:
            self.sensor_imu = message
            self.sensor_imu_receive_time = rospy.Time.now()

    def _fcu_imu_callback(self, message):
        with self.lock:
            self.fcu_imu = message
            self.fcu_imu_receive_time = rospy.Time.now()

    def _reject(self, reason, latch=False):
        self.last_rejection = reason
        self.stable_since = None
        self.alignment_candidate = None
        if latch and not self.latched_fault:
            self.latched_fault = reason
            rospy.logerr("Bridge latched fault; restart required: %s", reason)

    def _validate_odom(self, message, now):
        if message.header.frame_id != self.expected_frame:
            raise ValueError(
                "unexpected frame_id {!r}".format(message.header.frame_id)
            )
        if message.child_frame_id != self.expected_child_frame:
            raise ValueError(
                "unexpected child_frame_id {!r}".format(message.child_frame_id)
            )
        stamp = message.header.stamp.to_sec()
        if not math.isfinite(stamp) or stamp <= 0.0:
            raise ValueError("invalid source timestamp")
        age = now.to_sec() - stamp
        if age > self.max_source_age_s or age < -0.20:
            raise ValueError("source timestamp age {:.3f}s is invalid".format(age))

        position = self._message_vector(message.pose.pose.position)
        quaternion = self._message_quaternion(message.pose.pose.orientation)
        if not _finite(position):
            raise ValueError("source position contains non-finite values")
        rotation = _rotation_from_quaternion(quaternion)

        if self.last_source_stamp is not None and stamp <= self.last_source_stamp:
            raise RuntimeError("source timestamp moved backwards or repeated")
        if self.last_source_position is not None:
            position_jump = float(np.linalg.norm(position - self.last_source_position))
            if position_jump > self.max_position_jump_m:
                raise RuntimeError(
                    "source position jumped {:.3f}m".format(position_jump)
                )
            angle_jump = _rotation_angle_deg(self.last_source_rotation, rotation)
            if angle_jump > self.max_orientation_jump_deg:
                raise RuntimeError(
                    "source attitude jumped {:.1f}deg".format(angle_jump)
                )
        return stamp, position, rotation

    def _initialisation_inputs(self, now):
        if not self.publish_enabled:
            return None, "publishing disabled by startup safety gate"
        if not self.mount_confirmed:
            return None, "Airy-IMU-to-base_link transform is not confirmed"
        if self.state is None or self.state_receive_time is None:
            return None, "waiting for MAVROS state"
        if (now - self.state_receive_time).to_sec() > self.max_state_age_s:
            return None, "MAVROS state is stale"
        if not self.state.connected:
            return None, "FCU is not connected"
        if self.state.armed:
            return None, "refusing to initialise while FCU is armed"
        timesync_ok, timesync_reason = self._timesync_healthy(now)
        if not timesync_ok:
            return None, timesync_reason
        if self.sensor_imu is None or self.fcu_imu is None:
            return None, "waiting for both Airy and FCU IMUs"
        if (now - self.sensor_imu_receive_time).to_sec() > self.max_imu_age_s:
            return None, "Airy IMU is stale"
        if (now - self.fcu_imu_receive_time).to_sec() > self.max_imu_age_s:
            return None, "FCU IMU is stale"
        source_rate = self._rate(self.source_receipts)
        if source_rate < self.min_source_rate_hz:
            return None, "FAST-LIO rate {:.1f}Hz is below {:.1f}Hz".format(
                source_rate, self.min_source_rate_hz
            )

        sensor_angular = self._message_vector(self.sensor_imu.angular_velocity)
        fcu_angular = self._message_vector(self.fcu_imu.angular_velocity)
        if not _finite(sensor_angular) or not _finite(fcu_angular):
            return None, "IMU angular velocity is non-finite"
        if max(np.linalg.norm(sensor_angular), np.linalg.norm(fcu_angular)) > \
                self.max_stationary_angular_rate:
            return None, "aircraft must remain stationary during alignment"

        sensor_acceleration = self._message_vector(self.sensor_imu.linear_acceleration)
        fcu_acceleration = self._message_vector(self.fcu_imu.linear_acceleration)
        try:
            mapped_sensor_acceleration = self.rotation_body_sensor.dot(sensor_acceleration)
            mismatch = _vector_angle_deg(mapped_sensor_acceleration, fcu_acceleration)
        except ValueError:
            return None, "invalid acceleration vector during gravity check"
        self.gravity_mismatch_deg = mismatch
        if mismatch > self.max_gravity_mismatch_deg:
            return None, (
                "Airy/FCU gravity directions differ by {:.1f}deg; "
                "mount transform or IMU convention is inconsistent"
            ).format(mismatch)

        try:
            rotation_enu_body = _rotation_from_quaternion(
                self._message_quaternion(self.fcu_imu.orientation)
            )
        except ValueError as error:
            return None, "invalid FCU orientation: {}".format(error)
        return rotation_enu_body, ""

    def _timesync_healthy(self, now):
        if self.timesync is None or self.timesync_receive_time is None:
            return False, "waiting for MAVROS timesync"
        if (now - self.timesync_receive_time).to_sec() > self.max_timesync_age_s:
            return False, "MAVROS timesync is stale"
        rtt_ms = float(self.timesync.round_trip_time_ms)
        if not math.isfinite(rtt_ms) or rtt_ms > self.max_timesync_rtt_ms:
            return False, "MAVROS timesync RTT {:.2f}ms is unhealthy".format(rtt_ms)
        if len(self.timesync_offsets_ns) < 10:
            return False, "collecting MAVROS timesync samples"
        offset_std_ms = float(np.std(self.timesync_offsets_ns)) / 1.0e6
        if (not math.isfinite(offset_std_ms)
                or offset_std_ms > self.max_timesync_offset_std_ms):
            return False, "MAVROS timesync offset std {:.2f}ms is unhealthy".format(
                offset_std_ms
            )
        return True, ""

    def _update_passive_gravity_check(self, now):
        """Evaluate the configured mount even while publishing is blocked."""
        if (self.sensor_imu is None or self.fcu_imu is None
                or self.sensor_imu_receive_time is None
                or self.fcu_imu_receive_time is None):
            return
        if ((now - self.sensor_imu_receive_time).to_sec() > self.max_imu_age_s
                or (now - self.fcu_imu_receive_time).to_sec() > self.max_imu_age_s):
            return
        sensor_acceleration = self._message_vector(self.sensor_imu.linear_acceleration)
        fcu_acceleration = self._message_vector(self.fcu_imu.linear_acceleration)
        try:
            self.gravity_mismatch_deg = _vector_angle_deg(
                self.rotation_body_sensor.dot(sensor_acceleration),
                fcu_acceleration,
            )
        except ValueError:
            self.gravity_mismatch_deg = float("nan")

    def _try_initialise(self, now, position_world_sensor, rotation_world_sensor):
        rotation_enu_body, reason = self._initialisation_inputs(now)
        if rotation_enu_body is None:
            self._reject(reason)
            return

        rotation_candidate, translation_candidate = compute_alignment(
            rotation_world_sensor,
            rotation_enu_body,
            self.rotation_body_sensor,
            position_world_sensor,
            self.sensor_position_in_body,
        )
        if self.alignment_candidate is None:
            self.alignment_candidate = rotation_candidate
            self.stable_since = now
            self.last_rejection = "collecting stationary alignment samples"
            return
        change = _rotation_angle_deg(self.alignment_candidate, rotation_candidate)
        if change > self.max_alignment_change_deg:
            self._reject(
                "alignment changed {:.1f}deg while collecting samples".format(change)
            )
            return
        if (now - self.stable_since).to_sec() < self.stable_seconds:
            self.last_rejection = "collecting stationary alignment samples"
            return

        self.rotation_enu_world = rotation_candidate
        self.translation_enu_world = translation_candidate
        self.aligned = True
        self.last_rejection = ""
        rospy.logwarn(
            "Bridge alignment locked while disarmed; publishing native FAST-LIO "
            "poses without rate upsampling"
        )

    def _publish_pose(self, source_message, position_world_sensor,
                      rotation_world_sensor, now_sec):
        if self.latched_fault or not self.aligned:
            return
        if self.state is None or not self.state.connected:
            self._reject("FCU disconnected after alignment", latch=True)
            return
        timesync_ok, timesync_reason = self._timesync_healthy(rospy.Time.from_sec(now_sec))
        if not timesync_ok:
            self._reject(timesync_reason, latch=True)
            return
        if self._rate(self.source_receipts) < self.min_source_rate_hz:
            self._reject("FAST-LIO rate fell below the configured minimum", latch=True)
            return

        rotation_enu_body, position_enu_body = transform_body_pose(
            self.rotation_enu_world,
            self.translation_enu_world,
            rotation_world_sensor,
            position_world_sensor,
            self.rotation_body_sensor,
            self.sensor_position_in_body,
        )
        quaternion = _quaternion_from_rotation(rotation_enu_body)
        if not _finite(position_enu_body) or float(np.linalg.norm(position_enu_body)) > 1000.0:
            self._reject("transformed position is invalid", latch=True)
            return

        output = PoseStamped()
        output.header.stamp = source_message.header.stamp
        output.header.frame_id = self.output_frame
        output.pose.position.x = float(position_enu_body[0])
        output.pose.position.y = float(position_enu_body[1])
        output.pose.position.z = float(position_enu_body[2])
        output.pose.orientation.x = float(quaternion[0])
        output.pose.orientation.y = float(quaternion[1])
        output.pose.orientation.z = float(quaternion[2])
        output.pose.orientation.w = float(quaternion[3])
        self.pose_publisher.publish(output)
        self.output_receipts.append(now_sec)
        self._trim_receipts(self.output_receipts, now_sec)

    def _odom_callback(self, message):
        now = rospy.Time.now()
        now_sec = now.to_sec()
        with self.lock:
            # A latched fault is restart-only by design.  Freeze the triggering
            # gap and rejection details instead of letting every later source
            # callback inflate them into a misleading multi-second value.
            if self.latched_fault:
                return
            try:
                stamp, position, rotation = self._validate_odom(message, now)
            except RuntimeError as error:
                self._reject(str(error), latch=self.aligned)
                return
            except ValueError as error:
                self._reject(str(error), latch=False)
                return

            # A callback can be delayed briefly by Linux scheduling while ROS
            # has already queued every odometry message.  Treat header-stamp
            # spacing as the actual source-frame gap; retain receipt spacing as
            # a separate process/scheduling-stall gate with a looser threshold.
            # _health() also detects an input that remains stale without any
            # callback returning.
            receipt_gap = (
                0.0 if self.last_valid_source_receipt is None
                else now_sec - self.last_valid_source_receipt
            )
            source_gap = (
                0.0 if self.last_source_stamp is None
                else stamp - self.last_source_stamp
            )
            self.last_source_receipt_gap_s = receipt_gap
            self.max_observed_source_receipt_gap_s = max(
                self.max_observed_source_receipt_gap_s, receipt_gap
            )
            self.last_source_gap_s = source_gap
            self.max_observed_source_gap_s = max(
                self.max_observed_source_gap_s, source_gap
            )
            if self.aligned and source_gap > self.max_source_gap_s:
                self._reject(
                    "FAST-LIO source timestamp gap {:.3f}s exceeded {:.3f}s".format(
                        source_gap, self.max_source_gap_s
                    ),
                    latch=True,
                )
                return
            if self.aligned and receipt_gap > self.max_receipt_stall_s:
                self._reject(
                    "bridge callback receipt stall {:.3f}s exceeded {:.3f}s".format(
                        receipt_gap, self.max_receipt_stall_s
                    ),
                    latch=True,
                )
                return
            self.last_valid_source_receipt = now_sec
            self.source_receipts.append(now_sec)
            self._trim_receipts(self.source_receipts, now_sec)

            self.latest_odom = message
            self.latest_odom_receive_time = now
            self.last_source_stamp = stamp
            self.last_source_position = position
            self.last_source_rotation = rotation

            if not self.aligned and not self.latched_fault:
                self._try_initialise(now, position, rotation)
            self._publish_pose(message, position, rotation, now_sec)

    def _health(self, now):
        if self.latched_fault:
            return DiagnosticStatus.ERROR, "FAULT_LATCHED"
        if not self.publish_enabled:
            return DiagnosticStatus.WARN, "BLOCKED_BY_STARTUP_GATE"
        if not self.mount_confirmed:
            return DiagnosticStatus.ERROR, "MOUNT_UNCONFIRMED"
        state_stale = bool(
            self.state_receive_time is None
            or (now - self.state_receive_time).to_sec() > self.max_state_age_s
        )
        if self.aligned and (
            self.state is None or state_stale or not self.state.connected
        ):
            self._reject("MAVROS state/FCU link was lost after alignment", latch=True)
            return DiagnosticStatus.ERROR, "FAULT_LATCHED"
        if self.state is None or state_stale or not self.state.connected:
            return DiagnosticStatus.ERROR, "FCU_DISCONNECTED"
        if self.latest_odom_receive_time is None:
            return DiagnosticStatus.WARN, "WAITING_FASTLIO"
        if (now - self.latest_odom_receive_time).to_sec() > self.max_source_age_s:
            if self.aligned:
                self._reject("FAST-LIO source became stale", latch=True)
                return DiagnosticStatus.ERROR, "FAULT_LATCHED"
            return DiagnosticStatus.WARN, "FASTLIO_STALE"
        if self.aligned and self._rate(self.source_receipts) < self.min_source_rate_hz:
            self._reject("FAST-LIO rate fell below the configured minimum", latch=True)
            return DiagnosticStatus.ERROR, "FAULT_LATCHED"
        if self.aligned:
            timesync_ok, timesync_reason = self._timesync_healthy(now)
            if not timesync_ok:
                self._reject(timesync_reason, latch=True)
                return DiagnosticStatus.ERROR, "FAULT_LATCHED"
        if not self.aligned:
            if self.state.armed:
                return DiagnosticStatus.ERROR, "ARMED_BEFORE_ALIGNMENT"
            return DiagnosticStatus.WARN, "WAITING_STATIONARY_ALIGNMENT"
        if self.pose_publisher.get_num_connections() < 1:
            return DiagnosticStatus.ERROR, "MAVROS_NOT_SUBSCRIBED"
        return DiagnosticStatus.OK, "PUBLISHING_NATIVE_RATE"

    def _timer_callback(self, _event):
        now = rospy.Time.now()
        with self.lock:
            self._update_passive_gravity_check(now)
            level, message = self._health(now)
            now_sec = now.to_sec()
            self._trim_receipts(self.source_receipts, now_sec)
            self._trim_receipts(self.output_receipts, now_sec)
            source_age = (
                float("inf") if self.latest_odom_receive_time is None
                else (now - self.latest_odom_receive_time).to_sec()
            )
            state = self.state
            timesync_rtt_ms = (
                float("nan") if self.timesync is None
                else float(self.timesync.round_trip_time_ms)
            )
            timesync_offset_std_ms = (
                float("nan") if len(self.timesync_offsets_ns) < 2
                else float(np.std(self.timesync_offsets_ns)) / 1.0e6
            )
            values = {
                "publish_enabled": self.publish_enabled,
                "mount_confirmed": self.mount_confirmed,
                "aligned": self.aligned,
                "latched_fault": self.latched_fault or "none",
                "last_rejection": self.last_rejection or "none",
                "fcu_connected": bool(state and state.connected),
                "fcu_armed": bool(state and state.armed),
                "input_rate_hz": "{:.2f}".format(self._rate(self.source_receipts)),
                "output_rate_hz": "{:.2f}".format(self._rate(self.output_receipts)),
                "input_age_s": "{:.3f}".format(source_age),
                "last_source_gap_s": "{:.3f}".format(self.last_source_gap_s),
                "max_observed_source_gap_s": "{:.3f}".format(
                    self.max_observed_source_gap_s
                ),
                "last_source_receipt_gap_s": "{:.3f}".format(
                    self.last_source_receipt_gap_s
                ),
                "max_observed_source_receipt_gap_s": "{:.3f}".format(
                    self.max_observed_source_receipt_gap_s
                ),
                "max_receipt_stall_s": "{:.3f}".format(
                    self.max_receipt_stall_s
                ),
                "max_source_gap_s": "{:.3f}".format(self.max_source_gap_s),
                "state_age_s": (
                    "unknown" if self.state_receive_time is None
                    else "{:.3f}".format((now - self.state_receive_time).to_sec())
                ),
                "timesync_rtt_ms": (
                    "unknown" if not math.isfinite(timesync_rtt_ms)
                    else "{:.3f}".format(timesync_rtt_ms)
                ),
                "timesync_offset_std_ms": (
                    "unknown" if not math.isfinite(timesync_offset_std_ms)
                    else "{:.3f}".format(timesync_offset_std_ms)
                ),
                "gravity_mismatch_deg": (
                    "unknown" if not math.isfinite(self.gravity_mismatch_deg)
                    else "{:.2f}".format(self.gravity_mismatch_deg)
                ),
                "output_subscribers": self.pose_publisher.get_num_connections(),
                "source_frame": self.expected_frame,
                "output_frame": self.output_frame,
            }
            status = DiagnosticStatus()
            status.level = level
            status.name = "airy_px4/fastlio_to_mavros_bridge"
            status.message = message
            status.hardware_id = "robosense_airy_px4"
            status.values = [
                KeyValue(key=str(key), value=str(value))
                for key, value in sorted(values.items())
            ]
            array = DiagnosticArray()
            array.header.stamp = now
            array.status = [status]
            self.diagnostic_publisher.publish(array)


def _self_test():
    identity = np.eye(3)
    zero = np.zeros(3)
    rotation, translation = compute_alignment(
        identity, identity, identity, zero, zero
    )
    output_rotation, output_position = transform_body_pose(
        rotation, translation, identity, zero, identity, zero
    )
    assert np.allclose(output_rotation, identity, atol=1.0e-9)
    assert np.allclose(output_position, zero, atol=1.0e-9)

    body_sensor = tft.euler_matrix(math.pi, 0.0, 0.0)[:3, :3]
    lever_arm = np.array([0.20, -0.05, 0.10])
    rotation, translation = compute_alignment(
        identity, identity, body_sensor, zero, lever_arm
    )
    output_rotation, output_position = transform_body_pose(
        rotation, translation, identity, zero, body_sensor, lever_arm
    )
    assert np.allclose(output_rotation, identity, atol=1.0e-9)
    assert np.allclose(output_position, zero, atol=1.0e-9)

    moved_sensor = np.array([1.0, 0.0, 0.0])
    _, moved_body = transform_body_pose(
        rotation, translation, identity, moved_sensor, body_sensor, lever_arm
    )
    assert np.allclose(moved_body, rotation.dot(moved_sensor), atol=1.0e-9)
    print("fastlio_to_mavros math self-test: PASS")


def main():
    if "--self-test" in sys.argv:
        _self_test()
        return
    rospy.init_node("fastlio_to_mavros_bridge", anonymous=False)
    try:
        FastlioToMavrosBridge()
    except (TypeError, ValueError) as error:
        rospy.logfatal("Invalid bridge configuration: %s", error)
        raise SystemExit(2)
    rospy.spin()


if __name__ == "__main__":
    main()
