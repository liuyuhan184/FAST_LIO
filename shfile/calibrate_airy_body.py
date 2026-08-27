#!/usr/bin/env python3
"""Read-only Airy IMU to aircraft base_link/FLU rotation calibration helper.

The program subscribes to two IMU streams and MAVROS state.  It never creates
an application publisher, calls a service, changes a parameter/mode, or arms
the vehicle.  A result is printed only after the state and data-quality gates
all pass.
"""

import argparse
import bisect
import math
import sys
import threading
import time
from collections import namedtuple

import numpy as np


Sample = namedtuple("Sample", "stamp gyro acceleration")


class CalibrationFailure(RuntimeError):
    """A fail-closed calibration result with safe, non-transform diagnostics."""

    def __init__(self, message, diagnostics=None):
        super().__init__(message)
        self.diagnostics = dict(diagnostics or {})


def _finite_vector(values):
    vector = np.asarray(values, dtype=float)
    return vector.shape == (3,) and bool(np.all(np.isfinite(vector)))


def _unit_rows(vectors):
    vectors = np.asarray(vectors, dtype=float)
    if vectors.ndim != 2 or vectors.shape[1] != 3:
        raise ValueError("cannot normalise zero or malformed vectors")
    norms = np.linalg.norm(vectors, axis=1)
    if np.any(norms < 1.0e-9):
        raise ValueError("cannot normalise zero or malformed vectors")
    return vectors / norms[:, None]


def _rotation_angle_deg(left, right):
    relative = np.asarray(left, dtype=float).T.dot(np.asarray(right, dtype=float))
    cosine = np.clip((float(np.trace(relative)) - 1.0) * 0.5, -1.0, 1.0)
    return math.degrees(math.acos(cosine))


def _vector_angles_deg(left, right):
    left_unit = _unit_rows(left)
    right_unit = _unit_rows(right)
    cosine = np.clip(np.sum(left_unit * right_unit, axis=1), -1.0, 1.0)
    return np.degrees(np.arccos(cosine))


def _wahba(source_vectors, target_vectors):
    """Return the proper R that minimises ||target - R * source||."""
    source = np.asarray(source_vectors, dtype=float)
    target = np.asarray(target_vectors, dtype=float)
    cross = target.T.dot(source)
    left, _singular, right_t = np.linalg.svd(cross)
    correction = np.eye(3)
    correction[2, 2] = 1.0 if np.linalg.det(left.dot(right_t)) >= 0.0 else -1.0
    rotation = left.dot(correction).dot(right_t)
    if np.linalg.det(rotation) < 0.999999:
        raise CalibrationFailure("Wahba 求解未得到合法右手旋转")
    return rotation


def _quaternion_xyzw(rotation):
    """Convert a proper 3x3 rotation matrix to a normalised xyzw quaternion."""
    matrix = np.asarray(rotation, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array([
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        diagonal = int(np.argmax(np.diag(matrix)))
        if diagonal == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            quaternion = np.array([
                0.25 * scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
            ])
        elif diagonal == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            quaternion = np.array([
                (matrix[0, 1] + matrix[1, 0]) / scale,
                0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
            ])
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            quaternion = np.array([
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                0.25 * scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            ])
    quaternion /= np.linalg.norm(quaternion)
    if quaternion[3] < 0.0:
        quaternion *= -1.0
    return quaternion


def _rotation_from_axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    skew = np.array([
        [0.0, -axis[2], axis[1]],
        [axis[2], 0.0, -axis[0]],
        [-axis[1], axis[0], 0.0],
    ])
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * skew.dot(skew)


def pair_samples(sensor_samples, fcu_samples, max_delta_s):
    """Uniquely pair the lower-rate stream with nearest timestamps."""
    sensor = sorted(sensor_samples, key=lambda sample: sample.stamp)
    fcu = sorted(fcu_samples, key=lambda sample: sample.stamp)
    if not sensor or not fcu:
        return []

    if len(sensor) >= len(fcu):
        reference, candidates, reference_is_fcu = fcu, sensor, True
    else:
        reference, candidates, reference_is_fcu = sensor, fcu, False
    stamps = [sample.stamp for sample in candidates]
    last_index = -1
    pairs = []
    for reference_sample in reference:
        insertion = bisect.bisect_left(stamps, reference_sample.stamp, last_index + 1)
        possible = []
        for index in (insertion - 1, insertion):
            if last_index < index < len(candidates):
                possible.append(index)
        if not possible:
            continue
        index = min(possible, key=lambda item: abs(stamps[item] - reference_sample.stamp))
        delta = stamps[index] - reference_sample.stamp
        if abs(delta) > max_delta_s:
            continue
        last_index = index
        candidate = candidates[index]
        if reference_is_fcu:
            pairs.append((candidate, reference_sample, -delta))
        else:
            pairs.append((reference_sample, candidate, delta))
    return pairs


def _format_value(value):
    if isinstance(value, np.ndarray):
        return "[{}]".format(", ".join("{:.5g}".format(float(item)) for item in value))
    if isinstance(value, (float, np.floating)):
        return "{:.5g}".format(float(value))
    return str(value)


def _fail(message, diagnostics):
    raise CalibrationFailure(message, diagnostics)


def analyse_calibration(stationary_pairs, dynamic_pairs, limits):
    """Solve R_BI and enforce timing, excitation, residual, and gravity gates."""
    diagnostics = {
        "stationary_pairs": len(stationary_pairs),
        "dynamic_pairs": len(dynamic_pairs),
    }
    if len(stationary_pairs) < limits.min_stationary_pairs:
        _fail("静止近时配对样本不足", diagnostics)
    if len(dynamic_pairs) < limits.min_pairs:
        _fail("运动近时配对样本不足", diagnostics)

    stationary_dt = np.abs([pair[2] for pair in stationary_pairs]) * 1000.0
    dynamic_dt = np.abs([pair[2] for pair in dynamic_pairs]) * 1000.0
    diagnostics.update({
        "pair_dt_rms_ms": math.sqrt(float(np.mean(dynamic_dt ** 2))),
        "pair_dt_p95_ms": float(np.percentile(dynamic_dt, 95.0)),
        "pair_dt_max_ms": float(np.max(dynamic_dt)),
    })

    stationary_sensor_gyro = np.asarray([pair[0].gyro for pair in stationary_pairs])
    stationary_fcu_gyro = np.asarray([pair[1].gyro for pair in stationary_pairs])
    bias_sensor = np.median(stationary_sensor_gyro, axis=0)
    bias_fcu = np.median(stationary_fcu_gyro, axis=0)
    stationary_sensor_noise = np.linalg.norm(stationary_sensor_gyro - bias_sensor, axis=1)
    stationary_fcu_noise = np.linalg.norm(stationary_fcu_gyro - bias_fcu, axis=1)
    diagnostics.update({
        "stationary_sensor_gyro_p95_rad_s": float(np.percentile(stationary_sensor_noise, 95.0)),
        "stationary_fcu_gyro_p95_rad_s": float(np.percentile(stationary_fcu_noise, 95.0)),
    })
    if max(diagnostics["stationary_sensor_gyro_p95_rad_s"],
           diagnostics["stationary_fcu_gyro_p95_rad_s"]) > limits.max_stationary_rate:
        _fail("静止去偏置阶段发生了移动或振动过大", diagnostics)

    sensor_gyro = np.asarray([pair[0].gyro for pair in dynamic_pairs]) - bias_sensor
    fcu_gyro = np.asarray([pair[1].gyro for pair in dynamic_pairs]) - bias_fcu
    sensor_norm = np.linalg.norm(sensor_gyro, axis=1)
    fcu_norm = np.linalg.norm(fcu_gyro, axis=1)
    ratio = fcu_norm / np.maximum(sensor_norm, 1.0e-9)
    dynamic_mask = (
        (sensor_norm >= limits.min_dynamic_rate)
        & (fcu_norm >= limits.min_dynamic_rate)
        & (sensor_norm <= limits.max_dynamic_rate)
        & (fcu_norm <= limits.max_dynamic_rate)
        & (ratio >= limits.min_rate_ratio)
        & (ratio <= limits.max_rate_ratio)
    )
    sensor_gyro = sensor_gyro[dynamic_mask]
    fcu_gyro = fcu_gyro[dynamic_mask]
    diagnostics["dynamic_samples_after_rate_filter"] = int(len(sensor_gyro))
    if len(sensor_gyro) < limits.min_pairs:
        _fail("去偏置和动态速率筛选后的样本不足；请三轴缓慢、连续转动", diagnostics)

    # Unit-vector Wahba prevents the fastest axis from dominating the answer.
    inliers = np.ones(len(sensor_gyro), dtype=bool)
    for _iteration in range(5):
        rotation = _wahba(
            _unit_rows(sensor_gyro[inliers]),
            _unit_rows(fcu_gyro[inliers]),
        )
        predicted = sensor_gyro.dot(rotation.T)
        angles = _vector_angles_deg(predicted, fcu_gyro)
        vector_errors = np.linalg.norm(predicted - fcu_gyro, axis=1)
        active_angles = angles[inliers]
        active_errors = vector_errors[inliers]
        angle_median = float(np.median(active_angles))
        angle_mad = float(np.median(np.abs(active_angles - angle_median)))
        error_median = float(np.median(active_errors))
        error_mad = float(np.median(np.abs(active_errors - error_median)))
        angle_cut = min(
            limits.outlier_angle_deg,
            max(2.0, angle_median + 4.0 * 1.4826 * max(angle_mad, 0.05)),
        )
        error_cut = min(
            limits.outlier_rate_error,
            max(0.03, error_median + 4.0 * 1.4826 * max(error_mad, 0.002)),
        )
        updated = (angles <= angle_cut) & (vector_errors <= error_cut)
        if np.array_equal(updated, inliers):
            break
        if int(np.count_nonzero(updated)) < limits.min_pairs:
            _fail("离群点筛选后样本不足；时间同步或两路 IMU 数据可能不一致", diagnostics)
        inliers = updated

    source_inliers = sensor_gyro[inliers]
    target_inliers = fcu_gyro[inliers]
    rotation = _wahba(_unit_rows(source_inliers), _unit_rows(target_inliers))
    predicted = source_inliers.dot(rotation.T)
    angles = _vector_angles_deg(predicted, target_inliers)
    vector_errors = np.linalg.norm(predicted - target_inliers, axis=1)
    diagnostics.update({
        "retained_samples": int(len(source_inliers)),
        "retained_fraction": float(len(source_inliers)) / float(len(sensor_gyro)),
        "angular_rms_deg": math.sqrt(float(np.mean(angles ** 2))),
        "angular_p95_deg": float(np.percentile(angles, 95.0)),
        "angular_max_deg": float(np.max(angles)),
        "rate_error_rms_rad_s": math.sqrt(float(np.mean(vector_errors ** 2))),
    })

    direction_moment = _unit_rows(source_inliers).T.dot(_unit_rows(source_inliers))
    direction_moment /= float(len(source_inliers))
    excitation_eigenvalues = np.linalg.eigvalsh(direction_moment)
    excitation_ratio = float(excitation_eigenvalues[0] / excitation_eigenvalues[-1])
    diagnostics.update({
        "excitation_eigenvalues_ascending": excitation_eigenvalues,
        "excitation_ratio_min_over_max": excitation_ratio,
        "sensor_axis_peak_rate_rad_s": np.max(np.abs(source_inliers), axis=0),
    })
    if excitation_ratio < limits.min_excitation_ratio:
        _fail("三轴激励不足，旋转不可观；不会输出可用安装旋转", diagnostics)
    if diagnostics["retained_fraction"] < limits.min_retained_fraction:
        _fail("有效样本占比过低，数据同步或刚性安装可能有问题", diagnostics)
    if diagnostics["angular_rms_deg"] > limits.max_rms_angle_deg:
        _fail("角速度方向 RMS 误差超过门槛", diagnostics)
    if diagnostics["angular_p95_deg"] > limits.max_p95_angle_deg:
        _fail("角速度方向 P95 误差超过门槛", diagnostics)
    if diagnostics["rate_error_rms_rad_s"] > limits.max_rms_rate_error:
        _fail("角速度向量 RMS 误差超过门槛", diagnostics)

    sensor_acceleration = np.median(
        np.asarray([pair[0].acceleration for pair in stationary_pairs]), axis=0
    )
    fcu_acceleration = np.median(
        np.asarray([pair[1].acceleration for pair in stationary_pairs]), axis=0
    )
    sensor_acceleration_norm = float(np.linalg.norm(sensor_acceleration))
    fcu_acceleration_norm = float(np.linalg.norm(fcu_acceleration))
    diagnostics.update({
        "stationary_sensor_accel_norm_m_s2": sensor_acceleration_norm,
        "stationary_fcu_accel_norm_m_s2": fcu_acceleration_norm,
    })
    if not (limits.min_gravity_norm <= sensor_acceleration_norm <= limits.max_gravity_norm
            and limits.min_gravity_norm <= fcu_acceleration_norm <= limits.max_gravity_norm):
        _fail("静止加速度模长不合理，不能执行重力一致性检查", diagnostics)
    gravity_angle = float(_vector_angles_deg(
        np.asarray([rotation.dot(sensor_acceleration)]),
        np.asarray([fcu_acceleration]),
    )[0])
    diagnostics["gravity_mismatch_deg"] = gravity_angle
    if gravity_angle > limits.max_gravity_angle_deg:
        _fail("求解结果未通过独立的静止重力一致性检查", diagnostics)

    quaternion = _quaternion_xyzw(rotation)
    return rotation, quaternion, bias_sensor, bias_fcu, diagnostics


class RosCollector:
    """Thread-safe subscriber-only ROS capture with a latched safety stop."""

    def __init__(self, rospy_module, imu_type, state_type, arguments):
        self.rospy = rospy_module
        self.lock = threading.RLock()
        self.phase = "idle"
        self.sensor_samples = []
        self.fcu_samples = []
        self.state = None
        self.state_receipt_monotonic = None
        self.safety_fault = ""
        self.invalid_sensor_messages = 0
        self.invalid_fcu_messages = 0
        self.sensor_frames = set()
        self.fcu_frames = set()
        # These are the only three application ROS interfaces: all subscribers.
        self.subscribers = [
            rospy_module.Subscriber(
                arguments.sensor_topic, imu_type, self._sensor_callback, queue_size=1000
            ),
            rospy_module.Subscriber(
                arguments.fcu_topic, imu_type, self._fcu_callback, queue_size=500
            ),
            rospy_module.Subscriber(
                arguments.state_topic, state_type, self._state_callback, queue_size=20
            ),
        ]

    @staticmethod
    def _sample_from_message(message):
        stamp = float(message.header.stamp.to_sec())
        gyro = np.array([
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        ], dtype=float)
        acceleration = np.array([
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        ], dtype=float)
        if not math.isfinite(stamp) or stamp <= 0.0:
            raise ValueError("invalid timestamp")
        if not _finite_vector(gyro) or not _finite_vector(acceleration):
            raise ValueError("non-finite IMU vector")
        return Sample(stamp, gyro, acceleration)

    def _sensor_callback(self, message):
        try:
            sample = self._sample_from_message(message)
        except ValueError:
            with self.lock:
                self.invalid_sensor_messages += 1
            return
        with self.lock:
            self.sensor_frames.add(message.header.frame_id or "<empty>")
            if self.phase != "idle":
                self.sensor_samples.append(sample)

    def _fcu_callback(self, message):
        try:
            sample = self._sample_from_message(message)
        except ValueError:
            with self.lock:
                self.invalid_fcu_messages += 1
            return
        with self.lock:
            self.fcu_frames.add(message.header.frame_id or "<empty>")
            if self.phase != "idle":
                self.fcu_samples.append(sample)

    def _state_callback(self, message):
        with self.lock:
            self.state = message
            self.state_receipt_monotonic = time.monotonic()
            if self.phase != "idle":
                if not message.connected:
                    self.safety_fault = "采样期间 FCU 断开连接"
                elif message.armed:
                    self.safety_fault = "采样期间检测到 FCU 已解锁"

    def state_is_safe(self, max_age_s=1.5):
        with self.lock:
            if self.state is None or self.state_receipt_monotonic is None:
                return False, "等待 MAVROS state"
            if time.monotonic() - self.state_receipt_monotonic > max_age_s:
                return False, "MAVROS state 已过期"
            if not self.state.connected:
                return False, "FCU 未连接"
            if self.state.armed:
                return False, "FCU 已解锁"
            return True, ""

    def begin_phase(self, phase):
        with self.lock:
            safe, reason = self.state_is_safe()
            if not safe:
                raise CalibrationFailure(reason)
            self.phase = phase
            self.sensor_samples = []
            self.fcu_samples = []
            self.safety_fault = ""

    def finish_phase(self):
        with self.lock:
            self.phase = "idle"
            if self.safety_fault:
                raise CalibrationFailure(self.safety_fault)
            safe, reason = self.state_is_safe()
            if not safe:
                raise CalibrationFailure(reason)
            return list(self.sensor_samples), list(self.fcu_samples)

    def assert_sampling_safe(self):
        with self.lock:
            if self.safety_fault:
                raise CalibrationFailure(self.safety_fault)
            safe, reason = self.state_is_safe()
            if not safe:
                self.safety_fault = reason
                raise CalibrationFailure(reason)


def _capture_phase(rospy_module, collector, name, seconds):
    collector.begin_phase(name)
    deadline = time.monotonic() + seconds
    try:
        while not rospy_module.is_shutdown() and time.monotonic() < deadline:
            collector.assert_sampling_safe()
            time.sleep(0.05)
        if rospy_module.is_shutdown():
            raise CalibrationFailure("ROS 正在退出")
        return collector.finish_phase()
    except BaseException:
        with collector.lock:
            collector.phase = "idle"
        raise


def _wait_for_safe_state(rospy_module, collector, timeout_s):
    deadline = time.monotonic() + timeout_s
    last_reason = "等待 MAVROS state"
    while not rospy_module.is_shutdown() and time.monotonic() < deadline:
        safe, last_reason = collector.state_is_safe()
        if safe:
            return
        time.sleep(0.1)
    raise CalibrationFailure("安全状态检查失败：{}".format(last_reason))


def _interactive_confirmation(arguments):
    warning = (
        "安全要求：拆下全部桨叶，将飞机牢固支撑；飞控必须保持 connected=true、"
        "armed=false。随后需手持/固定机体，分别绕前-后 X、左-右 Y、上-下 Z 三轴"
        "缓慢转动。此工具只读，不会切模式或解锁。"
    )
    print(warning)
    if arguments.confirm_props_removed:
        print("已通过 --confirm-props-removed 显式确认拆桨。")
        return
    try:
        answer = input("确认全部桨叶已拆下后，输入 REMOVED 继续：").strip()
    except EOFError:
        raise CalibrationFailure(
            "非交互终端必须在人工确认拆桨后使用 --confirm-props-removed"
        )
    if answer != "REMOVED":
        raise CalibrationFailure("未收到精确的拆桨确认，已停止")


def _validate_frames(collector, expected_fcu_frame):
    with collector.lock:
        sensor_frames = sorted(collector.sensor_frames)
        fcu_frames = sorted(collector.fcu_frames)
        invalid_sensor = collector.invalid_sensor_messages
        invalid_fcu = collector.invalid_fcu_messages
    print("Airy IMU frame_id: {}".format(", ".join(sensor_frames) or "未收到"))
    print("FCU IMU frame_id: {}".format(", ".join(fcu_frames) or "未收到"))
    print("无效消息计数: Airy={}, FCU={}".format(invalid_sensor, invalid_fcu))
    if fcu_frames != [expected_fcu_frame]:
        raise CalibrationFailure(
            "FCU IMU frame_id 必须唯一且等于 {!r}，当前为 {}".format(
                expected_fcu_frame, fcu_frames or ["未收到"]
            )
        )
    if len(sensor_frames) != 1 or "<empty>" in sensor_frames:
        raise CalibrationFailure(
            "Airy IMU frame_id 必须唯一且非空，当前为 {}".format(
                sensor_frames or ["未收到"]
            )
        )


def _print_diagnostics(diagnostics):
    print("质量报告：")
    order = (
        "stationary_pairs", "dynamic_pairs", "dynamic_samples_after_rate_filter",
        "retained_samples", "retained_fraction", "pair_dt_rms_ms",
        "pair_dt_p95_ms", "pair_dt_max_ms",
        "stationary_sensor_gyro_p95_rad_s", "stationary_fcu_gyro_p95_rad_s",
        "excitation_eigenvalues_ascending", "excitation_ratio_min_over_max",
        "sensor_axis_peak_rate_rad_s", "angular_rms_deg", "angular_p95_deg",
        "angular_max_deg", "rate_error_rms_rad_s",
        "stationary_sensor_accel_norm_m_s2", "stationary_fcu_accel_norm_m_s2",
        "gravity_mismatch_deg",
    )
    for key in order:
        if key in diagnostics:
            print("  {:42s} {}".format(key + ":", _format_value(diagnostics[key])))


def _print_success(rotation, quaternion, bias_sensor, bias_fcu, diagnostics):
    _print_diagnostics(diagnostics)
    print("\nPASS：三轴激励、拟合误差和重力一致性均通过。")
    print("R_BI（Airy IMU I -> aircraft base_link/FLU B）：")
    for row in rotation:
        print("  [{: .9f}, {: .9f}, {: .9f}]".format(*row))
    print("四元数 xyzw: [{:.9f}, {:.9f}, {:.9f}, {:.9f}]".format(*quaternion))
    print("静止估计偏置 rad/s（仅本次求解使用，不写入任何配置）：")
    print("  Airy: {}".format(_format_value(bias_sensor)))
    print("  FCU:  {}".format(_format_value(bias_fcu)))
    print("\n可人工复核后填写 shfile/airy_px4.env：")
    print("SENSOR_TO_BODY_Q_X={:.9f}".format(quaternion[0]))
    print("SENSOR_TO_BODY_Q_Y={:.9f}".format(quaternion[1]))
    print("SENSOR_TO_BODY_Q_Z={:.9f}".format(quaternion[2]))
    print("SENSOR_TO_BODY_Q_W={:.9f}".format(quaternion[3]))
    print("工具未写文件、未发业务话题、未改参数、未切模式且未解锁。")


def _run_ros(arguments):
    try:
        import rospy
        from mavros_msgs.msg import State
        from sensor_msgs.msg import Imu
    except ImportError as error:
        raise CalibrationFailure("缺少 ROS Noetic Python 消息依赖：{}".format(error))

    rospy.init_node("calibrate_airy_body", anonymous=False, disable_signals=True)
    collector = RosCollector(rospy, Imu, State, arguments)
    print("只读订阅：{}、{}、{}".format(
        arguments.sensor_topic, arguments.fcu_topic, arguments.state_topic
    ))
    print("应用层 ROS 接口：3 个 Subscriber，0 个 Publisher，0 个 Service。")
    _wait_for_safe_state(rospy, collector, arguments.state_timeout)
    _interactive_confirmation(arguments)

    if arguments.start_delay > 0.0:
        print("{} 秒后开始静止偏置阶段，请保持飞机完全静止。".format(arguments.start_delay))
        deadline = time.monotonic() + arguments.start_delay
        while time.monotonic() < deadline:
            safe, reason = collector.state_is_safe()
            if not safe:
                raise CalibrationFailure(reason)
            time.sleep(0.05)

    print("采集静止数据 {:.1f} 秒……".format(arguments.bias_seconds))
    stationary_sensor, stationary_fcu = _capture_phase(
        rospy, collector, "stationary", arguments.bias_seconds
    )
    print("请绕飞机 X、Y、Z 三轴依次缓慢转动；避免碰撞、平移冲击和快速甩动。")
    if not arguments.confirm_props_removed:
        try:
            input("准备好后按 Enter 开始运动采集：")
        except EOFError:
            raise CalibrationFailure("无法在非交互终端确认开始运动采集")
    else:
        print("运动采集将在 3 秒后开始。")
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            safe, reason = collector.state_is_safe()
            if not safe:
                raise CalibrationFailure(reason)
            time.sleep(0.05)

    print("采集运动数据 {:.1f} 秒……请覆盖三个旋转轴。".format(arguments.capture_seconds))
    dynamic_sensor, dynamic_fcu = _capture_phase(
        rospy, collector, "dynamic", arguments.capture_seconds
    )
    _validate_frames(collector, arguments.expected_fcu_frame)
    max_delta_s = arguments.max_pair_dt_ms / 1000.0
    stationary_pairs = pair_samples(stationary_sensor, stationary_fcu, max_delta_s)
    dynamic_pairs = pair_samples(dynamic_sensor, dynamic_fcu, max_delta_s)
    rotation, quaternion, bias_sensor, bias_fcu, diagnostics = analyse_calibration(
        stationary_pairs, dynamic_pairs, arguments
    )
    _print_success(rotation, quaternion, bias_sensor, bias_fcu, diagnostics)


def _synthetic_samples(rotation, weak_excitation=False):
    random = np.random.RandomState(1907)
    bias_sensor = np.array([0.012, -0.018, 0.009])
    bias_fcu = np.array([-0.006, 0.014, -0.011])
    gravity_fcu = np.array([0.18, -0.22, 9.80])
    gravity_sensor = rotation.T.dot(gravity_fcu)
    stationary = []
    for index in range(400):
        stamp = 10.0 + index * 0.01
        sensor = Sample(
            stamp,
            bias_sensor + random.normal(0.0, 0.002, 3),
            gravity_sensor + random.normal(0.0, 0.015, 3),
        )
        fcu = Sample(
            stamp + random.normal(0.0, 0.0008),
            bias_fcu + random.normal(0.0, 0.002, 3),
            gravity_fcu + random.normal(0.0, 0.015, 3),
        )
        stationary.append((sensor, fcu, fcu.stamp - sensor.stamp))

    dynamic = []
    for index in range(1800):
        if weak_excitation:
            omega_sensor = np.array([
                0.50 + 0.08 * math.sin(index * 0.02), 0.0, 0.0
            ])
        else:
            direction = random.normal(0.0, 1.0, 3)
            direction /= np.linalg.norm(direction)
            omega_sensor = direction * random.uniform(0.25, 0.95)
        stamp = 30.0 + index * 0.01
        sensor = Sample(
            stamp,
            omega_sensor + bias_sensor + random.normal(0.0, 0.003, 3),
            gravity_sensor,
        )
        fcu = Sample(
            stamp + random.normal(0.0, 0.001),
            rotation.dot(omega_sensor) + bias_fcu + random.normal(0.0, 0.003, 3),
            gravity_fcu,
        )
        dynamic.append((sensor, fcu, fcu.stamp - sensor.stamp))
    return stationary, dynamic


def _run_self_test(arguments):
    expected = _rotation_from_axis_angle([0.3, -0.7, 0.4], math.radians(137.0))
    stationary, dynamic = _synthetic_samples(expected)
    rotation, quaternion, _bias_sensor, _bias_fcu, diagnostics = analyse_calibration(
        stationary, dynamic, arguments
    )
    error = _rotation_angle_deg(expected, rotation)
    if error > 0.15:
        raise AssertionError("synthetic rotation error {:.4f}deg".format(error))
    if abs(np.linalg.norm(quaternion) - 1.0) > 1.0e-9:
        raise AssertionError("quaternion is not normalised")
    reconstructed = _wahba(np.eye(3), rotation.T)
    if _rotation_angle_deg(reconstructed, rotation) > 1.0e-6:
        raise AssertionError("rotation reconstruction failed")
    weak_stationary, weak_dynamic = _synthetic_samples(expected, weak_excitation=True)
    try:
        analyse_calibration(weak_stationary, weak_dynamic, arguments)
    except CalibrationFailure as error_failure:
        if "激励不足" not in str(error_failure):
            raise AssertionError("weak excitation failed for wrong reason: {}".format(error_failure))
    else:
        raise AssertionError("weak excitation was incorrectly accepted")

    # Exercise nearest-time unique pairing independently of ROS.
    zero = np.zeros(3)
    sensor = [Sample(index * 0.01, zero, zero) for index in range(10)]
    fcu = [Sample(index * 0.02 + 0.001, zero, zero) for index in range(5)]
    paired = pair_samples(sensor, fcu, 0.003)
    if len(paired) != 5 or max(abs(pair[2]) for pair in paired) > 0.003:
        raise AssertionError("timestamp pairing self-test failed")
    print("SELF-TEST PASS")
    print("  known-rotation error: {:.6f} deg".format(error))
    print("  fit angular RMS: {:.6f} deg".format(diagnostics["angular_rms_deg"]))
    print("  weak three-axis excitation: correctly rejected")
    print("  timestamp pairing: correctly matched unique nearest samples")


def _positive(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise argparse.ArgumentTypeError("must be a finite positive number")
    return result


def _build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "只读标定 Airy IMU(I) 到飞机 base_link/FLU(B) 的安装旋转。"
            "仅在 FCU 已连接且未解锁时采样；失败时不输出可用旋转。"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--self-test", action="store_true", help="不连接 ROS master 的合成数据自测")
    parser.add_argument("--sensor-topic", default="/rslidar_imu_data", help="Airy sensor_msgs/Imu 话题")
    parser.add_argument("--fcu-topic", default="/liu/mavros/imu/data", help="MAVROS sensor_msgs/Imu 话题")
    parser.add_argument("--state-topic", default="/liu/mavros/state", help="MAVROS State 只读话题")
    parser.add_argument("--expected-fcu-frame", default="base_link", help="必须匹配的 FCU IMU FLU frame_id")
    parser.add_argument("--confirm-props-removed", action="store_true", help="显式确认已经拆下全部桨叶（仅用于已人工确认的非交互运行）")
    parser.add_argument("--state-timeout", type=_positive, default=15.0, help="等待 connected 且 disarmed 的秒数")
    parser.add_argument("--start-delay", type=float, default=3.0, help="静止采集前延迟秒数")
    parser.add_argument("--bias-seconds", type=_positive, default=6.0, help="保持完全静止以估计陀螺偏置的秒数")
    parser.add_argument("--capture-seconds", type=_positive, default=45.0, help="三轴缓慢转动采集秒数")
    parser.add_argument("--max-pair-dt-ms", type=_positive, default=12.0, help="两路 IMU 最大近时配对误差")
    parser.add_argument("--min-stationary-pairs", type=int, default=150, help="最少静止配对样本")
    parser.add_argument("--min-pairs", type=int, default=300, help="筛选后最少运动配对样本")
    parser.add_argument("--max-stationary-rate", type=_positive, default=0.06, help="静止去偏后 P95 最大角速度 rad/s")
    parser.add_argument("--min-dynamic-rate", type=_positive, default=0.18, help="动态样本最小角速度 rad/s")
    parser.add_argument("--max-dynamic-rate", type=_positive, default=1.80, help="动态样本最大角速度 rad/s")
    parser.add_argument("--min-rate-ratio", type=_positive, default=0.65, help="FCU/Airy 角速度模长比下限")
    parser.add_argument("--max-rate-ratio", type=_positive, default=1.45, help="FCU/Airy 角速度模长比上限")
    parser.add_argument("--outlier-angle-deg", type=_positive, default=18.0, help="鲁棒筛选最大角方向残差")
    parser.add_argument("--outlier-rate-error", type=_positive, default=0.25, help="鲁棒筛选最大角速度向量误差")
    parser.add_argument("--min-retained-fraction", type=_positive, default=0.65, help="鲁棒筛选最小保留比例")
    parser.add_argument("--min-excitation-ratio", type=_positive, default=0.10, help="方向矩阵最小/最大特征值门槛")
    parser.add_argument("--max-rms-angle-deg", type=_positive, default=6.0, help="拟合角方向 RMS 上限")
    parser.add_argument("--max-p95-angle-deg", type=_positive, default=10.0, help="拟合角方向 P95 上限")
    parser.add_argument("--max-rms-rate-error", type=_positive, default=0.12, help="角速度向量 RMS 误差上限 rad/s")
    parser.add_argument("--min-gravity-norm", type=_positive, default=7.0, help="静止加速度模长下限 m/s^2")
    parser.add_argument("--max-gravity-norm", type=_positive, default=12.5, help="静止加速度模长上限 m/s^2")
    parser.add_argument("--max-gravity-angle-deg", type=_positive, default=8.0, help="映射后双 IMU 重力夹角上限")
    return parser


def _validate_arguments(arguments):
    if arguments.start_delay < 0.0 or not math.isfinite(arguments.start_delay):
        raise CalibrationFailure("--start-delay 必须是有限非负数")
    if arguments.min_stationary_pairs < 20 or arguments.min_pairs < 30:
        raise CalibrationFailure("最少样本门槛过低")
    if arguments.min_dynamic_rate >= arguments.max_dynamic_rate:
        raise CalibrationFailure("动态角速度下限必须小于上限")
    if arguments.min_rate_ratio >= arguments.max_rate_ratio:
        raise CalibrationFailure("角速度模长比下限必须小于上限")
    if not (0.0 < arguments.min_retained_fraction <= 1.0):
        raise CalibrationFailure("--min-retained-fraction 必须在 (0,1] 内")
    if not (0.0 < arguments.min_excitation_ratio < 1.0):
        raise CalibrationFailure("--min-excitation-ratio 必须在 (0,1) 内")
    if arguments.min_gravity_norm >= arguments.max_gravity_norm:
        raise CalibrationFailure("重力模长下限必须小于上限")


def main():
    arguments = _build_parser().parse_args()
    try:
        _validate_arguments(arguments)
        if arguments.self_test:
            _run_self_test(arguments)
        else:
            _run_ros(arguments)
    except CalibrationFailure as error:
        print("\nFAIL：{}".format(error), file=sys.stderr)
        if error.diagnostics:
            _print_diagnostics(error.diagnostics)
        print("未输出可用安装旋转；未写参数/配置，也未切模式或解锁。", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("\n用户中止；没有写入或控制操作。", file=sys.stderr)
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
