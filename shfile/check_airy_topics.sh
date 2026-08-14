#!/usr/bin/env bash
# 检查已启动的 rslidar_sdk 是否提供 FAST-LIO 所需的 Airy 数据。

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly SETUP_FILE="${SCRIPT_DIR}/setup_fastlio2_Airy.bash"

if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "noetic" ]]; then
  printf '[ERROR] 当前终端加载的是 ROS %s，不能与 Noetic 混用。\n' "${ROS_DISTRO}" >&2
  exit 1
fi
if [[ -z "${FAST_LIO_ROOT:-}" ]]; then
  # shellcheck disable=SC1090
  source "${SETUP_FILE}"
fi

python3 - <<'PY'
import math
import sys

import rospy
from sensor_msgs.msg import Imu, PointCloud2

cloud_topic = "/rslidar_points"
imu_topic = "/rslidar_imu_data"
timeout = 20.0

rospy.init_node("check_airy_topics", anonymous=True, disable_signals=True)

try:
    cloud = rospy.wait_for_message(cloud_topic, PointCloud2, timeout=timeout)
except rospy.ROSException as exc:
    sys.exit("[ERROR] 20 秒内没有收到 {}：{}".format(cloud_topic, exc))

field_names = {field.name for field in cloud.fields}
required_fields = {"x", "y", "z", "intensity", "ring", "timestamp"}
missing = sorted(required_fields - field_names)
if missing:
    sys.exit("[ERROR] {} 缺少字段 {}；请用 POINT_TYPE=XYZIRT 重新编译 rslidar_sdk。"
             .format(cloud_topic, ", ".join(missing)))
if cloud.width * cloud.height == 0 or not cloud.data:
    sys.exit("[ERROR] {} 收到空点云。".format(cloud_topic))
if cloud.header.stamp.to_sec() <= 0.0:
    sys.exit("[ERROR] {} 的消息时间戳无效。".format(cloud_topic))

try:
    imu = rospy.wait_for_message(imu_topic, Imu, timeout=timeout)
except rospy.ROSException as exc:
    sys.exit("[ERROR] 20 秒内没有收到 {}：{}".format(imu_topic, exc))

imu_values = (
    imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z,
    imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z,
)
if imu.header.stamp.to_sec() <= 0.0 or not all(math.isfinite(v) for v in imu_values):
    sys.exit("[ERROR] {} 的时间戳或测量值无效。".format(imu_topic))

delta = abs(cloud.header.stamp.to_sec() - imu.header.stamp.to_sec())
if delta > 2.0:
    sys.exit("[ERROR] 点云与 IMU 消息时间相差 {:.3f} 秒，请检查雷达时钟配置。".format(delta))

print("[OK] {}: sensor_msgs/PointCloud2, {} 点".format(
    cloud_topic, cloud.width * cloud.height))
print("[OK] 点字段: {}".format(", ".join(sorted(field_names))))
print("[OK] {}: sensor_msgs/Imu".format(imu_topic))
print("[OK] 点云/IMU 时间差: {:.6f} 秒".format(delta))
print("[OK] Airy ROS 输入满足 FAST-LIO 接口要求。")
PY
