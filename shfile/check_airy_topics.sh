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
from sensor_msgs import point_cloud2
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

point_time_min = math.inf
point_time_max = -math.inf
for (point_time,) in point_cloud2.read_points(
        cloud, field_names=("timestamp",), skip_nans=True):
    if math.isfinite(point_time) and point_time > 0.0:
        point_time_min = min(point_time_min, point_time)
        point_time_max = max(point_time_max, point_time)
if not math.isfinite(point_time_min):
    sys.exit("[ERROR] {} 没有有效逐点 timestamp。".format(cloud_topic))
point_span = point_time_max - point_time_min
if point_span < 0.0 or point_span > 0.2:
    sys.exit("[ERROR] 单帧逐点时间跨度 {:.6f} 秒异常。".format(point_span))

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

now = rospy.Time.now().to_sec()
cloud_age = now - cloud.header.stamp.to_sec()
point_age = now - point_time_max
imu_age = now - imu.header.stamp.to_sec()
for label, age in (("点云 header", cloud_age), ("逐点时间", point_age), ("IMU", imu_age)):
    if age < -0.2 or age > 1.0:
        sys.exit("[ERROR] {} 与 ROS 系统时间不在同一时间轴，age={:.3f} 秒。"
                 .format(label, age))

delta = abs(cloud.header.stamp.to_sec() - imu.header.stamp.to_sec())
if delta > 2.0:
    sys.exit("[ERROR] 点云与 IMU 消息时间相差 {:.3f} 秒，请检查雷达时钟配置。".format(delta))

print("[OK] {}: sensor_msgs/PointCloud2, {} 点".format(
    cloud_topic, cloud.width * cloud.height))
print("[OK] 点字段: {}".format(", ".join(sorted(field_names))))
print("[OK] {}: sensor_msgs/Imu".format(imu_topic))
print("[OK] 单帧逐点时间跨度: {:.6f} 秒".format(point_span))
print("[OK] 时间新鲜度: cloud={:.3f}s point={:.3f}s imu={:.3f}s".format(
    cloud_age, point_age, imu_age))
print("[OK] 点云/IMU 时间差: {:.6f} 秒".format(delta))
print("[OK] Airy ROS 输入满足 FAST-LIO 接口要求。")
PY
