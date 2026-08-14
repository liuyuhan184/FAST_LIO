#!/usr/bin/env bash
# 使用方法：source shfile/setup_fastlio2_Airy.bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  printf '[ERROR] 请使用 source 加载本文件。\n' >&2
  exit 1
fi
if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "noetic" ]]; then
  printf '[ERROR] 当前终端已加载 ROS %s，不能与 Noetic 混用。\n' "${ROS_DISTRO}" >&2
  return 1
fi

_airy_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
export FAST_LIO_ROOT="$(cd -- "${_airy_script_dir}/.." && pwd -P)"
export RSLIDAR_SDK_ROOT="${FAST_LIO_ROOT}/environment/rslidar_sdk"
export RSLIDAR_WS="${FAST_LIO_ROOT}/environment/rslidar_ws"
_airy_arch="$(dpkg --print-architecture)"
_airy_tag="noetic-${_airy_arch}"
_airy_ros_setup="/opt/ros/noetic/setup.bash"
_airy_driver_setup="${RSLIDAR_WS}/devel/${_airy_tag}/setup.bash"
_airy_fastlio_setup="${FAST_LIO_ROOT}/build/${_airy_tag}-airy/devel/setup.bash"

# A stale ROS_HOSTNAME/ROS_MASTER_URI from another Wi-Fi or onboard network
# makes roslaunch wait for a host that no longer exists. Airy and FAST-LIO run
# from a local master by default. Set AIRY_KEEP_ROS_NETWORK=1 before sourcing
# this file when intentionally using a remote/multi-machine ROS master.
if [[ "${AIRY_KEEP_ROS_NETWORK:-0}" != "1" ]]; then
  unset ROS_HOSTNAME ROS_IP
  export ROS_MASTER_URI="http://127.0.0.1:11311"
fi

for _airy_setup in "${_airy_ros_setup}" "${_airy_driver_setup}" "${_airy_fastlio_setup}"; do
  if [[ ! -f "${_airy_setup}" ]]; then
    printf '[ERROR] 找不到 %s，请先运行 install_fastlio2_Airy.sh。\n' "${_airy_setup}" >&2
    unset _airy_script_dir _airy_arch _airy_tag _airy_ros_setup
    unset _airy_driver_setup _airy_fastlio_setup _airy_setup
    return 1
  fi
done

# shellcheck disable=SC1090
source "${_airy_ros_setup}"
# shellcheck disable=SC1090
source "${_airy_driver_setup}"
# shellcheck disable=SC1090
source "${_airy_fastlio_setup}"
printf '[INFO] Airy + FAST-LIO2 环境已加载（%s，ROS master: %s）。\n' \
  "${_airy_tag}" "${ROS_MASTER_URI}"

unset _airy_script_dir _airy_arch _airy_tag _airy_ros_setup
unset _airy_driver_setup _airy_fastlio_setup _airy_setup
