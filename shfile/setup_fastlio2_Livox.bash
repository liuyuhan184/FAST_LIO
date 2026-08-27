#!/usr/bin/env bash
# 使用方法：source shfile/setup_fastlio2_Livox.bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  printf '[ERROR] 请使用 source 加载此文件：source shfile/setup_fastlio2_Livox.bash\n' >&2
  exit 1
fi

if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "noetic" ]]; then
  printf '[ERROR] 当前终端已加载 ROS %s，不能与 ROS Noetic 混用。请打开新终端后重新 source。\n' \
    "${ROS_DISTRO}" >&2
  return 1
fi

_fastlio_setup_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
export FAST_LIO_ROOT="$(cd -- "${_fastlio_setup_dir}/.." && pwd -P)"
export LIVOX_DRIVER_WS="${FAST_LIO_ROOT}/environment/ws_livox"

_fastlio_ros_distro="noetic"
_fastlio_dpkg_arch="$(dpkg --print-architecture)"
_fastlio_build_tag="${_fastlio_ros_distro}-${_fastlio_dpkg_arch}"
_fastlio_ros_setup="/opt/ros/${_fastlio_ros_distro}/setup.bash"
_fastlio_driver_setup="${LIVOX_DRIVER_WS}/devel/${_fastlio_build_tag}/setup.bash"
_fastlio_project_setup="${FAST_LIO_ROOT}/build/${_fastlio_build_tag}/devel/setup.bash"

for _fastlio_setup_file in \
  "${_fastlio_ros_setup}" \
  "${_fastlio_driver_setup}" \
  "${_fastlio_project_setup}"; do
  if [[ ! -f "${_fastlio_setup_file}" ]]; then
    printf '[ERROR] 找不到环境文件：%s\n' "${_fastlio_setup_file}" >&2
    printf '请先运行：bash "%s/shfile/install_fastlio2_Livox.sh"\n' "${FAST_LIO_ROOT}" >&2
    unset _fastlio_setup_dir _fastlio_ros_distro _fastlio_dpkg_arch
    unset _fastlio_build_tag _fastlio_ros_setup _fastlio_driver_setup
    unset _fastlio_project_setup _fastlio_setup_file
    return 1
  fi
done

# 按 Catkin overlay 顺序加载环境。
# shellcheck disable=SC1090
source "${_fastlio_ros_setup}"
# shellcheck disable=SC1090
source "${_fastlio_driver_setup}"
# shellcheck disable=SC1090
source "${_fastlio_project_setup}"

printf '[INFO] FAST-LIO2 环境已加载（%s）。\n' "${_fastlio_build_tag}"

unset _fastlio_setup_dir _fastlio_ros_distro _fastlio_dpkg_arch
unset _fastlio_build_tag _fastlio_ros_setup _fastlio_driver_setup
unset _fastlio_project_setup _fastlio_setup_file
