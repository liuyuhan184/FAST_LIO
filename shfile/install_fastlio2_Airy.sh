#!/usr/bin/env bash
# Ubuntu 20.04 + ROS Noetic 原生部署：RoboSense Airy -> FAST-LIO2

set -Eeuo pipefail
IFS=$'\n\t'

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
readonly RSLIDAR_SOURCE_DIR="${PROJECT_ROOT}/environment/rslidar_sdk"
readonly RSLIDAR_WORKSPACE="${PROJECT_ROOT}/environment/rslidar_ws"
readonly RSLIDAR_PACKAGE_LINK="${RSLIDAR_WORKSPACE}/src/rslidar_sdk"
readonly TARGET_ROS_DISTRO="noetic"

BUILD_JOBS="${BUILD_JOBS:-}"
SKIP_APT_UPDATE="${SKIP_APT_UPDATE:-0}"
INSTALL_DESKTOP_FULL="${INSTALL_DESKTOP_FULL:-0}"
ALLOW_UNSUPPORTED_UBUNTU="${ALLOW_UNSUPPORTED_UBUNTU:-0}"
CURRENT_STAGE="初始化"
SUDO_KEEPALIVE_PID=""
ROOT_CMD=()

if [[ -t 1 ]]; then
  readonly GREEN=$'\033[1;32m' YELLOW=$'\033[1;33m' RED=$'\033[1;31m' RESET=$'\033[0m'
else
  readonly GREEN="" YELLOW="" RED="" RESET=""
fi

info() { printf '%s[INFO]%s %s\n' "${GREEN}" "${RESET}" "$*"; }
warn() { printf '%s[WARN]%s %s\n' "${YELLOW}" "${RESET}" "$*" >&2; }
die() { printf '%s[ERROR]%s %s\n' "${RED}" "${RESET}" "$*" >&2; exit 1; }

usage() {
  cat <<'EOF'
用法：bash shfile/install_fastlio2_Airy.sh [选项]

  --jobs N                 指定并行编译任务数
  --skip-apt-update        跳过 apt-get update
  --desktop-full           安装 ros-noetic-desktop-full（含 RViz）
  --allow-unsupported-os   允许在非 Ubuntu 20.04 上尝试
  -h, --help               显示帮助
EOF
}

while (($# > 0)); do
  case "$1" in
    --jobs)
      (($# >= 2)) || die "--jobs 后必须提供正整数。"
      BUILD_JOBS="$2"
      shift 2
      ;;
    --skip-apt-update) SKIP_APT_UPDATE=1; shift ;;
    --desktop-full) INSTALL_DESKTOP_FULL=1; shift ;;
    --allow-unsupported-os) ALLOW_UNSUPPORTED_UBUNTU=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "未知选项：$1" ;;
  esac
done

[[ -z "${BUILD_JOBS}" || "${BUILD_JOBS}" =~ ^[1-9][0-9]*$ ]] || \
  die "编译任务数必须为正整数，当前值：${BUILD_JOBS}"
if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "${TARGET_ROS_DISTRO}" ]]; then
  die "当前终端加载的不是 ROS Noetic，请打开新终端后再运行。"
fi

cleanup() {
  if [[ -n "${SUDO_KEEPALIVE_PID}" ]]; then
    kill "${SUDO_KEEPALIVE_PID}" 2>/dev/null || true
    wait "${SUDO_KEEPALIVE_PID}" 2>/dev/null || true
  fi
}

on_error() {
  local code=$?
  printf '%s[ERROR]%s 阶段“%s”失败（第 %s 行，退出码 %s）。\n' \
    "${RED}" "${RESET}" "${CURRENT_STAGE}" "${BASH_LINENO[0]:-unknown}" "${code}" >&2
  printf '修正上方第一条错误后可直接重新运行，构建会增量继续。\n' >&2
  exit "${code}"
}
trap cleanup EXIT
trap on_error ERR

require_file() { [[ -f "$1" ]] || die "缺少文件：$1"; }

validate_sources() {
  CURRENT_STAGE="检查 Airy 与 FAST-LIO2 源码"
  require_file "${RSLIDAR_SOURCE_DIR}/CMakeLists.txt"
  require_file "${RSLIDAR_SOURCE_DIR}/package.xml"
  require_file "${RSLIDAR_SOURCE_DIR}/config/config_airy.yaml"
  require_file "${RSLIDAR_SOURCE_DIR}/launch/start_airy.launch"
  require_file "${RSLIDAR_SOURCE_DIR}/src/rs_driver/CMakeLists.txt"
  require_file "${RSLIDAR_SOURCE_DIR}/src/rs_driver/src/rs_driver/driver/decoder/decoder_RSAIRY.hpp"
  require_file "${PROJECT_ROOT}/config/airy.yaml"
  require_file "${PROJECT_ROOT}/launch/mapping_airy.launch"
  info "官方 rslidar_sdk、rs_driver 子模块和 FAST-LIO2 Airy 配置检查通过。"
}

acquire_sudo() {
  CURRENT_STAGE="获取管理员权限"
  if ((EUID == 0)); then
    warn "当前以 root 运行，构建文件可能归 root 所有。"
    ROOT_CMD=()
    return
  fi
  command -v sudo >/dev/null 2>&1 || die "未安装 sudo。"
  info "需要使用 sudo 安装 rslidar_sdk/FAST-LIO2 依赖，请输入密码。"
  sudo -v
  ROOT_CMD=(sudo)
  (while sudo -n true 2>/dev/null; do sleep 50; done) &
  SUDO_KEEPALIVE_PID=$!
}

detect_platform() {
  CURRENT_STAGE="检测系统和架构"
  require_file /etc/os-release
  # shellcheck disable=SC1091
  source /etc/os-release
  [[ "${ID:-}" == "ubuntu" ]] || die "仅支持 Ubuntu，当前为 ${PRETTY_NAME:-unknown}。"
  if [[ "${VERSION_ID:-}" != "20.04" && "${ALLOW_UNSUPPORTED_UBUNTU}" != "1" ]]; then
    die "ROS Noetic 官方目标系统为 Ubuntu 20.04；当前为 ${PRETTY_NAME}。"
  fi
  if [[ "${VERSION_ID:-}" != "20.04" ]]; then
    warn "正在非官方系统 ${PRETTY_NAME} 上尝试 ROS Noetic 构建。"
  fi

  readonly DPKG_ARCH="$(dpkg --print-architecture)"
  readonly KERNEL_ARCH="$(uname -m)"
  case "${DPKG_ARCH}" in
    amd64|i386|arm64|armhf) ;;
    *) die "不支持架构 ${DPKG_ARCH}（kernel=${KERNEL_ARCH}）。" ;;
  esac

  local model="unknown"
  if [[ -r /proc/device-tree/model ]]; then
    model="$(tr '\0' ' ' </proc/device-tree/model)"
  elif command -v lscpu >/dev/null 2>&1; then
    model="$(lscpu | awk -F: '/Model name/ && !found++ {sub(/^[[:space:]]+/, "", $2); print $2}')"
  fi

  readonly BUILD_TAG="noetic-${DPKG_ARCH}"
  readonly RSLIDAR_BUILD_DIR="${RSLIDAR_WORKSPACE}/build/${BUILD_TAG}"
  readonly RSLIDAR_DEVEL_DIR="${RSLIDAR_WORKSPACE}/devel/${BUILD_TAG}"
  readonly FASTLIO_BUILD_DIR="${PROJECT_ROOT}/build/${BUILD_TAG}-airy"
  readonly FASTLIO_DEVEL_DIR="${FASTLIO_BUILD_DIR}/devel"
  info "系统：${PRETTY_NAME}"
  info "架构：${DPKG_ARCH} / ${KERNEL_ARCH}"
  info "处理器/开发板：${model:-unknown}"
}

choose_jobs() {
  CURRENT_STAGE="计算编译并行度"
  local cpu_jobs memory_kb memory_jobs
  cpu_jobs="$(nproc 2>/dev/null || printf '1')"
  memory_kb="$(awk '/MemAvailable:/ {print $2; exit}' /proc/meminfo)"
  memory_jobs=$((memory_kb / 2097152))
  ((memory_jobs >= 1)) || memory_jobs=1
  if [[ -z "${BUILD_JOBS}" ]]; then
    BUILD_JOBS="${cpu_jobs}"
    ((BUILD_JOBS <= memory_jobs)) || BUILD_JOBS="${memory_jobs}"
    ((BUILD_JOBS <= 4)) || BUILD_JOBS=4
  fi
  info "并行任务：${BUILD_JOBS}（可用内存 $((memory_kb / 1024)) MiB）"
}

install_dependencies() {
  CURRENT_STAGE="安装基础依赖"
  local -a packages=(
    build-essential cmake git pkg-config
    libboost-all-dev libeigen3-dev libpcap-dev libpcl-dev libyaml-cpp-dev
    python3-dev python3-matplotlib
    ros-noetic-ros-base ros-noetic-catkin ros-noetic-roscpp ros-noetic-rospy
    ros-noetic-roslib ros-noetic-std-msgs ros-noetic-sensor-msgs
    ros-noetic-geometry-msgs ros-noetic-nav-msgs ros-noetic-visualization-msgs
    ros-noetic-tf ros-noetic-pcl-ros ros-noetic-pcl-conversions
    ros-noetic-eigen-conversions ros-noetic-message-generation
    ros-noetic-message-runtime ros-noetic-rosbag ros-noetic-roslaunch
  )
  [[ "${INSTALL_DESKTOP_FULL}" != "1" ]] || packages+=(ros-noetic-desktop-full)
  if [[ "${SKIP_APT_UPDATE}" != "1" ]]; then
    "${ROOT_CMD[@]}" apt-get update
  else
    warn "已跳过 apt-get update。"
  fi
  "${ROOT_CMD[@]}" env DEBIAN_FRONTEND=noninteractive \
    apt-get install -y --no-install-recommends "${packages[@]}"
  require_file /opt/ros/noetic/setup.bash
}

prepare_rslidar_workspace() {
  CURRENT_STAGE="准备 rslidar_sdk Catkin 工作空间"
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
  mkdir -p "${RSLIDAR_WORKSPACE}/src"
  if [[ -e "${RSLIDAR_PACKAGE_LINK}" || -L "${RSLIDAR_PACKAGE_LINK}" ]]; then
    [[ "$(realpath "${RSLIDAR_PACKAGE_LINK}")" == "${RSLIDAR_SOURCE_DIR}" ]] || \
      die "${RSLIDAR_PACKAGE_LINK} 没有指向 ${RSLIDAR_SOURCE_DIR}。"
  else
    ln -s ../../rslidar_sdk "${RSLIDAR_PACKAGE_LINK}"
  fi
  if [[ ! -e "${RSLIDAR_WORKSPACE}/src/CMakeLists.txt" ]]; then
    catkin_init_workspace "${RSLIDAR_WORKSPACE}/src"
  fi
}

build_rslidar() {
  CURRENT_STAGE="编译 Airy 官方 ROS 驱动"
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
  info "[1/2] 编译 rslidar_sdk：XYZIRT 点型，启用 Airy IMU。"
  catkin_make \
    -C "${RSLIDAR_WORKSPACE}" \
    --source "${RSLIDAR_WORKSPACE}/src" \
    --build "${RSLIDAR_BUILD_DIR}" \
    --force-cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DPOINT_TYPE=XYZIRT \
    -DENABLE_IMU_DATA_PARSE=ON \
    -DCATKIN_DEVEL_PREFIX="${RSLIDAR_DEVEL_DIR}" \
    -j"${BUILD_JOBS}" -l"${BUILD_JOBS}"

  require_file "${RSLIDAR_DEVEL_DIR}/setup.bash"
  [[ -x "${RSLIDAR_DEVEL_DIR}/lib/rslidar_sdk/rslidar_sdk_node" ]] || \
    die "没有生成 rslidar_sdk_node。"
  grep -q '^POINT_TYPE:STRING=XYZIRT$' "${RSLIDAR_BUILD_DIR}/CMakeCache.txt" || \
    die "rslidar_sdk 点型不是 XYZIRT。"
  grep -q '^ENABLE_IMU_DATA_PARSE:BOOL=ON$' "${RSLIDAR_BUILD_DIR}/CMakeCache.txt" || \
    die "Airy IMU 解析未启用。"
}

build_fastlio() {
  CURRENT_STAGE="编译 Airy 版 FAST-LIO2"
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
  # shellcheck disable=SC1090
  source "${RSLIDAR_DEVEL_DIR}/setup.bash"
  info "[2/2] 编译 FAST-LIO2 Airy 时间戳适配。"
  cmake -S "${PROJECT_ROOT}" -B "${FASTLIO_BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=Release -DENABLE_LIVOX_SUPPORT=OFF
  cmake --build "${FASTLIO_BUILD_DIR}" --parallel "${BUILD_JOBS}"
  require_file "${FASTLIO_DEVEL_DIR}/setup.bash"
  [[ -x "${FASTLIO_DEVEL_DIR}/lib/fast_lio/fastlio_mapping" ]] || \
    die "没有生成 fastlio_mapping。"
}

verify() {
  CURRENT_STAGE="验证 Airy 部署"
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
  # shellcheck disable=SC1090
  source "${RSLIDAR_DEVEL_DIR}/setup.bash"
  # shellcheck disable=SC1090
  source "${FASTLIO_DEVEL_DIR}/setup.bash"

  local driver="${RSLIDAR_DEVEL_DIR}/lib/rslidar_sdk/rslidar_sdk_node"
  local fastlio="${FASTLIO_DEVEL_DIR}/lib/fast_lio/fastlio_mapping"
  local missing
  missing="$(ldd "${driver}" "${fastlio}" | awk '/not found/ {print}' || true)"
  [[ -z "${missing}" ]] || die "存在未解析动态库：${missing}"
  [[ "$(realpath "$(rospack find rslidar_sdk)")" == "${RSLIDAR_SOURCE_DIR}" ]] || \
    die "ROS 没有找到 environment/rslidar_sdk。"
  [[ "$(rospack find fast_lio)" == "${PROJECT_ROOT}" ]] || die "ROS 没有找到当前 fast_lio。"
  roslaunch --files rslidar_sdk start_airy.launch >/dev/null
  roslaunch --files fast_lio mapping_airy.launch >/dev/null

  printf '\n%sAiry + FAST-LIO2 部署完成。%s\n' "${GREEN}" "${RESET}"
  printf '加载环境：source %q\n' "${SCRIPT_DIR}/setup_fastlio2_Airy.bash"
  printf '启动驱动：roslaunch rslidar_sdk start_airy.launch\n'
  printf '驱动与建图：roslaunch fast_lio mapping_airy.launch\n'
}

main() {
  validate_sources
  acquire_sudo
  detect_platform
  choose_jobs
  install_dependencies
  prepare_rslidar_workspace
  build_rslidar
  build_fastlio
  verify
}

main
