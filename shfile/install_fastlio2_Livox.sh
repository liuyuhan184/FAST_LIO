#!/usr/bin/env bash
# 在 Ubuntu + ROS Noetic 主机上原生编译并部署：
# Livox-SDK -> livox_ros_driver -> FAST-LIO2

set -Eeuo pipefail
IFS=$'\n\t'

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
readonly SDK_SOURCE_DIR="${PROJECT_ROOT}/environment/Livox-SDK"
readonly DRIVER_WORKSPACE="${PROJECT_ROOT}/environment/ws_livox"
readonly DRIVER_SOURCE_DIR="${DRIVER_WORKSPACE}/src/livox_ros_driver"

INITIAL_ROS_DISTRO="${ROS_DISTRO:-}"
ROS_DISTRO="noetic"
BUILD_JOBS="${BUILD_JOBS:-}"
SKIP_APT_UPDATE="${SKIP_APT_UPDATE:-0}"
INSTALL_DESKTOP_FULL="${INSTALL_DESKTOP_FULL:-0}"
ALLOW_UNSUPPORTED_UBUNTU="${ALLOW_UNSUPPORTED_UBUNTU:-0}"

CURRENT_STAGE="初始化"
SUDO_KEEPALIVE_PID=""
ROOT_CMD=()

if [[ -t 1 ]]; then
  readonly COLOR_GREEN=$'\033[1;32m'
  readonly COLOR_YELLOW=$'\033[1;33m'
  readonly COLOR_RED=$'\033[1;31m'
  readonly COLOR_BLUE=$'\033[1;34m'
  readonly COLOR_RESET=$'\033[0m'
else
  readonly COLOR_GREEN=""
  readonly COLOR_YELLOW=""
  readonly COLOR_RED=""
  readonly COLOR_BLUE=""
  readonly COLOR_RESET=""
fi

info() { printf '%s[INFO]%s %s\n' "${COLOR_GREEN}" "${COLOR_RESET}" "$*"; }
note() { printf '%s[NOTE]%s %s\n' "${COLOR_BLUE}" "${COLOR_RESET}" "$*"; }
warn() { printf '%s[WARN]%s %s\n' "${COLOR_YELLOW}" "${COLOR_RESET}" "$*" >&2; }
die() {
  printf '%s[ERROR]%s %s\n' "${COLOR_RED}" "${COLOR_RESET}" "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
用法：
  bash shfile/install_fastlio2_Livox.sh [选项]

选项：
  --jobs N                 指定并行编译任务数
  --skip-apt-update        跳过 apt-get update
  --desktop-full           额外安装 ros-noetic-desktop-full（板端通常不需要）
  --allow-unsupported-os   允许在非 20.04 的其他 Ubuntu 版本上尝试部署
  -h, --help               显示帮助

也可使用环境变量 BUILD_JOBS、SKIP_APT_UPDATE、INSTALL_DESKTOP_FULL 和
ALLOW_UNSUPPORTED_UBUNTU 设置上述选项。
EOF
}

while (($# > 0)); do
  case "$1" in
    --jobs)
      (($# >= 2)) || die "--jobs 后必须提供正整数。"
      BUILD_JOBS="$2"
      shift 2
      ;;
    --skip-apt-update)
      SKIP_APT_UPDATE=1
      shift
      ;;
    --desktop-full)
      INSTALL_DESKTOP_FULL=1
      shift
      ;;
    --allow-unsupported-os)
      ALLOW_UNSUPPORTED_UBUNTU=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "未知选项：$1（使用 --help 查看帮助）"
      ;;
  esac
done

if [[ -n "${BUILD_JOBS}" && ! "${BUILD_JOBS}" =~ ^[1-9][0-9]*$ ]]; then
  die "编译任务数必须为正整数，当前值：${BUILD_JOBS}"
fi
if [[ -n "${INITIAL_ROS_DISTRO}" && "${INITIAL_ROS_DISTRO}" != "noetic" ]]; then
  die "当前终端已加载 ROS ${INITIAL_ROS_DISTRO}。请打开未加载其他 ROS 发行版的新终端，再运行本脚本。"
fi

cleanup() {
  if [[ -n "${SUDO_KEEPALIVE_PID}" ]]; then
    kill "${SUDO_KEEPALIVE_PID}" 2>/dev/null || true
    wait "${SUDO_KEEPALIVE_PID}" 2>/dev/null || true
  fi
}

on_error() {
  local exit_code=$?
  local line_no=${BASH_LINENO[0]:-unknown}
  printf '%s[ERROR]%s 阶段“%s”失败（脚本第 %s 行，退出码 %s）。\n' \
    "${COLOR_RED}" "${COLOR_RESET}" "${CURRENT_STAGE}" "${line_no}" "${exit_code}" >&2
  printf '请查看上方第一条编译/安装错误；再次运行本脚本可从已有结果继续。\n' >&2
  exit "${exit_code}"
}

trap cleanup EXIT
trap on_error ERR

require_file() {
  [[ -f "$1" ]] || die "缺少文件：$1"
}

require_dir() {
  [[ -d "$1" ]] || die "缺少目录：$1"
}

acquire_sudo() {
  CURRENT_STAGE="获取管理员权限"
  if ((EUID == 0)); then
    ROOT_CMD=()
    warn "当前脚本由 root 运行；生成的部分构建文件可能归 root 所有。"
    return
  fi

  command -v sudo >/dev/null 2>&1 || die "未安装 sudo；请先以 root 身份安装 sudo。"
  info "后续需要安装 APT 依赖并将 Livox-SDK 安装到 /usr/local，请输入 sudo 密码。"
  sudo -v
  ROOT_CMD=(sudo)

  # 编译期间刷新 sudo 时间戳，避免长时间构建后再次询问密码。
  (
    while sudo -n true 2>/dev/null; do
      sleep 50
    done
  ) &
  SUDO_KEEPALIVE_PID=$!
}

detect_platform() {
  CURRENT_STAGE="检测系统与 CPU 架构"
  require_file /etc/os-release
  # shellcheck disable=SC1091
  source /etc/os-release

  [[ "${ID:-}" == "ubuntu" ]] || die "仅支持 Ubuntu；当前系统为 ${PRETTY_NAME:-unknown}。"
  if [[ "${VERSION_ID:-}" != "20.04" ]]; then
    if [[ "${ALLOW_UNSUPPORTED_UBUNTU}" == "1" ]]; then
      warn "ROS Noetic 官方二进制目标系统是 Ubuntu 20.04；正在 ${PRETTY_NAME} 上尝试。"
    else
      die "检测到 ${PRETTY_NAME}。ROS Noetic 官方目标系统是 Ubuntu 20.04；如已自行配置好 Noetic，可加 --allow-unsupported-os 尝试。"
    fi
  fi

  command -v dpkg >/dev/null 2>&1 || die "系统缺少 dpkg，无法识别 Debian/Ubuntu 架构。"
  readonly DPKG_ARCH="$(dpkg --print-architecture)"
  readonly KERNEL_ARCH="$(uname -m)"
  case "${DPKG_ARCH}" in
    amd64|i386|arm64|armhf)
      ;;
    *)
      die "暂不支持架构 ${DPKG_ARCH}（uname -m=${KERNEL_ARCH}）。支持 amd64、i386、arm64 和 armhf。"
      ;;
  esac

  local board_model="unknown"
  if [[ -r /proc/device-tree/model ]]; then
    board_model="$(tr '\0' ' ' </proc/device-tree/model)"
  elif command -v lscpu >/dev/null 2>&1; then
    board_model="$(lscpu | awk -F: '/Model name/ && !found++ {sub(/^[[:space:]]+/, "", $2); print $2}')"
  fi

  readonly BUILD_TAG="${ROS_DISTRO}-${DPKG_ARCH}"
  readonly SDK_BUILD_DIR="${SDK_SOURCE_DIR}/build/fastlio_deploy_${BUILD_TAG}"
  readonly DRIVER_BUILD_DIR="${DRIVER_WORKSPACE}/build/${BUILD_TAG}"
  readonly DRIVER_DEVEL_DIR="${DRIVER_WORKSPACE}/devel/${BUILD_TAG}"
  readonly FASTLIO_BUILD_DIR="${PROJECT_ROOT}/build/${BUILD_TAG}"
  readonly FASTLIO_DEVEL_DIR="${FASTLIO_BUILD_DIR}/devel"

  info "系统：${PRETTY_NAME:-unknown}"
  info "架构：dpkg=${DPKG_ARCH}, kernel=${KERNEL_ARCH}"
  info "处理器/开发板：${board_model:-unknown}"
  if [[ "${DPKG_ARCH}" == "arm64" ]] && grep -Eiq 'rk3588|rockchip.*3588' <<<"${board_model}"; then
    note "已识别 RK3588（ARM64），将执行板端原生编译，不使用交叉编译器。"
  fi
}

choose_build_jobs() {
  CURRENT_STAGE="计算安全的编译并行度"
  local cpu_jobs memory_kb memory_jobs
  cpu_jobs="$(nproc 2>/dev/null || printf '1')"
  memory_kb="$(awk '/MemAvailable:/ {print $2; exit}' /proc/meminfo 2>/dev/null || true)"
  [[ -n "${memory_kb}" ]] || memory_kb="$(awk '/MemTotal:/ {print $2; exit}' /proc/meminfo)"

  # FAST-LIO/PCL 的 C++ 模板编译占用较大，每个任务预留约 2 GiB。
  memory_jobs=$((memory_kb / 2097152))
  ((memory_jobs >= 1)) || memory_jobs=1

  if [[ -z "${BUILD_JOBS}" ]]; then
    BUILD_JOBS="${cpu_jobs}"
    ((BUILD_JOBS <= memory_jobs)) || BUILD_JOBS="${memory_jobs}"
    ((BUILD_JOBS <= 4)) || BUILD_JOBS=4
  fi
  [[ "${BUILD_JOBS}" =~ ^[1-9][0-9]*$ ]] || die "编译任务数必须为正整数，当前值：${BUILD_JOBS}"
  info "编译并行度：${BUILD_JOBS}（CPU 逻辑核心：${cpu_jobs}，可用内存：$((memory_kb / 1024)) MiB）"
}

install_dependencies() {
  CURRENT_STAGE="安装基础编译与 ROS 依赖"
  local -a packages=(
    build-essential
    cmake
    git
    pkg-config
    libapr1-dev
    libboost-all-dev
    libeigen3-dev
    libpcl-dev
    python3-dev
    python3-matplotlib
    "ros-${ROS_DISTRO}-ros-base"
    "ros-${ROS_DISTRO}-catkin"
    "ros-${ROS_DISTRO}-roscpp"
    "ros-${ROS_DISTRO}-rospy"
    "ros-${ROS_DISTRO}-std-msgs"
    "ros-${ROS_DISTRO}-sensor-msgs"
    "ros-${ROS_DISTRO}-geometry-msgs"
    "ros-${ROS_DISTRO}-nav-msgs"
    "ros-${ROS_DISTRO}-visualization-msgs"
    "ros-${ROS_DISTRO}-tf"
    "ros-${ROS_DISTRO}-pcl-ros"
    "ros-${ROS_DISTRO}-pcl-conversions"
    "ros-${ROS_DISTRO}-eigen-conversions"
    "ros-${ROS_DISTRO}-message-generation"
    "ros-${ROS_DISTRO}-message-runtime"
    "ros-${ROS_DISTRO}-rosbag"
    "ros-${ROS_DISTRO}-roslaunch"
  )
  if [[ "${INSTALL_DESKTOP_FULL}" == "1" ]]; then
    packages+=("ros-${ROS_DISTRO}-desktop-full")
  fi

  if [[ "${SKIP_APT_UPDATE}" != "1" ]]; then
    info "更新 APT 软件索引……"
    "${ROOT_CMD[@]}" apt-get update
  else
    warn "已按要求跳过 apt-get update。"
  fi

  info "安装完整编译依赖（已安装的软件包会被 APT 自动跳过）……"
  "${ROOT_CMD[@]}" env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "${packages[@]}"

  require_file "/opt/ros/${ROS_DISTRO}/setup.bash"
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  command -v catkin_make >/dev/null 2>&1 || die "安装后仍找不到 catkin_make。"
  command -v catkin_init_workspace >/dev/null 2>&1 || die "安装后仍找不到 catkin_init_workspace。"
}

validate_sources() {
  CURRENT_STAGE="检查本地源码"
  require_dir "${SDK_SOURCE_DIR}"
  require_file "${SDK_SOURCE_DIR}/CMakeLists.txt"
  require_file "${SDK_SOURCE_DIR}/sdk_core/include/livox_sdk.h"
  require_dir "${DRIVER_WORKSPACE}/src"
  require_file "${DRIVER_SOURCE_DIR}/CMakeLists.txt"
  require_file "${DRIVER_SOURCE_DIR}/package.xml"
  require_file "${DRIVER_SOURCE_DIR}/msg/CustomMsg.msg"
  require_file "${PROJECT_ROOT}/CMakeLists.txt"
  require_file "${PROJECT_ROOT}/package.xml"
  require_file "${PROJECT_ROOT}/include/ikd-Tree/ikd_Tree.cpp"
  info "三套源码结构检查通过。"
}

build_livox_sdk() {
  CURRENT_STAGE="编译 Livox-SDK"
  info "[1/3] 配置 Livox-SDK：${SDK_BUILD_DIR}"
  cmake -S "${SDK_SOURCE_DIR}" -B "${SDK_BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release
  cmake --build "${SDK_BUILD_DIR}" --parallel "${BUILD_JOBS}"

  CURRENT_STAGE="安装 Livox-SDK"
  info "将 Livox-SDK 安装到 /usr/local……"
  "${ROOT_CMD[@]}" cmake --install "${SDK_BUILD_DIR}"
  "${ROOT_CMD[@]}" ldconfig
  require_file /usr/local/include/livox_sdk.h
  require_file /usr/local/lib/liblivox_sdk_static.a
  info "Livox-SDK 编译并安装成功。"
}

build_livox_driver() {
  CURRENT_STAGE="编译 livox_ros_driver"
  # 官方源码布局是 ws_livox/src/livox_ros_driver；补齐 Catkin 顶层文件。
  if [[ ! -e "${DRIVER_WORKSPACE}/src/CMakeLists.txt" ]]; then
    catkin_init_workspace "${DRIVER_WORKSPACE}/src"
  fi

  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  info "[2/3] 编译 livox_ros_driver：${DRIVER_BUILD_DIR}"
  catkin_make \
    -C "${DRIVER_WORKSPACE}" \
    --source "${DRIVER_WORKSPACE}/src" \
    --build "${DRIVER_BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCATKIN_DEVEL_PREFIX="${DRIVER_DEVEL_DIR}" \
    -j"${BUILD_JOBS}" \
    -l"${BUILD_JOBS}"

  require_file "${DRIVER_DEVEL_DIR}/setup.bash"
  require_file "${DRIVER_DEVEL_DIR}/include/livox_ros_driver/CustomMsg.h"
  [[ -x "${DRIVER_DEVEL_DIR}/lib/livox_ros_driver/livox_ros_driver_node" ]] || \
    die "livox_ros_driver 可执行文件未生成。"
  info "livox_ros_driver 编译成功。"
}

build_fastlio() {
  CURRENT_STAGE="编译 FAST-LIO2"
  # 按 overlay 顺序加载，确保 FAST-LIO 能找到 livox_ros_driver/CustomMsg.h。
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  # shellcheck disable=SC1090
  source "${DRIVER_DEVEL_DIR}/setup.bash"

  info "[3/3] 配置 FAST-LIO2：${FASTLIO_BUILD_DIR}"
  cmake \
    -S "${PROJECT_ROOT}" \
    -B "${FASTLIO_BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=Release
  cmake --build "${FASTLIO_BUILD_DIR}" --parallel "${BUILD_JOBS}"

  require_file "${FASTLIO_DEVEL_DIR}/setup.bash"
  [[ -x "${FASTLIO_DEVEL_DIR}/lib/fast_lio/fastlio_mapping" ]] || \
    die "FAST-LIO2 可执行文件未生成。"
  info "FAST-LIO2 编译成功。"
}

verify_installation() {
  CURRENT_STAGE="验证最终部署"
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  # shellcheck disable=SC1090
  source "${DRIVER_DEVEL_DIR}/setup.bash"
  # shellcheck disable=SC1090
  source "${FASTLIO_DEVEL_DIR}/setup.bash"

  local fastlio_executable="${FASTLIO_DEVEL_DIR}/lib/fast_lio/fastlio_mapping"
  local driver_executable="${DRIVER_DEVEL_DIR}/lib/livox_ros_driver/livox_ros_driver_node"
  local missing_libraries=""
  missing_libraries="$(ldd "${fastlio_executable}" | awk '/not found/ {print}' || true)"
  missing_libraries+="$(ldd "${driver_executable}" | awk '/not found/ {print}' || true)"
  [[ -z "${missing_libraries}" ]] || die "存在未解析的动态库：${missing_libraries}"

  [[ "$(rospack find livox_ros_driver)" == "${DRIVER_SOURCE_DIR}" ]] || \
    die "ROS 找到的 livox_ros_driver 不是本项目内的源码。"
  [[ "$(rospack find fast_lio)" == "${PROJECT_ROOT}" ]] || \
    die "ROS 无法定位当前 FAST-LIO2 包。"
  roslaunch --files fast_lio mapping_avia.launch >/dev/null

  info "动态库、ROS 包索引和 FAST-LIO launch 文件验证通过。"
  printf '\n%s部署完成。%s\n' "${COLOR_GREEN}" "${COLOR_RESET}"
  printf '当前平台：%s / %s\n' "${PRETTY_NAME}" "${DPKG_ARCH}"
  printf '环境加载：source %q\n' "${SCRIPT_DIR}/setup_fastlio2_Livox.bash"
  printf 'FAST-LIO（板端无界面）：roslaunch fast_lio mapping_avia.launch rviz:=false\n'
  printf 'Livox 驱动：roslaunch livox_ros_driver livox_lidar_msg.launch\n'
}

main() {
  validate_sources
  acquire_sudo
  detect_platform
  choose_build_jobs

  local available_kb
  available_kb="$(df -Pk "${PROJECT_ROOT}" | awk 'NR == 2 {print $4}')"
  if [[ -n "${available_kb}" ]] && ((available_kb < 6291456)); then
    warn "项目所在磁盘可用空间不足 6 GiB，APT 安装或 PCL/FAST-LIO 编译可能失败。"
  fi

  install_dependencies
  build_livox_sdk
  build_livox_driver
  build_fastlio
  verify_installation
}

main
