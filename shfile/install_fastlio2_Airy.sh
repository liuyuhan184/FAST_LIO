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
readonly AIRY_SYSCTL_TEMPLATE="${SCRIPT_DIR}/99-fastlio-airy.conf"
readonly AIRY_SYSCTL_TARGET="/etc/sysctl.d/99-fastlio-airy.conf"

BUILD_JOBS="${BUILD_JOBS:-}"
SKIP_APT_UPDATE="${SKIP_APT_UPDATE:-0}"
INSTALL_DESKTOP_FULL="${INSTALL_DESKTOP_FULL:-0}"
ALLOW_UNSUPPORTED_UBUNTU="${ALLOW_UNSUPPORTED_UBUNTU:-0}"
BUILD_ONLY="${BUILD_ONLY:-0}"
ONLINE_ONLY="${ONLINE_ONLY:-0}"
CONFIGURE_NETWORK=0
NETWORK_CONNECTION=""
AIRY_HOST_CIDR="${AIRY_HOST_CIDR:-192.168.1.102/24}"
AIRY_LIDAR_IP="${AIRY_LIDAR_IP:-192.168.1.200}"
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

默认行为：安装 Ubuntu/ROS 依赖，编译支持在线雷达和离线 PCAP 的 Airy 驱动，
再编译 FAST-LIO2，并持久配置 Airy 所需 UDP 接收缓冲。重复运行会复用已有源码、
增量构建并幂等更新 /etc/sysctl.d/99-fastlio-airy.conf。

构建选项：
  --jobs N                    指定并行编译任务数
  --build-only                不运行 sudo/apt，仅检查依赖并编译
  --online-only               禁用 PCAP 解析，不要求 libpcap-dev
  --skip-apt-update           安装依赖前跳过 apt-get update
  --desktop-full              安装 ros-noetic-desktop-full（含 RViz）
  --allow-unsupported-os      允许在非 20.04 的其他 Ubuntu 版本上尝试

可选网络配置（只有明确传入 --configure-network 才会修改连接）：
  --configure-network NAME    将 NetworkManager 有线连接 NAME 配为 Airy 专网
  --host-cidr IPv4/PREFIX     板端静态地址（默认 192.168.1.102/24）
  --lidar-ip IPv4             雷达地址/连通性检查目标（默认 192.168.1.200）
  注：--build-only 为保证零 sudo，不能与 --configure-network 同时使用。

其他：
  -h, --help                  显示帮助

常用示例：
  bash shfile/install_fastlio2_Airy.sh
  bash shfile/install_fastlio2_Airy.sh --build-only --online-only
  bash shfile/install_fastlio2_Airy.sh --configure-network '有线连接 1'

对应环境变量：BUILD_JOBS、SKIP_APT_UPDATE、INSTALL_DESKTOP_FULL、
ALLOW_UNSUPPORTED_UBUNTU、BUILD_ONLY、ONLINE_ONLY、AIRY_HOST_CIDR、
AIRY_LIDAR_IP。网络连接名必须通过 --configure-network 显式传入。
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
    --build-only) BUILD_ONLY=1; shift ;;
    --online-only) ONLINE_ONLY=1; shift ;;
    --configure-network)
      (($# >= 2)) || die "--configure-network 后必须提供 NetworkManager 连接名。"
      [[ -n "$2" && "$2" != --* ]] || die "--configure-network 的连接名无效：$2"
      NETWORK_CONNECTION="$2"
      CONFIGURE_NETWORK=1
      shift 2
      ;;
    --host-cidr)
      (($# >= 2)) || die "--host-cidr 后必须提供 IPv4/CIDR。"
      AIRY_HOST_CIDR="$2"
      shift 2
      ;;
    --lidar-ip)
      (($# >= 2)) || die "--lidar-ip 后必须提供 IPv4 地址。"
      AIRY_LIDAR_IP="$2"
      shift 2
      ;;
    -h|--help) usage; exit 0 ;;
    *) die "未知选项：$1" ;;
  esac
done

validate_switch() {
  local name="$1" value="$2"
  [[ "${value}" == "0" || "${value}" == "1" ]] || \
    die "${name} 只能为 0 或 1，当前值：${value}"
}

is_ipv4() {
  local address="$1" octet
  local -a octets=()
  local IFS=.
  read -r -a octets <<<"${address}"
  ((${#octets[@]} == 4)) || return 1
  for octet in "${octets[@]}"; do
    [[ "${octet}" =~ ^[0-9]{1,3}$ ]] || return 1
    ((10#${octet} <= 255)) || return 1
  done
}

is_ipv4_cidr() {
  local cidr="$1" address prefix
  [[ "${cidr}" == */* ]] || return 1
  address="${cidr%/*}"
  prefix="${cidr##*/}"
  is_ipv4 "${address}" || return 1
  [[ "${prefix}" =~ ^[0-9]{1,2}$ ]] || return 1
  ((10#${prefix} >= 1 && 10#${prefix} <= 32))
}

validate_switch SKIP_APT_UPDATE "${SKIP_APT_UPDATE}"
validate_switch INSTALL_DESKTOP_FULL "${INSTALL_DESKTOP_FULL}"
validate_switch ALLOW_UNSUPPORTED_UBUNTU "${ALLOW_UNSUPPORTED_UBUNTU}"
validate_switch BUILD_ONLY "${BUILD_ONLY}"
validate_switch ONLINE_ONLY "${ONLINE_ONLY}"
[[ -z "${BUILD_JOBS}" || "${BUILD_JOBS}" =~ ^[1-9][0-9]*$ ]] || \
  die "编译任务数必须为正整数，当前值：${BUILD_JOBS}"
is_ipv4_cidr "${AIRY_HOST_CIDR}" || die "无效的板端 IPv4/CIDR：${AIRY_HOST_CIDR}"
is_ipv4 "${AIRY_LIDAR_IP}" || die "无效的雷达 IPv4 地址：${AIRY_LIDAR_IP}"
[[ "${AIRY_HOST_CIDR%/*}" != "${AIRY_LIDAR_IP}" ]] || \
  die "板端地址和雷达地址不能相同：${AIRY_LIDAR_IP}"
if [[ "${BUILD_ONLY}" == "1" && "${CONFIGURE_NETWORK}" == "1" ]]; then
  die "--build-only 保证不使用 sudo/apt，不能同时配置网络；请去掉 --configure-network，或单独运行完整安装。"
fi
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
require_command() { command -v "$1" >/dev/null 2>&1 || die "缺少命令：$1"; }

collect_dependency_packages() {
  local destination="$1"
  # Ubuntu 20.04 默认 Bash 5，使用 nameref 避免安装和检查维护两份依赖清单。
  local -n packages_ref="${destination}"
  packages_ref=(
    build-essential cmake git pkg-config coreutils procps util-linux
    libboost-all-dev libeigen3-dev libpcl-dev libyaml-cpp-dev
    python3-dev python3-matplotlib python3-numpy geographiclib-tools
    ros-noetic-ros-base ros-noetic-catkin ros-noetic-roscpp ros-noetic-rospy
    ros-noetic-roslib ros-noetic-std-msgs ros-noetic-sensor-msgs
    ros-noetic-geometry-msgs ros-noetic-nav-msgs ros-noetic-visualization-msgs
    ros-noetic-tf ros-noetic-pcl-ros ros-noetic-pcl-conversions
    ros-noetic-diagnostic-msgs ros-noetic-mavros ros-noetic-mavros-extras
    ros-noetic-eigen-conversions ros-noetic-message-generation
    ros-noetic-message-runtime ros-noetic-rosbag ros-noetic-roslaunch
  )
  [[ "${ONLINE_ONLY}" == "1" ]] || packages_ref+=(libpcap-dev)
  [[ "${INSTALL_DESKTOP_FULL}" != "1" ]] || packages_ref+=(ros-noetic-desktop-full)
  [[ "${CONFIGURE_NETWORK}" != "1" ]] || packages_ref+=(network-manager)
}

validate_sources() {
  CURRENT_STAGE="检查 Airy 与 FAST-LIO2 源码"
  require_file "${PROJECT_ROOT}/CMakeLists.txt"
  require_file "${PROJECT_ROOT}/package.xml"
  require_file "${RSLIDAR_SOURCE_DIR}/CMakeLists.txt"
  require_file "${RSLIDAR_SOURCE_DIR}/package.xml"
  require_file "${RSLIDAR_SOURCE_DIR}/config/config_airy.yaml"
  require_file "${RSLIDAR_SOURCE_DIR}/launch/start_airy.launch"
  require_file "${RSLIDAR_SOURCE_DIR}/src/rs_driver/CMakeLists.txt"
  require_file "${RSLIDAR_SOURCE_DIR}/src/rs_driver/src/rs_driver/driver/decoder/decoder_RSAIRY.hpp"
  require_file "${PROJECT_ROOT}/config/airy.yaml"
  require_file "${PROJECT_ROOT}/launch/mapping_airy.launch"
  require_file "${PROJECT_ROOT}/launch/mavros_px4_safe.launch"
  require_file "${PROJECT_ROOT}/shfile/start_airy_px4.sh"
  require_file "${PROJECT_ROOT}/shfile/stop_airy_px4.sh"
  require_file "${PROJECT_ROOT}/shfile/fastlio_to_mavros.py"
  require_file "${PROJECT_ROOT}/shfile/monitor_airy_px4.py"
  require_file "${PROJECT_ROOT}/shfile/calibrate_airy_body.py"
  require_file "${PROJECT_ROOT}/shfile/check_airy_topics.sh"
  require_file "${PROJECT_ROOT}/shfile/airy_px4.env.example"
  require_file "${AIRY_SYSCTL_TEMPLATE}"
  grep -Eq '^[[:space:]]*use_lidar_clock:[[:space:]]*false([[:space:]]|$)' \
    "${RSLIDAR_SOURCE_DIR}/config/config_airy.yaml" || \
    warn "use_lidar_clock 不是 false；只有已验证 PTP/GPS UTC 同步时才允许使用雷达时钟。"
  info "rslidar_sdk、仓库内 rs_driver 源码和 FAST-LIO2 Airy 配置检查通过。"
}

acquire_sudo() {
  CURRENT_STAGE="获取管理员权限"
  if ((EUID == 0)); then
    warn "当前以 root 运行，构建文件可能归 root 所有。"
    ROOT_CMD=()
    return
  fi
  command -v sudo >/dev/null 2>&1 || die "未安装 sudo。"
  if [[ "${CONFIGURE_NETWORK}" == "1" ]]; then
    info "需要使用 sudo 安装依赖并配置 Airy 有线网络，请输入密码。"
  else
    info "需要使用 sudo 安装 rslidar_sdk/FAST-LIO2 依赖，请输入密码。"
  fi
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
  local -a packages=()
  collect_dependency_packages packages
  # 运行手册中的网络验收工具；不列入 --build-only 的编译依赖检查。
  packages+=(iproute2 iputils-ping tcpdump)
  if [[ "${SKIP_APT_UPDATE}" != "1" ]]; then
    "${ROOT_CMD[@]}" apt-get update
  else
    warn "已跳过 apt-get update。"
  fi
  "${ROOT_CMD[@]}" env DEBIAN_FRONTEND=noninteractive \
    apt-get install -y --no-install-recommends "${packages[@]}"
  require_file /opt/ros/noetic/setup.bash
  local geographiclib_installer="/opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh"
  if [[ ! -s /usr/share/GeographicLib/geoids/egm96-5.pgm && \
        ! -s /usr/local/share/GeographicLib/geoids/egm96-5.pgm ]]; then
    require_file "${geographiclib_installer}"
    info "安装 MAVROS 坐标转换所需 GeographicLib 数据集（首次运行需要网络）。"
    "${ROOT_CMD[@]}" bash "${geographiclib_installer}"
    if [[ ! -s /usr/share/GeographicLib/geoids/egm96-5.pgm && \
          ! -s /usr/local/share/GeographicLib/geoids/egm96-5.pgm ]]; then
      die "GeographicLib 安装脚本结束后仍缺少 egm96-5 数据集。"
    fi
  fi
}

validate_build_environment() {
  CURRENT_STAGE="检查本机构建依赖"
  require_command dpkg-query
  local -a packages=() missing=()
  local package status missing_text=""
  collect_dependency_packages packages
  for package in "${packages[@]}"; do
    status="$(dpkg-query -W -f='${db:Status-Status}' "${package}" 2>/dev/null || true)"
    [[ "${status}" == "installed" ]] || missing+=("${package}")
  done
  if ((${#missing[@]} > 0)); then
    printf -v missing_text ' %q' "${missing[@]}"
    die "--build-only 检测到缺失依赖：${missing_text}。请去掉 --build-only 安装，或在仅使用在线雷达时加 --online-only。"
  fi
  require_file /opt/ros/noetic/setup.bash
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
  for package in catkin roscpp sensor_msgs pcl_ros pcl_conversions eigen_conversions \
    diagnostic_msgs mavros mavros_msgs mavros_extras; do
    rospack find "${package}" >/dev/null 2>&1 || die "ROS Noetic 缺少包：${package}"
  done
  for package in cmake catkin_make g++ make pkg-config ldd roslaunch rospack \
    flock realpath setsid ps timeout; do
    require_command "${package}"
  done
  if [[ ! -s /usr/share/GeographicLib/geoids/egm96-5.pgm && \
        ! -s /usr/local/share/GeographicLib/geoids/egm96-5.pgm ]]; then
    die "缺少 MAVROS 所需 GeographicLib egm96-5 数据；请运行 sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh。"
  fi
  if [[ "${ONLINE_ONLY}" == "1" ]]; then
    info "--build-only 依赖检查通过（在线模式，不要求 libpcap-dev）。"
  else
    info "--build-only 依赖检查通过（含离线 PCAP 支持）。"
  fi
}

configure_airy_network() {
  CURRENT_STAGE="配置 Airy 有线网络"
  require_command nmcli
  local connection_type method addresses gateway never_default ipv6_method
  connection_type="$("${ROOT_CMD[@]}" nmcli -g connection.type connection show "${NETWORK_CONNECTION}" 2>/dev/null)" || \
    die "找不到 NetworkManager 连接：${NETWORK_CONNECTION}。可用 nmcli connection show 查看连接名。"
  case "${connection_type}" in
    802-3-ethernet|ethernet) ;;
    *) die "连接“${NETWORK_CONNECTION}”不是有线连接（类型：${connection_type:-unknown}）。" ;;
  esac

  info "将“${NETWORK_CONNECTION}”配置为 ${AIRY_HOST_CIDR}，雷达目标 ${AIRY_LIDAR_IP}。"
  "${ROOT_CMD[@]}" nmcli connection modify "${NETWORK_CONNECTION}" \
    connection.autoconnect yes \
    ipv4.method manual \
    ipv4.addresses "${AIRY_HOST_CIDR}" \
    ipv4.gateway "" \
    ipv4.dns "" \
    ipv4.never-default yes \
    ipv6.method disabled

  if "${ROOT_CMD[@]}" nmcli connection up "${NETWORK_CONNECTION}"; then
    info "Airy 有线连接已激活。"
  else
    warn "配置已经保存，但连接暂时无法激活；请检查网线和 Airy 供电后执行：nmcli connection up '${NETWORK_CONNECTION}'"
  fi

  method="$("${ROOT_CMD[@]}" nmcli -g ipv4.method connection show "${NETWORK_CONNECTION}")"
  addresses="$("${ROOT_CMD[@]}" nmcli -g ipv4.addresses connection show "${NETWORK_CONNECTION}")"
  gateway="$("${ROOT_CMD[@]}" nmcli -g ipv4.gateway connection show "${NETWORK_CONNECTION}")"
  never_default="$("${ROOT_CMD[@]}" nmcli -g ipv4.never-default connection show "${NETWORK_CONNECTION}")"
  ipv6_method="$("${ROOT_CMD[@]}" nmcli -g ipv6.method connection show "${NETWORK_CONNECTION}")"
  [[ "${method}" == "manual" ]] || die "网络配置复核失败：ipv4.method=${method}"
  grep -Fxq "${AIRY_HOST_CIDR}" <<<"${addresses}" || \
    die "网络配置复核失败：ipv4.addresses=${addresses}"
  [[ -z "${gateway}" || "${gateway}" == "--" ]] || \
    die "网络配置复核失败：ipv4.gateway=${gateway}"
  [[ "${never_default}" == "yes" ]] || \
    die "网络配置复核失败：ipv4.never-default=${never_default}"
  [[ "${ipv6_method}" == "disabled" ]] || \
    die "网络配置复核失败：ipv6.method=${ipv6_method}"
  info "网络配置复核通过：无网关、不接管默认路由、IPv6 已禁用。"
}

configure_airy_udp_buffers() {
  CURRENT_STAGE="配置 Airy UDP 接收缓冲"
  local rmem_max="0" backlog="0"

  if [[ "${BUILD_ONLY}" != "1" ]]; then
    "${ROOT_CMD[@]}" install -m 0644 "${AIRY_SYSCTL_TEMPLATE}" "${AIRY_SYSCTL_TARGET}"
    "${ROOT_CMD[@]}" sysctl -p "${AIRY_SYSCTL_TARGET}" >/dev/null
    info "已持久写入 ${AIRY_SYSCTL_TARGET}。"
  fi

  rmem_max="$(sysctl -n net.core.rmem_max 2>/dev/null || printf '0')"
  backlog="$(sysctl -n net.core.netdev_max_backlog 2>/dev/null || printf '0')"
  if [[ "${rmem_max}" =~ ^[0-9]+$ ]] && ((rmem_max >= 4194304)); then
    info "Airy UDP 接收上限检查通过：net.core.rmem_max=${rmem_max}。"
  else
    warn "net.core.rmem_max=${rmem_max:-unknown}，低于 rs_driver 请求的 4194304；在线运行可能丢包。"
  fi
  if [[ "${backlog}" =~ ^[0-9]+$ ]] && ((backlog >= 5000)); then
    info "网卡接收 backlog 检查通过：net.core.netdev_max_backlog=${backlog}。"
  else
    warn "net.core.netdev_max_backlog=${backlog:-unknown}，高负载时可能出现接收队列丢包。"
  fi
}

check_airy_network() {
  CURRENT_STAGE="检查 Airy 网络"
  local route="" expected_host="${AIRY_HOST_CIDR%/*}"
  if command -v ip >/dev/null 2>&1; then
    if route="$(ip -4 route get "${AIRY_LIDAR_IP}" 2>/dev/null)"; then
      info "Airy 路由：${route}"
      if [[ "${route}" != *" src ${expected_host} "* && "${route}" != *" src ${expected_host}" ]]; then
        warn "到 ${AIRY_LIDAR_IP} 的源地址不是建议值 ${expected_host}；请检查有线网口地址。"
      fi
    else
      warn "当前没有到 Airy ${AIRY_LIDAR_IP} 的 IPv4 路由。编译不受影响，运行前请配置雷达网口。"
    fi
  else
    warn "缺少 ip 命令，跳过 Airy 路由检查。"
  fi

  if command -v ping >/dev/null 2>&1; then
    if ping -c 1 -W 1 "${AIRY_LIDAR_IP}" >/dev/null 2>&1; then
      info "Airy ${AIRY_LIDAR_IP} 可以连通。"
    else
      warn "Airy ${AIRY_LIDAR_IP} 暂未响应 ping。若雷达未接电/未接线可忽略，运行前再检查。"
    fi
  else
    warn "缺少 ping 命令，跳过 Airy 连通性检查。"
  fi
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
  local disable_pcap_parse="OFF"
  [[ "${ONLINE_ONLY}" != "1" ]] || disable_pcap_parse="ON"
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
  if [[ "${ONLINE_ONLY}" == "1" ]]; then
    info "[1/2] 编译 rslidar_sdk：XYZIRT + Airy IMU，仅在线 UDP（禁用 PCAP）。"
  else
    info "[1/2] 编译 rslidar_sdk：XYZIRT + Airy IMU，启用在线 UDP 和离线 PCAP。"
  fi
  catkin_make \
    -C "${RSLIDAR_WORKSPACE}" \
    --source "${RSLIDAR_WORKSPACE}/src" \
    --build "${RSLIDAR_BUILD_DIR}" \
    --force-cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DPOINT_TYPE=XYZIRT \
    -DENABLE_IMU_DATA_PARSE=ON \
    -DDISABLE_PCAP_PARSE="${disable_pcap_parse}" \
    -DCATKIN_DEVEL_PREFIX="${RSLIDAR_DEVEL_DIR}" \
    -j"${BUILD_JOBS}" -l"${BUILD_JOBS}"

  require_file "${RSLIDAR_DEVEL_DIR}/setup.bash"
  [[ -x "${RSLIDAR_DEVEL_DIR}/lib/rslidar_sdk/rslidar_sdk_node" ]] || \
    die "没有生成 rslidar_sdk_node。"
  [[ -s "${RSLIDAR_DEVEL_DIR}/lib/rslidar_sdk/rslidar_sdk_node" ]] || \
    die "rslidar_sdk_node 为空文件。"
  require_file "${RSLIDAR_BUILD_DIR}/CMakeCache.txt"
  local driver_flags="${RSLIDAR_BUILD_DIR}/rslidar_sdk/CMakeFiles/rslidar_sdk_node.dir/flags.make"
  require_file "${driver_flags}"
  grep -q '^CMAKE_BUILD_TYPE:STRING=Release$' "${RSLIDAR_BUILD_DIR}/CMakeCache.txt" || \
    die "rslidar_sdk 不是 Release 构建。"
  grep -q '^POINT_TYPE:STRING=XYZIRT$' "${RSLIDAR_BUILD_DIR}/CMakeCache.txt" || \
    die "rslidar_sdk 点型不是 XYZIRT。"
  grep -q '^ENABLE_IMU_DATA_PARSE:BOOL=ON$' "${RSLIDAR_BUILD_DIR}/CMakeCache.txt" || \
    die "Airy IMU 解析未启用。"
  grep -q "^DISABLE_PCAP_PARSE:BOOL=${disable_pcap_parse}$" \
    "${RSLIDAR_BUILD_DIR}/CMakeCache.txt" || \
    die "rslidar_sdk PCAP 模式与请求不一致。"
  grep -q -- '-DPOINT_TYPE_XYZIRT' "${driver_flags}" || \
    die "rslidar_sdk 实际编译宏不是 POINT_TYPE_XYZIRT。"
  grep -q -- '-DENABLE_IMU_DATA_PARSE' "${driver_flags}" || \
    die "rslidar_sdk 实际编译时没有启用 Airy IMU。"
  if [[ "${disable_pcap_parse}" == "ON" ]]; then
    grep -q -- '-DDISABLE_PCAP_PARSE' "${driver_flags}" || \
      die "--online-only 已请求，但实际编译宏没有禁用 PCAP。"
  elif grep -q -- '-DDISABLE_PCAP_PARSE' "${driver_flags}"; then
    die "默认完整模式已请求，但实际编译宏仍禁用了 PCAP。"
  fi
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
  [[ -s "${FASTLIO_DEVEL_DIR}/lib/fast_lio/fastlio_mapping" ]] || \
    die "fastlio_mapping 为空文件。"
  require_file "${FASTLIO_BUILD_DIR}/CMakeCache.txt"
  local fastlio_flags="${FASTLIO_BUILD_DIR}/CMakeFiles/fastlio_mapping.dir/flags.make"
  require_file "${fastlio_flags}"
  grep -q '^CMAKE_BUILD_TYPE:STRING=Release$' "${FASTLIO_BUILD_DIR}/CMakeCache.txt" || \
    die "FAST-LIO2 不是 Release 构建。"
  grep -q '^ENABLE_LIVOX_SUPPORT:BOOL=OFF$' "${FASTLIO_BUILD_DIR}/CMakeCache.txt" || \
    die "FAST-LIO2 Airy 构建错误地启用了 Livox 依赖。"
  grep -q -- '-DMP_PROC_NUM=' "${fastlio_flags}" || \
    die "FAST-LIO2 构建缺少匹配线程数宏。"
  if (( $(nproc) > 2 )); then
    grep -q -- '-DMP_EN' "${fastlio_flags}" || \
      die "多核平台上的 FAST-LIO2 没有启用 OpenMP 匹配循环。"
  fi
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
  [[ -x "${driver}" && -s "${driver}" ]] || die "Airy 驱动产物无效：${driver}"
  [[ -x "${fastlio}" && -s "${fastlio}" ]] || die "FAST-LIO2 产物无效：${fastlio}"
  missing="$(ldd "${driver}" "${fastlio}" | awk '/not found/ {print}' || true)"
  [[ -z "${missing}" ]] || die "存在未解析动态库：${missing}"
  ldd "${fastlio}" | grep -q 'libgomp' || die "FAST-LIO2 没有链接 OpenMP/libgomp。"
  [[ "$(realpath "$(rospack find rslidar_sdk)")" == "${RSLIDAR_SOURCE_DIR}" ]] || \
    die "ROS 没有找到 environment/rslidar_sdk。"
  [[ "$(rospack find fast_lio)" == "${PROJECT_ROOT}" ]] || die "ROS 没有找到当前 fast_lio。"
  roslaunch --files rslidar_sdk start_airy.launch >/dev/null
  roslaunch --files fast_lio mapping_airy.launch >/dev/null
  roslaunch --files "${PROJECT_ROOT}/launch/mavros_px4_safe.launch" >/dev/null
  bash -n "${SCRIPT_DIR}/setup_fastlio2_Airy.bash"
  bash -n "${SCRIPT_DIR}/check_airy_topics.sh"
  bash -n "${SCRIPT_DIR}/start_airy_px4.sh"
  bash -n "${SCRIPT_DIR}/stop_airy_px4.sh"
  python3 -m py_compile \
    "${SCRIPT_DIR}/fastlio_to_mavros.py" \
    "${SCRIPT_DIR}/monitor_airy_px4.py" \
    "${SCRIPT_DIR}/calibrate_airy_body.py"
  python3 "${SCRIPT_DIR}/fastlio_to_mavros.py" --self-test >/dev/null
  python3 "${SCRIPT_DIR}/calibrate_airy_body.py" --self-test >/dev/null

  printf '\n%sAiry + FAST-LIO2 部署完成。%s\n' "${GREEN}" "${RESET}"
  if [[ "${ONLINE_ONLY}" == "1" ]]; then
    printf '构建模式：在线 UDP（PCAP 已禁用，不支持驱动直接回放 PCAP 文件）\n'
  else
    printf '构建模式：完整模式（在线 UDP + 离线 PCAP）\n'
  fi
  printf '雷达网络：板端 %s -> Airy %s（MSOP 6699 / DIFOP 7788 / IMU 6688）\n' \
    "${AIRY_HOST_CIDR}" "${AIRY_LIDAR_IP}"
  if [[ "${CONFIGURE_NETWORK}" == "1" ]]; then
    printf '网络连接：%s（已写入 NetworkManager 并完成参数复核）\n' "${NETWORK_CONNECTION}"
  fi
  printf '驱动产物：%s\n' "${driver}"
  printf '算法产物：%s\n' "${fastlio}"
  printf '加载环境：source %q\n' "${SCRIPT_DIR}/setup_fastlio2_Airy.bash"
  printf '雷达 SLAM 启动：roslaunch fast_lio mapping_airy.launch\n'
  printf '飞机配置模板：cp %q %q\n' \
    "${SCRIPT_DIR}/airy_px4.env.example" "${SCRIPT_DIR}/airy_px4.env"
  printf '飞机端首次诊断：bash %q --diagnostic-only --test-seconds 30\n' \
    "${SCRIPT_DIR}/start_airy_px4.sh"
  printf '飞机端停止全部相关进程：bash %q\n' "${SCRIPT_DIR}/stop_airy_px4.sh"
  printf '仅启动驱动：roslaunch rslidar_sdk start_airy.launch\n'
  printf '数据自检（另一个已加载环境的终端）：bash %q\n' "${SCRIPT_DIR}/check_airy_topics.sh"
}

main() {
  validate_sources
  detect_platform
  choose_jobs
  if [[ "${BUILD_ONLY}" == "1" ]]; then
    [[ "${SKIP_APT_UPDATE}" != "1" ]] || warn "--build-only 下 --skip-apt-update 不生效。"
    [[ "${INSTALL_DESKTOP_FULL}" != "1" ]] || \
      warn "--build-only 不会安装 desktop-full，只会检查它是否已存在。"
    validate_build_environment
  else
    acquire_sudo
    install_dependencies
  fi
  [[ "${CONFIGURE_NETWORK}" != "1" ]] || configure_airy_network
  configure_airy_udp_buffers
  prepare_rslidar_workspace
  build_rslidar
  build_fastlio
  check_airy_network
  verify
}

main
