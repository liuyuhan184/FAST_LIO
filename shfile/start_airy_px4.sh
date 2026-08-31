#!/usr/bin/env bash
# Airy + FAST-LIO2 + MAVROS/PX4 + safe vision bridge, one-key launcher.

set -Eeuo pipefail
IFS=$'\n\t'

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
readonly AIRY_SETUP="${SCRIPT_DIR}/setup_fastlio2_Airy.bash"
readonly BRIDGE_SCRIPT="${SCRIPT_DIR}/fastlio_to_mavros.py"
readonly MONITOR_SCRIPT="${SCRIPT_DIR}/monitor_airy_px4.py"
readonly MAVROS_SAFE_LAUNCH="${PROJECT_ROOT}/launch/mavros_px4_safe.launch"
readonly MIN_AIRY_UDP_RMEM_MAX=4194304

# Safe defaults.  A local shfile/airy_px4.env may override them.
UAV_NAME="${UAV_NAME:-liu}"
FCU_URL="${FCU_URL:-/dev/ttyTHS0:921600}"
GCS_URL="${GCS_URL:-}"
MAV_SYS_ID="${MAV_SYS_ID:-1}"
MAV_COMP_ID="${MAV_COMP_ID:-1}"
EXPECTED_PX4_MAJOR="${EXPECTED_PX4_MAJOR:-1}"
EXPECTED_PX4_MINOR="${EXPECTED_PX4_MINOR:-13}"
ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
START_ROSCORE="${START_ROSCORE:-1}"
START_MAVROS="${START_MAVROS:-1}"
START_FASTLIO="${START_FASTLIO:-1}"
START_BRIDGE="${START_BRIDGE:-1}"
START_MONITOR="${START_MONITOR:-1}"

# Publishing requires every gate below; defaults are intentionally blocked.
PUBLISH_VISION="${PUBLISH_VISION:-0}"
MOUNT_CONFIRMED="${MOUNT_CONFIRMED:-0}"
DIRECTION_TEST_CONFIRMED="${DIRECTION_TEST_CONFIRMED:-0}"
ALLOW_APPROXIMATE_DIRECTION_PUBLISH="${ALLOW_APPROXIMATE_DIRECTION_PUBLISH:-0}"
ALLOW_VISION_YAW_FUSION="${ALLOW_VISION_YAW_FUSION:-0}"
SENSOR_TO_BODY_Q_X="${SENSOR_TO_BODY_Q_X:-0.0}"
SENSOR_TO_BODY_Q_Y="${SENSOR_TO_BODY_Q_Y:-0.0}"
SENSOR_TO_BODY_Q_Z="${SENSOR_TO_BODY_Q_Z:-0.0}"
SENSOR_TO_BODY_Q_W="${SENSOR_TO_BODY_Q_W:-1.0}"
SENSOR_POSITION_IN_BODY_X="${SENSOR_POSITION_IN_BODY_X:-0.0}"
SENSOR_POSITION_IN_BODY_Y="${SENSOR_POSITION_IN_BODY_Y:-0.0}"
SENSOR_POSITION_IN_BODY_Z="${SENSOR_POSITION_IN_BODY_Z:-0.0}"

ALIGN_STABLE_SECONDS="${ALIGN_STABLE_SECONDS:-5.0}"
MIN_FASTLIO_RATE_HZ="${MIN_FASTLIO_RATE_HZ:-8.0}"
FLIGHT_MIN_POSE_RATE_HZ="${FLIGHT_MIN_POSE_RATE_HZ:-30.0}"
MIN_LOCAL_POSE_RATE_HZ="${MIN_LOCAL_POSE_RATE_HZ:-2.0}"
MAX_SOURCE_AGE_S="${MAX_SOURCE_AGE_S:-0.5}"
MAX_SOURCE_GAP_S="${MAX_SOURCE_GAP_S:-0.30}"
MAX_RECEIPT_STALL_S="${MAX_RECEIPT_STALL_S:-0.50}"
MAX_POSITION_JUMP_M="${MAX_POSITION_JUMP_M:-1.0}"
MAX_ORIENTATION_JUMP_DEG="${MAX_ORIENTATION_JUMP_DEG:-45.0}"
MAX_GRAVITY_MISMATCH_DEG="${MAX_GRAVITY_MISMATCH_DEG:-10.0}"
SESSION_ROOT="${SESSION_ROOT:-${PROJECT_ROOT}/runtime/airy_px4}"

CONFIG_FILE="${AIRY_PX4_CONFIG:-${SCRIPT_DIR}/airy_px4.env}"
CONFIG_EXPLICIT=0
CONFIG_FILE_LOADED=0
DIAGNOSTIC_ONLY=0
DRY_RUN=0
TEST_SECONDS=0
PRINT_STATUS_ONLY=0
UDP_RMEM_MAX="unknown"

ORIGINAL_ARGS=("$@")
for ((argument_index = 0; argument_index < ${#ORIGINAL_ARGS[@]}; argument_index++)); do
  if [[ "${ORIGINAL_ARGS[argument_index]}" == "--config" ]]; then
    ((argument_index + 1 < ${#ORIGINAL_ARGS[@]})) || {
      printf '[ERROR] --config 后必须提供文件路径。\n' >&2
      exit 2
    }
    CONFIG_FILE="${ORIGINAL_ARGS[argument_index + 1]}"
    CONFIG_EXPLICIT=1
  fi
done

if [[ -f "${CONFIG_FILE}" ]]; then
  # shellcheck disable=SC1090
  source "${CONFIG_FILE}"
  CONFIG_FILE_LOADED=1
elif [[ "${CONFIG_EXPLICIT}" == "1" ]]; then
  printf '[ERROR] 找不到配置文件：%s\n' "${CONFIG_FILE}" >&2
  exit 2
fi

usage() {
  cat <<'EOF'
用法：bash shfile/start_airy_px4.sh [选项]

一键启动 Airy 驱动、FAST-LIO2、MAVROS/PX4 串口链路、坐标桥和只读监控器。
脚本绝不自动切模式、解锁、起飞、写 PX4 参数或发送 OFFBOARD 控制量。

选项：
  --config FILE          使用指定的本机安装/外参配置
  --diagnostic-only      强制禁止向 MAVROS 发布外部视觉，仅做整链路诊断
  --publish-vision       请求发布；仍必须通过安装外参和 PX4 参数等硬门控
  --test-seconds N       运行 N 秒未解锁台架测试后自动退出
  --dry-run              打印配置和将启动的组件，不访问硬件
  --status-only          不启动组件，只读取当前 ROS 状态
  -h, --help             显示帮助

首次使用：
  cp shfile/airy_px4.env.example shfile/airy_px4.env
  bash shfile/start_airy_px4.sh --diagnostic-only --test-seconds 30

完成安装外参标定、PX4 参数配置和拆桨方向测试后，才可在本机配置中设置
PUBLISH_VISION=1。Position 模式与解锁始终由遥控器完成。
仅做拆桨 Position 接受性验证时，优先使用 --publish-vision --test-seconds N，
并保持 ALLOW_APPROXIMATE_DIRECTION_PUBLISH=0。把该变量设为 1 会允许方向未确认时
持续发布，属于额外高风险豁免，且不会把方向测试状态伪装为已完成。

停止本项目相关进程：
  bash shfile/stop_airy_px4.sh
EOF
}

set -- "${ORIGINAL_ARGS[@]}"
while (($# > 0)); do
  case "$1" in
    --config)
      (($# >= 2)) || { printf '[ERROR] --config 缺少路径。\n' >&2; exit 2; }
      shift 2
      ;;
    --diagnostic-only) DIAGNOSTIC_ONLY=1; shift ;;
    --publish-vision) PUBLISH_VISION=1; shift ;;
    --test-seconds)
      (($# >= 2)) || { printf '[ERROR] --test-seconds 缺少秒数。\n' >&2; exit 2; }
      TEST_SECONDS="$2"
      shift 2
      ;;
    --dry-run) DRY_RUN=1; shift ;;
    --status-only) PRINT_STATUS_ONLY=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) printf '[ERROR] 未知选项：%s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -t 1 ]]; then
  readonly GREEN=$'\033[1;32m' YELLOW=$'\033[1;33m' RED=$'\033[1;31m' RESET=$'\033[0m'
else
  readonly GREEN="" YELLOW="" RED="" RESET=""
fi
info() { printf '%s[INFO]%s %s\n' "${GREEN}" "${RESET}" "$*"; }
warn() { printf '%s[WARN]%s %s\n' "${YELLOW}" "${RESET}" "$*" >&2; }
die() { printf '%s[ERROR]%s %s\n' "${RED}" "${RESET}" "$*" >&2; exit 1; }

validate_bool() {
  local name="$1" value="$2"
  [[ "${value}" == "0" || "${value}" == "1" ]] || die "${name} 只能是 0 或 1，当前为 ${value}。"
}
validate_number() {
  local name="$1" value="$2"
  [[ "${value}" =~ ^[0-9]+([.][0-9]+)?$ ]] || die "${name} 必须是非负数，当前为 ${value}。"
}
require_range() {
  local name="$1" value="$2" minimum="$3" maximum="$4"
  awk -v value="${value}" -v minimum="${minimum}" -v maximum="${maximum}" \
    'BEGIN {exit !(value+0 >= minimum+0 && value+0 <= maximum+0)}' || \
    die "${name} 必须在 [${minimum}, ${maximum}] 内，当前为 ${value}。"
}

for switch_name in START_ROSCORE START_MAVROS START_FASTLIO START_BRIDGE START_MONITOR \
  PUBLISH_VISION MOUNT_CONFIRMED DIRECTION_TEST_CONFIRMED \
  ALLOW_APPROXIMATE_DIRECTION_PUBLISH ALLOW_VISION_YAW_FUSION; do
  validate_bool "${switch_name}" "${!switch_name}"
done
for number_name in SENSOR_TO_BODY_Q_X SENSOR_TO_BODY_Q_Y SENSOR_TO_BODY_Q_Z \
  SENSOR_TO_BODY_Q_W SENSOR_POSITION_IN_BODY_X SENSOR_POSITION_IN_BODY_Y \
  SENSOR_POSITION_IN_BODY_Z ALIGN_STABLE_SECONDS MIN_FASTLIO_RATE_HZ \
  FLIGHT_MIN_POSE_RATE_HZ MIN_LOCAL_POSE_RATE_HZ MAX_SOURCE_AGE_S MAX_SOURCE_GAP_S \
  MAX_RECEIPT_STALL_S MAX_POSITION_JUMP_M \
  MAX_ORIENTATION_JUMP_DEG MAX_GRAVITY_MISMATCH_DEG; do
  # Transform components may be negative.
  if [[ "${number_name}" == SENSOR_* ]]; then
    [[ "${!number_name}" =~ ^-?[0-9]+([.][0-9]+)?([eE][-+]?[0-9]+)?$ ]] || \
      die "${number_name} 必须是有限数字，当前为 ${!number_name}。"
  else
    validate_number "${number_name}" "${!number_name}"
  fi
done
[[ "${UAV_NAME}" =~ ^[A-Za-z][A-Za-z0-9_]*$ ]] || die "UAV_NAME 不是合法 ROS 命名空间：${UAV_NAME}"
[[ "${MAV_SYS_ID}" =~ ^[1-9][0-9]*$ ]] || die "MAV_SYS_ID 必须为正整数。"
[[ "${MAV_COMP_ID}" =~ ^[1-9][0-9]*$ ]] || die "MAV_COMP_ID 必须为正整数。"
[[ "${EXPECTED_PX4_MAJOR}" =~ ^[0-9]+$ ]] || die "EXPECTED_PX4_MAJOR 必须为非负整数。"
[[ "${EXPECTED_PX4_MINOR}" =~ ^[0-9]+$ ]] || die "EXPECTED_PX4_MINOR 必须为非负整数。"
[[ "${TEST_SECONDS}" =~ ^[0-9]+$ ]] || die "TEST_SECONDS 必须为非负整数秒。"
require_range ALIGN_STABLE_SECONDS "${ALIGN_STABLE_SECONDS}" 5 60
require_range MIN_FASTLIO_RATE_HZ "${MIN_FASTLIO_RATE_HZ}" 8 100
require_range FLIGHT_MIN_POSE_RATE_HZ "${FLIGHT_MIN_POSE_RATE_HZ}" 30 100
require_range MIN_LOCAL_POSE_RATE_HZ "${MIN_LOCAL_POSE_RATE_HZ}" 1 20
require_range MAX_SOURCE_AGE_S "${MAX_SOURCE_AGE_S}" 0.05 0.5
require_range MAX_SOURCE_GAP_S "${MAX_SOURCE_GAP_S}" 0.3 1.0
require_range MAX_RECEIPT_STALL_S "${MAX_RECEIPT_STALL_S}" 0.5 2.0
require_range MAX_POSITION_JUMP_M "${MAX_POSITION_JUMP_M}" 0.05 5
require_range MAX_ORIENTATION_JUMP_DEG "${MAX_ORIENTATION_JUMP_DEG}" 1 90
require_range MAX_GRAVITY_MISMATCH_DEG "${MAX_GRAVITY_MISMATCH_DEG}" 1 15
if ((TEST_SECONDS > 0 && TEST_SECONDS < 10)); then
  die "--test-seconds 至少为 10 秒，0 表示持续运行。"
fi
if [[ "${SESSION_ROOT}" != /* ]]; then
  SESSION_ROOT="${PROJECT_ROOT}/${SESSION_ROOT#./}"
fi
command -v realpath >/dev/null 2>&1 || die "缺少命令：realpath"
SESSION_ROOT="$(realpath -m -- "${SESSION_ROOT}")"
[[ -r /proc/sys/kernel/random/boot_id ]] || die "无法读取系统 boot_id。"
BOOT_ID="$(tr -d '[:space:]' </proc/sys/kernel/random/boot_id)"
[[ "${BOOT_ID}" =~ ^[0-9a-fA-F-]{36}$ ]] || die "系统 boot_id 格式无效。"
readonly BOOT_ID

readonly SESSION_SCHEMA=2
readonly MAVROS_PREFIX="/${UAV_NAME}/mavros"
readonly STATE_TOPIC="${MAVROS_PREFIX}/state"
readonly VISION_TOPIC="${MAVROS_PREFIX}/vision_pose/pose"
readonly PARAM_GET_SERVICE="${MAVROS_PREFIX}/param/get"
readonly PARAM_PULL_SERVICE="${MAVROS_PREFIX}/param/pull"
readonly VEHICLE_INFO_SERVICE="${MAVROS_PREFIX}/vehicle_info_get"
readonly SESSION_STAMP="$(date +%Y%m%d_%H%M%S)"
readonly SESSION_DIR="${SESSION_ROOT}/${SESSION_STAMP}_${UAV_NAME}_$$"
readonly LOG_DIR="${SESSION_DIR}/logs"
readonly SESSION_META="${SESSION_DIR}/session.meta"
readonly PROCESS_MANIFEST="${SESSION_DIR}/processes.tsv"
readonly ACTIVE_SESSION_POINTER="${SESSION_ROOT}/active_${UAV_NAME}.session"
readonly SESSION_LOCK_ROOT="${PROJECT_ROOT}/runtime/airy_px4_locks"
readonly SESSION_LOCK_FILE="${SESSION_LOCK_ROOT}/launcher_uid${UID}_${UAV_NAME}.lock"

OWNED_PIDS=()
OWNED_PGIDS=()
OWNED_TICKS=()
OWNED_NAMES=()
CLEANUP_STARTED=0
COMPONENT_START_IN_PROGRESS=0
PENDING_PID=""
PENDING_NAME=""
POINTER_SAFE_TO_CLEAR=1

process_start_ticks() {
  local pid="$1" stat_line stat_tail
  [[ -r "/proc/${pid}/stat" ]] || return 1
  IFS= read -r stat_line <"/proc/${pid}/stat" || return 1
  stat_tail="${stat_line##*) }"
  awk '{print $20}' <<<"${stat_tail}"
}

process_pgid() {
  local pid="$1" pgid
  pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"
  [[ "${pgid}" =~ ^[1-9][0-9]*$ ]] || return 1
  printf '%s\n' "${pgid}"
}

process_group_has_members() {
  local pgid="$1"
  [[ "${pgid}" =~ ^[1-9][0-9]*$ ]] || return 1
  ps -eo pgid=,uid=,stat= | awk -v wanted_pgid="${pgid}" -v wanted_uid="${UID}" '
    $1 == wanted_pgid && $2 == wanted_uid && $3 !~ /^Z/ {found=1}
    END {exit(found ? 0 : 1)}'
}

same_process() {
  local pid="$1" expected_ticks="$2" actual_ticks owner_uid
  [[ "${pid}" =~ ^[1-9][0-9]*$ && "${expected_ticks}" =~ ^[0-9]+$ ]] || return 1
  [[ -d "/proc/${pid}" ]] || return 1
  owner_uid="$(stat -c '%u' "/proc/${pid}" 2>/dev/null)" || return 1
  [[ "${owner_uid}" == "${UID}" ]] || return 1
  actual_ticks="$(process_start_ticks "${pid}")" || return 1
  [[ "${actual_ticks}" == "${expected_ticks}" ]]
}

owned_process_matches() {
  local index="$1" pid expected_pgid expected_ticks actual_pgid
  pid="${OWNED_PIDS[index]}"
  expected_pgid="${OWNED_PGIDS[index]}"
  expected_ticks="${OWNED_TICKS[index]}"
  same_process "${pid}" "${expected_ticks}" || return 1
  actual_pgid="$(process_pgid "${pid}")" || return 1
  [[ "${actual_pgid}" == "${expected_pgid}" && "${expected_pgid}" == "${pid}" ]]
}

meta_value() {
  local file="$1" key="$2"
  awk -F= -v wanted="${key}" '$1 == wanted {sub(/^[^=]*=/, ""); print; exit}' "${file}"
}

ensure_no_live_recorded_session() {
  local pointer_session meta_file manifest_file meta_schema meta_project meta_uav meta_boot_id
  local launcher_pid launcher_ticks name pid pgid start_ticks record_boot_id extra last_byte
  [[ -f "${ACTIVE_SESSION_POINTER}" ]] || return 0
  pointer_session=""
  IFS= read -r pointer_session <"${ACTIVE_SESSION_POINTER}" || true
  [[ -n "${pointer_session}" ]] || \
    die "活动会话指针为空；请先运行 stop_airy_px4.sh 做严格兜底清理。"
  case "${pointer_session}" in
    "${SESSION_ROOT}"/*) ;;
    *) die "活动会话指针超出 SESSION_ROOT，拒绝覆盖：${pointer_session}" ;;
  esac
  meta_file="${pointer_session}/session.meta"
  manifest_file="${pointer_session}/processes.tsv"
  [[ -f "${meta_file}" && -f "${manifest_file}" ]] || \
    die "活动会话记录不完整；请先运行 stop_airy_px4.sh 清理：${pointer_session}"
  meta_schema="$(meta_value "${meta_file}" schema)"
  meta_project="$(meta_value "${meta_file}" project_root)"
  meta_uav="$(meta_value "${meta_file}" uav_name)"
  meta_boot_id="$(meta_value "${meta_file}" boot_id)"
  [[ "${meta_project}" == "${PROJECT_ROOT}" && "${meta_uav}" == "${UAV_NAME}" ]] || \
    die "活动会话身份与本项目不一致，拒绝覆盖：${pointer_session}"
  [[ "${meta_schema}" == "${SESSION_SCHEMA}" ]] || \
    die "旧会话元数据版本无效；请先运行 stop_airy_px4.sh 做严格兜底清理。"
  [[ "${meta_boot_id}" == "${BOOT_ID}" ]] || \
    die "旧会话来自另一系统启动批次；请先运行 stop_airy_px4.sh 清除断电前记录。"
  launcher_pid="$(meta_value "${meta_file}" launcher_pid)"
  launcher_ticks="$(meta_value "${meta_file}" launcher_start_ticks)"
  if same_process "${launcher_pid}" "${launcher_ticks}"; then
    die "${UAV_NAME} 的综合启动器仍在运行（PID ${launcher_pid}）；请勿重复启动。"
  fi
  [[ -s "${manifest_file}" ]] || \
    die "旧会话清单为空且启动器已退出；请先运行 stop_airy_px4.sh 做严格兜底清理。"
  last_byte="$(tail -c 1 -- "${manifest_file}" | od -An -t u1 | tr -d '[:space:]')"
  [[ "${last_byte}" == "10" ]] || \
    die "旧会话清单存在未完整写入的记录；请先运行 stop_airy_px4.sh。"
  while IFS=$'\t' read -r name pid pgid start_ticks record_boot_id extra; do
    [[ -z "${extra:-}" && -n "${name}" && "${pid}" =~ ^[1-9][0-9]*$ && \
      "${pgid}" =~ ^[1-9][0-9]*$ && "${start_ticks}" =~ ^[0-9]+$ && \
      "${record_boot_id}" == "${BOOT_ID}" ]] || \
      die "旧会话清单格式无效；请先运行 stop_airy_px4.sh。"
    if same_process "${pid}" "${start_ticks}" && \
       [[ "$(process_pgid "${pid}" 2>/dev/null || true)" == "${pgid}" ]]; then
      die "上次会话仍有组件 ${name}（PID ${pid}）存活；请先运行 stop_airy_px4.sh。"
    fi
    if process_group_has_members "${pgid}"; then
      die "上次会话组件 ${name} 的进程组 ${pgid} 仍有成员；请先运行 stop_airy_px4.sh。"
    fi
  done <"${manifest_file}"
  die "检测到已退出但尚未由停止脚本确认清理的旧会话；请先运行 stop_airy_px4.sh：${pointer_session}"
}

manifest_record_for_pid() {
  local wanted_pid="$1" record_name record_pid record_pgid record_ticks record_boot_id extra
  MANIFEST_RECORD_NAME=""
  MANIFEST_RECORD_PGID=""
  MANIFEST_RECORD_TICKS=""
  [[ -f "${PROCESS_MANIFEST}" ]] || return 1
  while IFS=$'\t' read -r record_name record_pid record_pgid record_ticks record_boot_id extra; do
    [[ "${record_pid}" == "${wanted_pid}" ]] || continue
    [[ -z "${extra:-}" && -n "${record_name}" && "${record_pgid}" =~ ^[1-9][0-9]*$ && \
      "${record_ticks}" =~ ^[0-9]+$ && "${record_boot_id}" == "${BOOT_ID}" ]] || return 1
    MANIFEST_RECORD_NAME="${record_name}"
    MANIFEST_RECORD_PGID="${record_pgid}"
    MANIFEST_RECORD_TICKS="${record_ticks}"
    return 0
  done <"${PROCESS_MANIFEST}"
  return 1
}

stop_pending_process() {
  local pid="${PENDING_PID}" attempt start_ticks="" pgid="" current_pgid=""
  local observed_ticks="" observed_pgid="" manifest_identity_found=0
  if [[ ! "${pid}" =~ ^[1-9][0-9]*$ ]]; then
    if [[ "${COMPONENT_START_IN_PROGRESS}" == "1" ]]; then
      POINTER_SAFE_TO_CLEAR=0
      warn "组件启动刚进入 fork 窗口但尚未取得 PID；保留活动会话指针供停止脚本恢复。"
    fi
    return 0
  fi
  if manifest_record_for_pid "${pid}"; then
    start_ticks="${MANIFEST_RECORD_TICKS}"
    pgid="${MANIFEST_RECORD_PGID}"
    manifest_identity_found=1
  fi
  for ((attempt = 0; attempt < 20; attempt++)); do
    observed_ticks="$(process_start_ticks "${pid}" 2>/dev/null || true)"
    observed_pgid="$(process_pgid "${pid}" 2>/dev/null || true)"
    if [[ "${observed_ticks}" =~ ^[0-9]+$ && "${observed_pgid}" =~ ^[1-9][0-9]*$ ]]; then
      start_ticks="${observed_ticks}"
      pgid="${observed_pgid}"
      break
    fi
    kill -0 "${pid}" 2>/dev/null || break
    sleep 0.01
  done
  if [[ "${manifest_identity_found}" == "0" ]] && manifest_record_for_pid "${pid}"; then
    [[ "${start_ticks}" =~ ^[0-9]+$ ]] || start_ticks="${MANIFEST_RECORD_TICKS}"
    [[ "${pgid}" =~ ^[1-9][0-9]*$ ]] || pgid="${MANIFEST_RECORD_PGID}"
    manifest_identity_found=1
  fi
  if [[ "${start_ticks}" =~ ^[0-9]+$ ]] && same_process "${pid}" "${start_ticks}"; then
    if [[ "${pgid}" == "${pid}" ]]; then
      kill -TERM -- "-${pgid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
    else
      kill -TERM "${pid}" 2>/dev/null || true
    fi
    for ((attempt = 0; attempt < 30; attempt++)); do
      same_process "${pid}" "${start_ticks}" || break
      sleep 0.1
    done
    if same_process "${pid}" "${start_ticks}"; then
      current_pgid="$(process_pgid "${pid}" 2>/dev/null || true)"
      if [[ "${current_pgid}" == "${pid}" ]]; then
        kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
      else
        kill -KILL "${pid}" 2>/dev/null || true
      fi
    fi
  elif kill -0 "${pid}" 2>/dev/null; then
    # An unreaped direct child PID cannot be reused.  Kill the child itself,
    # but keep the pointer because its process-group identity was unavailable.
    kill -TERM "${pid}" 2>/dev/null || true
    sleep 0.1
    kill -KILL "${pid}" 2>/dev/null || true
    POINTER_SAFE_TO_CLEAR=0
    warn "无法确认待启动组件 ${PENDING_NAME:-unknown} 的进程组；已停止直接子进程并保留会话指针。"
  fi
  wait "${pid}" 2>/dev/null || true
  if [[ "${pgid}" =~ ^[1-9][0-9]*$ ]] && process_group_has_members "${pgid}"; then
    POINTER_SAFE_TO_CLEAR=0
    warn "待启动组件 ${PENDING_NAME:-unknown} 的进程组 ${pgid} 仍有成员；保留活动会话指针。"
  elif [[ ! "${pgid}" =~ ^[1-9][0-9]*$ ]]; then
    POINTER_SAFE_TO_CLEAR=0
    warn "无法证明待启动组件 ${PENDING_NAME:-unknown} 的进程组已经清空；保留活动会话指针。"
  fi
  PENDING_PID=""
  PENDING_NAME=""
  COMPONENT_START_IN_PROGRESS=0
}

clear_active_session_pointer() {
  local active_session=""
  [[ -f "${ACTIVE_SESSION_POINTER}" ]] || return 0
  IFS= read -r active_session <"${ACTIVE_SESSION_POINTER}" || true
  if [[ "${active_session}" == "${SESSION_DIR}" ]]; then
    rm -f -- "${ACTIVE_SESSION_POINTER}"
  fi
}

cleanup() {
  local index pid wait_step
  trap '' INT TERM
  trap - EXIT
  if [[ "${CLEANUP_STARTED}" == "1" ]]; then
    return
  fi
  CLEANUP_STARTED=1
  stop_pending_process
  if ((${#OWNED_PIDS[@]} == 0)); then
    [[ "${POINTER_SAFE_TO_CLEAR}" != "1" ]] || clear_active_session_pointer
    return
  fi
  info "停止本脚本启动的进程（不会触碰已有 ROS 进程）..."
  for ((index = ${#OWNED_PIDS[@]} - 1; index >= 0; index--)); do
    pid="${OWNED_PIDS[index]}"
    if owned_process_matches "${index}"; then
      kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
  for ((wait_step = 0; wait_step < 30; wait_step++)); do
    local any_alive=0
    for ((index = 0; index < ${#OWNED_PIDS[@]}; index++)); do
      owned_process_matches "${index}" && any_alive=1
    done
    [[ "${any_alive}" == "0" ]] && break
    sleep 0.1
  done
  for ((index = 0; index < ${#OWNED_PIDS[@]}; index++)); do
    pid="${OWNED_PIDS[index]}"
    if owned_process_matches "${index}"; then
      kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
    fi
    wait "${pid}" 2>/dev/null || true
  done
  for ((index = 0; index < ${#OWNED_PGIDS[@]}; index++)); do
    if process_group_has_members "${OWNED_PGIDS[index]}"; then
      POINTER_SAFE_TO_CLEAR=0
      warn "${OWNED_NAMES[index]:-unknown} 的进程组 ${OWNED_PGIDS[index]} 仍有成员；保留活动会话指针供停止脚本恢复。"
    fi
  done
  printf 'stopped_epoch=%s\n' "$(date +%s)" >>"${SESSION_META}" 2>/dev/null || true
  [[ "${POINTER_SAFE_TO_CLEAR}" != "1" ]] || clear_active_session_pointer
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

source_environment() {
  [[ -f "${AIRY_SETUP}" ]] || die "缺少 ${AIRY_SETUP}，请先运行一键安装脚本。"
  export ROS_MASTER_URI
  unset ROS_IP ROS_HOSTNAME
  export AIRY_KEEP_ROS_NETWORK=1
  # shellcheck disable=SC1090
  source "${AIRY_SETUP}" >/dev/null
  unset AIRY_KEEP_ROS_NETWORK
  export ROS_MASTER_URI
  unset ROS_IP ROS_HOSTNAME
}

start_component() {
  local name="$1" log_file="$2" pid pgid start_ticks record_attempt record_found=0
  shift 2
  info "启动 ${name}；日志：${log_file}"
  {
    printf 'COMMAND:'
    printf ' %q' "$@"
    printf '\n'
  } >>"${log_file}"
  # The new session writes its identity before exec.  If the launcher is ever
  # SIGKILLed in this narrow window, the surviving component is still recorded.
  COMPONENT_START_IN_PROGRESS=1
  setsid bash -c '
    set -Eeuo pipefail
    manifest=$1
    component_name=$2
    component_boot_id=$3
    shift 3
    child_pid=$$
    child_pgid=$(ps -o pgid= -p "${child_pid}" | tr -d "[:space:]")
    IFS= read -r stat_line <"/proc/${child_pid}/stat"
    stat_tail=${stat_line##*) }
    child_ticks=$(awk "{print \$20}" <<<"${stat_tail}")
    [[ "${child_pgid}" == "${child_pid}" && "${child_ticks}" =~ ^[0-9]+$ ]]
    printf "%s\t%s\t%s\t%s\t%s\n" \
      "${component_name}" "${child_pid}" "${child_pgid}" "${child_ticks}" \
      "${component_boot_id}" >>"${manifest}"
    exec "$@"
  ' airy-session-child "${PROCESS_MANIFEST}" "${name}" "${BOOT_ID}" "$@" >>"${log_file}" 2>&1 &
  pid=$!
  PENDING_PID="${pid}"
  PENDING_NAME="${name}"
  for ((record_attempt = 0; record_attempt < 100; record_attempt++)); do
    if manifest_record_for_pid "${pid}"; then
      record_found=1
      break
    fi
    kill -0 "${pid}" 2>/dev/null || break
    sleep 0.01
  done
  if [[ "${record_found}" != "1" ]]; then
    tail -n 30 "${log_file}" >&2 || true
    die "${name} 未能在启动时限内写入会话身份记录。"
  fi
  pgid="${MANIFEST_RECORD_PGID}"
  start_ticks="${MANIFEST_RECORD_TICKS}"
  if [[ "${MANIFEST_RECORD_NAME}" != "${name}" || "${pgid}" != "${pid}" ]] || \
     ! same_process "${pid}" "${start_ticks}"; then
    die "${name} 的会话身份或独立进程组校验失败，拒绝继续启动。"
  fi
  OWNED_PGIDS+=("${pgid}")
  OWNED_TICKS+=("${start_ticks}")
  OWNED_NAMES+=("${name}")
  # PID is the commit marker: cleanup only iterates records after all parallel
  # identity arrays have been populated.
  OWNED_PIDS+=("${pid}")
  COMPONENT_START_IN_PROGRESS=0
  PENDING_PID=""
  PENDING_NAME=""
  sleep 0.5
  if ! owned_process_matches "$((${#OWNED_PIDS[@]} - 1))"; then
    tail -n 30 "${log_file}" >&2 || true
    die "${name} 启动后立即退出。"
  fi
}

check_owned_processes() {
  local index
  for ((index = 0; index < ${#OWNED_PIDS[@]}; index++)); do
    if ! owned_process_matches "${index}"; then
      warn "${OWNED_NAMES[index]} 已异常退出，最后日志如下："
      tail -n 30 "${LOG_DIR}/${OWNED_NAMES[index]}.log" >&2 || true
      return 1
    fi
  done
}

master_online() { rostopic list >/dev/null 2>&1; }
wait_for_master() {
  local elapsed=0
  until master_online; do
    ((elapsed >= 20)) && return 1
    sleep 1
    elapsed=$((elapsed + 1))
  done
}
wait_for_service() {
  local service="$1" timeout_seconds="$2" elapsed=0
  until rosservice info "${service}" >/dev/null 2>&1; do
    ((elapsed >= timeout_seconds)) && return 1
    sleep 1
    elapsed=$((elapsed + 1))
  done
}
wait_for_topic_message() {
  local topic="$1" timeout_seconds="$2" elapsed=0
  until timeout 2s rostopic echo -n 1 "${topic}" >/dev/null 2>&1; do
    ((elapsed >= timeout_seconds)) && return 1
    sleep 1
    elapsed=$((elapsed + 1))
  done
}

normalise_ros_string() {
  local value="$1"
  case "${value}" in
    "''"|'""'|null|'~') printf '' ;;
    \'*\') value="${value#\'}"; printf '%s' "${value%\'}" ;;
    \"*\") value="${value#\"}"; printf '%s' "${value%\"}" ;;
    *) printf '%s' "${value}" ;;
  esac
}

verify_mavros_gcs_url() {
  local raw actual
  raw="$(rosparam get "${MAVROS_PREFIX}/gcs_url" 2>/dev/null)" || return 1
  actual="$(normalise_ros_string "${raw}")"
  if [[ -z "${GCS_URL}" ]]; then
    [[ -z "${actual}" ]] || {
      warn "MAVROS 意外启用了 GCS 转发：${actual}。"
      return 1
    }
    info "MAVROS GCS 转发已明确禁用。"
  else
    [[ "${actual}" == "${GCS_URL}" ]] || {
      warn "MAVROS GCS URL 与请求不符：期望 ${GCS_URL}，实际 ${actual:-<empty>}。"
      return 1
    }
    info "MAVROS GCS 转发地址核验通过：${actual}。"
  fi
}

state_snapshot() {
  timeout 3s rostopic echo -n 1 "${STATE_TOPIC}" 2>/dev/null || true
}
wait_for_fcu_disarmed() {
  local timeout_seconds="$1" elapsed=0 snapshot connected armed
  while ((elapsed < timeout_seconds)); do
    snapshot="$(state_snapshot)"
    connected="$(awk '/^connected:/ {print $2; exit}' <<<"${snapshot}")"
    armed="$(awk '/^armed:/ {print $2; exit}' <<<"${snapshot}")"
    if [[ "${connected}" == "True" && "${armed}" == "False" ]]; then
      return 0
    fi
    if [[ "${armed}" == "True" ]]; then
      die "检测到飞控已经解锁；本脚本只允许在未解锁状态初始化。"
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 1
}

node_exists() {
  local nodes
  nodes="$(rosnode list 2>/dev/null)" || return 1
  grep -Fxq "$1" <<<"${nodes}"
}
publisher_count() {
  local topic="$1" topic_info
  if ! topic_info="$(rostopic info "${topic}" 2>/dev/null)"; then
    printf '0\n'
    return 0
  fi
  awk '
    /^Publishers:/ {in_publishers=1; next}
    /^Subscribers:/ {in_publishers=0}
    in_publishers && /^[[:space:]]*\*/ {count++}
    END {print count+0}' <<<"${topic_info}"
}

topic_has_subscriber() {
  local topic="$1" expected_node="$2" topic_info
  topic_info="$(rostopic info "${topic}" 2>/dev/null)" || return 1
  awk -v expected="${expected_node}" '
    /^Subscribers:/ {in_subscribers=1; next}
    in_subscribers && /^[[:space:]]*\*/ {
      node=$2
      if (node == expected) found=1
    }
    END {exit(found ? 0 : 1)}' <<<"${topic_info}"
}

get_px4_param() {
  local param_name="$1" response
  response="$(timeout 8s rosservice call "${PARAM_GET_SERVICE}" "param_id: '${param_name}'" 2>/dev/null)" || return 1
  grep -Eq '^success: (True|true)$' <<<"${response}" || return 1
  PX4_PARAM_INTEGER="$(awk '/^[[:space:]]*integer:/ {print $2; exit}' <<<"${response}")"
  PX4_PARAM_REAL="$(awk '/^[[:space:]]*real:/ {print $2; exit}' <<<"${response}")"
  [[ -n "${PX4_PARAM_INTEGER}" && -n "${PX4_PARAM_REAL}" ]]
}

get_vehicle_info() {
  local response version available_info
  response="$(timeout 8s rosservice call "${VEHICLE_INFO_SERVICE}" \
    "sysid: ${MAV_SYS_ID}
compid: ${MAV_COMP_ID}
get_all: false" 2>/dev/null)" || return 1
  grep -Eq '^success: (True|true)$' <<<"${response}" || return 1
  FCU_AUTOPILOT="$(awk '/^[[:space:]]*autopilot:/ {print $2; exit}' <<<"${response}")"
  available_info="$(awk '/^[[:space:]]*available_info:/ {print $2; exit}' <<<"${response}")"
  version="$(awk '/^[[:space:]]*flight_sw_version:/ {print $2; exit}' <<<"${response}")"
  [[ "${FCU_AUTOPILOT}" =~ ^[0-9]+$ && "${available_info}" =~ ^[0-9]+$ \
    && "${version}" =~ ^[0-9]+$ ]] || return 1
  (( (available_info & 2) != 0 )) || return 1
  FCU_VERSION_MAJOR=$(( (version >> 24) & 255 ))
  FCU_VERSION_MINOR=$(( (version >> 16) & 255 ))
  FCU_VERSION_PATCH=$(( (version >> 8) & 255 ))
}

wait_for_vehicle_info() {
  local elapsed=0 timeout_seconds="$1"
  while ((elapsed < timeout_seconds)); do
    get_vehicle_info && return 0
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 1
}

pull_px4_params() {
  local response received
  response="$(timeout 40s rosservice call "${PARAM_PULL_SERVICE}" "force_pull: true" 2>/dev/null)" || return 1
  grep -Eq '^success: (True|true)$' <<<"${response}" || return 1
  received="$(awk '/^param_received:/ {print $2; exit}' <<<"${response}")"
  [[ "${received}" =~ ^[1-9][0-9]*$ ]] || return 1
  PX4_PARAM_COUNT="${received}"
}

print_status() {
  local snapshot bridge_status monitor_status
  snapshot="$(state_snapshot)"
  printf '%s\n' '--- MAVROS state ---'
  [[ -n "${snapshot}" ]] && printf '%s\n' "${snapshot}" || printf 'no message\n'
  printf '%s\n' '--- bridge diagnostics ---'
  bridge_status="$(timeout 3s rostopic echo -n 1 /airy_px4/bridge/diagnostics 2>/dev/null || true)"
  [[ -n "${bridge_status}" ]] && printf '%s\n' "${bridge_status}" || printf 'no message\n'
  printf '%s\n' '--- readiness diagnostics ---'
  monitor_status="$(timeout 3s rostopic echo -n 1 /airy_px4/monitor/diagnostics 2>/dev/null || true)"
  [[ -n "${monitor_status}" ]] && printf '%s\n' "${monitor_status}" || printf 'no message\n'
}

diagnostic_value() {
  local message="$1" wanted_key="$2"
  awk -v wanted="${wanted_key}" '
    $1 == "key:" {
      key=$2
      gsub(/"/, "", key)
      if (key == wanted) {found=1; next}
    }
    found && /value:/ {
      sub(/^[[:space:]]*value:[[:space:]]*/, "")
      gsub(/^"|"$/, "")
      print
      exit
    }' <<<"${message}"
}

assert_diagnostic_test() {
  local failures=0 snapshot bridge_status monitor_status
  local connected armed bridge_message input_rate input_age source_stamp_age output_rate last_rejection
  local monitor_message timesync_ok solution_flags_ok local_pose_continuous
  local vision_fresh bridge_ok bridge_state bridge_gate_blocked recent_px4_fault
  local ev_fusion_verified position_mode_acceptance_verified
  local position_mode_acceptance_unverified position_data_ready
  local blocking_reasons required_reason
  snapshot="$(state_snapshot)"
  connected="$(awk '/^connected:/ {print $2; exit}' <<<"${snapshot}")"
  armed="$(awk '/^armed:/ {print $2; exit}' <<<"${snapshot}")"
  [[ "${connected}" == "True" ]] || { warn "验收失败：FCU 未连接。"; failures=$((failures + 1)); }
  [[ "${armed}" == "False" ]] || { warn "验收失败：FCU 不是未解锁状态。"; failures=$((failures + 1)); }

  bridge_status="$(timeout 3s rostopic echo -n 1 /airy_px4/bridge/diagnostics 2>/dev/null || true)"
  bridge_message="$(awk '/^[[:space:]]*message:/ {gsub(/"/, "", $2); print $2; exit}' <<<"${bridge_status}")"
  input_rate="$(diagnostic_value "${bridge_status}" input_rate_hz)"
  input_age="$(diagnostic_value "${bridge_status}" input_age_s)"
  source_stamp_age="$(diagnostic_value "${bridge_status}" source_stamp_age_s)"
  output_rate="$(diagnostic_value "${bridge_status}" output_rate_hz)"
  last_rejection="$(diagnostic_value "${bridge_status}" last_rejection)"
  if [[ "${PUBLISH_EFFECTIVE}" == "1" ]]; then
    [[ "${bridge_message}" == "PUBLISHING_NATIVE_RATE" ]] || {
      warn "验收失败：发布台架模式桥状态异常：${bridge_message:-missing}。"
      failures=$((failures + 1))
    }
  else
    [[ "${bridge_message}" == "BLOCKED_BY_STARTUP_GATE" ]] || {
      warn "验收失败：诊断模式桥状态异常：${bridge_message:-missing}。"
      failures=$((failures + 1))
    }
  fi
  [[ "${input_rate}" =~ ^[0-9]+([.][0-9]+)?$ ]] && \
    awk -v value="${input_rate}" -v minimum="${MIN_FASTLIO_RATE_HZ}" \
      'BEGIN {exit !(value+0 >= minimum+0)}' || {
      warn "验收失败：FAST-LIO 合法位姿频率 ${input_rate:-missing} Hz。"
      failures=$((failures + 1))
    }
  [[ "${input_age}" =~ ^[0-9]+([.][0-9]+)?$ ]] && \
    awk -v value="${input_age}" -v maximum="${MAX_SOURCE_AGE_S}" \
      'BEGIN {exit !(value+0 >= 0 && value+0 <= maximum+0)}' || {
      warn "验收失败：FAST-LIO 位姿时间新鲜度 ${input_age:-missing} s。"
      failures=$((failures + 1))
    }
  [[ "${source_stamp_age}" =~ ^-?[0-9]+([.][0-9]+)?$ ]] && \
    awk -v value="${source_stamp_age}" -v maximum="${MAX_SOURCE_AGE_S}" \
      'BEGIN {exit !(value+0 >= -0.2 && value+0 <= maximum+0)}' || {
      warn "验收失败：FAST-LIO header.stamp 延迟 ${source_stamp_age:-missing} s。"
      failures=$((failures + 1))
    }
  if [[ "${PUBLISH_EFFECTIVE}" == "1" ]]; then
    [[ "${output_rate}" =~ ^[0-9]+([.][0-9]+)?$ ]] && \
      awk -v value="${output_rate}" -v minimum="${MIN_FASTLIO_RATE_HZ}" \
        'BEGIN {exit !(value+0 >= minimum+0)}' || {
          warn "验收失败：发布台架模式视觉输出仅 ${output_rate:-missing} Hz。"
          failures=$((failures + 1))
        }
  else
    [[ "${output_rate}" =~ ^[0-9]+([.][0-9]+)?$ ]] && \
      awk -v value="${output_rate}" 'BEGIN {exit !(value+0 == 0)}' || {
        warn "验收失败：安全门控阻止发布时仍出现视觉输出。"
        failures=$((failures + 1))
      }
  fi
  if [[ "${last_rejection}" == *timestamp* || "${last_rejection}" == *non-finite* ]]; then
    warn "验收失败：桥接输入校验错误：${last_rejection}。"
    failures=$((failures + 1))
  fi

  monitor_status="$(timeout 3s rostopic echo -n 1 /airy_px4/monitor/diagnostics 2>/dev/null || true)"
  monitor_message="$(awk '/^[[:space:]]*message:/ {gsub(/"/, "", $2); print $2; exit}' <<<"${monitor_status}")"
  timesync_ok="$(diagnostic_value "${monitor_status}" timesync_ok)"
  solution_flags_ok="$(diagnostic_value "${monitor_status}" px4_solution_flags_ok)"
  local_pose_continuous="$(diagnostic_value "${monitor_status}" px4_local_pose_continuous)"
  vision_fresh="$(diagnostic_value "${monitor_status}" vision_fresh)"
  bridge_ok="$(diagnostic_value "${monitor_status}" bridge_ok)"
  bridge_state="$(diagnostic_value "${monitor_status}" bridge_state)"
  bridge_gate_blocked="$(diagnostic_value "${monitor_status}" bridge_gate_blocked)"
  recent_px4_fault="$(diagnostic_value "${monitor_status}" recent_px4_fault)"
  ev_fusion_verified="$(diagnostic_value "${monitor_status}" ev_fusion_verified)"
  position_mode_acceptance_verified="$(diagnostic_value "${monitor_status}" position_mode_acceptance_verified)"
  position_mode_acceptance_unverified="$(diagnostic_value "${monitor_status}" position_mode_acceptance_unverified)"
  position_data_ready="$(diagnostic_value "${monitor_status}" position_data_ready)"
  blocking_reasons="$(diagnostic_value "${monitor_status}" blocking_reasons)"

  [[ -n "${monitor_message}" ]] || {
    warn "验收失败：没有读到监控总体状态。"
    failures=$((failures + 1))
  }
  [[ "${position_data_ready}" == "False" ]] || {
    warn "验收失败：监控在缺少直接融合/模式接受证据时错误声明 Position 数据就绪。"
    failures=$((failures + 1))
  }
  [[ "${ev_fusion_verified}" == "False" ]] || {
    warn "验收失败：当前监控不具备外部视觉融合证明，却返回 ev_fusion_verified=${ev_fusion_verified:-missing}。"
    failures=$((failures + 1))
  }
  case "${position_mode_acceptance_verified}/${position_mode_acceptance_unverified}" in
    True/False|False/True) ;;
    *)
      warn "验收失败：Position 接受状态自相矛盾：verified=${position_mode_acceptance_verified:-missing} unverified=${position_mode_acceptance_unverified:-missing}。"
      failures=$((failures + 1))
      ;;
  esac
  for required_reason in EV_FUSION_UNVERIFIED; do
    case ",${blocking_reasons}," in
      *",${required_reason},"*) ;;
      *)
        warn "验收失败：监控 blocking_reasons 缺少 ${required_reason}。"
        failures=$((failures + 1))
        ;;
    esac
  done
  if [[ "${position_mode_acceptance_verified}" != "True" ]]; then
    case ",${blocking_reasons}," in
      *",POSITION_MODE_ACCEPTANCE_UNVERIFIED,"*) ;;
      *)
        warn "验收失败：尚未观察到 POSCTL 时，监控没有保留 Position 接受未验证原因。"
        failures=$((failures + 1))
        ;;
    esac
  fi
  [[ "${recent_px4_fault}" == "none" ]] || {
    warn "验收失败：PX4 最近报告故障：${recent_px4_fault:-missing}。"
    failures=$((failures + 1))
  }

  if [[ "${PUBLISH_EFFECTIVE}" == "1" ]]; then
    [[ "${solution_flags_ok}" == "True" ]] || {
      warn "验收失败：PX4 的通用位置/速度 solution flags 尚未满足保守台架判据；这不等价于视觉融合状态。"
      failures=$((failures + 1))
    }
    [[ "${local_pose_continuous}" == "True" ]] || {
      warn "验收失败：PX4 local_position/pose 未达到持续频率和最小样本门。"
      failures=$((failures + 1))
    }
    [[ "${vision_fresh}" == "True" && "${bridge_ok}" == "True" ]] || {
      warn "验收失败：视觉流或安全桥未处于健康发布状态。"
      failures=$((failures + 1))
    }
  else
    [[ "${bridge_state}" == "BLOCKED_BY_STARTUP_GATE" && \
       "${bridge_gate_blocked}" == "True" ]] || {
      warn "验收失败：监控未明确识别诊断安全门控：state=${bridge_state:-missing} gate=${bridge_gate_blocked:-missing}。"
      failures=$((failures + 1))
    }
    case ",${blocking_reasons}," in
      *",DIAGNOSTIC_GATE_BLOCKED,"*) ;;
      *)
        warn "验收失败：监控没有把诊断门控列入 blocking_reasons。"
        failures=$((failures + 1))
        ;;
    esac
  fi
  [[ "${timesync_ok}" == "True" ]] || {
    warn "验收失败：MAVROS/PX4 时间同步不健康。"
    failures=$((failures + 1))
  }
  [[ "${PX4_AID_MASK}" != "unknown" ]] || {
    warn "验收失败：未能读取 EKF2_AID_MASK。"
    failures=$((failures + 1))
  }
  [[ "${PX4_HGT_MODE}" != "unknown" ]] || {
    warn "验收失败：未能读取 EKF2_HGT_MODE。"
    failures=$((failures + 1))
  }
  ((failures == 0))
}

if [[ "${DRY_RUN}" != "1" && "${PRINT_STATUS_ONLY}" != "1" && \
      "${AIRY_PX4_INTERNAL_LOCK_HELD:-0}" != "1" ]]; then
  command -v flock >/dev/null 2>&1 || die "缺少命令：flock"
  mkdir -p -- "${SESSION_ROOT}" "${SESSION_LOCK_ROOT}"
  lock_result=0
  flock -n -o -E 73 "${SESSION_LOCK_FILE}" \
    env AIRY_PX4_INTERNAL_LOCK_HELD=1 \
    bash "${SCRIPT_DIR}/start_airy_px4.sh" "${ORIGINAL_ARGS[@]}" || lock_result=$?
  if [[ "${lock_result}" == "73" ]]; then
    die "${UAV_NAME} 的综合启动器已在运行；拒绝并发启动。"
  fi
  exit "${lock_result}"
fi

if [[ "${CONFIG_FILE_LOADED}" != "1" && "${DRY_RUN}" != "1" && \
      "${PRINT_STATUS_ONLY}" != "1" ]]; then
  warn "未找到本机配置 ${CONFIG_FILE}；正在使用环境变量/脚本安全默认值。"
  warn "当前 PUBLISH_VISION=${PUBLISH_VISION}、MOUNT_CONFIRMED=${MOUNT_CONFIRMED}；安全默认均为 0，任一未确认都会阻止视觉发布。"
fi

source_environment

if [[ -r /proc/sys/net/core/rmem_max ]]; then
  read -r UDP_RMEM_MAX </proc/sys/net/core/rmem_max || UDP_RMEM_MAX="unknown"
  [[ "${UDP_RMEM_MAX}" =~ ^[0-9]+$ ]] || UDP_RMEM_MAX="unknown"
fi

if [[ "${DRY_RUN}" == "1" ]]; then
  cat <<EOF
[DRY-RUN] 配置文件：${CONFIG_FILE} $([[ -f "${CONFIG_FILE}" ]] && printf '(已加载)' || printf '(不存在，使用安全默认值)')
[DRY-RUN] ROS master：${ROS_MASTER_URI}
[DRY-RUN] MAVROS：namespace=/${UAV_NAME}, fcu_url=${FCU_URL}, gcs_url=${GCS_URL:-<disabled>}
[DRY-RUN] 组件：roscore=${START_ROSCORE} mavros=${START_MAVROS} fastlio=${START_FASTLIO} bridge=${START_BRIDGE} monitor=${START_MONITOR}
[DRY-RUN] 发布请求：${PUBLISH_VISION}；安装外参确认：${MOUNT_CONFIRMED}；方向测试确认：${DIRECTION_TEST_CONFIRMED}；近似方向发布：${ALLOW_APPROXIMATE_DIRECTION_PUBLISH}
[DRY-RUN] UDP 接收上限：net.core.rmem_max=${UDP_RMEM_MAX}（要求 >=${MIN_AIRY_UDP_RMEM_MAX}）
[DRY-RUN] 脚本不会切模式、解锁、写 PX4 参数或启动 PX4 SITL。
EOF
  exit 0
fi

for command_name in roscore roslaunch rosnode rostopic rosservice rosparam rospack timeout setsid python3; do
  command -v "${command_name}" >/dev/null 2>&1 || die "缺少命令：${command_name}"
done
[[ -f "${BRIDGE_SCRIPT}" ]] || die "缺少桥接脚本：${BRIDGE_SCRIPT}"
[[ -f "${MONITOR_SCRIPT}" ]] || die "缺少监控脚本：${MONITOR_SCRIPT}"
[[ -f "${MAVROS_SAFE_LAUNCH}" ]] || die "缺少受控 MAVROS launch：${MAVROS_SAFE_LAUNCH}"

if [[ "${PRINT_STATUS_ONLY}" == "1" ]]; then
  master_online || die "ROS master 未运行。"
  print_status
  exit 0
fi

ensure_no_live_recorded_session
mkdir -p "${LOG_DIR}"
export ROS_LOG_DIR="${LOG_DIR}/ros"
export ROS_HOME="${LOG_DIR}/ros_home"
mkdir -p "${ROS_LOG_DIR}" "${ROS_HOME}"
launcher_start_ticks="$(process_start_ticks "$$")"
[[ "${launcher_start_ticks}" =~ ^[0-9]+$ ]] || die "无法记录综合启动器的进程身份。"
{
  printf 'schema=%s\n' "${SESSION_SCHEMA}"
  printf 'project_root=%s\n' "${PROJECT_ROOT}"
  printf 'session_dir=%s\n' "${SESSION_DIR}"
  printf 'uav_name=%s\n' "${UAV_NAME}"
  printf 'boot_id=%s\n' "${BOOT_ID}"
  printf 'launcher_pid=%s\n' "$$"
  printf 'launcher_start_ticks=%s\n' "${launcher_start_ticks}"
  printf 'ros_master_uri=%s\n' "${ROS_MASTER_URI}"
  printf 'udp_rmem_max=%s\n' "${UDP_RMEM_MAX}"
  printf 'started_epoch=%s\n' "$(date +%s)"
} >"${SESSION_META}"
: >"${PROCESS_MANIFEST}"
active_pointer_tmp="${ACTIVE_SESSION_POINTER}.$$"
printf '%s\n' "${SESSION_DIR}" >"${active_pointer_tmp}"
mv -f -- "${active_pointer_tmp}" "${ACTIVE_SESSION_POINTER}"
info "会话目录：${SESSION_DIR}"

if ! master_online; then
  [[ "${START_ROSCORE}" == "1" ]] || die "ROS master 未运行且 START_ROSCORE=0。"
  start_component roscore "${LOG_DIR}/roscore.log" roscore
  wait_for_master || die "roscore 20 秒内未就绪。"
else
  info "复用当前 ROS master（退出时不会停止它）。"
fi

if [[ "${START_MAVROS}" == "1" ]]; then
  node_exists "${MAVROS_PREFIX}" && die "已存在 ${MAVROS_PREFIX}；请先停止旧 MAVROS，避免串口被重复占用。"
  if [[ "${FCU_URL}" == /dev/* ]]; then
    serial_device="${FCU_URL%%:*}"
    [[ -e "${serial_device}" ]] || die "串口不存在：${serial_device}"
    [[ -r "${serial_device}" && -w "${serial_device}" ]] || \
      die "当前用户无权读写 ${serial_device}；请确认 dialout 组后重新登录。"
  fi
  rospack find mavros >/dev/null 2>&1 || die "未安装 ros-noetic-mavros。"
  rospack find mavros_extras >/dev/null 2>&1 || die "未安装 ros-noetic-mavros-extras（vision_pose 插件在该包中）。"
  mavros_launch_args=(
    "${MAVROS_SAFE_LAUNCH}"
    "fcu_url:=${FCU_URL}"
    "tgt_system:=${MAV_SYS_ID}"
    "tgt_component:=${MAV_COMP_ID}"
    "log_output:=screen"
    "respawn_mavros:=false"
  )
  [[ -z "${GCS_URL}" ]] || mavros_launch_args+=("gcs_url:=${GCS_URL}")
  start_component mavros "${LOG_DIR}/mavros.log" \
    env ROS_NAMESPACE="${UAV_NAME}" roslaunch "${mavros_launch_args[@]}"
elif ! node_exists "${MAVROS_PREFIX}"; then
  die "START_MAVROS=0，但没有找到已有 ${MAVROS_PREFIX}。"
fi

wait_for_fcu_disarmed 30 || {
  tail -n 40 "${LOG_DIR}/mavros.log" >&2 || true
  die "30 秒内未建立未解锁的 PX4/MAVROS 链路。"
}
info "PX4/MAVROS 已连接，当前保持未解锁。"
verify_mavros_gcs_url || die "MAVROS GCS 转发配置不符合请求；拒绝继续启动。"

PUBLISH_EFFECTIVE=0
PUBLISH_BLOCK_REASONS=()
if [[ "${DIAGNOSTIC_ONLY}" == "1" ]]; then
  PUBLISH_BLOCK_REASONS+=("命令行指定 diagnostic-only")
elif [[ "${PUBLISH_VISION}" != "1" ]]; then
  PUBLISH_BLOCK_REASONS+=("PUBLISH_VISION 未显式设为 1")
fi
[[ "${MOUNT_CONFIRMED}" == "1" ]] || PUBLISH_BLOCK_REASONS+=("Airy IMU→base_link 安装外参未确认")
[[ "${START_BRIDGE}" == "1" ]] || PUBLISH_BLOCK_REASONS+=("START_BRIDGE=0，禁止绕过本项目坐标与失效门控")
[[ "${START_MONITOR}" == "1" ]] || PUBLISH_BLOCK_REASONS+=("START_MONITOR=0，禁止在无健康监控时发布")
if [[ "${UDP_RMEM_MAX}" == "unknown" ]]; then
  PUBLISH_BLOCK_REASONS+=("无法读取 net.core.rmem_max，不能确认 Airy UDP 接收缓冲")
elif ((UDP_RMEM_MAX < MIN_AIRY_UDP_RMEM_MAX)); then
  PUBLISH_BLOCK_REASONS+=("net.core.rmem_max=${UDP_RMEM_MAX} 小于 Airy 驱动要求 ${MIN_AIRY_UDP_RMEM_MAX}")
else
  info "Airy UDP 接收上限检查通过：net.core.rmem_max=${UDP_RMEM_MAX}。"
fi
if [[ "${DIRECTION_TEST_CONFIRMED}" != "1" && "${TEST_SECONDS}" == "0" && \
      "${ALLOW_APPROXIMATE_DIRECTION_PUBLISH}" != "1" ]]; then
  PUBLISH_BLOCK_REASONS+=("方向测试未确认；必须设置 --test-seconds，或在本机配置中显式允许近似方向发布")
elif [[ "${DIRECTION_TEST_CONFIRMED}" != "1" && \
        "${ALLOW_APPROXIMATE_DIRECTION_PUBLISH}" == "1" ]]; then
  warn "已显式允许使用近似安装方向持续发布；仅用于拆桨 Position 接受性验证，不代表方向测试或飞行验收完成。"
fi
topic_has_subscriber "${VISION_TOPIC}" "${MAVROS_PREFIX}" || \
  PUBLISH_BLOCK_REASONS+=("${VISION_TOPIC} 没有 MAVROS vision_pose 插件订阅者")

if wait_for_service "${VEHICLE_INFO_SERVICE}" 10 && wait_for_vehicle_info 10; then
  info "只读识别：MAV_AUTOPILOT=${FCU_AUTOPILOT}，PX4 固件 ${FCU_VERSION_MAJOR}.${FCU_VERSION_MINOR}.${FCU_VERSION_PATCH}。"
  [[ "${FCU_AUTOPILOT}" == "12" ]] || PUBLISH_BLOCK_REASONS+=("飞控 autopilot 类型不是 PX4/value 12")
  if [[ "${FCU_VERSION_MAJOR}" != "${EXPECTED_PX4_MAJOR}" || \
        "${FCU_VERSION_MINOR}" != "${EXPECTED_PX4_MINOR}" ]]; then
    PUBLISH_BLOCK_REASONS+=("固件不是已验证的 PX4 ${EXPECTED_PX4_MAJOR}.${EXPECTED_PX4_MINOR}.x")
  fi
else
  FCU_VERSION_MAJOR=unknown
  FCU_VERSION_MINOR=unknown
  FCU_VERSION_PATCH=unknown
  PUBLISH_BLOCK_REASONS+=("无法只读确认飞控类型和固件版本")
fi

if wait_for_service "${PARAM_PULL_SERVICE}" 10 && pull_px4_params; then
  info "只读同步 PX4 参数表完成：${PX4_PARAM_COUNT} 项。"
  PARAM_TABLE_READY=1
else
  PARAM_TABLE_READY=0
  PUBLISH_BLOCK_REASONS+=("无法只读同步 PX4 参数表")
fi

if [[ "${PARAM_TABLE_READY}" == "1" ]] && wait_for_service "${PARAM_GET_SERVICE}" 15 \
    && get_px4_param EKF2_AID_MASK; then
  PX4_AID_MASK="${PX4_PARAM_INTEGER}"
  info "只读检查：EKF2_AID_MASK=${PX4_AID_MASK}。"
  (( (PX4_AID_MASK & 8) != 0 )) || PUBLISH_BLOCK_REASONS+=("EKF2_AID_MASK 未开启视觉位置 bit 3/value 8")
  if (( (PX4_AID_MASK & 16) != 0 )) && [[ "${ALLOW_VISION_YAW_FUSION}" != "1" ]]; then
    PUBLISH_BLOCK_REASONS+=("视觉航向 bit 4/value 16 已开启但未获准")
  fi
  (( (PX4_AID_MASK & 64) == 0 )) || PUBLISH_BLOCK_REASONS+=("外部视觉旋转 bit 6/value 64 会与桥接 ENU 转换重复")
  (( (PX4_AID_MASK & 256) == 0 )) || PUBLISH_BLOCK_REASONS+=("视觉速度 bit 8/value 256 已开启，但当前桥不发送速度")
else
  PX4_AID_MASK="unknown"
  PUBLISH_BLOCK_REASONS+=("无法只读获取 EKF2_AID_MASK")
fi

if [[ "${PARAM_TABLE_READY}" == "1" ]] && get_px4_param EKF2_HGT_MODE; then
  PX4_HGT_MODE="${PX4_PARAM_INTEGER}"
  info "只读检查：EKF2_HGT_MODE=${PX4_HGT_MODE}。"
  [[ "${PX4_HGT_MODE}" == "3" ]] || \
    PUBLISH_BLOCK_REASONS+=("EKF2_HGT_MODE 不是外部视觉高度/value 3")
else
  PX4_HGT_MODE="unknown"
  PUBLISH_BLOCK_REASONS+=("无法只读获取 EKF2_HGT_MODE")
fi

if [[ "${PARAM_TABLE_READY}" == "1" ]] && get_px4_param SYS_MC_EST_GROUP; then
  PX4_ESTIMATOR_GROUP="${PX4_PARAM_INTEGER}"
  info "只读检查：SYS_MC_EST_GROUP=${PX4_ESTIMATOR_GROUP}。"
  [[ "${PX4_ESTIMATOR_GROUP}" == "2" ]] || PUBLISH_BLOCK_REASONS+=("SYS_MC_EST_GROUP 不是 EKF2/value 2")
else
  PX4_ESTIMATOR_GROUP=unknown
  PUBLISH_BLOCK_REASONS+=("无法确认活动估计器为 EKF2")
fi

if [[ "${PARAM_TABLE_READY}" == "1" ]] && get_px4_param EKF2_EV_DELAY; then
  PX4_EV_DELAY_MS="${PX4_PARAM_REAL}"
  info "只读记录：EKF2_EV_DELAY=${PX4_EV_DELAY_MS} ms（必须用 PX4 日志实测调节）。"
else
  PX4_EV_DELAY_MS=unknown
  warn "无法读取 EKF2_EV_DELAY；正式融合前必须用日志确认延迟。"
fi

for lever_param in EKF2_EV_POS_X EKF2_EV_POS_Y EKF2_EV_POS_Z; do
  if [[ "${PARAM_TABLE_READY}" == "1" ]] && get_px4_param "${lever_param}"; then
    if awk -v value="${PX4_PARAM_REAL}" 'BEGIN {if (value < -0.0001 || value > 0.0001) exit 0; exit 1}'; then
      PUBLISH_BLOCK_REASONS+=("${lever_param}=${PX4_PARAM_REAL} 非零，会与桥内杆臂补偿重复")
    fi
  else
    PUBLISH_BLOCK_REASONS+=("无法读取 ${lever_param}，不能排除重复杆臂补偿")
  fi
done

if ((${#PUBLISH_BLOCK_REASONS[@]} == 0)); then
  PUBLISH_EFFECTIVE=1
  warn "外部视觉发布门控已通过；桥仍会等待未解锁静置对齐后才开始发布。"
  if [[ "${DIRECTION_TEST_CONFIRMED}" != "1" ]]; then
    warn "当前 DIRECTION_TEST_CONFIRMED=0：监控将继续报告方向未确认；不得据此判定可以装桨飞行。"
  fi
else
  warn "外部视觉发布被安全门控禁止："
  for reason in "${PUBLISH_BLOCK_REASONS[@]}"; do
    warn "  - ${reason}"
  done
  warn "本次启动只验证链路，${VISION_TOPIC} 将保持 0 Hz。"
  warn "PX4 不会因此建立外部本地位置；此状态下遥控器无法切入 Position 属于预期保护。"
fi

if [[ "${START_FASTLIO}" == "1" ]]; then
  node_exists /laserMapping && die "已存在 /laserMapping；请先停止旧 FAST-LIO。"
  node_exists /rslidar_sdk_node && die "已存在 /rslidar_sdk_node；请先停止旧 Airy 驱动。"
  start_component fastlio "${LOG_DIR}/fastlio.log" \
    roslaunch fast_lio mapping_airy.launch start_driver:=true rviz:=false
fi

wait_for_topic_message /Odometry 45 || {
  tail -n 50 "${LOG_DIR}/fastlio.log" >&2 || true
  die "45 秒内没有收到 /Odometry；请检查 Airy 网络、点云、IMU 和初始化。"
}
info "FAST-LIO /Odometry 已输出。"

if [[ "${START_BRIDGE}" == "1" ]]; then
  node_exists /fastlio_to_mavros_bridge && die "已存在 /fastlio_to_mavros_bridge。"
  existing_publishers="$(publisher_count "${VISION_TOPIC}")"
  ((existing_publishers == 0)) || die "${VISION_TOPIC} 已有 ${existing_publishers} 个发布者；禁止多源同时写 PX4。"
  publish_bool=false
  mount_bool=false
  [[ "${PUBLISH_EFFECTIVE}" == "1" ]] && publish_bool=true
  [[ "${MOUNT_CONFIRMED}" == "1" ]] && mount_bool=true
  start_component bridge "${LOG_DIR}/bridge.log" \
    env PYTHONUNBUFFERED=1 python3 "${BRIDGE_SCRIPT}" \
      "_publish_enabled:=${publish_bool}" "_mount_confirmed:=${mount_bool}" \
      "_odom_topic:=/Odometry" "_sensor_imu_topic:=/rslidar_imu_data" \
      "_fcu_imu_topic:=${MAVROS_PREFIX}/imu/data" "_state_topic:=${STATE_TOPIC}" \
      "_timesync_topic:=${MAVROS_PREFIX}/timesync_status" \
      "_output_topic:=${VISION_TOPIC}" \
      "_sensor_to_body_quaternion_xyzw:=[${SENSOR_TO_BODY_Q_X},${SENSOR_TO_BODY_Q_Y},${SENSOR_TO_BODY_Q_Z},${SENSOR_TO_BODY_Q_W}]" \
      "_sensor_position_in_body_xyz_m:=[${SENSOR_POSITION_IN_BODY_X},${SENSOR_POSITION_IN_BODY_Y},${SENSOR_POSITION_IN_BODY_Z}]" \
      "_stable_seconds:=${ALIGN_STABLE_SECONDS}" \
      "_min_source_rate_hz:=${MIN_FASTLIO_RATE_HZ}" \
      "_max_source_age_s:=${MAX_SOURCE_AGE_S}" \
      "_max_source_gap_s:=${MAX_SOURCE_GAP_S}" \
      "_max_receipt_stall_s:=${MAX_RECEIPT_STALL_S}" \
      "_max_position_jump_m:=${MAX_POSITION_JUMP_M}" \
      "_max_orientation_jump_deg:=${MAX_ORIENTATION_JUMP_DEG}" \
      "_max_gravity_mismatch_deg:=${MAX_GRAVITY_MISMATCH_DEG}"
  wait_for_topic_message /airy_px4/bridge/diagnostics 10 || die "桥接器没有输出诊断。"
fi

if [[ "${START_MONITOR}" == "1" ]]; then
  node_exists /airy_px4_readiness_monitor && die "已存在 /airy_px4_readiness_monitor。"
  direction_bool=false
  [[ "${DIRECTION_TEST_CONFIRMED}" == "1" ]] && direction_bool=true
  start_component monitor "${LOG_DIR}/monitor.log" \
    env PYTHONUNBUFFERED=1 python3 "${MONITOR_SCRIPT}" \
      "_uav_name:=${UAV_NAME}" "_source_min_rate_hz:=${MIN_FASTLIO_RATE_HZ}" \
      "_flight_min_pose_rate_hz:=${FLIGHT_MIN_POSE_RATE_HZ}" \
      "_local_pose_min_rate_hz:=${MIN_LOCAL_POSE_RATE_HZ}" \
      "_max_data_age_s:=${MAX_SOURCE_AGE_S}" \
      "_direction_test_confirmed:=${direction_bool}"
  wait_for_topic_message /airy_px4/monitor/diagnostics 10 || die "监控器没有输出诊断。"
fi

cat <<EOF

================ Airy + PX4 启动完成 ================
FCU 链路：CONNECTED，启动时确认 DISARMED
FCU 固件：PX4 ${FCU_VERSION_MAJOR}.${FCU_VERSION_MINOR}.${FCU_VERSION_PATCH}
FAST-LIO：/Odometry 正常输出
视觉发布请求：${PUBLISH_VISION}
视觉实际发布门控：${PUBLISH_EFFECTIVE}
近似方向发布许可：${ALLOW_APPROXIMATE_DIRECTION_PUBLISH}
方向动态测试确认：${DIRECTION_TEST_CONFIRMED}
EKF2_AID_MASK：${PX4_AID_MASK}
EKF2_HGT_MODE：${PX4_HGT_MODE}
活动估计器：SYS_MC_EST_GROUP=${PX4_ESTIMATOR_GROUP}
外部视觉延迟参数：EKF2_EV_DELAY=${PX4_EV_DELAY_MS} ms（只读）
日志目录：${SESSION_DIR}

本脚本没有切换模式、解锁、写参数或发送控制量。
$(if [[ "${PUBLISH_EFFECTIVE}" == "0" ]]; then
    printf '当前视觉输出被门控为 0 Hz：本次启动不能用于 Position 模式。\n'
  fi)
监控只把 ESTIMATOR_STATUS 解释为与来源无关的 PX4 solution flags，不会据此
声称 Airy 外部视觉已经融合。若本会话实际观察到 POSCTL，会单独记录
position_mode_acceptance_verified=True；ev_fusion_verified 仍保持 False，
在缺少直接融合证据时绝不输出 POSITION_DATA_READY。
必须另用 PX4 日志/uORB 融合证据和拆桨方向/失效测试完成验收；不得把本次
监控结果作为切换 Position 或解锁飞行的依据。
正常退出请按 Ctrl+C。
=====================================================
EOF

if [[ "${TEST_SECONDS}" != "0" ]]; then
  info "开始 ${TEST_SECONDS} 秒未解锁台架测试。"
  test_start="$(date +%s)"
  while (( $(date +%s) - test_start < TEST_SECONDS )); do
    check_owned_processes || die "台架测试中有组件退出。"
    snapshot="$(state_snapshot)"
    armed="$(awk '/^armed:/ {print $2; exit}' <<<"${snapshot}")"
    [[ "${armed}" != "True" ]] || die "台架测试期间飞控被解锁，立即结束测试。"
    sleep 1
  done
  print_status | tee "${SESSION_DIR}/final_status.txt"
  assert_diagnostic_test || die "未解锁台架测试未通过；请查看上方验收失败项和 ${SESSION_DIR}。"
  info "未解锁可观测数据链台架测试通过；EV 融合与 Position 模式接受仍未验证，不能据此进入飞行。"
  exit 0
fi

while true; do
  check_owned_processes || die "某个启动组件已退出。"
  sleep 2
done
