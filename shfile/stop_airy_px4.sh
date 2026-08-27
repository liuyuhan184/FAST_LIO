#!/usr/bin/env bash
# Stop only the Airy + FAST-LIO + MAVROS/PX4 stack owned by this project.

set -Eeuo pipefail
IFS=$'\n\t'

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"

UAV_NAME="${UAV_NAME:-liu}"
ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
SESSION_ROOT="${SESSION_ROOT:-${PROJECT_ROOT}/runtime/airy_px4}"
CONFIG_FILE="${AIRY_PX4_CONFIG:-${SCRIPT_DIR}/airy_px4.env}"
CONFIG_EXPLICIT=0
DRY_RUN=0

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
elif [[ "${CONFIG_EXPLICIT}" == "1" ]]; then
  printf '[ERROR] 找不到配置文件：%s\n' "${CONFIG_FILE}" >&2
  exit 2
fi

usage() {
  cat <<'EOF'
用法：bash shfile/stop_airy_px4.sh [选项]

停止本项目综合启动器创建的 Airy 驱动、FAST-LIO、视觉坐标桥、状态监控器和
MAVROS。优先使用启动器记录的 PID、启动时间和进程组清单；清单不存在时，才按
本项目绝对路径和指定 MAVROS 命名空间清理当前用户的本机残留进程。

选项：
  --config FILE  使用与启动器相同的本机配置文件
  --dry-run      只显示将停止的对象，不发送信号
  -h, --help     显示帮助

通常直接执行：
  bash shfile/stop_airy_px4.sh

安全边界：
  - 不会向 PX4 发送切模式、解锁、参数或控制命令。
  - 只处理当前用户且进程启动时间仍匹配的目标，避免 PID 复用误伤。
  - 只停止会话清单明确记录为本次创建的 roscore；复用的 ROS master 始终保留。
  - 重复执行时，“当前没有相关进程”也视为成功。
EOF
}

set -- "${ORIGINAL_ARGS[@]}"
while (($# > 0)); do
  case "$1" in
    --config)
      (($# >= 2)) || { printf '[ERROR] --config 缺少路径。\n' >&2; exit 2; }
      shift 2
      ;;
    --dry-run) DRY_RUN=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) printf '[ERROR] 未知选项：%s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "${UAV_NAME}" =~ ^[A-Za-z][A-Za-z0-9_]*$ ]] || {
  printf '[ERROR] UAV_NAME 不是合法 ROS 命名空间：%s\n' "${UAV_NAME}" >&2
  exit 2
}
if [[ "${SESSION_ROOT}" != /* ]]; then
  SESSION_ROOT="${PROJECT_ROOT}/${SESSION_ROOT#./}"
fi
command -v realpath >/dev/null 2>&1 || {
  printf '[ERROR] 缺少命令：realpath\n' >&2
  exit 1
}
SESSION_ROOT="$(realpath -m -- "${SESSION_ROOT}")"
[[ -r /proc/sys/kernel/random/boot_id ]] || {
  printf '[ERROR] 无法读取系统 boot_id。\n' >&2
  exit 1
}
BOOT_ID="$(tr -d '[:space:]' </proc/sys/kernel/random/boot_id)"
[[ "${BOOT_ID}" =~ ^[0-9a-fA-F-]{36}$ ]] || {
  printf '[ERROR] 系统 boot_id 格式无效。\n' >&2
  exit 1
}
readonly BOOT_ID
readonly SESSION_SCHEMA=2

if [[ -t 1 ]]; then
  readonly GREEN=$'\033[1;32m' YELLOW=$'\033[1;33m' RED=$'\033[0;31m' RESET=$'\033[0m'
else
  readonly GREEN="" YELLOW="" RED="" RESET=""
fi
info() { printf '%s[INFO]%s %s\n' "${GREEN}" "${RESET}" "$*"; }
warn() { printf '%s[WARN]%s %s\n' "${YELLOW}" "${RESET}" "$*" >&2; }
die() { printf '%s[ERROR]%s %s\n' "${RED}" "${RESET}" "$*" >&2; exit 1; }

readonly MAVROS_NODE="/${UAV_NAME}/mavros"
readonly ACTIVE_SESSION_POINTER="${SESSION_ROOT}/active_${UAV_NAME}.session"
readonly STOP_LOCK_ROOT="${PROJECT_ROOT}/runtime/airy_px4_locks"
readonly STOP_LOCK_FILE="${STOP_LOCK_ROOT}/launcher_uid${UID}_${UAV_NAME}.lock"
readonly SELF_PID="$$"
readonly PARENT_PID="${PPID}"
SELF_PGID="$(ps -o pgid= -p "${SELF_PID}" 2>/dev/null | tr -d '[:space:]')"

STOP_COUNT=0
SESSION_HANDLED=0
SESSION_DIR_HANDLED=""
STALE_POINTER_MARKED=0
STALE_POINTER_VALUE=""
MANIFEST_LOAD_ERROR=""
CHECK_SESSION_GROUPS_AFTER_FALLBACK=0
SESSION_LAUNCHER_PID=""
SESSION_LAUNCHER_TICKS=""
SESSION_BOOT_ID=""
SESSION_NAMES=()
SESSION_PIDS=()
SESSION_PGIDS=()
SESSION_TICKS=()
FALLBACK_PIDS=()
FALLBACK_PGIDS=()
FALLBACK_TICKS=()
FALLBACK_DESCRIPTIONS=()
STOP_LOCK_FD_OPEN=0
STOP_LOCK_HELD=0

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
  [[ "${pid}" != "${SELF_PID}" && -d "/proc/${pid}" ]] || return 1
  owner_uid="$(stat -c '%u' "/proc/${pid}" 2>/dev/null)" || return 1
  [[ "${owner_uid}" == "${UID}" ]] || return 1
  actual_ticks="$(process_start_ticks "${pid}")" || return 1
  [[ "${actual_ticks}" == "${expected_ticks}" ]]
}

meta_value() {
  local file="$1" key="$2"
  awk -F= -v wanted="${key}" '$1 == wanted {sub(/^[^=]*=/, ""); print; exit}' "${file}"
}

signal_validated_pid() {
  local signal_name="$1" pid="$2" expected_ticks="$3" description="$4"
  same_process "${pid}" "${expected_ticks}" || return 0
  if [[ "${DRY_RUN}" == "1" ]]; then
    info "[DRY-RUN] ${signal_name} PID ${pid}：${description}"
    return 0
  fi
  # Revalidate immediately before signaling so a reused PID is never targeted.
  same_process "${pid}" "${expected_ticks}" || return 0
  if kill -"${signal_name}" "${pid}" 2>/dev/null; then
    STOP_COUNT=$((STOP_COUNT + 1))
  fi
}

signal_recorded_group() {
  local signal_name="$1" pid="$2" expected_pgid="$3" expected_ticks="$4" description="$5"
  local actual_pgid
  same_process "${pid}" "${expected_ticks}" || return 0
  actual_pgid="$(process_pgid "${pid}" 2>/dev/null || true)"
  if [[ "${actual_pgid}" != "${expected_pgid}" || "${expected_pgid}" != "${pid}" || \
        "${expected_pgid}" == "${SELF_PGID}" ]]; then
    signal_validated_pid "${signal_name}" "${pid}" "${expected_ticks}" \
      "${description}（仅 PID；进程组身份不匹配）"
    return 0
  fi
  if [[ "${DRY_RUN}" == "1" ]]; then
    info "[DRY-RUN] ${signal_name} 进程组 ${expected_pgid}：${description}"
    return 0
  fi
  same_process "${pid}" "${expected_ticks}" || return 0
  actual_pgid="$(process_pgid "${pid}" 2>/dev/null || true)"
  [[ "${actual_pgid}" == "${expected_pgid}" ]] || return 0
  if kill -"${signal_name}" -- "-${expected_pgid}" 2>/dev/null; then
    STOP_COUNT=$((STOP_COUNT + 1))
  fi
}

wait_for_identity_exit() {
  local pid="$1" expected_ticks="$2" attempts="$3" step
  for ((step = 0; step < attempts; step++)); do
    same_process "${pid}" "${expected_ticks}" || return 0
    sleep 0.1
  done
  return 1
}

acquire_stop_lock() {
  local wait_seconds="${1:-0}"
  [[ "${DRY_RUN}" != "1" ]] || return 0
  [[ "${STOP_LOCK_HELD}" != "1" ]] || return 0
  command -v flock >/dev/null 2>&1 || die "缺少命令：flock"
  if [[ "${STOP_LOCK_FD_OPEN}" != "1" ]]; then
    mkdir -p -- "${STOP_LOCK_ROOT}"
    exec 9>"${STOP_LOCK_FILE}"
    STOP_LOCK_FD_OPEN=1
  fi
  if [[ "${wait_seconds}" == "0" ]]; then
    flock -n 9 || return 1
  else
    flock -w "${wait_seconds}" 9 || return 1
  fi
  STOP_LOCK_HELD=1
}

require_stop_lock() {
  acquire_stop_lock 5 || \
    die "综合启动器仍占用启动锁；为避免与新组件创建竞态，已保留活动会话指针。请确认上方启动器 PID 后重试停止。"
}

session_processes_remain() {
  local index
  if same_process "${SESSION_LAUNCHER_PID}" "${SESSION_LAUNCHER_TICKS}"; then
    return 0
  fi
  for ((index = 0; index < ${#SESSION_PIDS[@]}; index++)); do
    same_process "${SESSION_PIDS[index]}" "${SESSION_TICKS[index]}" && return 0
    process_group_has_members "${SESSION_PGIDS[index]}" && return 0
  done
  return 1
}

remove_handled_pointer_if_stale() {
  local current_pointer=""
  [[ "${DRY_RUN}" != "1" ]] || return 0
  session_processes_remain && return 0
  [[ -f "${ACTIVE_SESSION_POINTER}" ]] || return 0
  IFS= read -r current_pointer <"${ACTIVE_SESSION_POINTER}" || true
  if [[ "${current_pointer}" == "${SESSION_DIR_HANDLED}" ]]; then
    rm -f -- "${ACTIVE_SESSION_POINTER}"
  fi
}

load_session_manifest() {
  local manifest_file="$1" name pid pgid start_ticks record_boot_id extra last_byte
  MANIFEST_LOAD_ERROR=""
  SESSION_NAMES=()
  SESSION_PIDS=()
  SESSION_PGIDS=()
  SESSION_TICKS=()
  [[ -s "${manifest_file}" ]] || return 0
  last_byte="$(tail -c 1 -- "${manifest_file}" | od -An -t u1 | tr -d '[:space:]')"
  if [[ "${last_byte}" != "10" ]]; then
    MANIFEST_LOAD_ERROR="会话进程清单存在未完整写入的记录：${manifest_file}"
    return 1
  fi
  while IFS=$'\t' read -r name pid pgid start_ticks record_boot_id extra; do
    if [[ -n "${extra:-}" || -z "${name}" || ! "${pid}" =~ ^[1-9][0-9]*$ || \
          ! "${pgid}" =~ ^[1-9][0-9]*$ || ! "${start_ticks}" =~ ^[0-9]+$ || \
          "${record_boot_id}" != "${SESSION_BOOT_ID}" || "${record_boot_id}" != "${BOOT_ID}" ]]; then
      MANIFEST_LOAD_ERROR="会话进程清单格式无效：${manifest_file}"
      SESSION_NAMES=()
      SESSION_PIDS=()
      SESSION_PGIDS=()
      SESSION_TICKS=()
      return 1
    fi
    SESSION_NAMES+=("${name}")
    SESSION_PIDS+=("${pid}")
    SESSION_PGIDS+=("${pgid}")
    SESSION_TICKS+=("${start_ticks}")
  done <"${manifest_file}"
}

remove_stale_pointer_after_fallback() {
  local current_pointer=""
  [[ "${STALE_POINTER_MARKED}" == "1" && -f "${ACTIVE_SESSION_POINTER}" ]] || return 0
  IFS= read -r current_pointer <"${ACTIVE_SESSION_POINTER}" || true
  if [[ "${current_pointer}" == "${STALE_POINTER_VALUE}" ]]; then
    rm -f -- "${ACTIVE_SESSION_POINTER}"
    warn "已移除完成严格兜底后的失效活动会话指针。"
  fi
}

stop_loaded_session_components() {
  local index wait_step any_alive
  for ((index = ${#SESSION_PIDS[@]} - 1; index >= 0; index--)); do
    signal_recorded_group TERM \
      "${SESSION_PIDS[index]}" "${SESSION_PGIDS[index]}" "${SESSION_TICKS[index]}" \
      "会话组件 ${SESSION_NAMES[index]}"
  done
  [[ "${DRY_RUN}" != "1" ]] || return 0
  for ((wait_step = 0; wait_step < 50; wait_step++)); do
    any_alive=0
    for ((index = 0; index < ${#SESSION_PIDS[@]}; index++)); do
      same_process "${SESSION_PIDS[index]}" "${SESSION_TICKS[index]}" && any_alive=1
    done
    ((any_alive == 0)) && break
    sleep 0.1
  done
  for ((index = ${#SESSION_PIDS[@]} - 1; index >= 0; index--)); do
    signal_recorded_group KILL \
      "${SESSION_PIDS[index]}" "${SESSION_PGIDS[index]}" "${SESSION_TICKS[index]}" \
      "未响应 TERM 的会话组件 ${SESSION_NAMES[index]}"
  done
}

stop_recorded_session() {
  local pointer_session session_dir meta_file manifest_file
  local meta_schema meta_project meta_uav meta_master meta_boot_id
  local index launcher_signaled=0 initial_manifest_count=0 reconciled_manifest_count=0

  [[ -f "${ACTIVE_SESSION_POINTER}" ]] || return 0
  pointer_session=""
  IFS= read -r pointer_session <"${ACTIVE_SESSION_POINTER}" || true
  if [[ -z "${pointer_session}" ]]; then
    STALE_POINTER_MARKED=1
    STALE_POINTER_VALUE=""
    warn "活动会话指针为空；完成严格路径兜底后将移除该指针。"
    return 0
  fi
  session_dir="$(realpath -m -- "${pointer_session}")"
  case "${session_dir}" in
    "${SESSION_ROOT}"/*) ;;
    *)
      STALE_POINTER_MARKED=1
      STALE_POINTER_VALUE="${pointer_session}"
      warn "活动会话指针超出 SESSION_ROOT；完成严格路径兜底后仅移除指针本身：${pointer_session}"
      return 0
      ;;
  esac
  meta_file="${session_dir}/session.meta"
  manifest_file="${session_dir}/processes.tsv"
  if [[ ! -f "${meta_file}" || ! -f "${manifest_file}" ]]; then
    STALE_POINTER_MARKED=1
    STALE_POINTER_VALUE="${pointer_session}"
    warn "活动会话清单不完整，将使用本项目路径兜底清理：${session_dir}"
    return 0
  fi

  meta_schema="$(meta_value "${meta_file}" schema)"
  meta_project="$(meta_value "${meta_file}" project_root)"
  meta_uav="$(meta_value "${meta_file}" uav_name)"
  meta_master="$(meta_value "${meta_file}" ros_master_uri)"
  meta_boot_id="$(meta_value "${meta_file}" boot_id)"
  if [[ -n "${meta_project}" && "${meta_project}" != "${PROJECT_ROOT}" ]] || \
     [[ -n "${meta_uav}" && "${meta_uav}" != "${UAV_NAME}" ]]; then
    die "活动会话明确属于其他项目或 UAV，拒绝清理：${session_dir}。请使用创建该会话的项目与配置。"
  fi
  if [[ "${meta_schema}" != "${SESSION_SCHEMA}" || -z "${meta_project}" || \
        -z "${meta_uav}" || ! "${meta_boot_id}" =~ ^[0-9a-fA-F-]{36}$ ]]; then
    STALE_POINTER_MARKED=1
    STALE_POINTER_VALUE="${pointer_session}"
    warn "活动会话元数据缺失或损坏，将按本项目路径严格兜底：${session_dir}"
    return 0
  fi
  if [[ "${meta_boot_id}" != "${BOOT_ID}" ]]; then
    STALE_POINTER_MARKED=1
    STALE_POINTER_VALUE="${pointer_session}"
    warn "活动会话来自另一系统启动批次；绝不使用断电前 PID，将按本项目路径严格兜底。"
    return 0
  fi
  SESSION_BOOT_ID="${meta_boot_id}"
  if [[ ! "${meta_master}" =~ ^https?://[^[:space:]]+$ ]]; then
    warn "会话中的 ros_master_uri 记录无效；停止过程不会访问 ROS master，将继续只按 PID 清理。"
  elif [[ "${meta_master}" != "${ROS_MASTER_URI}" ]]; then
    warn "会话 ROS master 为 ${meta_master}，当前环境为 ${ROS_MASTER_URI}；将只按本机会话 PID 清理，不访问 ROS master。"
  fi

  SESSION_DIR_HANDLED="${session_dir}"
  SESSION_LAUNCHER_PID="$(meta_value "${meta_file}" launcher_pid)"
  SESSION_LAUNCHER_TICKS="$(meta_value "${meta_file}" launcher_start_ticks)"
  if [[ ! "${SESSION_LAUNCHER_PID}" =~ ^[1-9][0-9]*$ || \
        ! "${SESSION_LAUNCHER_TICKS}" =~ ^[0-9]+$ ]]; then
    STALE_POINTER_MARKED=1
    STALE_POINTER_VALUE="${pointer_session}"
    warn "会话中的启动器身份记录损坏，将按本项目路径严格兜底。"
    return 0
  fi

  if ! load_session_manifest "${manifest_file}"; then
    warn "${MANIFEST_LOAD_ERROR}"
    if same_process "${SESSION_LAUNCHER_PID}" "${SESSION_LAUNCHER_TICKS}"; then
      if [[ "${DRY_RUN}" == "1" ]]; then
        warn "启动器仍在运行；dry-run 不会修改损坏清单，将改用严格路径显示兜底目标。"
      else
        signal_validated_pid TERM "${SESSION_LAUNCHER_PID}" "${SESSION_LAUNCHER_TICKS}" \
          "综合启动器（先触发其退出 trap，再恢复损坏清单）"
        launcher_signaled=1
        wait_for_identity_exit "${SESSION_LAUNCHER_PID}" "${SESSION_LAUNCHER_TICKS}" 120 || \
          warn "综合启动器 12 秒内未退出，将继续严格路径兜底。"
        if load_session_manifest "${manifest_file}"; then
          info "启动器退出后会话清单已恢复为完整记录。"
        else
          warn "${MANIFEST_LOAD_ERROR}"
        fi
      fi
    fi
    if [[ "${DRY_RUN}" == "1" || -n "${MANIFEST_LOAD_ERROR}" ]]; then
      STALE_POINTER_MARKED=1
      STALE_POINTER_VALUE="${pointer_session}"
      warn "无法安全使用会话清单，将按本项目路径严格兜底；可能存在的旧 roscore 会保留。"
      return 0
    fi
  fi
  if ((${#SESSION_PIDS[@]} == 0)) && \
     ! same_process "${SESSION_LAUNCHER_PID}" "${SESSION_LAUNCHER_TICKS}"; then
    STALE_POINTER_MARKED=1
    STALE_POINTER_VALUE="${pointer_session}"
    warn "会话清单为空且启动器已退出；将按本项目路径严格兜底。无法证明旧会话是否创建过 roscore，因此不会扫描或停止其他 ROS master。"
    return 0
  fi

  initial_manifest_count="${#SESSION_PIDS[@]}"
  SESSION_HANDLED=1
  info "找到综合启动器会话：${session_dir}"
  if [[ "${launcher_signaled}" == "0" ]]; then
    signal_validated_pid TERM "${SESSION_LAUNCHER_PID}" "${SESSION_LAUNCHER_TICKS}" \
      "综合启动器（由其退出 trap 清理组件）"
  fi
  if [[ "${DRY_RUN}" != "1" && "${launcher_signaled}" == "0" ]]; then
    wait_for_identity_exit "${SESSION_LAUNCHER_PID}" "${SESSION_LAUNCHER_TICKS}" 120 || \
      warn "综合启动器 12 秒内未退出，继续按会话清单清理。"
  fi
  if [[ "${DRY_RUN}" != "1" ]]; then
    if same_process "${SESSION_LAUNCHER_PID}" "${SESSION_LAUNCHER_TICKS}"; then
      signal_validated_pid KILL "${SESSION_LAUNCHER_PID}" "${SESSION_LAUNCHER_TICKS}" \
        "未响应 TERM 的综合启动器"
      wait_for_identity_exit "${SESSION_LAUNCHER_PID}" "${SESSION_LAUNCHER_TICKS}" 20 || \
        die "综合启动器在 KILL 后仍未退出；已保留活动会话指针。"
    fi
    # The launcher-wide lock is independent of SESSION_ROOT.  Holding it closes
    # the gap between the last manifest append and final residual verification.
    require_stop_lock
    # A component may have been starting when TERM arrived.  The component-side
    # recorder appends before exec, so reload after the launcher has quiesced.
    if ! load_session_manifest "${manifest_file}"; then
      SESSION_HANDLED=0
      STALE_POINTER_MARKED=1
      STALE_POINTER_VALUE="${pointer_session}"
      warn "${MANIFEST_LOAD_ERROR}"
      warn "启动器退出后清单仍损坏，将按本项目路径严格兜底；可能存在的旧 roscore 会保留。"
      return 0
    fi
    if ((${#SESSION_PIDS[@]} < initial_manifest_count)); then
      SESSION_HANDLED=0
      STALE_POINTER_MARKED=1
      STALE_POINTER_VALUE="${pointer_session}"
      warn "会话清单在启动器退出时被截短，将按本项目路径严格兜底；可能存在的旧 roscore 会保留。"
      return 0
    fi
    if ((${#SESSION_PIDS[@]} == 0)); then
      SESSION_HANDLED=0
      STALE_POINTER_MARKED=1
      STALE_POINTER_VALUE="${pointer_session}"
      warn "启动器退出后会话清单仍为空；将等待并按本项目路径严格兜底，避免遗漏尚未完成 exec 的组件。"
      return 0
    fi
  fi

  stop_loaded_session_components
  if [[ "${DRY_RUN}" != "1" ]]; then
    # A wrapper that had not appended when the first reload happened is an exact
    # project-path fallback target.  Stop it while the launcher lock is held,
    # then reload once more: append always precedes exec, including for roscore.
    stop_fallback_processes
    reconciled_manifest_count="${#SESSION_PIDS[@]}"
    if ! load_session_manifest "${manifest_file}"; then
      SESSION_HANDLED=0
      STALE_POINTER_MARKED=1
      STALE_POINTER_VALUE="${pointer_session}"
      warn "${MANIFEST_LOAD_ERROR}"
      warn "残留清理后清单损坏，将保留安全校验并继续严格路径兜底。"
      return 0
    fi
    if ((${#SESSION_PIDS[@]} < reconciled_manifest_count)); then
      SESSION_HANDLED=0
      STALE_POINTER_MARKED=1
      STALE_POINTER_VALUE="${pointer_session}"
      warn "会话清单在残留清理期间被截短；将继续严格路径兜底并保留活动指针直到验证完成。"
      return 0
    fi
    if ((${#SESSION_PIDS[@]} > reconciled_manifest_count)); then
      info "发现并接管 ${#SESSION_PIDS[@]} 条会话记录（停止期间有组件完成身份写入）。"
    fi
    stop_loaded_session_components
    stop_fallback_processes
    for ((index = 0; index < ${#SESSION_PGIDS[@]}; index++)); do
      if process_group_has_members "${SESSION_PGIDS[index]}"; then
        warn "会话组件 ${SESSION_NAMES[index]} 的进程组 ${SESSION_PGIDS[index]} 仍有成员；改用严格路径兜底并保留组校验。"
        CHECK_SESSION_GROUPS_AFTER_FALLBACK=1
        SESSION_HANDLED=0
        STALE_POINTER_MARKED=1
        STALE_POINTER_VALUE="${pointer_session}"
      fi
    done
    [[ "${SESSION_HANDLED}" == "1" ]] || return 0
  fi
}

process_cwd() {
  local pid="$1"
  readlink -f -- "/proc/${pid}/cwd" 2>/dev/null
}

is_related_process() {
  local pid="$1" cwd executable argument wrapper_manifest
  local -a argv=()
  executable="$(readlink -f -- "/proc/${pid}/exe" 2>/dev/null || true)"
  case "${executable}" in
    "${PROJECT_ROOT}"/build/*/devel/lib/fast_lio/fastlio_mapping|\
    "${PROJECT_ROOT}"/environment/rslidar_ws/devel/*/lib/rslidar_sdk/rslidar_sdk_node)
      return 0
      ;;
  esac
  mapfile -d '' -t argv 2>/dev/null <"/proc/${pid}/cmdline" || return 1
  ((${#argv[@]} > 0)) || return 1
  case "${argv[0]}" in
    "${PROJECT_ROOT}/shfile/start_airy_px4.sh"|\
    "${PROJECT_ROOT}/shfile/fastlio_to_mavros.py"|\
    "${PROJECT_ROOT}/shfile/monitor_airy_px4.py") return 0 ;;
  esac
  if [[ "${executable}" == /opt/ros/noetic/lib/mavros/mavros_node ]]; then
    for argument in "${argv[@]}"; do
      if [[ "${argument}" == "__ns:=/${UAV_NAME}" || "${argument}" == "__ns:=${UAV_NAME}" ]]; then
        return 0
      fi
    done
  fi
  if [[ "${executable}" == */python3 || "${executable}" == */python3.* ]]; then
    if ((${#argv[@]} >= 2)) && \
       [[ "${argv[1]}" == "${PROJECT_ROOT}/shfile/fastlio_to_mavros.py" || \
          "${argv[1]}" == "${PROJECT_ROOT}/shfile/monitor_airy_px4.py" ]]; then
      return 0
    fi
  fi
  if [[ "${executable}" == */bash ]] && ((${#argv[@]} >= 2)) && \
     [[ "${argv[1]}" == "${PROJECT_ROOT}/shfile/start_airy_px4.sh" ]]; then
    return 0
  fi
  if [[ "${executable}" == */bash ]] && ((${#argv[@]} >= 5)) && \
     [[ "${argv[1]}" == "-c" && "${argv[3]}" == "airy-session-child" ]]; then
    wrapper_manifest="$(realpath -m -- "${argv[4]}")"
    case "${wrapper_manifest}" in
      "${SESSION_ROOT}"/*/processes.tsv) return 0 ;;
    esac
  fi
  cwd="$(process_cwd "${pid}" || true)"
  [[ "${cwd}" == "${PROJECT_ROOT}" ]] || return 1
  if [[ "${executable}" == */bash ]] && ((${#argv[@]} >= 2)); then
    case "${argv[1]}" in
      shfile/start_airy_px4.sh|./shfile/start_airy_px4.sh) return 0 ;;
    esac
  fi
  if [[ "${executable}" == */python3 || "${executable}" == */python3.* ]] && \
     ((${#argv[@]} >= 2)); then
    case "${argv[1]}" in
      shfile/fastlio_to_mavros.py|./shfile/fastlio_to_mavros.py|\
      shfile/monitor_airy_px4.py|./shfile/monitor_airy_px4.py) return 0 ;;
    esac
  fi
  return 1
}

collect_related_processes() {
  local pid pgid arguments start_ticks
  FALLBACK_PIDS=()
  FALLBACK_PGIDS=()
  FALLBACK_TICKS=()
  FALLBACK_DESCRIPTIONS=()
  while IFS=$' \t' read -r pid pgid arguments; do
    [[ "${pid}" =~ ^[1-9][0-9]*$ && "${pid}" != "${SELF_PID}" && \
      "${pid}" != "${PARENT_PID}" && "${pgid}" =~ ^[1-9][0-9]*$ ]] || continue
    [[ "${arguments}" != *"stop_airy_px4.sh"* ]] || continue
    is_related_process "${pid}" "${arguments}" || continue
    start_ticks="$(process_start_ticks "${pid}" 2>/dev/null || true)"
    [[ "${start_ticks}" =~ ^[0-9]+$ ]] || continue
    FALLBACK_PIDS+=("${pid}")
    FALLBACK_PGIDS+=("${pgid}")
    FALLBACK_TICKS+=("${start_ticks}")
    FALLBACK_DESCRIPTIONS+=("${arguments}")
  done < <(ps -ww -u "${UID}" -o pid=,pgid=,args=)
}

stop_fallback_processes() {
  local pass index wait_step any_alive
  for pass in 1 2; do
    collect_related_processes
    if ((${#FALLBACK_PIDS[@]} == 0)); then
      if [[ "${pass}" == "1" && "${STALE_POINTER_MARKED}" == "1" && "${DRY_RUN}" != "1" ]]; then
        sleep 0.2
        continue
      fi
      return 0
    fi
    for ((index = 0; index < ${#FALLBACK_PIDS[@]}; index++)); do
      signal_recorded_group TERM "${FALLBACK_PIDS[index]}" "${FALLBACK_PGIDS[index]}" \
        "${FALLBACK_TICKS[index]}" \
        "本项目残留进程：${FALLBACK_DESCRIPTIONS[index]}"
    done
    [[ "${DRY_RUN}" == "1" ]] && return 0
    for ((wait_step = 0; wait_step < 50; wait_step++)); do
      any_alive=0
      for ((index = 0; index < ${#FALLBACK_PIDS[@]}; index++)); do
        same_process "${FALLBACK_PIDS[index]}" "${FALLBACK_TICKS[index]}" && any_alive=1
      done
      ((any_alive == 0)) && break
      sleep 0.1
    done
    for ((index = 0; index < ${#FALLBACK_PIDS[@]}; index++)); do
      signal_recorded_group KILL "${FALLBACK_PIDS[index]}" "${FALLBACK_PGIDS[index]}" \
        "${FALLBACK_TICKS[index]}" \
        "未响应 TERM 的本项目残留进程"
    done
  done
}

verify_recorded_session_stopped() {
  local index failures=0
  if same_process "${SESSION_LAUNCHER_PID}" "${SESSION_LAUNCHER_TICKS}"; then
    warn "综合启动器 PID ${SESSION_LAUNCHER_PID} 仍在运行。"
    failures=$((failures + 1))
  fi
  for ((index = 0; index < ${#SESSION_PIDS[@]}; index++)); do
    if same_process "${SESSION_PIDS[index]}" "${SESSION_TICKS[index]}"; then
      warn "会话组件 ${SESSION_NAMES[index]} PID ${SESSION_PIDS[index]} 仍在运行。"
      failures=$((failures + 1))
    fi
    if process_group_has_members "${SESSION_PGIDS[index]}"; then
      warn "会话组件 ${SESSION_NAMES[index]} 的进程组 ${SESSION_PGIDS[index]} 仍有成员。"
      failures=$((failures + 1))
    fi
  done
  collect_related_processes
  for ((index = 0; index < ${#FALLBACK_PIDS[@]}; index++)); do
    warn "仍有未清单化的本项目目标 PID ${FALLBACK_PIDS[index]}：${FALLBACK_DESCRIPTIONS[index]}"
    failures=$((failures + 1))
  done
  ((failures == 0))
}

verify_fallback_stopped() {
  local index failures=0
  collect_related_processes
  for ((index = 0; index < ${#FALLBACK_PIDS[@]}; index++)); do
    warn "仍有本项目目标 PID ${FALLBACK_PIDS[index]}：${FALLBACK_DESCRIPTIONS[index]}"
    failures=$((failures + 1))
  done
  if [[ "${CHECK_SESSION_GROUPS_AFTER_FALLBACK}" == "1" ]]; then
    for ((index = 0; index < ${#SESSION_PGIDS[@]}; index++)); do
      if process_group_has_members "${SESSION_PGIDS[index]}"; then
        warn "会话进程组 ${SESSION_PGIDS[index]} 仍有无法安全归属的成员；活动指针将保留。"
        failures=$((failures + 1))
      fi
    done
  fi
  ((failures == 0))
}

if [[ "${DRY_RUN}" != "1" ]]; then
  acquire_stop_lock 0 || true
fi

stop_recorded_session
if [[ "${SESSION_HANDLED}" == "0" ]]; then
  stop_fallback_processes
  if [[ "${DRY_RUN}" != "1" ]]; then
    # A launcher without a usable manifest may have owned the fixed lock.  Once
    # the first strict sweep has stopped it, take the lock and sweep again so a
    # concurrently starting component cannot appear after verification.
    require_stop_lock
    stop_fallback_processes
  fi
fi

if [[ "${DRY_RUN}" == "1" ]]; then
  info "dry-run 完成；未发送任何信号。"
elif [[ "${SESSION_HANDLED}" == "1" ]]; then
  if verify_recorded_session_stopped; then
    remove_handled_pointer_if_stale
    if ((STOP_COUNT == 0)); then
      info "记录会话中已没有存活进程。"
    else
      info "本次会话记录的 Airy/PX4 综合链路已停止。"
    fi
  else
    die "仍有会话进程未停止；已保留活动指针，请查看上方 PID。"
  fi
elif verify_fallback_stopped; then
  remove_stale_pointer_after_fallback
  if ((STOP_COUNT == 0)); then
    info "当前没有发现本项目 Airy/PX4 综合链路相关进程。"
  else
    info "本项目遗留的 Airy/PX4 综合链路进程已停止。"
  fi
else
  die "仍有本项目相关进程未停止；请查看上方 PID 后再处理。"
fi
