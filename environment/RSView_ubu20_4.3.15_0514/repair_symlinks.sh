#!/usr/bin/env bash
# Repair symbolic links that were flattened into tiny text files while RSView
# was extracted or copied. Original placeholder files are kept in a backup.

set -Eeuo pipefail
IFS=$'\n\t'

readonly RSView_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly BACKUP_ROOT="${RSView_ROOT}/.flattened_symlink_backup"
converted=0

while IFS= read -r -d '' candidate; do
  size="$(stat -c '%s' -- "${candidate}")"
  ((size > 0 && size <= 255)) || continue

  case "$(basename -- "${candidate}")" in
    *.so|*.so.*) ;;
    *) continue ;;
  esac

  target="$(<"${candidate}")"
  [[ -n "${target}" ]] || continue
  [[ "${target}" =~ ^[A-Za-z0-9._+-]+$ ]] || continue
  [[ "${target}" != "$(basename -- "${candidate}")" ]] || continue

  candidate_dir="$(dirname -- "${candidate}")"
  [[ -e "${candidate_dir}/${target}" || -L "${candidate_dir}/${target}" ]] || continue

  relative_path="${candidate#"${RSView_ROOT}/"}"
  backup_path="${BACKUP_ROOT}/${relative_path}"
  if [[ -e "${backup_path}" || -L "${backup_path}" ]]; then
    printf '[ERROR] 备份文件已经存在：%s\n' "${backup_path}" >&2
    exit 1
  fi

  mkdir -p -- "$(dirname -- "${backup_path}")"
  mv -- "${candidate}" "${backup_path}"
  if ! ln -s -- "${target}" "${candidate}"; then
    mv -- "${backup_path}" "${candidate}"
    printf '[ERROR] 无法创建符号链接：%s -> %s\n' "${candidate}" "${target}" >&2
    exit 1
  fi

  converted=$((converted + 1))
done < <(find "${RSView_ROOT}" -path "${BACKUP_ROOT}" -prune -o -type f -size -256c -print0)

dangling=0
while IFS= read -r -d '' link; do
  if [[ ! -e "${link}" ]]; then
    printf '[ERROR] 无效符号链接：%s -> %s\n' "${link}" "$(readlink -- "${link}")" >&2
    dangling=$((dangling + 1))
  fi
done < <(find "${RSView_ROOT}" -path "${BACKUP_ROOT}" -prune -o -type l -print0)

((dangling == 0)) || exit 1
printf '[OK] 已修复 %d 个 RSView 符号链接。\n' "${converted}"
if ((converted > 0)); then
  printf '[INFO] 原文本占位文件备份在：%s\n' "${BACKUP_ROOT}"
fi
