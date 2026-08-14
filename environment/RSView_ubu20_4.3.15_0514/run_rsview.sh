#!/usr/bin/env bash

set -Eeuo pipefail

readonly RSView_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"

if [[ "$(uname -m)" != "x86_64" ]]; then
  printf '[ERROR] 此 RSView 安装包是 x86-64 版本，不能在 %s 架构运行。\n' "$(uname -m)" >&2
  exit 1
fi

# Some archive/copy tools turn symbolic links into small text files. Repair
# them locally before the dynamic loader tries to open libpcap/VTK libraries.
if [[ -f "${RSView_ROOT}/lib/libpcap.so.1" && ! -L "${RSView_ROOT}/lib/libpcap.so.1" ]]; then
  "${RSView_ROOT}/repair_symlinks.sh"
fi

export LD_LIBRARY_PATH="${RSView_ROOT}/lib:${RSView_ROOT}/lib/rsview-4.3:${RSView_ROOT}/lib/paraview-4.3:${RSView_ROOT}/boost_runtime:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="${RSView_ROOT}/lib/paraview-4.3:${RSView_ROOT}/lib/paraview-4.3/site-packages:${RSView_ROOT}/lib/paraview-4.3/site-packages/vtk:${RSView_ROOT}/lib/rsview-4.3/site-packages:${PYTHONPATH:-}"

exec "${RSView_ROOT}/bin/RSView" "$@"
