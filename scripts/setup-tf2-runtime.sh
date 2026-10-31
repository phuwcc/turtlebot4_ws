#!/usr/bin/env bash
# PC only: use official Humble packages containing the upstream TF deadlock fix.
# Extract locally; do not change system packages or require sudo.
set -euo pipefail
export LC_ALL=C
tb4_tmp="$(mktemp -d -t tb4-tf2-runtime.XXXXXX)"
trap 'rm -rf -- "$tb4_tmp"' EXIT
tb4_packages=(ros-humble-tf2 ros-humble-tf2-ros)
tb4_versions=()
for tb4_package in "${tb4_packages[@]}"; do
  tb4_version="$(apt-cache policy "$tb4_package" | awk '/Candidate:/ {print $2; exit}')"
  if [[ "$tb4_version" != 0.25.* ]] || ! dpkg --compare-versions "$tb4_version" ge 0.25.24; then
    echo "$tb4_package: cần candidate Humble >= 0.25.24; cập nhật APT index từ kho ROS chính thức trước." >&2
    exit 1
  fi
  tb4_versions+=("$tb4_version")
done
[[ "${tb4_versions[0]%%-*}" == "${tb4_versions[1]%%-*}" ]] || {
  echo 'Hai gói tf2/tf2_ros phải cùng phiên bản upstream.' >&2; exit 1;
}
cd "$tb4_tmp"
apt-get download "${tb4_packages[0]}=${tb4_versions[0]}" "${tb4_packages[1]}=${tb4_versions[1]}"
tb4_prefix="${XDG_DATA_HOME:-$HOME/.local/share}/tb4_multi_robot/geometry2/${tb4_versions[0]}"
mkdir -p "$tb4_prefix"
for tb4_deb in ./*.deb; do
  [[ "$(dpkg-deb -f "$tb4_deb" Architecture)" == "$(dpkg --print-architecture)" ]] || exit 1
  dpkg-deb -x "$tb4_deb" "$tb4_prefix"
done
python3 - "$tb4_prefix" <<'PY'
import hashlib,json,os,shlex,subprocess,sys,tempfile
from pathlib import Path
prefix=Path(sys.argv[1]);manifest={}
for deb in Path.cwd().glob('*.deb'):
    package=subprocess.check_output(['dpkg-deb','-f',str(deb),'Package'],text=True).strip()
    version=subprocess.check_output(['dpkg-deb','-f',str(deb),'Version'],text=True).strip()
    manifest[package]={'version':version,'sha256':hashlib.sha256(deb.read_bytes()).hexdigest()}
(prefix/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
config=Path.home()/'.config/turtlebot4/runtime_env.sh';config.parent.mkdir(parents=True,exist_ok=True)
lines=config.read_text().splitlines() if config.exists() else []
lines=[line for line in lines if not line.startswith('export TB4_TF2_RUNTIME_LIB=')]
lines.append('export TB4_TF2_RUNTIME_LIB='+shlex.quote(str(prefix/'opt/ros/humble/lib')))
with tempfile.NamedTemporaryFile(mode='w',dir=config.parent,delete=False) as stream:
    stream.write('\n'.join(lines)+'\n');temporary=stream.name
os.replace(temporary,config)
print('Đã cấu hình TF2 runtime riêng:',prefix)
print('Khởi động lại phiên PC bằng scripts/pc-start.sh để áp dụng.')
PY
