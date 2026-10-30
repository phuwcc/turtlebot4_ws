#!/usr/bin/env bash
# Install missing dependencies, then build. No driver/service is started here.
set -eo pipefail
tb4_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
tb4_role="${1:-}"
case "$tb4_role" in
  pc) tb4_packages=(ros-humble-rviz2 ros-humble-nav2-bringup ros-humble-navigation2
                   ros-humble-slam-toolbox ros-humble-turtlebot4-description) ;;
  robot) tb4_packages=(ros-humble-turtlebot4-bringup ros-humble-create3-republisher) ;;
  *) echo 'Usage: bash scripts/setup.sh pc|robot [--refresh]' >&2; exit 2 ;;
esac
if (( $# > 2 )) || [[ -n "${2:-}" && "${2:-}" != --refresh ]]; then
  echo 'Usage: bash scripts/setup.sh pc|robot [--refresh]' >&2
  exit 2
fi
tb4_packages+=(python3-colcon-common-extensions python3-yaml
               ros-humble-rmw-fastrtps-cpp ros-humble-nav2-common)
echo "[1/3] Kiểm tra dependency cho $tb4_role..."
tb4_missing=()
for tb4_package in "${tb4_packages[@]}"; do
  if [[ "$(dpkg-query -W -f='${Status}' "$tb4_package" 2>/dev/null || true)" != 'install ok installed' ]]; then
    tb4_missing+=("$tb4_package")
  fi
done
if (( ${#tb4_missing[@]} == 0 )) && [[ "${2:-}" != --refresh ]]; then
  echo '[2/3] Đã đủ gói. Bỏ qua apt update/install.'
else
  echo "[2/3] Cập nhật danh sách gói (tối đa 180 giây). Gói thiếu: ${tb4_missing[*]:-không có; --refresh được chọn}"
  echo 'Sau các dòng Hit, Ubuntu có thể đang chạy apt-check để cập nhật thông báo hệ thống.'
  if sudo timeout --signal=INT --kill-after=10s 180s apt-get \
      -o Acquire::http::Timeout=15 -o Acquire::https::Timeout=15 \
      -o Acquire::Retries=1 -o APT::Update::Error-Mode=any update; then
    :
  else
    tb4_status=$?
    echo "apt update chưa hoàn tất (mã $tb4_status). Chưa cài gói hoặc build." >&2
    echo 'Kiểm tra kết nối/kho gói hoặc tiến trình apt-check, rồi chạy lại script.' >&2
    exit "$tb4_status"
  fi
  sudo apt-get -o DPkg::Lock::Timeout=300 \
    -o Acquire::http::Timeout=15 -o Acquire::https::Timeout=15 \
    -o Acquire::Retries=1 install -y "${tb4_packages[@]}"
fi
echo '[3/3] Build workspace. Đợi tới khi thấy Summary; build có thể mất vài phút trên Pi.'
exec bash "$tb4_repo/scripts/build.sh"
