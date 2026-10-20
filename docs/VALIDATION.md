# Kiểm chứng cấu hình ba robot và một RViz

Đã kiểm tra trong workspace ROS 2 Humble ngày 20/10/2026:

- `bash scripts/build.sh`: build thành công hai package `tb4_multi_tools` và
  `tb4_multi_bringup`.
- `src/tb4_multi_bringup/test/test_fleet_view.py`: kiểm thử tích hợp với ROS thật,
  TF/map/scan và action server Nav2 giả, domain 173, localhost. Đã qua kiểm tra
  prefix frame, giữ map chung, static TF của ba robot với subscriber mở muộn,
  map transient-local, scan robot3, pose/goal tới đúng từng namespace, ba goal
  hoạt động đồng thời, từ chối goal trùng/frame sai/robot ID sai và hủy riêng
  robot3 trong khi robot1/robot2 tiếp tục chạy.
- `tests/smoke_navigation.py robot3`: Nav2/AMCL thật với map và cảm biến giả,
  domain 170, localhost. AMCL, map server, controller, planner và BT navigator
  đều `active`; lập đường thành công (`status=4`, 8 pose), không phát map/TF ra
  topic gốc `/map` hoặc `/tf`. Không chạy hardware và không gửi goal điều khiển
  chuyển động.

Chạy lại kiểm thử fleet trong một terminal:

```bash
(
  set -e
  cd "$HOME/turtlebot4_ws"
  source /opt/ros/humble/setup.bash
  source "$HOME/turtlebot4_ws/install/setup.bash"
  python3 src/tb4_multi_bringup/test/test_fleet_view.py -v
)
```

Bài test fleet tự đặt domain 173/localhost, gỡ discovery/profile/namespace của
robot khỏi môi trường trước khi tạo node. Không dùng domain này cho robot thật
trong lúc kiểm thử. Test Nav2 cũng tự đặt domain riêng theo robot: 168/169/170.

Chưa có màn hình/X server trong môi trường kiểm tra để xác nhận hình ảnh RViz.
Cấu hình RViz đã được đối chiếu plugin cài sẵn; cần mở trên laptop để kiểm tra
model, scan, đường đi và nhãn robot trong thực tế. Chưa kiểm thử kết nối DDS
qua ba Pi hoặc navigation đồng thời trên ba robot thật. Không có bộ điều phối
tránh xung đột giữa các robot.
