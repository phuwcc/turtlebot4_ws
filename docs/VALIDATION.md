# Kiểm chứng kiến trúc PC navigation + socket

Ngày 22/10/2026, trong workspace ROS 2 Humble:

- Build thành công `tb4_multi_tools` và `tb4_multi_bringup`.
- 5 test cấu hình PC và 3 test fleet 1–3 robot đã qua.
- 3 test mới trong `test_remote_control.py` đã qua: bàn phím C++ phát
  TwistStamped và tự timeout; watchdog Pi dừng sau hết hạn, bỏ lệnh cũ/NaN/
  đảo thứ tự, dừng khi nhiều nguồn; TCP dùng chung goal với RViz, validate
  robot/toạ độ, publish pose, từ chối goal chồng, hủy goal và đọc kết quả.
- `tests/smoke_navigation.py --pc robot1` đã qua trên domain 172, localhost:
  chạy **launch PC kết hợp thật**, không RViz; AMCL, map server, controller,
  planner, BT navigator đều active; đặt pose qua socket thật; lập đường thành
  công (`status=4`, 8 pose). Đã kiểm tra remap controller → smoother,
  đầu ra `pc_cmd_vel` → relay `remote_cmd_vel`, PC không publish `cmd_vel`
  bỏ qua guard, không rò topic `/map` hoặc `/tf` ra namespace gốc.

Một lần chạy smoke gặp timeout phản hồi lifecycle AMCL khi khởi động trên
máy Pi; chạy lại trên domain sạch đã qua. Nếu AMCL/map server chưa active,
kiểm tra log lifecycle và kết nối DDS trước khi gửi goal. Các bài test mới
không phát lệnh tới robot thật.

Chạy lại toàn bộ kiểm thử sau khi build:

```bash
bash scripts/test.sh
```

Các bài test tự đặt domain riêng và localhost, bỏ môi trường discovery của
robot. Không dùng domain 168–174 cho robot thật trong lúc chạy test.
Log smoke gần nhất: `log/navigation_smoke/pc/result.json`,
`log/navigation_smoke/pc/navigation.log` (không commit).

Chưa chạy cấu hình mới trên máy PC case thật, chưa xác nhận DDS PC↔ba Pi,
chưa thử mất Wi-Fi hoặc navigation đồng thời trên ba robot thật. Không có màn
hình để kiểm tra trực quan RViz. Cần triển khai/build cùng phiên bản trên PC
và Pi trước khi kiểm thử thực tế. Không thay đổi dịch vụ robot đang chạy trong
lượt sửa code này.

---

## Kết quả lịch sử trước khi chuyển navigation sang PC

# Kiểm chứng cấu hình ba robot và một RViz

Đã kiểm tra trong workspace ROS 2 Humble ngày 20/10/2026:

- `bash scripts/build.sh`: build thành công hai package `tb4_multi_tools` và
  `tb4_multi_bringup`.
- `tests/test_pc_config.py`: 5 test không cần ROS đã qua: lưu/đọc lại cấu hình
  1/2/3 robot; từ chối IP trống, sai octet, trùng, loopback/multicast/broadcast;
  hỏi lại giá trị sai; chọn 2 không hỏi IP robot3; ngắt giữa chừng giữ nguyên
  file cũ; từ chối cấu hình `:11811;:11811;:11811;` từ lỗi `read`.
- `src/tb4_multi_bringup/test/test_fleet_view.py`: 3 test đã qua, gồm cấu hình RViz và kiểm thử tích hợp với ROS thật,
  TF/map/scan và action server Nav2 giả, domain 173, localhost. Đã qua kiểm tra
  chỉ mở interface/nhóm RViz của 1/2/3 robot, từ chối chọn robot ngoài phiên,
  khóa số robot sau khi node khởi động, prefix frame, giữ map chung, static TF của ba robot với subscriber mở muộn,
  map transient-local, scan robot3, pose/goal tới đúng từng namespace, ba goal
  hoạt động đồng thời, từ chối goal trùng/frame sai/robot ID sai và hủy riêng
  robot3 trong khi robot1/robot2 tiếp tục chạy.
- `tests/smoke_navigation.py robot3`: Nav2/AMCL thật với map và cảm biến giả,
  domain 170, localhost. AMCL, map server, controller, planner và BT navigator
  đều `active`; lập đường thành công (`status=4`, 8 pose), không phát map/TF ra
  topic gốc `/map` hoặc `/tf`. Không chạy hardware và không gửi goal điều khiển
  chuyển động.

Đã chạy launch với danh sách discovery sai: launch thoát trước khi mở bất kỳ
process RViz/fleet nào, kèm hướng dẫn chạy `bash scripts/laptop.sh configure`.
Build và kiểm tra cú pháp các khối Bash README cũng đã qua.

Chạy lại kiểm thử cấu hình laptop (không cần ROS):

```bash
cd "$HOME/turtlebot4_ws"
python3 tests/test_pc_config.py -v
```

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
