# Kiến trúc đa robot: PC navigation, Pi hardware

ROS 2 Humble / Ubuntu 22.04. Cùng repo trên PC và các Pi; giữ 1–3 robot,
namespace riêng và một RViz. [README](../README.md) là hướng dẫn chạy chính.

## Phân chia tiến trình

| Máy | Thành phần / script |
| --- | --- |
| PC | `pc-start.sh`: map server, AMCL, controller/planner/BT/behavior/velocity smoother riêng cho từng namespace, gateway socket và RViz |
| PC | `pc-mapping.sh`: SLAM + một RViz, thay cho phiên navigation |
| Pi N | `robot-start.sh robotN`: hardware, TF/description và watchdog |
| PC hoặc Pi | Teleop khi navigation đã dừng; khi mapping, dùng teleop trên Pi |

PC chỉ cần một bản đồ YAML/ảnh. Mỗi AMCL vẫn cần pose ban đầu đúng cho robot
của nó. Chưa có phối hợp tránh va chạm hoặc phân công nhiệm vụ giữa các robot.

## Đường dữ liệu

- Pi → PC: `/robotN/scan`, `/robotN/odom`, `/robotN/tf`, `/robotN/tf_static` và trạng thái Create 3 qua DDS.
- Trên PC: controller → `cmd_vel_nav` → velocity smoother → `pc_cmd_vel`.
  Behavior server cũng phát vào `pc_cmd_vel`.
- PC: `remote_velocity` đóng dấu thời gian, phát `/robotN/remote_cmd_vel` (`TwistStamped`).
- Pi: `remote_velocity` chế độ guard kiểm tra timestamp và số nguồn, phát `/robotN/cmd_vel` (`Twist`) cho republisher/Create 3.
- Teleop C++ với `remote:=true` phát thẳng `remote_cmd_vel` có timestamp.

Watchdog dùng QoS best effort, depth 1 cho remote command. Hết hạn 0,5 giây,
lệnh timestamp cũ/đảo thứ tự/không hữu hạn bị bỏ qua; hơn một publisher làm
đầu ra bằng 0. Đồng bộ giờ giữa PC và Pi là bắt buộc. Khi mạng phục hồi,
Nav2 có thể tiếp tục goal còn hoạt động; không có cơ chế bắt buộc re-arm.
E‑Stop và các bảo vệ Create 3 vẫn giữ nguyên, không tự bị mở bởi script.

Không phát trực tiếp vào `cmd_vel`, không chạy stock joystick song song.
Profile stock `turtlebot4` không có watchdog này; `robot-start.sh` dừng service
stock rồi chạy profile repo. Bringup/republisher chạy tay khác phải dừng riêng.

## Một chủ sở hữu goal cho RViz và socket

`SocketControl` kế thừa `FleetView`: các goal RViz/TCP dùng chung `busy`, goal
handle và cancel pending. TCP handler chỉ parse request rồi đưa vào hàng đợi;
callback trên ROS executor thực hiện thao tác, không chờ action đồng bộ trong
callback ROS. Tối đa 32 yêu cầu chờ, deadline 2 giây; request còn trong hàng
đợi đã hết hạn bị hủy, không được thực thi muộn.

Mỗi robot nhận một goal từ gateway. `goal` xác nhận gửi bất đồng bộ;
`status` phân biệt pending/accepted/kết quả cuối. `cancel` chỉ hủy goal do
cầu nối sở hữu, không hủy goal từ một client ROS ngoài hệ thống này.
`select` đổi đích nhận pose/goal RViz, không hủy goal robot đang chạy.
Mất kết nối TCP hoặc đóng menu không hủy goal đã gửi.

TCP JSON-lines trên loopback `127.0.0.1:8765`; truy cập từ máy khác qua SSH
tunnel. Xem protocol và client `.sh` trong README.

## TF và hiển thị

Frame thật trên từng robot vẫn là `map`, `odom`, `base_link`... trên topic TF
riêng namespace. Nav2 trên PC đọc cây gốc của đúng robot. Fleet view chỉ tạo
bản sao hiển thị trên `/tb4_view/tf`, `/tb4_view/tf_static`: prefix frame ngoài
`map` bằng `robotN/`, giúp một RViz hiển thị nhiều robot. Không đưa cây hiển
thị ngược vào buffer của Nav2.

## Khóa phiên và kiểm thử

`pc.sh` dùng `flock` để tránh các phiên nav/mapping/teleop/RViz chồng nhau trên
cùng PC/tài khoản. Khóa không điều phối nhiều PC hoặc terminal ROS chạy tay.
Pi kiểm tra số publisher remote command để dừng khi nhiều nguồn.

Kiểm thử phần mềm: [VALIDATION.md](VALIDATION.md). Chưa xác nhận navigation
đồng thời trên ba robot thật hoặc chất lượng Wi-Fi của môi trường triển khai.
