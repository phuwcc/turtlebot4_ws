# Mạng cho ba TurtleBot 4 Humble

Ba robot cùng ROS domain 0, namespace khác nhau. Pi 1 dùng discovery server ID 0,
Pi 2 dùng ID 1, Pi 3 dùng ID 2; port 11811 giống nhau trên ba IP khác nhau.
Giữ Fast DDS nhất quán trên Pi, Create 3 và laptop.

Trên **từng Pi**, chạy `turtlebot4-setup`, vào ROS Setup/Bash Setup và Discovery
Server. Cấu hình rồi Apply theo [hướng dẫn Humble chính thức](https://turtlebot.github.io/turtlebot4-user-manual/setup/discovery_server.html).
Thao tác Apply có thể khởi động lại Create 3 và ghi lại dịch vụ bringup.

| Mục | Pi 1 | Pi 2 | Pi 3 |
| --- | --- | --- | --- |
| `ROBOT_NAMESPACE` | `/robot1` | `/robot2` | `/robot3` |
| Domain | 0 | 0 | 0 |
| Onboard server ID | 0 | 1 | 2 |
| Onboard port | 11811 | 11811 | 11811 |
| Create 3 namespace cần xác minh | `/robot1/_do_not_use` | `/robot2/_do_not_use` | `/robot3/_do_not_use` |

Đối chiếu namespace trong webserver Create 3 với YAML của repo. Republisher
dùng namespace tuyệt đối, tránh lỗi `//`. Nếu script setup trên image cũ cho kết
quả khác, xử lý cấu hình đó trước khi chạy hardware của repo. Không chỉ đổi
`ROS_DOMAIN_ID`/namespace ở terminal rồi coi robot đã đổi cấu hình.

Không tự chép cấu hình `/etc` của robot 1 sang robot 2: IP, discovery server ID
và namespace phải đúng từng robot. Repo không sửa các file đó khi build/chạy launch.

Laptop cần kết nối được tới IP Wi-Fi của cả ba Pi và đồng bộ đồng hồ. Tạo
file `$HOME/.config/turtlebot4/pc_env.sh` bằng script hỏi IP ở
[README — cấu hình laptop](../README.md#laptop-network).
Chọn số robot từ 1 đến 3 bằng `bash scripts/pc-configure.sh`. Danh sách discovery có thứ tự server ID
0, ID 1 rồi ID 2; chỉ lưu IP của robot đang dùng. Ví dụ chọn 2 thì danh sách
là `10.11.63.119:11811;10.11.52.137:11811;`, không thêm `:11811` trống cho robot3.
`pc_config.py` kiểm tra IPv4 đầy đủ, không nhận IP trống/trùng/sai định dạng.

Trên PC, script tự source ROS và cấu hình DDS:

```bash
bash scripts/pc-check.sh
```

Phải nhìn thấy namespace của tất cả robot đã chọn trước khi gửi goal.
Nếu đã chạy ROS CLI daemon với cấu hình cũ, dừng nó bằng `ros2 daemon stop`,
rồi chạy lại lệnh với môi trường mới. Kiểm tra `/robot1/scan`, `/robot2/scan` và `/robot3/scan`
có nguồn riêng, không có publisher LiDAR hoặc lệnh vận tốc chung ở `/scan`, `/cmd_vel`.

TF trên mỗi Pi vẫn dùng topic riêng `/robotN/tf`, `/robotN/tf_static` với frame
`map`, `odom`, `base_link`... Node `fleet_view` chạy trên laptop chuyển bản sao
của các cây đang dùng sang `/tb4_view/tf` và `/tb4_view/tf_static`, prefix mọi frame ngoài
map bằng `robotN/`. Map là frame chung vì các robot dùng cùng bản đồ đã lưu.
RViz chỉ nghe cây hiển thị này; không remap ngược dữ liệu đó vào Nav2 trên PC.

Chạy `pc-start.sh` theo README để mở Nav2, gateway và một RViz. Dùng menu
laptop chọn `selected_robot` trước khi đặt pose/gửi goal. Không phát cả TF gốc
và TF hiển thị vào cùng một RViz buffer. Chế độ `visualization.launch.py robot_id:=robotN` cũ vẫn có thể dùng riêng để debug từng robot.

Số robot trong RViz lấy từ `TB4_ROBOT_COUNT`, truyền vào launch bằng
`robot_count:=N`. Launch tạo cấu hình RViz chỉ chứa nhóm robot1 đến robotN,
node chỉ tạo subscription/publisher/action client cho những robot đó. Đổi số
robot cần dừng và mở lại launch. Launch cũng kiểm tra `ROS_DISCOVERY_SERVER`
trước khi tạo tiến trình DDS, kể cả khi được gọi trực tiếp.


## Navigation tập trung trên PC

`pc-start.sh` mở map server, AMCL và Nav2 cho từng namespace trên PC; Pi chỉ
chạy `robot-start.sh robotN`. Bản đồ chỉ cần nằm trên PC. Cần đồng bộ giờ
PC/Pi vì `remote_cmd_vel` là TwistStamped, watchdog trên Pi bỏ lệnh quá hạn
0,5 giây và phát `cmd_vel` bằng 0 khi mất luồng lệnh. TF/scan/odom vẫn qua DDS.

TCP nhiệm vụ chỉ nghe `127.0.0.1:8765` trên PC; dùng SSH tunnel nếu client ở
máy khác. Cổng này không truyền scan hoặc lệnh motor trực tiếp. Chi tiết protocol
và các script: [README](../README.md).

## Lỗi sequence size exceeds remaining buffer

`bash scripts/pc-check.sh` đọc DDS discovery dưới dạng byte trước khi chạy ROS CLI.
Nếu phát hiện GID 16 byte từ node `/robotN/_do_not_use/...`, trong khi Humble
dùng GID 24 byte, script báo đúng namespace và kết thúc với mã lỗi. Đây là
lệch định dạng ROS graph; tăng buffer hoặc đổi Fixed Frame không sửa được lỗi này.

Trên webserver **đế Create 3** của robot được báo, mở trang Update và kiểm tra
phiên bản firmware/ROS. PC và Pi của repo dùng Humble; đế cần firmware **H.x**,
ví dụ [H.2.6 chính thức](https://iroboteducation.github.io/create3_docs/releases/h_2_6/).
[Hướng dẫn cập nhật](https://iroboteducation.github.io/create3_docs/webserver/update/)
cho phép chọn bản ROS hoặc tải file firmware cụ thể lên. Không chọn mặc định
latest khi đang cần chuyển từ Iron sang Humble. Sau cập nhật, kiểm tra lại
namespace `/robotN/_do_not_use`, Fast DDS, domain 0 và discovery server của đế.
Khởi động lại bringup Pi và phiên PC rồi chạy lại `pc-check.sh`.

Nếu map server chưa active, RViz chưa có bản đồ để hiển thị. Khi map đã hiện,
chọn từng robot và đặt **2D Pose Estimate** đúng vị trí/hướng thực tế để AMCL
phát `map -> odom`. Không thêm static TF giả để làm mất cảnh báo `map`.

Ngày 2026-10-23, kiểm tra trực tiếp thấy `/robot2/_do_not_use` gửi GID 16 byte,
còn phần cứng `/robot1`, `/robot2` trên Pi gửi GID 24 byte. Chẩn đoán này
không xác nhận số phiên bản firmware cụ thể; cần đọc trên webserver của đế.


## Kiểm tra thực tế ngày 2026-10-30

Cả hai Create 3 của robot1/robot2 đã báo H.2.6/Humble; kiểm tra DDS không còn
phát hiện GID 16 byte trong các mẫu quan sát. Chẩn đoán ngày 23/10 ở trên là
kết quả lịch sử, không phải trạng thái firmware hiện tại.

Robot1 có RMW Profile Override thay thế profile mặc định nhưng không cấu hình
client discovery. Sau khi lưu bản sao, xóa override và restart ứng dụng ROS
trên đế, odom và TF đã hoạt động. Namespace/domain/discovery đúng trên trang
Configuration chưa đủ nếu một override riêng đang thay middleware profile.

Đã thử tiến ngắn và watchdog trên cả robot1/robot2, nhưng lifecycle Nav2 trên PC
vẫn có timeout qua mạng thật. Xem [VALIDATION.md](VALIDATION.md) trước khi gửi goal.
