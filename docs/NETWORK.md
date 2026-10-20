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
file `$HOME/.config/turtlebot4/pc_env.sh` bằng khối lệnh hỏi IP ở
[README — cấu hình laptop](../README.md#laptop-network).
Chọn số robot từ 1 đến 3 trong D2. Danh sách discovery có thứ tự server ID
0, ID 1 rồi ID 2; chỉ lưu IP của robot đang dùng. Ví dụ chọn 2 thì danh sách
là `10.11.63.119:11811;10.11.52.137:11811;`, không thêm `:11811` trống cho robot3.
`pc_config.py` kiểm tra IPv4 đầy đủ, không nhận IP trống/trùng/sai định dạng.

Mỗi terminal laptop:

```bash
cd "$HOME/turtlebot4_ws"
source /opt/ros/humble/setup.bash
source "$HOME/turtlebot4_ws/install/setup.bash"
bash "$HOME/turtlebot4_ws/scripts/laptop.sh" check
```

Phải nhìn thấy namespace của tất cả robot đã chọn trước khi gửi goal.
Nếu đã chạy ROS CLI daemon với cấu hình cũ, dừng nó bằng `ros2 daemon stop`,
rồi chạy lại lệnh với môi trường mới. Kiểm tra `/robot1/scan`, `/robot2/scan` và `/robot3/scan`
có nguồn riêng, không có publisher LiDAR hoặc lệnh vận tốc chung ở `/scan`, `/cmd_vel`.

TF trên mỗi Pi vẫn dùng topic riêng `/robotN/tf`, `/robotN/tf_static` với frame
`map`, `odom`, `base_link`... Node `fleet_view` chạy trên laptop chuyển bản sao
của các cây đang dùng sang `/tb4_view/tf` và `/tb4_view/tf_static`, prefix mọi frame ngoài
map bằng `robotN/`. Map là frame chung vì các robot dùng cùng bản đồ đã lưu.
RViz chỉ nghe cây hiển thị này; không remap ngược dữ liệu đó vào Nav2 trên Pi.

Chạy `fleet_visualization.launch.py` theo README để mở một RViz. Dùng menu
laptop chọn `selected_robot` trước khi đặt pose/gửi goal. Không phát cả TF gốc
và TF hiển thị vào cùng một RViz buffer. Chế độ `visualization.launch.py robot_id:=robotN` cũ vẫn có thể dùng riêng để debug từng robot.

Số robot trong RViz lấy từ `TB4_ROBOT_COUNT`, truyền vào launch bằng
`robot_count:=N`. Launch tạo cấu hình RViz chỉ chứa nhóm robot1 đến robotN,
node chỉ tạo subscription/publisher/action client cho những robot đó. Đổi số
robot cần dừng và mở lại launch. Launch cũng kiểm tra `ROS_DISCOVERY_SERVER`
trước khi tạo tiến trình DDS, kể cả khi được gọi trực tiếp.
