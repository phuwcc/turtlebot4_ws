# Mạng cho hai TurtleBot 4 Humble

Hai robot cùng ROS domain 0, namespace khác nhau. Pi 1 dùng discovery server ID 0,
Pi 2 dùng ID 1; port 11811 có thể giống nhau vì nằm trên hai IP khác nhau.
Giữ Fast DDS nhất quán trên Pi, Create 3 và laptop.

Trên **từng Pi**, chạy `turtlebot4-setup`, vào ROS Setup/Bash Setup và Discovery
Server. Cấu hình rồi Apply theo [hướng dẫn Humble chính thức](https://turtlebot.github.io/turtlebot4-user-manual/setup/discovery_server.html).
Thao tác Apply có thể khởi động lại Create 3 và ghi lại dịch vụ bringup.

| Mục | Pi 1 | Pi 2 |
| --- | --- | --- |
| `ROBOT_NAMESPACE` | `/robot1` | `/robot2` |
| Domain | 0 | 0 |
| Onboard server ID | 0 | 1 |
| Onboard port | 11811 | 11811 |
| Create 3 namespace cần xác minh | `/robot1/_do_not_use` | `/robot2/_do_not_use` |

Đối chiếu namespace trong webserver Create 3 với YAML của repo. Republisher
dùng namespace tuyệt đối, tránh lỗi `//`. Nếu script setup trên image cũ cho kết
quả khác, xử lý cấu hình đó trước khi chạy hardware của repo. Không chỉ đổi
`ROS_DOMAIN_ID`/namespace ở terminal rồi coi robot đã đổi cấu hình.

Không tự chép cấu hình `/etc` của robot 1 sang robot 2: IP, discovery server ID
và namespace phải đúng từng robot. Repo không sửa các file đó khi build/chạy launch.

Laptop cần kết nối được tới IP Wi-Fi của cả hai Pi và đồng bộ đồng hồ. Cấu hình
PC với công cụ `configure_discovery.sh` trong tài liệu upstream, hoặc tạo file
môi trường ngoài Git dựa trên `scripts/pc_env.example.sh`. Thay hai placeholder
bằng IP thật, danh sách có thứ tự server ID 0 rồi ID 1.

Mỗi terminal laptop:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source /duong/dan/pc_env_da_sua.sh
ros2 node list --no-daemon --spin-time 10
```

Phải nhìn thấy cả `/robot1/...` và `/robot2/...` trước khi mở RViz/gửi goal.
Nếu đã chạy ROS CLI daemon với cấu hình cũ, dừng nó bằng `ros2 daemon stop`,
rồi chạy lại lệnh với môi trường mới. Kiểm tra `/robot1/scan` và `/robot2/scan`
có nguồn riêng, không có publisher LiDAR hoặc lệnh vận tốc chung ở `/scan`, `/cmd_vel`.

TF dùng `/robot1/tf`, `/robot1/tf_static` và bộ tương ứng của robot 2. Frame ID
bên trong vẫn là `map`, `odom`, `base_link`...; tên topic namespace không đổi
frame ID tự động. Hai RViz có remap riêng. Bộ điều phối sau này phải dùng buffer
TF theo từng robot hoặc bổ sung lớp tổng hợp frame có prefix.
