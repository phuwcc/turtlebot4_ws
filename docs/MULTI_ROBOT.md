# TurtleBot 4 — repo chung cho hai robot

ROS 2 Humble / Ubuntu 22.04. **Cùng code trên hai Pi**, chọn `robot1` hoặc
`robot2` khi chạy. Mapping và teleop là C++; AMCL/Nav2 dùng các node C++ có sẵn.
Python chỉ dùng cho launch/cấu hình và kiểm thử.

| | Pi robot 1 | Pi robot 2 |
| --- | --- | --- |
| Namespace ứng dụng | `/robot1` | `/robot2` |
| Namespace Create 3 | `/robot1/_do_not_use` | `/robot2/_do_not_use` |
| ROS domain | `0` | `0` |
| Discovery server ID | `0` | `1` |
| Port trên mỗi Pi | `11811` | `11811` |

Repo chưa tự thay cấu hình mạng/Create 3 hoặc dịch vụ hệ thống. Bản robot đơn
đang chạy vẫn giữ nguyên. Cần thực hiện bước cấu hình mỗi Pi dưới đây trước khi
dùng phần cứng thật với namespace mới.

## 1. Cài và build trên cả hai Pi

Code dùng chung nằm trực tiếp trong `src/tb4_multi_tools` và
`src/tb4_multi_bringup` của repository
[`phuwcc/turtlebot4_ws`](https://github.com/phuwcc/turtlebot4_ws).
Trên Pi thứ hai hoặc máy chưa có code:

```bash
git clone https://github.com/phuwcc/turtlebot4_ws.git ~/turtlebot4_ws
cd ~/turtlebot4_ws
```

Nếu đã clone repository, cập nhật từ thư mục gốc bằng `git pull --ff-only`,
rồi build ngay tại `~/turtlebot4_ws`. Đây là **một workspace duy nhất** cho cả
robot đơn và hai robot; các package đa robot dùng các dependency TurtleBot/Nav2
đã cài trên Pi.

Pi cần ROS 2 Humble và các gói TurtleBot 4 đã được cấu hình cho robot thật:

```bash
sudo apt install python3-colcon-common-extensions python3-yaml \
  ros-humble-turtlebot4-bringup ros-humble-turtlebot4-description \
  ros-humble-create3-republisher ros-humble-navigation2 \
  ros-humble-nav2-bringup ros-humble-slam-toolbox
```

Tại thư mục repo:

```bash
bash scripts/build.sh
source scripts/env.sh
```

Script build một luồng, chọn hai package đa robot trong `src/`, dùng chung
`~/turtlebot4_ws/build`, `install`, `log`. Lệnh colcon tương đương:

```bash
source /opt/ros/humble/setup.bash
cd ~/turtlebot4_ws
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build \
  --packages-select tb4_multi_tools tb4_multi_bringup --symlink-install \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release \
  '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG'
source install/setup.bash
```

Không có workspace con `tb4_multi_robot/`; các file build không được commit.

## 2. Cấu hình mạng và định danh — một lần trên mỗi robot

Xem [NETWORK.md](NETWORK.md). Dùng `turtlebot4-setup` để đặt namespace,
domain và discovery server ID theo bảng đầu trang, đồng bộ với Create 3.
Sau khi Apply, xác minh namespace Create 3 đúng; `export` ở terminal không đủ
để thay cấu hình firmware hoặc dịch vụ đang chạy.

Các thông số model/frame nằm trong:

```text
src/tb4_multi_bringup/config/robots/robot1.yaml
src/tb4_multi_bringup/config/robots/robot2.yaml
```

Mặc định model `standard`. Sửa thành `lite` nếu đúng phần cứng của bạn và build lại.
Không đổi frame chỉ để các trục nhìn cùng hướng trên RViz.

## 3. Chạy phần cứng — Terminal A trên từng Pi

Repo cung cấp bringup LiDAR 2D gồm robot node, base node (Standard), một
republisher, một LiDAR và robot description. Không mở camera/joystick.

**Chọn một bringup:** nếu dùng bringup của repo, dừng dịch vụ cũ trước:

```bash
sudo systemctl stop turtlebot4
pgrep -af '[/]rplidar_ros/rplidar_composition'
pgrep -af '[/]create3_republisher/create3_republisher'
```

Hai lệnh `pgrep` phải không còn tiến trình cũ. Nếu có dịch vụ độc lập tự mở lại
driver, dừng dịch vụ đó; không để hai tiến trình cùng giữ USB.

Trên Pi 1:

```bash
bash scripts/robot.sh robot1 hardware
```

Trên Pi 2:

```bash
bash scripts/robot.sh robot2 hardware
```

Giữ Terminal A chạy. Nếu đã có bringup chuẩn chạy đúng namespace và đủ dữ liệu,
tiếp tục dùng nó, bỏ qua bước mở hardware của repo. Wrapper từ chối mở thêm khi
phát hiện driver đang chạy trên cùng Pi. Script không tự disable dịch vụ cũ.

Terminal khác, chọn đúng ID của Pi:

```bash
bash scripts/robot.sh robot1 check
```

Phải có đúng một publisher `/robot1/scan`, một `/robot1/odom`; Pi 2 tương tự.
LiDAR dùng `/dev/RPLIDAR`, không dùng cứng `ttyUSB0`.

## 4. Quét map — dùng một robot trước

Terminal B trên Pi 1:

```bash
bash scripts/robot.sh robot1 mapping
```

Giữ robot đứng yên đến khi thấy `Start origin captured` và SLAM đăng ký LiDAR.
Gốc map là vị trí/hướng thân robot tại thời điểm bắt đầu.

Terminal C có TTY trên Pi 1:

```bash
bash scripts/robot.sh robot1 teleop
```

Phím `i/,` tiến/lùi, `j/l` quay, `u/o/m/.` đi vòng cung, `k` hoặc Space dừng,
`Ctrl+C` thoát. Tốc độ mặc định 0,08 m/s và 0,3 rad/s. Tự dừng sau 0,6 giây
không nhận phím di chuyển. Giữ phím để lặp lệnh.

Lưu map khi robot đứng yên, SLAM vẫn chạy:

```bash
source scripts/env.sh
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f "$PWD/maps/site" --ros-args \
  -r __ns:=/robot1 -r map:=/robot1/map -p save_map_timeout:=15.0
```

Copy **cả YAML và ảnh PGM** sang repo trên Pi 2, giữ cùng nội dung bản đồ.
`maps/my_room.*` đi kèm là bản đồ từ workspace cũ, chỉ phù hợp đúng môi trường đó.
Repo không hợp nhất map nếu hai robot SLAM cùng lúc.

## 5. Navigation — trên mỗi Pi

Dừng teleop bằng `k`, `Ctrl+C`, rồi dừng mapping bằng `Ctrl+C`. Giữ hardware.

Ví dụ Pi 1, robot thực sự ở gốc bắt đầu quét:

```bash
bash scripts/robot.sh robot1 navigation map:="$PWD/maps/site.yaml" \
  initialize_pose:=true initial_x:=0.0 initial_y:=0.0 initial_yaw:=0.0
```

Pi 2 nếu chưa biết pose chính xác:

```bash
bash scripts/robot.sh robot2 navigation map:="$PWD/maps/site.yaml"
```

Chọn **2D Pose Estimate** trên RViz của robot 2, đặt đúng vị trí và hướng thân
robot trong cùng map. Sau đó kiểm tra scan khớp tường rồi mới gửi Nav2 Goal.
Có thể dùng `initialize_pose:=true` cho Pi 2 nếu cung cấp đủ x/y/yaw đúng thực tế.
**Không đặt cả hai robot về `(0,0,0)` khi chúng đứng khác chỗ.**

Một launch navigation mở cả map server, AMCL và Nav2 cho robot đó. Chế độ tự
khởi tạo pose bỏ qua click `/initialpose` để tránh ghi đè nhầm; chế độ mặc định
cho phép chọn pose trên RViz. Các script khóa mapping/navigation trên từng robot
để tránh cùng phát TF định vị; chạy trực tiếp launch không có khóa shell này.

## 6. RViz trên laptop

Cài ROS 2 Humble, `ros-humble-rviz2`, `ros-humble-nav2-bringup` và build repo.
Làm bước môi trường PC trong [NETWORK.md](NETWORK.md).

Mở hai terminal laptop, mỗi terminal ở thư mục repo:

```bash
bash scripts/robot.sh robot1 rviz
```

```bash
bash scripts/robot.sh robot2 rviz
```

Mỗi cửa sổ nhận TF, scan, map và gửi goal tới đúng robot. Dùng hai cửa sổ vì hai
cây TF hiện được cô lập theo topic; chưa tổng hợp thành một cây TF fleet chung.
Dừng teleop trước khi gửi Nav2 Goal. Mỗi Pi giữ Nav2 riêng; repo hiện chưa có
phân công nhiệm vụ hoặc giữ quyền qua cửa/hành lang hẹp cho cả đội.

## 7. Dừng và kiểm thử

- Hủy goal trên RViz trước khi dừng Nav2.
- Teleop: `k`, rồi `Ctrl+C`.
- Mapping/navigation: `Ctrl+C`; mapping helper dừng SLAM con cùng nó.
- Hardware: `Ctrl+C` sau khi các ứng dụng đã dừng.
- Khi trở về cấu hình một robot cũ, cần khôi phục namespace/network tương ứng;
  chỉ khởi động dịch vụ cũ là chưa đủ nếu đã đổi namespace Create 3.

Kiểm thử không điều khiển robot thật:

```bash
source install/setup.bash
ctest --test-dir build/tb4_multi_tools --output-on-failure
python3 tests/smoke_two_robots.py
python3 tests/smoke_navigation.py
python3 tests/smoke_navigation.py robot2
```

Smoke test dùng domain **167**, localhost, cảm biến giả và hai SLAM C++ thật;
kiểm tra hai map/TF tách biệt và gốc riêng. Xem [VALIDATION.md](VALIDATION.md)
để biết phạm vi đã kiểm chứng. Không xem test này là đã chạy hai robot vật lý.

## Tổ chức source

```text
turtlebot4_ws/
├── src/
│   ├── turtlebot4/         # Repo upstream hiện có
│   ├── tb4_tools/          # Công cụ robot đơn hiện có
│   ├── tb4_multi_tools/    # Mapping origin + teleop C++ đa robot
│   └── tb4_multi_bringup/  # Launch và cấu hình robot1/robot2
├── scripts/               # Build, môi trường và chọn robot/chế độ
├── maps/                  # Map chung
├── tests/                 # Kiểm tra namespace bằng cảm biến giả
├── build/
├── install/
└── log/
```

Hướng dẫn upstream: [multi-robot TurtleBot 4](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/multiple_robots.html),
[Discovery Server](https://turtlebot.github.io/turtlebot4-user-manual/setup/discovery_server.html).
