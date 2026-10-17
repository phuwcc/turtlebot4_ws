# TurtleBot 4 — hai robot, một workspace

Dùng chung repo trên **hai Pi**, ROS 2 Humble. Mỗi Pi chạy phần cứng và Nav2 của
robot mình; laptop chạy hai cửa sổ RViz.

**Build ngay tại `~/turtlebot4_ws`. Không có workspace con `tb4_multi_robot/`.**

```text
turtlebot4_ws/
├── src/
│   ├── turtlebot4/           # Source TurtleBot 4 có sẵn
│   ├── tb4_tools/            # Công cụ cho cấu hình một robot cũ
│   ├── tb4_multi_tools/      # Mapping origin và teleop C++ cho hai robot
│   └── tb4_multi_bringup/    # Launch và cấu hình từng robot
├── scripts/                 # Build, môi trường, chọn robot/chế độ
├── maps/                    # YAML + PGM dùng chung
├── docs/                    # Hướng dẫn chi tiết
└── build/, install/, log/   # Kết quả build chung tại gốc workspace
```

## 1. Cài và build — trên cả hai Pi

Nếu đã có repo, dùng thư mục hiện tại. Khi cài máy mới:

```bash
git clone https://github.com/phuwcc/turtlebot4_ws.git ~/turtlebot4_ws
```

Cài dependencies:

```bash
sudo apt install python3-colcon-common-extensions python3-yaml \
  ros-humble-turtlebot4-bringup ros-humble-turtlebot4-description \
  ros-humble-create3-republisher ros-humble-navigation2 \
  ros-humble-nav2-bringup ros-humble-slam-toolbox
```

Build hai package đa robot trong workspace hiện tại, một luồng để hạn chế RAM:

```bash
cd ~/turtlebot4_ws
bash scripts/build.sh
source scripts/env.sh
```

Các terminal tiếp theo đều bắt đầu bằng `cd ~/turtlebot4_ws`.
`scripts/robot.sh` tự source môi trường khi chạy.

## 2. Đặt định danh và mạng — làm một lần

| Cấu hình | Pi 1 | Pi 2 |
| --- | --- | --- |
| Robot ID / namespace | `robot1` / `/robot1` | `robot2` / `/robot2` |
| ROS domain | `0` | `0` |
| Discovery server ID | `0` | `1` |
| Discovery port | `11811` | `11811` |
| Create 3 namespace | `/robot1/_do_not_use` | `/robot2/_do_not_use` |

Dùng `turtlebot4-setup` trên từng Pi và Apply cấu hình tương ứng cho Create 3.
Làm theo [hướng dẫn mạng](docs/NETWORK.md) trước khi chạy tiếp; chỉ `export`
trong terminal chưa đổi được cấu hình dịch vụ/firmware.

Kiểm tra model trong [robot1.yaml](src/tb4_multi_bringup/config/robots/robot1.yaml)
và [robot2.yaml](src/tb4_multi_bringup/config/robots/robot2.yaml): mặc định
`standard`, đổi thành `lite` nếu đúng loại robot và build lại.

## 3. Bật phần cứng — Terminal A trên từng Pi

Nếu dịch vụ hiện tại đã phát đúng topic có namespace, dùng tiếp dịch vụ đó.
Nếu dùng hardware launch của repo, dừng bringup cũ trước:

```bash
sudo systemctl stop turtlebot4
pgrep -af '[/]rplidar_ros/rplidar_composition'
pgrep -af '[/]create3_republisher/create3_republisher'
```

Chỉ mở hardware khi hai lệnh `pgrep` không còn thấy driver cũ. Nếu driver do
dịch vụ riêng quản lý, dừng dịch vụ đó trước. Mỗi Pi chỉ chạy lệnh của robot mình:

```bash
# Pi 1 — giữ terminal này chạy
bash scripts/robot.sh robot1 hardware
```

```bash
# Pi 2 — giữ terminal này chạy
bash scripts/robot.sh robot2 hardware
```

Launch này chạy LiDAR, robot/base node, republisher và robot description;
không mở camera. Terminal khác trên Pi 1 kiểm tra:

```bash
bash scripts/robot.sh robot1 check
```

Phải có một publisher `/robot1/scan` và một `/robot1/odom`.
Trên Pi 2 thay `robot1` bằng `robot2`.

## 4. Quét map và teleop — dùng robot 1 trước

Terminal B trên Pi 1:

```bash
bash scripts/robot.sh robot1 mapping
```

Giữ robot đứng yên đến khi thấy `Start origin captured` và SLAM đăng ký LiDAR.
Vị trí/hướng thân robot lúc đó là gốc map `(0,0,0)`.

Terminal C trên Pi 1:

```bash
bash scripts/robot.sh robot1 teleop
```

Phím `i/,` tiến/lùi, `j/l` quay; `k` hoặc Space dừng, `Ctrl+C` thoát.
Mặc định 0,08 m/s và 0,3 rad/s; tự dừng sau 0,6 giây không nhận phím di chuyển.

Lưu map ở terminal khác khi robot đứng yên và SLAM vẫn chạy:

```bash
source scripts/env.sh
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f "$PWD/maps/site" --ros-args \
  -r __ns:=/robot1 -r map:=/robot1/map -p save_map_timeout:=15.0
```

Chép **cả `site.yaml` và `site.pgm`** vào `~/turtlebot4_ws/maps/` trên Pi 2.
Hai robot dùng cùng bản đồ; repo chưa ghép map từ hai phiên SLAM.

## 5. Chạy navigation — trên từng Pi

Dừng teleop bằng `k`, `Ctrl+C`; dừng mapping bằng `Ctrl+C`. Giữ hardware chạy.
Một lệnh navigation bên dưới mở cả map server, AMCL và Nav2; không cần mở
localization riêng.

Pi 1, nếu robot đã trở về đúng vị trí **và hướng** lúc bắt đầu quét:

```bash
bash scripts/robot.sh robot1 navigation map:="$PWD/maps/site.yaml" \
  initialize_pose:=true initial_x:=0.0 initial_y:=0.0 initial_yaw:=0.0
```

Pi 2, khởi tạo vị trí bằng RViz:

```bash
bash scripts/robot.sh robot2 navigation map:="$PWD/maps/site.yaml"
```

Trên RViz của robot 2, dùng **2D Pose Estimate** đặt đúng vị trí và hướng thực tế.
Nếu Pi 1 chưa ở gốc map, cũng chạy chế độ mặc định rồi chọn pose trên RViz.
Không đặt hai robot cùng `(0,0,0)` khi đứng khác chỗ. Chế độ
`initialize_pose:=true` bỏ qua click initial pose để tránh ghi đè vị trí đã khai báo.

## 6. Visualize và gửi goal — laptop

Laptop cần ROS 2 Humble, `ros-humble-rviz2`, `ros-humble-nav2-bringup`, cùng
repo đã build. Cấu hình discovery tới **cả hai Pi** theo
[NETWORK.md](docs/NETWORK.md), dựa trên [file môi trường mẫu](scripts/pc_env.example.sh).
Source file môi trường đã điền IP thật trong mỗi terminal laptop trước khi chạy:

```bash
cd ~/turtlebot4_ws
source /duong/dan/pc_env_da_sua.sh
bash scripts/robot.sh robot1 rviz
```

Terminal laptop thứ hai làm tương tự, đổi lệnh cuối thành:

```bash
bash scripts/robot.sh robot2 rviz
```

Kiểm tra scan khớp tường trên map, dừng teleop rồi dùng **Nav2 Goal** ở cửa sổ
đúng robot. Hai cây TF nằm trên `/robot1/tf` và `/robot2/tf`, nên dùng hai RViz.
Hiện mỗi robot điều hướng riêng; chưa có bộ phân công nhiệm vụ/điều phối giao thông.

## 7. Dừng phiên và kiểm tra lỗi

1. Hủy goal trên RViz; teleop nhấn `k` rồi `Ctrl+C`.
2. `Ctrl+C` ở terminal mapping/navigation, sau đó mới dừng hardware.
3. Trước khi chạy lại, dùng `bash scripts/robot.sh robot1 check` (hoặc `robot2`)
   để kiểm tra node và publisher còn lại.

Chỉ chạy một mapping hoặc navigation cho mỗi robot. Script có khóa ngăn hai chế
độ chạy trùng; không mở thêm SLAM/AMCL bằng lệnh launch cũ.

Build và các kiểm thử với cảm biến giả đã chạy; **chưa xác nhận navigation đồng
thời trên hai robot thật**. Kết quả và giới hạn ở [VALIDATION.md](docs/VALIDATION.md).

- [Hướng dẫn đa robot đầy đủ, lệnh kiểm thử](docs/MULTI_ROBOT.md)
- [Cấu hình mạng Pi / Create 3 / laptop](docs/NETWORK.md)
- [Hướng dẫn một robot cũ](docs/SINGLE_ROBOT.md) — dùng khi chạy cấu hình không namespace.
