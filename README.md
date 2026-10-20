# TurtleBot 4 — hai robot, một workspace

Dùng chung repo trên **hai Pi**, ROS 2 Humble. Mỗi Pi chạy phần cứng và Nav2 của
robot mình; laptop chạy hai cửa sổ RViz.

**Build ngay tại `~/turtlebot4_ws`. Không có workspace con `tb4_multi_robot/`.**

## Chọn đúng máy và terminal trước khi chạy

| Máy | Terminal | Chạy gì | Mục hướng dẫn |
| --- | --- | --- | --- |
| Pi 1 | A | Hardware robot1, giữ chạy | Mục 3 |
| Pi 2 | A | Hardware robot2, giữ chạy | Mục 3 |
| Pi 1 | B | Mapping robot1; dừng mapping trước khi chạy navigation | Mục 4 và 5 |
| Pi 1 | C | Teleop khi quét map | Mục 4 |
| Pi 1 | D | Lưu map trong lúc mapping còn chạy | Mục 4 |
| Pi 2 | B | Navigation robot2 sau khi đã chép map | Mục 5 |
| Laptop | Terminal cài đặt | Cài RViz, build và lưu IP hai Pi một lần | Mục 6.1 và 6.2 |
| Laptop | 1 | RViz robot1, giữ chạy | Mục 6.3 |
| Laptop | 2 | RViz robot2, giữ chạy | Mục 6.4 |
| Laptop | 3 | Kiểm tra kết nối cả hai robot | Mục 6.5 |

**Nếu Pi đã chạy hardware và navigation, trên laptop bắt đầu từ mục 6.**
Mỗi khối lệnh chạy trong terminal của đúng máy ghi phía trên. Terminal laptop
đã SSH vào Pi là terminal của Pi; mở terminal mới trên laptop để chạy RViz.

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
git clone https://github.com/phuwcc/turtlebot4_ws.git "$HOME/turtlebot4_ws"
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
cd "$HOME/turtlebot4_ws"
source /opt/ros/humble/setup.bash
bash "$HOME/turtlebot4_ws/scripts/build.sh"
source "$HOME/turtlebot4_ws/scripts/env.sh"
```

**Mỗi terminal có môi trường riêng**: phải chạy lại các dòng `source` trong
đúng terminal đó. Các khối lệnh bên dưới đã kèm đủ đường dẫn, có thể copy cả khối.
`$HOME/turtlebot4_ws` là workspace của tài khoản đang đăng nhập: trên Pi trong
ảnh là `/home/ubuntu/turtlebot4_ws`, trên laptop tài khoản `phuc` là
`/home/phuc/turtlebot4_ws`. Các lệnh dùng trực tiếp đường dẫn này, không cần thay tên tài khoản.
`scripts/robot.sh` cũng tự source môi trường Pi khi chạy.

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
cd "$HOME/turtlebot4_ws"
source /opt/ros/humble/setup.bash
source "$HOME/turtlebot4_ws/scripts/env.sh"
sudo systemctl stop turtlebot4
pgrep -af '[/]rplidar_ros/rplidar_composition'
pgrep -af '[/]create3_republisher/create3_republisher'
```

Chỉ mở hardware khi hai lệnh `pgrep` không còn thấy driver cũ. Nếu driver do
dịch vụ riêng quản lý, dừng dịch vụ đó trước. Mỗi Pi chỉ chạy lệnh của robot mình:

```bash
# Pi 1 — Terminal A, giữ terminal này chạy
cd "$HOME/turtlebot4_ws"
source /opt/ros/humble/setup.bash
source "$HOME/turtlebot4_ws/scripts/env.sh"
bash "$HOME/turtlebot4_ws/scripts/robot.sh" robot1 hardware
```

```bash
# Pi 2 — Terminal A, giữ terminal này chạy
cd "$HOME/turtlebot4_ws"
source /opt/ros/humble/setup.bash
source "$HOME/turtlebot4_ws/scripts/env.sh"
bash "$HOME/turtlebot4_ws/scripts/robot.sh" robot2 hardware
```

Launch này chạy LiDAR, robot/base node, republisher và robot description;
không mở camera. Terminal khác trên Pi 1 kiểm tra:

```bash
cd "$HOME/turtlebot4_ws"
source /opt/ros/humble/setup.bash
source "$HOME/turtlebot4_ws/scripts/env.sh"
bash "$HOME/turtlebot4_ws/scripts/robot.sh" robot1 check
```

Phải có một publisher `/robot1/scan` và một `/robot1/odom`.
Terminal kiểm tra trên Pi 2:

```bash
cd "$HOME/turtlebot4_ws"
source /opt/ros/humble/setup.bash
source "$HOME/turtlebot4_ws/scripts/env.sh"
bash "$HOME/turtlebot4_ws/scripts/robot.sh" robot2 check
```

## 4. Quét map và teleop — dùng robot 1 trước

Terminal B trên Pi 1:

```bash
cd "$HOME/turtlebot4_ws"
source /opt/ros/humble/setup.bash
source "$HOME/turtlebot4_ws/scripts/env.sh"
bash "$HOME/turtlebot4_ws/scripts/robot.sh" robot1 mapping
```

Giữ robot đứng yên đến khi thấy `Start origin captured` và SLAM đăng ký LiDAR.
Vị trí/hướng thân robot lúc đó là gốc map `(0,0,0)`.

Terminal C trên Pi 1:

```bash
cd "$HOME/turtlebot4_ws"
source /opt/ros/humble/setup.bash
source "$HOME/turtlebot4_ws/scripts/env.sh"
bash "$HOME/turtlebot4_ws/scripts/robot.sh" robot1 teleop
```

Phím `i/,` tiến/lùi, `j/l` quay; `k` hoặc Space dừng, `Ctrl+C` thoát.
Mặc định 0,08 m/s và 0,3 rad/s; tự dừng sau 0,6 giây không nhận phím di chuyển.

Lưu map ở terminal khác khi robot đứng yên và SLAM vẫn chạy:

```bash
cd "$HOME/turtlebot4_ws"
source /opt/ros/humble/setup.bash
source "$HOME/turtlebot4_ws/scripts/env.sh"
mkdir -p "$HOME/turtlebot4_ws/maps"
ros2 run nav2_map_server map_saver_cli -f "$HOME/turtlebot4_ws/maps/site" --ros-args \
  -r __ns:=/robot1 -r map:=/robot1/map -p save_map_timeout:=15.0
```

Chép **cả `site.yaml` và `site.pgm`** vào `~/turtlebot4_ws/maps/` trên Pi 2.
Hai robot dùng cùng bản đồ; repo chưa ghép map từ hai phiên SLAM.

## 5. Chạy navigation — trên từng Pi

Dừng teleop bằng `k`, `Ctrl+C`; dừng mapping bằng `Ctrl+C`. Giữ hardware chạy.
Một lệnh navigation bên dưới mở cả map server, AMCL và Nav2; không cần mở
localization riêng.

Pi 1 — Terminal B (sau khi dừng mapping), nếu robot đã trở về đúng vị trí
**và hướng** lúc bắt đầu quét:

```bash
cd "$HOME/turtlebot4_ws"
source /opt/ros/humble/setup.bash
source "$HOME/turtlebot4_ws/scripts/env.sh"
bash "$HOME/turtlebot4_ws/scripts/robot.sh" robot1 navigation \
  map:="$HOME/turtlebot4_ws/maps/site.yaml" \
  initialize_pose:=true initial_x:=0.0 initial_y:=0.0 initial_yaw:=0.0
```

Pi 2 — Terminal B, khởi tạo vị trí bằng RViz:

```bash
cd "$HOME/turtlebot4_ws"
source /opt/ros/humble/setup.bash
source "$HOME/turtlebot4_ws/scripts/env.sh"
bash "$HOME/turtlebot4_ws/scripts/robot.sh" robot2 navigation \
  map:="$HOME/turtlebot4_ws/maps/site.yaml"
```

Trên RViz của robot 2, dùng **2D Pose Estimate** đặt đúng vị trí và hướng thực tế.
Nếu Pi 1 chưa ở gốc map, cũng chạy chế độ mặc định rồi chọn pose trên RViz.
Không đặt hai robot cùng `(0,0,0)` khi đứng khác chỗ. Chế độ
`initialize_pose:=true` bỏ qua click initial pose để tránh ghi đè vị trí đã khai báo.

## 6. Lệnh trên laptop — cài một lần, sau đó mở từng terminal

Các terminal bên dưới chạy **trực tiếp trên laptop có màn hình**, sau khi mở
terminal mới; terminal đã SSH vào Pi vẫn đang chạy trên Pi. Hardware,
mapping và navigation ở các mục trên chạy trên Pi tương ứng.

### 6.1. Cài RViz và build trên laptop — làm một lần

Laptop phải có Ubuntu 22.04 và ROS 2 Humble tại `/opt/ros/humble`.
Nếu chưa có repo trên laptop, chạy:

```bash
git clone https://github.com/phuwcc/turtlebot4_ws.git "$HOME/turtlebot4_ws"
```

Nếu đã có thư mục đó, bỏ qua lệnh clone. Trong terminal laptop:

```bash
cd "$HOME/turtlebot4_ws"
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-yaml \
  ros-humble-rviz2 ros-humble-nav2-bringup ros-humble-navigation2 \
  ros-humble-slam-toolbox ros-humble-rmw-fastrtps-cpp
bash "$HOME/turtlebot4_ws/scripts/build.sh"
source "$HOME/turtlebot4_ws/install/setup.bash"
ros2 pkg prefix rviz2
ros2 pkg prefix tb4_multi_bringup
```

Nếu gặp `package 'rviz2' not found` như ảnh, chạy khối cài đặt này trên
**máy sẽ mở RViz**, rồi chạy lại terminal RViz bên dưới.

### 6.2. Lưu môi trường mạng laptop — làm một lần

Laptop phải kết nối được tới cả hai Pi. Hoàn tất cấu hình discovery trong
[NETWORK.md](docs/NETWORK.md): Pi 1 server ID `0`, Pi 2 server ID `1`,
cùng domain `0`, port `11811`. Trên từng Pi có thể dùng `hostname -I` để xem
địa chỉ; chọn IP Wi-Fi/LAN mà laptop truy cập được.

Copy cả khối sau vào **Bash trên laptop**. Khối này hỏi IP thật của từng Pi,
rồi lưu cấu hình vào `$HOME/.config/turtlebot4/pc_env.sh`; không cần sửa file
bằng tay. Thứ tự IP là Pi của `robot1` rồi Pi của `robot2`.

```bash
(
  read -r -p 'IP Wi-Fi/LAN của Pi robot1 (server ID 0): ' tb4_pi1
  read -r -p 'IP Wi-Fi/LAN của Pi robot2 (server ID 1): ' tb4_pi2
  if [[ ! "$tb4_pi1" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ||
        ! "$tb4_pi2" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo 'Cần nhập đủ hai địa chỉ IPv4; cấu hình chưa được ghi.' >&2
    exit 1
  fi
  mkdir -p "$HOME/.config/turtlebot4"
  cat > "$HOME/.config/turtlebot4/pc_env.sh" <<EOF
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER='${tb4_pi1}:11811;${tb4_pi2}:11811;'
export ROS_SUPER_CLIENT=True
export ROS_LOCALHOST_ONLY=0
EOF
)
```

Chỉ chạy lại bước này khi IP hai Pi thay đổi. File cấu hình nằm ngoài repo.

### 6.3. Laptop — Terminal 1: RViz robot1

```bash
cd "$HOME/turtlebot4_ws"
source /opt/ros/humble/setup.bash
source "$HOME/turtlebot4_ws/install/setup.bash"
source "$HOME/.config/turtlebot4/pc_env.sh"
bash "$HOME/turtlebot4_ws/scripts/robot.sh" robot1 rviz
```

Giữ terminal và cửa sổ RViz này chạy.

### 6.4. Laptop — Terminal 2: RViz robot2

```bash
cd "$HOME/turtlebot4_ws"
source /opt/ros/humble/setup.bash
source "$HOME/turtlebot4_ws/install/setup.bash"
source "$HOME/.config/turtlebot4/pc_env.sh"
bash "$HOME/turtlebot4_ws/scripts/robot.sh" robot2 rviz
```

Giữ terminal và cửa sổ RViz này chạy. Dùng **2D Pose Estimate** trong cửa sổ
đúng robot để đặt vị trí/hướng thực tế nếu chưa khởi tạo pose.

### 6.5. Laptop — Terminal 3: kiểm tra kết nối hai robot

```bash
cd "$HOME/turtlebot4_ws"
source /opt/ros/humble/setup.bash
source "$HOME/turtlebot4_ws/install/setup.bash"
source "$HOME/.config/turtlebot4/pc_env.sh"
ros2 daemon stop
ros2 node list --no-daemon --spin-time 10
bash "$HOME/turtlebot4_ws/scripts/robot.sh" robot1 check
bash "$HOME/turtlebot4_ws/scripts/robot.sh" robot2 check
```

Phải thấy node của cả `/robot1/...` và `/robot2/...`, mỗi robot có nguồn
`scan` và `odom`. Nếu báo `Invalid frame ID "odom"` như ảnh, kiểm tra hardware
trên Pi tương ứng đã chạy và có publisher odometry; kiểm tra TF và đồng hồ
Pi/laptop khi scan báo timestamp sớm hơn dữ liệu TF. Chỉ source môi trường
không tạo ra odometry hoặc sửa được thời gian TF.

Kiểm tra scan khớp tường trên map, dừng teleop rồi dùng **Nav2 Goal** ở cửa sổ
đúng robot. Hai cây TF nằm trên `/robot1/tf` và `/robot2/tf`, nên dùng hai RViz.
Hiện mỗi robot điều hướng riêng; chưa có bộ phân công nhiệm vụ/điều phối giao thông.

## 7. Dừng phiên và kiểm tra lỗi

1. Hủy goal trên RViz; teleop nhấn `k` rồi `Ctrl+C`.
2. `Ctrl+C` ở terminal mapping/navigation, sau đó mới dừng hardware.
3. Trước khi chạy lại, dùng khối lệnh kiểm tra tại mục 3 trên Pi tương ứng,
   hoặc mục 6.5 trên laptop để kiểm tra node và publisher còn lại.

Chỉ chạy một mapping hoặc navigation cho mỗi robot. Script có khóa ngăn hai chế
độ chạy trùng; không mở thêm SLAM/AMCL bằng lệnh launch cũ.

Build và các kiểm thử với cảm biến giả đã chạy; **chưa xác nhận navigation đồng
thời trên hai robot thật**. Kết quả và giới hạn ở [VALIDATION.md](docs/VALIDATION.md).

- [Hướng dẫn đa robot đầy đủ, lệnh kiểm thử](docs/MULTI_ROBOT.md)
- [Cấu hình mạng Pi / Create 3 / laptop](docs/NETWORK.md)
- [Hướng dẫn một robot cũ](docs/SINGLE_ROBOT.md) — dùng khi chạy cấu hình không namespace.
