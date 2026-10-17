# TurtleBot 4 — quét map, teleop và navigation

**Chạy hai robot:** xem [hướng dẫn đa robot](docs/MULTI_ROBOT.md).
Các package đa robot nằm ngay trong `src/` và build tại `~/turtlebot4_ws` bằng
`bash scripts/build.sh`; dùng chung `build/`, `install/`, `log/` của workspace này.

Dành cho **robot thật / ROS 2 Humble**, workspace `~/turtlebot4_ws`.
**Pi chạy SLAM, teleop và Nav2; laptop chạy RViz.**

Repo đã có các node cần thiết:

| File trong `turtlebot4_navigation/launch/` | Công dụng |
| --- | --- |
| `slam.launch.py` | Quét map bằng `slam_toolbox` |
| `localization.launch.py` | Đọc map đã lưu, định vị bằng AMCL |
| `nav2.launch.py` | Điều hướng tới goal |

Map tự tạo lưu ở `~/turtlebot4_ws/maps/`. Nếu cần ghép các launch thành một lệnh
sau này, đặt trong package riêng `src/tb4_bringup/` để dễ cập nhật repo gốc.

## 1. Chuẩn bị trên Pi

Workspace dùng package C++ riêng `src/tb4_tools/` cho gốc mapping và teleop.
Sau khi sửa C++, build riêng package này (một luồng để hạn chế RAM):

```bash
source /opt/ros/humble/setup.bash
cd ~/turtlebot4_ws
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build \
  --packages-select tb4_tools --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG'
```

**Mỗi terminal mới trên Pi** đều chạy:

```bash
source ~/turtlebot4_ws/robot_env.sh
```

Kiểm tra driver và dữ liệu:

```bash
systemctl is-active turtlebot4 discovery
ls -l /dev/RPLIDAR
ros2 topic info /scan --verbose --no-daemon --spin-time 5
ros2 topic info /odom --verbose --no-daemon --spin-time 5
```

Cần có **một publisher `/scan`** và publisher `/odom`. Nếu cần kiểm tra sâu hơn,
chạy từng lệnh sau, `Ctrl+C` trước khi chuyển lệnh:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link rplidar_link
```

`/scan` và `/odom` là đầu vào của SLAM, không cần mở SLAM mới có. LiDAR dùng
`auto_standby` có thể chờ subscriber; `topic hz /scan` cũng là một subscriber.
Không mở thêm driver LiDAR nếu dịch vụ robot đã chạy nó.

## 2. Quét map

### Terminal A — SLAM trên Pi

**Bước 1: Kiểm tra phiên cũ** sau khi source môi trường ở mục 1:

```bash
pgrep -af '[/]tb4_tools/start_relative_mapping'
pgrep -af '[r]os2 launch.*slam.launch.py'
pgrep -af '[/]slam_toolbox/.*slam_toolbox_node'
ros2 topic info /map --verbose --no-daemon --spin-time 5
```

- Chưa có tiến trình SLAM: chuyển sang bước 3. `/map` chưa tồn tại là bình thường.
- Đã có một phiên: tiếp tục dùng phiên đó, hoặc dừng theo bước 2 để quét lại.
- Có nhiều phiên: dừng các phiên thừa trước khi tiếp tục. Hai SLAM cùng phát
  `map → odom` có thể làm TF nhảy và map lệch dù robot đứng yên.

**Bước 2: Tắt phiên cũ nếu muốn mở lại.** Lưu map theo mục 3 trước khi dừng nếu
muốn giữ kết quả. Có terminal cũ thì nhấn `Ctrl+C`. Nếu phiên chạy nền:

```bash
# Xem PID của launcher C++ (hoặc launch Python cũ nếu còn):
pgrep -af '[/]tb4_tools/start_relative_mapping'
pgrep -af '[r]os2 launch.*slam.launch.py'
```

Chọn đúng **PID của `start_relative_mapping`** từ kết quả trên; node này sẽ dừng
SLAM con khi nhận SIGINT. Với phiên kiểu cũ, chọn PID `ros2 launch`.
Thay `12345` bằng PID thực tế:

```bash
TB4_SLAM_PID=12345
ps -p "$TB4_SLAM_PID" -o pid,ppid,args
# Đọc kết quả ps, xác nhận đúng phiên SLAM rồi mới chạy:
kill -INT "$TB4_SLAM_PID"
```

Chờ vài giây, kiểm tra lại:

```bash
pgrep -af '[/]tb4_tools/start_relative_mapping'
pgrep -af '[r]os2 launch.*slam.launch.py'
pgrep -af '[/]slam_toolbox/.*slam_toolbox_node'
```

Không còn dòng nào nghĩa là SLAM cục bộ đã dừng. Nếu chỉ còn node không có
launch, xem PID/PPID bằng `ps -p PID_THUC_TE -o pid,ppid,args`, rồi gửi SIGINT
cho đúng node đó. Không dùng PID từ log cũ hoặc kill hàng loạt mọi tiến trình ROS.

**Bước 3: Mở đúng một phiên SLAM:**

```bash
cd ~/turtlebot4_ws
bash run_slam.sh
```

Script có kiểm tra tiến trình SLAM cục bộ và từ chối mở trùng. Giữ terminal này
mở. **Giữ robot đứng yên khi khởi động:** script chờ odometry/TF mới và robot
đứng yên ít nhất một giây, rồi lấy vị trí/hướng đó làm gốc mapping `(x=0, y=0,
yaw=0)`. Chờ log `Start origin captured` và SLAM đăng ký LiDAR rồi mới teleop.

Phần này nằm ở `src/tb4_tools/src/start_relative_mapping.cpp`. Node C++ khởi chạy
trực tiếp executable C++ của SLAM, không dùng helper Python cũ.
Chuỗi TF khi quét là `map → mapping_odom → odom → base_link → … → rplidar_link`.
Frame `mapping_odom` cố định theo pose bắt đầu; odometry Create 3 được giữ nguyên.
Không cần chọn **2D Pose Estimate** khi quét map mới. SLAM vẫn có thể hiệu chỉnh
pose để khớp scan/khép vòng; việc đặt gốc không khóa cứng vị trí robot trong map.

Mỗi lần chạy lại sẽ chụp gốc mới và tạo map mới; map YAML/PGM đã lưu không tự được
nạp để tiếp tục SLAM. Vị trí gốc là tâm frame `base_link`, không phải tâm LiDAR.
Pose đã chụp được ghi tại `log/mapping_start_origin.json`.

### Terminal B — RViz trên laptop

Laptop cần Ubuntu 22.04 và ROS 2 Humble. Cài một lần:

```bash
sudo apt update
sudo apt install ros-humble-rviz2 ros-humble-nav2-bringup \
  ros-humble-turtlebot4-description ros-humble-rmw-fastrtps-cpp
```

Chạy khối sau **trên laptop**, không phải terminal SSH. Đổi IP nếu Pi đổi địa chỉ
(xem `hostname -I` trên Pi):

```bash
source /opt/ros/humble/setup.bash
export TB4_PI_IP=10.11.62.51
mkdir -p ~/.config/tb4
scp ubuntu@"${TB4_PI_IP}":/etc/turtlebot4/fastdds_rpi.xml ~/.config/tb4/fastdds.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_DISCOVERY_SERVER="${TB4_PI_IP}:11811;"
export ROS_SUPER_CLIENT=True
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.config/tb4/fastdds.xml"
export ROS_LOCALHOST_ONLY=0
ros2 node list --no-daemon --spin-time 10
ros2 launch nav2_bringup rviz_launch.py use_namespace:=false
```

Laptop và Pi phải liên lạc được qua mạng, cùng domain/RMW và đồng bộ giờ.
Trong RViz đặt:

| Mục | Giá trị |
| --- | --- |
| Global Options → Fixed Frame | `map` |
| Map | `/map`, Reliable, Transient Local |
| LaserScan | `/scan`, Best Effort |
| TF | Chỉ bật `map`, `mapping_odom`, `base_link`, `rplidar_link` để dễ quan sát |
| RobotModel (tùy chọn) | Bật; Description Topic `/robot_description`, Transient Local |

Pi hiện chưa cài RViz. Nếu có desktop/màn hình, có thể cài `ros-humble-rviz2`,
source môi trường Pi rồi chạy cùng lệnh RViz ngay trên Pi.

### Terminal C — teleop trên Pi

Dùng terminal tương tác; nếu SSH thì dùng `ssh -t ubuntu@IP_CUA_PI`.
Source môi trường mục 1, sau đó chạy:

```bash
ros2 run tb4_tools teleop_keyboard --ros-args \
  -p speed:=0.08 -p turn:=0.3 -p key_timeout:=0.6 \
  -r cmd_vel:=/cmd_vel
```

Click vào terminal teleop để nhận phím. Tốc độ ban đầu: **0,08 m/s**, quay **0,3 rad/s**.

| Phím | Điều khiển |
| --- | --- |
| `i` / `,` | Tiến / lùi |
| `j` / `l` | Quay trái / phải |
| `u` / `o` | Tiến và rẽ trái / phải |
| `m` / `.` | Lùi và rẽ |
| `k` hoặc Space | Dừng |
| `q` / `z` | Tăng / giảm cả hai tốc độ 10% |
| `w` / `x` | Tăng / giảm tốc độ tiến 10% |
| `e` / `c` | Tăng / giảm tốc độ quay 10% |
| `Ctrl+C` | Dừng và thoát teleop |

Teleop C++ phát lệnh ở 20 Hz, tự gửi vận tốc 0 sau **0,6 giây không nhận phím
di chuyển**. Giữ phím để hệ điều hành lặp phím; `k` hoặc Space dừng ngay.
Đóng teleop Python cũ trước khi mở teleop C++ để tránh hai nguồn lệnh.
Đi chậm, quay chậm, đi qua các vùng cần quét và quay lại vùng đã đi để khép vòng.
Robot phải ra khỏi dock và được phép chạy động cơ. Chỉ dùng một nguồn điều khiển
mỗi lúc; dừng teleop trước khi gửi goal Nav2.

## 3. Lưu map — terminal khác trên Pi

Nhấn `k` để robot đứng yên, giữ SLAM chạy. Source môi trường mục 1 rồi lưu:

```bash
mkdir -p ~/turtlebot4_ws/maps
ros2 run nav2_map_server map_saver_cli \
  -f "$HOME/turtlebot4_ws/maps/my_room" \
  --ros-args -p use_sim_time:=false -p save_map_timeout:=15.0
ls -lh ~/turtlebot4_ws/maps/my_room.*
```

Cần có `my_room.yaml` và `my_room.pgm`, log báo `Map saved successfully`.
Đổi tên `my_room` để tránh ghi đè map cũ; giữ cả hai file khi copy.
Đây là occupancy map cho AMCL/Nav2, không chứa pose graph để tiếp tục phiên SLAM cũ.

## 4. Navigation trên map đã lưu

**Trước khi chuyển:** dừng teleop bằng `k` rồi `Ctrl+C`, lưu map, sau đó dừng SLAM
bằng `Ctrl+C` hoặc cách tìm PID ngay ở mục Terminal A. Giữ driver robot chạy.

### Terminal A — localization trên Pi

Source môi trường mục 1:

```bash
cd ~/turtlebot4_ws
bash run_localization.sh
```

Lệnh trên tự khởi tạo AMCL ở `(0,0,0)`: dùng khi map được quét bằng `run_slam.sh`
mới và robot đã được đặt lại **đúng vị trí lẫn hướng thân robot lúc bắt đầu quét**.
Ở chế độ này, AMCL bỏ qua topic `/initialpose` của **2D Pose Estimate** trong RViz
để tránh click nhầm ghi đè pose đang đúng. AMCL vẫn cập nhật vị trí khi robot chạy;
TF không bị khóa cứng. `initial_yaw` tính bằng radian,
không phải góc trục LiDAR; map YAML `origin` là góc dưới ảnh, không phải pose robot.

Nếu robot ở vị trí khác hoặc map được quét theo gốc khác: dùng đúng tọa độ
`initial_x`, `initial_y`, `initial_yaw`, hoặc đặt `set_initial_pose:=false` rồi
dùng **2D Pose Estimate** trong RViz. Kéo mũi tên theo hướng tiến của **thân robot**.
Khi chọn tay, vị trí click là vị trí robot, hướng kéo mũi tên là yaw; kéo ngược
hướng làm pose xoay khoảng 180° và scan lệch theo. Muốn chuyển từ tự động sang
chọn tay, dừng localization rồi chạy lại với `set_initial_pose:=false`.
Kiểm tra scan khớp tường trước khi gửi goal. Không bỏ `map:=...` để tránh dùng map mẫu.

Terminal A chỉ mở `/map_server` và `/amcl`. Chạy Terminal B bên dưới mới có
controller, planner và navigator. Giữ cả hai terminal mở.

### Terminal B — Nav2 trên Pi

Source môi trường mục 1:

```bash
cd ~/turtlebot4_ws
bash run_navigation.sh
```

Script dùng `src/tb4_tools/config/nav2_pi.yaml`: controller 10 Hz, giảm số mẫu
vận tốc DWB, tắt debug trajectory và point cloud voxel không dùng. Giới hạn tốc
độ, tránh vật cản và kích thước robot giữ như cấu hình gốc. Nav2 chạy các component
C++ riêng theo mặc định. Có thể thử composition bằng
`bash run_navigation.sh use_composition:=True` để gom vào một `nav2_container`;
chế độ này chưa được xác nhận điều hướng tới goal trên Pi này.
Các node SLAM/Nav2 vốn
đã là C++; file launch `.py` chỉ cấu hình lúc khởi động, không phải vòng điều khiển.

Kiểm tra ở terminal khác:

```bash
ros2 lifecycle get /amcl --no-daemon --spin-time 5
ros2 lifecycle get /controller_server --no-daemon --spin-time 5
ros2 lifecycle get /planner_server --no-daemon --spin-time 5
ros2 lifecycle get /bt_navigator --no-daemon --spin-time 5
```

Các node cần `active`. Trong RViz chọn **Nav2 Goal**, click điểm đích trong vùng
trống đã biết và kéo hướng cuối. Nếu goal vào hàng chờ, dùng **Start Navigation**
trong panel Navigation 2. Robot di chuyển khi gửi goal; hủy bằng nút Cancel của panel.
Nếu `Node not found`, kiểm tra đã source mục 1, viết đúng `/amcl` (không phải
`/amcll`), và cả Terminal A/B vẫn đang chạy. `ros2 node list --no-daemon --spin-time 5`
giúp đối chiếu node thực tế; chỉ Terminal A thì thiếu các node Nav2 là bình thường.

**Chỉ một nguồn TF `map → odom`:** dùng AMCL với map đã lưu, hoặc dùng SLAM với
map đang xây dựng. Không chạy AMCL và SLAM cùng lúc. Muốn navigation khi đang
quét map: giữ SLAM, mở Nav2, bỏ bước localization và dừng teleop trước khi gửi goal.

## 5. Dừng phiên

- Teleop: `k` rồi `Ctrl+C`.
- SLAM: lưu map, rồi `Ctrl+C`; phiên nền xem ngay mục Terminal A.
- Nav2/localization: hủy goal, rồi `Ctrl+C` ở terminal tương ứng.
- RViz: đóng cửa sổ hoặc `Ctrl+C` trên laptop.

Nếu Nav2/localization chạy nền, tìm launch rồi kiểm tra PID và dùng `kill -INT`
như cách dừng SLAM:

```bash
pgrep -af '[r]os2 launch.*(nav2.launch.py|localization.launch.py|rviz_launch.py)'
```

Không dừng dịch vụ `turtlebot4`/`discovery` chỉ để đổi chế độ mapping/navigation.
`ros2 daemon stop` chỉ dừng CLI daemon, không dừng SLAM/Nav2.

## 6. Lỗi thường gặp

| Hiện tượng | Xử lý |
| --- | --- |
| `Unknown topic` nhưng node vẫn có | Source lại môi trường; dùng `--no-daemon --spin-time 5`. Có thể chạy `ros2 daemon stop` rồi `ros2 daemon start` sau khi export `ROS_SUPER_CLIENT=True`. |
| Map/TF nhảy khi đứng yên | Kiểm tra chỉ một SLAM, một driver LiDAR; không chạy AMCL cùng SLAM. |
| RViz báo `queue is full` | Kiểm tra TF và timestamp trên cả Pi/laptop; dùng `ros2 run tf2_ros tf2_echo map rplidar_link`. Kiểm tra đồng hồ bằng `timedatectl status`. |
| Trục LiDAR vuông góc thân robot | URDF gốc đặt yaw LiDAR 90°; đây là góc thiết kế. Đừng sửa URDF chỉ để các trục cùng hướng. |
| Không có `/odom` | Kiểm tra Create 3/republisher: `journalctl -u turtlebot4 -b --no-pager`. Máy từng gặp tên topic lỗi `//_do_not_use/battery_state`. |
| Không có `/scan` | Kiểm tra USB/nguồn, `/dev/RPLIDAR` và driver; tránh hai driver cùng mở cổng. |
| Teleop không nhận phím | Dùng TTY/SSH `-t` và focus đúng terminal. |
| Teleop có lệnh nhưng robot không chạy | Kiểm tra dock, trạng thái dừng động cơ/hazard và nguồn điều khiển khác. |
| Lag khi chạy navigation | Dùng `run_navigation.sh`, chỉ một phiên AMCL/Nav2, đóng teleop trước khi gửi goal. Kiểm tra nhiệt độ Pi, nguồn và CPU/RAM; đổi helper sang C++ không tự sửa lỗi TF, mạng hoặc map sai. |

<details>
<summary>Cài dependency hoặc build lại khi cần</summary>

Pi đã có repo nhánh `humble` và đã build. Chỉ dùng phần này khi cài lại hoặc đổi mã nguồn.

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-teleop-twist-keyboard
```

Nếu chưa có repo:

```bash
mkdir -p ~/turtlebot4_ws/src
cd ~/turtlebot4_ws/src
git clone --branch humble --single-branch https://github.com/turtlebot/turtlebot4.git
```

Build một luồng, giảm tối ưu C++ để hạn chế RAM trên Pi:

```bash
source /opt/ros/humble/setup.bash
cd ~/turtlebot4_ws
export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS=-j1
colcon build --symlink-install --executor sequential \
  --allow-overriding turtlebot4_description turtlebot4_msgs turtlebot4_node \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  '-DCMAKE_CXX_FLAGS_RELEASE=-O0 -DNDEBUG' -DBUILD_TESTING=OFF
source install/setup.bash
```

</details>

Đã build và kiểm tra C++: gốc mapping/TF, phát map, chống mở trùng SLAM;
teleop tiến/quay/dừng theo timeout được thử trên topic riêng, không điều khiển robot.
Các kiểm tra toán học của gốc map và phím teleop đã qua.

**Navigation chưa xác nhận tới goal:** phiên thử gặp timeout phản hồi lifecycle
Fast DDS và scan/TF trễ; chuyển C++ không tự giải quyết các lỗi này. Pi đã lên
khoảng 82°C, chưa có quạt, và có ghi nhận giảm xung trong lịch sử. Cần bổ sung
tản nhiệt rồi kiểm tra lại lifecycle, độ trễ TF và lập đường trước khi chạy goal.
File launch `.py` vẫn giữ vì chỉ dùng để cấu hình; SLAM, AMCL và Nav2 vốn là C++.
Cấu hình SLAM hiện có `min_laser_range: 0.2`.
