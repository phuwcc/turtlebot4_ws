# TurtleBot 4 — chọn 1–3 robot, 1 RViz, mỗi terminal một khối lệnh

**Mỗi ô lệnh dưới đây là toàn bộ lệnh cho một terminal mới. Copy cả ô và chạy.**
Các dòng `cd` và `source` đã có sẵn trong từng ô. Dùng Bash trên Ubuntu 22.04,
ROS 2 Humble. `$HOME` tự lấy đúng tài khoản: `/home/phuc` trên laptop nếu đăng nhập
bằng `phuc`, `/home/ubuntu` trên Pi nếu đăng nhập bằng `ubuntu`.

**Máy đã cài và có map:** chạy các terminal Pi ở phần B, sau đó các terminal
laptop ở phần A. **Máy mới:** làm phần D một lần trước. **Đã dùng bản trước:** build lại
trên laptop và các Pi đang dùng, rồi chạy D2 để chọn số robot và lưu IP. **Chưa có map:** làm phần C.

## A. LAPTOP — lệnh chạy trực tiếp trên laptop

Mở terminal trên laptop có màn hình. Terminal đã SSH vào Pi là terminal của Pi.
Cấu hình mạng laptop được lưu ở `$HOME/.config/turtlebot4/pc_env.sh` bằng phần D2.
Chọn **1, 2 hoặc 3 robot**; chọn 2 thì chỉ chạy robot1 và robot2, không cần IP Pi 3.
`laptop.sh` kiểm tra cấu hình trước khi mở ROS. Nếu file cũ sai hoặc chưa có,
script tự hỏi lại số robot và IP. Mỗi terminal tự source môi trường qua script.

### LAPTOP — Terminal 1: một RViz cho các robot đã chọn

```bash
(
  set -e
  cd "$HOME/turtlebot4_ws"
  source /opt/ros/humble/setup.bash
  source "$HOME/turtlebot4_ws/install/setup.bash"
  bash "$HOME/turtlebot4_ws/scripts/laptop.sh" rviz
)
```

Giữ terminal này chạy. Nhãn, scan và đường đi của robot1 màu xanh dương,
robot2 màu cam, robot3 màu xanh lá; mỗi robot có model riêng. Các robot đang dùng phải có **cùng
bản đồ, cùng gốc và hướng map**. Map hiển thị mặc định lấy từ robot1.

### LAPTOP — Terminal 2: chọn robot nhận pose/goal và hủy goal

Copy cả khối rồi nhập số robot để chọn đích điều khiển. Menu chỉ chấp nhận
robot có trong phiên. Mặc định chọn robot1. Chọn robot **trước mỗi lần** đặt
pose/gửi goal và xem nhãn `Pose / Goal -> robotN` trên RViz để xác nhận.
Chọn robot khác không hủy goal đang chạy của robot trước.

```bash
(
  set -e
  cd "$HOME/turtlebot4_ws"
  source /opt/ros/humble/setup.bash
  source "$HOME/turtlebot4_ws/install/setup.bash"
  bash "$HOME/turtlebot4_ws/scripts/laptop.sh" menu
)
```

Trên RViz, dùng **2D Pose Estimate** đặt đúng vị trí và hướng thực tế của robot
đang chọn. Khi scan khớp tường, dùng **2D Goal Pose** để gửi goal cho robot đó.
Để robot khác di chuyển, chọn số của robot đó trong menu rồi gửi goal trên RViz.
Không đặt ba robot ở `(0,0,0)` khi chúng đứng khác chỗ.

Mỗi robot nhận một goal tại một thời điểm từ cửa sổ này. Muốn đổi goal, chọn
đúng robot rồi nhập `c`, đợi terminal 1 báo goal kết thúc, sau đó gửi goal mới.
Hủy goal chỉ tác động goal đã gửi qua cửa sổ fleet này. Thoát menu hoặc đóng
RViz **không tự hủy các goal đang chạy**; hủy goal từng robot trước khi đóng.

### LAPTOP — Terminal 3: kiểm tra các robot đã chọn

```bash
(
  set -e
  cd "$HOME/turtlebot4_ws"
  source /opt/ros/humble/setup.bash
  source "$HOME/turtlebot4_ws/install/setup.bash"
  bash "$HOME/turtlebot4_ws/scripts/laptop.sh" check
)
```

Phải thấy node của từng robot đã chọn (`/robot1/...` đến `/robotN/...`), mỗi robot có
publisher `scan` và `odom` riêng. Node `fleet_view` trên laptop ghép bản sao TF
vào `/tb4_view/tf` và `/tb4_view/tf_static`, thêm prefix robot cho frame ngoài
`map`; dữ liệu trên Pi giữ nguyên. RobotModel dùng TF Prefix tương ứng.

## B. PI 1, PI 2 VÀ PI 3 — chạy navigation khi đã có map

Cần có **cả `site.yaml` và `site.pgm`** trong `$HOME/turtlebot4_ws/maps/`
trên các Pi đang dùng. Chọn 1 robot: chỉ chạy Pi 1; chọn 2: Pi 1 và Pi 2;
chọn 3: chạy cả ba Pi. Dừng mapping và teleop trước khi chạy navigation.

### PI 1 — Terminal 1: bật hardware robot1

Khối này dừng dịch vụ bringup cũ rồi chạy hardware của repo. Nếu dịch vụ cũ
đã phát đúng topic `/robot1/...`, giữ dịch vụ và bỏ qua terminal hardware này.

```bash
(
  set -e
  cd "$HOME/turtlebot4_ws"
  source /opt/ros/humble/setup.bash
  source "$HOME/turtlebot4_ws/scripts/env.sh"
  sudo systemctl stop turtlebot4
  bash "$HOME/turtlebot4_ws/scripts/robot.sh" robot1 hardware
)
```

Giữ terminal này chạy. Nếu script báo hardware đang chạy, dừng dịch vụ driver
riêng đang mở LiDAR/republisher trước khi chạy lại; không mở hai driver cùng lúc.

### PI 1 — Terminal 2: navigation robot1

```bash
(
  set -e
  cd "$HOME/turtlebot4_ws"
  source /opt/ros/humble/setup.bash
  source "$HOME/turtlebot4_ws/scripts/env.sh"
  test -f "$HOME/turtlebot4_ws/maps/site.yaml"
  bash "$HOME/turtlebot4_ws/scripts/robot.sh" robot1 navigation \
    map:="$HOME/turtlebot4_ws/maps/site.yaml"
)
```

Giữ terminal này chạy, chọn `1` trong menu laptop rồi đặt pose robot1 bằng RViz.

### PI 2 — Terminal 1: bật hardware robot2

Nếu dịch vụ cũ đã phát đúng topic `/robot2/...`, giữ dịch vụ và bỏ qua
terminal hardware này.

```bash
(
  set -e
  cd "$HOME/turtlebot4_ws"
  source /opt/ros/humble/setup.bash
  source "$HOME/turtlebot4_ws/scripts/env.sh"
  sudo systemctl stop turtlebot4
  bash "$HOME/turtlebot4_ws/scripts/robot.sh" robot2 hardware
)
```

Giữ terminal này chạy.

### PI 2 — Terminal 2: navigation robot2

```bash
(
  set -e
  cd "$HOME/turtlebot4_ws"
  source /opt/ros/humble/setup.bash
  source "$HOME/turtlebot4_ws/scripts/env.sh"
  test -f "$HOME/turtlebot4_ws/maps/site.yaml"
  bash "$HOME/turtlebot4_ws/scripts/robot.sh" robot2 navigation \
    map:="$HOME/turtlebot4_ws/maps/site.yaml"
)
```

Giữ terminal này chạy, chọn `2` trong menu laptop rồi đặt pose robot2 bằng RViz.
Mỗi lệnh navigation đã mở map server, AMCL và Nav2; không mở thêm localization riêng.

### PI 3 — Terminal 1: bật hardware robot3

Nếu dịch vụ cũ đã phát đúng topic `/robot3/...`, giữ dịch vụ và bỏ qua
terminal hardware này.

```bash
(
  set -e
  cd "$HOME/turtlebot4_ws"
  source /opt/ros/humble/setup.bash
  source "$HOME/turtlebot4_ws/scripts/env.sh"
  sudo systemctl stop turtlebot4
  bash "$HOME/turtlebot4_ws/scripts/robot.sh" robot3 hardware
)
```

Giữ terminal này chạy.

### PI 3 — Terminal 2: navigation robot3

```bash
(
  set -e
  cd "$HOME/turtlebot4_ws"
  source /opt/ros/humble/setup.bash
  source "$HOME/turtlebot4_ws/scripts/env.sh"
  test -f "$HOME/turtlebot4_ws/maps/site.yaml"
  bash "$HOME/turtlebot4_ws/scripts/robot.sh" robot3 navigation \
    map:="$HOME/turtlebot4_ws/maps/site.yaml"
)
```

Giữ terminal này chạy, chọn `3` trong menu laptop rồi đặt pose robot3 bằng RViz.

## C. CHƯA CÓ MAP — quét bằng robot1

Giữ **PI 1 — Terminal 1: hardware** ở phần B chạy. Nếu navigation robot1 đang
chạy, nhấn `Ctrl+C` trong terminal navigation trước khi mở mapping.

### PI 1 — Terminal 2: mapping robot1

```bash
(
  set -e
  cd "$HOME/turtlebot4_ws"
  source /opt/ros/humble/setup.bash
  source "$HOME/turtlebot4_ws/scripts/env.sh"
  bash "$HOME/turtlebot4_ws/scripts/robot.sh" robot1 mapping
)
```

Giữ terminal này chạy. Để robot đứng yên đến khi thấy `Start origin captured`
và SLAM đăng ký LiDAR. Vị trí/hướng robot lúc đó là gốc map `(0,0,0)`.

### PI 1 — Terminal 3: lái robot để quét map

```bash
(
  set -e
  cd "$HOME/turtlebot4_ws"
  source /opt/ros/humble/setup.bash
  source "$HOME/turtlebot4_ws/scripts/env.sh"
  bash "$HOME/turtlebot4_ws/scripts/robot.sh" robot1 teleop
)
```

`i/,`: tiến/lùi; `j/l`: quay; `k` hoặc Space: dừng; `Ctrl+C`: thoát.
Mặc định 0,08 m/s và 0,3 rad/s; tự dừng sau 0,6 giây không nhận phím di chuyển.

### PI 1 — Terminal 4: lưu map

Cho robot đứng yên, giữ mapping chạy rồi copy cả khối này:

```bash
(
  set -e
  cd "$HOME/turtlebot4_ws"
  source /opt/ros/humble/setup.bash
  source "$HOME/turtlebot4_ws/scripts/env.sh"
  mkdir -p "$HOME/turtlebot4_ws/maps"
  ros2 run nav2_map_server map_saver_cli -f "$HOME/turtlebot4_ws/maps/site" --ros-args \
    -r __ns:=/robot1 -r map:=/robot1/map -p save_map_timeout:=15.0
)
```

Chép **cả `site.yaml` và `site.pgm`** từ `maps/` của Pi 1 vào
`$HOME/turtlebot4_ws/maps/` trên **Pi 2 và Pi 3**. Sau đó nhấn `k`, `Ctrl+C` ở teleop,
`Ctrl+C` ở mapping và chạy các terminal navigation ở phần B. Ba robot dùng
chung bản đồ; repo chưa ghép map từ hai phiên SLAM.

## D. CÀI ĐẶT MỘT LẦN

### D1. LAPTOP — một terminal cài RViz và build

Laptop cần có ROS 2 Humble tại `/opt/ros/humble`. Khối này tự clone nếu chưa
có workspace; nếu đã có thì dùng repo hiện tại.

```bash
(
  set -e
  source /opt/ros/humble/setup.bash
  if [[ ! -d "$HOME/turtlebot4_ws" ]]; then
    git clone https://github.com/phuwcc/turtlebot4_ws.git "$HOME/turtlebot4_ws"
  fi
  cd "$HOME/turtlebot4_ws"
  sudo apt update
  sudo apt install -y python3-colcon-common-extensions python3-yaml \
    ros-humble-rviz2 ros-humble-nav2-bringup ros-humble-navigation2 \
    ros-humble-slam-toolbox ros-humble-rmw-fastrtps-cpp \
    ros-humble-turtlebot4-description
  bash "$HOME/turtlebot4_ws/scripts/build.sh"
  source "$HOME/turtlebot4_ws/install/setup.bash"
  ros2 pkg prefix rviz2
  ros2 pkg prefix tb4_multi_bringup
)
```

### D2. LAPTOP — chọn số robot và nhập IP (làm một lần)

<a id="laptop-network"></a>

Copy cả khối dưới và chạy nguyên trạng. Khi script hỏi, **gõ câu trả lời rồi
nhấn Enter**. Không thay dòng lệnh bằng IP, không cần sửa file bằng tay.

```bash
(
  set -e
  cd "$HOME/turtlebot4_ws"
  python3 "$HOME/turtlebot4_ws/scripts/pc_config.py"
)
```

Ví dụ phiên dùng hai robot, nhập lần lượt:

```text
Số robot chạy trong phiên này [1/2/3]: 2
IP Wi-Fi/LAN robot1 (server ID 0): 10.11.63.119
IP Wi-Fi/LAN robot2 (server ID 1): 10.11.52.137
```

Hai IP trên lấy từ phiên bạn cung cấp; chỉ dùng nếu vẫn là IP hiện tại của hai Pi.
Chọn 2 thì script không hỏi robot3. Địa chỉ trống, sai IPv4 hoặc trùng nhau sẽ
bị từ chối và hỏi lại. File chỉ được lưu sau khi nhập đủ, đúng IP của các robot
đã chọn; cấu hình cũ không bị ghi đè nếu bạn dừng nhập giữa chừng.

Chạy lại D2 khi đổi số robot/IP, rồi **đóng RViz cũ và chạy lại Terminal 1, 2, 3**.
Script lưu số robot cùng môi trường DDS tại `$HOME/.config/turtlebot4/pc_env.sh`.
Frame, model, scan, nhãn và action client của robot không được chọn sẽ không
được mở trong RViz/node fleet.

### D3. MỖI PI — một terminal cài và build

Chạy nguyên khối trên từng Pi bạn đang dùng (từ Pi 1 đến Pi N):

```bash
(
  set -e
  source /opt/ros/humble/setup.bash
  if [[ ! -d "$HOME/turtlebot4_ws" ]]; then
    git clone https://github.com/phuwcc/turtlebot4_ws.git "$HOME/turtlebot4_ws"
  fi
  cd "$HOME/turtlebot4_ws"
  sudo apt update
  sudo apt install -y python3-colcon-common-extensions python3-yaml \
    ros-humble-turtlebot4-bringup ros-humble-turtlebot4-description \
    ros-humble-create3-republisher ros-humble-navigation2 \
    ros-humble-nav2-bringup ros-humble-slam-toolbox
  bash "$HOME/turtlebot4_ws/scripts/build.sh"
  source "$HOME/turtlebot4_ws/scripts/env.sh"
)
```

Build ngay tại `$HOME/turtlebot4_ws`, không có workspace con `tb4_multi_robot/`.
Script build dùng một luồng để hạn chế RAM.

### D4. Mạng và định danh robot — thiết lập một lần trước khi chạy

| Cấu hình | Pi 1 | Pi 2 | Pi 3 |
| --- | --- | --- | --- |
| Namespace | `/robot1` | `/robot2` | `/robot3` |
| ROS domain | `0` | `0` | `0` |
| Discovery server ID | `0` | `1` | `2` |
| Discovery port | `11811` | `11811` | `11811` |
| Create 3 namespace | `/robot1/_do_not_use` | `/robot2/_do_not_use` | `/robot3/_do_not_use` |

Dùng `turtlebot4-setup` trên từng Pi và Apply cấu hình tương ứng cho Create 3
như [hướng dẫn mạng](docs/NETWORK.md). Chỉ export trong terminal không đổi cấu
hình dịch vụ/firmware. Laptop và ba Pi phải kết nối được với nhau và đồng bộ
đồng hồ. Kiểm tra model trong [robot1.yaml](src/tb4_multi_bringup/config/robots/robot1.yaml),
[robot2.yaml](src/tb4_multi_bringup/config/robots/robot2.yaml)
và [robot3.yaml](src/tb4_multi_bringup/config/robots/robot3.yaml): mặc định `standard`,
đổi thành `lite` nếu đúng loại robot và build lại.

## E. DỪNG VÀ XỬ LÝ LỖI TRONG ẢNH

- Dừng: chọn lần lượt từng robot trong menu laptop và nhấn `c` để hủy goal; teleop nhấn `k` rồi `Ctrl+C`; dừng mapping/navigation
  bằng `Ctrl+C`, sau cùng mới dừng hardware.
- `package 'rviz2' not found`: chạy phần D1 trên laptop sẽ mở RViz.
- File môi trường không tồn tại hoặc `Wrong ipv4 address passed into the server's list`:
  chạy D2, chọn số robot và nhập IP sau dấu nhắc; chạy lại Terminal 1. Không đặt
  IP vào chuỗi lời nhắc của `read`: chuỗi đó chỉ là chữ hiển thị, không gán IP.
  Các lệnh mới tự phát hiện file lỗi và yêu cầu cấu hình lại trước khi mở DDS.
- `Stereo is NOT SUPPORTED`: thông báo khả năng hiển thị stereo của RViz,
  không phải lỗi địa chỉ IPv4 và không cản trở xem bản đồ 2D.
- `Invalid frame ID "odom"`: kiểm tra terminal hardware của Pi tương ứng và
  publisher odometry bằng LAPTOP — Terminal 3.
- Scan báo timestamp sớm hơn TF: kiểm tra đồng hồ và dữ liệu TF của các máy.
  Source môi trường không tạo odometry hoặc sửa thời gian TF.

Chỉ chạy một mapping hoặc navigation cho mỗi robot. Hiện mỗi robot điều hướng
riêng; chưa có bộ phân công nhiệm vụ/điều phối giao thông. Build và kiểm thử với
TF, map, scan và Nav2 giả đã chạy; **chưa xác nhận navigation đồng thời trên ba robot thật**.

- [Hướng dẫn đa robot và kiểm thử](docs/MULTI_ROBOT.md)
- [Cấu hình mạng Pi / Create 3 / laptop](docs/NETWORK.md)
- [Kết quả và giới hạn kiểm thử](docs/VALIDATION.md)
- [Hướng dẫn một robot cũ, không namespace](docs/SINGLE_ROBOT.md)
