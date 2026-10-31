# TurtleBot 4 — Nav2 trên PC, hardware trên robot

Giữ concept **1–3 robot, namespace `/robot1`…`/robotN`, một RViz**, cùng bản đồ.
PC chạy toàn bộ navigation. Socket TCP nhận nhiệm vụ trên PC; PC và các Pi vẫn
trao đổi ROS 2/DDS. Các lệnh dưới chạy tại thư mục `~/turtlebot4_ws`; script tự
source ROS và workspace, không cần copy các block môi trường dài.

| Chạy ở đâu | Thành phần |
| --- | --- |
| **PC / máy case** | Map server + AMCL + Nav2 riêng cho từng robot; SLAM khi tạo map; một RViz; gateway TCP nhận nhiệm vụ |
| **Mỗi Pi / robot** | LiDAR, driver Create 3, robot description/TF, bộ giám sát lệnh vận tốc |
| **Create 3** | Điều khiển bánh xe, odometry, IMU, pin, E‑Stop và bảo vệ phần cứng |

```text
Ứng dụng / pc-command.sh ── TCP JSON ──> PC: gateway ──> Nav2 từng robot
RViz ────────────────────────────────────────┘              │
PC <── DDS: scan / odom / TF / pin ── Pi                    │
PC ── DDS: remote_cmd_vel có timestamp ──> Pi: watchdog ──> Create 3
```

Socket dùng để gửi `pose`, `goal`, `cancel`, `status`, `select`; không thay DDS
bằng một giao thức tự truyền scan/TF. Chế độ navigation thông thường điều hướng
từng robot độc lập. Chế độ [MARCO](#marco-trên-hai-turtlebot-thật) thêm phân công
nhiệm vụ, dependency và điều phối đường đi giữa các robot trên PC.

## 1. Cài đặt một lần

Yêu cầu Ubuntu 22.04, ROS 2 Humble tại `/opt/ros/humble`, repo ở
`~/turtlebot4_ws` trên PC và các Pi. Script kiểm tra gói đã cài, chỉ chạy apt nếu còn thiếu rồi build;
không bật robot. Khi đã đủ gói, script bỏ qua apt để tránh chờ cập nhật hệ thống.
Muốn cập nhật gói, thêm `--refresh`, ví dụ `bash scripts/setup.sh robot --refresh`.
Bước cài gói chờ khóa APT tối đa 300 giây; nếu cập nhật tự động đang chạy,
đợi hoàn tất rồi chạy lại, không xóa file khóa APT.

### Clone repo và MARCO submodule

**Trên PC, clone kèm submodule:**

```bash
git clone --recurse-submodules https://github.com/phuwcc/turtlebot4_ws.git ~/turtlebot4_ws
cd ~/turtlebot4_ws
```

**Nếu đã có repo trên PC:**

```bash
cd ~/turtlebot4_ws
git pull --ff-only
git submodule update --init --recursive MARCO
```

`MARCO/` là submodule từ [TranThang27/MARCO](https://github.com/TranThang27/MARCO),
được ghim tại revision `bef1ac7d3161fb6ce009a507beac86a06fe92371` đã kiểm tra.
Adapter nằm trong `src/tb4_marco_bridge/`; giữ source upstream MARCO nguyên bản.
Lệnh update lấy revision được repo cha ghim, không tự chuyển sang bản upstream mới.
Nếu có thay đổi riêng trong MARCO, giữ lại chúng trước khi cập nhật submodule.

**Trên Pi**, clone repo cha bằng `git clone` thông thường; không cần khởi tạo
submodule vì MARCO và toàn bộ navigation chạy trên PC.

**Trên PC:**

```bash
bash scripts/setup.sh pc
```

**Trên mỗi Pi:**

```bash
bash scripts/setup.sh robot
```

Khi chuyển từ bản cũ, dừng các terminal mapping/navigation/teleop cũ trên Pi
và PC trước khi chạy phiên mới. Sau khi cập nhật code, build lại:

```bash
# PC: cập nhật submodule rồi build 4 package
git submodule update --init --recursive MARCO
bash scripts/build.sh

# Mỗi Pi: chỉ build 3 package hardware
TB4_BUILD_PROFILE=hardware bash scripts/build.sh
```

### Cấu hình PC

<a id="laptop-network"></a>

```bash
bash scripts/pc-configure.sh
```

Nhập số robot 1/2/3 và IP của từng Pi. Chọn 2 thì chỉ nhập robot1, robot2.
Cấu hình lưu ở `~/.config/turtlebot4/pc_env.sh`. Chạy lại khi đổi IP/số robot,
rồi dừng và mở lại phiên PC. PC phải kết nối được tới các Pi và đồng bộ giờ.

Nếu chỉ chạy robot2, có thể giữ đúng ID discovery bằng
`bash scripts/pc-configure.sh --robots robot2` (server ID 1), thay vì đổi tên robot2
thành robot1. Mặc định vẫn chọn robot1 đến robotN.

### Cấu hình từng robot

Dùng `turtlebot4-setup` cấu hình **Pi và Create 3**, theo [NETWORK.md](docs/NETWORK.md).
Không chỉ export namespace trong terminal.

| Mục | Robot 1 | Robot 2 | Robot 3 |
| --- | --- | --- | --- |
| Namespace | `/robot1` | `/robot2` | `/robot3` |
| ROS domain | 0 | 0 | 0 |
| Discovery server ID | 0 | 1 | 2 |
| Discovery port | 11811 | 11811 | 11811 |
| Create 3 namespace | `/robot1/_do_not_use` | `/robot2/_do_not_use` | `/robot3/_do_not_use` |

Model trong `src/tb4_multi_bringup/config/robots/robotN.yaml` mặc định là
`standard`; đổi thành `lite` rồi build lại nếu robot là Lite. PC và Pi dùng
cùng cấu hình robot. Profile hardware của repo phục vụ LiDAR navigation,
không mở camera và joystick.

Profile này đặt `power_saver: false` cho `turtlebot4_node` để không gọi
`oakd/start_camera` hoặc `oakd/stop_camera` khi vào/ra dock vì camera không
được chạy. LiDAR vẫn dùng `auto_standby` của driver.

## 2. Chạy navigation hằng ngày

### ROBOT — mỗi Pi một terminal hardware

Trên **Pi 1**:

```bash
cd ~/turtlebot4_ws
bash scripts/robot-start.sh robot1
```

Trên **Pi 2**, nếu dùng:

```bash
cd ~/turtlebot4_ws
bash scripts/robot-start.sh robot2
```

Trên **Pi 3**, nếu dùng:

```bash
bash scripts/robot-start.sh robot3
```

Giữ các terminal chạy. Script kiểm tra namespace, dừng service `turtlebot4`
cũ, rồi mở hardware của repo kèm watchdog. Nếu báo hardware đã chạy, dừng
bringup/republisher tự mở ở terminal khác trước; không mở hai driver.
**Không giữ service cũ để bỏ qua bước này:** service cũ không có watchdog mới.
Không chạy AMCL, map server, SLAM hoặc Nav2 trên Pi.

`robot-start.sh` tự dừng service cũ và mở watchdog; không cần chạy watchdog riêng.
Nếu gặp thông báo **Hardware is already running**, dừng hardware cũ rồi chạy lại.
Trên **Pi robot1**:

```bash
cd ~/turtlebot4_ws
bash scripts/robot-stop.sh robot1
bash scripts/robot-start.sh robot1
```

Trên **Pi robot2**:

```bash
cd ~/turtlebot4_ws
bash scripts/robot-stop.sh robot2
bash scripts/robot-start.sh robot2
```

`robot-stop.sh` dừng service cũ, launch và các driver còn sót của đúng namespace,
đợi tiến trình thoát rồi mới báo thành công. Nếu chưa dừng được, script trả lỗi;
không mở phiên hardware mới. Script dùng SIGTERM vì phiên mở ngầm bằng `nohup`
có thể bỏ qua SIGINT; chỉ dừng PID của launch cũng có thể để lại driver con.
Không cần tìm/thay PID thủ công và không dùng lệnh `kill -INT PID_LAUNCH`.
Với hardware chạy ở terminal trước mặt, vẫn có thể nhấn **Ctrl+C** để dừng.

Kiểm tra khi cần:

```bash
systemctl status turtlebot4 --no-pager
pgrep -af 'ros2 launch|create3_republisher|rplidar_composition|remote_velocity_guard'
```

### PC — Terminal 1: toàn bộ Nav2 + socket + một RViz

Đặt **cả `site.yaml` và file ảnh bản đồ mà YAML tham chiếu** trong `maps/` trên
PC. Không cần chép bản đồ lên các Pi.

```bash
bash scripts/pc-start.sh
```

Mặc định đọc `maps/site.yaml`. Nếu dùng map đã có trong repo:

```bash
bash scripts/pc-start.sh maps/my_room.yaml
```

Chỉ dùng map phù hợp với môi trường thực tế. Cả robot1…robotN dùng cùng map,
cùng gốc và hướng. Chạy không có màn hình bằng
`bash scripts/pc-start.sh maps/site.yaml rviz:=false`.

Khi chỉ cần xem map, hai robot và kiểm tra localization, dùng chế độ nhẹ:

```bash
bash scripts/pc-start.sh maps/my_room.yaml localization_only:=true
```

Chế độ này chạy AMCL/map server và RViz; chưa chạy planner/controller hoặc
relay vận tốc, nên chưa nhận goal điều hướng. Dừng bằng **Ctrl+C**, rồi chạy
lại không có `localization_only:=true` khi cần Nav2 đầy đủ. Có thể kết hợp
`initial_poses:=...` với cả hai chế độ.

Các node chạy liên tục `fleet_view`, `socket_control`, `remote_velocity` dùng C++.
Launch và công cụ cấu hình vẫn dùng Python. Build mặc định tuần tự, ưu tiên thấp,
giới hạn 75% một lõi CPU và 2,5 GiB RAM nếu có systemd user manager. Có thể chỉnh
`TB4_BUILD_CPU_QUOTA`, `TB4_BUILD_MEMORY_HIGH`, `TB4_BUILD_MEMORY_MAX`; không nên
build nhiều phiên cùng lúc. Xem [kết quả kiểm chứng](docs/VALIDATION.md).

AMCL của từng robot được khởi động/activate cùng phiên PC. Để AMCL tự nhận
pose riêng ngay khi khởi động, chép `src/tb4_multi_bringup/config/initial_poses.example.yaml`
thành `config/initial_poses.yaml`, sửa tọa độ đúng vị trí thực tế trên map rồi chạy:

```bash
bash scripts/pc-start.sh maps/my_room.yaml initial_poses:="$PWD/config/initial_poses.yaml"
```

File phải có `x`, `y`, `yaw` cho mỗi robot đang dùng; góc yaw tính bằng radian.
Nếu không truyền file, AMCL chờ **2D Pose Estimate** hoặc lệnh socket `pose`.
Có thể sửa pose trong RViz cả sau khi đã tự khởi tạo từ file.

RViz bật map, mô hình và **AMCL pose** của từng robot, giới hạn 10 FPS.
**Robot TF trees** mặc định tắt để giảm tải; có thể bật trong Displays khi cần debug.
Scan dành cho hiển thị được giới hạn 5 Hz; AMCL và costmap vẫn nhận scan gốc.
Trong view chung, chuỗi TF là `map -> robotN/odom -> robotN/base_link -> ...`;
TF gốc trên `/robotN/tf` vẫn dùng các frame riêng trong namespace robot.
Mô hình đọc `/robotN/robot_description` từ hardware trên Pi. Nếu thiếu
`odom -> base_link`, kiểm tra dữ liệu odom/TF từ Create 3 và republisher trước;
đặt pose AMCL không tạo được TF odometry bị thiếu.

### MARCO trên hai TurtleBot thật

Giữ hardware/watchdog trên hai Pi chạy theo phần trên. Với robot1 đứng đúng
gốc map đã quét `(0,0)`, quay theo +X (`yaw=0`), robot2 bên trái khoảng 0,5 m,
cùng hướng, mở trên **PC**:

```bash
git submodule update --init --recursive MARCO
bash scripts/pc-marco.sh maps/my_room.yaml
```

Script tự khởi tạo robot1, gửi gợi ý từ `config/marco/localization.yaml` cho
robot2 và chờ AMCL hội tụ trước khi bật Nav2. Tọa độ robot2 do AMCL ước lượng;
gợi ý không khóa TF hoặc vị trí. Sửa gợi ý khi đổi bố trí. Nếu khởi động lại
sau khi robot đã đi khỏi gốc, truyền tọa độ thực tế bằng file `x/y/yaw`:

```bash
bash scripts/pc-marco.sh maps/my_room.yaml initial_poses:="$PWD/config/initial_poses.yaml"
```

Phiên MARCO mặc định có ngân sách **200% CPU, tối đa 2 GiB RAM** khi có user
systemd; script thông báo nếu chỉ áp dụng nice. Chỉ cần xem map và hai robot:

```bash
bash scripts/pc-marco.sh maps/my_room.yaml localization_only:=true
```

Chỉ chạy một phiên PC tại một thời điểm. Ở **terminal PC khác**, kiểm tra trạng
thái và chọn goal trong vùng trống thực tế trước khi xem kế hoạch:

```bash
bash scripts/marco.sh status
cp config/marco/mission.example.json config/marco/mission.json
# Sửa goal (x,y,yaw), candidates và depends_on trong mission.json theo map thực tế
bash scripts/marco.sh plan config/marco/mission.json --out log/marco/preview.json
```

`status` phải báo `localized=true` cho cả hai robot. `plan` hiển thị đường
MARCO trong RViz và không điều khiển robot. Các goal mẫu không phải vị trí đã
được xác nhận an toàn. Khi preview phù hợp, robot đã khỏi dock, vùng chạy trống
và có người giám sát, thực thi riêng bằng:

```bash
bash scripts/marco.sh run config/marco/mission.json --execute --out log/marco/execution.json
```

Đường đi được kiểm tra trên costmap rồi thực thi bằng Nav2 `FollowPath`, giới
hạn 0,05 m/s và 0,2 rad/s. Lease và permission hết hạn hoặc mất localization sẽ
hủy path; watchdog trên Pi vẫn giám sát luồng lệnh. Nhấn **Ctrl+C** ở terminal
mission để hủy các path còn hoạt động.

**Kiểm chứng 31/10/2026:** goal riêng của cả robot1 và robot2 đã hoàn thành qua
MARCO/socket/Nav2 trên phần cứng thật, nhưng phải hủy và chạy tiếp; chưa đạt một
lượt hai goal liên tục không lỗi. Mẫu PC đứng yên với đầy đủ Nav2/RViz đạt trung
bình khoảng 153% CPU, peak 170%; không suy ra tải khi chạy goal. Xem
[báo cáo và ảnh RViz](docs/VALIDATION.md), [luồng và giới hạn MARCO](docs/MARCO.md).

### PC — Terminal 2: chọn robot và hủy goal

```bash
bash scripts/pc-menu.sh
```

Nhập `1/2/3` chọn robot; `c` hủy goal của robot đã chọn; `s` xem trạng thái;
`q` thoát menu. Xác nhận nhãn `Pose / Goal -> robotN` trong RViz trước mỗi lần
đặt **2D Pose Estimate** hoặc **2D Goal Pose**.

Đặt pose đúng vị trí/hướng thực tế, chờ scan khớp tường, rồi gửi goal.
Không đặt mọi robot về `(0,0,0)` khi chúng đứng khác chỗ. Chọn robot khác
không hủy goal của robot trước. Mỗi robot nhận một goal qua gateway/RViz;
hủy và đợi goal kết thúc trước khi gửi goal khác hoặc đặt lại pose.

### PC — Terminal 3: kiểm tra kết nối khi cần

```bash
bash scripts/pc-check.sh
```

Phải thấy node và publisher `scan`, `odom` riêng cho từng robot đã chọn.
Nếu có topic nhưng chưa chắc có dữ liệu, kiểm tra thêm tốc độ topic/diagnostics;
script check liệt kê graph, không chứng nhận robot đủ điều kiện di chuyển.

## 3. Gửi nhiệm vụ qua socket

Gateway chạy cùng `pc-start.sh`, mặc định **TCP `127.0.0.1:8765` trên PC**.
Client không cần ROS. Mỗi kết nối gửi một JSON kết thúc bằng newline, nhận
một JSON rồi đóng; giới hạn request 8192 byte.

Trên PC:

```bash
bash scripts/pc-command.sh status
```

Đặt pose ban đầu, tọa độ mét và góc yaw radian trong frame map:

```bash
bash scripts/pc-command.sh pose robot1 1.0 2.0 0.0
```

Gửi goal:

```bash
bash scripts/pc-command.sh goal robot1 3.0 2.5 1.57
```

Hủy goal:

```bash
bash scripts/pc-command.sh cancel robot1
```

Dữ liệu tương ứng ứng dụng của bạn cần gửi:

```json
{"command":"goal","robot":"robot1","x":3.0,"y":2.5,"yaw":1.57}
```

`ok: true` của goal nghĩa là đã gửi yêu cầu tới Nav2, chưa phải đã đến đích.
Poll `status`: `busy` báo còn goal đang xử lý, `accepted` báo có goal được Nav2
chấp nhận; `last_result` là `succeeded`, `canceled`, `aborted`, `rejected`
hoặc `null` nếu chưa có kết quả. `nav_ready` chỉ báo action server có mặt,
không bảo đảm AMCL đã định vị đúng. `pose` chỉ xác nhận đã publish initial pose.
RViz và socket dùng **cùng bộ quản lý goal**, nên không tự giành goal của nhau.
Không gửi goal trực tiếp bằng client ROS khác nếu muốn gateway theo dõi đầy đủ.

Nếu ứng dụng chạy ở máy khác, dùng SSH tunnel tới PC, ví dụ
`ssh -N -L 8765:127.0.0.1:8765 USER@PC_IP`, rồi kết nối localhost:8765.
Socket chưa có xác thực riêng nên gateway chỉ bind loopback. Đổi port khi cần
bằng `socket_port:=8766` ở `pc-start.sh` và `TB4_SOCKET_PORT=8766` ở client.
Mất kết nối socket/thoát menu **không hủy nhiệm vụ đã gửi**.

## 4. Chưa có map — SLAM chạy trên PC

Giữ hardware trên các Pi chạy. Dừng phiên navigation và teleop trước.

**PC — Terminal 1:**

```bash
bash scripts/pc-mapping.sh robot1
```

Lệnh mở SLAM và một RViz. Để robot đứng yên tới khi thấy
`Start origin captured`; vị trí và hướng khi đó là gốc map. Khi mapping,
không dùng 2D Pose Estimate/Goal Pose để điều khiển robot.

**Pi 1 — Terminal 2:**

```bash
bash scripts/robot-teleop.sh robot1
```

`i/,`: tiến/lùi; `j/l`: quay; `k` hoặc Space: dừng; `Ctrl+C`: thoát.
Mặc định 0,08 m/s và 0,3 rad/s, tự dừng sau 0,6 giây không nhận phím di chuyển.

**PC — Terminal 2**, robot đứng yên, SLAM vẫn chạy:

```bash
bash scripts/pc-save-map.sh robot1
```

Map lưu thành `maps/site.yaml` và `maps/site.pgm` trên PC. Dừng teleop,
dừng mapping rồi chạy `pc-start.sh`. Không cần chép map lên robot.

## 5. Teleop riêng, dừng và mất mạng

Khi không chạy navigation/mapping, có thể lái từ PC:

```bash
bash scripts/pc-teleop.sh robot2
```

Hoặc lái ngay trên Pi bằng `robot-teleop.sh robot2` khi mapping trên PC.
**Dừng navigation trước khi mở teleop**. PC script khóa các phiên
nav/mapping/teleop/RViz trên cùng PC để tránh mở chồng. Pi sẽ phát lệnh dừng
nếu có nhiều publisher `remote_cmd_vel`; cơ chế này không tự chọn nguồn ưu tiên.
Không chạy thêm joystick hoặc publisher trực tiếp vào `cmd_vel`.

Pi kiểm tra tuổi timestamp và tự phát vận tốc 0 khi lệnh quá hạn **0,5 giây**;
lệnh cũ/không hợp lệ bị bỏ qua. Vì vậy PC/Pi phải đồng bộ giờ. Nếu PC/Wi-Fi
mất, robot dừng khi luồng lệnh hết hạn; nếu kết nối phục hồi khi goal vẫn còn
chạy, robot **có thể tiếp tục nhiệm vụ**. Hủy goal nếu không muốn tiếp tục.
Watchdog không thay E‑Stop hay bảo vệ Create 3; repo không tự mở E‑Stop/undock.

Dừng bình thường: chọn từng robot và `c` trong menu, đợi goal kết thúc; teleop
nhấn `k` rồi `Ctrl+C`; `Ctrl+C` terminal PC; cuối cùng `Ctrl+C` hardware từng Pi.
Đóng RViz sẽ kết thúc phiên PC/navigation. Service hardware cũ đã dừng và không
được tự bật lại khi thoát script; muốn dùng lại profile cũ, tự start lại service.

Kiểm thử phần cứng trên PC (dừng Nav2/teleop, giữ hardware Pi chạy):

```bash
bash scripts/pc-test-hardware.sh
# Chỉ thêm cờ này khi robot đã ra khỏi dock, vùng thử trống và có người giám sát:
bash scripts/pc-test-hardware.sh --allow-motion
```

Mặc định không di chuyển. Chế độ motion thử lần lượt từng robot ở 0,04 m/s,
tối đa 2 giây, đo odometry và dừng watchdog. Không tự undock/mở E-Stop.
Kết quả lưu ở `log/pc/hardware-test/result.json`.
Chẩn đoán sensor, map và lifecycle bằng `bash scripts/pc-diagnose.sh --map maps/my_room.yaml`.

## 6. Giới hạn và tài liệu

Các kiểm thử phần mềm chạy trong ROS domain riêng với cảm biến/Nav2 giả;
không coi đó là xác nhận navigation đồng thời trên ba robot thật.
Chạy lại kiểm thử phần mềm bằng `bash scripts/test.sh` sau khi build.
Cần kiểm tra thực tế Wi-Fi, đồng hồ, pose/map, E‑Stop và phản hồi dừng sau
khi triển khai PC + Pi. Ngày 2026-10-30 đã build trên PC và hai Pi, thử tiến ngắn và watchdog thành công trên robot1/robot2.
Navigation thật vẫn có lỗi timeout lifecycle qua DDS; chưa chứng nhận tự điều hướng.
Xem [kết quả và lỗi còn mở](docs/VALIDATION.md).

- [Kiến trúc và kiểm thử](docs/MULTI_ROBOT.md)
- [Mạng Pi / Create 3 / PC](docs/NETWORK.md)
- [Kết quả kiểm thử](docs/VALIDATION.md)
- [Hướng dẫn một robot cũ](docs/SINGLE_ROBOT.md) — tham khảo kiến trúc cũ, không chạy cùng phiên mới.
