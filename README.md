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

## Chạy hằng ngày — hai lệnh

Trong thư mục repo, dùng hai điểm vào sau. Các script sẽ hỏi thông tin trong
terminal; không cần nhớ các option ROS hoặc sửa file shell.

**1. Trên mỗi Pi:**

```bash
bash run_robot.sh
```

Chọn **1 — Hardware + watchdog**, rồi chọn đúng robot1/robot2. Giữ terminal này
chạy. Script vẫn kiểm tra namespace, dừng service cũ và chặn driver chạy trùng.
Menu còn có lái bàn phím khi mapping, dừng hardware và kiểm tra cảm biến.
Không chạy Nav2/AMCL/SLAM trên Pi trong luồng này.

**2. Trên PC:**

```bash
bash run_pc.sh
```

Menu PC:

| Mục | Công việc |
| --- | --- |
| 1 | MARCO + Nav2 + RViz |
| 2 | Chỉ localization + RViz, không điều hướng |
| 3 | Quét map bằng một robot, mặc định robot1 |
| 4 | Chọn robot, đặt pose/goal, hủy goal |
| 5 | Xem kế hoạch từ mission JSON |
| 6 | Preview rồi thực thi mission JSON |
| 7 | Lưu map đang quét |
| 8 | Đổi số robot và IP |

Lần đầu, PC hỏi số robot và IP từng Pi; lần sau hiển thị cấu hình đã lưu để
chọn dùng tiếp hoặc nhập lại. Khi mở navigation/localization, menu hỏi file map
và cách khởi tạo: xác nhận bố trí đã khai báo hoặc nhập file pose thực tế của
từng robot. Không chọn seed `(0,0)` nếu robot1 đã đi khỏi gốc map.

Thiếu workspace build, script hỏi trước khi gọi setup/build. Thiếu MARCO ở chế
độ cần planner, script hỏi trước khi lấy submodule đã ghim. Thao tác cài gói có
thể yêu cầu mật khẩu sudo trong terminal.

Giữ menu 1/2/3 chạy ở terminal PC thứ nhất. Mở **cùng lệnh `bash run_pc.sh`** ở
terminal PC thứ hai, chọn 4/5/6/7 để điều khiển hoặc làm việc với phiên đó.
Chỉ mở một phiên navigation/localization/mapping; khóa phiên cũ vẫn được giữ.
Mapping: dùng terminal Pi thứ hai với `bash run_robot.sh`, chọn mục 2 để lái,
sau khi đã dừng navigation. Đổi IP/số robot thì khởi động lại phiên PC.

Chọn mục 6 không tự chạy robot ngay: phải có mission JSON hợp lệ, preview thành
công, rồi nhập **CHAY** sau khi xem đường đi, đưa robot khỏi dock và giám sát.
Preview lỗi hoặc hủy nhập liệu sẽ dừng trước khi gửi yêu cầu thực thi.

Test nhỏ cho menu/cấu hình, không dùng ROS hoặc điều khiển robot:

```bash
python3 -B tests/test_run_menu.py -v
python3 -B tests/test_pc_config.py -v
```

## Quét map bằng một robot (mặc định robot1)

Trên PC chạy `bash run_pc.sh`, chọn **3**, nhấn Enter ở câu hỏi robot để dùng
**robot1**. Hoặc gọi trực tiếp:

```bash
bash scripts/pc-mapping.sh
# Chọn robot khác khi cần:
# bash scripts/pc-mapping.sh robot2
```

Mỗi phiên chỉ chạy một SLAM và một robot trong RViz/gateway, kể cả khi cấu
hình PC lưu hai hoặc ba robot. Phiên mapping tự chọn discovery của robot đó,
giữ đúng server ID; cấu hình fleet trong `~/.config/turtlebot4/pc_env.sh`
không bị sửa. Robot được chọn phải có trong cấu hình PC. Mặc định dòng lệnh
là robot1; nếu cấu hình chỉ có robot2, truyền `robot2` rõ ràng.

Trước khi mapping, dừng phiên navigation/localization trên PC. Trên Pi robot1,
giữ hardware/watchdog chạy bằng `bash scripts/robot-start.sh robot1`. Khi PC
báo **Start origin captured**, dùng terminal thứ hai **trên Pi robot1** để lái:

```bash
bash scripts/robot-teleop.sh robot1
```

Ở terminal PC khác, lưu map trong lúc SLAM vẫn chạy:

```bash
bash scripts/pc-save-map.sh robot1 maps/site
```

Map lưu thành `maps/site.yaml` và `maps/site.pgm`. Trong menu, mục **7** cũng
mặc định chọn robot1 để lưu map. Dừng teleop rồi dừng mapping trước khi mở
navigation với map vừa lưu. Các robot còn lại đứng yên; không cần bật chúng
để quét map bằng robot1.

## Chạy testcase hai robot: robot1 → A, robot2 → B

Luồng dưới đã chạy thành công trên hai robot thật ngày **10/11/2026** với
`maps/my_room.yaml`. Chạy các lệnh PC từ `~/turtlebot4_ws`.

**1. Trên mỗi Pi**, nếu chưa build repo, chạy `bash scripts/setup.sh robot`
một lần. Sau đó mở hardware và giữ terminal chạy:

```bash
# Pi robot1 — 10.11.52.230
bash scripts/robot-start.sh robot1

# Pi robot2 — 10.11.52.243 (chạy ở terminal của Pi thứ hai)
bash scripts/robot-start.sh robot2
```

Nếu script báo hardware đã chạy, dùng `bash scripts/robot-stop.sh robot1`
trên Pi 1 hoặc `bash scripts/robot-stop.sh robot2` trên Pi 2 rồi mở lại.
Phiên hardware phải có `remote_velocity_guard`; service mặc định không đủ.

**2. Trên PC**, lưu IP hai Pi; chỉ cần làm lại khi đổi IP hoặc số robot:

```bash
python3 scripts/pc_config.py --robots robot1,robot2 <<'EOF'
10.11.52.230
10.11.52.243
EOF
```

Nếu chưa setup/build PC, chạy `git submodule update --init --recursive MARCO`
rồi `bash scripts/setup.sh pc` một lần. Tạo pose ban đầu **đúng vị trí thực tế**.
Ví dụ sau dành cho robot1 ở gốc map, robot2 bên trái 0,5 m, cùng hướng +X:

```bash
cat > config/initial_poses.yaml <<'EOF'
robot1: {x: 0.0, y: 0.0, yaw: 0.0}
robot2: {x: 0.0, y: 0.5, yaw: 0.0}
EOF
bash scripts/pc-marco.sh maps/my_room.yaml initial_poses:="$PWD/config/initial_poses.yaml"
```

Giữ terminal PC này chạy. Chỉ mở một phiên PC; dừng phiên cũ trước nếu có.
Sau khi robot đã đi, sửa pose theo vị trí hiện tại trước khi khởi động lại;
không dùng lại `(0,0)` cho robot1 nếu chưa đưa robot về gốc map.

**3. Terminal PC khác**, kiểm tra rồi preview testcase 01:

```bash
bash scripts/marco.sh status
bash scripts/pc-command.sh status
bash scripts/marco.sh plan config/marco/testcases/01_parallel_ab.json --out log/marco/testcases/01-preview.json
```

Chờ cả hai robot báo `localized=true`, `odom_fresh=true` trong MARCO status,
và `path_ready=true` trong PC status. Testcase 01 gán robot1 đến
**A `(x=0.15, y=-0.10, yaw=0)`**, robot2 đến **B `(x=0.15, y=0.60, yaw=0)`**
trong frame `map` (mét/radian). Hai task không có dependency; lượt preview
đã kiểm tra có hai path trong cùng batch. `plan` chỉ hiển thị đường trong RViz.
Đổi `goal` trong JSON nếu A/B thực tế khác; các tọa độ này dành cho map và bố
trí đã thử, không tự áp dụng cho phòng khác.

**4. Chạy thật**, khi preview phù hợp, robot đã khỏi dock, vùng chạy trống
và có người giám sát:

```bash
bash scripts/marco.sh run config/marco/testcases/01_parallel_ab.json --execute --out log/marco/testcases/01-run.json
bash scripts/pc-command.sh status
```

Tốc độ tối đa 0,05 m/s và 0,2 rad/s. Nhấn **Ctrl+C ở terminal mission** để hủy
các path còn hoạt động. Thành công khi báo cáo có `result: "succeeded"`,
`completed_tasks` chứa `1` và `2`, status hai robot có `busy=false`,
`last_result="succeeded"`. Mất điều kiện định vị sẽ hủy nhiệm vụ; chờ định vị
ổn định rồi preview lại từ vị trí hiện tại trước khi chạy lại. Không nới ngưỡng
AMCL để ép chạy.

**Kết quả 10/11/2026:** lượt đầu bị hủy do covariance AMCL robot1 vượt ngưỡng;
lượt chạy lại hoàn thành cả hai task, hai lệnh vận tốc về 0. Các log phiên này
ở `log/marco/testcases/01-retry-run.json` và `01-retry-stop-check.json` (log cục
bộ, không lưu trong Git). Chưa kiểm chứng thực thi testcase đổi đích và tuần tự.
Xem [toàn bộ testcase và cách đổi kịch bản](config/marco/testcases/README.md).

**Kiểm tra lại 11/11/2026 trên map cũ:** testcase 01 hoàn thành sau một lượt
thử lại từ vị trí quan sát; robot1 đã ở A từ lượt đầu, robot2 hoàn thành B ở
lượt sau. Testcase 02 có kế hoạch nhường đường nhưng bị hủy khi lệch hành lang;
chưa đạt đổi chỗ hoàn chỉnh trên phần cứng. Testcase 09 đối đầu khoảng 1 m
mới đạt trên map tổng hợp, chưa chạy thật vì robot2 chưa đạt localization.
Xem [kết quả và giới hạn thực tế](docs/VALIDATION.md).

## Thêm 5 kịch bản kiểm tra xung đột MARCO

Các file **04–08** dưới đây là mẫu cho bố trí test riêng, có kiểm thử bằng
MARCO thật trên map tổng hợp; **chưa chạy trên hai robot thật**. Tọa độ dùng
frame `map`, mét/radian. Không mặc định chúng phù hợp với `my_room.yaml`:
chọn vùng trống phù hợp trên map thực tế, sửa `goal` trong JSON và file pose
khởi tạo theo vị trí thật trước khi preview. Không đặt lại pose chỉ để làm
robot trên RViz giống bố trí trong bảng.

| Kịch bản / file trong `config/marco/testcases/` | Bố trí ban đầu (x, y, yaw) | Goal và tiêu chí đạt |
| --- | --- | --- |
| **04 — Cắt ngang tại giao lộ**, `04_crossing.json` | robot1 `(-1,0,0)`; robot2 `(0,-1,1.5708)`; giao lộ có vùng tránh | robot1 → `(1,0,0)`; robot2 → `(0,1,1.5708)`. MARCO phát hiện hai đường giao nhau, chọn ưu tiên/nhường; hoàn thành task 1, 2, không cho hai path xung đột chạy đồng thời. |
| **05 — Đối đầu**, `05_head_on.json` | robot1 `(-1,0,0)`; robot2 `(1,0,3.1416)`; có vùng trống bên cạnh để tránh | Hai robot đổi vị trí. Preview phải có can thiệp `relocate` hoặc `short_replan`, sau đó robot được ưu tiên đi qua; cuối lượt hoàn thành task 1, 2. |
| **06 — Robot đứng chắn đường**, `06_idle_blocker.json` | robot1 `(-1,0,0)`; robot2 `(0,0,0)`; có chỗ tránh | Chỉ giao task robot1 → `(1,0,0)`. MARCO phải xét cả robot2 đang idle, chọn nhường đường hoặc đường vòng; task 1 hoàn thành. Robot2 vẫn có thể nhận động tác nhường dù không có task riêng. |
| **07 — Xung đột kèm dependency**, `07_conflict_dag.json` | Như 05, có chỗ tránh | Hai robot đổi chỗ (task 1, 2), rồi quay về (task 3, 4). Cả task 3 và 4 phụ thuộc **cả 1 và 2**. Chỉ bắt đầu lượt về sau khi hai task lượt đi đã hoàn thành; kết quả có đủ task 1–4. |
| **08 — Không có chỗ nhường**, `08_no_escape.json` | Như 05 nhưng trong hành lang hẹp, kín hai đầu, không đủ khoảng trống để tránh hoặc lùi ra | Hai robot đổi chỗ. Kết quả mong đợi là **từ chối kế hoạch**, không dispatch path và không di chuyển. Đây là negative test; không ép planner chạy bằng cách giảm bán kính/margin. |

Kịch bản 08 khác 05 ở **hình học map**, không chỉ ở goal JSON. Fixture phần
mềm dùng hành lang rộng khoảng 0,9 m, dài khoảng 2,7 m trước khi inflate, với
hai robot ở x=−1 và x=1; hai đầu không đủ khoảng lùi để nhường. Hành lang dài
hơn có thể cho robot lùi và lập được kế hoạch, khi đó không phải bố trí test 08.
Fixture tái lập của cả năm kịch bản nằm trong `tests/test_marco_scenarios.py`.

Ví dụ kiểm tra kịch bản cắt ngang, sau khi đã mở phiên MARCO và định vị đúng:

```bash
bash scripts/marco.sh status
bash scripts/marco.sh plan config/marco/testcases/04_crossing.json --out log/marco/testcases/04-preview.json
# Chỉ thực thi sau khi kiểm tra bố trí và đường preview thực tế:
bash scripts/marco.sh run config/marco/testcases/04_crossing.json --execute --out log/marco/testcases/04-run.json
```

Thay tên file tương ứng để thử 05–07. Với 08, chỉ gọi `plan` và kiểm tra báo
cáo `result="failed"`, `events` không có `dispatch`; **không gọi run**.
`plan` chỉ hiển thị batch đầu; khi chạy, MARCO tính tiếp từ pose quan sát sau
mỗi batch. Can thiệp nhường có `task: null`; một robot hoàn thành task vẫn có
thể phải nhường để robot kia đi tiếp. Không diễn giải danh sách task đã hoàn
thành là bảo đảm cả hai robot vẫn giữ nguyên goal cuối cùng.

Với 04–07, xem `events[].batch.audit.strategy`, thứ tự `dispatch`/`completed`
và `completed_tasks` trong báo cáo. `disjoint_routes` là đường tách biệt;
`relocate`/`short_replan` và `committed_clearance`/`committed_priority` thể hiện
can thiệp và việc giữ ưu tiên đã chọn. Nếu preview không phát hiện xung đột,
bố trí hiện tại chưa tái tạo trường hợp cần test. Chạy kiểm thử phần mềm:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -B tests/test_marco_scenarios.py -v
```

## Kiểm tra giao tiếp socket

```text
Client JSON/TCP → PC gateway → Nav2/FollowPath → DDS → Pi watchdog → Create 3
MARCO planner  → PC gateway (path + lease) ────────────┘
```

Gateway nghe **127.0.0.1:8765 trên PC**, được mở cùng phiên navigation hoặc
mapping. Mỗi kết nối gửi một JSON object kết thúc bằng `\n`, nhận một JSON
object kết thúc bằng `\n`, rồi đóng. Request tối đa **65.536 byte tính cả
newline**. Status/select/pose/goal/cancel dùng `scripts/pc-command.sh`; MARCO
dùng thêm `snapshot`, `preview`, `path`, `lease` để giữ nguyên đường đã điều phối.
Socket không truyền trực tiếp LiDAR hay lệnh motor sang Pi; phần đó qua DDS.

```bash
bash scripts/pc-command.sh status
bash scripts/pc-command.sh select robot1
bash scripts/marco.sh status
```

`Connection refused` nghĩa là chưa có gateway nghe tại host/port đó: mở phiên
PC trước. `ok=true` khi gửi goal/path chỉ là đã tiếp nhận, chưa phải tới đích;
poll status đến khi `busy=false` và kiểm tra `last_result`. Các goal độc lập
gửi bằng `pc-command.sh goal` **không đi qua coordinator MARCO**; dùng mission
`marco.sh plan/run` để thử xử lý xung đột nhiều robot.

Kiểm tra socket bằng fake Nav2/cảm biến, ROS domain riêng không nối robot thật:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -B tests/test_socket_protocol.py -v
python3 -B src/tb4_multi_bringup/test/test_native_runtime.py -v
python3 -B src/tb4_multi_bringup/test/test_path_runtime.py PathRuntimeTest.test_marco_cli_through_two_fake_hardware_chains -v
```

Các test bao phủ TCP chia gói, JSON lỗi/phản hồi thiếu newline, giới hạn kích
thước, client gửi dở dang, chọn/hủy đúng robot, chống goal chồng nhau, lease và
luồng MARCO → socket → FollowPath → watchdog với hai base giả. Kết quả phần
mềm không thay thế thử nghiệm xung đột trên phần cứng.

<details>
<summary>Tham khảo kỹ thuật: cài đặt, các script thành phần và giao thức socket</summary>

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

**Kiểm chứng 10/11/2026:** testcase 01 hai robot đi A/B đã hoàn thành trong
cùng một lượt chạy lại, sau lượt đầu bị hủy do chất lượng localization. Xem
hướng dẫn testcase phía trên. Kết quả lịch sử 31/10 ghi nhận goal riêng từng
robot; mẫu PC đứng yên với đầy đủ Nav2/RViz đạt trung bình khoảng 153% CPU,
peak 170%, không suy ra tải khi chạy goal. Xem
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
một JSON rồi đóng; giới hạn request 65.536 byte tính cả newline.

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

</details>
