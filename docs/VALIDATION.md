# MARCO và localization — 2026-10-31

Đã hoàn thành goal riêng của robot1 và robot2 trên phần cứng thật qua MARCO →
socket C++ → kiểm tra costmap → Nav2 FollowPath → stamper → watchdog Pi.
Đây là các lượt chạy tiếp nối; **chưa phải một lượt hai goal liên tục không lỗi**.
[Kết quả có cấu trúc](validation/2026-10-31-marco.json).

- Lượt đầu: robot1 đi 0,057 m rồi bị hủy khi variance AMCL vượt 0,01 m²;
  robot2 đứng yên. Không nới ngưỡng localization.
- Đổi ngưỡng cập nhật AMCL trong managed mode từ 0,25 m/0,2 rad xuống
  0,02 m/0,05 rad cho tốc độ thấp và goal ngắn. Không giảm motion noise model.
- Lượt tiếp: robot1 hoàn thành goal, odometry ghi 0,083 m trong lượt đó.
  Khi chuẩn bị goal mới bị từ chối vì footprint không an toàn trong map,
  chuỗi shell thử nghiệm vẫn tiếp tục mission cũ đã được cho phép; đã hủy
  khi robot2 mới đi khoảng 0,009 m. Không coi lượt này đạt toàn bộ.
- Chạy tiếp riêng goal robot2: đạt, odometry ghi 0,147 m; robot1 không dịch chuyển.
  Vận tốc lớn nhất quan sát 0,05 m/s và 0,2 rad/s. Cả hai có lệnh zero mới
  liên tục sau lượt cuối. Goal checker có tolerance 0,05 m nên quãng đường
  thực tế có thể ngắn hơn 0,20 m từ vị trí khởi tạo.

Cả hai đã được xác nhận khỏi dock và có người giám sát. Không tự undock,
không gửi Twist trực tiếp từ MARCO, không chạy BT recovery lùi/xoay.
Kết quả không chứng nhận hoạt động tự hành dài hạn hoặc độ chính xác tuyệt đối.

MARCO nguyên bản revision `bef1ac7` được giữ sạch. Bridge dùng native assignment
và coordinator DAG-aware thật; source/module/cache sinh ra đều ở ngoài MARCO.
Đã bổ sung kiểm tra lease hết hạn, permission timestamp/out-of-order, hủy khi
mất localization, gợi ý khởi tạo robot2 và giải phóng giới hạn vùng gợi ý sau
hội tụ. Đã sửa race khi goal hoàn thành giữa lúc đọc status và gia hạn lease.
Xem [cách chạy và giới hạn](MARCO.md).

Mẫu 15 giây với hai Nav2/RViz sau khi ổn định: trung bình **152,8% CPU**,
peak **170%**, tổng RSS **1321 MiB** (cộng RSS có thể đếm lặp trang chia sẻ).
User cgroup xác nhận CPU quota **200%**, memory max **2 GiB**; memory current
khoảng **553 MiB**. Đây là mẫu đứng yên, không suy ra tốc độ tăng hay đảm bảo tải
khi chạy goal. Các goal thật ở trên hoàn thành trước khi bật giới hạn cuối cùng;
đã kiểm tra lại startup, freshness, localization và Nav2 readiness với giới hạn.
Thử composition gặp segfault ở domain giả nên không triển khai cấu hình đó.
Trong startup có một lỗi bond map_server tự phục hồi và cần reseed robot2 khi
AMCL trả variance âm cỡ 1e-14. Supervisor mới chấp nhận sai số roundoff >=-1e-10,
vẫn từ chối variance âm thực sự. Log còn vài lần extrapolation vài chục ms;
không thấy `queue is full` ở phiên cuối. [Ảnh RViz hai robot, Global Status OK](marco-physical-goals.png).

PC: build 4 package; native path runtime **7/7**, planner **6/6**, fleet config
**5/5**, hardware stop **4/4**. Bộ test đầy đủ đã đạt trước các thay đổi cuối;
các bài liên quan được chạy lại sau khi sửa. Test giả dùng domain localhost riêng.
Hai Pi đã đồng bộ source, build profile hardware **3 package**, mỗi Pi đạt
**5/5 CTest** và **2/2 native watchdog/relay** trong domain cô lập. Giữ các
process hardware đang chạy; logic guard không đổi. Service `turtlebot4` cũ đã
bị disable để reboot không tự chạy lại bringup thiếu watchdog.

---

# Kiểm chứng trên PC, robot1 và robot2 — 2026-10-30

**Cập nhật: map và localization thật của cả robot1/robot2 đã hoạt động; chưa thử goal điều hướng thật.**
Không coi kết quả cảm biến giả hoặc localization đứng yên là chứng nhận tự điều hướng.
[Kết quả có cấu trúc](validation/2026-10-30.json).

| Kiểm tra | PC | Pi robot1 | Pi robot2 |
| --- | --- | --- | --- |
| Build toàn bộ repo (3 package) | Đạt | Đạt | Đạt |
| CTest C++ (controls x2, DDS graph wire, service QoS) | 4/4 | 4/4 | 4/4 |
| Python config/fleet/gateway/watchdog | Đạt | Đạt | Đạt |
| AMCL/Nav2/planner/socket/TF/SLAM với cảm biến giả | Đạt | Chạy trên PC | Chạy trên PC |
| Scan, odom, TF từ phần cứng | Nhận cả hai robot | Đạt | Đạt |
| Tiến chậm qua PC → DDS → watchdog → Create 3 | — | 0,0703 m | 0,0952 m |
| Watchdog về lệnh dừng khi ngắt luồng | — | 0,496 s | 0,519 s |
| Map/AMCL/Nav2 startup với cảm biến thật | Lifecycle active, map và TF đạt | Hardware C++ hoạt động | Hardware C++ hoạt động |

Các lần chạy thật được người dùng cho phép và giám sát, lần lượt từng robot,
ở 0,04 m/s trong tối đa 2 giây. Odometry xác nhận có dịch chuyển và đã dừng;
robot còn lại không dịch chuyển trong mỗi lượt. Không gửi goal Nav2 thật,
không tự undock hoặc thay E-Stop. Khi cả hai còn trên dock, bài thử đã từ chối chạy.

Đã gặp publisher DDS tồn dư và trường hợp watchdog không chuyển tiếp dù một
node quan sát mới nhận lệnh. Không bỏ điều kiện bảo vệ: làm sạch discovery,
build/khởi động lại hardware đồng bộ rồi thử lại; cả hai đã qua. Điều này không
chứng minh hệ thống tự phục hồi mọi lỗi mạng hoặc mọi trường hợp mất tiến trình.

Robot1 có RMW XML override trên Create 3 làm mất kết nối discovery; đã lưu bản
sao, bỏ override và restart ứng dụng ROS. Cả hai đế báo H.2.6/Humble.
Cấu hình đế và ~/.config/turtlebot4/pc_env.sh là cấu hình máy riêng, không nằm
trong bản source đồng bộ.

## Sửa lỗi map và giảm tải PC

Đã chuyển gateway socket, adapter frame RViz và watchdog/relay vận tốc sang C++.
Bài test black box kiểm tra namespace 1–3 robot, cache static TF cho RViz mở muộn,
đổi frame Path/map, giới hạn scan hiển thị, goal/cancel độc lập, đầu vào lỗi,
timeout/timestamp/out-of-order/nhiều publisher và lệnh zero khi shutdown.
Socket đã được kiểm tra cả chuỗi với Nav2 thật và cảm biến giả: goal tạo lệnh
vận tốc có timestamp, sau đó cancel trả trạng thái canceled. Domain test tách
khỏi robot thật; bài này không chứng nhận chuyển động vật lý.
Bộ test phần mềm đầy đủ trên PC đạt; native binary build thành công trên cả hai Pi. Cả hai Pi cũng đạt 5/5 bài test
black box native và 4/4 CTest, trong ROS domain cô lập.

Profile Fast DDS mới chỉ tăng `max_blocking_time` của publisher profile `service`
lên 5 giây. Nó giải quyết timeout phản hồi lifecycle gặp trên mạng thực tế này;
CTest kiểm tra XML hợp lệ, service QoS và QoS topic mặc định không đổi.
Nếu đã đặt `FASTRTPS_DEFAULT_PROFILES_FILE` hoặc `FASTDDS_DEFAULT_PROFILES_FILE`,
launch giữ nguyên profile tùy chỉnh đó; cần tự ghép cấu hình service nếu cần.
Xem [cơ chế gửi response của rmw_fastrtps](https://github.com/ros2/rmw_fastrtps/blob/humble/rmw_fastrtps_shared_cpp/src/rmw_response.cpp).
Bond vẫn bật; timeout manager tăng từ 4 lên 15 giây để chờ discovery Wi-Fi,
vì thế phát hiện mất node Nav2 chậm hơn. Watchdog vận tốc độc lập vẫn 0,5 giây.
Không thêm static TF thay thế localization hoặc odometry.

Đã kiểm tra trên map `my_room.yaml`: robot1 gần `(0,0)`, robot2 gần `(0,0.5)` m,
yaw khởi tạo 0 theo giả định cùng hướng. Median khoảng cách endpoint scan tới ô
vật cản bằng 0; khoảng 83–88% endpoint nằm trong 20 cm của ô vật cản.
Đây là phép đối chiếu scan/map khi đứng yên, không đo độ chính xác tuyệt đối.
Chưa gửi NavigateToPose thật trong đợt migration này.

RViz mặc định 10 FPS, tắt vẽ toàn bộ cây TF/local plan/footprint; giữ map và mô hình.
Adapter giới hạn scan hiển thị 5 Hz, không giảm dữ liệu AMCL/obstacle layer.
Chế độ `localization_only:=true` bỏ các node điều hướng và relay vận tốc khi chỉ
cần xem map/localization. Build tuần tự với nice/ionice và cgroup giới hạn tài nguyên.
Không có bằng chứng journal đủ để kết luận OOM hoặc lỗi GPU là nguyên nhân đóng app.
Các số đo tải là cửa sổ lấy mẫu ngắn khi đứng yên, không đảm bảo tải khi chạy goal.

Kết quả mới: [JSON native/performance](validation/2026-10-30-native.json),
[ảnh RViz hai robot và map](fleet-localization-light.png).
Trên PC này, cửa sổ lấy mẫu 20 giây sau khi ổn định: chế độ localization/RViz
trung bình **31,84% CPU**, peak **45,22%**, tổng RSS **519 MiB**;
Nav2 đầy đủ/RViz trong mẫu 15 giây trung bình **185,46%**, peak **262,57%**,
tổng RSS **1190 MiB**. 100% tương đương một CPU logic; máy có 12 CPU logic.
RSS cộng theo tiến trình có thể đếm lặp các trang chia sẻ. Không có số đo
before/after có kiểm soát cho bản cũ, nên không suy ra phần trăm tăng tốc C++.

## Chạy lại

```bash
bash scripts/build.sh
bash scripts/test.sh
```

Test phần mềm dùng ROS domain riêng, localhost và cảm biến/action server giả.
Robot3 trong test chỉ là namespace mô phỏng trên PC; phần cứng đợt này chỉ có
robot1 và robot2. Các test thật không nằm trong scripts/test.sh.

Giữ hardware trên hai Pi chạy, dừng Nav2/teleop trước khi kiểm tra từ PC:

```bash
bash scripts/pc-test-hardware.sh
# Chỉ khi đã đưa robot khỏi dock, có vùng trống và người giám sát:
bash scripts/pc-test-hardware.sh --allow-motion
```

Mặc định chỉ đọc cảm biến/TF/trạng thái. Cờ --allow-motion mới tạo publisher
lệnh, kiểm tra từng robot và kiểm tra dừng khi ngắt luồng; không vượt qua E-Stop,
không tự undock. Script dùng robot đã chọn trong pc_env.sh, khóa phiên để tránh
chạy chồng Nav2/teleop, xuất log/pc/hardware-test/result.json. Lệnh dừng và timeout
vẫn được áp dụng khi bài thử thất bại.

Log đầy đủ đợt này lưu dưới log/repo_validation/ (không commit). Mã cũ trên Pi
được giữ trong Git stash trước khi đồng bộ. Các bản sao source ban đầu nằm ở
log/repo_validation/backups/ trên PC.

---

# Ghi chép lịch sử (giữ nguyên kết quả và giới hạn tại thời điểm kiểm tra)

# Kiểm chứng kiến trúc PC navigation + socket

Ngày 22/10/2026, trong workspace ROS 2 Humble:

- Build thành công `tb4_multi_tools` và `tb4_multi_bringup`.
- 5 test cấu hình PC và 3 test fleet 1–3 robot đã qua.
- 3 test mới trong `test_remote_control.py` đã qua: bàn phím C++ phát
  TwistStamped và tự timeout; watchdog Pi dừng sau hết hạn, bỏ lệnh cũ/NaN/
  đảo thứ tự, dừng khi nhiều nguồn; TCP dùng chung goal với RViz, validate
  robot/toạ độ, publish pose, từ chối goal chồng, hủy goal và đọc kết quả.
- `tests/smoke_navigation.py --pc robot1` đã qua trên domain 172, localhost:
  chạy **launch PC kết hợp thật**, không RViz; AMCL, map server, controller,
  planner, BT navigator đều active; đặt pose qua socket thật; lập đường thành
  công (`status=4`, 8 pose). Đã kiểm tra remap controller → smoother,
  đầu ra `pc_cmd_vel` → relay `remote_cmd_vel`, PC không publish `cmd_vel`
  bỏ qua guard, không rò topic `/map` hoặc `/tf` ra namespace gốc.

Một lần chạy smoke gặp timeout phản hồi lifecycle AMCL khi khởi động trên
máy Pi; chạy lại trên domain sạch đã qua. Nếu AMCL/map server chưa active,
kiểm tra log lifecycle và kết nối DDS trước khi gửi goal. Các bài test mới
không phát lệnh tới robot thật.

Chạy lại toàn bộ kiểm thử sau khi build:

```bash
bash scripts/test.sh
```

Các bài test tự đặt domain riêng và localhost, bỏ môi trường discovery của
robot. Không dùng domain 168–174 cho robot thật trong lúc chạy test.
Log smoke gần nhất: `log/navigation_smoke/pc/result.json`,
`log/navigation_smoke/pc/navigation.log` (không commit).

Chưa chạy cấu hình mới trên máy PC case thật, chưa xác nhận DDS PC↔ba Pi,
chưa thử mất Wi-Fi hoặc navigation đồng thời trên ba robot thật. Không có màn
hình để kiểm tra trực quan RViz. Cần triển khai/build cùng phiên bản trên PC
và Pi trước khi kiểm thử thực tế. Không thay đổi dịch vụ robot đang chạy trong
lượt sửa code này.

---

## Kết quả lịch sử trước khi chuyển navigation sang PC

# Kiểm chứng cấu hình ba robot và một RViz

Đã kiểm tra trong workspace ROS 2 Humble ngày 20/10/2026:

- `bash scripts/build.sh`: build thành công hai package `tb4_multi_tools` và
  `tb4_multi_bringup`.
- `tests/test_pc_config.py`: 5 test không cần ROS đã qua: lưu/đọc lại cấu hình
  1/2/3 robot; từ chối IP trống, sai octet, trùng, loopback/multicast/broadcast;
  hỏi lại giá trị sai; chọn 2 không hỏi IP robot3; ngắt giữa chừng giữ nguyên
  file cũ; từ chối cấu hình `:11811;:11811;:11811;` từ lỗi `read`.
- `src/tb4_multi_bringup/test/test_fleet_view.py`: 3 test đã qua, gồm cấu hình RViz và kiểm thử tích hợp với ROS thật,
  TF/map/scan và action server Nav2 giả, domain 173, localhost. Đã qua kiểm tra
  chỉ mở interface/nhóm RViz của 1/2/3 robot, từ chối chọn robot ngoài phiên,
  khóa số robot sau khi node khởi động, prefix frame, giữ map chung, static TF của ba robot với subscriber mở muộn,
  map transient-local, scan robot3, pose/goal tới đúng từng namespace, ba goal
  hoạt động đồng thời, từ chối goal trùng/frame sai/robot ID sai và hủy riêng
  robot3 trong khi robot1/robot2 tiếp tục chạy.
- `tests/smoke_navigation.py robot3`: Nav2/AMCL thật với map và cảm biến giả,
  domain 170, localhost. AMCL, map server, controller, planner và BT navigator
  đều `active`; lập đường thành công (`status=4`, 8 pose), không phát map/TF ra
  topic gốc `/map` hoặc `/tf`. Không chạy hardware và không gửi goal điều khiển
  chuyển động.

Đã chạy launch với danh sách discovery sai: launch thoát trước khi mở bất kỳ
process RViz/fleet nào, kèm hướng dẫn chạy `bash scripts/laptop.sh configure`.
Build và kiểm tra cú pháp các khối Bash README cũng đã qua.

Chạy lại kiểm thử cấu hình laptop (không cần ROS):

```bash
cd "$HOME/turtlebot4_ws"
python3 tests/test_pc_config.py -v
```

Chạy lại kiểm thử fleet trong một terminal:

```bash
(
  set -e
  cd "$HOME/turtlebot4_ws"
  source /opt/ros/humble/setup.bash
  source "$HOME/turtlebot4_ws/install/setup.bash"
  python3 src/tb4_multi_bringup/test/test_fleet_view.py -v
)
```

Bài test fleet tự đặt domain 173/localhost, gỡ discovery/profile/namespace của
robot khỏi môi trường trước khi tạo node. Không dùng domain này cho robot thật
trong lúc kiểm thử. Test Nav2 cũng tự đặt domain riêng theo robot: 168/169/170.

Ngày 2026-10-29 đã mở RViz với hai robot giả và xác nhận hình ảnh mô hình,
TF, scan, AMCL pose, bản đồ và nhãn robot. Cần kiểm tra lại với robot thật.
Chưa kiểm thử kết nối DDS
qua ba Pi hoặc navigation đồng thời trên ba robot thật. Không có bộ điều phối
tránh xung đột giữa các robot.

## AMCL và mô hình đa robot — 2026-10-29

- Build hai package thành công; 5 test cấu hình PC, 4 test fleet và 3 test
  remote control đều qua.
- Smoke navigation PC: AMCL/map server/controller/planner/BT navigator đều
  active, planner trả đường 8 pose, kiểm tra routing lệnh và namespace qua.
- `tests/smoke_fleet_localization.py`: hai AMCL/Nav2 thật, robot state publisher
  với URDF TurtleBot4 thật, cảm biến giả trong domain 176/localhost. Hai robot
  nhận pose khởi động riêng x=0 và x=1 từ YAML. Mỗi robot có đầy đủ TF từ `map`
  tới 18 link có visual. Đặt lại pose robot2 về x=2 sau auto-init thành công,
  robot1 giữ nguyên pose. Không gửi goal chuyển động.
- Chạy thêm `python3 tests/smoke_fleet_localization.py --rviz` để xem cửa sổ
  kiểm thử trong 15 giây. [Ảnh RViz](fleet-localization.png) chụp từ lần chạy này.

Log hardware ngày 2026-10-29 cho thấy AMCL đã active nhưng costmap robot1
thiếu `odom -> base_link`. Kiểm thử giả không chứng nhận dữ liệu TF trên Pi;
cần kiểm tra Create 3/republisher nếu còn lỗi này trên phần cứng.
