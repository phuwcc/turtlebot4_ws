# MARCO trên TurtleBot 4 thật

MARCO được giữ nguyên tại `MARCO/` hoặc đường dẫn `MARCO_ROOT`. Adapter ngoài
repo dùng `core.assignment`, `core.coordination.coordinator` và các hàm hình học
của MARCO, không chạy launcher Gazebo/AI2-THOR hay controller mô phỏng.
Mọi cache, module native, preview và log nằm trong `build/`, `install/`, `log/`
của workspace. Worker chạy Python `-B`, không tạo `__pycache__` trong MARCO.

## Khởi động

PC cần checkout MARCO riêng (không copy sang Pi). Adapter đã kiểm tra với
revision `bef1ac7d3161fb6ce009a507beac86a06fe92371` của
[TranThang27/MARCO](https://github.com/TranThang27/MARCO). Nếu chưa có checkout,
clone repo đó vào `MARCO/`; không chạy lệnh clone đè lên thư mục đã có.

Build workspace (4 package), giữ hardware/watchdog trên hai Pi chạy theo README:

```bash
bash scripts/build.sh
# Nếu máy còn tf2/tf2_ros 0.25.23 gây kẹt bộ lọc TF:
bash scripts/setup-tf2-runtime.sh
bash scripts/pc-marco.sh maps/my_room.yaml
```

Script TF2 lấy hai gói Humble chính thức từ APT, kiểm tra phiên bản >=0.25.24 và
cùng phiên bản upstream, giải nén vào thư mục riêng trong `~/.local/share`.
Không sửa gói hệ thống; `runtime_env.sh` chỉ áp dụng cho các phiên PC của repo.
Bản sửa upstream: [geometry2 #990](https://github.com/ros2/geometry2/pull/990).

Chỉ xem localization/RViz, giảm tải và không bật các node điều hướng:

```bash
bash scripts/pc-marco.sh maps/my_room.yaml localization_only:=true
bash scripts/marco.sh status
```

Không có mission hoặc lệnh di chuyển nào được tự khởi chạy bởi `pc-marco.sh`.
Khi có user systemd, toàn phiên ROS/RViz chạy trong cgroup: CPU quota 200%
(tổng hai lõi logic), memory high 1536 MiB, hard max 2048 MiB và nice=5.
Các biến `TB4_PC_CPU_QUOTA`, `TB4_PC_MEMORY_HIGH`, `TB4_PC_MEMORY_MAX` cho phép
đổi ngân sách. `TB4_PC_RESOURCE_LIMITS=0` tắt giới hạn; thiếu user cgroup thì
script thông báo chỉ dùng nice. Quota giới hạn tài nguyên, không phải tăng tốc
thuật toán; cấu hình quá thấp có thể gây trễ và hủy goal theo freshness gate.
Không bật composition Nav2 trên bộ binary hiện tại vì thử nghiệm cô lập gặp
segfault khi bắt đầu điều khiển. Giữ các node ở tiến trình riêng.
Planner và Nav2 ở PC. Pi chỉ chạy hardware và watchdog; không cài/chạy MARCO,
map server, AMCL hoặc SLAM trên Pi. `setup.sh robot` dùng profile build hardware
(3 package, bỏ các executable chỉ dành PC); service TurtleBot cũ bị disable khi
chạy `robot-start.sh` để reboot không tự quay về bringup thiếu watchdog. Sau
reboot, chạy lại hardware bằng `robot-start.sh` trong terminal trên đúng Pi.

## Localization

Robot1 phải **đứng đúng gốc của map đã quét**, quay theo +X (yaw=0). Việc đặt
robot ở chỗ khác rồi khai báo (0,0) không thay đổi gốc của một map đã có.
Supervisor chỉ gửi initial pose có covariance; không tạo static `map -> odom`.
AMCL vẫn tính và cập nhật transform thật từ scan/odom.
Chế độ managed cập nhật AMCL sau mỗi 2 cm hoặc 0,05 rad chuyển động, phù hợp
goal ngắn và tốc độ thấp; không giảm noise model hoặc nới ngưỡng covariance.
Ý nghĩa các ngưỡng cập nhật: [Nav2 AMCL](https://docs.nav2.org/configuration/packages/configuring-amcl.html).

`config/marco/localization.yaml` hiện có gợi ý do người vận hành xác nhận:
robot2 bên trái robot1 khoảng 0,5 m, cùng hướng. Đây là phân bố khởi tạo
`(0,0.5,0)` với độ lệch chuẩn 0,2 m/radian, không phải tọa độ bị khóa.
AMCL tiếp tục ước lượng vị trí. Hãy sửa/xóa gợi ý nếu bố trí robot thay đổi.
Giới hạn vùng gợi ý chỉ kiểm tra lúc khởi tạo; sau khi đã hội tụ, robot được
đi ra khỏi vùng đó và vẫn phải đạt các điều kiện scan/TF/covariance.
Nếu khởi động lại PC sau khi robot đã đi khỏi gốc, truyền `initial_poses:=file.yaml`
(x/y/yaw trong map cho từng robot) hoặc đưa robot về vị trí khởi tạo đã khai báo.
Không chạy lại seed (0,0) cho robot1 đang đứng ở vị trí khác.
Với robot không có gợi ý, supervisor gọi global localization của AMCL, yêu cầu
no-motion updates và có thể dùng scan–map matching giới hạn thời gian để hỗ trợ
khởi tạo khi tìm được một mode đủ khác biệt. Map đối xứng/thiếu đặc trưng có
thể không xác định duy nhất vị trí khi robot đứng yên: hệ thống sẽ chờ gợi ý
hoặc **2D Pose Estimate**, không tự coi một giả thuyết bất kỳ là chính xác.

Có thể bỏ toàn bộ gợi ý cho một lần chạy:

```bash
bash scripts/pc-marco.sh maps/my_room.yaml localization_hints:=''
```

`status` chỉ báo `localized=true` khi có TF/odom/scan mới, scan khớp map >=70%,
variance x/y <=0,01 m² và yaw <=0,04 rad², liên tục trong 3 giây.
Đây là tiêu chí vận hành, không phải chứng nhận độ chính xác tuyệt đối.
Nếu chưa đạt, nhãn RViz là `LOCALIZING`; MARCO không được gửi đường thực thi.
Mất chất lượng localization sẽ thu hồi command permission; lỗi kéo dài 2 giây
sẽ pause Nav2. Debounce này tránh vòng lặp deactivation/activation khi Wi-Fi chập
chờn thoáng qua, không trì hoãn việc đóng permission hoặc watchdog.

Lifecycle khởi động có thứ tự: discovery → map server/AMCL → khởi tạo/hội tụ →
Nav2. Bond vẫn bật, timeout 40 giây (initial handshake khoảng 20 giây) và có
một lượt reset/retry có giới hạn. Thời gian phát hiện lỗi node qua bond dài hơn;
watchdog vận tốc trên Pi vẫn 0,5 giây. Trong lúc chưa có `map -> odom`, adapter
không đẩy scan hiển thị vào RViz để tránh tích queue; scan gốc AMCL vẫn giữ nguyên.

## Xem kế hoạch và thực thi

```bash
cp config/marco/mission.example.json config/marco/mission.json
# Sửa goal theo những điểm trống thực tế trên my_room; ví dụ không phải điểm đã chứng nhận an toàn.
bash scripts/marco.sh plan config/marco/mission.json --out log/marco/preview.json
```

`plan` không điều khiển robot. Nó dùng snapshot mới, MARCO phân công nhiệm vụ
và chọn batch/can thiệp đầu tiên, rồi hiển thị `MARCO reserved path` trong RViz.
Mission hỗ trợ tối đa 8 task điều hướng, `candidates` gồm robot đang hoạt động,
`depends_on` là các task ID. Không triển khai thao tác gắp/đặt đồ của simulator.
Hai đường tách biệt có thể thực thi song song; xung đột dùng coordinator DAG-aware
và `dynamic_events_guarded` của MARCO. Can thiệp được thực thi từng bước và tính
lại từ vị trí quan sát sau mỗi batch/động tác nhường đường. Sau khi chọn can thiệp,
giữ robot ưu tiên và thứ tự clearance đã chọn đến khi đi qua, tránh đổi ưu tiên
qua lại giữa các lần lập kế hoạch. Không dựa vào đồng hồ
mô phỏng để giả định robot đã đi tới nơi.

Chỉ khi localization đạt, preview phù hợp, vùng chạy trống và có người giám sát:

```bash
bash scripts/marco.sh run config/marco/mission.json --execute --out log/marco/execution.json
```

Gateway C++ nhận đường dày điểm và kiểm tra `IsPathValid` trên costmap Nav2,
kiểm tra điểm bắt đầu, rồi gửi `FollowPath`. Không đổi đường MARCO thành một
goal tự do để global planner chọn lại lộ trình khác. Không publish `cmd_vel`
trực tiếp từ MARCO. Controller → stamper giới hạn 0,05 m/s và 0,2 rad/s → Pi guard
→ Create 3. FollowPath không chạy các recovery lùi/xoay của Behavior Tree.

Mỗi path có owner token và lease 2 giây; adapter gia hạn ở tần số thấp. Mất lease,
localization không đạt, lệch hành lang dự kiến, map đổi hoặc quá thời hạn sẽ
hủy path. Permission có timestamp và hết hạn độc lập sau 0,3 giây; thiếu permission
thì stamper gửi zero. Pi vẫn kiểm tra tuổi, thứ tự và số nguồn lệnh.
Ctrl+C ở terminal mission hủy những path còn hoạt động; lease vẫn bảo vệ khi
process bị kill. Khi có vật cản mới làm đường không còn hợp lệ, nhiệm vụ dừng
và ghi lỗi để lập kế hoạch lại, không bỏ kiểm tra costmap.

Bán kính robot + safety margin + tracking tolerance được tính vào geometry;
unknown cells không được coi là vùng trống. Nếu robot đứng quá sát nhau hoặc
một goal không đủ clearance, planner có thể từ chối thay vì ép chạy.
Giới hạn theo mặc định: một worker ưu tiên thấp, 2 GiB address space, deadline
RPC 6 giây, tìm can thiệp 3 giây, tối đa 32 batch. Makespan là ước lượng cho
adapter thật, không có tuyên bố tối ưu toàn cục hay bảo đảm va chạm ngoài đời.

## Kiểm tra

```bash
bash scripts/test.sh
# Phải sạch; không build native bằng pip trực tiếp trong MARCO:
git -C MARCO status --short
```

Test dùng ROS domain localhost riêng, map/cảm biến/base giả. Có lượt đi trọn
MARCO → socket C++ → FollowPath → permission → stamper → guard → odom/TF của
hai base giả, kiểm tra DAG và dừng. Kết quả đó không thay thế thử nghiệm vật lý.
Các test phần cứng thật chỉ chạy với sự giám sát và mục tiêu đã chọn phù hợp.

Observer riêng cho goal ngắn (mặc định chỉ đọc, cần source ROS và môi trường PC):

```bash
python3 -B tests/marco_live_trial.py path/to/short-mission.json
# Chỉ thêm --execute khi đã kiểm tra preview và có người giám sát.
```

Mission này phải có 1–2 task gán riêng cho robot1/robot2, dependency tuần tự
nếu có hai task, `max_action_length_m <= 0.35`, `speed_mps <= 0.05`.
Observer kiểm tra dock, scan/odom, clearance, vận tốc, quãng đường và dừng;
nó không tạo publisher vận tốc. Xem [kết quả thực tế](VALIDATION.md).
