# TurtleBot 4 và MARCO trên robot thật

## Chuẩn bị PC

Chạy cấu hình IP khi thiết lập lần đầu hoặc khi IP của Pi thay đổi. Sau khi kéo bản mới, chạy bước build trước khi mở RViz.

```bash
cd ~/turtlebot4_ws
test -f MACRO/src/core/assignment_cpp.py || git submodule update --init --recursive MACRO
bash scripts/setup.sh pc
bash scripts/pc-configure.sh --robots robot1,robot2 <<'IPS'
10.11.52.230
10.11.62.202
IPS
bash run.bash hardware
bash run.bash check
```

## Bản đồ và định vị

AMCL định vị mỗi robot trên bản đồ `.yaml` + `.pgm` có sẵn. EKF chạy trên PC, fuse odom bánh xe với yaw rate từ IMU. Có thể dùng ngay `maps/room_real.yaml` cho Task 2. Khi đổi bản đồ, kiểm tra lại pose đầu và tọa độ đích trên RViz. [Kiến trúc và các lệnh kiểm tra](docs/REAL_ROBOT.md)

## Task 1 — Hai robot tới vùng xanh

Đặt hai robot về vị trí đầu trong `config/marco/room_green_start.yaml`, rời dock và kiểm tra hướng trên RViz. Hai đích trong `room_green_60cm.json` cách nhau **60 cm giữa hai tâm**. `robot1` được ưu tiên và nhận điểm sâu hơn trong vùng xanh. MARCO tính lượt đi và nhường trước khi gửi từng đường `FollowPath` qua bộ điều khiển `CorridorFollow`.

**Terminal 1** mở RViz và xem trước lượt đầu của mission. Giữ terminal này mở.

```bash
cd ~/turtlebot4_ws
bash multi_robot.bash --marco --keep-alive \
  --map config/marco/room_green_map.yaml \
  --poses config/marco/room_green_start.yaml \
  --mission config/marco/room_green_60cm.json
```

RViz hiển thị pose, LiDAR, footprint và đường MARCO của từng robot. Chờ cả hai báo `READY` và đối chiếu pose với vị trí thật. Dùng **2D Pose Estimate** cho robot đang được chọn nếu cần cập nhật pose gần đúng.

**Terminal 2** chạy mission khi lối đi và vùng đích trống, có người giám sát. `check --drive` chỉ đọc trạng thái. Kết quả cần có `drive_ready: true`, `docked: false` và `wheels_enabled: true` cho cả hai robot. Nếu bánh đang khóa, chọn **EStop** trên màn hình của robot đó hoặc nhấn ngắn nút **Power** trên đế, rồi chạy lại cụm lệnh.

```bash
cd ~/turtlebot4_ws
bash run.bash check --drive && \
  bash run.bash mission config/marco/room_green_60cm.json --execute
```

Nhấn **Ctrl+C** tại terminal chạy mission để hủy đường đang chạy. Khi đã kết thúc, nhấn **Ctrl+C** ở terminal RViz. Log phiên và mission nằm trong `log/real/`.

## Task 2 — Hai robot đổi chỗ đối đầu

Đặt robot1 tại vị trí khởi tạo của robot2 trong Task 1 và quay thân robot **180°**. Đặt robot2 ngay phía trước robot1, hướng mặt về robot1 và cách **1,15 m giữa hai tâm**. File `room_swap_start.yaml` chứa hai pose khởi tạo tương ứng. Hai robot cần rời dock và có khoảng trống bên cạnh để nhường đường. Đối chiếu pose với vị trí thật trên RViz và dùng **2D Pose Estimate** nếu cần tinh chỉnh.

**Terminal 1** mở phiên MARCO để định vị.


```bash
cd ~/turtlebot4_ws
bash multi_robot.bash --marco --map maps/room_real.yaml \
  --poses config/marco/room_swap_start.yaml
```

Khi cả hai đã định vị ổn định và đứng yên, dùng **Terminal 2** để lưu A từ pose của robot1 và B từ pose của robot2. Lệnh kiểm tra mỗi robot hướng về robot còn lại trong ±45°. Mission giữ nguyên A/B đã chụp trong suốt lượt chạy. Nếu bố trí lại hai robot, chụp vào một file mới.

```bash
cd ~/turtlebot4_ws
python3 scripts/capture_marco_swap.py log/marco/swap-ab-amcl-ekf.json
```

Ở **Terminal 1**, dừng phiên vừa dùng để chụp pose bằng **Ctrl+C**, rồi mở lại với mission để Nav2 áp dụng tốc độ **0,10 m/s** và hiện đường preview trên RViz. Nếu bạn vừa chỉnh pose trên RViz, cập nhật pose đầu tương ứng trong `room_swap_start.yaml` trước khi mở lại. Giữ terminal này mở.

```bash
cd ~/turtlebot4_ws
bash multi_robot.bash --marco --keep-alive --map maps/room_real.yaml \
  --poses config/marco/room_swap_start.yaml --mission log/marco/swap-ab-amcl-ekf.json
```

Khi preview hợp lệ, đường trống và có người giám sát, chạy ở **Terminal 2**.

```bash
cd ~/turtlebot4_ws
bash run.bash check --drive && \
  bash run.bash mission log/marco/swap-ab-amcl-ekf.json --execute
```

Kỳ vọng robot2 nhường đường khi hai lộ trình xung đột, robot1 tới B trước và robot2 tới A sau. Preview hiển thị đường của lượt đầu. Nhấn **Ctrl+C** ở terminal chạy mission để hủy.

Nếu một robot vượt hành lang 6 cm hoặc dữ liệu định vị quá hạn trong đoạn nhường, MARCO hủy đoạn đó và ghi chi tiết vào `execution.json`. Khi cả hai đã đứng yên và định vị ổn định, MARCO tính lại đường tối đa một lần với A/B đã lưu. Sau khi build bản sửa, mở lại phiên RViz để nạp gateway và bộ giám sát định vị mới.

## Chọn đích trực tiếp trên RViz

Mở chế độ MARCO tương tác để dùng hai nút **Goal robot1** và **Goal robot2** (phím **1** và **2**). Bấm tại đích, kéo theo hướng quay rồi thả để gửi goal cho robot đã chọn. RViz hiển thị mũi tên đích, đường đi và trạng thái điều phối.

```bash
cd ~/turtlebot4_ws
bash multi_robot.bash --marco \
  --map config/marco/room_green_map.yaml \
  --poses config/marco/room_green_start.yaml
```

Dùng `--rviz-goals` thay cho `--marco` khi cần thử NavigateToPose riêng từng robot.

## Dừng phần cứng

```bash
bash run.bash stop
```
