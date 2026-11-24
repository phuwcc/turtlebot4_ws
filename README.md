# TurtleBot 4 và MARCO trên robot thật

## Chuẩn bị PC

Chạy cấu hình IP khi thiết lập lần đầu hoặc khi IP của Pi thay đổi. Sau khi kéo bản mới, chạy bước build trước khi mở RViz.

```bash
cd ~/turtlebot4_ws
test -f MACRO/src/core/assignment_cpp.py || git submodule update --init --recursive MACRO
bash scripts/setup.sh pc
bash scripts/pc-configure.sh --robots robot1,robot2,robot3 <<'IPS'
10.11.101.38
10.11.101.39
10.11.101.40
IPS
```

Khi cần khởi động và kiểm tra cả ba robot, chạy `bash run.bash hardware` rồi `bash run.bash check`.

## Quét bản đồ bằng `robot1`

Nếu đây là lần đầu PC kết nối SSH tới IP mới, chạy `ssh -i ~/.ssh/id_ed25519_tb4_debug ubuntu@10.11.101.38 true` và đối chiếu fingerprint trước khi xác nhận khóa máy chủ.

Đóng phiên Nav2/MARCO nếu đang mở. Đưa `robot1` ra khỏi dock và đặt robot đứng yên ở vị trí muốn lấy làm gốc bản đồ. Giữ `robot2` và `robot3` ngoài khu vực quét. Chạy lệnh dưới đây trong terminal trên PC.

```bash
cd ~/turtlebot4_ws
bash mapping.bash --robot robot1 --save-as maps/room_scan_r1
```

Chờ terminal báo **SLAM sẵn sàng** rồi dùng bàn phím tại terminal đó. Nhấn `i` để tiến, `,` để lùi, `j`/`l` để quay và `k` hoặc `Space` để dừng. Lái chậm và quan sát bản đồ trong RViz. Khi quét xong, dừng robot và nhấn **Ctrl+C** để lưu `maps/room_scan_r1.yaml` và `maps/room_scan_r1.pgm`.

## Bản đồ và định vị

AMCL định vị mỗi robot trên bản đồ `.yaml` + `.pgm` có sẵn. EKF chạy trên PC, fuse odom bánh xe với yaw rate từ IMU. Cả hai bài đổi chỗ dùng bản quét hiện tại của `maps/room_real.yaml`. Khi đổi bản đồ, cập nhật pose đầu và kiểm tra lại tọa độ đích trên RViz. [Kiến trúc và các lệnh kiểm tra](docs/REAL_ROBOT.md)

## Task 1 — Hai robot đổi chỗ đối đầu

Chọn `robot1` và `robot2` trên PC. Cấu hình này cũng dùng cho ví dụ chọn đích trực tiếp trên RViz.

```bash
cd ~/turtlebot4_ws
bash scripts/pc-configure.sh --robots robot1,robot2 <<'IPS'
10.11.101.38
10.11.101.39
IPS
```

Đặt hai robot đối đầu, cách **1,15 m giữa hai tâm** theo `config/marco/room_swap_start.yaml`. Hai robot cần rời dock và có khoảng trống bên cạnh để nhường đường. Đối chiếu pose với vị trí thật trên RViz và dùng **2D Pose Estimate** nếu cần tinh chỉnh.

**Terminal 1** mở phiên MARCO để định vị.


```bash
cd ~/turtlebot4_ws
bash multi_robot.bash --marco --map maps/room_real.yaml \
  --poses config/marco/room_swap_start.yaml
```

Khi cả hai đã định vị ổn định và đứng yên, dùng **Terminal 2** để lưu A từ pose của robot1 và B từ pose của robot2. Lệnh kiểm tra mỗi robot hướng về robot còn lại trong ±45°. Mission giữ nguyên A/B đã chụp trong suốt lượt chạy. Nếu bố trí lại hai robot, chụp vào một file mới.

```bash
cd ~/turtlebot4_ws
python3 scripts/capture_marco_swap.py log/marco/task1-swap.json
```

Ở **Terminal 1**, dừng phiên vừa dùng để chụp pose bằng **Ctrl+C**, rồi mở lại với mission để Nav2 áp dụng tốc độ **0,10 m/s** và hiện đường preview trên RViz. Nếu bạn vừa chỉnh pose trên RViz, cập nhật pose đầu tương ứng trong `room_swap_start.yaml` trước khi mở lại. Giữ terminal này mở.

```bash
cd ~/turtlebot4_ws
bash multi_robot.bash --marco --keep-alive --map maps/room_real.yaml \
  --poses config/marco/room_swap_start.yaml --mission log/marco/task1-swap.json
```

Khi preview hợp lệ, đường trống và có người giám sát, chạy ở **Terminal 2**.

```bash
cd ~/turtlebot4_ws
bash run.bash check --drive && \
  bash run.bash mission log/marco/task1-swap.json --execute
```

Kỳ vọng robot2 nhường đường khi hai lộ trình xung đột, robot1 tới B trước và robot2 tới A sau. Preview hiển thị đường của lượt đầu. Nhấn **Ctrl+C** ở terminal chạy mission để hủy.

Nếu một robot vượt hành lang 6 cm hoặc dữ liệu định vị quá hạn trong đoạn nhường, MARCO hủy đoạn đó và ghi chi tiết vào `execution.json`. Khi cả hai đã đứng yên và định vị ổn định, MARCO tính lại đường tối đa một lần với A/B đã lưu. Sau khi build bản sửa, mở lại phiên RViz để nạp gateway và bộ giám sát định vị mới.

## Task 2 — Ba robot đổi chỗ theo vòng

`robot1` đi từ A sang B, `robot2` từ B sang C và `robot3` từ C về A.
A/B/C lấy từ pose thực tế lúc bắt đầu. MARCO ưu tiên `robot1 → robot3 → robot2`
và chọn điểm nhường trong vùng trống đã quét. Nhánh phía sau cho `robot3`
chỗ lùi, còn khả năng đổi chỗ phụ thuộc vào khoảng trống ở các điểm nhường.

**Terminal 1** chọn đội ba robot và mở RViz. File `room_cycle_start.yaml`
chứa pose khởi tạo trên bản đồ hiện tại. Sau khi quét bản đồ khác hoặc đổi bố
trí, cập nhật tọa độ và hướng thực tế trước khi dùng lại file này.

```bash
cd ~/turtlebot4_ws
bash scripts/pc-configure.sh --robots robot1,robot2,robot3 <<'IPS'
10.11.101.38
10.11.101.39
10.11.101.40
IPS
bash run.bash check --drive && \
  TB4_PC_RESOURCE_LIMITS=0 LIBGL_ALWAYS_SOFTWARE=1 \
    bash multi_robot.bash --marco --map maps/room_real.yaml \
    --poses config/marco/room_cycle_start.yaml
```

Giữ phiên này mở. Phiên MARCO ba robot khởi động các robot lần lượt và mặc
định chạy ở **0,10 m/s**. Đối chiếu vị trí và hướng của từng robot trên RViz.
Dùng **2D Pose Estimate** nếu cần chỉnh pose và chờ cả ba định vị ổn định.

**Terminal 2** chụp A/B/C vào một thư mục mới, rồi kiểm tra toàn bộ nhiệm vụ
với các pose hiện tại.

```bash
cd ~/turtlebot4_ws
mkdir -p log/marco
task2_dir=$(mktemp -d log/marco/task2-cycle.XXXXXX)
python3 scripts/capture_marco_swap.py --three-robots "$task2_dir/mission.json" && \
  bash scripts/marco.sh plan "$task2_dir/mission.json" --full \
    --out "$task2_dir/preview.json"
```

Chỉ chạy lệnh dưới đây khi `preview.json` có kết quả `full_preview`, khu vực
chạy trống và có người giám sát. Kiểm tra toàn bộ dùng pose cuối dự kiến
của mỗi lượt. RViz hiển thị đường của lượt đầu, rồi cập nhật đường khi
MARCO điều phối từ pose đo được. Nếu kiểm tra báo thiếu đường nhường,
giữ robot đứng yên và xem lỗi trong `preview.json`.

Với bố trí hiện tại trên bản đồ quét, lần kiểm tra thực tế đã chụp đủ A/B/C
và dừng ở preview với lỗi `Branch-and-Bound found no complete
conflict-free relocation plan`. Đoạn nhánh sâu cho một robot lùi, nhưng
MARCO chưa tìm được đường để hoàn tất cả ba đích trong bố trí này.

```bash
python3 -c 'import json, sys; sys.exit(json.load(open(sys.argv[1]))["result"] != "full_preview")' "$task2_dir/preview.json" && \
  bash run.bash check --drive && \
  bash scripts/marco.sh run "$task2_dir/mission.json" --execute \
    --out "$task2_dir/execution.json"
```

Nhấn **Ctrl+C** ở Terminal 2 để hủy nhiệm vụ. Đóng Terminal 1 bằng **Ctrl+C**
để tắt phiên MARCO, Nav2 và RViz. Khi bố trí lại robot, chụp A/B/C vào một
thư mục mới.

## Chọn đích trực tiếp trên RViz

Mở chế độ MARCO tương tác để dùng hai nút **Goal robot1** và **Goal robot2** (phím **1** và **2**). Bấm tại đích, kéo theo hướng quay rồi thả để gửi goal cho robot đã chọn. RViz hiển thị mũi tên đích, đường đi và trạng thái điều phối.

```bash
cd ~/turtlebot4_ws
bash multi_robot.bash --marco \
  --map maps/room_real.yaml \
  --poses config/marco/room_swap_start.yaml
```

Dùng `--rviz-goals` thay cho `--marco` khi cần thử NavigateToPose riêng từng robot.

## Dừng phần cứng

Lệnh này dừng các robot trong cấu hình PC hiện tại. Để dừng cả ba, chọn cấu hình ba robot ở phần Chuẩn bị PC trước khi chạy lệnh.

```bash
bash run.bash stop
```
