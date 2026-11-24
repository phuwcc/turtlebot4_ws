# Mô phỏng đổi chỗ ba robot trên bản đồ phòng thật

[Bản đồ phòng đã lọc](maps/room_right.yaml) giữ phần bên phải của [bản đồ quét từ phòng thật](../maps/room_real.yaml).
[Bản dùng để đổi chỗ](maps/room_right_short_branch.yaml) khôi phục thêm
75 cm theo trục `+Y` từ các ô trống của bản đồ quét để đủ chỗ nhường.
Trong Gazebo, nền xám kéo qua đầu vách cũ. Dùng lớp
**Map** trong RViz để xem biên giữa vùng trống và vùng chưa biết.

[File pose ban đầu](maps/room_right_poses.yaml) đặt `robot1`, `robot2` và
`robot3` lần lượt ở A, B và C. [Mission đổi chỗ](room_right_branch_mission.json)
cho `robot3` lùi trước. Sau đó MARCO tự chọn thứ tự và điểm nhường cho
ba đích độc lập B, C, A, cùng cách giao đích như bài `t_junction`.
Cuối mission, `robot2` trở lại C nếu đã rời chỗ để nhường.

Chạy từ thư mục gốc workspace đã build, với ROS 2 Humble, Gazebo Fortress,
TurtleBot4 Simulator và Nav2. Lệnh dùng hai script thử nghiệm cục bộ
`log/sim_auto_marco/check.py` và `log/sim_auto_marco/world.launch.py`.
Hai file này nằm trong thư mục Git bỏ qua và chỉ có trên workspace đã
dùng để thử nghiệm.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 -u -B log/sim_auto_marco/check.py 3 \
  --tuned --lookahead 0.12 --gui --reverse-first --retry-on-stop \
  --map sim/maps/room_right_short_branch.yaml \
  --poses sim/maps/room_right_poses.yaml \
  --mission sim/room_right_branch_mission.json
```

Phiên chạy dùng ROS domain cục bộ 184, tự khởi động AMCL từ pose ban đầu,
chờ định vị sẵn sàng rồi giao đường cho Nav2. `--reverse-first` cho
`robot3` lùi ở bước đầu, sau đó trả bộ điều khiển về chế độ chạy tiến.
`--lookahead 0.12` đặt khoảng nhìn trước của Nav2 trong mô phỏng là 12 cm.
Mission đặt điểm kết thúc lượt lùi của `robot3` cách C 60 cm dọc nhánh.
`safety_margin` là 1 cm. Với bán kính 17,5 cm và dung sai bám đường 6 cm,
khoảng cách cơ sở giữa hai tâm robot là 49 cm trước khi tính phần đệm
theo ô lưới. Ngưỡng giám sát lệch đường
của MARCO là 6 cm. Với `--gui`, hai cửa sổ giữ nguyên sau khi mission
thành công. Nhấn **Ctrl+C** trong terminal chạy lệnh để tắt cả phiên.

Sau bước lùi, `--retry-on-stop` cho phép lập lại đường cho các bước còn
thiếu tối đa ba lần khi MARCO dừng do lệch đường hoặc định vị. Mỗi lần thử
tiếp chỉ bắt đầu khi cả ba robot đã đứng yên và định vị sẵn sàng.

Terminal in đường dẫn thư mục log. Xem kết quả từng bước trong
`reverse-run.json`, `run.json` và `retry-*-run.json` nếu có lần thử lại.
Kết quả kiểm tra cuối nằm trong `summary.json`.

- `CorridorViolation` báo pose lệch khỏi đường đã giữ chỗ. Một pose có
  thể vẫn nằm trên ô trống của bản đồ khi MARCO dừng vì vượt ngưỡng này.
- `LocalizationUnavailable` kèm `amcl_not_converged` báo định vị AMCL
  chưa đạt điều kiện để chạy tiếp.

## Kết quả thử bản nhánh ngắn

`robot3` đã lùi dọc nhánh, `robot1` tới B và `robot2` tới C. Khi hai
robot trong nhánh nhường chỗ lần cuối để `robot3` về A, MARCO chặn
đường với lỗi `Path enters a held peer corridor`. Lỗi lặp lại khi thử
riêng đoạn cuối với `safety_margin` 1 cm. Chưa xác nhận hoàn tất bài
đổi chỗ trong Gazebo. [Log lượt đầu](../log/sim_auto_marco/task2-152924/retry-1-run.json) · [Log thử đoạn cuối](../log/sim_auto_marco/task2-153420/run.json)

## Lỗi đã ghi nhận khi thử bản nhánh dài

| Lỗi trong log | Thời điểm xảy ra | Log cục bộ |
| --- | --- | --- |
| `CorridorViolation (0.065 m > 0.060 m)` | Bước 2. `robot3` đã lùi xong và vừa quay để tiến sang điểm chờ E. | [run.json](../log/sim_auto_marco/task2-143656/run.json) |
| `CorridorViolation (0.072 m > 0.060 m)` | Bước 2. `robot3` đang tiến gần điểm chờ E. | [run.json](../log/sim_auto_marco/task2-144609/run.json) |
| `CorridorViolation (0.064 m > 0.060 m)` | Bước 7. `robot3` đã tới gần A và đang quay tại chỗ để căn hướng đích. Hướng đích A sau đó được đổi theo hướng đi vào. | [run.json](../log/sim_auto_marco/task2-144152/run.json) |
| `LocalizationUnavailable` / `amcl_not_converged` | `robot3` đang vào E ở bước 2 hoặc `robot2` bắt đầu rời F ở bước 8, tùy lượt thử. | [Log robot3](../log/sim_auto_marco/task2-144029/run.json), [log robot2](../log/sim_auto_marco/task2-142000/run.json) |
| `MARCO: No route without displacing the higher-priority robot` | Lượt gần nhất hoàn thành bước 1–4. MARCO dừng khi lập đường cho `robot3` từ E về cửa nhánh ở bước 5. | [run.json](../log/sim_auto_marco/task2-144812/run.json) |

Các log trên nằm trong `log/sim_auto_marco/` của máy đã chạy mô phỏng.
Để đối chiếu pose tại lúc dừng, xem sự kiện `corridor_violation` hoặc
`localization_unavailable` trong `run.json` và mục `after.robots` trong
`summary.json` cùng thư mục.
