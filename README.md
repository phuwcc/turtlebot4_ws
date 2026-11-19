```bash
# PC — bật hardware và kiểm tra hai robot
cd ~/turtlebot4_ws
bash run.bash hardware
bash run.bash check
```

```bash
# PC — quét map bằng robot1; i/, tiến/lùi, j/l quay, k/Space dừng
# Rời dock, giữ yên lúc khởi tạo. Ctrl+C lưu maps/room_real.yaml + maps/room_real.pgm.
bash mapping.bash
```

```bash
# PC — sửa pose và goal theo map/vị trí thật
cp config/marco/initial_poses.example.yaml config/marco/initial_poses.local.yaml
cp config/marco/testcases/01_parallel_ab.json config/marco/mission.local.json
nano config/marco/initial_poses.local.yaml
nano config/marco/mission.local.json
```

```bash
# PC — mở đa robot + MARCO bằng maps/room_real.yaml; kiểm tra pose trên RViz
bash multi_robot.bash --poses config/marco/initial_poses.local.yaml
```

```bash
# PC, terminal thứ hai — preview rồi chạy mission
bash run.bash mission config/marco/mission.local.json
bash run.bash mission config/marco/mission.local.json --execute
```

```bash
# PC — dừng hardware
bash run.bash stop
```

RViz có các nút **Goal robot1**, **Goal robot2** (phím tắt **1**, **2**).
Khi navigation đã sẵn sàng, chọn nút của robot rồi bấm giữ tại đích trên map,
kéo để chọn hướng và thả chuột để gửi goal. Mũi tên và nhãn goal của từng robot
có màu tương ứng; đường global/local plan hiển thị trong nhóm robot đó.
Chế độ `localization_only:=true` chỉ định vị, chưa nhận goal navigation.
Khi lái bằng goal RViz, thêm `localization_profile:=navigation` vào lệnh khởi động
PC để dùng kiểm tra AMCL phù hợp navigation thông thường. Mặc định `strict`
giữ các ngưỡng định vị của bài kiểm thử MACRO.
Profile `config/navigation/nav2.yaml` dành cho goal RViz/MARCO: quay theo hướng
đường trước khi tiến, tối đa 0,16 m/s và 0,60 rad/s (profile controller ROS của
MARCO), inflation nhỏ 0,25 m (cost scaling 10). Dùng cùng
`localization_profile:=navigation`; profile MACRO vẫn giữ tốc độ kiểm thử riêng.

```bash
# Goal RViz đi thẳng Nav2 (mỗi robot độc lập, đổi goal giữa chừng được)
bash multi_robot.bash --poses config/marco/initial_poses.local.yaml --rviz-goals
# Goal RViz đi qua MARCO (điều phối xung đột, robot1 ưu tiên)
bash multi_robot.bash --poses config/marco/initial_poses.local.yaml --marco
```

Cả hai chế độ bật `peer_footprints:=true`: node C++ `fleet_footprints` (bản
native của `MACRO/src/simulations/ros2/fleet_obstacles.py`) vẽ footprint tròn
0,171 m + 0,01 m của robot kia vào `/robotN/fleet_map` cho costmap; AMCL vẫn dùng
map gốc. RViz hiện vòng footprint, footprint costmap và global costmap (inflation).
MARCO chỉ lập kế hoạch khi mỗi robot cách vật cản trên map hơn ~0,34 m
(0,175 thân + 0,05 an toàn + 0,08 bám path); nếu báo "footprint overlaps
unknown/occupied" thì đưa robot ra chỗ thoáng hơn trước.

Thêm `coordinated_goals:=true` khi mở PC navigation để đưa hai nút Goal qua
MARCO thay vì gửi NavigateToPose trực tiếp. Robot1 có ưu tiên cao hơn robot2;
đường tách biệt được chạy đồng thời, khi xung đột robot2 phải chờ/nhường.
Đổi goal trong lúc chạy sẽ dừng các path cũ rồi lập lại kế hoạch từ pose mới.
Trạng thái, goal và đường MARCO hiển thị trên RViz; không có phương án hợp lệ
thì robot giữ dừng và RViz hiện lỗi. Bộ điều phối giữ khóa mission và lease path.
