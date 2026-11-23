# TurtleBot 4 thật với AMCL và EKF

## Định vị trên PC

Mỗi robot có một EKF `robot_localization` ở chế độ 2D, một AMCL, một map server và một Nav2. Lifecycle manager khởi động map server và AMCL. Bộ giám sát kích hoạt Nav2 sau khi pose đầu hợp lệ, scan khớp map, IMU/odom/EKF còn mới và TF sẵn sàng.

| Dữ liệu | Nguồn và nơi sử dụng |
| --- | --- |
| `/robotN/odom` | Create 3 republisher. Adapter lấy vận tốc tiến `vx` cho EKF. |
| `/robotN/imu` | Create 3 republisher. Adapter dùng các trục gyro đã đối chiếu với `imu_link` trong URDF Standard. EKF dùng yaw rate qua TF `base_link → imu_link`. |
| `/robotN/odometry/filtered` | EKF 2D. Nav2 và gateway đọc odometry này. |
| `/robotN/scan` | LiDAR của robot đó. AMCL và costmap Nav2 dùng cùng scan. |
| `/robotN/pose` | Pose đã scan-match trong frame `map` từ AMCL. RViz và các lớp footprint sử dụng dữ liệu này. |
| `/robotN/map` | Nav2 map server nạp occupancy grid tĩnh. Các robot dùng cùng file YAML để MACRO có map ổn định. |

EKF phát `robotN/odom → robotN/base_link` và AMCL phát `map → robotN/odom` trên `/robotN/navigation/tf`. Adapter loại các cạnh odom gốc của Create 3 khỏi cây navigation và giữ TF thiết bị. Pi tiếp tục dùng các topic và TF gốc cho phần cứng và mapping. Gateway gộp TF các robot vào `/tb4_view/tf` cho RViz và đọc pose trong `map` cho MARCO. Mỗi robot dùng scan và TF thiết bị trên namespace riêng.

EKF chỉ fuse `vx` và gyro Z. Giá trị covariance dương từ nguồn được giữ lại. Create 3 đang báo covariance bằng 0, nên adapter đặt variance cấu hình trong `config/common/ekf.yaml`. Reset pose định vị qua **2D Pose Estimate** trên RViz hoặc gửi pose tương đối với `map` bằng gateway. Chờ robot báo sẵn sàng trở lại trước khi gửi goal.

## Chuẩn bị, quét map và chạy

```bash
# PC — chuẩn bị lần đầu
cd ~/turtlebot4_ws
git submodule update --init --recursive MACRO
bash scripts/setup.sh pc
bash scripts/pc-configure.sh --robots robot1,robot2 <<'IPS'
10.11.52.230
10.11.62.202
IPS
```

```bash
# Pi 1 — chuẩn bị và bật hardware tự chạy sau reboot
cd ~/turtlebot4_ws
bash scripts/setup.sh robot
bash run.bash hardware robot1 --install-service
```

```bash
# Pi 2 — chuẩn bị và bật hardware tự chạy sau reboot
cd ~/turtlebot4_ws
bash scripts/setup.sh robot
bash run.bash hardware robot2 --install-service
```

```bash
# PC — chạy hằng ngày
cd ~/turtlebot4_ws
bash run.bash hardware
bash run.bash check
```

```bash
# PC — robot1 rời dock, giữ yên khi lấy gốc map
# i/, tiến/lùi; j/l quay; k/Space dừng; Ctrl+C lưu occupancy map
bash mapping.bash --robot robot1 --save-as maps/room_real
```

```bash
# PC — sửa x/y/yaw và goal đúng map thật trước khi chạy
cp config/marco/initial_poses.example.yaml config/marco/initial_poses.local.yaml
cp config/marco/testcases/01_parallel_ab.json config/marco/mission.local.json
nano config/marco/initial_poses.local.yaml
nano config/marco/mission.local.json
```

```bash
# PC — hardware → định vị → MARCO preview → thực thi
# Robot rời dock; pose/goal đã kiểm tra. Bỏ --execute để chỉ preview.
bash multi_robot.bash --marco --map maps/room_real.yaml \
  --poses config/marco/initial_poses.local.yaml \
  --mission config/marco/mission.local.json --execute
```

```bash
# PC — dừng hardware; xem log trên Pi 1/Pi 2 khi cần
bash run.bash stop
```

```bash
# Trên từng Pi
journalctl -u tb4-hardware.service -n 50 --no-pager
```

## Lưu lại và kiểm tra

Khi phiên mapping còn mở và robot đứng yên, có thể lưu thêm một bản từ Terminal 2:

```bash
cd ~/turtlebot4_ws
bash run.bash save-map maps/room_real --robot robot1
```

Giữ `.yaml` và `.pgm` của cùng một lần lưu. Đặt pose đầu và đích theo hệ tọa độ của bản đồ đó.

Để mở RViz và kiểm tra định vị riêng trước khi mở Nav2:

```bash
cd ~/turtlebot4_ws
bash run_localization.sh maps/room_real.yaml initial_poses:=config/marco/initial_poses.local.yaml
```

Trong Terminal khác:

```bash
cd ~/turtlebot4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source "$HOME/.config/turtlebot4/pc_env.sh"
ros2 node list --no-daemon
python3 scripts/socket_client.py status
bash scripts/marco.sh status
for robot in ${TB4_ROBOT_IDS//,/ }; do
  ros2 topic info "/$robot/scan" --verbose
  ros2 topic echo "/$robot/odometry/filtered" --once
  ros2 topic echo "/$robot/pose" --once
  ros2 topic info "/$robot/navigation/tf" --verbose
  ros2 topic echo "/$robot/localization_status" --qos-durability transient_local --once
  timeout 5s ros2 run tf2_ros tf2_echo map "$robot/base_link" --ros-args \
    -r "/tf:=/$robot/navigation/tf" -r "/tf_static:=/$robot/navigation/tf_static"
done
```

Chờ cả hai robot có `localized: true` và pose khớp mặt bằng trên RViz. Đóng phiên định vị riêng trước khi mở phiên Nav2/MARCO.

Các log thử nghiệm cũ trong `docs/validation/` mô tả các phiên trước khi chuyển đổi. Các demo độc lập trong submodule `MACRO/` dùng launch và dữ liệu riêng.

Tham khảo [AMCL Humble](https://github.com/ros-navigation/navigation2/tree/humble/nav2_amcl) và [tham số EKF Humble](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/params/ekf.yaml).
