```bash
# Pi 1 — bật hardware tự chạy sau reboot
cd ~/turtlebot4_ws
bash run.bash hardware robot1 --install-service
bash scripts/robot-service.sh robot1 status
```

```bash
# Pi 2 — bật hardware tự chạy sau reboot
cd ~/turtlebot4_ws
bash run.bash hardware robot2 --install-service
bash scripts/robot-service.sh robot2 status
```

```bash
# Trên từng Pi — kiểm tra driver/log
ls -l /dev/RPLIDAR
journalctl -u tb4-hardware.service -n 50 --no-pager
```

```bash
# PC — kiểm tra cảm biến, sửa pose thật trước khi mở đa robot
bash run.bash check
nano config/marco/initial_poses.local.yaml
```

Tham số EKF và AMCL nằm trong `src/tb4_multi_bringup/config/common/`.
AMCL nạp occupancy map gồm YAML và ảnh bản đồ.
[Hướng dẫn quét và kiểm tra định vị](../docs/REAL_ROBOT.md)
