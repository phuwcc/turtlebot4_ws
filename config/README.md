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
