```bash
# PC — chuẩn bị lần đầu
cd ~/turtlebot4_ws
git submodule update --init --recursive MACRO
bash scripts/setup.sh pc
bash scripts/pc-configure.sh --robots robot1,robot2 <<'IPS'
10.11.52.230
10.11.52.243
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
# i/, tiến/lùi; j/l quay; k/Space dừng; Ctrl+C lưu map và thoát
bash mapping.bash --save-as maps/room_real
```

```bash
# PC — sửa x/y/yaw và goal đúng map thật trước khi chạy
cp config/marco/initial_poses.example.yaml config/marco/initial_poses.local.yaml
cp config/marco/testcases/01_parallel_ab.json config/marco/mission.local.json
nano config/marco/initial_poses.local.yaml
nano config/marco/mission.local.json
```

```bash
# PC — toàn bộ luồng: hardware → định vị → MARCO preview → thực thi
# Robot rời dock; pose/goal đã kiểm tra. Bỏ --execute để chỉ preview.
bash multi_robot.bash --map maps/room_real.yaml \
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
