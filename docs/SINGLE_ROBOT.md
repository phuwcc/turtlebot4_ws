```bash
# PC — chọn robot1
cd ~/turtlebot4_ws
bash scripts/pc-configure.sh --robots robot1 <<'IPS'
10.11.52.230
IPS
bash run.bash hardware robot1
bash run.bash check robot1
```

```bash
# Quét map; Ctrl+C để lưu và thoát
bash mapping.bash --save-as maps/room_real
```

```bash
# Hoặc SLAM và bàn phím ở hai terminal PC riêng
bash mapping.bash --no-teleop --save-as maps/room_real
```

```bash
# Terminal PC thứ hai
bash run.bash teleop robot1
```

```bash
# Dừng robot1
bash run.bash stop robot1
```
