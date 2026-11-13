```bash
# PC
cd ~/turtlebot4_ws
bash run.bash hardware
bash run.bash check
```

```bash
# Sửa pose/goal cho map và bố trí thật
nano config/marco/initial_poses.local.yaml
nano config/marco/mission.local.json
bash multi_robot.bash --map maps/room_real.yaml --poses config/marco/initial_poses.local.yaml
```

```bash
# PC, terminal thứ hai — preview rồi chạy
bash run.bash mission config/marco/mission.local.json
bash run.bash mission config/marco/mission.local.json --execute
```

```bash
# Ctrl+C tại terminal mission để hủy; Ctrl+C tại terminal ROS để đóng phiên
bash run.bash stop
```
