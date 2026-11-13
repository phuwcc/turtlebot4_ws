```bash
# PC — chuẩn bị pose và goal thật
cd ~/turtlebot4_ws
cp config/marco/initial_poses.example.yaml config/marco/initial_poses.local.yaml
cp config/marco/testcases/01_parallel_ab.json config/marco/mission.local.json
nano config/marco/initial_poses.local.yaml
nano config/marco/mission.local.json
```

```bash
# Mở hardware + AMCL/Nav2 + RViz; kiểm tra pose hai robot
bash multi_robot.bash --map maps/room_real.yaml --poses config/marco/initial_poses.local.yaml
```

```bash
# Terminal PC thứ hai — MARCO preview rồi thực thi
bash run.bash mission config/marco/mission.local.json
bash run.bash mission config/marco/mission.local.json --execute
```

```bash
# Hoặc chạy toàn bộ bằng một lệnh sau khi đã kiểm tra pose/goal
bash multi_robot.bash --map maps/room_real.yaml \
  --poses config/marco/initial_poses.local.yaml \
  --mission config/marco/mission.local.json --execute
```
