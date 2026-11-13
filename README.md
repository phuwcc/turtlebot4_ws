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
