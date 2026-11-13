```bash
# PC — đặt robot và sửa pose/goal phù hợp map thật trước mỗi testcase
cd ~/turtlebot4_ws
nano config/marco/initial_poses.local.yaml
bash multi_robot.bash --map maps/room_real.yaml --poses config/marco/initial_poses.local.yaml
```

```bash
# Terminal PC thứ hai — chọn từng testcase để sửa goal và preview
# 01: A/B; 02: đổi đích; 03: tuần tự; 04: giao cắt; 05: đối đầu
# 06: robot đứng chắn; 07: xung đột + DAG; 08: không chỗ nhường; 09: đổi chỗ đối mặt
tb4_case=01_parallel_ab
# Các tên khác: 02_cross_ba, 03_sequential_ab, 04_crossing, 05_head_on,
# 06_idle_blocker, 07_conflict_dag, 08_no_escape, 09_head_on_1m
nano "config/marco/testcases/${tb4_case}.json"
bash run.bash mission "config/marco/testcases/${tb4_case}.json"
```

```bash
# Chỉ thực thi sau khi kiểm tra pose/goal và đường preview; testcase 08 chỉ preview
if [[ "$tb4_case" != 08_no_escape ]]; then
  bash run.bash mission "config/marco/testcases/${tb4_case}.json" --execute
fi
# Ctrl+C để hủy mission
```
