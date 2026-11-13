```bash
# PC — cấu hình hai IP
cd ~/turtlebot4_ws
bash scripts/pc-configure.sh --robots robot1,robot2 <<'IPS'
10.11.52.230
10.11.52.243
IPS
```

```bash
# PC — SSH bằng khóa hiện có
ssh -i ~/.ssh/id_ed25519_tb4_debug ubuntu@10.11.52.230
```

```bash
# Terminal PC khác
ssh -i ~/.ssh/id_ed25519_tb4_debug ubuntu@10.11.52.243
```

```bash
# Trên từng Pi — chỉnh namespace, domain và discovery nếu cần
sudo turtlebot4-setup
```

```bash
# PC — kiểm tra lại
bash run.bash check
```
