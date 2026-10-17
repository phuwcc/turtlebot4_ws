# Cấu hình bringup trên Pi

Đợt kiểm tra 17/10/2026 phát hiện hai driver LiDAR cùng mở một cổng USB,
republisher tích hợp lỗi namespace, và kernel báo USB `over-current` khi camera
khởi động; camera và LiDAR cùng bị ngắt USB.

Các thay đổi đã áp dụng trên Pi:

- Tắt và disable `turtlebot4-rplidar.service`: bringup đã mở LiDAR.
- `robot.launch.py`: namespace Create 3 là `/_do_not_use`, tránh topic lỗi
  `//_do_not_use/...`. Dùng republisher tích hợp, disable dịch vụ bù
  `create3-republisher.service` để không mở trùng.
- `standard.launch.py`: cho phép bật/tắt camera bằng `TB4_ENABLE_CAMERA`.
- `systemd/95-lidar-navigation.conf`: tạm đặt `TB4_ENABLE_CAMERA=false` cho
  quét map/navigation bằng LiDAR 2D, giảm tải CPU và USB. Cần kiểm tra nguồn,
  cáp/hub trước khi bật lại camera; đây không phải kết luận camera hỏng.

File được cài vào:

```text
config/robot.launch.py
  -> /opt/ros/humble/share/turtlebot4_bringup/launch/robot.launch.py
config/standard.launch.py
  -> /etc/ros/humble/turtlebot4.d/standard.launch.py
config/systemd/95-lidar-navigation.conf
  -> /etc/systemd/system/turtlebot4.service.d/95-lidar-navigation.conf
```

Bản trước khi sửa nằm trong `log/check_all_backups/`. Cập nhật gói bringup có
thể ghi đè launch trong `/opt/ros`; cần đối chiếu lại cấu hình sau cập nhật.
Profile `/etc/turtlebot4/fastdds_rpi.xml` hiện dùng UDP từ lần kiểm tra trước;
không đổi RMW hoặc cập nhật gói ROS trong đợt kiểm tra này.

Để bật lại camera **sau khi xử lý nguồn/cáp USB**, sửa drop-in trên Pi thành
`Environment=TB4_ENABLE_CAMERA=true`, rồi chạy:

```bash
sudo systemctl daemon-reload
sudo systemctl restart turtlebot4
```

Không enable lại hai dịch vụ bù khi bringup tích hợp đang chạy. Kiểm tra:

```bash
systemctl is-active turtlebot4 discovery
pgrep -af '[/]rplidar_ros/rplidar_composition'
pgrep -af '[/]create3_republisher/create3_republisher'
sudo fuser /dev/RPLIDAR
```

Mỗi loại chỉ nên có một tiến trình. Tên `ttyUSB0`/`ttyUSB1` có thể đổi sau reset;
driver dùng symlink `/dev/RPLIDAR`.
