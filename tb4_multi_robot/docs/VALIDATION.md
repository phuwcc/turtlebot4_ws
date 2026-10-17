# Phạm vi kiểm chứng

Ngày 17/10/2026, ROS 2 Humble, Ubuntu 22.04 aarch64 trên Pi.

| Kiểm tra | Kết quả |
| --- | --- |
| Build hai package độc lập | Thành công, một luồng, C++17 |
| CTest `controls` | Qua: phép nghịch đảo pose, phím teleop và timeout |
| Hai SLAM C++ cùng lúc, domain 167 | Cả hai phát map theo namespace riêng |
| Gốc robot 1 giả | Chụp đúng `(2, -3, 0.4)`; mapping → base gần `(0,0,0)` |
| Gốc robot 2 giả | Chụp đúng `(-1.5, 4, -0.7)`; mapping → base gần `(0,0,0)` |
| Rò topic ở phép thử hai robot | Không có publisher `/map` hay `/tf` gốc |
| AMCL/Nav2 namespace robot1 | Map server, AMCL, controller, planner, navigator `active`; lập đường thành công |
| AMCL/Nav2 namespace robot2 | Cùng kiểm tra, `active`; lập đường thành công, 8 pose |
| Tự khởi tạo pose thiếu x/y/yaw | Bị từ chối trước khi launch node |
| Tham số launch, YAML/XML và shell | Đã kiểm tra cú pháp/nạp launch |

Phép thử navigation chạy **lần lượt** cho mỗi namespace, với map/scan/odom/TF giả
trong domain 168 và 169. Chỉ gọi `ComputePathToPose`, không gọi action điều khiển
chuyển động `NavigateToPose`. Phép thử hai SLAM dùng chung domain 167 để kiểm tra
đúng trường hợp frame ID giống nhau nhưng hai luồng TF tách biệt.

Lệnh tái hiện từ thư mục repo:

```bash
source install/setup.bash
ctest --test-dir build/tb4_multi_tools --output-on-failure
python3 tests/smoke_two_robots.py
python3 tests/smoke_navigation.py robot1
python3 tests/smoke_navigation.py robot2
```

Kết quả máy đọc được tạo trong `log/two_robot_smoke/result.json` và
`log/navigation_smoke/{robot1,robot2}/result.json`. Thư mục `log` được Git bỏ qua.

Chưa xác nhận trên hai robot vật lý, chưa đổi namespace firmware của Pi hiện tại,
chưa thử RViz có GUI trên laptop hoặc điều khiển tới goal thật. Mạng hai Pi,
namespace Create 3, nguồn USB và pose khởi tạo cần được kiểm tra tại chỗ theo README.
Repo chưa có fleet manager, hợp nhất map hoặc điều phối chống tranh đường.
