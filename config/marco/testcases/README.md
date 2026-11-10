# Testcase hai robot trên map my_room

Các điểm mẫu trong frame `map`, đơn vị mét/radian:

| Điểm | x | y | yaw |
| --- | --- | --- | --- |
| A | 0.15 | -0.10 | 0.00 |
| B | 0.15 | 0.60 | 0.00 |

Testcase 1 với các tọa độ này đã chạy thành công trên hai robot thật ngày
10/11/2026; testcase 2 và 3 mới được preview. Kiểm tra lại điểm và đường đi khi
đổi môi trường hoặc bố trí robot. Sửa `goal` theo điểm A/B thực tế. `label`
chỉ để đọc; planner dùng tọa độ `goal`. Nếu đổi A/B, sửa nhất quán cả ba file.

| File | Nhiệm vụ | Mục đích |
| --- | --- | --- |
| `01_parallel_ab.json` | robot1 → A; robot2 → B | Hai task không có dependency; có thể chạy song song khi đường đi cho phép |
| `02_cross_ba.json` | robot1 → B; robot2 → A | Kiểm tra điều phối khi hai đường có thể cắt nhau |
| `03_sequential_ab.json` | robot1 → A xong, robot2 → B | Kiểm tra dependency: task 2 chờ task 1 hoàn thành |

Mỗi testcase được lập kế hoạch từ vị trí robot hiện tại. File không reset vị
trí robot và không giả định testcase trước đã chạy. Muốn so sánh cùng điều kiện,
đưa robot về bố trí ban đầu và kiểm tra localization trước mỗi lượt.
Trong testcase 3, robot2 chưa nhận task khi robot1 đang đi, nhưng MARCO có thể
cho robot2 nhường đường nếu cần. Hai task độc lập cũng không bảo đảm hai bánh
xe bắt đầu quay đúng cùng thời điểm.

Xem trạng thái và preview (không di chuyển robot):

```bash
bash scripts/marco.sh status
bash scripts/marco.sh plan config/marco/testcases/01_parallel_ab.json --out log/marco/testcases/01-preview.json
```

Preview hiển thị batch đầu trong RViz. Xem `assignment` và `events` trong báo
cáo JSON để biết task được gán và đường nào được chọn. Với testcase 3, preview
có thể chỉ có task 1; lúc thực thi adapter lập kế hoạch tiếp sau mỗi batch.

Preview trên phiên robot thật ngày 10/11/2026 đã tạo kế hoạch cho cả ba file:
testcase 1 có hai path trong cùng batch; testcase 2 bắt đầu bằng một động tác
nhường đường của robot2; testcase 3 bắt đầu bằng task của robot1. Đây là kết
quả lập kế hoạch từ vị trí lúc kiểm tra. Testcase 1 sau đó được thực thi:
lượt đầu bị hủy do covariance AMCL robot1 vượt ngưỡng; lượt chạy lại báo
`succeeded`, hoàn thành task 1 và 2. Kiểm tra sau lượt chạy thấy lệnh vận tốc
hai robot bằng 0. Testcase 2 và 3 chưa được thực thi trên phần cứng.

Sau khi đã kiểm tra vị trí A/B, đường preview, robot khỏi dock và vùng chạy
có người giám sát, thực thi riêng:

```bash
bash scripts/marco.sh run config/marco/testcases/01_parallel_ab.json --execute --out log/marco/testcases/01-run.json
```

Đổi tên file để chọn testcase 2 hoặc 3. Nhấn Ctrl+C tại terminal chạy mission
để hủy những path còn hoạt động. Một lượt đạt khi báo cáo có
`result: "succeeded"` và `completed_tasks` chứa đủ `1` và `2`; task bị hủy hoặc
từ chối không được tính là đạt. Preview thành công chưa phải kết quả chạy thật.
