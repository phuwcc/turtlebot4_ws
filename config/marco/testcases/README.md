# Testcase hai robot trên map my_room

Các điểm mẫu trong frame `map`, đơn vị mét/radian:

| Điểm | x | y | yaw |
| --- | --- | --- | --- |
| A | 0.15 | -0.10 | 0.00 |
| B | 0.15 | 0.60 | 0.00 |

Testcase 1 với các tọa độ này đã chạy thành công trên hai robot thật ngày
10/11/2026; testcase 2 và 3 mới được preview. Kiểm tra lại điểm và đường đi khi
đổi môi trường hoặc bố trí robot. Sửa `goal` theo điểm A/B thực tế. `label`
chỉ để đọc; planner dùng tọa độ `goal`. Nếu đổi A/B, sửa nhất quán các file 01–03.

| File | Nhiệm vụ | Mục đích |
| --- | --- | --- |
| `01_parallel_ab.json` | robot1 → A; robot2 → B | Hai task không có dependency; có thể chạy song song khi đường đi cho phép |
| `02_cross_ba.json` | robot1 → B; robot2 → A | Kiểm tra điều phối khi hai đường có thể cắt nhau |
| `03_sequential_ab.json` | robot1 → A xong, robot2 → B | Kiểm tra dependency: task 2 chờ task 1 hoàn thành |
| `04_crossing.json` | Hai đường cắt nhau ở giao lộ | Chọn ưu tiên, nhường trước khi đi qua |
| `05_head_on.json` | Hai robot đối đầu và đổi chỗ | Cần vùng trống bên cạnh để tránh |
| `06_idle_blocker.json` | Robot1 đi qua robot2 đang idle | Xét vật cản là robot đứng yên, có thể điều phối robot2 nhường |
| `07_conflict_dag.json` | Đổi chỗ rồi quay về | Task 3/4 chờ cả task 1/2 |
| `08_no_escape.json` | Đối đầu trong hành lang kín không chỗ nhường | Negative test: planner từ chối, không dispatch |
| `09_head_on_1m.json` | Robot1 `(0,0,π)`; robot2 `(-1,0,0)`, đổi vị trí | Khoảng cách **tâm** 1 m; kiểm thử trên map tổng hợp có vùng nhường |

File 04–08 dùng bố trí test riêng, không dùng tọa độ A/B phía trên. Xem
[bảng vị trí ban đầu, yêu cầu map và tiêu chí đạt](../../../README.md#thêm-5-kịch-bản-kiểm-tra-xung-đột-marco).
Chúng được kiểm tra trên map tổng hợp trong `tests/test_marco_scenarios.py`,
chưa thực thi trên phần cứng. Phải chọn map/bố trí thực tế tương ứng trước khi
preview. Với 08 chỉ gọi `plan`, không gọi `run`.

Testcase 09 dùng goal mẫu đúng hai vị trí ban đầu nêu trong bảng; không tự đặt
hoặc di chuyển robot về đó. Phân biệt khoảng cách tâm với khoảng trống giữa
hai thân robot. Với bố trí thực tế khác, sửa cả pose khởi tạo và goal đổi chỗ
theo vị trí đã định vị. Thử nghiệm phần cứng ngày 11/11/2026 chưa tới bước chạy
09 vì robot2 chưa đạt chất lượng localization trên `my_room`; không coi kết
quả kế hoạch trên map tổng hợp là đã đổi chỗ thành công ngoài đời.

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
