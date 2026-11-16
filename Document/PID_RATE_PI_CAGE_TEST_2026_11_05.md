# UAV_DATN — bản Rate PI phòng thử có lồng/dây, 05/11/2026

## Tình trạng thực tế

- Đã kiểm tra log tĩnh sau hiệu chuẩn gá MPU6500 (`uav_20261105_120859.csv`): Roll trung bình +0,122°, Pitch -0,008°. **Không chỉnh trim thêm**.
- Log có cánh `uav_20261105_121328.csv`: 4.873 dòng, 1.232 dòng motor xuất lệnh; Roll lớn nhất +21,03°, gyro Yaw lớn nhất 236,9°/s.
- Khi máy báo gần độ cao 15–23 cm và các cần Roll/Pitch/Yaw ở gần giữa: 428/460 bản ghi có |Roll|>5° và PID Roll đã yêu cầu sửa ngược chiều lệch. Kết luận: **chưa thể đổ hoàn toàn cho gain PID**. Cần xác minh thứ tự motor, mô-men, loại cánh, hướng IMU và việc dây/lồng có kéo nghiêng hoặc cản Roll. Lệnh DShot không phải vòng/phút motor.
- Log ghi khoảng 40 Hz không đủ chứng minh đáp ứng rate 1 kHz, không cung cấp điện áp pin/dòng/RPM.

## Source đã chỉnh, không đổi các hệ thống còn lại

**`Core/Src/flight_pid.c`**

- Giữ nguyên Angle Kp=3.0, Rate Kp=1.5/1.5/1.0, Rate Kd=0.0; không tự ý tăng P sau khi thấy Roll lệch kéo dài.
- Thêm **Rate Ki mặc định rất nhỏ: Roll=0.20, Pitch=0.20, Yaw=0.10**. Với `integral_limit=30`, thành phần I không vượt 6 / 6 / 3 **đơn vị correction DShot** (trừ khi người dùng đổi gain runtime).
- Sửa anti-windup: nếu PID đã saturate, chỉ cho tích phân cập nhật theo chiều rời khỏi bão hòa. Nếu điều kiện I bị tắt, **xóa integrator ngay**, không giữ lực bù khi sang lượt thử mới.
- Bộ lọc riêng cho D-term dùng alpha tính từ `dt` thực và ngưỡng 25 Hz. **Kd mặc định vẫn 0**; không kích hoạt D khi chưa có log gyro tốc độ cao.
- Bổ sung các giá trị chẩn đoán rate setpoint, rate error, P/I/D trong `FlightPIDState`. **Không thay đổi `FlightPIDGains` 52 byte**, không đổi gói telemetry nhị phân.

**`Core/Src/uav_test.c`**

- Chỉ cho I-term hoạt động khi `armed && spinning`, |Roll|/|Pitch| < 6°, |gyro 3 trục| < 35°/s, và cả ba cần ở trung tâm. Ra khỏi vùng đó -> đặt tích phân về 0 để tránh tích lũy lực chống lại dây/lồng.
- Giữ nguyên KILL, ARM, SBUS failsafe, MCU protection, DShot300, PINOUT, mixer Quad-X, chống quá hạn IMU, ESP32-CAM, MT06, DPS310 và Alt Hold (vẫn đang tắt).
- Thêm `RT=`, `RE=`, `IT=` ở cuối **log text 5 Hz qua LPUART1**: giá trị (rate target/ rate error / I term) **nhân 10**, theo Roll/Pitch/Yaw. **Ground Station CSV 50 Hz chưa có thêm trường mới**; không đổi protocol UDP/ESP32.

## Điều kiện thử trên dây/lồng

1. Tháo cánh để kiểm tra trong thực tế góc Roll/Pitch khi nghiêng và chiều correction của từng motor; **đừng suy chiều mô-men từ tên chân PA6/PA7/PB0/PB1**. Cần xác minh đúng vị trí và chiều quay của từng motor và loại cánh.
2. Với lồng/dây: dây nên đủ độ rơ để không tạo mô-men Roll/Yaw tại thời điểm đo đáp ứng. Dây căng hoặc chạm khung sẽ làm lệch kết quả PID, và có thể gây tăng correction liên tục dù gyro bằng 0.
3. Sau khi nạp, mở ESP32-CAM trong chế độ CH8 CONFIG, KILL ON, ARM OFF, **Đọc PID**. Flash đã lưu cấu hình sẽ ghi đè `default_gains`. Đảm bảo lần thử này là `Angle Kp=3.0; Rate Kp=1.5/1.5/1.0; Ki=0.20/0.20/0.10; Kd=0; integral_limit=30; correction_limit=200; max_rate=80`. Chỉ cập nhật khi DISARMED; sau đó đọc lại để xác minh.
4. Ghi một log tĩnh 60s để so lại zero, sau đó thử từng trục trên giá có hạn chế hành trình. **Không bay tự do** khi chưa xác nhận chiều phản hồi và công suất ESC. Đổi từng tham số một và ghi lại giá trị trước/sau; nếu dao động tăng lên hoặc yaw quay tự phát, KILL và quay về cấu hình cũ.
5. Không dùng chỉ riêng `mixer_scale=1` hoặc `dma_errors=0` để kết luận motor đủ lực: cần đo điện áp khi có tải và RPM nếu ESC hỗ trợ.

## Các file tiện dụng

- `tools/analyze_flight_pid_csv.py` đọc CSV, in tỷ lệ sai lệch góc kéo dài dù correction đã ngược dấu, không điều khiển UAV.
- `tests/run_tests.sh` chạy bộ test host với mock HAL, UBSan. Xem `Document/TEST_RESULTS_PID_RATE_PI_2026_11_05.txt`.
- CSV dữ liệu mới kèm theo trong `Telemetry/UAV_GroundStation_Realtime_Charts/logs/`.

**Tuyên bố giới hạn:** Đây là bản thuật toán thử nghiệm từ log có sẵn, không phải tune chứng minh đã treo ổn định. Chưa thể xác minh mapping mô-men vật lý hay hiệu quả PID mà không có phép thử có kiểm soát trên máy thật. Không có build ARM/CubeIDE hoặc nạp STM32 trong môi trường hiện tại.
