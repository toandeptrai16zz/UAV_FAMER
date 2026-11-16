# UAV PID Yaw/Pitch R1 — sửa source từ log 16/11/2026

## Tình trạng và giới hạn

**ĐÃ CHỈNH SOURCE, CHƯA XÁC NHẬN HOVER ỔN ĐỊNH.** Log CSV 50 Hz cho thấy yaw nhanh (|gyro_z_body| đến 352,7°/s), pitch sai lệch đến 15,27°, chủ yếu khi RC yaw trung tâm. Motor đúng chiều và đủ lực cất cánh là thông tin người vận hành đã xác nhận; điều đó chưa đủ xác nhận dấu mô-men yaw thực theo mixer hay đáp ứng trễ ESC/khung.

Gói này giữ source gốc bên ngoài bốn tệp thay đổi, bao gồm SBUS, STM32G474RE, DShot300, ESC, KILL/ARM, failsafe MC7RE, MPU6500, MT06, DPS310, ESP32-CAM, Ground Station, cấu hình PID qua Wi-Fi và code chế độ Alt Hold (vẫn **OFF**). KHÔNG thay đổi thứ tự motor, chiều trục đã hiệu chuẩn, PID gains lưu trong Flash, giao thức binary telemetry 88 byte hoặc kích thước config 52 byte.

## File đã thay đổi

1. `Core/Src/uav_test.c`: sửa bộ lọc attitude: tích phân Roll/Pitch đúng theo Euler 3-2-1 khi gyro có Yaw; giữ nguyên IMU axis signs, trim và khả năng từ chối gia tốc giả. Thêm reset I-term khi motor OFF.
2. `Core/Src/flight_pid.c`: biến đổi lệnh Euler roll/pitch rate sang setpoint body gyro p/q với yaw đo thực, tránh nhầm hai hệ trục khi craft đang yaw; giữ các giới hạn rate/mixer/correction và gain cũ. Tích phân nhỏ được **giữ** trong dao động ngắn nhưng không tiếp tục cộng; chỉ reset khi motor off/disarm.
3. `Core/Inc/flight_pid.h`: thêm trường `reset_integrator` trong `FlightPIDInput` (struct nội bộ, **không** đổi protocol PID qua Wi-Fi).
4. `tests/test_flight.c`: thêm các regression test quaternion/Euler coupling math được dùng, target-to-body rate, integrator freeze/reset; vẫn chạy test cũ.

## Điểm KHÔNG sửa và chưa biết

- **Không tự tune Kp/Ki/Kd**: chưa có log high-rate có đủ setpoint/gyro/motor RPM để ước lượng gain đúng. Source giữ mặc định Angle Kp=3.0; Rate Kp=1.5/1.5/1.0, Ki=0.2/0.2/0.1, Kd=0; gain lưu ở Flash có thể khác.
- Không đổi giới hạn DShot, cánh, mixer yaw signs, vị trí dây. Một bộ điều khiển đúng toán vẫn có thể xoắn vì mô-men yaw thực không khớp mô hình, mất đồng bộ ESC, trễ động cơ, nhiễu IMU khi có tải, hoặc Rate P sai. Đây là bản sửa thuật toán rõ ràng có thể đo lại, KHÔNG bảo đảm bay được.
- Bộ attitude là complementary Euler 3-2-1 cải tiến (không phải quaternion; vẫn có kỳ dị gần 90° pitch và không xử lý bay lật).
- Mẫu UART ASCII `RT/RE/IT` vẫn chỉ 5 Hz; CSV Ground Station vẫn 50 Hz. Để tune Rate một cách đáng tin cậy nên bổ sung capture high-rate 200–500 Hz vào bộ đệm riêng sau khi kiểm chứng timing.
- Chưa build ARM CubeIDE trên máy thực (không có toolchain arm-none-eabi và phần cứng). Đã chạy **host tests với GCC và UBSan PASS**; đây không phải thử bay.

## Áp dụng lên dự án đang dùng

Giữ backup ZIP gốc. Đối với bản project `UAV_DATN(4).zip`, chỉ cần chép đè đúng ba file firmware:

- `Core/Src/uav_test.c`
- `Core/Src/flight_pid.c`
- `Core/Inc/flight_pid.h`

Các tệp khác giữ nguyên. Có thể dùng trọn gói `UAV_DATN_YAW_PITCH_FIX_R1.zip` trong STM32CubeIDE, nhưng ZIP rút gọn đã lược bỏ file build tạm, log CSV cũ, `.git`, và STEP khổng lồ từ phần PCB; **các source và drivers vẫn có**. Nếu cần toàn bộ PCB 3D thì giữ STEP từ ZIP gốc.

## Kiểm tra bắt buộc trước khi thử có cánh

1. STM32CubeIDE **Clean/Build** và đối chiếu cấu hình nạp, xác nhận không phát sinh warning/lỗi mới; không tự generate IOC ghi đè custom code.
2. **Tháo toàn bộ cánh**, KILL ON, ARM OFF: kiểm tra Roll/Pitch có dấu đúng khi nghiêng; khi giữ roll/pitch nghiêng và xoay yaw chậm, góc ước lượng phải biến thiên liên tục, không nhảy/rơi về 0 vô lý.
3. Kiểm tra mix command khi nghiêng và xoay trên dashboard; không suy dấu mô-men từ chiều rotor đơn thuần; xác minh phương pháp tĩnh không cánh an toàn.
4. Đọc PID hiện có từ ESP32-CAM. Flash cũ ghi đè `default_gains`; **chưa thay gain theo cảm tính**. Nếu chỉnh một tham số trên giá thử động lực có chặn/che chắn, giữ bản lưu giá trị trước đó và chỉ đổi một giá trị mỗi lần.
5. Không thử hover tự do từ bản firmware chưa kiểm chứng. Nếu có dao động, nóng motor/ESC hoặc lắc bất thường, ngắt thử trên giá; kiểm tra log timestamps/gyro/mixer và ESC response.

## Test

```bash
cd UAV_DATN_YAW_PITCH_FIX_R1
sh tests/run_tests.sh
```

Cập nhật: 2026-11-16. Bản này là sửa khuyết điểm động học tư thế gây coupling khi yaw, **không phải chứng nhận an toàn bay**.
