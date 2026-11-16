# UAV R5.6 — Tare tư thế khung chủ động, tách gyro bias

## Chính xác thay đổi gì

- MPU6500 tiếp tục hiệu chuẩn **gyro bias** từ 1.500 mẫu khi startup. Không cần mặt phẳng ngang cho gyro bias, chỉ cần khung đứng yên.
- Bộ ước lượng góc `roll_angle/pitch_angle`, các trim gá IMU hiện có, DLPF, PID gain, tốc độ gyro/rate, mixer, ARM/KILL/cage guard **không thay đổi**.
- Bổ sung **mốc tư thế khung do người dùng tự chọn** `level_ref_roll/pitch`: đầu vào Roll/Pitch Angle PID = góc ước lượng gốc trừ mốc. Telemetry Roll/Pitch hiển thị góc tương đối mốc. Trạng thái vật lý vẫn xét góc gốc cho arm/cage safety.
- **KHÔNG** tự xem nền nhà là ngang; **KHÔNG** tự lấy mốc lúc bật nguồn. Tare **không chứng minh khung thật sự song song mặt đất, không tự động sửa được góc gá sai theo cơ khí**. Chỉ xóa offset hiển thị/đầu vào Angle controller dựa trên tư thế bạn chọn.
- Để tạo mốc: giữ disarm, KILL ON, ga thấp, RC tốt, motors = 0, IMU tươi, gia tốc ~1g, cả 3 gyro rate <1.5°/s và góc ổn định liên tục 100 mẫu x 20ms (~2s); từ chối mốc > ±5° hoặc dao động trong cửa sổ >0.6°. Chỉ sau khi nhấn **Đặt mốc vào RAM**.
- Có thể **Đọc mốc / Đặt mốc vào RAM / Lưu Flash / Xóa mốc RAM** ngay trên `http://192.168.4.1` với token cũ. Quyền ghi: cờ `CONFIG_READY` từ STM32, không mở khóa từ ESP.
- Khi **Lưu Flash**, mốc và 13 gain PID hiện tại lưu cùng nhau trên 2 trang Flash cuối theo journal có checksum và commit. Vẫn tương thích với flash R5.5 cũ chỉ có PID; hai trang cuối vẫn phải được bảo vệ bởi linker và SWD/OTA.

## Cách chạy thử — KHÔNG LẮP CÁNH

1. Backup dự án, chép patch lên thư mục gốc rồi **Build và Flash cả STM32 và ESP32**. Chỉ tạo firmware; chưa xác minh trên phần cứng.
2. Tháo cánh, ngắt nguồn ESC, đặt UAV theo **tư thế tham chiếu bạn chủ động chọn** và giữ hoàn toàn yên. Đừng tưởng nền nhà chính xác.
3. Điều khiển CH5 ARM OFF / CH6 KILL ON, ga thấp nhất; chờ >4 giây để CONFIG_READY và đủ 2 giây cửa sổ yên.
4. Vào `192.168.4.1`, nhập token, tick xác nhận an toàn; **Đọc PID** để mở quyền cấu hình, **Đọc mốc**, rồi **Đặt mốc vào RAM**.
5. Xem Dashboard: tại chính tư thế được chọn, Roll/Pitch sẽ quanh 0°. Nghiêng khung bằng tay (vẫn không cánh/không ESC) và xác nhận đúng dấu Roll/Pitch và tốc độ gyro.
6. Nếu tương ứng cơ khí hợp lý, nhấn **Lưu Flash**, reset STM32, **Đọc mốc** và xác nhận giá trị tồn tại. Nếu không, **Xóa mốc RAM** và chưa lưu. Xóa Flash bằng **Xóa mốc RAM → Lưu Flash**.
7. Không thử cất cánh tự do chỉ dựa vào thao tác tare: mốc do người dùng chọn không phải tự đo cân bằng trọng lực. Giá đỡ lệch hoặc gắn cánh/khung lệch vẫn cần kiểm tra cơ khí và hành vi torque.

**Mốc sau tare ≠ IMU mount alignment 3D**: khi IMU bị xoay khỏi trục frame, cần kiểm tra phép biến đổi trục bằng thao tác xoay tay từng trục. Thao tác mới chỉ hiệu chỉnh gốc góc, không đổi hướng trục hoặc xoay ma trận 3D.

### Phần mềm cần nạp
- `Core/Src/uav_test.c`
- `Core/Src/uav_config.c`
- `Core/Inc/uav_config.h`
- `Telemetry/ESP32_CAM/src/main.cpp`

`YAW Kp` không thay đổi trong source. Tiếp tục đọc lại PID trong web; nếu RAM/Flash trước đó là 0.7, nó được giữ. Không mặc định ép giá trị khi reset.

**Kiểm thử**: cần Build STM32/ESP32 với toolchain cài trên máy của bạn, nạp thử trên phần cứng và xác thực checkpoint sau reset. Đánh giá ổn định dưới tải chưa hoàn thành.
