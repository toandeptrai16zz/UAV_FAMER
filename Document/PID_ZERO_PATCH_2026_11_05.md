# Bản sửa ZERO IMU + PID Angle thử nghiệm — 05/11/2026

## Đầu vào và điều kiện
- Source nền: `UAV_DATN_ON_DINH_SOURCE_2026_11_05.zip` (đã có bộ lọc complementary tau=0,5 giây).
- Log thực tế mới: `uav_20261105_114917(1).csv`, 2.283 mẫu/56,108 giây, UAV được người dùng xác nhận đứng yên trên nền đất, motor tắt, không cánh trong phép đo tĩnh.
- Roll trung bình = **-1,2988°** (STD 0,0094°), Pitch = **-2,81026°** (STD 0,0081°).
- Gyro Roll = -0,26636°/s, Pitch = 0,06075°/s, Yaw = 0,01297°/s.
- `armed=True` nhưng `spinning=False`, các lệnh output motor đều 0. PID Roll trung bình +8,19; Pitch +16,77: đây là giá trị nội bộ, chưa tạo lực đẩy trong log tĩnh.

**Lưu ý về mặt phẳng:** "nằm im trên đất" chứng minh tĩnh nhưng không tự động chứng minh nền ngang tuyệt đối. Hai trim tính dưới đây là phép bù thử nghiệm **nếu nền ghi log được xác nhận là ngang**. Nếu mặt đất gồ ghề, cần hiệu chuẩn trên bàn cân bằng trước khi thử có cánh.

## Chỉnh sửa trong source
1. `Core/Src/uav_test.c`:
   - `IMU_ROLL_MOUNT_TRIM_DEG`: **-2,13° → +0,68°**.
   - `IMU_PITCH_MOUNT_TRIM_DEG`: **-1,06° → +0,24°**.
   - Lưu ý **hai trục đặt tên trim bị đổi chéo trong phép biến đổi**: `roll = -sensor_pitch + PITCH_TRIM`, `pitch = -sensor_roll + ROLL_TRIM`.
   - Dự đoán trên **chính log cũ**, Roll sau bù gần **+0,0012°**, Pitch gần **-0,00026°**. Đây là bù số học từ log, chưa chứng minh trên phần cứng mới hoặc trong khi bay.
   - `imu_control_valid` giờ bắt buộc `attitude_initialized` và cả hai góc hữu hạn; tránh coi giá trị 0 mặc định là góc hợp lệ nếu vừa bỏ qua dữ liệu gia tốc khi khởi tạo.
2. `Core/Src/flight_pid.c`:
   - Outer **Angle Kp từ 4,0 xuống 3,0 (-25%)**, chỉ để thử giảm mức correction yêu cầu theo góc.
   - Giữ nguyên **Rate Kp=1,5/1,5/1,0**, **Ki=Kd=0**, `max_rate_dps=80`, `correction_limit=200`, mixer, DShot300, watchdog, failsafe, ARM/KILL, ESP32-CAM, PID runtime, MT06, DPS310, Alt Hold vẫn tắt.
   - Tham số mới không phải bộ PID được đảm bảo ổn định với khung thực tế. Nếu drone không hồi góc đủ nhanh hoặc nghiêng ngày càng lớn, cần dừng và kiểm tra dấu, khả năng tạo mô-men, vị trí motor/prop và nguồn, **không cứ giảm P tiếp**.
3. `tests/test_flight.c`: cập nhật giá trị mặc định, thêm test bù zero từ dữ liệu tĩnh và phản ứng PID khi không di chuyển.

## Cực quan trọng: PID lưu trong Flash ghi đè giá trị mặc định
`UAVConfig_Init()` gọi `load_gains()` ở khởi động: nếu STM32 có config PID hợp lệ trong Flash, **Angle Kp có thể vẫn là 4,0 sau khi flash firmware**.

Để xác minh trên bộ hiện tại (luôn tháo cánh, nguồn động lực ngắt):
1. Tắt ARM, bật KILL, ga thấp, gạt **CH8 CONFIG ON**; bật ESP32-CAM ở điều kiện an toàn.
2. Kết nối mạng trực tiếp của ESP32, truy cập `http://192.168.4.1/`, nhập API token của chính thiết bị (không chia sẻ token).
3. Bấm **Đọc PID**, ghi lại các hệ số hiện hành. Nếu outer **Angle Kp = 4,0**, đổi thành **3,0** và giữ nguyên các thông số còn lại.
4. Bấm **Áp dụng RAM**, đọc lại. Chỉ khi đã kiểm tra đúng hãy cân nhắc **Lưu Flash** để giữ qua reboot. Luôn dùng CH8 CONFIG + KILL + ARM OFF; không cập nhật PID giữa chuyến bay.
5. Không tự động xóa Flash cấu hình: giữ nguyên dữ liệu và các preset đang có.

## Trình tự kiểm tra bắt buộc
1. **Tháo cánh hoàn toàn**. Nạp firmware mới bằng STM32CubeIDE, kiểm tra build và nạp thực tế.
2. Đặt máy lên **mặt phẳng được kiểm tra bằng thước thủy**, đợi gyro calibration hoàn tất, KILL ON, ARM OFF. Ghi khoảng 60 giây: kỳ vọng Roll/Pitch gần 0° và không trôi liên tục. Nếu vẫn lệch, không bù thêm dựa vào một mẫu tức thời; đánh giá lại góc gá/độ phẳng và offset.
3. Nghiêng tay từng trục khi tháo cánh: kiểm tra góc/gyro đúng chiều và lệnh PID phải chống lại sự nghiêng. Không phụ thuộc vào RPM thực tế để chứng minh chiều khi không có cánh.
4. Đo phản ứng motor với cánh tháo và motor thấp có kiểm soát; xác minh thứ tự 4 motor, chiều CW/CCW và cách nối DShot, phát hiện motor giật/không đồng đều.
5. Chỉ sử dụng thử bay có bảo hộ và quy trình an toàn sau khi đã chứng minh chiều điều khiển và lực đẩy đúng; log ở tốc độ cao hơn 40–50 Hz để tuning inner Rate trước Outer Angle.

## Kiểm thử phần mềm
- `sh tests/run_tests.sh`: PASS tất cả (host GCC/UBSan + protocol tests; mixer 34.606 trường hợp).
- Kiểm thử thêm các sai lệch góc tĩnh và correction sau bù; kiểm tra safety logic init attitude.
- **Chưa build ARM cross-toolchain trong môi trường này** (thiếu `arm-none-eabi-gcc`); chưa bench hoặc thử bay. Không thể cam kết đã bay ổn định.

## Tại sao không thêm Ki/Kd ngay
Trong firmware trước, Ki=0 nên lỗi lắc chậm sau vài giây **không thể quy trực tiếp cho integral windup**. Kd dễ khuếch đại rung gyro. Cần đo `rate_setpoint` vs `gyro_rate` thực tế và kiểm tra hướng tạo mô-men trước. Tài liệu PX4: https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter
