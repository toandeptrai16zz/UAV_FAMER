# UAV V3.2.8 — Kiểm tra Yaw thụ động, KHÔNG CÁNH

Bản vá nhỏ trên nền V3.2.7, dành riêng cho STM32G474RE. Chỉ thay đổi
`Core/Src/uav_test.c` và giữ nguyên `flight_pid.c`, `.ioc`, DShot300, chân motor.

## Dữ liệu xác định lý do thay đổi

- M1: trước trái CCW, M2: trước phải CW, M3: sau trái CW, M4: sau phải CCW.
- Khi xoay khung theo chiều kim đồng hồ nhìn từ trên (theo phép thử đã hướng dẫn), log V3.2.7 xuất `GZ` âm và `PID Yaw` dương.
- `FlightPID_MixX` của V3.2.7 tăng các motor CCW (M1/M4) khi PID Yaw dương. Theo phản lực, điều này tạo mô-men quay thân *theo chiều kim đồng hồ*. Đó là phản hồi cùng chiều xáo trộn, không phải chống xáo trộn.
- V3.2.8 đổi duy nhất tốc độ Yaw đầu vào vòng PID từ `gyro_z_rate_dps()` thành `-gyro_z_rate_dps()` để thống nhất với quy ước Yaw dương là *theo chiều kim đồng hồ khi nhìn từ trên*.

Thay đổi này **chỉ hợp lệ nếu thực sự xoay CW ở log trên** và cảm biến giữ nguyên cách lắp. Nếu thao tác khác hoặc lắp đảo IMU, không áp dụng như hệ số bay; phải kiểm tra lại hướng đo.

## Cách cài

1. Sao lưu `Core/Src/uav_test.c` V3.2.7.
2. Chép file `Core/Src/uav_test.c` của bản vá này vào project, ghi đè file cùng tên.
3. Giữ nguyên `flight_pid.c`, `flight_pid.h`, `.ioc` và toàn bộ TIM3 DMA.
4. Đảm bảo `Core/tests/test_v327_mixer.c` đã Exclude from Build, tránh trùng `main()`.
5. Clean → Build → nạp, khi **đã tháo toàn bộ cánh và ngắt nguồn pin ESC**. Chỉ cấp nguồn STM32.

## Test Roll / Pitch / Yaw (KHÔNG phát xung motor)

Đặt ARM OFF, KILL ON, stick trung tâm, cố định MPU chắc chắn. Log sẽ bắt đầu bằng `V...` và banner `V328 PASSIVE_YAW_SIGN_VERIFY`.

- Nằm ngang: xem R/P và PID/MIX. Giá trị PID tại vị trí ngang không nhất thiết bằng zero vì chưa hiệu chỉnh trim góc lắp và trim tay RC.
- Nghiêng bên phải xuống 15°: R dương, M2/M4 MIX cao hơn M1/M3 (đúng theo log cũ). Nghiêng bên trái xuống: ngược lại.
- Chúi đầu xuống: P âm, M1/M2 MIX cao hơn M3/M4. Ngẩng đầu lên: ngược lại.
- Xoay khung **theo chiều kim đồng hồ** khi nhìn từ trên: raw `GZ` âm như ảnh cũ, PID Yaw trong V328 phải **âm** (khi cần Yaw ở giữa). MIX M2/M3 tăng tương đối so với nhóm M1/M4 sau khi xét Roll/Pitch. Trong khi xoay CCW, dấu Yaw phải ngược lại.

Lưu ý: hiệu chỉnh MIX có thêm Roll và Pitch, vì thế khi kiểm tra Yaw so sánh *tổng cặp*: `(M2+M3) - (M1+M4)` phải dương khi yaw thực tế CW.

## Điều kiện không thay đổi

- `UAV_BENCH_ARM_LOCK=1`
- `UAV_PROPS_OFF_ESC_TEST=0`
- DShot output = 0, tuyệt đối không test bay bằng V328.
- Kiểm tra áp suất, lực đẩy, failsafe, đo dòng ESC và PID động chưa hoàn tất. Không dùng bộ PID bench này để bay tự do.
