# UAV R5.2 – Blackbox bắt được dữ liệu khi R5.1 ngắt quá sớm

## Mục tiêu
Bản R5.1 đôi khi tạo CSV `_PID200.csv` chỉ có header vì phải chờ Yaw ≥45°/s hoặc 1,2 giây mới bắt đầu ghi sự kiện, và khi tự ngắt đột ngột có thể không thu đủ 160 mẫu hậu sự kiện. Bản này sửa đường **thu thập dữ liệu** để chẩn đoán được lượt thử bị ngắt sớm, KHÔNG mở giới hạn ga hay bỏ bảo vệ.

## Thay đổi (STM32)
- `Core/Src/uav_test.c`: Blackbox kích hoạt ngay khi có mẫu motor thật đầu tiên, thay vì chờ 1,2 giây; khi Cage Guard chuyển sang trạng thái khóa do lỗi, đóng băng ngay dữ liệu đã có, kể cả khi sau đó IMU không còn mẫu hợp lệ.
- `Core/Inc/uav_rate_blackbox.h` và `Core/Src/uav_rate_blackbox.c`: thêm `UAVBB_FreezeNow()` không chờ thêm mẫu và không truyền UART trong vòng điều khiển.
- `tests/test_blackbox.c`: test dừng sớm sau 10 mẫu motor và trường hợp mất mẫu IMU.

## Giữ nguyên để không tăng rủi ro
- Cảnh báo và tự ngắt yaw/roll/pitch, giới hạn tốc độ, thời gian và DShot (1550), KILL/ARM, SBUS và failsafe.
- Không thay PID, mixer, tần số điều khiển hay thông số motor.
- Chỉ truyền Blackbox khi disarm + KILL ON + cả 4 DShot = 0. Sau khi ngắt, giữ nguồn STM32/ESP32 để tải CSV.
- Nếu ngắt trước khi kịp có mẫu motor chạy, vẫn có thể nhận log sự kiện nhưng không thể khôi phục dữ liệu motor chưa từng được ghi.

## Cập nhật
Giải nén bản vá tại thư mục gốc dự án STM32CubeIDE, sao lưu source trước; Refresh → Clean → Build. Không cần nạp lại ESP32 hay thay dashboard R5.1 vì giao thức gói không thay đổi.

## Trước khi cấp nguồn ESC
Tháo cánh; xác minh KILL ngắt ngay, motor mapping, IMU và cấp nguồn logic ổn định. Giá thử phải là kết cấu cứng có che chắn; không coi dây mềm là biện pháp an toàn đủ cho quad đã từng quay quá 500°/s. Không thử bay tự do để kiểm tra bản vá ghi log.

## Giới hạn kiểm chứng
Đã chạy test native, chưa xác nhận build ARM/hoạt động với STM32 thực. R5.2 chỉ sửa ghi Blackbox; **không phải bản sửa PID**.
