# UAV R3 – SỬA DẢI GYRO VÀ CHẨN ĐOÁN YAW (16/11/2026)

> **Bản R3 này KHÔNG PHẢI firmware bay.** ARM bị khóa ở thời điểm biên dịch để kiểm tra lỗi trong điều kiện không có cánh. Không dùng R3 để bay tự do hoặc thử mô-men dưới tải.

## Tại sao cần R3?

Log `uav_20261116_163935.csv` từ lần thử không dây cho thấy gia tốc quay Yaw lớn; telemetry có đỉnh khoảng **501,1°/s**, trùng ngưỡng do dải MPU6500 ±500°/s (`GYRO_CONFIG = 0x08`, `65.5 LSB/(°/s)`). Tốc độ quay thực tế có thể lớn hơn vì cảm biến bị giới hạn phép đo. Ở nhiều đoạn, PID Yaw đã bù đến giới hạn ±200 nhưng không chặn được xoắn.

Không thể quy kết chỉ Kp quá lớn hoặc sai dấu motor từ CSV: log chỉ có lệnh DShot, **không có tốc độ motor thực, mô-men quay, hay phép đo chiều xoay độc lập**. Vì vậy không tự đảo trục Yaw/mixer hoặc gán một bộ PID “bay ổn” khi chưa thử xác nhận phần cứng.

## Sửa thật trong mã nguồn

1. `Core/Src/uav_test.c` đặt `GYRO_CONFIG=0x18` (**±2000°/s**), hệ số `GYRO_LSB_PER_DPS=16.4f`, nhất quán giữa register/đọc dữ liệu/PID/telemetry.
2. Đọc lại thanh ghi `GYRO_CONFIG` sau khi ghi. Nếu phần cứng không nhận dải đo mới thì IMU init thất bại: không coi cảm biến là hợp lệ.
3. Tính lại ngưỡng kiểm tra IMU đứng yên khi hiệu chuẩn: 4°/s × 16,4 LSB/(°/s), không dùng ngưỡng raw 260 counts cũ.
4. Thêm `UAV_YAW_DIAGNOSTIC_ARM_LOCK=1`. Khoá ngay cả tại điều kiện `can_arm` và **ngay trước DShot_send_all**; giữ KILL/ARM/failsafe như cũ. R3 không quay motor khi gạt ARM hay nâng ga.
5. Giữ nguyên bộ PID R2 và thuật toán góc R1, giao thức ESP32, MCU RC 6 kênh và vị trí motor. Chưa đổi dấu `body_yaw_rate_dps()` (vẫn `-gyro_z`).
6. `tests/test_flight.c`: bổ sung kiểm tra mock SPI ghi/đọc thanh ghi, phát hiện cấu hình sai, phép đổi dải ±2000°/s, và khóa motor không phụ thuộc lệnh ARM. Chạy cả bộ hồi quy R2 ở chế độ giả lập với khóa chẩn đoán tắt — **chỉ trong test host**.
7. `tools/analyze_yaw_log.py`: thống kê Yaw và trạng thái bão hòa từ CSV, chỉ hỗ trợ chẩn đoán, không chứng nhận bay.

## Bạn làm trên Ubuntu / STM32CubeIDE

1. **Tháo hết cánh** (4 cánh), ngắt nguồn ESC, để KILL ON và ARM OFF. Sao lưu bản project trước đó.
2. Giải nén `UAV_DATN_YAW_DIAGNOSTIC_R3.zip`, mở project STM32CubeIDE trong thư mục R3, build và nạp qua ST-LINK. Không đổi `UAV_YAW_DIAGNOSTIC_ARM_LOCK` từ 1 thành 0 để bay.
3. Kiểm tra khi bật nguồn: R3 có đọc IMU; ARM không cho motor hoạt động. Không đấu cánh vào thử với bản chẩn đoán.
4. Mở dashboard telemetry trên Wi-Fi UAV. Đặt drone trên bàn, xác nhận Roll/Pitch hợp lý.
5. **Nhìn TỪ TRÊN XUỐNG** (mũi drone hướng ra xa): cầm drone bằng tay quay mũi **theo chiều kim đồng hồ (CW)** khoảng 30–60°, quan sát **gyro Yaw có dấu DƯƠNG (+)**. Quay ngược chiều kim đồng hồ (CCW): gyro Yaw phải có dấu ÂM (−). Động cơ phải ở 0 trong suốt phép thử.
6. Gửi ảnh/video dashboard hiển thị hướng xoay và dấu Yaw; nếu dấu ngược nhau, **dừng**, chưa tự đổi `UAV_BODY_YAW_SENSOR_SIGN`. Cần phân biệt lắp IMU, chiều quy ước góc và chiều mô-men cánh trước khi thay dấu.
7. Bước độc lập tiếp theo (sau khi dấu đúng): dùng thiết bị đo lực/mô-men và giá thử có che chắn để xác minh lệnh M1/M4 so với M2/M3 có tác dụng quay đúng chiều. Phép kiểm tra phải do người có thiết bị bảo vệ phù hợp thực hiện, không dùng R3 để cấp motor.

### Đọc lại log cũ

```bash
python3 tools/analyze_yaw_log.py /duong/dan/uav_20261116_163935.csv
```

### Kiểm thử phần mềm

```bash
bash tests/run_tests.sh
```

Đã chạy thành công trên Linux với compiler C host và Python. **Chưa compile/link firmware ARM cho STM32G474RE hoặc thử với cảm biến/ESC thật** do môi trường hiện tại không có ARM toolchain/phần cứng.

### PID / Flash

- Bộ R2 chưa chứng minh ổn định khi không dây; lần bay vừa qua vẫn đạt đến ngưỡng gyro ±500°/s.
- Yaw Kp=0,7 trước đó là cấu hình **Áp dụng RAM**. Khi reset hoặc nạp lại STM32 có thể lấy lại cấu hình Flash/default, **không giả định 0,7 vẫn còn**.
- Không lưu 0,7 vào Flash và không tiếp tục tăng ga/bay tự do chỉ vì bản R3 đo được tới ±2000°/s.

### Giới hạn quan trọng

R3 mới sửa **giới hạn đo lường** (một lỗi thật có bằng chứng). Không làm thay đổi mô-men motor thực, không xác minh bộ cánh, không tự tuning PID, không giải quyết chắc chắn sự xoắn. **ARM bị khóa là thiết kế chủ ý.**
