# UAV Ground Station — biểu đồ realtime (không cần Internet)

Bản này **chỉ chạy trên laptop**. Không thay đổi firmware STM32/ESP32, không gửi lệnh ARM/PID/motor, không dùng dữ liệu giả trong chế độ LIVE. Đã đối chiếu bộ giải mã với `uav_ground_station.py` của project UAV_DATN5, frame 88 byte phiên bản 1 (CRC-16/CCITT-FALSE).

## Cài / chạy trên Ubuntu Linux

1. Kết nối laptop vào mạng `UAV-DIRECT-XXXXXX` của ESP32-CAM. Laptop cần có IP `192.168.4.x`; ESP32 là `192.168.4.1`.
2. Dừng chương trình `uav_ground_station.py` cũ (Ctrl+C), vì dashboard mới dùng cùng UDP port 14550.
3. Giải nén gói ZIP vào thư mục tùy ý, sau đó:

```bash
cd ~/Downloads/UAV_GroundStation_Realtime_Charts
python3 realtime_dashboard.py --ip 192.168.4.1
```

4. Trên **chính laptop**, mở **http://127.0.0.1:8765** (không phải 192.168.4.1). Không cần Internet, không cần `pip install`.
5. `Ctrl+C` để dừng. CSV tự ghi vào `logs/uav_YYYYmmdd_HHMMSS.csv` và được flush khoảng mỗi giây.

Nếu đã copy vào `Telemetry/GroundStation/`, có thể chạy `python3 realtime_dashboard.py --ip 192.168.4.1` trong đó. Luôn giữ `uav_ground_station.py` bên cạnh vì đây là decoder chính xác của giao thức.

## Đọc số liệu

- Trang hiển thị Roll/Pitch, gyro XYZ, accel XYZ raw, RC/SBUS, PID correction, bốn DShot output, MT-06 range.
- Đồ thị tải dữ liệu đã nhận 10 lần/giây (tối đa 140 giây), vẽ khoảng 15/30/60/120 giây.
- **Toàn bộ gói hợp lệ** nhận được đều được ghi CSV, không chỉ mẫu 10Hz dùng cho đồ thị.
- Phía trên hiển thị `receive_hz_5s`, tuổi gói cuối, gói hợp lệ, gói lỗi và số `seq_gap`.
- `seq_gap` bao gồm cả các lượt STM32 bỏ truyền do UART bận, **không được coi là chỉ số riêng về mất gói qua sóng Wi-Fi**.
- Nếu không nhận dữ liệu quá 1,5 giây, dashboard báo `MẤT TELEMETRY`. Chỉ số này phục vụ ghi nhận, không thay thế failsafe hoặc điều khiển RC.
- Độ trễ biểu đồ thông thường phụ thuộc Wi-Fi và tốc độ cập nhật (~0,8 giây); không phải nguồn điều khiển bay thời gian thực cứng.

## Những dữ liệu CHƯA có trong frame hiện tại

- **DPS310**: STM32 có driver và cập nhật cảm biến, nhưng khung 88 byte V1 **không mang** áp suất, nhiệt độ và độ cao khí áp. Cần sửa protocol/cả decoder hai đầu nếu muốn đo từ DPS310 realtime.
- **Pin và dòng điện**: trong `uav_test.c` đang điền `0xFFFF` (không đo). Không có đồ thị giá trị thật cho đến khi có cảm biến và driver.
- **Gia tốc** đang là ADC/raw MPU6500, không tự đổi sang g (cần xác nhận full-scale).
- **MT-06 range** chỉ có dữ liệu khi cảm biến hợp lệ; trống khi giá trị `0xFFFF`.

## Kiểm tra an toàn và phạm vi test

Thử trên bàn, tháo cánh và ngắt nguồn động lực. Chỉ chuyển sang kiểm tra ngoài trời sau khi đã đánh giá range test RC 2.4 GHz, nhiễu ESP32 Wi-Fi 2.4 GHz, failsafe, chức năng ARM/KILL, nguồn, log mất gói và hiệu chỉnh PID. Wi-Fi UDP có thể rớt kết nối; dashboard tuyệt đối không dùng để điều khiển motor.
