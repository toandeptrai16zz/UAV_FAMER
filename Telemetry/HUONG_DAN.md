# ESP32-CAM UAV: Wi-Fi TRỰC TIẾP, không cần router

Đây là project PlatformIO hoàn chỉnh với đúng **một** `ESP32_CAM/src/main.cpp` và một môi trường `[env:esp32cam]`. `build_src_filter` chỉ biên dịch `main.cpp`.

## Chức năng
- Chỉ bật SoftAP `UAV-DIRECT-XXXXXX` tại **192.168.4.1**. Không có lệnh `WiFi.begin()`, quét Wi-Fi nhà, chuyển sang STA, hay HTTP API đổi Wi-Fi.
- Giữ UART1 STM32 PA2(TX)->ESP GPIO13(RX), PA3(RX)<-ESP GPIO14(TX), GND chung, 115200 8N1.
- Giữ API PID, UDP telemetry cổng 14550 (Ground Station gửi `UAVT_HELLO`), và OTA ESP32 có xác thực SHA256 và điều kiện an toàn firmware cũ.
- Giữ nguyên bảng phân vùng OTA 4MB; không ghi đè NVS.
- Tự phát một bộ mật khẩu AP và API token **mới một lần** qua 2 khóa NVS `direct_ap` và `direct_tok`, vì bộ trước đã xuất hiện trong log chat. Xem Serial Monitor để lấy thông tin và giữ bí mật. Cấu hình Wi-Fi nhà cũ vẫn trong Flash nhưng không được sử dụng.

## Nạp USB qua đế nạp
Ở thư mục đang dùng: `/home/toan/Desktop/UAV_DATN/Telemetry/ESP32_CAM`
Giải nén ZIP, chép **duy nhất** `ESP32_CAM/src/main.cpp`, `ESP32_CAM/platformio.ini`, `ESP32_CAM/uav_ota4m.csv` vào đúng vị trí project hiện tại. Không để thêm *.cpp trong src.

```bash
cd ~/Desktop/UAV_DATN/Telemetry/ESP32_CAM
~/.platformio/penv/bin/pio run -e esp32cam -t upload --upload-port /dev/ttyUSB0
~/.platformio/penv/bin/pio device monitor -p /dev/ttyUSB0 -b 115200
```
Chọn cổng `/dev/ttyUSB0` hoặc `/dev/ttyUSB1` theo `ls /dev/ttyUSB*`.
Nếu dùng đế ESP32-CAM-MB, sau nạp nhấn RST/chuyển RUN. GPIO0 không nối GND khi chạy.

## Sử dụng
1. Mở Serial Monitor, lấy `Direct SSID`, `Direct password`, `API token` (không đưa token lên chat).
2. Kết nối laptop/điện thoại vào Wi-Fi `UAV-DIRECT-XXXXXX`; chọn **vẫn kết nối** nếu báo *không có Internet*. Ngắt VPN nếu VPN can thiệp route.
3. Mở `http://192.168.4.1/` (không phải trang router). Nhập API token để xem status/PID/OTA.
4. Trên laptop khi **đã kết nối AP**, chạy:

```bash
cd ~/Desktop/UAV_DATN/Telemetry/GroundStation
python3 uav_ground_station.py --ip 192.168.4.1
```
Nếu Ground Station của project cũ thiếu, dùng `ESP32_CAM/GroundStation/uav_ground_station.py` trong ZIP.

## Lưu ý
- `RX=1 frames=0` nghĩa là STM32 chưa truyền frame hợp lệ; đây là lỗi UART/STM32 riêng, không liên quan mạng AP.
- ESP32 AP **không cấp Internet** cho laptop. Laptop phải dùng kết nối khác khi cần Internet.
- Trước khi thay PID hay OTA: tháo cánh, ngắt ESC/motor, giữ trạng thái cấu hình an toàn. OTA không được thực hiện khi đang bay. Nếu không có telemetry STM32, chế độ OTA yêu cầu jumper GPIO2 xuống GND >=3s **sau khi boot** (không kéo GPIO2 thấp lúc khởi động).
- Không bay với Wi-Fi 2.4GHz đang bật gần receiver RC 2.4GHz trước khi kiểm tra nhiễu/độ phủ/tầm điều khiển.
- Firmware này đã được kiểm tra cấu trúc source và HTML/JS nhưng chưa build hoặc nạp thật trên máy user.
