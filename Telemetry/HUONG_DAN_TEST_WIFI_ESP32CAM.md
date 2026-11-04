# ESP32-CAM: xác minh lỗi không thấy Wi-Fi Setup

Dự án có DUY NHẤT `ESP32_CAM/src/main.cpp`; không chép thêm file .cpp vào src.

## BƯỚC 0 - AN TOÀN

Tháo cánh, ngắt nguồn ESC/motor, chỉ cấp ESP32-CAM bằng nguồn 5V ổn định. Để nguyên UART GPIO13/14 ngoài trong lần thử AP đầu tiên nếu có thể. Không sử dụng chức năng OTA hoặc PID trên UAV khi đang ARM/bay.

## BƯỚC 1 - Chế độ chẩn đoán AP-only

Tại `ESP32_CAM`:

```bash
~/.platformio/penv/bin/pio run -e esp32cam_ap_test -t clean
~/.platformio/penv/bin/pio run -e esp32cam_ap_test
~/.platformio/penv/bin/pio run -e esp32cam_ap_test -t upload --upload-port /dev/ttyUSB0
```

Thay cổng thật (`ls /dev/ttyUSB*`). Khi nạp ESP32-CAM bằng USB-UART: GPIO0 nối GND khi vào bootloader; nạp xong tháo GPIO0-GND rồi nhấn RST. **Không giữ GPIO0 ở GND khi chạy firmware.**

Mở Serial Monitor:

```bash
~/.platformio/penv/bin/pio device monitor -p /dev/ttyUSB0 -b 115200
```

Nhấn RST; tìm các dòng `[BOOT]`, `Setup SSID`, `Setup password`, `[WIFI] AP start ... softAP_ok=1`, `[AP CHECK] ...`. Nếu `softAP_ok=1`, trên điện thoại quét `UAV-SETUP-XXXXXX`, dùng mật khẩu in trong Serial, vào `http://192.168.4.1/health`. SSID không tự đổi / tự tắt do STA trong chế độ này. Vì AP-only chỉ để test, bấm "Lưu/Kết nối" sẽ không cho vào router trong bản này.

Nếu Serial không có `[BOOT]`: có thể chưa chạy firmware (GPIO0 đang GND, nạp sai port, reset/nguồn, flash). Nếu `AP start softAP_ok=0`: cần xem toàn bộ log Serial. Nếu AP báo OK nhưng điện thoại không nhìn thấy: kiểm tra nguồn 5V (ít nhất dòng đỉnh phù hợp), khoảng cách, bo/anten, đổi thiết bị quét, xác nhận ESP32-CAM không giữ GPIO0 ở mức LOW.

## BƯỚC 2 - Chỉ sau khi AP-only ổn định, chuyển chế độ đầy đủ

```bash
~/.platformio/penv/bin/pio run -e esp32cam -t clean
~/.platformio/penv/bin/pio run -e esp32cam
~/.platformio/penv/bin/pio run -e esp32cam -t upload --upload-port /dev/ttyUSB0
```

Không cần xóa NVS. Chế độ đầy đủ tạo AP trong 45 giây; sau đó mới tự thử Wi-Fi router đã lưu (nếu không có client AP). Khi kết nối STA mạng Setup có thể đổi kênh/bị gián đoạn khoảng 18 giây. Nếu router lỗi, nó quay lại AP; hệ thống còn IP `192.168.4.1`. Khi STA vào router, lấy IP thật từ Serial `[WIFI] CONNECTED ... IP=...`. Chỉ bắt đầu dùng Python Ground Station khi ESP32 có IP và STM32 thực sự gửi frame telemetry (trước đó RX=1 frames=0 là chưa có dữ liệu).

## Ý nghĩa log

- `[BOOT]`: ứng dụng chạy tới setup.
- `[WIFI] AP start mode_ok=1 ip_ok=1 softAP_ok=1`: driver SoftAP chấp nhận khởi tạo.
- `[AP CHECK] ... mode=2`: AP-only. `mode=3`: AP+STA. `mode=1`: STA-only.
- `clients=0`: chưa có máy nào nối vào AP.
- `softAP_ok=0` hoặc lặp reset: kiểm tra nguồn, dây chân boot/USB, Serial Monitor log.
- `RX=1 frames=0`: STM32 chưa cấp telemetry hợp lệ; không ngăn ESP32 phát Wi-Fi.

**Lưu ý**: chưa thể xác nhận build PlatformIO hoặc chạy thật trên bo trong môi trường tạo file. Đừng bay khi chưa kiểm chứng failsafe, RC và nhiễu 2.4 GHz.
