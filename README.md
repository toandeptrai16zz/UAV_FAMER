> **BẢN NÂNG CẤP 04/11/2026:** đọc [HUONG_DAN_NANG_CAP_WIFI_PID_OTA_VI.md](HUONG_DAN_NANG_CAP_WIFI_PID_OTA_VI.md) trước khi nạp. UDP telemetry giữ nguyên; đã bổ sung PID qua HTTP, UART hai chiều và thử nghiệm ROM OTA (khóa mặc định). Bản này CHƯA ĐƯỢC KIỂM NGHIỆM TRÊN PHẦN CỨNG.

# UAV DATN — STM32G474RE

Rà soát 03/11/2026: đọc [STABILITY_REVIEW_VI.md](STABILITY_REVIEW_VI.md)
trước khi build/nạp. Gói nâng cấp giữ source và tài liệu; đã lược bỏ thư mục `.git` và các binary cũ trong `Debug/Release` để tránh nạp nhầm.

Firmware thử nghiệm cho board NUCLEO-G474RE. Nhận SBUS, điều khiển bốn ESC
bằng DShot300, đọc IMU MPU6500 và hỗ trợ cảm biến DPS310, MT-06.

> **FLIGHT CANDIDATE:** ARM bay đã được mở, bộ quét ESC tuần tự đã tắt. Đây là
> bản chuyển từ bench sang kiểm tra điều khiển bay có kiểm soát, không phải firmware
> đã được chứng nhận an toàn bay. Phải hoàn thành kiểm tra chiều IMU/mixer và failsafe
> không cánh trước khi lắp cánh.

## Chức năng

- SBUS qua USART1; bốn kênh TIM3 phát DShot300 đồng bộ bằng DMA-burst.
- MPU6500 qua SPI2: ước lượng Roll/Pitch; bộ điều khiển góc + tốc độ góc dùng Roll/Pitch/Yaw.
- DPS310 qua I2C1: đọc áp suất, nhiệt độ và độ cao tương đối để ghi log; không
  tham gia điều khiển.
- MT-06 qua USART3: nhận khoảng cách MAVLink; dữ liệu chỉ dùng chẩn đoán.
- Log qua LPUART1.

## Kết nối

| Thiết bị / tín hiệu | Chân MCU | Ghi chú |
| --- | --- | --- |
| ESC 1–4 | PA6, PA7, PB0, PB1 | TIM3_CH1–CH4, DShot300 |
| SBUS RX | PB7 | USART1_RX, 100000 baud, 8E2, đảo mức RX bật |
| MPU6500 | PB13/PB14/PB15, CS PB12 | SPI2, mode 3 |
| DPS310 | PB8/PB9 | I2C1 SCL/SDA; địa chỉ 0x76 hoặc 0x77 |
| MT-06 RX | PC11 | USART3_RX, 115200 baud; cấu hình cảm biến UART/MAVLink |
| UART log | PA2/PA3 | LPUART1, 115200 baud |

### Sơ đồ đấu nối

```mermaid
flowchart LR
    FC["NUCLEO-G474RE<br/>STM32G474RE"]
    RX["Receiver SBUS"]
    ESC["ESC 1–4"]
    IMU["MPU6500"]
    BARO["DPS310"]
    RANGE["MT-06"]
    PC["Máy tính<br/>ST-LINK VCP"]

    RX -- "SBUS OUT → PB7<br/>USART1_RX" --> FC
    FC -- "PA6 / PA7 / PB0 / PB1<br/>TIM3_CH1–CH4, DShot300" --> ESC
    FC -- "PB13 SCK · PB14 MISO<br/>PB15 MOSI · PB12 CS" --> IMU
    FC -- "PB8 SCL · PB9 SDA<br/>I2C1" --> BARO
    RANGE -- "UART TX → PC11<br/>USART3_RX, 115200" --> FC
    FC -- "PA2 TX · PA3 RX<br/>LPUART1, 115200" --> PC
```

Nối chung GND giữa NUCLEO, receiver, cảm biến và tín hiệu ESC. Cấp nguồn cho
ESC từ nguồn phù hợp; không cấp nguồn động cơ qua NUCLEO. MT-06 chỉ cần nối
TX của cảm biến vào PC11 để gửi dữ liệu; PC10 (USART3_TX) không được driver
sử dụng. [PINOUT.png](PINOUT.png) tham khảo vị trí chân trên board.

Kiểm tra điện áp, cực tính tín hiệu và sơ đồ chân của board/module thực tế
trước khi đấu nối. I2C cần điện trở kéo lên phù hợp.

Kênh SBUS mặc định: CH1 Roll, CH2 Pitch, CH3 ga, CH4 Yaw, CH5 ARM,
CH6 KILL, CH7 Alt-Hold (đang bị tắt trong flight candidate). Tâm stick và dải ga
được lấy từ log SBUS gần nhất trong project; nếu đổi transmitter/receiver hoặc
trim stick, phải hiệu chuẩn lại.

## Trình tự ARM

Trong flight candidate, pilot ARM được mở bằng cấu hình biên dịch; bộ quét ESC đã tắt.
State machine vẫn yêu cầu SBUS mới và ổn định, KILL OFF, ga thấp ổn định, stick trung
lập, IMU mới/đã calibration, frame gần ngang và DShot DMA phục hồi. Phải gạt ARM OFF
trước, sau đó mới ON để tạo cạnh ARM hợp lệ. `AR=1` là Armed; `SP=1` chỉ xuất hiện
sau khi ga vượt ngưỡng bắt đầu quay.

**Trước khi lắp cánh:** hoàn thành checklist trong `FLIGHT_CANDIDATE_VI.md`.
Đặc biệt kiểm tra bằng gá cố định và tháo đủ cánh: nghiêng phải thì cặp motor bên
phải M2/M4 phải nhận correction tăng; chúi mũi xuống thì cặp trước M1/M2 phải nhận
correction tăng. Tắt transmitter để xác nhận receiver tạo đúng failsafe pattern.
Nếu một dấu bị ngược, không lắp cánh và không thử bay.

## Build

Cần CMake, Ninja và GNU Arm Embedded Toolchain (`arm-none-eabi-gcc`):

```sh
cmake --preset Debug
cmake --build --preset Debug
```

Firmware đầu ra: `build/Debug/UAV_.elf`.

## Tài liệu

- [PINOUT.png](PINOUT.png): sơ đồ chân NUCLEO-G474RE.
- [FLIGHT_CANDIDATE_VI.md](FLIGHT_CANDIDATE_VI.md): cấu hình, pre-arm và checklist trước cánh.

## UAV Telemetry (STM32G474RE + ESP32-CAM AI-Thinker)

Tài liệu sơ đồ UART/nguồn, packet protocol, firmware ESP32-CAM và Ground Station nằm tại [`Telemetry/README_VI.md`](Telemetry/README_VI.md).
