# UAV Telemetry – STM32G474RE + ESP32-CAM AI-Thinker

## 1. Mục tiêu và ranh giới an toàn

- STM32G474RE tiếp tục làm Flight Controller: SBUS, IMU, PID, mixer và DShot vẫn chạy độc lập.
- ESP32-CAM chỉ nhận dữ liệu UART và chuyển tiếp bằng Wi-Fi UDP tới laptop.
- ESP32 không gửi lệnh điều khiển ngược về STM32; không được đưa Wi-Fi/telemetry vào điều kiện ARM hoặc vòng điều khiển.
- Mất Wi-Fi, laptop tắt hoặc ESP32 reset không được làm thay đổi đầu ra điều khiển của STM32.
- Đây là telemetry để bench và quan sát thử nghiệm. Nó không thay thế bộ ghi log độc lập trên UAV và không chứng minh liên kết RC 2.4 GHz an toàn khi Wi-Fi cùng hoạt động.

## 2. Sơ đồ dây

### UART một chiều

| NUCLEO-G474RE | ESP32-CAM AI-Thinker | Ghi chú |
|---|---|---|
| PA2 – LPUART1_TX (CN9 D1/TX) | GPIO13 – UART1_RX | Nối qua điện trở nối tiếp 470 Ω–1 kΩ |
| GND | GND | Bắt buộc chung mass |
| Không nối PA3 | GPIO13 là RX duy nhất của bridge | UART0 GPIO1/GPIO3 dành cho nạp/debug |

**Không nối TX ESP32 về STM32 ở phiên bản này.** Không nối tín hiệu 5V vào GPIO13; UART logic của hai chip là 3.3V.

AI-Thinker ESP32-CAM dùng GPIO13 làm SD_DATA3 khi dùng microSD. Firmware này không khởi tạo camera hoặc SD_MMC; tháo thẻ microSD và không thêm thư viện/khởi tạo SD nếu muốn dùng GPIO13 làm UART1_RX.

### Nguồn

```text
LiPo / PDB
   ├── BEC 5V hiện có ──> nguồn Flight Controller theo thiết kế hiện tại
   └── Buck 5V riêng, ổn định ──> chân 5V ESP32-CAM
                                  GND ──> GND ESP32-CAM

STM32 GND ────────────────────────┘ (GND chung)
STM32 PA2 ── 470R..1k ── ESP32 GPIO13
```

- Cấp ESP32-CAM vào chân **5V**, không cấp 5V vào chân 3V3.
- Dùng bộ buck riêng cho ESP32-CAM, chọn nguồn 5V có dư địa dòng cho xung tải Wi-Fi; khởi điểm thiết kế nên dùng module 5V/1A trở lên, ưu tiên 2A nếu module/cách đi dây cho phép.
- Đặt tụ 470 µF (điện áp định mức tối thiểu 10 V) song song với 10 µF và 100 nF gần chân 5V/GND của ESP32-CAM.
- Không nối chập hai đầu ra dương của hai BEC/buck độc lập. Chỉ nối chung GND.
- Khi bench bằng USB, chú ý tránh back-feed 5V giữa NUCLEO, USB và buck ngoài. Chỉ nối GND chung và cấp ESP32 từ buck riêng.

## 3. Cấu hình STM32 hiện có

Project đã có LPUART1 trên PA2/PA3, 115200 baud, 8 data bits, no parity, 1 stop bit. Không cần thêm UART hoặc sửa pin trong CubeMX cho bản này.

Các file được thêm/sửa:

- `Core/Inc/uav_telemetry.h`: định nghĩa telemetry sample, cờ trạng thái và API.
- `Core/Src/uav_telemetry.c`: đóng gói packet 88 byte, CRC16-CCITT-FALSE và TX không chặn qua `HAL_UART_Transmit_IT`.
- `Core/Src/uav_test.c`: phát snapshot 50 Hz từ các biến trạng thái hiện có; vẫn giữ PID/mixer/DShot hiện tại và không phụ thuộc kết quả truyền telemetry.
- `cmake/stm32cubemx/CMakeLists.txt`: thêm `uav_telemetry.c` vào danh sách build tường minh.

UART LPUART1 trước đó được dùng cho log ASCII. Log ASCII vẫn được giữ nhưng giảm từ 5 Hz xuống 1 Hz (`LOG_PERIOD_MS=1000`) để ưu tiên băng thông cho telemetry. Telemetry nhị phân được ghép trên cùng đường UART; nếu UART đang bận, snapshot telemetry bị bỏ (không chờ, không block motor loop). ESP32 tìm magic `A5 5A`, xác thực version/type/CRC và bỏ qua dòng ASCII. Vì cùng một chân TX mang cả text và packet nhị phân, Serial Monitor/VCP ở PA2 sẽ không còn là console chữ thuần; dùng Ground Station Wi-Fi để xem telemetry.

Chu kỳ telemetry mục tiêu: 20 ms (50 Hz). Mỗi packet dài 88 byte, tương đương khoảng 44 kbit/s ở 50 Hz khi tính 10 bit UART/byte; vẫn có phần dự phòng cho log ASCII ở 115200 baud. `TLM` và `TLD` trong dòng ASCII lần lượt là số packet telemetry hoàn tất và số lần bỏ packet.

### Những dữ liệu được gửi

- RC raw: throttle, roll, pitch, yaw.
- Attitude: roll/pitch (0.01°), gyro body rate (0.1°/s), accelerometer raw.
- PID correction (0.1 DShot-count), BASE, MIX, mixer scale, mixer offset.
- Lệnh mỗi motor trước safety gate (REQ) và lệnh cuối cùng sau gate (OUT).
- Rangefinder distance nếu dữ liệu hợp lệ.
- IMU timing, motor loop timing, DShot error count và receiver age.
- `VBAT` và `CURRENT` hiện gửi `0xFFFF` (không có cảm biến/đường ADC được xác thực trong project); ground station hiển thị trống, tuyệt đối không coi là 0 V/0 A.

## 4. Firmware ESP32-CAM

Thư mục: `Telemetry/ESP32_CAM/`

- `UAV_Telemetry_Bridge.ino`
- `platformio.ini`

Firmware dùng UART1 RX GPIO13, 115200 8N1. GPIO1/GPIO3 (UART0) dành cho USB-to-UART nạp và Serial Monitor.

ESP32 tạo SoftAP:

| Thông số | Giá trị mặc định |
|---|---|
| SSID | `UAV-TELEM` |
| Password | `UAVtelemetry26` |
| AP IP | `192.168.4.1` |
| Wi-Fi channel | 1 (bench default; phải khảo sát phổ RF trước khi thử bay) |
| UDP port | 14550 |
| Max TX power | 2 dBm (cấu hình ban đầu thấp) |

Ground station gửi `UAVT_HELLO` qua UDP/14550; ESP32 ghi nhớ IP/port nguồn và unicast các packet telemetry đến laptop. Nếu không nhận HELLO trong 3.5 giây, bridge ngừng gửi cho peer cũ.

ESP-IDF định nghĩa tham số `esp_wifi_set_max_tx_power()` theo đơn vị 0.25 dBm; giá trị 8 tương ứng giới hạn 2 dBm. Mức thấp chỉ là cấu hình giảm công suất khởi đầu, không bảo đảm không gây nhiễu RC.

## 5. Ground Station (Pop!_OS / Ubuntu)

Thư mục: `Telemetry/GroundStation/`

```bash
cd Telemetry/GroundStation
python3 uav_ground_station.py
```

Trước đó kết nối laptop vào Wi-Fi `UAV-TELEM`, password `UAVtelemetry26`.

Tùy chọn đồ thị realtime:

```bash
python3 -m pip install matplotlib
python3 uav_ground_station.py --plot
```

Đổi file CSV:

```bash
python3 uav_ground_station.py --csv flight_test_01.csv
```

Nếu firewall chặn UDP, cho phép UDP inbound cổng 14550 trên laptop. Ground station bind `0.0.0.0:14550`, gửi HELLO đến `192.168.4.1:14550` và ghi packet hợp lệ ra CSV.

## 6. Định dạng packet v1 – 88 byte

Tất cả trường nhiều byte là little-endian. CRC là CRC-16/CCITT-FALSE (poly `0x1021`, init `0xFFFF`, xorout `0`) trên byte 0..85; CRC nằm ở byte 86..87.

| Offset | Kích thước | Trường | Scale / ghi chú |
|---:|---:|---|---|
| 0 | 2 | Sync | `A5 5A` |
| 2 | 1 | Version | 1 |
| 3 | 1 | Type | 1 = flight snapshot |
| 4 | 2 | Sequence | uint16, tăng mỗi slot gửi (có gap nếu drop) |
| 6 | 4 | Timestamp | HAL tick ms |
| 10 | 2 | Flags | bit 0 link, 1 failsafe, 2 armed, 3 spinning, 4 IMU fresh, 5 KILL active, 6 alt hold, 7 IMU calibrated, 8 range valid, 9 DMA error, 10 output active |
| 12 | 8 | RC | THR, ROLL, PITCH, YAW raw (4 × uint16) |
| 20 | 4 | Attitude | roll/pitch ×100, int16 |
| 24 | 6 | Gyro | roll/pitch/yaw ×10 dps, int16 |
| 30 | 6 | Accelerometer | X/Y/Z raw, int16 |
| 36 | 6 | PID | roll/pitch/yaw ×10, int16 |
| 42 | 2 | BASE | DShot count, uint16 |
| 44 | 2 | Mixer scale | ×1000, uint16 |
| 46 | 2 | Mixer offset | DShot count, int16 |
| 48 | 8 | MIX | M1..M4 ×10, int16 |
| 56 | 8 | REQ | M1..M4 requested DShot, uint16 |
| 64 | 8 | OUT | M1..M4 final DShot, uint16 |
| 72 | 2 | VBAT | mV; `FFFF` nếu chưa đo |
| 74 | 2 | CURRENT | mA; `FFFF` nếu chưa đo |
| 76 | 2 | Range | cm; `FFFF` nếu invalid |
| 78 | 2 | IMU dt max | ms |
| 80 | 2 | Motor dt max | ms |
| 82 | 2 | DShot errors | uint16 saturated |
| 84 | 2 | RX age | ms, uint16 saturated |
| 86 | 2 | CRC16 | CRC bytes 0..85 |

## 7. Trình tự kiểm tra

1. Tháo cả bốn cánh; bench với STM32 và ESP32 cấp nguồn riêng như sơ đồ. Chưa nối nguồn công suất ESC nếu chỉ kiểm tra telemetry.
2. Nạp firmware ESP32-CAM bằng USB-to-UART 3.3V logic: GPIO0 xuống GND khi flash; tháo GPIO0 khỏi GND và reset để chạy.
3. Build/flash project STM32 bằng STM32CubeIDE hoặc CMake toolchain ARM. Kiểm tra LPUART1 115200, PA2 TX.
4. Nối PA2 → điện trở 470 Ω–1 kΩ → GPIO13; nối GND chung.
5. Kết nối laptop vào `UAV-TELEM`, chạy ground station.
6. Xác nhận `packets` tăng khoảng 50 packet/s, `invalid=0`, `seq_gaps` thấp/0 trong điều kiện UART rảnh; ESP32 Serial Monitor phải cho `frames` và `udp_ok` tăng.
7. So sánh CSV với log ASCII. Thử ngắt Wi-Fi/đóng ground station: STM32 phải tiếp tục vòng điều khiển như trước; telemetry chỉ bị drop.

## 8. Lưu ý RF – đặc biệt quan trọng vì RC cũng 2.4 GHz

ESP32-CAM AI-Thinker dùng Wi-Fi 2.4 GHz; hệ RC của người dùng cũng dùng 2.4 GHz nhưng giao thức/model receiver chưa được xác định. Không thể kết luận kênh Wi-Fi nào an toàn chỉ từ thông tin “2.4 GHz”.

- Không bay chỉ vì telemetry đã nhận được packet.
- Thử nghiệm trước ở mặt đất, không lắp cánh: kiểm tra range test của RC theo đúng quy trình của nhà sản xuất khi Wi-Fi tắt và khi Wi-Fi bật.
- Theo dõi frame-loss/failsafe và RSSI/link quality từ thiết bị RC nếu có; nếu chất lượng giảm, tắt Wi-Fi hoặc chuyển sang đường telemetry khác băng tần.
- Đặt anten ESP32 và anten receiver cách nhau tối đa theo bố trí cơ khí, tránh đặt sát nhau hoặc song song sát đường nguồn ESC. Khoảng cách vật lý không thay thế kiểm tra range test.
- 2 dBm là mức TX power khởi đầu cho bench, không phải đảm bảo tương thích RF hay yêu cầu pháp quy.

## 9. Giới hạn đã biết

- Chưa có đường ADC/cảm biến dòng được xác thực trong project, nên VBAT/CURRENT chưa đo và sẽ hiển thị trống.
- Chưa có bộ ghi cục bộ microSD; ESP32 không khởi tạo microSD vì GPIO13 được dùng cho UART1 RX.
- Wi-Fi UDP không đảm bảo truyền đủ mọi packet; sequence và CSV giúp phát hiện mất gói.
- Chưa xác nhận build STM32 ARM hoặc upload ESP32 trong môi trường tạo gói nếu toolchain/board không hiện diện. Phải build trên STM32CubeIDE/PlatformIO trước khi cấp nguồn phần công suất.
