# HƯỚNG DẪN NÂNG CẤP UAV_DATN — STM32 + ESP32-CAM + Ground Station

**Phiên bản:** 2026-11-04 | **Đối tượng:** UAV sử dụng STM32G474RE, ESP32-CAM AI-Thinker, MC6C/MC7RE SBUS, DShot300.

> ⚠️ Đây là bản tích hợp/kỹ thuật thử nghiệm, **chưa có xác nhận nạp chạy trên bo thật và chưa được chứng nhận an toàn bay**. Bắt buộc tháo cánh quạt khi thử giao tiếp, flash PID, bootloader. Duy trì ST-LINK và SWD để phục hồi. Đừng thử OTA khi có cánh hoặc ESC đang cấp nguồn motor.

## 1. Những gì đã thực hiện

Đã **giữ nguyên** SBUS, logic ARM/KILL/failsafe, IMU MPU6500, DPS310, MT-06, PID cascade, mixer, DShot300/DMA, log ASCII, UDP telemetry 50 Hz và script thu CSV. Không thay đổi mapping ga của phi công và mixer motor.

Mở rộng:

1. `Core/Src/flight_pid.c` + `Core/Inc/flight_pid.h`: 13 tham số PID runtime, mặc định đúng với firmware gốc, kiểm tra finite/giới hạn. **APPLY chỉ trong RAM**.
2. `Core/Src/uav_config.c` + `Core/Inc/uav_config.h`: giao thức UART hai chiều CRC16, RX ngắt + ring buffer, TX không blocking, lệnh GET/APPLY/SAVE/BOOT với điều kiện an toàn.
3. `Core/Src/uav_test.c`: kiểm tra **RC CH8 (index 7)**, ARM OFF, KILL ON, ga thấp, RC link ổn định, failsafe; chặn nhận lệnh khi đang ARM. CH8 không tự cắt motor nếu đang bay; CH8 chỉ ngăn quá trình ARM mới.
4. PID lưu ở hai trang Flash cuối, có generation và checksum; commit ghi sau cùng. Chỉ hoạt động nếu STM32 đang dùng **512KB dual-bank, không bank-swap**.
5. `Telemetry/ESP32_CAM/UAV_Telemetry_Bridge.ino`: UDP v1 cũ (88 byte) và web HTTP mới; UART hai chiều, PID, firmware staging SPIFFS và bộ nạp ROM USART.
6. `Telemetry/GroundStation/uav_control.py`: laptop GET/SET/SAVE/OTA. `uav_ground_station.py` cũ vẫn dùng để xem telemetry và lưu CSV.
7. `STM32G474RETX_FLASH.ld` và `STM32G474xx_FLASH.ld`: đổi FLASH từ **512K thành 508K**, chừa 4 KiB cuối cho PID config.

## 2. Sơ đồ đấu dây

| STM32G474RE | ESP32-CAM AI Thinker | Chức năng |
|---|---|---|
| PA2 / LPUART1_TX | GPIO13 / UART1_RX | Telemetry, trả lời lệnh |
| PA3 / LPUART1_RX | GPIO14 / UART1_TX | ESP32 gửi lệnh PID / OTA |
| GND | GND | Mass chung |
| 3.3V logic | 3.3V logic | Không dùng mức UART 5 V |

ESP32-CAM nên được cấp từ **nguồn 5 V ổn định phù hợp yêu cầu bo** qua chân 5V; không nuôi cả ESP32-CAM bằng chân 3.3V yếu của ST-LINK. Hai mạch **phải chung GND**, các đường UART là logic 3.3 V. Cấp nguồn theo thiết kế điện thực tế, không nối chập hai ngõ ra nguồn 3.3 V.

- GPIO13, GPIO14 cùng dùng với microSD trên ESP32-CAM; **không dùng microSD** với firmware này.
- Kiểm tra solder bridge ST-LINK VCP của NUCLEO (PA2/PA3 có thể đã nối ST-LINK). **Không để TX ST-LINK và TX ESP32 đồng thời lái PA3.** Xác minh board schematic và hàn jumper thực tế.
- BOOT0 của STM32G474RE đang ghép **PB8/I2C1_SCL** với DPS310: **không đấu ESP32 vào PB8/BOOT0**. Đường OTA dùng software jump đến ROM bootloader, chỉ cho phép sau xác nhận riêng trên phần cứng.
- Không cần đấu NRST trong bản thử đầu. Nếu chuyển sang OTA sản phẩm, nghiên cứu mạch NRST có transistor/open-drain và chân ESP không ảnh hưởng strapping; **không nối thẳng tùy tiện**.

## 3. Điều kiện cho phép CONFIG

Từ máy phát RC (đọc giá trị **SBUS raw**, không phải microseconds):

| Điều kiện | Yêu cầu |
|---|---|
| CH5 ARM (index 4) | OFF, thấp hơn 1320 |
| CH6 KILL (index 5) | ON, từ 1500 trở lên |
| CH8 CONFIG (index 7) | ON, từ 1500 trở lên |
| CH3 throttle (index 2) | Ga thấp theo `RC_ThrottleIsLow` |
| Liên kết RC | Không failsafe, không preset MC7RE, ổn định 1000 ms |
| Trạng thái motor | Không ARM, không spinning, không bench scanner |

Cờ mới `config_ready` ở bit 11 của UDP telemetry cho biết STM32 hiện đủ điều kiện CONFIG. Nếu CH8 chưa được thiết lập trên tay điều khiển hoặc receiver không đưa dữ liệu ở kênh 8 thì **không mở được CONFIG**. Không thay bằng phần mềm bỏ điều kiện này; thay vào đó hãy cấu hình kênh RC hoặc thiết kế nút cho phép riêng trên bo.

**Không có lệnh Wi-Fi nào để ARM, đổi ga hoặc điều khiển motor.** Mất Wi-Fi chỉ làm mất kết nối cấu hình/telemetry; không thay logic failsafe RC.

## 4. Build và nạp STM32 lần đầu bằng ST-LINK

1. Backup firmware hiện có và lưu file `.elf` / `.bin` của bản đang chạy. Giữ nguyên dây SWD, nạp ST-LINK như cũ.
2. Mở thư mục `UAV_DATN` bằng **STM32CubeIDE**. Clean rồi Build (Project > Clean, Project > Build Project). File mới `Core/Src/uav_config.c` phải được IDE compile và link. Nếu dùng CMake, file đã được thêm trong `cmake/stm32cubemx/CMakeLists.txt`.
3. Đảm bảo linker script đang được dùng là `STM32G474RETX_FLASH.ld` (CubeIDE) hoặc `STM32G474xx_FLASH.ld` (CMake), và có: `ORIGIN = 0x8000000, LENGTH = 508K`.
4. **Không để phần mềm sinh lại file linker thành 512K**, nếu không SAVE có thể xóa code của chính mình. Nếu CubeMX regenerate hãy kiểm tra lại file linker và danh sách source.
5. Nạp STM32 bằng ST-LINK. Xem log/telemetry như trước. Trước khi thử chỉnh PID, chạy kiểm tra ARM/KILL, SBUS failsafe, DShot không cánh, các sensor.
6. Kiểm tra option byte `DBANK`. Nếu single-bank, lệnh SAVE sẽ từ chối. **Không tự thay option bytes khi chưa hiểu ảnh hưởng đến vùng Flash và cách phục hồi.** Trong trường hợp đó vẫn có thể GET và APPLY tạm thời nếu đủ điều kiện CONFIG.

Firmware STM32 khởi động với bộ PID mặc định: angle_kp=4, max_rate_dps=80; roll_kp=1.5, pitch_kp=1.5, yaw_kp=1; toàn bộ Ki/Kd=0; integral_limit=30; correction_limit=200. Những số này **không phải gain đã kiểm nghiệm bay**.

### Kích thước và giới hạn

- Flash MCU: 512KB, vùng ứng dụng **0x08000000..0x0807EFFF** (508 KiB).
- PID config 2 trang: **0x0807F000..0x0807F7FF**, **0x0807F800..0x0807FFFF**.
- Chỉ ghi Flash khi disarm/KILL/ga thấp. Save tạm dừng main-loop trong lúc erase/program; motor phải ở trạng thái an toàn.
- Không dùng cách lưu Flash này nếu board thay chip, nạp bootloader riêng, single-bank, swap-bank, hoặc thêm phân vùng khác mà chưa thiết kế lại linker.

## 5. Build và nạp ESP32-CAM

Mở `Telemetry/ESP32_CAM` trong **VS Code + PlatformIO**; dự án đã có `platformio.ini`, cấu hình `src_dir = .` để compile file `.ino` ngay trong thư mục.

1. Mở `secrets.h` và **đổi cả** `UAV_WIFI_PASSWORD` (từ 8 ký tự) và `UAV_API_TOKEN` (ít nhất 24 ký tự mạnh). Nếu giữ nguyên placeholder, chương trình **cố ý từ chối bật Wi-Fi**.
2. Upload ESP32-CAM bằng UART USB-TTL 3.3 V (đường serial logic). Khi flash ESP32-CAM, GPIO0 xuống GND theo quy trình board, sau đó tháo GPIO0 khỏi GND và reset để chạy.
3. Trong `Telemetry/ESP32_CAM`, chạy:

```sh
pio run -e esp32cam
pio run -e esp32cam -t upload
pio device monitor -b 115200
```

4. Gắn ESP32 UART1 với STM32 theo bảng đấu dây. Kết nối laptop đến AP **UAV-TELEM**, mật khẩu mới. Mở `http://192.168.4.1/` để xem giao diện web PID.
5. Nhập token trong giao diện; chọn **Đọc STM32**. Nếu trả `STM32 no reply`, kiểm tra cả hai dây TX/RX, GND, 115200 8N1, jumper ST-LINK VCP và việc STM32 đã nạp firmware mới.

## 6. Thử telemetry trước, rồi chỉnh PID

Dùng Python 3, **không cần package ngoài** nếu chỉ CLI/CSV:

```sh
cd Telemetry/GroundStation
python3 uav_ground_station.py --csv run_001.csv
```

Từ terminal khác (cũng đã kết nối Wi-Fi ESP32):

```sh
export UAV_API_TOKEN='TOKEN_DA_DOI_TRONG_secrets.h'
python3 uav_control.py status
python3 uav_control.py get
```

Thử chuyển công tắc CH8=ON, ARM=OFF, KILL=ON, throttle về thấp, chờ RC ổn định ít nhất 1 giây. Khi đó telemetry `config_ready=true` và API `config_allowed=true`.

Ví dụ **chỉ thử trên bệ tháo cánh** (không coi là giá trị khuyến nghị bay):

```sh
python3 uav_control.py set roll_kp 1.2     # APPLY vào RAM, chưa ghi Flash
python3 uav_control.py get                 # Xác minh giá trị đọc lại
python3 uav_control.py save                # Lưu hai-trang Flash nếu đủ điều kiện
```

Sau khi Save: reset STM32 bằng nút reset và GET lại để kiểm tra giá trị còn tồn tại. Khi không Save, reset sẽ nạp bộ đã lưu (hoặc giá trị mặc định nếu chưa có record hợp lệ).

### Các trường chỉnh PID

Giao thức gửi 13 float32 little-endian theo thứ tự: `angle_kp,max_rate_dps,roll_kp,pitch_kp,yaw_kp,roll_ki,pitch_ki,yaw_ki,roll_kd,pitch_kd,yaw_kd,integral_limit,correction_limit`.

Giới hạn code: angle 0..8, max_rate 1..150, rate Kp 0..3, rate Ki 0..1, rate Kd 0..0.1, integral_limit 0..50, correction_limit 1..250. Đây là **giới hạn từ chối giá trị vô lý**, không phải bảo đảm rằng toàn bộ miền giá trị đều ổn định cho UAV.

## 7. OTA STM32 qua ESP32 — **chỉ thử trên bàn**

### Vì sao không bật mặc định?

OTA có thể làm hỏng firmware đang chạy nếu mất điện giữa quá trình erase/write. Phương án hiện tại **chưa có rollback A/B cho firmware**, không có cơ chế tự cứu từ xa nếu bootloader không vào, và đường nhảy đến ROM/USART2 trên PA2/PA3 **chưa được xác nhận bằng bo thực tế**. Để tránh đổi trạng thái boot bất ngờ, `Core/Inc/uav_config.h` đang đặt:

```c
#define UAV_CONFIG_ENABLE_ROM_OTA 0
```

**Khi giá trị 0, `uav_control.py ota` sẽ bị STM32 từ chối, không xóa Flash.** Đây là hành vi an toàn có chủ đích, không phải OTA đã chạy thành công.

### Điều kiện để thử mở OTA

- Tự xác nhận ở bench STM32G474RET6 thực tế hỗ trợ jump ROM system memory tại **0x1FFF0000** và ROM bootloader giao tiếp bằng **USART2 PA2/PA3** 115200 8E1 sau `0x7F`. Theo ST **AN2606**, **AN3155**. Đo bằng logic analyzer trước.
- Cánh quạt đã tháo, pin động lực/ESC ngắt cấp điện, nguồn 3.3V/5V ổn định.
- Có ST-LINK/SWD nạp cứu firmware bất cứ lúc nào. Không áp dụng cho chuyến bay, không nạp ở ngoài hiện trường.
- Xác nhận `DBANK=1`, không swap bank, Flash đúng 512KB và linker đúng 508K.
- Đã chạy ổn định GET/APPLY/SAVE và test failsafe/CH8 trên bàn.

Chỉ sau đó mới đổi `#define UAV_CONFIG_ENABLE_ROM_OTA 1`, build/nạp **STM32 bằng ST-LINK một lần** để kích hoạt giao thức nhảy ROM.

### Quy trình gửi firmware

Trong STM32CubeIDE, build `Release`/`Debug` và xuất **raw `.bin` ứng dụng ở 0x08000000**, không gửi `.elf` hoặc `.hex`. Có thể dùng:

```sh
arm-none-eabi-objcopy -O binary build/UAV_.elf stm32_firmware.bin
```

Kiểm tra file `.bin` phải từ 8 byte đến 520192 byte; reset vector hợp lệ và thuộc vùng ứng dụng. Với bo **tháo cánh**, RC đang ở CONFIG và có SWD phục hồi:

```sh
cd Telemetry/GroundStation
export UAV_API_TOKEN='TOKEN_DA_DOI_TRONG_secrets.h'
python3 uav_control.py ota /path/to/stm32_firmware.bin --props-removed
```

Ứng dụng: laptop POST multipart kèm SHA-256 → ESP32 lưu SPIFFS và kiểm tra toàn file → STM32 ACK BOOT ở CONFIG → STM32 nhảy ROM → ESP32 đổi UART 8E1 → sync `0x7F` → Extended Erase **chỉ trang ứng dụng** → Write Memory (`0x31`) → Read Memory (`0x11`) xác minh từng block → Go (`0x21`) → ESP32 trở về UART 8N1. KHÔNG mass erase hai trang PID.

**Không dùng OTA nếu thử nhảy bootloader chưa thành công.** Lỗi tải/ghi/verify có thể khiến STM32 ở ROM hoặc trống ứng dụng; lúc đó dùng ST-LINK/SWD nạp lại file `.elf` hoặc `.bin` backup và kiểm tra lại vùng PID.

## 8. Wi-Fi 2.4 GHz và receiver

MQTT/HTTP/UDP đều dùng cùng radio Wi-Fi nếu chạy ESP32-CAM; đổi giao thức **không làm tách tần số khỏi receiver 2.4 GHz**. ESP32 hiện vẫn bật SoftAP như firmware cũ để giữ chức năng UDP telemetry khi bay, ở kênh 1, đặt max TX power mục tiêu 2 dBm. Công suất thấp không bảo đảm không nhiễu.

**Khuyến nghị để test đầu:** bật Wi-Fi chỉ khi trên bàn, vô hiệu hóa ESP32 phát sóng (ngắt nguồn riêng hoặc Wi-Fi OFF bằng firmware sau khi kiểm chứng) trước khi bay. Không cắt nguồn STM32 hoặc receiver. Nếu bắt buộc cần telemetry thời gian thực lúc bay, nên thiết kế đường truyền **5GHz hỗ trợ phần cứng** và kiểm tra antenna/RF/chất lượng receiver, hoặc dùng telemetry RF chuyên dụng phù hợp pháp luật, cách ly anten và đo failsafe thực tế. Không bao giờ dùng Wi-Fi link như điều kiện duy nhất quyết định trạng thái ARM.

## 9. Lỗi thường gặp

| Lỗi | Kiểm tra |
|---|---|
| ESP32 báo yêu cầu đổi `secrets.h` | Chưa thay Wi-Fi password/token; chương trình từ chối bật radio |
| Không thấy AP UAV-TELEM | GPIO0 còn kéo xuống GND; nguồn ESP yếu; sai build; secrets chưa đổi |
| UDP có nhưng GET PID timeout | Chưa đấu **GPIO14 → PA3**, chập PA3 với ST-LINK VCP; STM32 bản cũ; STM32 RX IRQ chưa hoạt động |
| `config_allowed=false` | CH8 chưa bật/mapping sai, ARM vẫn ON, KILL OFF, ga cao, link chưa ổn định hoặc failsafe MC7RE latch |
| PID GET được, APPLY trả status 1 | STM32 đang khóa CONFIG; đây là hành vi đúng |
| APPLY trả status 3 | Một trường không phải số hữu hạn, hoặc vượt range |
| SAVE trả status 5 | Dual bank / flash configuration chưa đúng, hoặc erase/program fail; **không ép ghi** |
| OTA trả status 4 | `UAV_CONFIG_ENABLE_ROM_OTA=0`: khóa mặc định; chưa xác nhận bench |
| OTA đã bật nhưng thất bại đồng bộ ROM | Kiểm tra ROM boot handshake/PA2-PA3, board VCP, 8E1; dùng SWD để cứu khi cần |
| STM32 không khởi động sau OTA | Không cố ARM; backup + ST-LINK khôi phục; kiểm tra 2 trang PID và image vector |
| UAV không giữ ổn định | Đây là vấn đề mixer/trục IMU/PID/ESC, không được giải quyết chỉ bằng Wi-Fi; làm quy trình kiểm thử không cánh trước |

## 10. Giao thức và API (cho người phát triển)

- UDP telemetry **không đổi**: sync `A5 5A`, v1 type 1, 88 byte, CRC16/CCITT-FALSE, UDP 14550. Cờ mới bit 11 = `CONFIG_READY`; các offset khác không đổi.
- UART config: `C3 3C | version=1 | type (GET 0x10, APPLY 0x11, SAVE 0x12, BOOT 0x13) | seq uint16 LE | len uint8 | payload | CRC16 LE`. Response `type|0x80`, payload `status:uint8`, `allowed:uint8`, có thể kèm 52 byte gains; size max 80.
- Status `0=OK`, `1=LOCKED`, `2=WRONG_LEN`, `3=OUT_OF_RANGE`, `4=DISABLED`, `5=FLASH_ERROR`, `6=BUSY`.
- HTTP `GET /api/pid`, `POST /api/pid` (13 số CSV ASCII), `POST /api/save`, `GET /api/status`, `POST /api/ota` (multipart file + SHA-256); tất cả API bắt buộc `X-Auth-Token`.
- HTTP không dùng TLS trên SoftAP; bảo vệ bằng WPA2 password mạnh và token riêng, **chỉ sử dụng mạng riêng cho lab**. SHA-256 chỉ bảo đảm toàn vẹn nội dung gửi lên, không thay chữ ký firmware hoặc secure boot.

## 11. Cách kiểm thử và phạm vi xác nhận

Máy phát triển tại thời điểm đóng gói không có `arm-none-eabi-gcc` hoặc PlatformIO/ESP32 compiler. Đã thực hiện:

- GCC host `-fsyntax-only` cho `flight_pid.c`, `uav_config.c`, `uav_test.c` với HAL headers gốc (có thể bỏ qua cảnh báo khác biệt con trỏ 32/64-bit của host).
- `bash tests/run_tests.sh`: chạy logic điều khiển với HAL mock và UBSan; gồm ARM/KILL, failsafe, độ trễ IMU, DShot, mixer, kiểm tra gain runtime, CH8 config.
- `python3 -m py_compile .../uav_control.py .../uav_ground_station.py`.

**Chưa xác nhận:** ARM firmware build/link, ESP32 Arduino/PlatformIO build, UART hai chiều vật lý, flash erase dual-bank thực tế, ROM bootloader handoff và OTA end-to-end, RF tương tác receiver, hay bay thử. Đây là những thử nghiệm phải hoàn tất trước khi gọi là "bản hoạt động ổn định".

## 12. Tài liệu gốc cần đối chiếu

- [ST AN2606 — System memory boot mode](https://www.st.com/resource/en/application_note/an2606-introduction-to-system-memory-boot-mode-on-stm32-mcus-stmicroelectronics.pdf), đặc biệt STM32G47xxx/48xxx và USART2 pin map.
- [ST AN3155 — USART bootloader protocol](https://www.st.com/resource/en/application_note/cd00264342-usart-protocol-used-in-the-stm32-bootloader-stmicroelectronics.pdf), sync/command/erase/read/write/Go.
- [ST RM0440 — STM32G4 Reference Manual](https://www.st.com/resource/en/reference_manual/dm00355726-stm32g4-series-reference-manual-stmicroelectronics.pdf), Flash single/dual bank, Option Bytes, page 2KiB.
- [Espressif ESP32 hardware design](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32/schematic-checklist.html), strapping pins và RF.

---

### Checklist trước bất kỳ thử nghiệm nào

- [ ] Đã backup firmware STM32 và có ST-LINK hoạt động.
- [ ] Đã tháo **tất cả cánh quạt**, ngắt nguồn động lực ESC để test config/OTA.
- [ ] Đã kiểm tra dây UART, GND, nguồn ESP32-CAM, VCP solder bridges.
- [ ] Đã đổi 2 secrets, kết nối SoftAP, GET telemetry, GET PID.
- [ ] Đã kiểm tra RC CH8/ARM/KILL/failsafe và cờ `config_ready`.
- [ ] Đã Apply một thông số thử nhỏ, GET phản hồi, reset kiểm tra Save trên bench.
- [ ] Chưa bật ROM OTA khi chưa có kết quả logic analyzer + phục hồi ST-LINK.
- [ ] Chưa bay khi chưa xác minh hướng IMU/mixer/ESC và tương tác RF 2.4GHz.
