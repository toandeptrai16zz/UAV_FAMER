# UAV STM32G474RE – DShot300 + SBUS + MPU6500 + DPS310

## 1. Phạm vi gói nguồn

Đây là **gói mã nguồn Core để thay thế/tích hợp vào project STM32CubeIDE hiện có**, không phải project CubeIDE độc lập hoàn chỉnh. Gói không chứa thư viện HAL/BSP, CMSIS startup, linker script hoặc các file cấu hình IDE của project gốc vì các phần đó không được gửi trong bộ file đầu vào.

Cấu trúc được chuẩn hóa theo tên file CubeIDE, không có các hậu tố `(1)`, `(5)`, `(8)` và không chứa chương trình test có `main()` thứ hai:

```text
UAV_G474_DShot300_clean/
├── Core/
│   ├── Inc/
│   │   ├── dps310.h
│   │   ├── dshot_protocol.h
│   │   ├── main.h
│   │   ├── stm32g4xx_hal_conf.h
│   │   ├── stm32g4xx_it.h
│   │   ├── stm32g4xx_nucleo_conf.h
│   │   └── uav_test.h
│   └── Src/
│       ├── dps310.c
│       ├── main.c
│       ├── stm32g4xx_hal_msp.c
│       ├── stm32g4xx_it.c
│       ├── syscalls.c
│       ├── sysmem.c
│       ├── system_stm32g4xx.c
│       └── uav_test.c
├── UAV_.ioc
└── README_VI.md
```

**Không có `Core/tests/test_dshot_vectors.c` trong gói firmware.** Đây là chương trình test host có hàm `main()` riêng; trước đây đưa nó vào build firmware đã gây lỗi `multiple definition of main`. Các file `syscalls.c`, `sysmem.c`, `system_stm32g4xx.c` không phải file thừa; đây là các file runtime/system do CubeIDE/CMSIS dùng.

## 2. Phần cứng và pin đang cấu hình

| Chức năng | Peripheral / chân | Ghi chú |
|---|---|---|
| DShot motor 1 | TIM3_CH1 – PA6 | M1 theo giả định mixer hiện tại |
| DShot motor 2 | TIM3_CH2 – PA4 | M2 |
| DShot motor 3 | TIM3_CH3 – PB0 | M3 |
| DShot motor 4 | TIM3_CH4 – PB1 | M4 |
| SBUS RX | USART1_RX – PB7 | 100 kbaud, 8E2, bật đảo RX trong cấu hình hiện tại |
| UART log | LPUART1 – PA2/PA3 | 115200 baud |
| MPU6500 SPI | SPI2 – PB13/PB14/PB15 | Mode 3; CS = PB12 |
| DPS310 I2C | I2C1 – PB8/PB9 | SCL/SDA; driver thử địa chỉ 7-bit 0x77 rồi 0x76 |

Kiểm tra lại pinout bo mạch thực tế và dây tín hiệu trước khi cấp nguồn ESC. PB8/PB9 cần điện trở kéo lên I2C phù hợp; driver đang để GPIO `NOPULL` vì giả định module DPS310 có pull-up sẵn.

## 3. DShot300 và DMA-burst

- System clock được cấu hình 170 MHz; APB1 prescaler = 1 nên TIM3 clock là 170 MHz.
- TIM3 dùng PSC = 0, ARR = 566, tức 567 timer ticks/bit: `170 MHz / 567 ≈ 299.82 kbit/s`.
- T0H = 213 ticks ≈ 1.253 µs; T1H = 425 ticks = 2.500 µs.
- Gói DShot một chiều dài 16 bit, telemetry request = 0; CRC4 không đảo.
- Một DMA1 Channel 1, request `TIM3_UP`, ghi vào `TIM3_DMAR`. `TIM3_DCR` đặt base từ CCR1 và burst 4 transfer để cập nhật CCR1..CCR4 cùng một lần cập nhật symbol. Bộ đệm có 17 hàng x 4 halfword: 16 symbol dữ liệu và một hàng mức thấp cuối frame.
- DShot được cập nhật theo vòng điều khiển 1 ms. DMA transfer dự kiến kết thúc sau khoảng 17 symbol, ngắn hơn nhiều chu kỳ điều khiển kế tiếp.

Cấu trúc này dùng tính năng timer DMA-burst được mô tả trong ST RM0440 cho nhóm TIM2/TIM3/TIM4/TIM5: DMA truy cập `TIMx_DMAR`, còn `DBA/DBL` xác định dải thanh ghi và độ dài burst. Cần xác minh dạng sóng thật trên bo mạch bằng oscilloscope/logic analyzer; kiểm thử mã không thay thế được phép đo phần cứng.

## 4. SBUS, ga, ARM và KILL

Trong `Core/Src/uav_test.c`, các kênh là index bắt đầu từ 0:

- CH_ROLL = 0; CH_PITCH = 1; CH_THR = 2; CH_YAW = 3; CH_ARM = 4; CH_KILL = 5.
- Mặc định hiện đặt `SBUS_THR_LOW_RAW = 1115` và `SBUS_THR_HIGH_RAW = 1640`, lấy từ các giá trị log đã được trao đổi trước đó. Đây **chưa phải hiệu chuẩn được xác nhận**. Phải đọc giá trị ga thấp nhất và cao nhất thật trong màn hình monitor của transmitter/receiver rồi sửa hai hằng số.
- Mặc định ARM active-high, ngưỡng 1320; KILL active-high, ngưỡng 1500. Có thể đổi `ARM_SWITCH_ACTIVE_HIGH` hoặc `KILL_SWITCH_ACTIVE_HIGH` thành 0 nếu switch thực tế active-low; vẫn phải đo hai trạng thái thật trước khi cắm cánh.
- Firmware yêu cầu SBUS hợp lệ, không failsafe/lost-frame, link ổn định, ga thấp ổn định, đã qua thời gian chờ ESC, ARM phải được đưa về OFF rồi bật ON, hiệu chuẩn ga hợp lệ và IMU sẵn sàng khi PID được bật.
- Khi mất SBUS, KILL được kích hoạt, IMU không khởi tạo/hiệu chuẩn được hoặc mẫu IMU cũ hơn `IMU_MAX_AGE_MS`, firmware disarm và gửi lệnh DShot stop.

### Vì sao DShot có thể cảm giác yếu hơn PWM?

Tốc độ truyền DShot150/300/600 **không quyết định lực motor**. Trong code hiện tại, ga được ánh xạ từ `SBUS_THR_LOW_RAW..SBUS_THR_HIGH_RAW` sang DShot 48..2047. Nếu giá trị `SBUS_THR_HIGH_RAW` lớn hơn giá trị ga tối đa thực tế, lệnh sẽ không chạm 2047 khi hết hành trình cần ga. Ngược lại, đặt thấp hơn endpoint thật có thể làm chạm max sớm.

Bản PWM cũ dùng dải SBUS khác (`SBUS_MIN=8`, `SBUS_MAX=1606` trong file cũ); bản DShot mới đang dùng 1115..1640. Vì hai dải này không giống nhau nên không thể so sánh cùng vị trí tay ga nếu chưa hiệu chuẩn lại endpoint. Đừng tăng gain hoặc bỏ giới hạn ga để bù cảm giác yếu trước khi xác nhận raw SBUS và lệnh DShot trong log.

`SLEW_UP_RATIO_PER_S=5.0` và chu kỳ điều khiển 1 ms giới hạn ramp tăng toàn dải khoảng 200 ms. Tốc độ ramp này tương đương 5.0/s trong code PWM cũ; bản hiện tại không chủ động đặt ramp chậm hơn PWM.

## 5. MPU6500 và PID

- MPU6500 được xác nhận qua WHO_AM_I = `0x70`, cấu hình gyro ±500 dps và accel ±4 g. Gyro bias được lấy trung bình 1500 mẫu sau 2 giây đứng yên; cần để bo mạch nằm yên trong giai đoạn hiệu chuẩn.
- Complementary filter cập nhật roll/pitch; PID hiện chỉ có roll và pitch, setpoint lấy từ SBUS roll/pitch với giới hạn ±25°.
- `ENABLE_ATTITUDE_PID=1` trong gói này, theo yêu cầu điều khiển cân bằng. Các hệ số `Kp/Ki/Kd` là giá trị khởi đầu chưa được tune trên UAV thật.
- Bộ trộn giả định Quad-X: M1 trước-trái, M2 sau-trái, M3 sau-phải, M4 trước-phải. Thứ tự và dấu trộn **phụ thuộc hướng lắp IMU, bố trí và chiều quay motor thực tế**. Mixer giới hạn bằng cách scale đồng thời hiệu chỉnh bốn motor để không cắt lệch từng motor khi hết headroom.
- Không có PID yaw và DPS310 hiện chỉ dùng để đọc/ghi log áp suất/độ cao tương đối; chưa có altitude-hold.

Không lắp cánh quạt cho đến khi xác minh hướng thay đổi góc IMU, dấu PID roll/pitch, thứ tự motor và chiều quay trên bàn thử. Không thể xác định đúng dấu mixer chỉ từ source vì chưa có ảnh hướng lắp IMU/frame và wiring motor.

## 6. DPS310 / I2C1

- `HAL_I2C_MODULE_ENABLED` đã được bật trong `stm32g4xx_hal_conf.h`.
- `main.c` khởi tạo I2C1; MSP chọn I2C1 kernel clock = HSI 16 MHz; timing register `0x00303D5B` theo cấu hình Standard-mode 100 kHz.
- I2C1 dùng PB8/PB9 và IRQ `I2C1_EV_IRQn`, `I2C1_ER_IRQn`. Driver dùng `HAL_I2C_Mem_Read_IT` khi chạy để không chặn vòng điều khiển DShot.
- DPS310 thử địa chỉ 7-bit 0x77 và fallback 0x76. Áp suất 64 Hz, OSR 8; nhiệt độ 1 Hz, OSR 1. Baseline độ cao là trung bình 64 mẫu đầu; để bo mạch đứng yên trên mặt đất trong giai đoạn lấy baseline.
- Barometer là tùy chọn: nếu không có DPS310 thì log `BARO=0`, nhưng không khóa IMU/PID hay vòng DShot.

**Quan trọng khi build (lỗi hiện tại của project):** Log của bạn cho thấy compiler không tìm thấy `stm32g4xx_hal_i2c.h` trong `Drivers/STM32G4xx_HAL_Driver/Inc`. Đây là file thuộc thư viện HAL của ST, không phải file ứng dụng; gói overlay này không tự tạo hoặc thay thế thư viện vendor.

Từ đúng phiên bản STM32CubeG4/HAL đang dùng trong project, cần có đủ 4 file sau:

- `Drivers/STM32G4xx_HAL_Driver/Inc/stm32g4xx_hal_i2c.h`
- `Drivers/STM32G4xx_HAL_Driver/Inc/stm32g4xx_hal_i2c_ex.h`
- `Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c.c`
- `Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c_ex.c`

Đặt hai header vào `Inc`, hai source vào `Src` và bảo đảm cả hai file `.c` được đưa vào build (không bị Exclude from Build). `stm32g4xx_hal_conf.h` trong bản v2 đã include cả `i2c.h` và `i2c_ex.h`, vì `main.c` gọi `HAL_I2CEx_ConfigAnalogFilter()` và `HAL_I2CEx_ConfigDigitalFilter()`.

Có thể tìm bộ file phù hợp trong kho STM32Cube đã cài bằng lệnh Linux:

```bash
find ~/STM32Cube/Repository -path '*/Drivers/STM32G4xx_HAL_Driver/Inc/stm32g4xx_hal_i2c.h' -print 2>/dev/null
```

Chỉ copy từ cùng phiên bản HAL với các file driver đang có trong project; không tải riêng một header từ phiên bản khác. Sau khi copy, kiểm tra Build log có biên dịch `stm32g4xx_hal_i2c.c` và `stm32g4xx_hal_i2c_ex.c`, rồi Clean Project và Build lại. Đường dẫn include hiện tại của bạn (`-I../Drivers/STM32G4xx_HAL_Driver/Inc`) đã đúng; lỗi là file chưa tồn tại ở thư mục đó.

Khi mở/regenerate `.ioc`, kiểm tra I2C1 kernel clock vẫn là HSI 16 MHz để phù hợp với `TIMINGR=0x00303D5B` trong `main.c` và MSP.

## 7. Build trong STM32CubeIDE

1. Sao lưu project hiện tại.
2. Chép đè các file trong `Core/Inc` và `Core/Src` của gói này vào đúng hai thư mục tương ứng trong project `UAV_`.
3. Dùng `UAV_.ioc` đi kèm để đối chiếu hoặc mở bằng CubeMX. Nếu regenerate code, kiểm tra các phần user code, I2C1 PB8/PB9, NVIC I2C1_EV/ER, TIM3_UP DMA và pin TIM3_CH1..4.
4. Trong HAL driver, bật/đưa vào build các module GPIO, RCC, PWR, CORTEX, DMA, TIM, UART, SPI và I2C; xác nhận `stm32g4xx_hal_i2c.c` + `stm32g4xx_hal_i2c_ex.c` được compile.
5. Không thêm file test có `main()` vào `Core/Src` hoặc `Core/tests` của firmware.
6. Chọn Project → Clean, sau đó Build Project. Kiểm tra chỉ còn một định nghĩa `main()` (ở `Core/Src/main.c`).

## 8. Kiểm thử và giới hạn xác nhận

Đã thực hiện trong môi trường làm việc:

- Kiểm tra cú pháp C của `uav_test.c` và `dps310.c` bằng GCC host với HAL stub tối thiểu, `-Wall -Wextra -Werror` (đây không phải ARM target build).
- Kiểm tra các vector gói DShot: 0 → `0x0000`; 48 → `0x0606`; 1046 → `0x82C6`; 2047 → `0xFFEE`.
- Kiểm tra số học mixer ở mức ga thấp/trung bình/cao: các lệnh nằm trong dải 48..2047 và trung bình bốn motor được giữ bằng ga cơ sở khi scale headroom.
- Kiểm tra tĩnh sự nhất quán của các cấu hình I2C1, PB8/PB9, IRQ, `HAL_I2C_MODULE_ENABLED`, TIM3_UP DMA và chỉ một `main()` trong nguồn firmware.

Chưa thể xác nhận build ARM GCC/CubeIDE trên máy người dùng vì không có toàn bộ HAL/BSP/CMSIS/startup/linker/project build trong bộ file được gửi. Chưa đo waveform trên bo STM32G474RE, chưa kiểm tra ESC/motor thật, chưa xác minh vị trí/chiều motor hoặc tune PID trên UAV. Vì vậy gói này là **bản nguồn đã rà soát và kiểm tra tĩnh**, không phải chứng nhận sẵn sàng bay.

## 9. Trình tự thử an toàn tối thiểu

1. Tháo toàn bộ cánh quạt; cấp nguồn logic và xác nhận log READY.
2. Xác minh SBUS raw min/max và switch ARM/KILL, điều chỉnh các hằng số rồi build lại.
3. Xác minh `IMU=1 CAL=1 imuFresh=1`, nghiêng bo mạch và quan sát `ANGLE R/P`, `PIDr/PIDp`.
4. Kiểm tra `DSHOT` khác 0 khi ARM + throttle hợp lệ; `txDone` tăng đều và `txErr` không tăng.
5. Đo PA6/PA4/PB0/PB1 bằng logic analyzer/oscilloscope: khoảng 3.33 µs mỗi bit, T0H khoảng 1.25 µs, T1H khoảng 2.50 µs.
6. Thử từng motor không cánh, xác minh số thứ tự và chiều quay. Chỉ nghiên cứu thử có cánh sau khi toàn bộ failsafe, mixer và PID đã được kiểm tra trên khung đúng cách.

Không giữ UAV bằng tay khi motor có thể quay và không thử PID với cánh quạt lắp nếu chưa xác minh chiều bù.
