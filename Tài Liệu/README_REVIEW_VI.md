# STM32G4 UAV – mã nguồn đã rà soát và chỉnh sửa

## Thay đổi đã thực hiện

1. **SBUS / USART1**
   - Đổi cấu hình từ 115200-8N1 sang SBUS chuẩn 100000 baud, 8E2.
   - Dùng `UART_WORDLENGTH_9B` kết hợp parity EVEN để USART STM32 nhận đủ 8 bit dữ liệu + bit parity.
   - Giữ đảo mức RX bằng phần cứng (`UART_ADVFEATURE_RXINV_ENABLE`) và chuyển USART1 sang RX-only.
   - Sửa kiểm tra footer SBUS2: kiểm tra nibble thấp `0x04` hoặc `0x08`, bỏ qua nibble cao chứa slot telemetry.

2. **DShot300 / TIM3**
   - Đồng bộ cấu hình CubeMX init với cấu hình lúc chạy: prescaler 0, ARR 566 (567 tick).
   - Ở timer clock 170 MHz, tần số bit là xấp xỉ 299.8 kbit/s.
   - Tăng GPIO output speed của bốn chân TIM3 lên VERY_HIGH để có cạnh tín hiệu rõ hơn.

3. **Ngắt và thời gian thực**
   - Hạ ưu tiên ngắt LPUART1 (log) từ 0 xuống 6, tránh để ngắt log tranh ưu tiên với DMA/control.
   - Giảm timeout giao tiếp SPI MPU6500 từ 5 ms xuống 2 ms; vẫn dư cho burst 14 byte ở tốc độ SPI đang cấu hình.

4. **DPS310 và khởi tạo**
   - Sửa chú thích Product ID/Revision ID theo bố trí thanh ghi 0x0D.
   - Thêm memory barrier trước khi callback I2C công bố trạng thái DONE và trước khi main loop xử lý dữ liệu RX.
   - Bỏ khởi tạo LED và nút nhấn BSP lặp lần thứ hai trong `main()`.

## Cấu hình phần cứng mà mã hiện giả định

- MCU STM32G4 chạy SYSCLK/HCLK/PCLK1/PCLK2 = 170 MHz.
- DShot300 qua TIM3: CH1=PA6, CH2=PA4, CH3=PB0, CH4=PB1; DMA1 Channel 1 / TIM3_UP.
- SBUS receiver nối USART1_RX (PB7), tín hiệu SBUS đảo mức; UART phần cứng được bật RX inversion.
- MPU6500 dùng SPI2 (PB13/PB14/PB15), CS=PB12.
- DPS310 dùng I2C1 (PB8/PB9), địa chỉ 7-bit 0x76 hoặc 0x77.

Nếu receiver xuất SBUS **không đảo mức**, cần tắt `UART_ADVFEATURE_RXINV_ENABLE`. Không thay đổi ngưỡng SBUS throttle, switch, hệ số PID, motor order hay trộn Quad-X vì những giá trị này phụ thuộc receiver, khung UAV và cách lắp IMU thực tế.

## Hành vi an toàn hiện đang giữ nguyên

- Chỉ cho ARM sau khi ESC init 2.5 s, link ổn định 1 s, ARM switch đã ở OFF rồi chuyển ON, và throttle thấp ổn định 500 ms.
- Mất link, cờ frame-lost hoặc failsafe đều làm disarm ngay. Lưu ý: hiện tại **một frame-lost cũng disarm**, không chỉ cờ failsafe/timeout; đây là lựa chọn fail-safe hiện có, cần cân nhắc kỹ khi thử thực tế vì có thể làm motor stop khi chỉ mất một frame.
- PID roll/pitch đang bật (`ENABLE_ATTITUDE_PID=1`); DPS310 chỉ phục vụ log/độ cao tương đối, không tham gia vòng PID.

## Kiểm tra đã chạy

- Host test biên cho `dshot_make_packet()` với lệnh 0, 1, 47, 48, 100, 1000, 2047 và 4095: PASS.
- Kiểm tra tĩnh các cấu hình SBUS 100000 baud, **8E2**, RX inversion, TIM3 prescaler/ARR và ưu tiên LPUART1: PASS.

## Chưa thể xác nhận

Gói đầu vào không chứa project STM32CubeIDE, đúng phiên bản STM32CubeG4 HAL, startup assembly, linker script hoặc file `.ioc`. Vì vậy chưa thể chạy build firmware hoàn chỉnh, xác nhận mapping DMA trên đúng mã MCU, hay xác minh pinout với đúng board. Những việc đó cần được kiểm tra trong project/board thực tế.

**An toàn thử nghiệm:** tháo toàn bộ cánh quạt khi thử SBUS/DShot/PID. Trước khi lắp cánh, phải xác nhận chiều trục IMU, dấu roll/pitch, thứ tự motor, chiều quay motor, công tắc ARM/KILL và failsafe của receiver. Không coi bộ mã này là flight-controller đã được tune hoặc chứng nhận bay an toàn.
