# CubeMX settings – MicoAir MT-06 on USART3

1. Open the actual project `.ioc` (it was not part of the uploaded files in this turn).
2. Connectivity -> USART3 -> Asynchronous.
3. Set PC10 = USART3_TX (AF7), PC11 = USART3_RX (AF7).
4. Parameter settings: 115200 baud, 8 data bits, no parity, 1 stop bit, no hardware flow control, oversampling 16.
5. NVIC: enable USART3 global interrupt. Use priority 3; keep the SBUS USART1 and DShot DMA priorities higher.
6. Generate code and verify `MX_USART3_UART_Init()` is called in `main()` before `uav_test_init()`.
7. The integrated source uses `huart3` for `MT06_Init()` and `USART3_IRQHandler()` for HAL UART handling.

Wiring:
- MT-06 TX -> MCU PC11 / USART3_RX / NUCLEO CN7 pin 2.
- MT-06 RX -> MCU PC10 / USART3_TX / NUCLEO CN7 pin 1 (optional for receive-only use).
- MT-06 5V -> regulated 5V; GND -> common GND.

Use UART mode, MAVLink protocol, 115200 baud, 8N1, downward orientation (MAVLink orientation 25). MT-06 UART signal logic is 3.3V LVTTL; do not drive STM32 RX with 5V logic.
