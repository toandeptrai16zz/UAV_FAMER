# Kết quả kiểm tra tĩnh — DShot300 fixed v3

Ngày kiểm tra: 2026-10-29

## Đã chạy

1. `sh tests/run_tests.sh`: PASS cho các vector đóng gói DShot:
   - `0 -> 0x0000`
   - `48 -> 0x0606`
   - `1046 -> 0x82C6`
   - `2047 -> 0xFFEE`
2. Kiểm tra cú pháp `uav_test.c` bằng GCC host với các stub HAL tối thiểu, bật `-Wall -Wextra -Wshadow -Wconversion -Werror -fsyntax-only`, cấu hình PID mặc định OFF: PASS.
3. Kiểm tra cú pháp tương tự với `ENABLE_ATTITUDE_PID=1`: PASS.
4. Kiểm tra gói và cấu hình ở mức tĩnh: TIM3 PSC=0/ARR=566; DMA1 Channel 1 dùng request TIM3_UP; DMA burst 4 transfer từ CCR1 đến CCR4; các chân AF2 PA6/PA4/PB0/PB1 có trong cấu hình.

## Chưa xác minh

- Đây không phải build thật với ARM GCC và thư viện HAL đầy đủ. Chưa có đủ startup, linker script, toàn bộ driver HAL và metadata project CubeIDE trong gói bàn giao.
- Chưa nạp lên NUCLEO-G474RE, chưa đo waveform bằng oscilloscope/logic analyzer, và chưa thử ESC/motor.
- Chưa hiệu chỉnh giá trị SBUS low/high, trạng thái ARM OFF/ON, cực tính KILL, bố trí motor/chiều quay, hướng trục IMU hoặc PID gains trên phần cứng.

## Điều kiện an toàn

PID roll/pitch được tắt mặc định (`ENABLE_ATTITUDE_PID=0`) để chỉ phục vụ thử nghiệm bench. Không gắn cánh quạt trong khi xác minh DShot. Không dùng bản này để bay cho tới khi hoàn tất build mục tiêu, đo tín hiệu và xác nhận controller/mixer trên phần cứng. Code hiện chưa có vòng điều khiển yaw và không phải flight controller đã được chứng nhận.
