# Ghi chú hiệu chỉnh UAV_DATN2 (STM32G474RE)

## Kết luận kiểm tra source

1. **Lỗi giới hạn ga có thể làm disarm ở cuối hành trình.** `RC_THR_VALID_MAX` trong `rc_input.c` là 1680, trong khi dải stick chung của project là 0..1811. `motor_step()` dùng `RC_ThrottleRawValid(ch_thr)` trong `throttle_cal_ok`, rồi dùng `can_arm` như điều kiện duy trì ARM. Nếu CH3 thực tế vượt 1680, điều kiện này có thể mất và code phát lệnh STOP cho cả bốn motor. Bản patch chấp nhận đến 1811; giá trị thực tế vẫn phải được xác nhận bằng log `T=` trên receiver.

2. **Mixer cũ triệt tiêu điều chỉnh attitude khi gần full throttle.** Khi `base` tiến sát 2047, `high_headroom` tiến về 0; thuật toán cũ nhân toàn bộ PID correction với hệ số scale tiến về 0. Bản patch thêm `FlightPID_DesaturateMixer()` để giới hạn dải bốn motor bằng cách dịch collective, giữ chênh lệch motor trong phạm vi DShot hợp lệ. Khi chạm giới hạn vật lý, collective vẫn có thể phải giảm để giữ quyền điều khiển attitude.

3. **Sai khác chân ESC2 giữa project và sơ đồ đã chọn trước đó.** Source/IOC cũ cấu hình TIM3_CH2 ở PA4, trong khi sơ đồ ESC đã chọn là PA6/PA7/PB0/PB1. Bản patch chuyển CH2 sang PA7 đồng thời sửa `.ioc` và `HAL_TIM_MspPostInit()`. STM32G474RE hỗ trợ TIM3_CH2 trên cả PA4 và PA7; phải đảm bảo dây tín hiệu ESC2 thực tế được cắm PA7 sau khi nạp bản này.

4. **Một hoặc hai motor dừng riêng lẻ không được giải thích bởi nhánh disarm toàn cục.** Các nhánh `outputs_force_zero()` đặt cả bốn lệnh về 0. Mixer bình thường cũng giới hạn mỗi lệnh trong 48..2047. Nếu log `O=x/x/x/x` vẫn có lệnh khác 0 ở motor đó nhưng motor vật lý ngừng, cần kiểm tra đường tín hiệu, nguồn pin, ESC bảo vệ quá dòng/nhiệt, motor/đầu nối hoặc mất đồng bộ ESC; không tăng PID để chữa.

5. **PID chưa phải PID bay đã hiệu chuẩn.** `RATE_KI=0` và `RATE_KD=0`; hai vòng hiện tại về bản chất là attitude P + rate P. Mã nguồn gốc ghi rõ dấu IMU-to-body, chiều moment motor và gain chưa được xác minh trên UAV thật. Không thể xác định gain bay chỉ từ source mà thiếu log đáp ứng và thông số lực đẩy.

## Thay đổi trong patch

- `Core/Src/rc_input.c`: tăng giới hạn hợp lệ CH3 từ 1680 lên 1811, đồng bộ với `RC_STICK_MAX` hiện có. Throttle map vẫn bão hòa tại `RC_THR_MAX_RAW=1640`.
- `Core/Inc/flight_pid.h`, `Core/Src/flight_pid.c`: thêm hàm mixer desaturation và giữ log hệ số scale/collective offset.
- `Core/Src/uav_test.c`: dùng mixer desaturation; thêm `RAT`, `BASE`, `MS`, `MO` vào log; mặc định khóa ARM bay và chỉ bật quét ESC từng motor mức DShot 120/180.
- `UAV_.ioc`, `Core/Src/stm32g4xx_hal_msp.c`: TIM3_CH2 đổi PA4 → PA7.
- `cmake/stm32cubemx/CMakeLists.txt`: bổ sung các file ứng dụng (`uav_test.c`, `dps310.c`, `mt06.c`, `flight_pid.c`, `rc_input.c`) và HAL I2C/SPI bị thiếu trong danh sách build.

## Kiểm thử đã thực hiện

- Host GCC test cho mixer desaturation: PASS.
- Host GCC test cho dải raw throttle và ngưỡng ga thấp: PASS.
- `uav_test.c` đã qua kiểm tra cú pháp C với header HAL/CMSIS của project; trên GCC host còn warning do cross-compile architecture. Chưa có `arm-none-eabi-gcc`, vì vậy chưa xác nhận full firmware link/build.
- Chưa có thử nghiệm oscilloscope trên chân DShot, thử ESC/motor trên bench, thrust stand hoặc bay thật.

## Quy trình xác minh bắt buộc trước khi bật ARM bay

1. Tháo cả bốn cánh quạt. Nạp patch và kiểm tra log startup `BENCH_PROPS_OFF`.
2. Xác minh PA6/PA7/PB0/PB1 tương ứng CH1/CH2/CH3/CH4. Quét từng ESC; xác nhận từng motor được phát lệnh, đúng ESC, không motor nào mất tín hiệu.
3. Gửi log UART cho từng motor; `TX` chỉ xác nhận DMA callback, không chứng minh ESC nhận DShot hay motor quay.
4. Quan sát log tại các vị trí ga: `T` là raw SBUS CH3; `AR` là ARM; `SP` là spinning; `O` là bốn lệnh DShot thực tế; `DE` là số lỗi DMA; `RAT` là ga chuẩn hóa ×1000; `BASE` là collective trước mixer; `MS` là mixer scale ×1000; `MO` là collective shift.
5. Nếu `O` có lệnh >0 nhưng motor ngừng, dừng kiểm tra và xử lý đường tín hiệu/nguồn/ESC/motor. Không tiếp tục tăng ga hoặc tăng PID.
6. Chỉ tạo flight build riêng sau khi xác nhận chiều quay, thứ tự motor, dấu phản hồi IMU/mixer, đầu ra lực đẩy, dòng và nhiệt độ ở mức tải. Gói này không tự mở khóa bay.
