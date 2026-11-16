# UAV R5.7 — BẢN BAY THỬ TRONG KHU CÁCH LY, KHÔNG TỰ NGẮT 5,5 GIÂY

**Nền source:** R5.6.1 Manual Frame Level Diagnostic. **Không phải** bản R5.6.2 props-off scanner; việc giải nén đè Core/Src/uav_test.c khôi phục ARM bay thường.

## Đã bỏ giới hạn thử nghiệm

- Tắt UAV_CAGE_TEST_ARM_ENABLE: không còn 5,5 giây TIMEOUT; không còn ngắt tự động do ngưỡng cage Yaw 110 độ/s trong 80 ms, rate 240 độ/s, góc Roll/Pitch 22/35 độ.
- Tắt UAV_YAW_DIAGNOSTIC_ARM_LOCK, giữ UAV_EXPERIMENTAL_ARM_ENABLE=1 và UAV_BENCH_ARM_LOCK=0: điều khiển ARM thường bằng CH5, không quét từng motor.
- UAV_EXPERIMENTAL_MAX_DSHOT dùng DSHOT_THROTTLE_MAX=2047 thay cho giới hạn cage 1550, không có giới hạn ga tăng theo thời gian trong firmware. Mixer vẫn khử bão hòa hợp lệ.
- PID và mốc IMU từ cấu hình Flash vẫn được đọc như cũ; không thay đổi Yaw Kp 0.7 được lưu trên máy.
- PID200 và IMU1000 vẫn ghi khi motor hoạt động. Khi DISARM/KILL, chốt 512 mẫu PID200 gần nhất và tối đa 1024 mẫu IMU1000 gần nhất, sau đó chỉ gửi khi motor=0 và KILL ON.

## Những bảo vệ CỐ Ý GIỮ (không phải ngắt 5,5 giây)

- CH5 ARM yêu cầu sườn OFF→ON, ga thấp và điều kiện sensor/receiver hợp lệ trước khi ARM.
- CH6 KILL ON sẽ ngắt ngay 4 output; mất/ lỗi SBUS, mẫu RF giả, IMU lỗi/mất dữ liệu, PID số không hợp lệ, DShot DMA lỗi đều ngắt output. Không bypass hard-failsafe.
- Giữ giới hạn vật lý gói DShot 48..2047, giới hạn mixer/PID nội bộ và kiểm soát cấp quyền sửa PID/Flash khi DISARM.
- Không đảm bảo an toàn bay: sai thứ tự motor/cánh/mixer có thể khiến xoắn rất mạnh, và tải động có thể gây sụt nguồn/reset.

## Cài (Ubuntu, STM32CubeIDE)

```bash
cd ~/Desktop/UAV_DATN
cp Core/Src/uav_test.c Core/Src/uav_test.c.before_R57
unzip -o ~/Downloads/UAV_R5_7_NET_FLIGHT_UNTIMED_PATCH.zip -d .
```

STM32CubeIDE: Refresh → Clean → Build → Flash **STM32**. Không cần nạp ESP32-CAM hay Dashboard nếu chúng đang dùng R5.6.1. Các file PID/IMU vẫn giữ nguyên giao thức.

Trước khi cấp điện ESC: ngắt nguồn động lực, xác minh thứ tự M1/M2/M3/M4 vật lý và chiều mô-men theo mixer, chiều quay thực tế, đúng cánh CW/CCW và chiều đặt cánh; xác nhận KILL và ngắt nguồn ESC độc lập hoạt động. Dùng khu cách ly chắn cánh, đứng ngoài vùng cánh và lưới.

Sau khi cấp điện: kiểm tra ARM OFF + KILL ON, ga thấp, nhận đủ SBUS, IMU hợp lệ; chỉ ARM khi người điều khiển và vùng thử đã sẵn sàng. Bản này không tự ngắt sau 5,5 giây; **phải chủ động KILL/ARM OFF** khi có dấu hiệu xoắn hoặc mất kiểm soát. Sau DISARM hãy bật KILL ON và giữ điện STM32/ESP32, dashboard ~25–35 giây để tải PID200 / IMU1000; không reset STM32 trước khi tải xong.

## Kiểm thử

Host GCC: `tests/run_tests.sh` PASS; test riêng `tests/test_r57_continuous.c` PASS chạy >10 giây không timeout, không cage yaw trip, KILL cắt 4 output, chốt PID200. Đây là mô phỏng HAL không phải nạp/chạy trên STM32 thật. Chưa có build ARM vì môi trường này không cài arm-none-eabi-gcc.
