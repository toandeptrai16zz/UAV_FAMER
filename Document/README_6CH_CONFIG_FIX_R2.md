# R2 — Sửa CONFIG PID cho tay điều khiển 6 kênh MC6C

## Lỗi đã xác nhận

Firmware R1 dùng `CH_CONFIG=7` (CH8 trên SBUS) làm điều kiện bắt buộc cho `CONFIG_READY`. Tay MC6C chỉ có CH1..CH6. CH5 ARM, CH6 KILL. Do đó nút Đọc PID hoạt động nhưng nút Áp dụng RAM/Lưu Flash bị khóa.

## R2 thay đổi gì

- `Core/Src/uav_test.c`: `UAV_CONFIG_REQUIRE_CH8=0u`, bỏ phụ thuộc CH8 khi xét ARM/CONFIG. Không đổi các mapping CH1..CH6, motor, DShot, mixer, PID gain, cảm biến, giao thức Wi-Fi, failsafe.
- `Core/Inc/uav_config_gate.h`: thêm khóa viết PID dựa trên **trạng thái an toàn giữ liên tục 2 giây**: link RC ổn định, RF guard OK, ARM OFF, KILL ON, ga thấp, disarmed, motor không chạy, bench test không hoạt động. Bất kỳ điều kiện nào mất sẽ khóa ngay và reset bộ đếm.
- `tests/test_config_gate.c`, `tests/run_tests.sh`: kiểm thử mở/khóa, từng điều kiện nguy hiểm, reset, tràn `HAL_GetTick()`.

## Các bước thử chỉ khi tháo cánh

1. Sao lưu firmware cũ; build với STM32CubeIDE bằng project R2, nạp STM32 qua ST-LINK. R2 là thay đổi firmware STM32, **không sửa chỉ giao diện web là đủ**.
2. Tháo cánh quạt, cô lập nguồn ESC/motor, kiểm tra thiết bị ở ARM OFF, KILL ON, ga thấp, RC có tín hiệu ổn định. Giữ nguyên ít nhất 2 giây.
3. Giao diện `http://192.168.4.1/`: bấm Đọc PID; xác nhận `CONFIG READY`. Checkbox xác nhận tháo cánh là thêm một cổng bảo vệ của giao diện.
4. Chỉ kiểm tra khả năng APPLY RAM với gain không thay đổi khi chắc chắn an toàn. Không SAVE Flash cho đến khi đánh giá lại PID; không lắp cánh chạy thử.
5. Kiểm tra khi bỏ KILL ON/đưa ARM ON (vẫn không có động lực) trạng thái lập tức trở lại `CHỈ ĐỌC / KHÓA GHI`. Nếu không: dừng, không dùng bản này.

**Ghi chú:** Chương trình không thể phát hiện cánh đã tháo, người vận hành phải xác nhận thực tế. Chức năng cập nhật STM32 qua ROM vẫn khóa (`UAV_CONFIG_ENABLE_ROM_OTA=0`), dùng ST-LINK. Đây là bản sửa lỗi 6CH CONFIG, không phải bản tuning PID Yaw; chưa kiểm chứng trên phần cứng hoặc hover.
