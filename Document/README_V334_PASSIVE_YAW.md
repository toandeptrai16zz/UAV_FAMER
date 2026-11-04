# UAV V3.3.4 — PASSIVE YAW-SIGN CHECK / KHÔNG PHẢI FIRMWARE BAY

## Mục đích
Bốn phép nghiêng khung đã cho đúng dấu Roll/Pitch trên V3.3.3.
V3.3.4 chỉ bổ sung bộ đếm dấu phản hồi Yaw **ngay khi khung đang xoay**.
Không kiểm tra được chiều mô-men thực tế khi lắp cánh. Bộ điều khiển vẫn là PID thử trên bàn, CHƯA TUNE.

## Nạp
- Chép `Core/Src/uav_test.c` và `Core/Inc/mc7re_guard.h` vào project STM32CubeIDE đã chạy V3.3.3.
- Giữ nguyên `rc_input.h` đã hiệu chuẩn và `flight_pid.c` mixer M1/M2/M3/M4 đúng.
- Clean -> Build -> Flash. Không thêm file test có main(), không sửa `.ioc`.

## Chạy thử KHÔNG CÁNH, NGẮT nguồn ESC
1. Giữ STM32 + receiver có nguồn, tay bật, ARM OFF (`A=1000`), KILL ON (`K=1800`), ga thấp (`T≈40`), ba cần Roll/Pitch/Yaw ở giữa (`N=1`).
2. Giữ thân UAV gần nằm ngang và xoay **cả khung** quanh trục thẳng đứng **theo chiều kim đồng hồ khi nhìn từ trên xuống** trong khoảng 1–2 giây, đều khoảng 45–90 độ/giây. Không chạm cần Yaw.
3. Xoay **ngược chiều kim đồng hồ** theo cách tương tự.
4. Đọc `YQ=`:
   - 0 = chưa thu đủ mẫu đủ nhanh trong cả hai chiều
   - 1 = chiều CW có >=30 mẫu cho PID yaw ngược dấu tốc độ xoay
   - 2 = chiều CCW có >=30 mẫu cho PID yaw ngược dấu tốc độ xoay
   - 3 = **đã có cả hai chiều** (chỉ xác nhận dấu bộ điều khiển theo đo gyro)
   - 4 = phát hiện ít nhất một mẫu dấu PID Yaw không chống lại tốc độ đo được
5. Nếu `YQ=3`, gửi 4–5 dòng log và ghi rõ khi xoay CW/CCW. `O=0/0/0/0 B=0 LK=1` luôn bắt buộc.
6. Nếu `YQ` không tăng, thử xoay đủ nhanh hơn, khung gần ngang; không thay đổi công tắc ARM/KILL.

## Giới hạn bắt buộc
- `UAV_BENCH_ARM_LOCK=1u`, `UAV_PROPS_OFF_ESC_TEST=0u`, barie đầu ra zero cố định. Không thể bay bằng V3.3.4.
- MC7RE tiếp tục xuất SBUS khi mất RF, bảo vệ hiện nay suy luận từ preset 1000/1000/1000, không đủ làm failsafe độc lập.
- PID Roll/Pitch/Yaw đang dùng gain thử nghiệm, chưa có xác nhận lực đẩy, chiều mô-men yaw với cánh, cơ cấu ngắt động cơ khi tải, thời gian vòng điều khiển, nguồn và mức ga treo. Tuyệt đối không đổi khóa ARM thành 0 để bay thử.
- Để bay thật cần receiver failsafe xác thực + xác minh với người có kinh nghiệm UAV về lực đẩy, cánh/motor, thử có tải có rào chắn, và tuning PID.
