# UAV V3.2.9.1 — cập nhật offset theo 2 tư thế A/B

Nguồn: V3.2.9; chỉ thay đổi hai hằng offset MPU6500, chú thích và banner phiên bản.
Giữ nguyên DShot, SBUS, PID, mixer, chân phần cứng và các khóa an toàn.

## Dữ liệu đã đo
- A (0°): Roll trung bình -227.74 centidegree, Pitch -117.96.
- B (xoay 180°): Roll trung bình -199.13 centidegree, Pitch -94.96.
- Trung bình A/B: Roll -213.43 (~-2.13°); Pitch -106.46 (~-1.06°).
- Nửa sai khác A/B: Roll 14.3 centidegree, Pitch 11.5 centidegree (~0.1-0.14°).

## Bản vá
- `IMU_ROLL_MOUNT_TRIM_DEG=-2.13f`
- `IMU_PITCH_MOUNT_TRIM_DEG=-1.06f`
- Banner `V3291 AB_TRIM_VERIFY`

## Thử
1. Sao lưu project V3.2.9 và thay `Core/Src/uav_test.c`. Không sửa IOC.
2. Build + Flash. Tháo cánh và ngắt nguồn ESC, chỉ cấp nguồn STM32.
3. Đặt UAV trên cùng một vị trí nền, lần lượt A và xoay 180° ở B, để yên mỗi tư thế 5 giây.
4. Kiểm tra log R/P sau bù: A khoảng -0.14°/-0.12°, B khoảng +0.14°/+0.11°; N=1, LV=1, O=0000, LK=1.

Các số liệu này chỉ kiểm chứng hiệu chuẩn tĩnh của IMU theo khung. Không xác nhận PID, mixer có tải hay điều kiện bay tự do. Nếu MPU hoặc khung dịch chuyển, cần đo lại.
