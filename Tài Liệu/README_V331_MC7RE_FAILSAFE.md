# UAV V3.3.1 — Phát hiện mẫu failsafe MC7RE (kiểm tra KHÔNG CÁNH)

## Vì sao cần bản này?

Log người dùng xác nhận receiver Microzone MC7RE vẫn phát frame SBUS (`RX=1`, `FS=0`, `F=10`) khi radio mất RF (LED nhấp nháy), nhưng Roll/Pitch/Yaw cùng về 1000, throttle về khoảng 40. `RX=1` chỉ là UART/SBUS còn dữ liệu, không chứng minh radio còn liên kết.

**Giới hạn quan trọng:** phát hiện mẫu 1000/1000/1000 là một suy luận từ kênh, KHÔNG PHẢI tín hiệu mất sóng RF độc lập. Nếu bộ thu hoặc người lái tạo đúng mẫu này trong lúc liên kết thật, có thể bị khóa nhầm. Nếu bộ thu thay mẫu failsafe, thuật toán có thể bỏ sót. Không sử dụng bản này để bay thật, không mở khóa ARM; cần nguồn báo RF link thực sự từ receiver khác/cấu hình receiver đã được kiểm chứng.

## Chép file

Chép hai file vào project đang chạy V3.3.0:

- `Core/Src/uav_test.c` (ghi đè)
- `Core/Inc/mc7re_guard.h` (mới)

Không đổi `.ioc`, TIM3, DMA, mixer, offset MPU6500, tâm RC hay giao thức DShot. Không có file test chứa `main()` trong ZIP, nên không gây lỗi `multiple definition of main` như bản trước. Clean → Build → Flash.

## Cơ chế

- Khi khởi động: `LAT=1`, `FS=5`, cho đến khi radio BẬT, ARM OFF, KILL ON, ga thấp và 3 cần trở về tâm đã hiệu chuẩn trong 2 giây.
- Khi nhận mẫu **Roll/Pitch/Yaw = 1000 ±2**, **Throttle ≤80**: `PRE=1`, `L=0` ngay. Nếu kéo dài ≥300ms: `LAT=1`, `FS=4`. Lệnh motor bị khóa; PID giữ mẫu bị xóa theo nhánh mất liên kết hiện hành.
- Nếu receiver ngừng phát frame hoặc đặt cờ SBUS `0x04`/`0x08`, khóa ngay: `FS=1/2/3`.
- Khi radio có tín hiệu lại: khóa **KHÔNG tự mất**. Muốn trở lại `LAT=0`, phải đặt ARM OFF (`A=1000`), KILL ON (`K=1800`), ga thấp, Roll/Pitch/Yaw ở tâm hiệu chuẩn (`952/1062/1099` ±25) liên tục ≥2s. Chỉ khi đó `L=1`.
- F=10 (`0x10`) KHÔNG phải cờ failsafe SBUS chuẩn; không lấy F=10 làm dấu hiệu duy nhất.

## Ký hiệu log

- `RX=1`: còn có frame SBUS trong 100ms.
- `L=1`: frame không báo lỗi, đã qua trạng thái guard, KHÔNG chứng minh RF thật.
- `PRE=1`: mẫu dự phòng phát hiện tức thời.
- `LAT=1`: khóa lỗi đang giữ; không tự ARM lại.
- `FS=0`: guard bình thường; `FS=1` timeout; `FS=2` frame_lost; `FS=3` failsafe flag; `FS=4` mẫu dự phòng; `FS=5` khóa từ khởi động.
- `O=0000`, `LK=1` xuyên suốt bản bench này.

## Test trên phần cứng

**Tháo cả 4 cánh; NGẮT pin nguồn ESC. Chỉ cấp nguồn STM32 + receiver.**

1. Bật radio, để ARM OFF (1000), KILL ON (1800), ga thấp và các cần giữa (952/1062/1099). Chờ ≥2 giây → `RX=1 L=1 FS=0 PRE=0 LAT=0`.
2. Tắt radio, không ngắt nguồn receiver. Khi LED nhấp nháy và kênh về 1000, log sẽ thấy `RX=1 PRE=1 L=0`, sau 300ms có `FS=4 LAT=1`.
3. Bật radio lại nhưng tạm để KILL OFF (1000): `LAT` vẫn **1**. Sau đó đặt ARM OFF, KILL ON, ga thấp, cần giữa liên tục ≥2 giây: `LAT=0 L=1 FS=0`.
4. Tùy chọn: ngắt nguồn riêng receiver khi ESC không cấp điện: `RX=0 L=0 FS=1 LAT=1`; bật receiver lại và làm bước khôi phục như trên.

Nếu bất kỳ bước nào không giống, ngắt nguồn ESC (vốn không được cấp điện), giữ khóa ARM và gửi vài dòng log có `RX L FS PRE LAT A K T RC`.

## Đã kiểm tra khi tạo patch

- Unit tests cho guard trên PC: PASS (khóa lúc boot, phát hiện preset sau 300ms, chặn tức thời, chặn tự hồi, giải khóa thủ công, timeout/cờ, nhiễu ngắn).
- Kiểm tra cú pháp `uav_test.c` với GCC + header STM32G4: PASS; các cảnh báo host 64-bit trong CMSIS/HAL không liên quan thay đổi.
- **Chưa Build ARM Cortex-M4 hoàn chỉnh; chưa kiểm tra nạp trên phần cứng.**

## LƯU Ý AN TOÀN

Đây là cơ chế khóa suy luận hỗ trợ debug, **không chứng minh đủ mức an toàn cho bay tự do**. Để bay, cần báo mất RF xác thực từ receiver, thử mất sóng khi motor quay trên bệ giữ chuyên dụng và kiểm chứng phản hồi điều khiển có tải. Không tự đổi `UAV_BENCH_ARM_LOCK` từ `1` thành `0`.
