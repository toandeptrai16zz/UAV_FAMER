# UAV V3.2.4 — CHẨN ĐOÁN M1 KHÔNG CÁNH, ARM BAY VẪN KHÓA

Đây là PATCH trên source V3.2.3, **không phải firmware đã đủ điều kiện bay**.

## Vì sao sửa
- Log E=3 chỉ ra chu trình 350 ms đã kết thúc, **không chứng minh ESC đã nhận tín hiệu hoặc M1 đã quay**.
- Code DShot TIM3 DMA so với firmware gốc không có thay đổi đáng kể; pulse thử cũ DShot=80, 350ms có thể không đủ để ESC/motor khởi động.
- Chưa có bằng chứng cần sửa DShot300, sơ đồ chân hay chuyển sang PWM.

## Sửa ở V3.2.4
- Chỉ thử **một motor** do `UAV_BENCH_TEST_MOTOR_INDEX` chọn, mặc định M1.
- Lệnh test 2 giai đoạn thấp: DShot 120 trong 600ms, rồi 180 trong 900ms, tổng **1,5 giây**, chỉ khi không có cánh. Đây là phép thử có giới hạn, không phải mức ga bay.
- Khi bất kỳ điều kiện an toàn mất (KILL ON, ARM OFF, mất SBUS, ga cao, IMU lỗi, DMA lỗi), dừng thử và báo E=4.
- Chỉ báo E=3 khi hết 1,5 giây **và** đủ ít nhất 200 lần callback DMA hoàn tất, không có lỗi DMA. Không chứng minh vật lý motor quay.
- Log bổ sung `M` (chỉ số motor thử), `CMD` (lệnh DShot đang yêu cầu), `FR` (số callback DMA trong phiên thử). `TX` vẫn là log UART, **không phải** DShot.
- Giữ `.ioc`, TIM3 DMA, SBUS, PID/mixer, `LOCK=1`, Alt Hold khóa. Không mở ARM bay.

## Thay vào project STM32CubeIDE
1. Giải nén ZIP và ghi đè `Core/Src/uav_test.c` trên project V3.2.3.
2. Project → Clean → Build → Flash.
3. Khởi động lại và xác nhận log chào bắt đầu bằng `V324 STAGED_MOTOR_TEST`.

## Test an toàn (không lắp cánh)
- Bắt buộc tháo **cả bốn** cánh quạt; đặt khung cố định vững, để tay cách xa motor, có phương án ngắt pin ESC ngay.
- Bật receiver, kiểm tra L=1. Để ARM OFF A=1000, KILL ON K=1800; hiệu chuẩn ga 2 đầu như bản trước, đưa ga về thấp nhất, chờ E=1 ít nhất 1 giây.
- Nhả KILL về 1000 khi ARM vẫn 1000, sau đó gạt ARM lên 1800, **không tăng ga**.
- Chờ xem `E=2 M=1 CMD=120` rồi `E=2 M=1 CMD=180`; khi thành công `E=3 M=1 CMD=0 FR>=200`. Nếu `E=4`, lỗi/mất điều kiện hoặc số DMA không đủ. Lập tức KILL ON để dừng khi bất thường.
- Sau test, KILL ON, ARM OFF và ngắt nguồn ESC. Quan sát riêng liệu M1 quay hay không. **E=3 không xác thực motor quay**.
- Nếu E=3 nhưng motor vẫn không quay, không tăng DShot nữa; kiểm tra DMA/frame thực bằng logic analyzer/máy hiện sóng, GND chung, vị trí M1 TIM3_CH1, trạng thái khởi động của ESC và so sánh file `.elf` firmware test DShot cũ.

### Giới hạn
- Firmware chưa được biên dịch ARM GCC hoặc chạy trên STM32 thực tế bởi người tạo patch.
- V3.2.4 chỉ là phép thử không cánh; không xác nhận motor/ESC đã hoạt động hoặc UAV có thể bay.
