# UAV V3.2.9 — Hiệu chỉnh tạm góc MPU và tâm SBUS (thụ động, KHÔNG bay)

Dựa trên V3.2.8 đã kiểm tra phản hồi dấu Roll/Pitch/Yaw bằng log và xác nhận cả 4 ESC/motor quay được với DShot300 ở V3.2.6. Đây là một **bản xác minh hiệu chuẩn**, KHÔNG phải firmware bay tự do.

## 1. Các file thay đổi

- `Core/Src/uav_test.c`: giữ DShot300 TIM3 DMA, Quad-X mixer V3.2.8 và dấu Yaw đã thử. Đặt trim lắp MPU **tạm tính** Roll **−2,35°**, Pitch **−1,16°**. Ghi góc gốc `S` và góc đã bù `R/P`, trạng thái `N` (RC giữa), `LV` (gần ngang và đứng yên theo cảm biến). Tăng buffer log từ 192 lên 288 byte để tránh cắt log dài.
- `Core/Inc/rc_input.h`: đặt tâm Roll **952**, Pitch **1062**, Yaw **1099** theo các log gần nhất. Giữ dải ga 40–1640, vùng chết 20.
- KHÔNG đổi `flight_pid.c`, `flight_pid.h`, `.ioc`, TIM3/DMA, chân M1–M4 hoặc phần MT-06/DPS310.
- KHÔNG chép file kiểm thử có `main()` vào project STM32CubeIDE.

**Chú ý:** Các offset trên chỉ là ước tính từ những log trước. Nếu khi ghi log bạn không đặt toàn bộ KHUNG UAV cân bằng bằng thước thủy, thì offset có thể sai. Nếu tay điều khiển bị trim/thay đổi, các tâm RC cũng phải đo lại. Không lấy `LV=1` làm chứng nhận phương tiện thực sự nằm ngang.

## 2. Nạp

Trên project đang chạy V3.2.8, sao lưu rồi chép đè đúng 2 file nêu trên; **giữ nguyên `Core/Src/rc_input.c`**. Trong CubeIDE: `Project → Clean`, `Build Project`, sau đó nạp khi **đã tháo 4 cánh quạt, ngắt nguồn ESC**, chỉ cấp nguồn vi điều khiển. Nếu Build có lỗi, ngừng và gửi lỗi để sửa.

Kỳ vọng banner có `V329 PASSIVE_TRIM_VERIFY`.

## 3. Test thụ động

1. Tháo toàn bộ cánh, **ngắt pin cấp cho ESC**, cố định IMU. Bật tay điều khiển, đặt ARM OFF (`A=1000`), KILL ON (`K=1800`), thả Roll/Pitch/Yaw về giữa.
2. Đặt **khung UAV** nằm ngang bằng thước thủy (không cần chip MPU nằm ngang với mặt khung), giữ yên 5–8 giây để gyro hiệu chuẩn và góc ổn định.
3. Đọc log `V...` 5 lần/giây. Ví dụ:
   `V100 L=1 RC=952/1062/1099 N=1 S=-235/-116 R=0 P=0 LV=1 G=0/0/0 PID=0/0/0 MIX=0/0/0/0 O=0000 LK=1`
4. Nghiêng sang phải 15° rồi đặt ngang; sau đó nghiêng trái, chúi đầu, ngẩng đầu lần lượt. Theo quy ước đang kiểm chứng, Roll phải đổi theo hướng nghiêng phải/trái; Pitch đổi khi chúi/ngẩng; khi về ngang, R/P trở về gần 0. Chỉ phân tích **lệnh mixer tương đối**; `O` vẫn phải bằng `0000`.
5. Không di chuyển khung, thử lệch từng cần Roll/Pitch/Yaw. `N` chuyển về 0 khi lệch khỏi vùng chết. Khi buông cần, `N` phải về 1. Nếu khi buông cần `N=0`, tâm RC còn sai và cần đo lại.
6. Tắt tay điều khiển khi motor hoàn toàn ngắt nguồn: `L` phải về 0, `N=0`, `O=0000`. Đây chỉ kiểm thử phát hiện mất link thụ động, chưa phải kiểm thử failsafe dưới tải động cơ.

Ý nghĩa log: `S` là Roll/Pitch từ cảm biến trước hiệu chỉnh (*đơn vị 0,01°*); `R`/`P` là sau hiệu chỉnh (*0,01°*); `G` là gyro XYZ (*0,1°/s*); `PID` là lệnh ba trục; `MIX` là hiệu chỉnh M1–M4 tương đối; `O=0000` là 4 lệnh DShot = 0; `LK=1` là khóa ARM.

`LV=1` nghĩa **phần mềm nhìn thấy góc đã bù trong ±2° và gyro trong ±3°/s**, KHÔNG chứng minh khung đặt ngang ngoài thực tế. `N=1` nghĩa ba kênh RC đang gần tâm và link hợp lệ.

## 4. Điều kiện chưa đạt để bay

- Các trị số trim và tâm cần được xác nhận khi khung cân bằng bằng thước thủy, không chỉ lấy từ một ảnh log.
- PID trong `flight_pid.c` vẫn là giá trị thử nghiệm, chưa nhận dạng quán tính/động lực học của UAV 1,25 kg; chưa đánh giá dao động, bão hòa/thời gian vòng lặp khi có tải.
- Chưa kiểm thử fail-safe mất SBUS/KILL khi motor có tải có kiểm soát, rung IMU do motor, nguồn pin 3S dưới tải, nhiệt ESC/motor với cánh 8045, hướng mô-men thực tế.
- Không bật Alt Hold: chưa đo ga treo, độ trễ MT-06, lọc đo và kịch bản mất cảm biến.

Biến khóa trong bản này bắt buộc: `UAV_BENCH_ARM_LOCK=1`, `UAV_PROPS_OFF_ESC_TEST=0`, `UAV_ENABLE_ALT_HOLD=0`. Có thêm kiểm tra `#error` nếu ai vô tình đổi chế độ khóa hoặc bật ESC test trong bản thụ động.

## 5. Những gì đã kiểm tra khi đóng gói

- GNU C syntax check `uav_test.c`, `flight_pid.c`, `rc_input.c` với header STM32G4 trên máy chủ: đạt. Có thể còn warning do compiler 64-bit host khác ARM.
- Bài test native kiểm tra tâm RC, phép trừ offset và chiều mixer Yaw: đạt.
- **Chưa biên dịch đầy đủ bằng arm-none-eabi-gcc/STM32CubeIDE trong môi trường này và chưa chạy trên phần cứng.**

Khi có log `V329`, gửi 5 dòng khi khung thực sự ngang và 2–3 dòng khi nghiêng phải/trái. Từ đó mới xác định được trim có giữ hay phải đo lại.
