# UAV_DATN2 — FLIGHT CANDIDATE (STM32G474RE)

## Đây là bản nào?

Bản này chuyển project ra khỏi chế độ `BENCH_PROPS_OFF`:

- `UAV_EXPERIMENTAL_ARM_ENABLE = 1`
- `UAV_BENCH_ARM_LOCK = 0`
- `UAV_PROPS_OFF_ESC_TEST = 0`
- `UAV_EXPERIMENTAL_MAX_DSHOT = 2047` (không đặt cap ga nhân tạo)
- `UAV_ENABLE_ALT_HOLD = 0` (MT-06 không tự điều khiển độ cao trong bản này)
- Dùng ga tay SBUS; attitude/rate controller đang hoạt động.

Đây là **firmware candidate cho giai đoạn kiểm tra có kiểm soát**, không phải tuyên bố máy đã đủ điều kiện bay. Không có cách suy ra PID bay an toàn chỉ từ mã nguồn hoặc từ việc ESC scanner đã quay đủ bốn motor.

## Trình tự gạt công tắc sau khi nạp firmware

`mc7re_guard_init()` cố ý đặt guard ở trạng thái khóa sau mỗi lần khởi động. Vì thế sau khi nạp/reset, làm theo thứ tự này (cánh vẫn tháo):

1. ARM OFF (`A` thấp), KILL ON (`K` cao), ga thấp, ba stick ở đúng tâm đã hiệu chuẩn; đặt khung nằm ngang và đứng yên.
2. Giữ trạng thái đó ít nhất 2 giây để MC7RE guard xác nhận receiver không còn phát preset failsafe. Chờ log `L=1`, `LAT=0`, `FS=0`.
3. Chuyển KILL OFF trong khi vẫn giữ ARM OFF. Chờ thêm ít nhất 1 giây để link ổn định và `RD=1`, `BLK=800` (bit ARM OFF).
4. Gạt ARM ON. Khi ga vẫn thấp, log mong đợi `LK=0 AR=1 SP=0`. Đây là Armed nhưng motor chưa được yêu cầu quay.
5. Chỉ sau khi hoàn tất kiểm tra cánh tháo mới thử tăng ga. Khi ga vượt ngưỡng start, `SP=1` và `O` phải có đủ bốn lệnh tương ứng.

Trong log hiện tại, `K=1000` là **KILL OFF** vì KILL được cấu hình active-high; `A=1800` là **ARM ON**. Do đó trạng thái log cũ không phải trạng thái recovery của MC7RE guard.

## Điều kiện ARM trong firmware

`AR` chỉ lên 1 khi tất cả điều kiện sau cùng đúng:

1. Đã qua `ESC_INIT_MS`.
2. SBUS mới, link ổn định ít nhất 1000 ms, không frame-lost/failsafe và MC7RE guard không latch.
3. KILL ở OFF (KILL switch được định nghĩa active-high).
4. IMU khởi tạo/calibration thành công và mẫu mới hơn 20 ms.
5. Ga nằm trong dải hợp lệ, đã giữ ga thấp ổn định ít nhất 500 ms.
6. Airframe gần ngang (Roll/Pitch <= 8 độ), tốc độ góc từng trục <= 15 dps.
7. Stick Roll/Pitch/Yaw ở vùng neutral.
8. Đã có 600 frame DShot DMA thành công sau fault gần nhất.
9. ARM phải được gạt OFF rồi ON. Nếu ARM đã ON từ lúc khởi động, state machine chưa sẵn sàng.

`SP` chỉ lên 1 khi ARM đã thành công và ga vượt ngưỡng khởi động. Giữ ga thấp thì `AR=1`, `SP=0`, `O=0/0/0/0` là bình thường. Nếu `AR=0`, đọc `BLK` theo bitmask; không tăng ga để cố vượt pre-arm.

## Thứ tự motor và mixer mà source đang giả định

| Index | Vị trí | TIM3 | Pin | Chiều quay nhìn từ trên |
|---|---|---|---|---|
| M1 | Front-left | CH1 | PA6 | CCW |
| M2 | Front-right | CH2 | PA7 | CW |
| M3 | Rear-left | CH3 | PB0 | CW |
| M4 | Rear-right | CH4 | PB1 | CCW |

Không tự đổi pin/slot hoặc đảo chiều motor mà không cập nhật đồng bộ `FlightPID_MixX()` và dây thực tế.

## Bộ điều khiển hiện tại

- Vòng góc: P (`ANGLE_KP=4`).
- Vòng rate: P (`RATE_KP_ROLL/PITCH=1.5`, `RATE_KP_YAW=1.0`).
- `RATE_KI=0`, `RATE_KD=0`.
- Giới hạn rate đặt 80 dps; correction giới hạn 200 DShot counts.

Vì vậy đây là cascade P-P có hồi tiếp gyro để tạo damping, chưa phải PID đã tuning đầy đủ. Giữ nguyên các gain khi chưa có log đáp ứng; không tăng gain chỉ để làm motor quay mạnh.

## BẮT BUỘC trước khi lắp cánh

1. **Tháo đủ 4 cánh quạt.** Build/flash bản này và xác nhận startup UART hiện `FLIGHT_CANDIDATE: ARM_ENABLE=1 ARM_LOCK=0 SCAN=0 ALT_HOLD=0`.
2. Khi ARM OFF, ga thấp, KILL OFF và frame nằm yên, chờ log có `BLK=800 RD=1 MAP=1 LS=1 LOW=1 FL=1 NE=1 IM=1` (các giá trị điều kiện có thể dao động theo trạng thái; `BLK` bằng 800 khi ARM OFF và các điều kiện khác đã đủ).
3. Gạt ARM ON. Xác nhận `AR=1`, `SP=0`; chưa nâng ga. Hạ ARM hoặc bật KILL để ngắt.
4. Với **cánh vẫn tháo**, dùng gá cố định chắc chắn; tăng ga vừa đủ để cả bốn motor quay ổn định. Khi nghiêng phải (right side down), lệnh hiệu chỉnh phải tăng cặp motor bên phải M2/M4 so với M1/M3. Khi chúi mũi xuống, lệnh hiệu chỉnh phải tăng cặp motor phía trước M1/M2 so với M3/M4. Nếu ngược, không lắp cánh; phải sửa dấu IMU/mixer trước.
5. Quan sát đủ bốn đầu ra `O=a/b/c/d`: không motor nào mất lệnh về 0 khi `AR=1`, `SP=1`, link và IMU còn hợp lệ. Nếu chỉ motor vật lý dừng trong khi `O` vẫn có lệnh, lỗi ở chân/tín hiệu/nguồn/ESC/motor chứ không phải lệnh mixer.
6. Xác nhận mất RF thật: tắt transmitter khi cánh tháo, receiver phải phát preset/failsafe đúng như guard đã giả định; firmware phải bỏ `L`, latch guard và đưa `O` về 0. Không được bỏ qua bài này.
7. Chỉ sau khi các kiểm tra trên đạt mới thực hiện lần hover đầu tiên ở khu vực thử nghiệm phù hợp, không có người gần vùng rotor. Không bật ALT_HOLD; MT-06 chỉ đo khoảng cách trong candidate này.

## Diễn giải log

- `A`: raw ARM switch; `K`: raw KILL switch; `T`: raw throttle.
- `LK`: compile-time bench lock (bản này phải là 0); `AR`: armed; `SP`: motors spinning state.
- `BLK`: bitmask lý do chưa arm; `RD`: arm_ready; `MAP`: throttle raw valid; `LS`: link stable; `LOW`: throttle low held; `FL`: level/rate prearm; `NE`: sticks neutral; `IM`: IMU fresh.
- `O`: bốn lệnh DShot thực tế theo thứ tự M1/M2/M3/M4.
- `RAT`: throttle ratio x1000; `BASE`: collective trước mixer; `MS`: desaturation scale x1000; `MO`: collective offset.
- `TX` là số callback DMA; không chứng minh ESC đã nhận packet. `DE` là số lỗi DMA.

## Build

Mở `UAV_.ioc` bằng STM32CubeIDE, xác nhận PA6/PA7/PB0/PB1 là TIM3 CH1..CH4, generate code nếu cần, sau đó build. Môi trường tạo gói không có GNU Arm Embedded Toolchain nên chưa thể xác nhận build/link ARM ở đây.

## Cập nhật rà soát 03/11/2026

ARM ON chỉ được xét tại cạnh OFF→ON; một lần thử không đạt bị tiêu thụ.
Nếu chưa đủ điều kiện, phải gạt OFF rồi ON lại; không giữ ON và chờ tự ARM.
Sau cạnh ON, `RD=0` là bình thường; xem `AR` để biết đã armed chưa.
Xem `STABILITY_REVIEW_VI.md` về thay đổi, kiểm thử và giới hạn chưa xác minh.
