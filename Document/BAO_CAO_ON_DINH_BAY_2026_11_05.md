# Phân tích UAV_DATN — log 05/11/2026 và bản thử nghiệm lọc góc

**Mục tiêu:** Giải thích vì sao khi tăng ga bộ điều khiển làm các motor lên/xuống thất thường; sửa có kiểm chứng các thành phần gây ước lượng góc sai mà không bỏ DShot/SBUS/KILL/failsafe/telemetry/MT06/DPS310/config/PID runtime.

**Giới hạn quan trọng:** Đây là patch phần mềm được thử bằng giả lập HAL (host), chưa phải firmware bay ổn định đã được chứng minh. Chưa có đo điện áp pin dưới tải, dòng từng motor, RPM, waveform DShot, log gyro 1 kHz hoặc flight test. **Không lắp cánh để thử sửa filter hoặc kiểm tra trục, không tự ý nâng P/I/D hoặc công suất để khắc phục motor giật.**

## 1. Kết quả đọc source & log thực tế

Input: `UAV_DATN(1).zip`, `Core/Src/uav_test.c`, `flight_pid.c` và hai CSV `Telemetry/UAV_GroundStation_Realtime_Charts/logs/uav_20261105_002500.csv` (301 mẫu), `uav_20261105_003040.csv` (4469 mẫu). 2 CSV có tổng cộng 4770 mẫu, khoảng ~50 Hz; packet nhận bị rớt một phần ở lớp truyền dẫn, không có log raw 1 kHz.

- Có 1159 mẫu `spinning=True`, lệnh motor đều khác 0. Khi không quay, output về 0 đúng logic; không phát hiện hàng nào `spinning=True` nhưng một motor có lệnh 0.
- Khi SBUS throttle >= 600 (272 mẫu), chênh lệch max-min DShot có **median 122** và **max 440**. Khi throttle <300 (537 mẫu, spinning), median chỉ **5**. Lệnh ga bị chia mạnh bởi PID khi rung/nghiêng.
- Ví dụ 00:31:57.313 ngày 05/11 giờ VN: throttle=878, `pitch=-20.65°`, `roll=-10.60°`, PID roll=108.7/pitch=111.5/yaw=51.5; `base=1060`; motor **1332 / 1012 / 1006 / 892**. Motor thấp hơn base là yêu cầu mixer; `mixer_scale=1`, `mixer_offset=0`, nên **không có giới hạn ga collective ẩn** ở điểm đó.
- Throttle raw cao nhất trong log = **880**. Với endpoint nguồn `40..1640` và deadband khoảng 40 count, base DShot cao nhất khoảng **1063/2047**, không phải toàn dải. **Nếu cần full stick nhưng log vẫn ~880, phải kiểm tra endpoint/switch/mapping radio; không thay PID để chữa throttle mapping.** Không có bằng chứng log này bao gồm thao tác full stick.
- Gia tốc tổng (theo 8192 LSB/g) ở throttle>=600 biến thiên **0.58–1.52g**. Tại sự kiện trên, accel x/y/z=-8696/3742/8128, tổng **1.52g**; góc suy riêng từ accel khác đáng kể góc đã lọc. Rung/di chuyển tịnh tiến có thể làm phép suy tilt từ gia tốc sai. Không thể khẳng định toàn bộ góc 20 độ là góc giả — drone có thể thực sự nghiêng.
- `dshot_errors=0` trên 2 log, `imu_dt_max_ms=1`, `motor_dt_max_ms=1`. Đây là cực đại ms trong telemetry; **không** chứng minh DShot waveform hoàn hảo, ESC đã nhận mọi packet, hay không có jitter dưới 1ms.
- Telemetry hiện đặt `battery_mv` và `current_ma` bằng 0xFFFF (không có cảm biến); **không loại trừ** pin tụt áp, nguồn BEC/ESC yếu, đầu nối kém, mô-tơ quá tải, hoặc ESC desync.
- Trong source, `UAV_ENABLE_ALT_HOLD=0`: chế độ bám cây/độ cao chưa tham gia điều khiển; đừng dựa vào MT06 để kết luận máy đã có Alt Hold.

## 2. Thay đổi nhỏ đã thực hiện trong `Core/Src/uav_test.c`

### 2.1. Sửa độ tin cậy của gia tốc kế trong complementary filter

**Source cũ:** `COMP_FILTER_ALPHA=0.98` tại mỗi 1 ms ⇒ thời hằng xấp xỉ **49.5 ms**. Khi máy tăng tốc/rung, ước lượng góc bị kéo về hướng của gia tốc biểu kiến quá nhanh. Ví dụ minh họa một sai số tilt gia tốc 45° kéo dài 30ms sẽ làm estimator lệch khoảng 20.5° nếu gyro yên và confidence=1.

**Source sửa thử:** `COMP_FILTER_TAU_S=0.50 s`, `alpha=exp(-dt/tau)`; confidence=1 khi |norm-1g|<=0.10g, về 0 khi >=0.30g, nội suy giữa hai mốc. Khi confidence=0 chỉ tích phân gyro; nếu init lần đầu mà gia tốc bất thường, hoãn khởi tạo attitude. Phần trục, trim và RC giữ nguyên. Tau/threshold chỉ là **giả thiết khởi đầu cần đo thực nghiệm**, không phải bộ lọc được tối ưu.

### 2.2. Sửa các conflict marker gây hỏng bộ kiểm thử

`tests/test_flight.c` và `tests/run_tests.sh` trong ZIP có `<<<<<<< HEAD`, `=======`, `>>>>>>>` chưa được resolve, khiến `sh tests/run_tests.sh` lỗi. Đã giữ các test đầy đủ (bao gồm CONFIG và runtime gains) rồi xóa conflict marker. `tests/test_protocol.py` tham chiếu `Telemetry/GroundStation`/`uav_control` không tồn tại; đã nối với `Telemetry/UAV_GroundStation_Realtime_Charts/uav_ground_station.py` thực tế và giữ kiểm tra wire-format.

### 2.3. Những gì KHÔNG thay đổi

- TIM3 DShot300 DMA/burst & PA6/PA7/PB0/PB1; motor mixer, full throttle DShot48..2047; không chặn ga hoặc thêm slew.
- Toàn bộ ARM/KILL/RF failsafe, watchdog sensor/PID, output stop, logic DAC/Flash config PID; `rate_kp=1.5/1.5/1.0`, `angle_kp=4.0`, `rate_ki=rate_kd=0`; PID có thể được **ghi đè từ flash config**, hãy GET gains trước khi tuning.
- ESP32-CAM, telemetry, GroundStation, barometer DPS310, MT06, code Alt Hold (đang tắt), project .ioc, driver HAL và MATLAB.

## 3. Phân biệt nguyên nhân trước khi chỉnh gain

| Hiện tượng | Giải thích khả dĩ | Kiểm tra không cánh/an toàn |
|---|---|---|
| `O1..O4` khác nhau mạnh và pitch/roll đo thay đổi | PID phản hồi góc/nghiêng/nhiễu accel | Đặt máy trên giá vững, log gyro/acc và nghiêng tay (ngắt ga); xác minh hướng điều chỉnh ở từng trục; đo rung IMU, chống rung cơ khí |
| `O1..O4` vẫn đều/lớn nhưng motor thật ngắt, kêu khục khặc | ESC desync, nguồn sụt, ESC/motor/cáp, quá tải cánh | Tháo cánh, kiểm tra động cơ từng kênh; dùng đồng hồ đo nguồn/công suất và oscilloscope; ghi RPM/ESC telemetry nếu ESC hỗ trợ; thử hoán đổi motor/ESC sau khi cắt nguồn |
| Full stick mà `T` không đến endpoint ~1640 | Transmitter endpoint/channel/mapping | Kiểm tra giá trị `T` trên log ở full stick khi KILL ON / ARM OFF; chỉ hiệu chỉnh lại `rc_input.h` khi có số đo thực |
| Nâng ga làm mất L/AR/SP hoặc `O=0` | Failsafe/IMU stale/power reset/logic stop | Quan sát `AR`, `SP`, `F`, `FS`, `AGE`, `IM`, `DE`, dấu hiệu reset MCU; mô phỏng mất RF khi cánh tháo |
| Lệch một góc dù IMU êm | Sai chiều motor/cánh, vị trí motor, trim, CG, gain thiếu | Xác minh tất cả vị trí cánh/chiều quay/trục/mixer, cân bằng khung; chưa tăng I khi chưa có rate-feedback chính xác |

## 4. Quy trình đưa đến bay ổn định (thứ tự ưu tiên)

**Bước A — xác minh phần cứng:** Kiểm tra chắc chắn motor/cánh/trọng tâm/khung/IMU và cáp tín hiệu ESC. Nếu dùng A2212 2200KV với 8045, cần đo lực đẩy, dòng và nhiệt độ thật với đúng hàng cụ thể; nhãn “40A ESC” không có nghĩa motor chịu được dòng cao. Nguồn pin/đấu dây phải đáp ứng tổng dòng khi cả 4 motor có tải.

**Bước B — đo mà không có cánh:** Nạp firmware patch, đảm bảo các interlock ARM/KILL/failsafe vẫn đúng. Kiểm tra chéo `roll/pitch/gyro` cùng chiều chuyển động và correction điều khiển ngược chiều lệch; không cho PID đánh motor sai góc. Dùng logic analyzer đo DShot300: TIM3 170MHz, period=567 ticks, 16 bit+tail; đo cả 4 kênh dưới nhiều mức ga. Đo từng motor và giai đoạn bắt đầu quay để xác định idle tối thiểu thật, không coi DShot=48 là RPM tối thiểu đảm bảo.

**Bước C — bổ sung dữ liệu:** Tần số telemetry hiện 50 Hz, chưa đủ phân tích rate/PID 1kHz. Log sensor gyro góc setpoint, góc ước lượng, raw accel norm, PID correction, motor command và saturation ở 200–500 Hz qua ring buffer (không chặn vòng motor); ưu tiên có **điện áp pin, dòng, RPM** từ ESC/cảm biến. Khi có log mới, so sánh trước/sau filter mà không đổi các điều kiện khác.

**Bước D — tuning có kiểm soát:** Sau khi hardware, signs và filtering được xác minh, mới chỉnh rate loop, angle loop, yaw và thêm I/D có anti-windup/derivative filter (không lấy thông số Betaflight/PX4 chép nguyên vì đơn vị PID khác). Đo đáp ứng trước/sau từng thay đổi trên giá thử an toàn hoặc bởi người có thiết bị thử nghiệm chuyên dụng; chỉ thử bay khi lực đẩy/nguồn/cánh/rung/failsafe đều được kiểm chứng.

**Bước E — chế độ nâng cao:** Giữ ALT_HOLD tắt; thêm logic chống mất mẫu MT06, đồng bộ dt và smoothing, hover thrust measured, voltage compensation và watchdog/brownout trước khi bật.

## 5. Test phần mềm đã chạy

`sh tests/run_tests.sh` ⇒ **PASS** toàn bộ bài kiểm tra host GCC+UBSan: ARM edge, KILL, failsafe, link timeout, watchdog PID/IMU, DShot packet/CRC, mixer 34.606 trường hợp bão hòa, config mode, runtime gain validation, UART telemetry v1 CRC/wire-format. Thêm test accel: từ chối tín hiệu 1.5g/30°, xử lý gravity-only 30° theo tau chậm, không khởi tạo attitude bằng acceleration spike. **Chưa** chạy STM32CubeIDE/ARM GCC cross-build, bench ESC hoặc bay thử.

## 6. Tài liệu công khai đối chiếu

- Betaflight PID Tuning: https://betaflight.com/docs/wiki/guides/current/PID-Tuning-Guide
- Betaflight RPM filtering và chống motor desync bằng Dynamic Idle: https://betaflight.com/docs/wiki/guides/current/DSHOT-RPM-Filtering
- PX4 — Airmode & mixer saturation: https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/config_mc/pid_tuning_guide_multicopter.md
- TDK MPU6500 Register Map: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6500-Register-Map2.pdf

**Chú ý:** Các nguồn trên là cách tiếp cận tham khảo, không chứng nhận patch này đã đủ bay an toàn.
