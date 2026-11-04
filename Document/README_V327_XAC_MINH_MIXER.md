# UAV V3.2.7 — KIỂM TRA DẤU IMU & MIXER, KHÔNG QUAY MOTOR

**Đây là bản PASSIVE, không phải firmware bay.** Giữ nguyên .ioc, TIM3, DMA và chân điều khiển. `UAV_BENCH_ARM_LOCK=1`, `UAV_PROPS_OFF_ESC_TEST=0` và buộc bốn `output_dshot[] = 0` trước mỗi lần gửi DMA. Không có chế độ quay thử motor trong bản này.

## Sơ đồ motor đã xác nhận
Nhìn từ trên xuống, đầu UAV hướng lên:

```text
             ĐẦU
   M1 FL(CCW)        M2 FR(CW)
       PA6             PA4
             [UAV]
   M3 RL(CW)         M4 RR(CCW)
       PB0             PB1
```

Hàm `FlightPID_MixX()` đã được đổi theo sơ đồ thực:

```
M1 = +Roll +Pitch +Yaw
M2 = -Roll +Pitch -Yaw
M3 = +Roll -Pitch -Yaw
M4 = -Roll -Pitch +Yaw
```

**Các dấu này theo quy ước giả định** `+Roll` = bên phải chúc xuống, `+Pitch` = mũi ngẩng, `+Yaw` = xoay theo chiều kim đồng hồ nhìn từ trên. Phải xác nhận MPU đã lắp đúng trục và dấu trước khi dùng cho điều khiển thực.

## Cách nạp

1. Đảm bảo đang dùng **project V3.2.6** có sẵn các file `Core/Src/flight_pid.c` và `Core/Inc/flight_pid.h`.
2. **Tháo cả 4 cánh và ngắt pin ESC 3S.** Cấp nguồn MCU qua USB/ST-LINK; chỉ cần cảm biến và SBUS hoạt động.
3. Sao lưu các file tương ứng và ghi đè **cả ba file** trong ZIP vào project (uav_test.c, flight_pid.c, flight_pid.h).
4. Project > Clean > Build > Flash, mở **một** chương trình Serial trên `/dev/ttyACM0` baud 115200.
5. Banner phải có `V327 PASSIVE_IMU_MIXER_VERIFY ... LOCK=1 ESC_TEST=0 OUTPUT=0`.

## Cách xác minh trên bàn

**Không gạt ARM ON để kiểm tra.** ARM OFF, KILL ON, ga thấp. Đặt ba cần Roll/Pitch/Yaw về giữa. Giữ khung thật nằm ngang (dùng thước thủy hoặc mặt phẳng) trong vài giây, sau đó nghiêng các trục *riêng lẻ* và quan sát các dòng `V...`.

1. **Nghiêng phải ~15° (bên phải xuống), giữ 1–2 giây:** nếu đúng dấu trục, `R` phải **tăng** so với lúc ngang; `P` gần như không đổi. Khi dừng nghiêng, `G` gần 0. Mục tiêu hiệu chỉnh là M2/M4 cao hơn M1/M3 trong `MIX` (chênh lệch tương đối, không phải lệnh DShot).
2. **Nghiêng trái ~15°:** `R` phải **giảm**; `MIX` M1/M3 cao hơn M2/M4.
3. **Ngẩng mũi ~15°:** `P` phải **tăng**; `MIX` M3/M4 cao hơn M1/M2 (phản lực để hạ mũi).
4. **Chúi mũi ~15°:** `P` phải **giảm**; `MIX` M1/M2 cao hơn M3/M4.
5. **Xoay thủ công theo chiều kim đồng hồ nhìn từ trên:** lúc xoay `GZ` phải theo **quy ước yaw dương**; `PIDyaw` phải chống lại chiều xoay. Trường hợp `GZ` ngược dấu, chưa được điều khiển yaw thật.

Ví dụ định dạng log (số liệu minh họa, không phải dữ liệu đo):

```
V100 L=1 RC=961/992/1099 R=1500 P=0 G=0/0/0 PID=-90/0/0 MIX=-90/90/-90/90 O=0000 LK=1
```

`R`, `P` = độ nhân 100. `G` = tốc độ góc X/Y/Z nhân 10. `PID` và `MIX` = hiệu chỉnh tương đối chưa đo thực tế; `O=0000` là toàn bộ bốn lệnh DShot bằng 0. `LK=1` là vẫn khóa ARM.

### Hai vấn đề đặc biệt cần ghi nhận
- Giá trị khi nằm ngang trước đây khoảng `R=-235`, `P=-115` (đơn vị 0,01 độ). Chưa tự động bù: phải dùng **thước thủy xác định khung cân** trước khi đo offset; nếu MPU lắp xoay ngược, chỉ trừ offset không sửa được trục.
- Kết quả `MIX` có thể chịu ảnh hưởng của Roll/Pitch/Yaw cùng lúc hoặc chuyển động còn rung. Giữ đúng một trục và đợi tốc độ quay gần 0 để đọc.

## Kết quả đạt mới là gì?
Chỉ đạt **đúng logic hình học mixer trên máy tính**, **chưa xác minh cảm biến có đúng dấu**. Đạt test không cánh không chứng minh PID đã được hiệu chuẩn, công suất đủ, hay máy có thể bay thật. Không chỉnh `UAV_BENCH_ARM_LOCK` thành 0 và không lắp cánh để thử bay tự do với firmware này.
