# UAV V3.3.2 — PROPS OFF: Kiểm tra ARM/KILL + MC7RE failsafe bằng motor thật

**CHỈ TEST TRÊN BỆ VỚI CẢ 4 CÁNH ĐÃ THÁO. KHÔNG PHẢI FIRMWARE BAY.**

## File cập nhật
- `Core/Src/uav_test.c` — từ bản V3.3.1 hiện đang chạy, thêm chế độ DShot scanner có bảo vệ.
- `Core/Inc/mc7re_guard.h` — cùng phiên bản với V3.3.1 (ghi đè nếu cần).

Giữ nguyên `.ioc`, HAL, DShot TIM3/DMA, chân M1=PA6 M2=PA4 M3=PB0 M4=PB1, RC, PID, offset IMU.

**LOCK=1** ở mọi thời điểm. Không mở ARM bay, không sử dụng PID để tăng ga. Khi quét từng motor, firmware chỉ gửi mức DShot 120 (0,6 s) rồi 180 (0,9 s), giữa các motor nghỉ 0,5 s. Chỉ một motor chạy tại một thời điểm. Không suy ra bay ổn định từ bài thử này.

## Quy trình thử (cánh tháo hết, cố định khung chắc chắn)
1. Ngắt nguồn ESC, nạp V3.3.2; bật tay điều khiển. Bật nguồn cho receiver/STM32 trước; cấp pin ESC khi đã xác nhận **4 cánh tháo hoàn toàn**. Giữ người/vật xa trục quay. Mở serial 115200.
2. Để **ARM OFF** (`A≈1000`), **KILL ON** (`K≈1800`), **ga thấp** (`T≈40`), các cần R/P/Y ở giữa (`N=1`), máy gần ngang, chờ >3 giây; nếu kiểm tra hiệu chuẩn đầu-cuối ga chưa hoàn thành thì giữ KILL ON + ARM OFF và đưa ga lên cao rồi về thấp theo hướng dẫn hiệu chuẩn đã làm trước đó (CHỈ khi motor không chạy, ARM OFF, KILL ON).
3. Khi `B=1` (READY) trên log, chuyển **KILL OFF** (`K≈1000`) và **gạt ARM OFF -> ON** (`A≈1800`) *một lần*. Khi bắt đầu phải thấy `B=2`; `M=1..4`, `CMD=120/180`, `O=120/0/0/0` v.v. Các motor lần lượt M1, M2, M3, M4 (mỗi motor tối đa 1,5s), tự dừng sau ~7,5s.
4. Bài test NGẮT KHẨN: tái chuẩn bị (ARM OFF + KILL ON + ga thấp >1s), rồi chạy lại. Đang khi có một motor quay, gạt ngay KILL ON. Motor phải dừng; `O=0/0/0/0`, `B=4` hoặc `B=1` tùy trạng thái chuẩn bị.
5. Bài test MẤT SÓNG: tái chuẩn bị, bắt đầu test; khi motor đang quay, tắt tay điều khiển. Receiver MC7RE sẽ xuất PRE=1 và/hoặc cờ; motor phải dừng ngay. Xác nhận `L=0`, sau 300ms `FS=4 LAT=1`, `O=0/0/0/0`. Nếu không dừng, **ngắt nguồn pin ESC ngay**, không tiếp tục test.
6. Sau khi bật tay lại, giữ ARM OFF, KILL ON, ga thấp, cần giữa >=2s. LAT phải về 0 và có thể tái chuẩn bị nếu cần. Ngắt nguồn ESC trước khi gắn các phần còn lại.

### Diễn giải
- `RX=1`: receiver vẫn xuất SBUS, không chứng minh còn RF.
- `L=0 FS=4 LAT=1`: firmware đã chốt khóa theo mẫu failsafe MC7RE.
- `B=0`: chưa chuẩn bị; `B=1`: sẵn sàng; `B=2`: đang quét motor; `B=3`: hoàn tất; `B=4`: hủy/lỗi.
- `M`: motor đang nhận DShot (1-4); `CMD`: mức DShot; `O`: vector gửi thực tế (0 nghĩa là DShot STOP); `TX` đếm DMA hoàn thành, `DE` lỗi DMA.
- `LK=1` luôn luôn; nếu không, tuyệt đối dừng thử.

### Giới hạn bắt buộc
- Failsafe hiện tại *suy ra từ giá trị kênh dự phòng*, không phải bằng chứng độc lập mất RF. Nếu preset receiver thay đổi, firmware có thể bỏ sót.
- Không lắp cánh trong lúc thử, không bay tự do hoặc buộc bệ với cánh. Cần kiểm tra hướng phản hồi dưới tải, PID/thrust, ESC command/failsafe, pin và khả năng chịu dòng trước khi phát hành bản bay.
- Bản vá chưa chạy trên STM32 vật lý hoặc build ARM đầy đủ; cần Build/Flash/test tại thiết bị. Chỉ các phép thử host được ghi nhận trong lúc tạo patch mới được coi đã thực hiện.
