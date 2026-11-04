# STM32G474RE — V2 chẩn đoán, tháo cánh quạt

**Không phải firmware đủ điều kiện bay.** Bản này giữ nguyên .ioc, khóa ARM bằng `UAV_BENCH_ARM_LOCK=1`, khóa Alt Hold bằng `UAV_ENABLE_ALT_HOLD=0`. Mục tiêu là thu số liệu an toàn để xác minh phần cứng và tìm lỗi PID/mixer mà không cho motor quay có lực đẩy.

## Các thay đổi so với bản trước
- Giữ 17 file source/header mới nhất và các sửa lỗi driver MT-06 (không lọc theo số lần gọi GetData).
- Bổ sung log `BENCH` mỗi ~1 giây: trạng thái SBUS, số frame lỗi, tuổi IMU, góc Roll/Pitch (đơn vị 0,01 độ), tuổi MT-06, CRC lỗi, lỗi UART, DPS310, lỗi DShot DMA.
- Không thay đổi file `.ioc` hoặc viết lại các driver STM32 HAL chuẩn.
- Không thay PID bằng các hệ số MATLAB/Simulink, vì mô hình MATLAB đang sử dụng giả định khối lượng 1,05/1,25 kg, các hệ số và đơn vị điều khiển khác với firmware.

## Cách ghép vào CubeIDE
Sao lưu dự án. Copy `Core/Src/*` và `Core/Inc/*` lên các file tương ứng trong dự án. Không copy thêm file có hậu tố `(9)`; package dùng đúng tên chuẩn. Giữ `.ioc` của bạn. Clean, Build. Nếu lỗi hãy gửi nguyên văn Console Build, không đoán được HAL vendor khi thiếu thư mục Drivers.

## Trình tự kiểm tra (KHÔNG CÁNH)
1. Bật nguồn, log báo `LOCK=1`, `ARM=0`, `DSHOT=0/0/0/0` dù gạt công tắc ARM/tăng ga. Nếu không đạt: ngừng test và ngắt pin.
2. Cử động radio: `LINK` chuyển 1 khi tín hiệu hợp lệ, gạt KILL, ngắt radio: xác minh failsafe trên log. Không lắp cánh.
3. Nghiêng thân về phải/trái, chúi/ngẩng: `R`/`P` biến thiên đúng dấu, số liệu ít nhiễu. Giá trị in ở đơn vị 0,01 độ; ví dụ `R=900` là 9 độ.
4. Di chuyển MT-06 trước mặt phẳng cố định: `MT_OK` tăng, `MT_BAD` không tăng nhanh, `MT_AGE` thường nhỏ hơn timeout. Che cảm biến và xác nhận `Rvalid=0` khi lỗi.
5. DShot cần kiểm chứng bằng oscilloscope/logic analyzer; lệnh DShot=0 trong bản kiểm tra này, không có nghĩa đã xác minh chiều quay motor hay mixer.

## Các dữ liệu bắt buộc để ra quyết định về bay
- Sơ đồ từng động cơ M1..M4: vị trí vật lý, chiều CW/CCW, TIM3_CH1..CH4, hướng lắp MPU6500, các chân CN5/CN10 là tín hiệu nào. Hai motor CW và hai motor CCW **chưa đủ** xác minh ma trận mixer.
- Dữ liệu đo thực: trọng lượng bay 1,2 kg chính xác, thrust/ga treo từ đo kiểm, loại motor+ESC+cánh, pin, vòng điều khiển thực tế, bộ PID với đơn vị điều khiển có thể so sánh.
- Bộ log 30–60s thử KILL, mất SBUS, IMU nghiêng, MT06; thời gian chạy vòng lặp, số lỗi DMA. Sẽ đối chiếu với mô phỏng MATLAB và dựng test motor không cánh có giám sát.

**Không được bỏ khóa ARM chỉ để thử cất cánh.** Một firmware tính toán đúng chưa đủ chứng minh an toàn bay trong môi trường thật; cần thử bench và thử có dây neo/khu vực kiểm soát bởi người có kinh nghiệm sau khi chứng minh các điều kiện an toàn.
