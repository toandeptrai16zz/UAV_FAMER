# UAV PID Blackbox R5 — Captive cage test (KHÔNG bay tự do)

## Khác R4
R4 khóa ARM để xoay thân bằng tay. R5 cho phép **một lần ARM và hoạt động motor có tải ngắn** trong lồng bảo vệ bằng compile flag `UAV_CAGE_TEST_ARM_ENABLE = 1`, không sửa PID, mixer, chiều motor, giao thức ESP32 hoặc dashboard. Thông số trong Flash vẫn áp dụng; phải Đọc PID trước khi thử. Việc chạy qua test phần mềm không chứng minh đã an toàn bay.

- Giữ CH5 ARM, CH6 KILL, kiểm tra SBUS, IMU, DShot, RF guard, và điều kiện ARM cũ.
- Chặn arm sau khi hết một lượt thử hoặc khi lỗi; **tắt/bật nguồn để thử lần tiếp theo**.
- Tự ngắt và khóa motor khi chạy motor ≥5,5 s, khi gyro Yaw ≥110°/s liên tục 80 ms, một trục gyro ≥240°/s, góc Roll/Pitch ≥22° liên tục 80 ms hoặc ≥35° tức thời, hoặc dữ liệu không hợp lệ.
- Giới hạn lệnh motor tối đa DShot **1550** bằng mixer desaturation. Không được coi đây là chỉ số thrust hoặc mức ga chính xác. Không cố tăng ga để vượt giới hạn.
- Ghi ring 200 Hz ~2,56 s với các trường cũ và **DShot thực**; kích hoạt khi motor chạy và gyro Yaw ≥45°/s (hoặc đã chạy ≥1,2s). Ghi thêm ~0,8 s, gửi CSV **chỉ khi motor dừng, CH6 KILL ON, ARM OFF**, không chen 512 gói vào dữ liệu khi đang bay.
- KILL trên tay **không thay thế khóa ngắt nguồn động lực vật lý độc lập**. Receiver có thể gửi lại kênh cũ sau khi mất RF, nên bắt buộc có ngắt nguồn ESC không phụ thuộc STM32.

## Quy trình trước thử có cánh (bắt buộc)
1. Build STM32CubeIDE và nạp STM32; ESP32 và Ground Station dùng đúng bản R4/R5 cùng giao thức type-2.
2. **Tháo cánh, ngắt nguồn động lực**, kiểm tra KILL, ARM, chuyển động IMU, các motor đúng thứ tự, kiểm tra nguồn và kiểm tra dashboard vẫn hoạt động. Không bỏ qua thử ngắt công suất ESC bên ngoài.
3. Dùng **lồng cứng có tấm chắn kín, cố định UAV vào bệ không cho khung thoát ra**, không dùng dây mềm vướng cánh; không có người trong tầm cánh hoặc vị trí bắn vỡ. Cố định tránh dây tạo mô-men Yaw/Pitch giả. Có người hỗ trợ quan sát ngắt nguồn riêng.
4. Cho chạy lần thử ngắn duy nhất. Nếu Yaw có xu hướng tăng, lắc quá mạnh, cánh chạm vật cản hoặc ESC bất thường: KILL và cắt nguồn động lực ngay. Không bay tự do.
5. Sau khi motor dừng: ARM OFF, KILL ON, ga về thấp. Giữ STM32/ESP32 có nguồn logic để gửi blackbox khoảng 512 frame (~10–20 s, phụ thuộc UART). Không tắt nguồn logic trước khi CSV xuất xong.
6. Trên Ubuntu chạy `python3 realtime_dashboard.py --ip 192.168.4.1`, gửi file `logs/*_PID200.csv` cùng file flight CSV để phân tích. Phải kiểm tra dòng số mẫu, thiếu gói, giới hạn PID, sai thời gian.

## Khác biệt và giới hạn
- R5 **không sửa nguyên nhân xoắn**, không chứng minh mô-men thực tế của motor. Chỉ hỗ trợ ghi dữ liệu phản hồi có tải trên giá thử có che chắn.
- Bộ lọc gyro R3 ±2000°/s và gia tốc vẫn dùng như cũ.
- Đã mở ARM thông qua compile flag; chuyển `UAV_CAGE_TEST_ARM_ENABLE` về `0u` để trở về **khóa ARM kiểu R4**.
- Không có OTA/bật chế độ này qua Wi-Fi; cần build/nạp lại STM32.
- Auto-disarm phụ thuộc cảm biến và mã; đây không phải hệ thống safety-rated. Thử KILL và ngắt nguồn động lực vật lý trên bệ trước khi lắp cánh.
- Phân biệt *test thuật toán trong lồng* và *cho phép bay tự do*: bản R5 chỉ dành cho cái đầu.
