# Rà soát UAV_DATN3 — điều kiện để bay ổn định

Ngày: 03/11/2026. Đầu vào: UAV_DATN3.zip.

## Kết luận

Đã rà soát toàn bộ mã ứng dụng trong Core/Src và Core/Inc, luồng khởi động/ngắt, cấu hình build, timer/DMA, SBUS, MPU6500, DPS310, MT06 và mã MATLAB. Đã đọc cấu trúc/khối của mô hình Simulink trong SLX; không chạy MATLAB/Simulink. Các bản MATLAB ở thư mục gốc và Matlab_Simulink giống nhau theo byte. Với thư viện HAL/CMSIS/BSP, kiểm tra các đường gọi liên quan, không tuyên bố đã audit từng dòng của toàn bộ thư viện và các ví dụ đi kèm.

Bản gốc có nền tảng điều khiển góc nhưng chưa đủ bằng chứng để kết luận bay ổn định. Bản sửa này khắc phục lỗi logic đã tìm được, bổ sung chẩn đoán và lọc gia tốc; KHÔNG phải bộ gain đã tuning theo khung thật. Không có phép thử trên STM32, ESC hoặc máy bay trong lần rà soát này.

**Không xóa chức năng hoặc tệp gốc.** Các trạng thái bật/tắt tính năng, chân kết nối, thứ tự motor, hướng mixer, ga trực tiếp và gain điều khiển được giữ. Các tệp build cũ cũng được giữ để bảo toàn gói, nhưng không đại diện cho source đã sửa: phải clean/rebuild và chỉ nạp ELF vừa build.

## 1. Chức năng thực tế, tránh nhầm với mô phỏng

| Khối | Trạng thái trong source gốc và bản sửa |
|---|---|
| STM32G474RE, clock 170 MHz | Giữ nguyên |
| SBUS USART1 PB7, 100000, 8E2, RX inversion | Giữ nguyên runtime; đồng bộ thông số còn thiếu trong IOC |
| DShot300 TIM3 DMA burst 4 motor | Giữ nguyên, không đổi callback hoặc cách phát packet |
| M1 PA6, M2 PA7, M3 PB0, M4 PB1 | Giữ nguyên; sửa chú thích PA4 sai trong flight_pid.h |
| Ga tay trực tiếp, dải 48–2047; 0 là STOP | Giữ nguyên, không thêm ramp hoặc cap ga |
| ARM/KILL, guard MC7RE, phục hồi DMA | Giữ; sửa ARM thành cạnh OFF→ON thực sự |
| Điều khiển góc roll/pitch và tốc độ yaw | Giữ nguyên gain và mixer |
| DPS310 áp suất/độ cao tương đối | Giữ driver; chưa tham gia điều khiển bay |
| MT06 đo khoảng cách và bù nghiêng | Giữ; sửa lấy lại thời gian trước khi xét freshness |
| ALT_HOLD / bám ngọn cây | Mã giữ nguyên; đã là 0 và có #error ngăn bật từ bản gốc |
| Scanner ESC không cánh | Mã giữ nguyên; vẫn tắt trong flight candidate |
| Log UART, kiểm tra yaw thụ động | Giữ mọi trường cũ, thêm trường timing/lỗi IMU |
| MATLAB: cất/hạ cánh, waypoint, ROW | Giữ tất cả; đây là mô phỏng, không phải chức năng đã chạy trên STM32 |

Không có vòng điều khiển vị trí XY hoặc giữ hướng yaw tuyệt đối trong firmware. Máy có thể tự cân bằng góc mà vẫn trôi vị trí; MT06 đo xuống không tự cung cấp vị trí ngang.

## 2. Những lỗi đã sửa

Các số dòng dưới đây trỏ tới bản gốc trong ZIP đầu vào; bản sửa có dịch dòng.

### 2.1. ARM có thể xảy ra muộn khi công tắc vẫn ON

Vị trí: Core/Src/uav_test.c, motor_step(), khoảng dòng 674.

Bản cũ đặt arm_ready khi ARM OFF, sau đó giữ biến này khi ARM ON nhưng một điều kiện chưa đạt. Nếu người dùng giữ ON và khung dần nằm ngang, hoặc đủ thời gian giữ ga/link, điều kiện có thể chuyển sang đúng và máy tự ARM. Điều này không đúng với mô tả “cạnh OFF→ON”.

Bản sửa theo dõi trạng thái công tắc trước đó, chỉ xét ARM tại cạnh tăng và tiêu thụ cả lần thử thất bại. Chưa đạt thì phải OFF→ON lại. KILL và mọi điều kiện pre-arm vẫn giữ nguyên.

### 2.2. Mẫu SPI mới có thể che gián đoạn ước lượng góc

Vị trí: uav_test_loop(), khoảng dòng 1041; mpu6500_read_spi(); điều kiện imu_fresh.

Bản cũ cập nhật imu_last_good_ms ngay khi SPI thành công, nhưng nếu attitude_dt > 20 ms thì bỏ update_attitude. motor_step có thể thấy IMU còn “mới” mặc dù góc đang là giá trị trước khoảng gián đoạn.

Bản sửa có imu_control_valid riêng. Mẫu phục hồi có dt >20 ms bị đánh dấu không dùng cho điều khiển, xóa lịch sử góc/PID và đi qua đường disarm hiện có. Mẫu đúng nhịp kế tiếp khởi tạo lại góc; ARM vẫn cần thao tác công tắc mới. Ngưỡng 20 ms giữ nguyên, không coi đây là thời gian trễ bình thường được phép chấp nhận khi bay.

### 2.3. PID không hợp lệ không được biến thành điều khiển hở

Vị trí: đoạn FlightPIDSampleHold_Read(), khoảng dòng 794.

Bản cũ khởi tạo pid_roll/pitch/yaw bằng 0 và vẫn có thể dùng ga collective khi không đọc được output PID hợp lệ. Bản sửa xóa kết quả giữ khi dt không hợp lệ; nếu đang ARM+SPIN mà không có PID hợp lệ thì disarm/STOP. Không gửi bốn lệnh ga bằng nhau để che lỗi điều khiển.

### 2.4. Bộ lọc gia tốc và hệ số phụ thuộc chu kỳ

Vị trí: mpu6500_init_spi(), update_attitude().

Bản gốc cấu hình gyro DLPF qua 0x1A nhưng không ghi ACCEL_CONFIG2 (0x1D). Theo register map TDK, reset 0x00 tương ứng băng thông gia tốc 460 Hz. Bản sửa ghi 0x03 để dùng DLPF gia tốc 41 Hz, ODR 1 kHz. Đây là cấu hình khởi đầu cần đo lại rung và độ trễ trên khung thật, không phải kết quả tuning tối ưu.

Alpha bổ sung thay đổi theo dt: alpha = 0.98^(dt_ms). Ở 1 ms giữ chính xác hệ số cũ; ở 2 ms là 0.9604. Việc này giữ thời gian suy giảm nhất quán hơn khi vòng lặp bỏ nhịp. Chưa thay estimator bằng quaternion và chưa thêm cơ chế loại gia tốc tịnh tiến; các giới hạn đó vẫn còn.

### 2.5. Timestamp khoảng cách, chẩn đoán và tài liệu

Sau SPI, lấy HAL_GetTick lại trước phép trừ now - imu_last_good_ms; tránh dùng now cũ hơn timestamp vừa ghi và làm phép trừ unsigned tràn.

Thêm cuối dòng log:

| Trường | Ý nghĩa |
|---|---|
| IDT | Khoảng cách lớn nhất giữa các lần cập nhật IMU thành công, ms |
| MDT | Khoảng cách lớn nhất giữa các lần motor_step, ms, bỏ qua lần đầu |
| LATE | Số khoảng motor_step > 1 ms, cộng dồn |
| IGAP | Số lần dt attitude không hợp lệ, cộng dồn |
| IERR | Số lỗi đọc SPI IMU, cộng dồn |

Đây là chẩn đoán độ phân giải ms, không chứng minh jitter ở mức microsecond hoặc cảm biến thực sự sinh một mẫu mới mỗi lần đọc. IDT/MDT là cực đại từ khi khởi động, không phải dt hiện tại.

Sửa chú thích M2 từ PA4 thành PA7. Sửa checklist: khi ARM OFF, đủ các điều kiện khác thì BLK=800 (bit ARM OFF), không phải 000. Sau cạnh ON, RD=0 vì lần thử đã được tiêu thụ; AR mới là trạng thái armed.

IOC gốc thiếu khai báo đầy đủ baud/word length/parity/stop/mode của SBUS và để priority LPUART1=0 trong khi C đang dùng 6. Đã đồng bộ IOC theo runtime. Chưa chạy CubeMX để xác minh round-trip; ưu tiên build trực tiếp source đã cung cấp. Nếu generate lại, phải so diff, đặc biệt SBUS 100000/9-bit+parity even/2 stop/RX đảo và các USER CODE/custom driver.

## 3. Những việc còn phải giải quyết để bay ổn định

### 3.1. Gain hiện tại là P–P, không phải PID đã tuning

Trong flight_pid.c: ANGLE_KP=4; RATE_KP_ROLL/PITCH=1.5; RATE_KP_YAW=1; RATE_KI=0; RATE_KD=0; rate giới hạn 80 dps; correction mỗi trục giới hạn 200 DShot counts.

Hồi tiếp gyro vẫn tạo damping, nên không thể kết luận chỉ vì Ki/Kd=0 thì chắc chắn không bay được. Tuy nhiên P-only không có tích phân để bù mô-men lệch kéo dài mà không cần sai số duy trì. Không có log động học để biết gain hiện tại thiếu hay thừa. Không tự chép một bộ gain “chuẩn F450” vào source.

Thứ tự làm: xác minh chiều → đo timing/rung → tuning vòng rate → tuning vòng góc → mới đến vòng độ cao. Khi thêm I/D cần đồng thời xử lý lọc D theo dt, anti-windup khi mixer bão hòa và giới hạn tích phân theo đơn vị output. Không chỉ đổi hai macro từ 0 lên số tùy ý. PX4 cũng hướng dẫn tuning từ vòng rate; không lấy giá trị gain của PX4 để dùng trực tiếp ở đây vì đơn vị/output khác.

### 3.2. Đo motor idle thực, không coi 48 là tốc độ quay bảo đảm

48 là đầu dải lệnh DShot, không phải chứng nhận cả bốn ESC/motor đều quay đều ở mức đó. Mixer đang cho từng motor xuống 48. Nếu motor mất đồng bộ/dừng ở đầu dải, máy có thể nghiêng mạnh dù số O vẫn hợp lệ.

Đo mức bắt đầu quay và mức duy trì quay của từng motor/ESC khi tháo cánh. Sau đó mới quyết định idle tối thiểu chung và kiểm chứng có tải phù hợp. Bản sửa không tự đặt idle 120/180 theo scanner, vì số scanner chỉ là lệnh test, không phải dữ liệu đo khả năng giữ quay.

Giữ nguyên đặc tính hạ ga xuống dưới ngưỡng sẽ dừng toàn bộ motor. Đây không phải airmode; phải hiểu rõ đặc tính này trước thử nghiệm bay.

### 3.3. Kiểm tra trục, motor, cánh và trọng tâm trước tuning

Source giả định nhìn từ trên: M1 trước-trái CCW; M2 trước-phải CW; M3 sau-trái CW; M4 sau-phải CCW. Mapping IMU tạm thời: roll=-sensor_pitch; pitch=-sensor_roll; yaw_rate=-gyro_z. Trim và tâm RC lấy từ các log cũ, chưa có dữ liệu mới xác minh trên khung hiện tại.

Tháo đủ cánh, kiểm tra thủ công:

| Thao tác với stick trung lập | Phản hồi mong đợi theo quy ước source |
|---|---|
| Nghiêng phải, bên phải thấp | Roll dương; correction nâng M2/M4 so với M1/M3 |
| Chúi mũi xuống | Pitch âm; correction nâng M1/M2 so với M3/M4 |
| Quay khung thuận chiều kim đồng hồ nhìn từ trên | Gyro yaw dương; correction yaw âm |

Khi kiểm tra, dùng chênh lệch motor/correction; không đòi giá trị ga tuyệt đối phải tăng trong mọi tình huống vì desaturation có thể hạ collective. Kiểm tra chiều quay thực của motor, đúng loại cánh, vị trí dây, nguồn IMU, GND, rung khung, cân bằng cánh và trọng tâm. YQ=3 chỉ kiểm tra dấu phản hồi trong phần mềm với phép xoay tay; không xác minh mô-men phản lực thật.

### 3.4. Timing chưa được đo trên MCU

Vòng lặp dùng HAL_GetTick 1 ms và SPI blocking. Tên macro 1 ms không tự bảo đảm 1000 Hz. SPI cấu hình 170 MHz/256 ≈ 664 kbit/s; 15 byte mất tối thiểu khoảng 181 us trên dây, chưa tính HAL/ngắt/tính toán. Khi SPI lỗi có timeout, vòng điều khiển sẽ bị trễ. Cần đo loop execution và khoảng cách DShot bằng GPIO/DWT/logic analyzer trước khi khẳng định tần số thực.

DPS310 runtime đã dùng I2C interrupt; đã đối chiếu HAL_I2C_Mem_Read_IT trong thư viện đi kèm: BUSY trả về ngay. Không phát hiện đường chờ 25 ms trong hàm IT này và không thay nó bằng driver khác.

Log 5 Hz hiện tại phù hợp đọc trạng thái nhưng quá thưa để nhận diện dao động nhanh/tune D. Bước tuning tiếp theo cần log có timestamp ở tốc độ cao hơn: rate setpoint, gyro, angle, motor, saturation, dt; nên dùng buffer và truyền không chặn, không chỉ tăng tần số chuỗi UART dài hiện tại.

### 3.5. Failsafe còn phụ thuộc receiver thực tế

Giữ nguyên guard MC7RE và timeout 100 ms. Preset 1000/1000/1000, ga thấp bị chặn ngay khi candidate; 300 ms là thời gian latch preset, không phải trì hoãn dừng motor. Cờ frame-lost đơn lẻ hiện cũng dẫn đến latch/disarm. Điều này có thể khiến máy dừng motor khi RF chập chờn; đây là chính sách của source gốc, không tự nới trong bản sửa.

Nếu receiver giữ nguyên lệnh cũ và không cờ failsafe khi mất RF, source không thể chứng minh còn link chỉ bằng việc vẫn nhận SBUS. Phải thử tắt tay phát và mất dây SBUS với cánh tháo; kiểm tra nhiều vị trí ga/công tắc, xác minh receiver thực sự phát cờ/preset đúng. Không suy ra “an toàn” chỉ từ RX=1 hoặc TX DMA tăng.

### 3.6. Không bật ALT_HOLD chỉ bằng bỏ #error

Mã cũ dùng dt=1 ms cho đạo hàm trong khi MT06 có tốc độ riêng; có thể tạo xung D lớn lúc mẫu range thay đổi. ALT_HOVER_THROTTLE=1100 chưa được đo; bật mode có thể ép spinning=true; chuyển mode/mất range chưa có chuyển ga liên tục. DPS310_GetData chưa có freshness timestamp và state async chưa có watchdog giao dịch, nên dữ liệu cũ có thể vẫn mang valid=true nếu bus/cảm biến ngừng cập nhật.

Giữ nguyên mã này và trạng thái tắt. Muốn hoàn thiện phải có mẫu range theo sequence/timestamp, lọc đạo hàm, đo hover thrust, giới hạn tốc độ đứng, điều kiện chuyển mode và quy tắc mất range; không triển khai “bám cây” chỉ dựa vào góc và một số đo khoảng cách.

MT06 parser hiện yêu cầu payload DISTANCE_SENSOR >=14 byte cho cả MAVLink v1/v2. MAVLink v2 có thể rút gọn byte 0 ở cuối; cần kiểm chứng packet cảm biến thực trước khi sửa parser. Không ảnh hưởng attitude hiện tại vì range chưa dùng để điều khiển.

### 3.7. Mô phỏng chưa khớp firmware

uav_pid.m dùng 1.05 kg, plant mô-men → góc và motor lag 50 ms; Mo_phong_3D.m dùng 1.25 kg, điều khiển đơn vị rad/N.m, feedback vị trí/velocity giả lập và cấu hình riêng. Firmware dùng độ, dps và DShot counts, không có cùng plant hoặc vòng vị trí. SLX có plant denominator [0.015 0 0]. Những mô hình này không chứng minh gain C đã đúng.

uav_pid.m có ký tự z thừa cuối tệp; tên hàm chính quadrotor_pid_testbench không khớp tên Mo_phong_3D.m. Để bảo toàn phần mô phỏng, chưa tự sửa/đổi tên các tệp này. Khi dùng mô phỏng cần sửa các lỗi chạy riêng, đo tham số thật và đồng bộ actuator model/đơn vị/mixer trước khi so đáp ứng.

## 4. Kiểm thử đã thực hiện

Chạy `sh tests/run_tests.sh` trên host với GCC và UndefinedBehaviorSanitizer. Bộ test gọi trực tiếp motor_step/uav_test_loop của source thật, mock HAL/peripheral, không viết lại state machine thành bản giả.

Đã đạt:

- ARM ở boot đang ON bị từ chối; OFF→ON đủ điều kiện được ARM.
- Cạnh ON khi khung chưa ngang bị từ chối; giữ ON rồi đặt ngang không tự ARM.
- KILL ngắt, nhả KILL không tự ARM lại.
- Cờ failsafe, timeout SBUS, preset MC7RE, lỗi DMA đều ra STOP.
- IMU stale, PID NaN và PID dt không hợp lệ ra STOP.
- SPI đọc thành công sau gap >20 ms không che lỗi góc; mẫu phục hồi tiếp theo không tự ARM.
- Hai mẫu cùng timestamp ms giữ output hợp lệ trước đó, không tạo lỗi mất IMU giả.
- Hệ số suy giảm filter: một bước 2 ms khớp hai bước 1 ms trong tình huống gyro=0.
- 34.606 tổ hợp desaturation: output trong dải, giữ chênh lệch correction theo scale; kiểm tra dấu roll/pitch và input NaN.
- Toàn bộ 2.048 payload DShot: giá trị/bit telemetry/checksum; giải mã đủ 16 kênh SBUS; tâm stick và ga thấp.

Kiểm tra cú pháp 12 tệp C trong Core/Src với header HAL/CMSIS thật: không lỗi; có một warning do phép đảo bit trên host 64-bit khác độ rộng unsigned long của ARM. Đây chỉ là kiểm tra cú pháp, không phải build firmware.

**Chưa build/link ARM:** môi trường không có cmake hoặc arm-none-eabi-gcc. Không tạo ELF/BIN mới, không xác minh footprint/stack/linker hay DMA waveform. Chưa chạy CubeMX, MATLAB hoặc test bay. Test host không đo được độ ổn định vật lý.

## 5. Cách dùng bản sửa và dữ liệu cần gửi tiếp

1. Giải nén sang thư mục làm việc riêng. Import project vào CubeIDE và Clean/Build từ source, không nạp binary cũ trong ZIP. Nếu dùng CMake: cần CMake/Ninja/GNU Arm Embedded; chạy `cmake --preset Debug`, `cmake --build --preset Debug` tại gốc project. Các lệnh này chưa được chạy thành công trong môi trường rà soát.
2. Tháo toàn bộ cánh, cố định khung; thực hiện lại ARM/KILL, tắt TX, rút SBUS, kiểm tra từng motor và chiều correction. Lỗi IMU/PID có chủ đích chỉ thử trên bàn, không tạo lỗi giữa lúc bay.
3. Đọc IDT/MDT/LATE/IGAP/IERR và DE. Không chấp nhận IGAP tăng, mất motor hoặc correction sai chiều trước thử nghiệm tiếp. Nếu LATE tăng, đo thực bằng DWT/GPIO để xác định độ trễ; không chỉ điều chỉnh PID để che lỗi timing.
4. Gửi log đặt yên 10–20 s và log nghiêng/phản hồi từng trục với cánh tháo; ảnh nhìn từ trên có đánh dấu mũi, vị trí IMU và motor; loại motor/KV, ESC/firmware, cánh, pin, khối lượng đầy đủ và mức ga motor quay đều.
5. Chỉ sau khi kiểm tra trên đạt mới tiến tới thử đáp ứng có kiểm soát và tuning bằng log có tải. Không thể xác định gain bay cuối cùng từ log xoay tay không cánh.

## 6. Tham khảo kỹ thuật

- TDK InvenSense, MPU-6500 Register Map, mục 4.5–4.8: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6500-Register-Map2.pdf
- MAVLink, Packet Serialization (quy tắc rút gọn payload v2): https://mavlink.io/en/guide/serialization.html
- PX4, Multicopter PID Tuning Guide: https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter

Bằng chứng chính cho các lỗi là source trong ZIP. Nguồn trên dùng kiểm tra cấu hình filter và phương pháp tuning; không chứng nhận bản firmware này.
