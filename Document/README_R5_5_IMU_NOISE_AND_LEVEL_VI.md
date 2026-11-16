# UAV R5.5 — Chẩn đoán rung IMU và hiệu chuẩn cân bằng khung

**Nền mã nguồn:** UAV R5.3 đầy đủ + bản vá UART/ESP32 R5.4 từ `UAV_R5_4_TELEMETRY_PID200_FIX.zip`.
**Mục đích:** Tạo log từ IMU thô để tìm nhiễu, và tạo **đề xuất** trim góc trên mặt phẳng ngang. Không dự đoán PID từ cân nặng.

## 1. Những thay đổi thực tế

- Thêm `Core/Src/uav_imu_raw.c` và `Core/Inc/uav_imu_raw.h`: bộ đệm vòng **1024 mẫu, 20 byte/mẫu ≈20 KiB**, nhận dữ liệu từ các lần đọc SPI MPU6500 thực tế. Mục tiêu đọc ~1.000 lần/giây; tốc độ thực phải xác nhận bằng CSV (không ép lịch ngắt thời gian thực).
- Lưu riêng sáu thanh ghi thô `Accel X/Y/Z`, `Gyro X/Y/Z`, `HAL_GetTick` và `DWT->CYCCNT`; **không ghi UART, tính FFT hoặc cấp phát động trong vòng 1 ms**.
- Khi khởi động xong và MPU6500 hiệu chuẩn thành công, giữ 1024 mẫu đầu tiên làm baseline `BOOT_STILL`; sau khi truyền xong, trở lại ghi vòng. Nếu motor được kích hoạt trước đó, ưu tiên dữ liệu motor.
- Nếu firmware đã chạy motor có tải trong chế độ thử của R5.x, bộ đệm giữ ~1 giây mẫu cuối và chốt khi motor ngừng, loại `MOTOR_STOP`. Không chủ động cấp nguồn hay ARM để chụp dữ liệu.
- Truyền **loại 3** (88 byte, CRC16) *sau* Blackbox PID200 loại 2, chỉ khi disarm, motor dừng và KILL ON/guard latched. Vẫn xen kẽ khung bay loại 1, tránh dashboard báo mất telemetry.
- Firmware ESP32 có thêm chuyển tiếp loại 3; Dashboard ghi file `uav_..._IMU1000.csv` riêng, vẫn giữ `uav_...csv` và `_PID200.csv` như R5.4.
- Thêm `tools/uav_level_calibration.py` để tính trim từ CSV telemetry **khi khung được đặt ngang thực sự**. Mặc định **không sửa** offset trước đây (0.68° và 0.24°). Tự sửa file header chỉ khi người vận hành xác nhận ngang bằng tùy chọn `--confirm-level` và `--write-header`.
- Thêm `tools/uav_imu_noise_report.py` để thống kê độ lệch chuẩn, peak-to-peak, thời gian mẫu và thành phần 2–35 Hz, không yêu cầu scipy/numpy.
- Cập nhật `cmake/stm32cubemx/CMakeLists.txt` và test mock STM32.

**Không đổi:** gain PID, toán học bộ điều khiển, mapping motor/ESC, DShot300, SBUS sáu kênh, ARM/KILL, ngưỡng cage guard, dòng UART 115200, DLPF MPU6500 (~41 Hz), thuật toán Complementary và tỷ lệ gyro ±2000°/s. Chưa thêm notch/low-pass do log không tải không chứng minh nhiễu cần lọc.

## 2. Cài đặt từ dự án R5.4 hiện tại

Sao lưu thư mục hiện tại trước khi sửa, đặc biệt `Core/Src/uav_test.c`, cấu hình PID và bản `.bin` ESP32 đang chạy. Tại **thư mục gốc `~/Desktop/UAV_DATN`**, giải nén file patch R5.5 (chứa các thư mục `Core/`, `Telemetry/`, `tools/`, `cmake/`). **Phải chép cả file .c VÀ .h**.

```bash
cd ~/Desktop/UAV_DATN
unzip -o ~/Downloads/UAV_R5_5_IMU_DIAGNOSTIC_PATCH.zip -d .
ls Core/Src/uav_imu_raw.c Core/Inc/uav_imu_raw.h \
   Core/Inc/uav_level_trim_user.h \
   Telemetry/UAV_GroundStation_Realtime_Charts/uav_imu_raw_decoder.py
```

STM32CubeIDE: chọn `Refresh` → `Clean` → `Build`. Kiểm tra không có lỗi link `UAVIMURaw_*` và dung lượng RAM/FLASH nằm trong giới hạn chip, sau đó mới flash khi đã tháo cánh và ngắt nguồn ESC.

ESP32: cũng phải nạp firmware mới vì đây là loại gói **3**, không thể chỉ nạp STM32. Trong `Telemetry/ESP32_CAM` chạy:

```bash
pio run -e esp32cam
```

Sau khi build, file OTA là `.pio/build/esp32cam/firmware.bin`. OTA chỉ khi đáp ứng điều kiện an toàn, không bypass khóa OTA. Có thể kiểm tra firmware bằng `http://192.168.4.1/health`, phiên bản mục tiêu `UAV-ESP32-DIRECT-AP-R5.5-IMU3`.

## 3. Thử lần đầu: KHÔNG CÁNH / KHÔNG NGUỒN ESC

1. Ngắt nguồn ESC, tháo 4 cánh. Đặt **toàn bộ khung** (không chỉ hộp IMU) trên mặt ngang đã kiểm chứng. ARM OFF, KILL ON, ga tối thiểu.
2. Khởi động STM32/ESP32, giữ không dịch chuyển khi MPU6500 tự hiệu chuẩn 3–5 giây. Mở Ground Station:

   ```bash
   cd ~/Desktop/UAV_DATN/Telemetry/UAV_GroundStation_Realtime_Charts
   python3 realtime_dashboard.py --ip 192.168.4.1
   ```

3. Giữ nguồn logic ít nhất **45–60 giây sau khi bắt đầu thấy telemetry**, vì 1024 gói IMU1000 truyền trên UART 115200 xen khung trạng thái có thể mất hơn 30 giây. Không cần ARM hoặc ga.
4. Kiểm tra `logs/uav_..._IMU1000.csv`: có nhiều dòng (lý tưởng ~1024, UDP có thể mất mẫu), cột `capture_kind=BOOT_STILL`, `relative_us` tăng, và số lượng mẫu thực tế. Nếu chỉ có tiêu đề, kiểm tra đúng phiên bản ESP32, KILL, CRC, thời gian chờ.
5. Gửi **cả** CSV telemetry thường và `_IMU1000.csv`. Blackbox PID200 vẫn được giữ nguyên.

## 4. Hiệu chuẩn ngang: không tự đoán offset

Với file CSV telemetry từ lần đặt ngang thực sự:

```bash
python3 tools/uav_level_calibration.py \
  Telemetry/UAV_GroundStation_Realtime_Charts/logs/uav_YYYYMMDD_HHMMSS.csv
```

Nó chỉ in khuyến nghị `IMU_ROLL_MOUNT_TRIM_DEG`, `IMU_PITCH_MOUNT_TRIM_DEG`. **Không được áp dụng khi đặt khung trên mặt nghiêng, tay cầm, hoặc cánh tay chưa phẳng.** Nếu sau khi kiểm tra khung *thực sự ngang*, có thể ghi header bằng:

```bash
python3 tools/uav_level_calibration.py logs/uav_YYYYMMDD_HHMMSS.csv \
  --confirm-level --write-header Core/Inc/uav_level_trim_user.h
```

Đường dẫn `logs/...` ở dòng trên phải thay bằng đường dẫn thật. Nên build nạp và xác nhận **Roll/Pitch trở về gần 0° khi đặt ngang** trong bài thử **không cánh** trước khi cân nhắc các phép thử động cơ. Không dùng auto-zero giữa chuyến bay.

Theo file `uav_20261116_210155.csv` trước đây: khi dùng 501 mẫu đứng yên, khuyến nghị toán học là `IMU_ROLL_MOUNT_TRIM_DEG=0.86761f` và `IMU_PITCH_MOUNT_TRIM_DEG=-3.02329f`, **NHƯNG chỉ hợp lệ nếu vị trí đặt lúc đó đã được xác nhận là mặt ngang thật sự**. Bản R5.5 không tự áp dụng hai số này.

## 5. Báo cáo rung từ IMU1000

```bash
python3 tools/uav_imu_noise_report.py \
  Telemetry/UAV_GroundStation_Realtime_Charts/logs/uav_YYYYMMDD_HHMMSS_IMU1000.csv
```

Kết quả gồm tỷ lệ mẫu nhận được, khoảng cách DWT, RMS (sigma), peak-to-peak, một số đỉnh 2–35 Hz. Dải gyro và accel vẫn dùng DLPF khoảng 41 Hz nên kết quả không mô tả trung thực nhiễu motor cao tần. **Không sử dụng FFT này để tự chọn tần số notch cao hơn dải DLPF**. Khi có bằng chứng cần đổi DLPF, thử riêng sau khi kiểm định trên bệ đo được che chắn.

## 6. Những giới hạn chưa kiểm chứng

- Đã chạy unit test của bộ đệm, thử gói C → Python CRC, UDP → CSV, và bộ hồi quy hiện có trên host. **Chưa build với toolchain arm-none-eabi hoặc PlatformIO trong môi trường này; chưa thử phần cứng thật.**
- Bộ ghi raw mục tiêu 1 kHz phụ thuộc vòng đọc SPI thực tế và DWT; không phải ADC/IMU mẫu hoàn toàn không lọc hoặc DMA hardware timestamp.
- Khi truyền đủ PID200 (~15 giây) và IMU1000 (~31 giây), tổng tải có thể gần 45–50 giây; **không reset STM32/ESP32 khi đang tải**.
- Bản R5.5 kế thừa chế độ thử có tải và khóa an toàn của R5.4. Do drone từng xoắn tới sát 500°/s, không sử dụng bản build này để thử bay tự do hay tắt bảo vệ. Hãy ưu tiên xác nhận capture boot tại bàn trước.
