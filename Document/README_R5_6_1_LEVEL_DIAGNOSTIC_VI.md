# UAV R5.6.1 — Sửa chẩn đoán thao tác đặt mốc IMU

## Lỗi người dùng gặp

Hai giá trị `Roll tham chiếu`/`Pitch tham chiếu` = 0.000° là **mốc đang lưu**, không phải góc IMU đang đo. Nếu CAPTURE bị STM32 từ chối, cả hai số vẫn bằng 0. Giao diện R5.6 chỉ ghi lỗi vào nhật ký dưới cùng, dễ bỏ sót.

## Đã thay đổi

- `Core/Src/uav_config.c`: thêm vào phản hồi LEVEL (GET/CAPTURE/SAVE/CLEAR) góc Roll/Pitch đang đo (trước tare), số mẫu liên tiếp đã ghi trong cửa sổ 100 mẫu và mã nguyên nhân chưa thể ghi.
- `Telemetry/ESP32_CAM/src/main.cpp`: giải mã phản hồi mới, hiện hai góc hiện tại **tách biệt** với góc tham chiếu, hiển thị số mẫu `n/100`, chỉ rõ nguyên nhân từ chối ngay trong thẻ IMU; phiên bản `/health` là `UAV-ESP32-DIRECT-AP-R5.6.1-LV-DIAG`.
- **Không thay** PID, Yaw Kp, mixer, DShot, IMU filter, gyro bias, ARM/KILL/CONFIG safety, giới hạn ±5° hay cách lưu Flash của R5.6.
- Tương thích với STM32 R5.6: ESP32 R5.6.1 vẫn đọc phản hồi cũ 11 byte nhưng khi đó góc trực tiếp sẽ hiển thị `—`. Để xem góc cần nạp cả hai.

## Cài đặt (dựa trên source R5.6)

```bash
cd ~/Desktop/UAV_DATN
unzip -o ~/Downloads/UAV_R5_6_1_LEVEL_DIAGNOSTIC_PATCH.zip -d .
```

Chọn dự án STM32 trong CubeIDE → Refresh → Clean → Build → Flash.

```bash
cd ~/Desktop/UAV_DATN/Telemetry/ESP32_CAM
pio run -e esp32cam
```

Sau đó nạp firmware.bin cho ESP32 qua OTA trong điều kiện an toàn; khi OTA bị khóa, không vượt qua bảo vệ bằng cách sửa code. Kiểm tra `http://192.168.4.1/health` chứa `R5.6.1-LV-DIAG`.

## Thử trên bàn, KHÔNG lắp cánh/không cấp nguồn ESC

1. ARM OFF, KILL ON, ga thấp, RC/IMU hoạt động, bấm Đọc PID để thấy CONFIG READY.
2. Giữ khung ở tư thế bạn chủ động chọn; **chỉ thay đổi vài độ** và giữ yên ≥3 giây trước khi nhấn Đặt mốc vào RAM. Đừng đo ngay lúc còn đang nghiêng.
3. Bấm Đọc mốc. Hai dòng **Roll/Pitch đang đo** phải thay đổi khi thay đổi tư thế; hai dòng **tham chiếu** chỉ thay đổi sau khi CAPTURE thành công.
4. Nếu mẫu tĩnh chưa đủ 100, hoặc lỗi do chuyển động, nhìn chỉ dẫn ngay dưới dòng mẫu. Nếu Roll/Pitch đang đo >±5° thì thao tác sẽ bị từ chối có chủ ý (để tránh mốc quá lớn gây nguy hiểm).
5. Sau CAPTURE thành công mốc sẽ đổi và dấu `ĐÃ ĐẶT MỐC` xuất hiện. Chỉ cân nhắc Lưu Flash sau khi kiểm tra telemetry và thao tác quay tay không cánh.

**Lưu ý:** Một mốc do người dùng chọn chỉ đặt lại số 0 tương đối của Angle PID và telemetry. Nó không chứng minh cân bằng cơ khí hoặc sửa ma trận lắp đặt IMU sai trục.

## Kiểm thử

`tests/run_tests.sh`: PASS trên host, bao gồm đóng gói/gỡ gói điều khiển STM32 (22 byte payload), độ dài, CRC, số mẫu, thử mốc ngoài ±5°, hồi quy PID/IMU/DShot. Đã kiểm tra cú pháp JS bằng `node --check`. Chưa build ARM/ESP32 bằng toolchain gốc hoặc thử thực tế trên bo mạch.
