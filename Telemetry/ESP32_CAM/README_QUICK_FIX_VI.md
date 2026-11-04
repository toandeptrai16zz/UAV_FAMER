# ESP32-CAM UAV — Fix loi build

## Sua gi

- Xoa bien `packet[88]` va hang `CTRL_MAX` khong su dung.
- Tuong thich SHA-256 ca mbedTLS 2.x (`*_ret`) va 3.x (khong co `_ret`).
- Kiem tra ma loi tu tung buoc SHA-256 truoc khi chap nhan firmware.
- Khong sua giao thuc UART, API PID, UDP telemetry, HTTP hay luong bootloader.

## Cach dung

1. Mo `include/secrets.h`. Thay `UAV_WIFI_PASSWORD` (it nhat 8 ky tu) va `UAV_API_TOKEN` (it nhat 24 ky tu). KHONG de gia tri CHANGE_*. Hai gia tri nay duoc gui duoi dang plaintext neu su dung HTTP; chi ket noi AP trong moi truong tin cay.
2. Mo nguyen thu muc nay bang VS Code + PlatformIO.
3. PlatformIO: `Clean` -> `Build` -> `Upload` (chi upload khi build thanh cong).
4. Kiem tra serial 115200 baud. Khong ket noi motor co canh quat khi test.

## Neu van build loi

- Trong terminal chay `pio run -e esp32cam -v`.
- Gui log tu dong dau tien co chu `error:` den `[FAILED]`, khong chi warning cuoi log.

## Luu y

- Mo moi project nay de khong co 2 file firmware `src/main.cpp` va `.ino` cung bi bien dich.
- Khong bao gom/thay doi firmware STM32; OTA STM32 van bi khoa mac dinh theo du an goc.
- Chua the kiem chung firmware tren thiet bi trong moi truong nay.
