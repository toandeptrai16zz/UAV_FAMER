STM32 NUCLEO-G474RE - chuyen UART ESP32 sang A4/A5 khong han cau

Source: UAV_DATN5.zip do nguoi dung gui. Chi sua 2 file:
- Core/Src/stm32g4xx_hal_msp.c: doi LPUART1 PA2/PA3 (AF12) thanh PC1/PC0 (AF8)
- UAV_.ioc: cap nhat pin LPUART1_TX=PC1, LPUART1_RX=PC0 de lan sau Generate Code khong bi hoan nguyen

DAY MOI:
STM32 CN8 A4 (PC1 LPUART1_TX) -> ESP32-CAM GPIO13 (RX)
STM32 CN8 A5 (PC0 LPUART1_RX) <- ESP32-CAM GPIO14 (TX)
STM32 GND <-> ESP32 GND

Chan A4/A5 dua ra PC1/PC0 qua SB35/SB36 (0 ohm theo anh cua nguoi dung).
Kiem tra thong mach khi mat nguon truoc khi cap dien; khong can thao SB13, SB17, SB19, SB23.
Khong ket noi dong thoi vao Arduino D1/D0 cu.

HUONG DAN AP DUNG:
1. Backup original project STM32CubeIDE.
2. Giai nen ZIP vao ROOT project (noi co file UAV_.ioc va thu muc Core).
3. Mo CubeIDE build/flash STM32 bang ST-LINK.
4. Ket noi day moi A4/A5, cap nguon logic khi canh quat thao va nguon dong luc ngat.
5. Wi-Fi UAV-DIRECT: GET http://192.168.4.1/api/status; uart_bytes va frames phai tang.

Khong co file ESP32 can sua. Khong thay doi DShot, PID, SBUS, MT06, DPS310.
Nguoi tao patch chua build bang STM32CubeIDE phan cung thuc.
