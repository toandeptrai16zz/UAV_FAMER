# UAV_DATN — Tóm tắt nâng cấp (2026-11-04)

**Đọc tài liệu hướng dẫn chính:** [`HUONG_DAN_NANG_CAP_WIFI_PID_OTA_VI.md`](HUONG_DAN_NANG_CAP_WIFI_PID_OTA_VI.md).

- Core/Inc + Core/Src: thêm `uav_config.[ch]`, tham số PID runtime, cờ CONFIG READY.
- Telemetry/ESP32_CAM: mở rộng UDP v1 + HTTP PID + uploader/ROM writer (bench only).
- Telemetry/GroundStation: thêm `uav_control.py`; giữ nguyên `uav_ground_station.py`.
- Linker: giảm FLASH 512K → 508K, dành 4KB cho bản ghi gain.
- CMake: khai báo source `uav_config.c`.
- Secrets phải đổi trước khi ESP bật AP. Tất cả API HTTP yêu cầu token.
- Đường OTA cố ý **disabled** trong STM32 (`UAV_CONFIG_ENABLE_ROM_OTA=0`) cho tới khi được kiểm nghiệm phần cứng.
- Logic bay, PID/mixer, DShot, RC mapping gốc được giữ. Bản này **chưa được nạp/kiểm chứng bay**.

Test trên host: `bash tests/run_tests.sh` và `python3 -m py_compile Telemetry/GroundStation/*.py`.
