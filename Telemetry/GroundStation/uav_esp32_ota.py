#!/usr/bin/env python3
"""Upload ESP32-CAM firmware.bin over the UAV HTTP portal, no external Python packages.

Usage:
    export UAV_API_TOKEN='...'
    python3 uav_esp32_ota.py --host 192.168.99.123 status
    python3 uav_esp32_ota.py --host 192.168.99.123 flash \
        --firmware ../ESP32_CAM/.pio/build/esp32cam/firmware.bin --confirm-bench

Safety: the ESP32 must independently approve OTA using fresh FC CONFIG telemetry
or a physical GPIO2-to-GND bench-service jumper (not a software override).
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import secrets
import sys
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


def get_base(host: str) -> str:
    host = host.strip().rstrip("/")
    if not host.startswith(("http://", "https://")):
        host = "http://" + host
    return host


def json_request(url: str, token: str, body: bytes | None = None, extra_headers: dict | None = None, timeout: int = 12) -> dict:
    headers = {"X-Auth-Token": token}
    if extra_headers:
        headers.update(extra_headers)
    req = Request(url, data=body, headers=headers, method="POST" if body is not None else "GET")
    try:
        with urlopen(req, timeout=timeout) as response:
            return json.loads(response.read().decode("utf-8"))
    except HTTPError as error:
        detail = error.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"HTTP {error.code}: {detail}") from error
    except URLError as error:
        raise RuntimeError(f"Khong ket noi ESP32: {error.reason}") from error


def main() -> int:
    ap = argparse.ArgumentParser(description="ESP32 OTA qua Wi-Fi, firmware.bin (khong phai STM32 OTA)")
    ap.add_argument("--host", required=True, help="IP that ESP32 tren router, hoac 192.168.4.1 khi noi mang Setup")
    ap.add_argument("--token", default=os.getenv("UAV_API_TOKEN", ""), help="Nen su dung bien moi truong UAV_API_TOKEN")
    sub = ap.add_subparsers(dest="action", required=True)
    sub.add_parser("status", help="Xem dieu kien cho phep update ESP32")
    flash = sub.add_parser("flash", help="Upload firmware.bin")
    flash.add_argument("--firmware", required=True, type=Path, help="Duong dan .pio/build/esp32cam/firmware.bin")
    flash.add_argument("--confirm-bench", action="store_true", help="Xac nhan da thao canh, ngat nguon ESC va da dat CONFIG an toan")
    args = ap.parse_args()
    if len(args.token) < 24:
        ap.error("Can API token ESP32 (export UAV_API_TOKEN='...')")
    root = get_base(args.host)
    try:
        status = json_request(root + "/api/esp32/ota/info", args.token)
        print(json.dumps(status, ensure_ascii=False, indent=2))
        if args.action == "status":
            return 0
        if not args.confirm_bench:
            ap.error("Yeu cau --confirm-bench. Thao canh, ngat nguon ESC va dat mode an toan truoc khi OTA")
        if not status.get("can_update", False):
            raise RuntimeError("ESP32 tu choi OTA: " + str(status.get("safety")) + ". Kiem tra FC/CH8 hoac GPIO2 jumper bench")
        path = args.firmware
        if path.name != "firmware.bin" or not path.is_file():
            raise RuntimeError("Chi upload file firmware.bin duoc build boi PlatformIO cho board esp32cam")
        data = path.read_bytes()
        if len(data) < 1024 or len(data) > int(status.get("ota_bytes_free", 0)):
            raise RuntimeError(f"File {len(data)} bytes khong vua OTA slot {status.get('ota_bytes_free')} bytes")
        if data[:1] != b"\xE9":
            raise RuntimeError("Khong phai ESP32 app firmware image (magic byte != 0xE9)")
        sha = hashlib.sha256(data).hexdigest()
        boundary = "----UAVESP32OTA" + secrets.token_hex(12)
        before = (
            f"--{boundary}\r\n"
            'Content-Disposition: form-data; name="firmware"; filename="firmware.bin"\r\n'
            "Content-Type: application/octet-stream\r\n\r\n"
        ).encode("ascii")
        after = f"\r\n--{boundary}--\r\n".encode("ascii")
        payload = before + data + after
        print(f"[OTA] Upload ESP32: {len(data)} bytes, SHA256={sha}")
        result = json_request(root + "/api/esp32/ota", args.token, payload, {
            "Content-Type": "multipart/form-data; boundary=" + boundary,
            "X-SHA256": sha,
            "X-Firmware-Size": str(len(data)),
        }, timeout=180)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        print("[OTA] Neu thanh cong, ESP32 se tu reboot khi van an toan. Kiem tra SSID/IP sau khi reboot.")
        return 0
    except (OSError, ValueError, RuntimeError) as exc:
        print("[ERROR]", str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
