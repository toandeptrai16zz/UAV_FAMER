#!/usr/bin/env python3
"""STM32 PID configuration and bench-only ROM firmware update via ESP32-CAM.

Only use with all propellers removed. HTTP runs on the isolated Wi-Fi SoftAP.
Auth token: read from ESP32_CAM/secrets.h and supply through UAV_API_TOKEN env.
"""
from __future__ import annotations

import argparse
import hashlib
import http.client
import json
import os
from pathlib import Path
import sys
import urllib.error
import urllib.request
from typing import Any

FIELDS = (
    "angle_kp", "max_rate_dps", "roll_kp", "pitch_kp", "yaw_kp",
    "roll_ki", "pitch_ki", "yaw_ki", "roll_kd", "pitch_kd", "yaw_kd",
    "integral_limit", "correction_limit",
)


def request(host: str, method: str, path: str, token: str, data: bytes | None = None) -> dict[str, Any]:
    req = urllib.request.Request(
        f"http://{host}{path}", data=data, method=method,
        headers={"X-Auth-Token": token, "Content-Type": "text/plain"},
    )
    try:
        with urllib.request.urlopen(req, timeout=10) as res:
            return json.loads(res.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", "replace")
        raise RuntimeError(f"HTTP {exc.code}: {body}") from exc


def ota(host: str, token: str, path: Path) -> dict[str, Any]:
    data_size = path.stat().st_size
    if not 8 <= data_size <= 508 * 1024:
        raise ValueError("STM32 firmware size outside 8..520192 bytes")
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(16384), b""):
            h.update(chunk)
    boundary = "uavstm32firmwareboundary"
    prefix = (
        f"--{boundary}\r\n"
        'Content-Disposition: form-data; name="firmware"; filename="stm32.bin"\r\n'
        "Content-Type: application/octet-stream\r\n\r\n"
    ).encode("ascii")
    suffix = f"\r\n--{boundary}--\r\n".encode("ascii")
    conn = http.client.HTTPConnection(host, 80, timeout=120)
    try:
        conn.putrequest("POST", "/api/ota")
        conn.putheader("X-Auth-Token", token)
        conn.putheader("X-SHA256", h.hexdigest())
        conn.putheader("Content-Type", f"multipart/form-data; boundary={boundary}")
        conn.putheader("Content-Length", len(prefix) + data_size + len(suffix))
        conn.endheaders()
        conn.send(prefix)
        with path.open("rb") as f:
            for chunk in iter(lambda: f.read(4096), b""):
                conn.send(chunk)
        conn.send(suffix)
        response = conn.getresponse()
        payload = response.read().decode("utf-8", "replace")
        if response.status != 200:
            raise RuntimeError(f"OTA HTTP {response.status}: {payload}")
        return json.loads(payload)
    finally:
        conn.close()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="192.168.4.1")
    ap.add_argument("--token", default=os.environ.get("UAV_API_TOKEN", ""))
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status"); sub.add_parser("get"); sub.add_parser("save")
    change = sub.add_parser("set", help="Apply ONE gain (RAM only, no flash write)")
    change.add_argument("field", choices=FIELDS)
    change.add_argument("value", type=float)
    fw = sub.add_parser("ota", help="Bench-only STM32 firmware; requires unlocked ROM_OTA build")
    fw.add_argument("firmware", type=Path)
    fw.add_argument("--props-removed", action="store_true", required=True,
                    help="You must confirm the propellers are removed")
    args = ap.parse_args()
    if len(args.token) < 24:
        ap.error("set UAV_API_TOKEN to the 24+ character value from ESP32 secrets.h")
    if args.cmd == "status":
        result = request(args.host, "GET", "/api/status", args.token)
    elif args.cmd == "get":
        result = request(args.host, "GET", "/api/pid", args.token)
    elif args.cmd == "save":
        result = request(args.host, "POST", "/api/save", args.token, b"")
    elif args.cmd == "set":
        current = request(args.host, "GET", "/api/pid", args.token)
        if len(current.get("gains", [])) != len(FIELDS):
            raise RuntimeError("STM32 reply did not provide 13 gains")
        idx = FIELDS.index(args.field)
        gains = [float(x) for x in current["gains"]]
        gains[idx] = args.value
        body = ",".join(f"{x:.9g}" for x in gains).encode("ascii")
        result = request(args.host, "POST", "/api/pid", args.token, body)
    elif args.cmd == "ota":
        if not args.props_removed:
            ap.error("remove propellers before any OTA operation")
        result = ota(args.host, args.token, args.firmware)
    else:
        ap.error("unsupported action")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, OSError, urllib.error.URLError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2)
