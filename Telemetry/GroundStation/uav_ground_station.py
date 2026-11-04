#!/usr/bin/env python3
"""UAV telemetry ground station: UDP receiver, CRC checker, CSV logger, optional live plot.

Network defaults: join Wi-Fi AP UAV-TELEM, ESP32 AP 192.168.4.1, UDP port 14550.
Protocol: 88-byte little-endian v1 frame; see Telemetry/README_VI.md.
"""
from __future__ import annotations

import argparse
import csv
import socket
import struct
import sys
import time
from collections import deque
from pathlib import Path
from typing import Any

SYNC = b"\xA5\x5A"
VERSION = 1
MESSAGE_TYPE_FLIGHT = 1
PACKET_SIZE = 88
UDP_PORT = 14550
HELLO = b"UAVT_HELLO"

FLAG_NAMES = {
    "link_ok": 0,
    "failsafe": 1,
    "armed": 2,
    "spinning": 3,
    "imu_fresh": 4,
    "kill_active": 5,
    "alt_hold": 6,
    "imu_calibrated": 7,
    "range_valid": 8,
    "dma_error": 9,
    "output_active": 10,
    "config_ready": 11,
}


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: polynomial 0x1021, init 0xFFFF, xorout 0."""
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def decode_frame(data: bytes) -> dict[str, Any] | None:
    if len(data) != PACKET_SIZE or data[:2] != SYNC:
        return None
    if data[2] != VERSION or data[3] != MESSAGE_TYPE_FLIGHT:
        return None
    received_crc, = struct.unpack_from("<H", data, 86)
    if crc16_ccitt(data[:86]) != received_crc:
        return None

    seq, = struct.unpack_from("<H", data, 4)
    timestamp_ms, = struct.unpack_from("<I", data, 6)
    flags, = struct.unpack_from("<H", data, 10)
    rc = struct.unpack_from("<4H", data, 12)
    angles = struct.unpack_from("<2h", data, 20)
    gyro = struct.unpack_from("<3h", data, 24)
    accel = struct.unpack_from("<3h", data, 30)
    pid = struct.unpack_from("<3h", data, 36)
    base, mixer_scale = struct.unpack_from("<2H", data, 42)
    mixer_offset, = struct.unpack_from("<h", data, 46)
    mix = struct.unpack_from("<4h", data, 48)
    requested = struct.unpack_from("<4H", data, 56)
    output = struct.unpack_from("<4H", data, 64)
    battery_mv, current_ma, range_cm, imu_dt, motor_dt, dma_errors, rx_age = struct.unpack_from("<7H", data, 72)

    row: dict[str, Any] = {
        "seq": seq,
        "timestamp_ms": timestamp_ms,
        "flags": flags,
        "rc_throttle": rc[0], "rc_roll": rc[1], "rc_pitch": rc[2], "rc_yaw": rc[3],
        "roll_deg": angles[0] / 100.0, "pitch_deg": angles[1] / 100.0,
        "gyro_roll_dps": gyro[0] / 10.0, "gyro_pitch_dps": gyro[1] / 10.0, "gyro_yaw_dps": gyro[2] / 10.0,
        "accel_x_raw": accel[0], "accel_y_raw": accel[1], "accel_z_raw": accel[2],
        "pid_roll": pid[0] / 10.0, "pid_pitch": pid[1] / 10.0, "pid_yaw": pid[2] / 10.0,
        "base_dshot": base, "mixer_scale": mixer_scale / 1000.0, "mixer_offset": mixer_offset,
        "mix_m1": mix[0] / 10.0, "mix_m2": mix[1] / 10.0, "mix_m3": mix[2] / 10.0, "mix_m4": mix[3] / 10.0,
        "req_m1": requested[0], "req_m2": requested[1], "req_m3": requested[2], "req_m4": requested[3],
        "out_m1": output[0], "out_m2": output[1], "out_m3": output[2], "out_m4": output[3],
        "battery_v": None if battery_mv == 0xFFFF else battery_mv / 1000.0,
        "current_a": None if current_ma == 0xFFFF else current_ma / 1000.0,
        "range_cm": None if range_cm == 0xFFFF else range_cm,
        "imu_dt_max_ms": imu_dt, "motor_dt_max_ms": motor_dt,
        "dshot_errors": dma_errors, "rx_age_ms": rx_age,
    }
    for name, bit in FLAG_NAMES.items():
        row[name] = bool(flags & (1 << bit))
    return row


class LivePlot:
    def __init__(self, history: int = 500) -> None:
        try:
            import matplotlib.pyplot as plt
        except ImportError as exc:
            raise RuntimeError("--plot requires matplotlib: python3 -m pip install matplotlib") from exc
        self.plt = plt
        self.times: deque[float] = deque(maxlen=history)
        self.roll: deque[float] = deque(maxlen=history)
        self.pitch: deque[float] = deque(maxlen=history)
        self.gyro_roll: deque[float] = deque(maxlen=history)
        self.gyro_pitch: deque[float] = deque(maxlen=history)
        self.fig, (self.ax_att, self.ax_gyro) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        self.line_roll, = self.ax_att.plot([], [], label="Roll")
        self.line_pitch, = self.ax_att.plot([], [], label="Pitch")
        self.line_gx, = self.ax_gyro.plot([], [], label="Gyro roll")
        self.line_gy, = self.ax_gyro.plot([], [], label="Gyro pitch")
        self.ax_att.set_ylabel("Angle (deg)")
        self.ax_gyro.set_ylabel("Rate (deg/s)")
        self.ax_gyro.set_xlabel("Time (s)")
        self.ax_att.grid(True); self.ax_gyro.grid(True)
        self.ax_att.legend(loc="upper right"); self.ax_gyro.legend(loc="upper right")
        self.fig.suptitle("UAV Telemetry – attitude and rate")
        self.fig.tight_layout()
        plt.ion(); plt.show(block=False)
        self.start = time.monotonic()
        self.last_draw = 0.0

    def update(self, row: dict[str, Any]) -> None:
        now = time.monotonic()
        self.times.append(now - self.start)
        self.roll.append(row["roll_deg"]); self.pitch.append(row["pitch_deg"])
        self.gyro_roll.append(row["gyro_roll_dps"]); self.gyro_pitch.append(row["gyro_pitch_dps"])
        if now - self.last_draw < 0.1:
            return
        self.last_draw = now
        x = list(self.times)
        self.line_roll.set_data(x, list(self.roll)); self.line_pitch.set_data(x, list(self.pitch))
        self.line_gx.set_data(x, list(self.gyro_roll)); self.line_gy.set_data(x, list(self.gyro_pitch))
        for ax in (self.ax_att, self.ax_gyro):
            ax.relim(); ax.autoscale_view()
        self.fig.canvas.draw_idle()
        self.plt.pause(0.001)


def run() -> int:
    parser = argparse.ArgumentParser(description="Receive UAV UDP telemetry and record CSV")
    parser.add_argument("--ip", default="192.168.4.1", help="ESP32-CAM SoftAP address")
    parser.add_argument("--port", type=int, default=UDP_PORT, help="UDP local/remote port")
    parser.add_argument("--csv", default="uav_telemetry.csv", help="CSV output path")
    parser.add_argument("--plot", action="store_true", help="show optional live matplotlib plots")
    args = parser.parse_args()

    plotter = LivePlot() if args.plot else None
    output_path = Path(args.csv)
    fieldnames = [
        "seq", "timestamp_ms", "flags", "link_ok", "failsafe", "armed", "spinning", "imu_fresh",
        "kill_active", "config_ready", "alt_hold", "imu_calibrated", "range_valid", "dma_error", "output_active",
        "rc_throttle", "rc_roll", "rc_pitch", "rc_yaw", "roll_deg", "pitch_deg",
        "gyro_roll_dps", "gyro_pitch_dps", "gyro_yaw_dps", "accel_x_raw", "accel_y_raw", "accel_z_raw",
        "pid_roll", "pid_pitch", "pid_yaw", "base_dshot", "mixer_scale", "mixer_offset",
        "mix_m1", "mix_m2", "mix_m3", "mix_m4", "req_m1", "req_m2", "req_m3", "req_m4",
        "out_m1", "out_m2", "out_m3", "out_m4", "battery_v", "current_a", "range_cm",
        "imu_dt_max_ms", "motor_dt_max_ms", "dshot_errors", "rx_age_ms",
    ]

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.port))
    sock.settimeout(0.1)

    packets = 0
    bad_packets = 0
    sequence_gaps = 0
    last_sequence: int | None = None
    last_hello = 0.0
    last_status = time.monotonic()
    start = time.monotonic()

    print(f"Listening UDP 0.0.0.0:{args.port}; HELLO -> {args.ip}:{args.port}")
    print(f"CSV: {output_path.resolve()}")
    print("Join Wi-Fi UAV-TELEM first. Ctrl+C to stop.")

    try:
        with output_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader(); csv_file.flush()
            while True:
                now = time.monotonic()
                if now - last_hello >= 0.75:
                    try:
                        sock.sendto(HELLO, (args.ip, args.port))
                    except OSError as exc:
                        print(f"HELLO send error: {exc}", file=sys.stderr)
                    last_hello = now

                try:
                    payload, address = sock.recvfrom(2048)
                except socket.timeout:
                    payload = b""
                    address = None

                if payload:
                    row = decode_frame(payload)
                    if row is None:
                        bad_packets += 1
                    else:
                        seq = int(row["seq"])
                        if last_sequence is not None:
                            gap = (seq - last_sequence - 1) & 0xFFFF
                            if gap < 0x8000:
                                sequence_gaps += gap
                        last_sequence = seq
                        packets += 1
                        writer.writerow(row)
                        if packets % 10 == 0:
                            csv_file.flush()
                        if plotter is not None:
                            plotter.update(row)

                now = time.monotonic()
                if now - last_status >= 1.0:
                    elapsed = max(now - start, 0.001)
                    rate = packets / elapsed
                    if last_sequence is None:
                        print(f"waiting... packets={packets} invalid={bad_packets}")
                    else:
                        # Most recent valid row is intentionally not retained outside CSV/plot.
                        print(f"{address or '-'} packets={packets} rate={rate:.1f}Hz invalid={bad_packets} seq_gaps={sequence_gaps}")
                    last_status = now
    except KeyboardInterrupt:
        print("\nStopping ground station...")
    finally:
        sock.close()
    print(f"Saved {packets} valid packets to {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
