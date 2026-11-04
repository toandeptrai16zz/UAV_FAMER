#!/usr/bin/env python3
"""Wire-level regression checks for the unchanged UDP v1 + config frames."""
from __future__ import annotations
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'Telemetry' / 'GroundStation'))
from uav_ground_station import crc16_ccitt, decode_frame, PACKET_SIZE
from uav_control import FIELDS


def control_frame(message_type: int, sequence: int, payload: bytes = b'') -> bytes:
    assert len(payload) <= 60
    msg = b'\xc3\x3c' + bytes([1, message_type]) + struct.pack('<HB', sequence, len(payload)) + payload
    return msg + struct.pack('<H', crc16_ccitt(msg))


def run() -> None:
    assert crc16_ccitt(b'123456789') == 0x29B1
    # Telemetry: 88-byte v1; bit 11 only, all historical offsets unchanged.
    packet = bytearray(PACKET_SIZE)
    packet[0:4] = b'\xa5\x5a\x01\x01'
    struct.pack_into('<H', packet, 4, 0xFEFF)
    struct.pack_into('<I', packet, 6, 123456)
    struct.pack_into('<H', packet, 10, (1 << 5) | (1 << 11))
    struct.pack_into('<H', packet, 86, crc16_ccitt(packet[:86]))
    values = decode_frame(bytes(packet))
    assert values and values['config_ready'] and values['kill_active']
    assert values['seq'] == 0xFEFF and values['timestamp_ms'] == 123456
    packet[20] ^= 0x01
    assert decode_frame(bytes(packet)) is None

    request = control_frame(0x10, 0x1234)
    assert request[:7] == b'\xc3\x3c\x01\x10\x34\x12\x00'
    assert len(request) == 9
    gains = struct.pack('<13f', 4.0, 80.0, 1.5, 1.5, 1.0, 0,0,0,0,0,0,30,200)
    assert len(FIELDS) == 13 and len(gains) == 52
    response = control_frame(0x90, 0x1234, b'\x00\x01' + gains)
    assert len(response) == 63
    assert crc16_ccitt(response[:-2]) == struct.unpack('<H', response[-2:])[0]
    assert struct.unpack('<13f', response[9:-2])[0:5] == (4.0, 80.0, 1.5, 1.5, 1.0)
    print('PASS CRC16 golden vector, v1 UDP frame+flag, UART request/response wire format')


if __name__ == '__main__':
    run()
