"""Separate V1/type=2 buffered 200 Hz PID snapshot decoder; unchanged flight V1/type=1."""
import struct

DIAG_FIELDS = [
    'received_at_utc', 'capture_id', 'capture_index', 'capture_total',
    'stamp_ms', 'pid_dt_us', 'motor_dt_us',
    'gyro_roll_dps', 'gyro_pitch_dps', 'gyro_yaw_dps',
    'rate_set_roll_dps', 'rate_set_pitch_dps', 'rate_set_yaw_dps',
    'roll_p', 'pitch_p', 'yaw_p',
    'roll_i', 'pitch_i', 'yaw_i',
    'roll_d', 'pitch_d', 'yaw_d',
    'out_m1', 'out_m2', 'out_m3', 'out_m4', 'base_dshot',
    'mixer_scale', 'imu_seq', 'flags',
]

def crc16_ccitt(data: bytes) -> int:
    c = 0xffff
    for b in data:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ (0x1021 if c & 0x8000 else 0)) & 0xffff
    return c

def decode_diag(raw: bytes):
    if len(raw) != 88 or raw[:4] != b'\xa5\x5a\x01\x02':
        return None
    if crc16_ccitt(raw[:86]) != struct.unpack_from('<H', raw, 86)[0]:
        return None
    cid, total, idx = struct.unpack_from('<HHH', raw, 12)
    if not 1 <= total <= 512 or idx >= total:
        return None
    values = struct.unpack_from('<IHH15h8H', raw, 20)
    stamp, pid_dt, motor_dt = values[:3]
    gyro = values[3:6]
    rate = values[6:9]
    pterm = values[9:12]
    iterm = values[12:15]
    dterm = values[15:18]
    motor = values[18:22]
    base, scale, imu_seq, flags = values[22:26]
    row = dict(capture_id=cid, capture_index=idx, capture_total=total,
               stamp_ms=stamp, pid_dt_us=pid_dt, motor_dt_us=motor_dt,
               base_dshot=base, mixer_scale=scale/1000.0,
               imu_seq=imu_seq, flags=flags)
    for names, data, factor in [
        (('gyro_roll_dps','gyro_pitch_dps','gyro_yaw_dps'), gyro, 10),
        (('rate_set_roll_dps','rate_set_pitch_dps','rate_set_yaw_dps'), rate, 10),
        (('roll_p','pitch_p','yaw_p'), pterm, 10),
        (('roll_i','pitch_i','yaw_i'), iterm, 10),
        (('roll_d','pitch_d','yaw_d'), dterm, 10),
    ]:
        row.update({name: val/factor for name, val in zip(names, data)})
    row.update({f'out_m{i+1}': v for i,v in enumerate(motor)})
    return row
