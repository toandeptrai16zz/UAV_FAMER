"""R5.5 type=3 88-byte IMU sample frames, stdlib only.

DWT cycle counter is modulo 2^32. Samples in a capture are <= ~1 sec apart;
the per-capture cycle difference is wrapped modulo 2^32, using HCLK MHz from
STM32 as its scale. 'relative_us' is derived by the dashboard receiver in
capture-index order (not in UDP arrival order).
"""
import struct
from uav_blackbox_decoder import crc16_ccitt

IMU_FIELDS = [
    'received_at_utc','capture_id','capture_kind','capture_index','capture_total',
    'stamp_ms','cycle_count','hclk_mhz', 'accel_x_raw','accel_y_raw','accel_z_raw',
    'gyro_x_raw','gyro_y_raw','gyro_z_raw', 'accel_x_g','accel_y_g','accel_z_g',
    'gyro_x_dps_uncal','gyro_y_dps_uncal','gyro_z_dps_uncal',
    'relative_us','sample_gap',
]

def decode_imu(raw: bytes):
    if len(raw) != 88 or raw[:4] != b'\xa5\x5a\x01\x03':
        return None
    if crc16_ccitt(raw[:86]) != struct.unpack_from('<H',raw,86)[0]:
        return None
    cid,total,idx,kind=struct.unpack_from('<HHHH',raw,12)
    if not (1 <= total <= 1024 and idx < total and kind in (1,2)):
        return None
    stamp,cycle=struct.unpack_from('<II',raw,20)
    values=struct.unpack_from('<6h',raw,28)
    mhz=struct.unpack_from('<H',raw,40)[0]
    if not 1 <= mhz <= 300: return None
    row=dict(capture_id=cid,capture_index=idx,capture_total=total,
             capture_kind='BOOT_STILL' if kind==1 else 'MOTOR_STOP',
             stamp_ms=stamp,cycle_count=cycle,hclk_mhz=mhz)
    for name,value in zip(('accel_x','accel_y','accel_z','gyro_x','gyro_y','gyro_z'),values):
        row[name+'_raw']=value
        row[name+('_g' if name.startswith('accel') else '_dps_uncal')]=round(value/(8192.0 if name.startswith('accel') else 16.4),6)
    row['relative_us']=''
    row['sample_gap']=''
    return row
