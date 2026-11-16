#!/usr/bin/env python3
"""Propose IMU level trims from DISARMED + motionless flight CSV.

It never assumes the log was physically level. To modify the firmware header,
the operator must assert --confirm-level and supply --write-header PATH.
"""
import argparse, csv, math, statistics
from pathlib import Path

def num(row,key):
    try: return float(row[key])
    except (ValueError,TypeError,KeyError): return None

def main():
    ap=argparse.ArgumentParser(description='IMU mount trim estimator, no automatic flight calibration')
    ap.add_argument('csv', type=Path, help='Flight telemetry CSV from a verified level surface')
    ap.add_argument('--seconds',type=float,default=10.0,help='Take most recent N seconds (default 10)')
    ap.add_argument('--confirm-level', action='store_true',help='Assert airframe was physically level, props off')
    ap.add_argument('--write-header',type=Path,help='Write Core/Inc/uav_level_trim_user.h only with --confirm-level')
    a=ap.parse_args()
    if a.write_header and not a.confirm_level: ap.error('--write-header requires --confirm-level')
    with a.csv.open(newline='',encoding='utf-8-sig') as f:
        rows=list(csv.DictReader(f))
    if not rows: ap.error('No CSV rows')
    last=num(rows[-1],'timestamp_ms')
    if last is None: ap.error('timestamp_ms missing')
    filtered=[]
    for r in rows:
        t=num(r,'timestamp_ms')
        if t is None or last-t > a.seconds*1000 or t>last: continue
        if r.get('armed','').strip().lower() in ('true','1'): continue
        if r.get('spinning','').strip().lower() in ('true','1'): continue
        if r.get('imu_calibrated','').strip().lower() not in ('true','1'): continue
        x=[num(r,k) for k in ('accel_x_raw','accel_y_raw','accel_z_raw',
                              'gyro_roll_dps','gyro_pitch_dps','gyro_yaw_dps')]
        if None in x: continue
        filtered.append(x)
    if len(filtered)<100: ap.error(f'Only {len(filtered)} valid samples. Need >=100 static samples.')
    gxs=list(zip(*filtered))
    ax,ay,az=(statistics.mean(col) for col in gxs[:3])
    gnorm=math.sqrt(ax*ax+ay*ay+az*az)/8192.0
    sig_norm=statistics.pstdev(math.sqrt(sum(v*v for v in x[:3]))/8192.0 for x in filtered)
    gs=[statistics.pstdev(col) for col in gxs[3:]]
    maxgyro=max(max(abs(v) for v in x[3:]) for x in filtered)
    if not (0.96<=gnorm<=1.04 and sig_norm<0.015 and maxgyro<2.0 and max(gs)<0.5):
        ap.error(f'Not stationary enough: |accel|={gnorm:.3f}g, sigma={sig_norm:.4f}g, maxgyro={maxgyro:.2f}dps, gyro sigma={gs}. Do not calibrate.')
    # Mirror EXACT equations in update_attitude() and original sensor mounting.
    accel_roll=math.degrees(math.atan2(ay,az))
    accel_pitch=math.degrees(math.atan2(-ax,math.sqrt(ay*ay+az*az)))
    print(f'Accepted {len(filtered)} stationary samples, accel norm {gnorm:.4f}g (sigma {sig_norm:.4f}g).')
    print(f'Measured sensor roll={accel_roll:+.4f} deg, sensor pitch={accel_pitch:+.4f} deg.')
    print('For a physically LEVEL airframe, proposed build-time trims:')
    print(f'#define IMU_ROLL_MOUNT_TRIM_DEG    {accel_roll:.5f}f')
    print(f'#define IMU_PITCH_MOUNT_TRIM_DEG   {accel_pitch:.5f}f')
    print('No calibration data is written unless both --confirm-level and --write-header are supplied.')
    if a.write_header:
        # Only calibration of physical mounting. Never change gyro bias, axis sign,
        # RC config, arming logic, PID gains or in-flight state.
        contents=(f'#ifndef UAV_LEVEL_TRIM_USER_H\n#define UAV_LEVEL_TRIM_USER_H\n'
                  f'/* Derived from {a.csv.name}, {len(filtered)} stationary samples; '
                  f'operator CONFIRMED airframe level. */\n'
                  f'#define IMU_ROLL_MOUNT_TRIM_DEG   {accel_roll:.5f}f\n'
                  f'#define IMU_PITCH_MOUNT_TRIM_DEG  {accel_pitch:.5f}f\n'
                  f'#endif\n')
        a.write_header.parent.mkdir(parents=True, exist_ok=True)
        if a.write_header.exists():
            a.write_header.with_suffix('.h.bak').write_text(a.write_header.read_text())
        a.write_header.write_text(contents)
        print('Updated:',a.write_header)

if __name__=='__main__': main()
