#!/usr/bin/env python3
"""Diagnose R5.5 IMU1000 CSV without installing scipy/numpy.

Use measured DWT-derived time deltas. MPU6500's current 41-Hz DLPF is NOT
bypassed, so peaks above ~35 Hz cannot establish an unfiltered motor spectrum.
"""
import argparse, csv, math, statistics
from pathlib import Path

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('csv',type=Path)
    a=ap.parse_args()
    with a.csv.open(newline='',encoding='utf-8-sig') as fp:
        rows=list(csv.DictReader(fp))
    captures={}
    for r in rows:
        try:
            cid=(int(r['capture_id']),r['capture_kind'])
            if r['relative_us']=='': continue
            t=float(r['relative_us'])/1e6
            values=[float(r[k]) for k in ('accel_x_g','accel_y_g','accel_z_g',
                                          'gyro_x_dps_uncal','gyro_y_dps_uncal','gyro_z_dps_uncal')]
            captures.setdefault(cid,[]).append((t,*values,int(r['capture_index']),int(r['capture_total'])))
        except (KeyError,ValueError): continue
    if not captures: ap.error('No valid samples in CSV')
    for (cid,kind),samples in captures.items():
        samples.sort(key=lambda x:x[-2]); n=len(samples)
        if n<100:
            print(f'Capture {cid}/{kind}: only {n} samples (need >=100)')
            continue
        times=[r[0] for r in samples]
        deltas=[(b-a)*1000 for a,b in zip(times,times[1:]) if b>a]
        idxs=[r[-2] for r in samples]
        lost=max(0,max(idxs)-min(idxs)+1-n)
        print(f'\nCapture {cid} [{kind}] {n}/{samples[0][-1]} samples, estimated missing={lost}')
        if deltas:
            print(f'Actual interval median={statistics.median(deltas):.3f} ms, '
                  f'p95={sorted(deltas)[int(.95*(len(deltas)-1))]:.3f} ms; '
                  f'observed sample rate ~{1000/statistics.median(deltas):.1f} Hz')
        for i,label in enumerate(('Accel X','Accel Y','Accel Z','Gyro X','Gyro Y','Gyro Z'),start=1):
            seq=[r[i] for r in samples]
            print(f'{label:9s} mean={statistics.mean(seq):+9.4f} '
                  f'sigma={statistics.pstdev(seq):7.4f} '
                  f'peak-to-peak={max(seq)-min(seq):7.3f} '
                  f'{"g" if i<=3 else "dps"}')
        # Spectrum only if near-uniform sampling and sufficiently complete.
        if n>=500 and lost<max(15,int(n*.03)) and deltas and max(deltas)<2.5:
            # DFT: remove mean; project onto Fourier basis 2..35 Hz.
            for i,label in [(4,'Gyro X'),(5,'Gyro Y'),(6,'Gyro Z')]:
                values=[r[i] for r in samples]
                mean=statistics.mean(values)
                energies=[]
                for freq in range(2,36):
                    re=im=0.0
                    for r,v in zip(samples,values):
                        angle=2*math.pi*freq*r[0]
                        re+=(v-mean)*math.cos(angle)
                        im+=(v-mean)*math.sin(angle)
                    energies.append((math.hypot(re,im)*2.0/n,freq))
                amp,freq=max(energies)
                print(f'{label} strongest 2..35 Hz component: {freq} Hz, ~{amp:.3f} dps amplitude')
        else:
            print('Not enough uniformly spaced samples for a meaningful spectrum.')
        print('NOTE: MPU6500 gyro/accel DLPF is still ~41 Hz; do not infer peaks above its passband.')

if __name__=='__main__': main()
