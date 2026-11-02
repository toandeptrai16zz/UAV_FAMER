# UAV V3.3.3 — PASSIVE BODY-AXIS CHECK (MPU6500 Roll/Pitch alignment)

**THIS IS NOT A FLIGHT FIRMWARE. REMOVE ALL FOUR PROPELLERS; DISCONNECT ESC POWER.**

## Reason for the patch: four physical attitude tests

The user's V3.3.2 log showed:

| Actual movement | Previous R | Previous P | Conclusion |
|:--|--:|--:|:--|
| Lean right | approx -3.5 deg | approx -20.5 deg | primarily old P |
| Lean left | approx +3.5 deg | approx +28 deg | primarily old P |
| Nose down | approx +22 deg | approx +1.4 deg | primarily old R |
| Nose up | approx -25 deg | approx -2.2 deg | primarily old R |

The apparent body-axis mapping **to verify** is:

```
body_roll_deg  = -(sensor_pitch_deg - sensor_pitch_trim_deg);
body_pitch_deg = -(sensor_roll_deg  - sensor_roll_trim_deg);
body_roll_rate_dps  = -sensor_gyro_y_dps;
body_pitch_rate_dps = -sensor_gyro_x_dps;
body_yaw_rate_dps   = -sensor_gyro_z_dps; // preserved V3.2.8 convention
```

*Raw sensor `S=` remains unchanged.* `R=` and `P=` are now adjusted BODY angles and `G=` is adjusted BODY rate (x10 dps). This is a provisional local-angle transform; proper full IMU mounting, 3D geometry and closed-loop response remain to be verified before flight.

## Files

Copy `Core/Src/uav_test.c` and `Core/Inc/mc7re_guard.h` to your STM32CubeIDE project (same paths). Do **not** add any C test file into Core/Src. Retain your working `Core/Inc/rc_input.h` with centers `952/1062/1099`. Also retain the **V3.2.7 corrected `Core/Src/flight_pid.c`** matching M1/front-left, M2/front-right, M3/rear-left, M4/rear-right; DO NOT restore the old V3.2 mixer. The previous serial MIX outputs confirm your existing mixer is updated. Clean / Build / Flash. No `.ioc` changes.

**Hardware output safety:** `UAV_BENCH_ARM_LOCK=1`, `UAV_PROPS_OFF_ESC_TEST=0`; `O=0/0/0/0`, `B=0`, `LK=1` at all times. Unlike V3.3.2, **the four-motor timed scan is disabled**. Do NOT attach props and do not connect ESC power for this check.

## How to test on serial at 115200 baud

1. Power STM32 and receiver only. Transmitter ON, ARM OFF (`A~1000`), KILL ON (`K~1800`), throttle MIN (`T~40`), and Roll/Pitch/Yaw sticks centered (`N=1`). Wait for `LAT=0 L=1`.
2. Lay frame level: expect `R`, `P` near 0 centidegrees (within roughly 30-50 counts on your previous floor readings); `O=0/0/0/0 LK=1 B=0`.
3. **Keep sticks centered and tilt the FRAME**, one axis at a time, 10-15 degrees:
   - Right lean (right side lowered): `R` POSITIVE (approximately +1000..+1500), `P` small; `PID_R` NEGATIVE, Mixer M2/M4 > M1/M3.
   - Left lean: `R` NEGATIVE, `P` small; `PID_R` POSITIVE, Mixer M1/M3 > M2/M4.
   - Nose DOWN: `P` NEGATIVE (approximately -1000..-1500), `R` small; `PID_P` POSITIVE, Mixer M1/M2 > M3/M4.
   - Nose UP: `P` POSITIVE, `R` small; `PID_P` NEGATIVE, Mixer M3/M4 > M1/M2.
   `PID` and `MIX` are theoretical/logged only; they NEVER command motors on this version.
4. Return upright and stop rotating: R/P near 0, G=near zero. If any axis/sign differs, do NOT continue toward flight.
5. Optionally examine gyro sign **during the motion**: body G_R should be positive while leaning right, G_P negative while lowering nose. Holding the tilted frame steady normally gives G~0.

## Safety limits

- A software inferred preset guard for MC7RE is not a reliable independent RF-link indicator; changing receiver preset could defeat it. Test failsafe under the intended receiver configuration.
- Motor stop on V3.3.2 was observed, but real KILL reaction latency and operating thrust behavior are not qualified.
- Passive motor mixer signs cannot prove actual torque direction, control stability, battery power margin, calibration, or prop compatibility.
- IMU orientation can affect yaw/cross-axis coupling; this mapping is only empirically derived from small manually induced tilts. Before flying you must verify physical sensor mounting axes and use appropriate complete attitude-estimation/motor feedback tests under qualified restraint.

No firmware flight release/arming unlock is included.

## Verification performed before packaging

- Host GCC syntax-check of `uav_test.c` against STM32G474/CMSIS/HAL headers: PASS (only a 64-bit host warning from the existing HAL timer macro).
- Compiled actual V3.2.7 `flight_pid.c` on host and tested the four V3.3.2 sample angles through this provisional axis transform: PASS in each direction.
- This was NOT a full ARM toolchain build and has NOT been run on the STM32 or tested under propeller thrust.
