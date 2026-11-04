# UAV V3.3.5 — Experimental ARM + throttle/manual attitude control

**WARNING:** This is a user-requested experimental motor-enable firmware, NOT a validated stable-flight or hover firmware. No full STM32 build or flight test has been performed. DShot motor reaction torque, PID loaded stability, MC7RE failsafe failure modes, power/thermal margins remain unverified. A KILL switch is NOT proof that a multicopter cannot tip or strike a person.

## Files to copy into existing STM32CubeIDE project
- `Core/Src/uav_test.c` (experimental ARM and DShot mixer release)
- `Core/Src/flight_pid.c` (M1 front-left / M2 front-right / M3 rear-left / M4 rear-right mixer)
- `Core/Inc/flight_pid.h`
- `Core/Inc/rc_input.h`
- `Core/Inc/mc7re_guard.h`
No new C files, no extra main() or .ioc changes.

## Changes
- `UAV_BENCH_ARM_LOCK=0`: removing this alone does not enable motors; released the later `outputs_force_zero()` barrier under interlocks.
- Enable attitude PID Roll/Pitch and yaw-rate with **pilot controlled throttle**. Altitude hold DISABLED.
- Motor commands limited to `DSHOT <= 1000` per channel; this cap is not calibrated against 14A measured current and may not provide enough lift. Never increase it without an independently verified motor+prop+ESC rating and actual per-motor currents.
- THR slew-up reduced to `0.80` normalized ratio/s.
- ARM requires a fresh MPU6500, throttle endpoint calibration, 1s good SBUS, RC sticks neutral, craft near-level, gyro near stationary, throttle low held 500ms, ARM OFF followed by ARM ON, KILL OFF, no MC7RE latch, no prior DMA errors.
- KILL ON, inferred RF preset, lost-frame flag, missing SBUS or stale IMU result in immediate command zero / disarm in the loop. Receiver may mimic valid frames on RF loss; 300ms inferred preset is not guaranteed detection for all failsafe modes.
- Prevent automatic re-arming after fault until ARM switched OFF then ON and conditions satisfied; MC7RE latched event requires safe recovery (ARM OFF, KILL ON, low throttle, neutral sticks for 2 seconds).
- Log `LK=0 AR=1 SP=1 CAP=1000 O=...` shows armed, spinning and actual commanded DShot.

## Safety constraints before any propeller test
1. **First boot without propellers, ESC power disconnected**: verify `AR=0 SP=0 O=0/0/0/0`, KILL ON=`K=1800`, ARM OFF=`A=1000`, `L=1 FS=0 LAT=0` after link recovery. Confirm KILL ON causes `AR=0`.
2. **Unloaded bench with propellers removed**: confirm arming only with low throttle + neutral sticks and that motors stop on KILL, radio off, or IMU fault. This new firmware has NOT passed these tests even if earlier V3.3.2 did.
3. Do not use for uncontrolled free flight until verified RF failsafe, thrust and yaw torque with props, loaded stability/control, power/temperature limits and watchdog/system-level fault behavior. Do not test hovering on a tether that lets the airframe flip or swing.

## Critical hardware issue
M1 drew 14A continuously at 11.8V in prior testing, but the exact A2212 2200KV continuous-rated current and 8045 prop suitability were not provided. If each of 4 motors were simultaneously at 14A, the system could draw ~56A (plus electronics), i.e. ~661W at 11.8V; this is an arithmetic estimate, not a test result. A 40A ESC label does NOT define motor current rating. Battery, PDB, connectors, wires and cooling must all be rated for expected current.

## Compile / validation status
Checked: local source transformations and GCC syntax against a stubbed HAL in this workspace if noted in assistant message. Not checked: STM32CubeIDE ARM build or on-board tests.
