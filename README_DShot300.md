# UAV STM32G474RE — DShot300 conversion

This source set converts the former 4-channel 400 Hz PWM test output into **unidirectional DShot300** using TIM3 and four DMA streams.

## Output pins

- TIM3_CH1: PA6
- TIM3_CH2: PA4
- TIM3_CH3: PB0
- TIM3_CH4: PB1

The code assumes these pins are wired to DShot-capable ESC signal inputs and share a suitable ground. It does not implement bidirectional DShot or ESC telemetry.

## What changed

- `uav_test.c`: DShot packet and inverted 4-bit checksum generation; DShot300 pulse widths; one `TIM3_UP` DMA stream with timer DMA-burst writing CCR1–CCR4 per bit period; 1 kHz motor-control scheduling; log values now display DShot throttle commands.
- `UAV_.ioc`: TIM3 prescaler and period changed to 0 and 566 for the DShot300 symbol clock; the single `TIM3_UP` DMA mapping is retained.
- `main.c` and `UAV_.ioc`: TIM3 timing changed to PSC=0 / ARR=566 (nominal 299.8 kbit/s at a 170 MHz timer clock).
- `stm32g4xx_it.c`: the original DMA1 channel 1 handler is sufficient because the burst implementation uses one DMA stream.

This revision uses one DMA stream on `TIM3_UP` plus the timer's DMA-burst engine (`TIM3_DCR`/`TIM3_DMAR`) to update CCR1–CCR4 as a synchronized four-value group. This avoids assigning the same DMAMUX request to four separate DMA channels, which ST RM0440 warns can cause unpredictable behavior when those channels are requested together. The original `.ioc` route for one `TIM3_UP` DMA channel is therefore intentional.

## Important notes

- The uploaded files are source files only, not the full STM32CubeIDE project with HAL drivers, startup assembly, linker script, and build metadata. This package cannot be treated as a fully built `.elf` until placed back into the original project.
- I have not run a hardware test or verified the resulting waveform with an oscilloscope/logic analyzer. This revision removes the duplicated-DMAMUX-request flaw from the earlier draft and uses a single burst stream, but still requires bench verification. Please treat this as a source conversion draft and validate the DShot pulse timing and DMA alignment on the bench before connecting motor power.
- Remove all propellers. First verify each pin produces a DShot300 frame and that ESCs report valid signal with throttle command `0`. Confirm receiver channel map and the active-high KILL switch before enabling motors.
- DShot command `0` is sent when unarmed, throttle is at zero, SBUS is stale/lost/failsafe, or KILL is active. The existing arm sequence and throttle filter are retained.
- Confirm the ESC firmware supports DShot300. Some ESCs require protocol selection or explicit DShot activation in their configurator.
- The existing SBUS configuration, including RX inversion, and MPU6500 readout are otherwise retained. Gyro calibration is moved to run after successful IMU initialization.
