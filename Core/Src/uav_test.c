/*
 * uav_test.c
 * SBUS -> 4-channel DShot300 (TIM3 DMA-burst) + MPU6500 attitude PID + MT06 Alt Hold
 */

#include "main.h"
#include "uav_test.h"
#include "dshot_protocol.h"
#include "dps310.h"
#include "mt06.h"
#include "flight_pid.h"
#include "rc_input.h"
#include "mc7re_guard.h"
#include "uav_telemetry.h"
#include "uav_rate_blackbox.h"
#include "uav_imu_raw.h"
#include "uav_level_trim_user.h"
#include "uav_cage_guard.h"
#include "uav_config.h"
#include "uav_config_gate.h"

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef hlpuart1;
extern TIM_HandleTypeDef htim3;
extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_tim3_up;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;

/* ---------------------------- DShot config ---------------------------- */
#define DSHOT_TIMER_PERIOD_TICKS    567u  
#define DSHOT_T0H_TICKS             213u  
#define DSHOT_T1H_TICKS             425u  
#define DSHOT_PACKET_BITS           16u
#define DSHOT_TRAILING_CELLS        1u
#define DSHOT_DMA_CELLS             (DSHOT_PACKET_BITS + DSHOT_TRAILING_CELLS)
#define DSHOT_CONTROL_PERIOD_MS     1u
#define DSHOT_THROTTLE_MIN          48u
#define DSHOT_THROTTLE_MAX          2047u
#define PWM_OUTPUT_COUNT            4u
#define TEST_MOTOR_MASK             0x0Fu

/* V340: direct pilot throttle: no artificial smoothing or slow-up ramp.
 * Keep normal DShot output bounds (0=stop, 48..2047=throttle) and safety gates. */

/* ----------------------------- SBUS config ---------------------------- */
#define SBUS_FRAME_SIZE             25u
#define SBUS_RX_BUFFER_SIZE         64u
#define SBUS_START_BYTE             0x0Fu
#define SBUS_LINK_TIMEOUT_MS        100u
#define SBUS_FLAG_FRAME_LOST        0x04u
#define SBUS_FLAG_FAILSAFE          0x08u

#define SBUS_THR_LOW_RAW             RC_THR_MIN_RAW
#define SBUS_THR_HIGH_RAW            RC_THR_MAX_RAW
#define SBUS_THR_LOW_FRACTION_PCT    RC_LOW_PERCENT
#define SBUS_STICK_MIN_RAW           RC_STICK_MIN
#define SBUS_STICK_MID_RAW           RC_PITCH_CENTER
#define SBUS_STICK_MAX_RAW           RC_STICK_MAX

/* Zero-based SBUS channel indexes. */
#define CH_ROLL                      0u
#define CH_PITCH                     1u
#define CH_THR                       2u
#define CH_YAW                       3u
#define CH_ARM                       4u
#define CH_KILL                      5u
/* The real MC6C transmitter has only six physical channels (CH1..CH6).
 * CH5 = ARM, CH6 = KILL. CH7/CH8 cannot be required for normal operation.
 * Change to 1 only if an eight-channel transmitter with a real, mapped
 * CH8 CONFIG switch is installed and validated. */
#define UAV_CONFIG_REQUIRE_CH8         0u
#define CH_CONFIG                     7u /* SBUS index only, when above is 1 */
#define CONFIG_SWITCH_THRESHOLD       1500u
#define CH_ALT_HOLD                  6u  /* [THÊM MỚI] Kênh 6 dùng để bật bám ngọn cây */

/* Switch thresholds */
#define ARM_SWITCH_THRESHOLD         1320u
#define ARM_SWITCH_ACTIVE_HIGH       1u
#define KILL_SWITCH_THRESHOLD        1500u
#define KILL_SWITCH_ACTIVE_HIGH      1u
#define ALT_HOLD_THRESHOLD           1500u /* [THÊM MỚI] Gạt công tắc kênh 6 lên > 1500 là BẬT */

/* ----------------------------- Arm safety ----------------------------- */
#define THR_DEADBAND                40.0f
#define THR_START_HYST              5.0f
#define THR_STOP_HYST               15.0f
#define THR_STABLE_BAND             30u
#define ESC_INIT_MS                 2500u
#define ARM_LOW_HOLD_MS             500u
#define LINK_STABLE_MS              1000u
/* When the 5 Hz ASCII report shares LPUART1 with binary telemetry,
 * its long interrupt-driven TX occupies 2-3 of every 10 flight TX slots.
 * This was measured as exactly two missing sequence numbers every 200 ms.
 * Keep startup/critical ASCII diagnostics, disable periodic ASCII by default.
 * 0 = reliable 50 Hz UDP telemetry; 1 = legacy ASCII diagnostics (not both).
 */
#ifndef UAV_SHARED_UART_PERIODIC_ASCII
#define UAV_SHARED_UART_PERIODIC_ASCII 0u
#endif
#define LOG_PERIOD_MS                200u
#define IMU_SAMPLE_PERIOD_MS        1u
#define IMU_MAX_AGE_MS               20u
#define SPI_TIMEOUT_MS              2u

/* V334 yaw-response sign check (passive, no motor output). */
#define YAW_CHECK_RATE_MIN_DPS   20.0f
#define YAW_CHECK_RATE_MAX_DPS  180.0f
#define YAW_CHECK_COUNT_TARGET  30u
/* V333 body-axis check: empirical sign map from the four hand-tilt logs.
 * Right lean => old sensor Pitch negative; nose down => old sensor Roll positive.
 * This is a provisional map, NOT a fully verified IMU mounting rotation or
 * qualification of rate feedback/motor torque. Only passive outputs allowed. */
/* Candidate attitude controller. Gains are initial/unvalidated and require controlled response checks. */
#define ENABLE_ATTITUDE_PID          1u
/* 2026-11-05 re-mounted IMU: static log uav_20261105_114917(1).csv,
 * props OFF, mean roll=-1.2988, pitch=-2.8103 degrees, 2283 records.
 * Controller mapping below: roll = -sensor_pitch + PITCH_TRIM;
 *                            pitch = -sensor_roll + ROLL_TRIM.
 * Compensate the measured static errors relative to the former mount:
 * new PITCH_TRIM = -1.06 + 1.2988 ~= +0.24 deg
 * new ROLL_TRIM  = -2.13 + 2.8103 ~= +0.68 deg.
 * Valid ONLY if the static test surface represents the intended level plane.
 * This is a one-off offset, not an in-flight auto-level algorithm; verify the
 * angle display on a known-level surface before allowing prop-on operation. */
#ifndef IMU_ROLL_MOUNT_TRIM_DEG
#define IMU_ROLL_MOUNT_TRIM_DEG      0.68f
#endif
#ifndef IMU_PITCH_MOUNT_TRIM_DEG
#define IMU_PITCH_MOUNT_TRIM_DEG     0.24f
#endif
/* R5.7 restricted-area flight trial: removes the old 5.5-second cage timer,
 * automatic cage tilt/yaw/rate trips and temporary DShot 1550 ceiling.
 * DOES NOT bypass manual KILL, SBUS loss/failsafe, IMU validity, DShot DMA
 * interlocks, low-throttle pre-ARM or ARM switch edge requirements.
 * Verify MOTOR ORDER, prop orientation and independent ESC power cutoff. */
#ifndef UAV_CAGE_TEST_ARM_ENABLE
#define UAV_CAGE_TEST_ARM_ENABLE       0u
#endif
/* The legacy flight candidate ARM machine remains underneath the new guard. */
#define UAV_EXPERIMENTAL_ARM_ENABLE  1u
#define UAV_EXPERIMENTAL_MAX_DSHOT   (UAV_CAGE_TEST_ARM_ENABLE ? 1550u : DSHOT_THROTTLE_MAX)
#define UAV_ARM_MAX_ANGLE_DEG       8.0f
#define UAV_ARM_MAX_RATE_DPS       15.0f
/* Use already measured MC6C SBUS endpoints (40..1640) from rc_input.h.
 * No full-throttle calibration sweep at every power-on. */
#define UAV_THROTTLE_MAPPING_VERIFIED 1u
/* Re-arm only after 600 completed, uninterrupted DShot DMA frames (~0.6 s).
 * A historical TX error counter is NOT a permanent lockout. */
#define UAV_DSHOT_REARM_GOOD_FRAMES   600u
/* Flight candidate: normal pilot ARM is enabled; bench lock is disabled. */
#define UAV_BENCH_ARM_LOCK           0u
/* Sequential props-off ESC scanner is disabled in this flight candidate. */
#define UAV_PROPS_OFF_ESC_TEST        0u
/* One channel at a time, never two simultaneously. REMOVE ALL PROPELLERS. */
#define UAV_BENCH_TEST_MOTOR_COUNT    4u
#define UAV_BENCH_TEST_GAP_MS         500u
/* Two bounded, low-throttle stages per output. */
#define UAV_BENCH_TEST_STAGE1_DSHOT  120u
#define UAV_BENCH_TEST_STAGE2_DSHOT  180u
#define UAV_BENCH_TEST_STAGE1_MS     600u
#define UAV_BENCH_TEST_STAGE2_MS     900u
#define UAV_BENCH_TEST_MOTOR_WINDOW_MS (UAV_BENCH_TEST_STAGE1_MS + UAV_BENCH_TEST_STAGE2_MS)
#define UAV_BENCH_TEST_SEGMENT_MS   (UAV_BENCH_TEST_MOTOR_WINDOW_MS + UAV_BENCH_TEST_GAP_MS)
#define UAV_BENCH_TEST_TOTAL_MS     ((UAV_BENCH_TEST_MOTOR_COUNT * UAV_BENCH_TEST_MOTOR_WINDOW_MS) + \
                                     ((UAV_BENCH_TEST_MOTOR_COUNT - 1u) * UAV_BENCH_TEST_GAP_MS))
#define UAV_BENCH_MIN_DMA_FRAMES    4500u
#define UAV_BENCH_PREP_HOLD_MS       1000u
#if (UAV_BENCH_TEST_STAGE1_DSHOT < DSHOT_THROTTLE_MIN) || (UAV_BENCH_TEST_STAGE2_DSHOT > 180u)
#error "Bench test DShot range out of safety limits"
#endif
#if UAV_BENCH_TEST_MOTOR_COUNT != PWM_OUTPUT_COUNT
#error "Bench channel scan must cover exactly four TIM3 outputs"
#endif
#if UAV_PROPS_OFF_ESC_TEST && !UAV_BENCH_ARM_LOCK
#error "PROPS-OFF test and unlocked flight ARM must never coexist"
#endif
/* Exactly one output mode may be selected. The diagnostic package defaults
 * to the propeller-off scanner; flight ARM requires a separate reviewed build. */
#if UAV_PROPS_OFF_ESC_TEST && (!UAV_BENCH_ARM_LOCK || UAV_EXPERIMENTAL_ARM_ENABLE)
#error "Propeller-off scan requires pilot ARM locked and experimental ARM disabled"
#endif
#if (!UAV_PROPS_OFF_ESC_TEST) && (UAV_BENCH_ARM_LOCK || !UAV_EXPERIMENTAL_ARM_ENABLE)
#error "No valid motor output mode selected"
#endif
#if (UAV_EXPERIMENTAL_MAX_DSHOT <= DSHOT_THROTTLE_MIN) || (UAV_EXPERIMENTAL_MAX_DSHOT > DSHOT_THROTTLE_MAX)
#error "Invalid experiment output cap"
#endif
#define UAV_ENABLE_ALT_HOLD          0u
#if UAV_ENABLE_ALT_HOLD
#error "Altitude hold needs verified hover thrust, sample-timed derivative and failsafe before enabling."
#endif
#define PID_ANGLE_LIMIT_DEG          25.0f
#define PID_YAW_RATE_LIMIT_DPS       90.0f
/* MPU6500 FS_SEL=3: +/-2000 dps, 16.4 LSB/(deg/s).
 * Prior +/-500 dps (65.5 LSB/dps) clipped repeatedly at 501.1 deg/s
 * in uav_20261116_163935.csv; clipped gyro invalidates rate PID feedback.
 * Updating the register AND scale is mandatory. This fixes measurement
 * saturation, NOT the as-yet-unverified physical yaw torque response. */
#define MPU_GYRO_CONFIG_REG           0x1Bu
#define MPU_GYRO_CONFIG_2000DPS       0x18u
#define GYRO_LSB_PER_DPS              16.4f
#define GYRO_CAL_STILLNESS_MAX_DPS    4.0f
/* Important: gyro Z mounting sign is a PHYSICAL convention. With no motors,
 * nose clockwise as seen from above must report POSITIVE gyro_yaw_dps.
 * Do not change without physically confirming the sensor's orientation. */
#define UAV_BODY_YAW_SENSOR_SIGN      (-1.0f)
/* This release uses the standard pilot-ARM state machine, not R4's
 * diagnostic motor-output lock. Safety interlocks above remain active. */
#ifndef UAV_YAW_DIAGNOSTIC_ARM_LOCK
#define UAV_YAW_DIAGNOSTIC_ARM_LOCK    0u
#endif
#if UAV_CAGE_TEST_ARM_ENABLE && UAV_YAW_DIAGNOSTIC_ARM_LOCK
#error "CAGE ARM requested but diagnostic lock remains enabled"
#endif
#if UAV_CAGE_TEST_ARM_ENABLE && (UAV_PROPS_OFF_ESC_TEST || UAV_BENCH_ARM_LOCK || !UAV_EXPERIMENTAL_ARM_ENABLE)
#error "CAGE mode conflicts with flight/bench output guards"
#endif
#if UAV_YAW_DIAGNOSTIC_ARM_LOCK && UAV_PROPS_OFF_ESC_TEST
#error "Yaw diagnostic firmware must not run the motor scanner"
#endif
/* Original 0.98 per 1 ms = ~49.5 ms time constant: accelerometer
 * transients at high throttle can be mistaken for 10-20 degree tilt.
 * Use a provisional 0.5 s time constant, and downweight accelerometer
 * whenever the measured total acceleration differs from ~1g.
 * MUST be validated on the actual airframe before flight; do not
 * compensate hardware vibration problems with PID gains. */
#define COMP_FILTER_TAU_S            0.50f
#define ACCEL_TRUST_FULL_ERR_G        0.10f
#define ACCEL_TRUST_ZERO_ERR_G        0.30f

/* [THÊM MỚI] PID Độ cao (Bám ngọn cây) */
#define ALT_TARGET_CM                100.0f  /* Mục tiêu bay cách ngọn cây 1 mét */
#define PID_ALT_KP                   2.5f    /* Tỉ lệ */
#define PID_ALT_KI                   0.2f    /* Tích phân */
#define PID_ALT_KD                   1.2f    /* Vi phân */
#define ALT_HOVER_THROTTLE           1100.0f /* [QUAN TRỌNG] Bạn phải thay đổi số này bằng mức ga treo thực tế của Drone! */

#define CS_LOW()  HAL_GPIO_WritePin(IMU_GPIO_Port, IMU_Pin, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(IMU_GPIO_Port, IMU_Pin, GPIO_PIN_SET)

/* --------------------------- DShot DMA state -------------------------- */
static uint16_t dshot_dma_buffer[DSHOT_DMA_CELLS][PWM_OUTPUT_COUNT];
static volatile uint32_t dshot_tx_errors;
static volatile uint32_t dshot_tx_done;
static volatile uint8_t dshot_dma_error_pending;
/* Count completed TX callbacks since the most recent error, not since boot.
 * Keep the cumulative error counter for diagnosis; never clear it to fake health. */
static uint32_t dshot_faults_observed;
static uint32_t dshot_tx_done_at_fault;
static bool dshot_tx_recovered;

static void dshot_set_compare_zero(void);

/* Match the user's previously functioning DShot DMA completion path.
 * Do NOT zero the CCR preload registers in this callback: the known-old
 * driver only disables update DMA here. The 17th DMA cell is already low.
 */
static void dshot_dma_tx_complete(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    __HAL_TIM_DISABLE_DMA(&htim3, TIM_DMA_UPDATE);
    dshot_tx_done++;
}

static void dshot_dma_tx_error(DMA_HandleTypeDef *hdma)
{
    (void)hdma; __HAL_TIM_DISABLE_DMA(&htim3, TIM_DMA_UPDATE);
    dshot_set_compare_zero(); dshot_tx_errors++; dshot_dma_error_pending = 1u;
}

/* ------------------------ Bench UART logging ------------------------- */
/* One owner, one persistent TX buffer. Do not reuse buffer until HAL reports
 * that the complete frame has left LPUART1. Never block the 1-ms motor loop. */
static char log_buffer[896];
static volatile uint8_t log_tx_inflight = 0u;
static volatile uint32_t log_tx_done = 0u;
static volatile uint32_t log_tx_dropped = 0u;
static volatile uint32_t log_tx_errors = 0u;
static uint32_t log_sequence = 0u;

static void log_async(const char *fmt, ...)
{
    int n;
    size_t length;
    va_list ap;
    if (log_tx_inflight || hlpuart1.gState != HAL_UART_STATE_READY) {
        log_tx_dropped++;
        return;
    }
    va_start(ap, fmt);
    n = vsnprintf(log_buffer, sizeof(log_buffer), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    length = (size_t)n;
    if (length >= sizeof(log_buffer)) length = sizeof(log_buffer) - 1u;
    log_tx_inflight = 1u;
    if (HAL_UART_Transmit_IT(&hlpuart1, (uint8_t *)log_buffer, (uint16_t)length) != HAL_OK) {
        log_tx_errors++;
        log_tx_inflight = 0u;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    UAVTelemetry_OnTxComplete(huart);
    UAVConfig_OnTxComplete(huart);
    if (huart == &hlpuart1 && log_tx_inflight) {
        log_tx_done++;
        log_tx_inflight = 0u;
    }
}

/* ----------------------------- MPU6500 & Sensors -------------------------------- */
static int16_t accel_x, accel_y, accel_z;
static int16_t gyro_x, gyro_y, gyro_z;
static bool imu_ready, imu_calibrated;
static bool imu_raw_capture_ready;
/* A successful SPI transfer alone does not prove attitude continuity. */
static bool imu_control_valid;
static uint32_t imu_gap_count, imu_dt_max_ms, motor_dt_max_ms, motor_late_count;
static DPS310_HandleTypeDef barometer;
static bool barometer_ready;
static MT06_HandleTypeDef rangefinder;
static bool rangefinder_ready;
static uint32_t imu_read_errors, imu_last_good_ms, imu_sample_sequence, pid_last_sample_sequence;

static int32_t gyro_bias_x_raw, gyro_bias_y_raw, gyro_bias_z_raw;
static float roll_angle = 0.0f, pitch_angle = 0.0f;
/* Base attitude stays in gravity/physical-mount coordinates. Only the
 * angle controller and displayed roll/pitch are zeroed to a pilot-chosen
 * disarmed reference pose; gyro bias and body rates are never touched. */
static float control_roll_angle(void) { return roll_angle - UAVConfig_LevelRoll(); }
static float control_pitch_angle(void) { return pitch_angle - UAVConfig_LevelPitch(); }
static float sensor_roll_angle = 0.0f, sensor_pitch_angle = 0.0f;
static bool attitude_initialized = false;
static FlightPIDState flight_pid;
static uint32_t last_pid_ms = 0, last_attitude_update_ms = 0;
/* High-rate blackbox timing: DWT enables microsecond-scale loop diagnostics. */
static uint32_t bb_cycles_per_us = 1u;
static uint32_t bb_pid_dt_us = 0u;
static uint16_t bb_motor_interval_us = 0u;
static uint32_t bb_prev_motor_cycle = 0u;
static uint32_t bb_last_sample_ms = 0u;
static uint32_t bb_last_imu_seq = 0u;
static uint32_t bb_last_tx_ms = 0u;

/* [THÊM MỚI] Biến toàn cục lưu trữ PID độ cao */
static float alt_integral = 0.0f;
static float alt_last_error = 0.0f;
static uint16_t current_vertical_range_cm = 0;
static bool current_range_valid = false;

static HAL_StatusTypeDef mpu_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {(uint8_t)(reg & 0x7Fu), value}; HAL_StatusTypeDef status;
    CS_LOW(); status = HAL_SPI_Transmit(&hspi2, tx, sizeof(tx), SPI_TIMEOUT_MS); CS_HIGH();
    return status;
}

static HAL_StatusTypeDef mpu_read_regs(uint8_t reg, uint8_t *buffer, uint16_t len)
{
    HAL_StatusTypeDef status; uint8_t address = (uint8_t)(reg | 0x80u); uint8_t dummy_tx[14] = {0u};
    if ((buffer == NULL) || (len == 0u) || (len > sizeof(dummy_tx))) return HAL_ERROR;
    CS_LOW(); status = HAL_SPI_Transmit(&hspi2, &address, 1u, SPI_TIMEOUT_MS);
    if (status == HAL_OK) status = HAL_SPI_TransmitReceive(&hspi2, dummy_tx, buffer, len, SPI_TIMEOUT_MS);
    CS_HIGH(); return status;
}

static bool mpu6500_init_spi(void)
{
    uint8_t who_am_i = 0u; CS_HIGH(); HAL_Delay(10u);
    if (mpu_read_regs(0x75u, &who_am_i, 1u) != HAL_OK) return false;
    if (who_am_i != 0x70u) return false;
    if (mpu_write_reg(0x6Bu, 0x80u) != HAL_OK) return false;
    HAL_Delay(100u);
    if (mpu_write_reg(0x6Bu, 0x01u) != HAL_OK) return false;
    if (mpu_write_reg(0x19u, 0x00u) != HAL_OK) return false;
    if (mpu_write_reg(0x1Au, 0x03u) != HAL_OK) return false;
    if (mpu_write_reg(MPU_GYRO_CONFIG_REG, MPU_GYRO_CONFIG_2000DPS) != HAL_OK) return false;
    /* Verify the actual silicon accepted the full-scale setting; do not
     * silently run a 16.4 divisor when the gyro is still at +/-500. */
    uint8_t gyro_cfg_readback = 0u;
    if (mpu_read_regs(MPU_GYRO_CONFIG_REG, &gyro_cfg_readback, 1u) != HAL_OK ||
        (gyro_cfg_readback & 0x18u) != MPU_GYRO_CONFIG_2000DPS) return false;
    if (mpu_write_reg(0x1Cu, 0x08u) != HAL_OK) return false;
    /* ACCEL_CONFIG2: 41 Hz DLPF, 1 kHz output (TDK register map). */
    if (mpu_write_reg(0x1Du, 0x03u) != HAL_OK) return false;
    HAL_Delay(10u); return true;
}

static bool mpu6500_read_spi(void)
{
    uint8_t buffer[14];
    if (mpu_read_regs(0x3Bu, buffer, sizeof(buffer)) != HAL_OK) { imu_read_errors++; return false; }
    accel_x = (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
    accel_y = (int16_t)(((uint16_t)buffer[2] << 8) | buffer[3]);
    accel_z = (int16_t)(((uint16_t)buffer[4] << 8) | buffer[5]);
    gyro_x  = (int16_t)(((uint16_t)buffer[8]  << 8) | buffer[9]);
    gyro_y  = (int16_t)(((uint16_t)buffer[10] << 8) | buffer[11]);
    gyro_z  = (int16_t)(((uint16_t)buffer[12] << 8) | buffer[13]);
    imu_last_good_ms = HAL_GetTick(); imu_sample_sequence++;
    /* Capture raw register values BEFORE scale/bias/attitude calculations.
     * No serial I/O, heap allocation, floating point or filtering here. */
    if (imu_raw_capture_ready) {
        UAVIMURawSample rs = {0};
        rs.tick_ms = imu_last_good_ms;
        rs.cycle_count = DWT->CYCCNT;
        rs.accel[0] = accel_x; rs.accel[1] = accel_y; rs.accel[2] = accel_z;
        rs.gyro[0] = gyro_x; rs.gyro[1] = gyro_y; rs.gyro[2] = gyro_z;
        UAVIMURaw_Push(&rs);
    }
    return true;
}

/* Reject calibration while moving or with implausible acceleration. */
static uint8_t imu_cal_failure_reason = 0u; /* 1=read, 2=motion, 3=accel */
static bool mpu6500_calibrate_gyro(void)
{
    int64_t sum_x = 0, sum_y = 0, sum_z = 0;
    int32_t min_x = INT32_MAX, min_y = INT32_MAX, min_z = INT32_MAX;
    int32_t max_x = INT32_MIN, max_y = INT32_MIN, max_z = INT32_MIN;
    uint32_t valid_samples = 0u;
    imu_cal_failure_reason = 0u;
    HAL_Delay(2000u);
    for (uint32_t i = 0u; i < 1500u; ++i) {
        if (mpu6500_read_spi()) {
            int32_t gx = gyro_x, gy = gyro_y, gz = gyro_z;
            int64_t a2 = (int64_t)accel_x*accel_x + (int64_t)accel_y*accel_y + (int64_t)accel_z*accel_z;
            /* MPU6500 ACCEL_CONFIG=0x08 -> 8192 LSB/g. */
            if (a2 < (int64_t)6500*6500 || a2 > (int64_t)10000*10000) {
                imu_cal_failure_reason = 3u; return false;
            }
            if (gx < min_x) min_x = gx;
            if (gy < min_y) min_y = gy;
            if (gz < min_z) min_z = gz;
            if (gx > max_x) max_x = gx;
            if (gy > max_y) max_y = gy;
            if (gz > max_z) max_z = gz;
            sum_x += gx; sum_y += gy; sum_z += gz; valid_samples++;
        }
        HAL_Delay(2u);
    }
    if (valid_samples < 1425u) { imu_cal_failure_reason = 1u; return false; }
    /* The same +/-4 dps bench calibration tolerance in PHYSICAL units,
     * rescaled for FS_SEL=3. A stale threshold of 260 raw counts would now
     * accept ~16 dps motion and corrupt the gyro bias. */
    const int32_t max_raw_range = (int32_t)(GYRO_CAL_STILLNESS_MAX_DPS * GYRO_LSB_PER_DPS + 0.5f);
    if ((max_x-min_x > max_raw_range) || (max_y-min_y > max_raw_range) ||
        (max_z-min_z > max_raw_range)) {
        imu_cal_failure_reason = 2u; return false;
    }
    gyro_bias_x_raw = (int32_t)(sum_x / (int64_t)valid_samples);
    gyro_bias_y_raw = (int32_t)(sum_y / (int64_t)valid_samples);
    gyro_bias_z_raw = (int32_t)(sum_z / (int64_t)valid_samples);
    return true;
}

static float gyro_x_rate_dps(void) { return (float)(gyro_x - gyro_bias_x_raw) / GYRO_LSB_PER_DPS; }
static float gyro_y_rate_dps(void) { return (float)(gyro_y - gyro_bias_y_raw) / GYRO_LSB_PER_DPS; }
static float gyro_z_rate_dps(void) { return (float)(gyro_z - gyro_bias_z_raw) / GYRO_LSB_PER_DPS; }
/* Provisional IMU -> frame axis/sign transform from physical tilt logs.
 * Signed body rates must match signed body angles supplied to the PID.
 * Yaw sign was previously checked by manually rotating the airframe. */
static float body_roll_rate_dps(void)  { return -gyro_y_rate_dps(); }
static float body_pitch_rate_dps(void) { return -gyro_x_rate_dps(); }
static float body_yaw_rate_dps(void)   { return UAV_BODY_YAW_SENSOR_SIGN * gyro_z_rate_dps(); }

/* Accelerometer tilt is valid for gravity-dominated motion only.
 * A large norm departure is evidence of transient translational acceleration
 * or vibration: preserve the gyro integration and reject that tilt update.
 * NOTE: norm near 1g does not PROVE absence of non-gravity acceleration. */
static float accel_tilt_confidence(float ax, float ay, float az)
{
    const float norm_g = sqrtf(ax*ax + ay*ay + az*az) / 8192.0f;
    const float error_g = fabsf(norm_g - 1.0f);
    if (!isfinite(norm_g)) return 0.0f;
    if (error_g <= ACCEL_TRUST_FULL_ERR_G) return 1.0f;
    if (error_g >= ACCEL_TRUST_ZERO_ERR_G) return 0.0f;
    return (ACCEL_TRUST_ZERO_ERR_G - error_g) /
           (ACCEL_TRUST_ZERO_ERR_G - ACCEL_TRUST_FULL_ERR_G);
}

static void update_attitude(uint32_t dt_ms)
{
    const float dt = (float)dt_ms / 1000.0f;
    if (dt <= 0.0f || dt > 0.020f) return;
    const float ax = (float)accel_x, ay = (float)accel_y, az = (float)accel_z;
    const float confidence = accel_tilt_confidence(ax, ay, az);
    const float accel_roll = atan2f(ay, az) * 57.2957795f;
    const float accel_pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.2957795f;
    if (!attitude_initialized) {
        /* Don't initialise attitude from a severe impact or acceleration. */
        if (confidence < 0.80f) return;
        sensor_roll_angle = accel_roll;
        sensor_pitch_angle = accel_pitch;
        attitude_initialized = true;
    }
    /* dt-invariant complementary filter: 0.998 @ 1 ms, ~0.961 @ 20 ms
     * when acc is trustworthy; accel influence is reduced on vibration. */
    const float blend = (1.0f - expf(-dt / COMP_FILTER_TAU_S)) * confidence;
    /* The old filter integrated IMU X/Y independently and ignored body yaw.
     * That is not an Euler-attitude model: while yawing with nonzero roll,
     * body-axis pitch rate and pitch Euler rate are DIFFERENT.  The 16-Nov
     * log measured |yaw|>350 deg/s, so this omission is significant.
     *
     * Use the existing, field-checked frame convention:
     *   roll  = pitch_mount_trim - sensor_pitch
     *   pitch = roll_mount_trim  - sensor_roll
     *   p/q/r = body_roll/pitch/yaw_rate_dps(), with +yaw clockwise.
     * Integrate rigid-body 3-2-1 Euler kinematics in the AIRFRAME axes,
     * then blend with the existing acceleration-based level reference.
     * No MPU6500 axis/trim calibration or external telemetry sign changes.
     * Attitude is used only inside the guarded existing control loop.
     */
    const float rad = 0.01745329251994329577f;
    const float old_roll = IMU_PITCH_MOUNT_TRIM_DEG - sensor_pitch_angle;
    const float old_pitch = IMU_ROLL_MOUNT_TRIM_DEG - sensor_roll_angle;
    const float phi = old_roll * rad;
    const float theta = old_pitch * rad;
    const float p = body_roll_rate_dps();
    const float q = body_pitch_rate_dps();
    const float r = body_yaw_rate_dps();
    /* Keep tan(theta) bounded if tilt approaches the Euler singularity.
     * Normal arming already requires <8 degrees of tilt. */
    const float bounded_pitch = fmaxf(-1.3962634f, fminf(1.3962634f, theta));
    const float sin_phi = sinf(phi), cos_phi = cosf(phi);
    const float roll_dot = p + (q * sin_phi + r * cos_phi) * tanf(bounded_pitch);
    const float pitch_dot = q * cos_phi - r * sin_phi;
    const float predicted_body_roll = old_roll + roll_dot * dt;
    const float predicted_body_pitch = old_pitch + pitch_dot * dt;
    const float acc_body_roll = IMU_PITCH_MOUNT_TRIM_DEG - accel_pitch;
    const float acc_body_pitch = IMU_ROLL_MOUNT_TRIM_DEG - accel_roll;
    roll_angle = predicted_body_roll + blend * (acc_body_roll - predicted_body_roll);
    pitch_angle = predicted_body_pitch + blend * (acc_body_pitch - predicted_body_pitch);
    /* Preserve sensor-angle semantics for the existing ASCII diagnostics. */
    sensor_pitch_angle = IMU_PITCH_MOUNT_TRIM_DEG - roll_angle;
    sensor_roll_angle = IMU_ROLL_MOUNT_TRIM_DEG - pitch_angle;
    /* Non-finite attitude must still trigger the original motor failsafe. */
}

#if ENABLE_ATTITUDE_PID
static float clampf_local(float x, float lo, float hi) { if (x < lo) return lo; if (x > hi) return hi; return x; }
static float stick_to_angle_deg(uint16_t raw, uint16_t center) {
    return RC_StickNormalized(raw, center)*PID_ANGLE_LIMIT_DEG;
}
#endif

/* ----------------------------- SBUS ----------------------------------- */
static uint8_t sbus_rx_buffer[SBUS_RX_BUFFER_SIZE]; static uint8_t sbus_frame[SBUS_FRAME_SIZE]; static uint8_t sbus_frame_pos;
static volatile uint16_t sbus_channels[16]; static volatile uint8_t sbus_flags;
static volatile uint32_t sbus_frames_ok, sbus_frames_bad, sbus_uart_errors, sbus_last_valid_ms;
static volatile uint8_t sbus_rx_rearm_pending = 1u;

static bool sbus_footer_valid(uint8_t footer) { const uint8_t slot_code = (uint8_t)(footer & 0x0Fu); return (footer == 0x00u) || (slot_code == 0x04u) || (slot_code == 0x08u); }
static void sbus_decode_frame(const uint8_t frame[SBUS_FRAME_SIZE]) {
    uint32_t acc = 0; uint8_t avail = 0, idx = 1;
    for (uint8_t ch = 0; ch < 16; ++ch) {
        while (avail < 11) { acc |= ((uint32_t)frame[idx++]) << avail; avail += 8; }
        sbus_channels[ch] = (uint16_t)(acc & 0x07FFu); acc >>= 11; avail -= 11;
    }
    sbus_flags = frame[23]; sbus_last_valid_ms = HAL_GetTick(); sbus_frames_ok++;
}
static void sbus_parser_recover(void) {
    for (uint8_t i = 1u; i < SBUS_FRAME_SIZE; ++i) {
        if (sbus_frame[i] == SBUS_START_BYTE) { uint8_t rem = (uint8_t)(SBUS_FRAME_SIZE - i); memmove(sbus_frame, &sbus_frame[i], rem); sbus_frame_pos = rem; return; }
    }
    sbus_frame_pos = 0u;
}
static void sbus_feed_byte(uint8_t byte) {
    if (sbus_frame_pos == 0u) { if (byte == SBUS_START_BYTE) { sbus_frame[0] = byte; sbus_frame_pos = 1u; } return; }
    sbus_frame[sbus_frame_pos++] = byte;
    if (sbus_frame_pos >= SBUS_FRAME_SIZE) {
        if ((sbus_frame[0] == SBUS_START_BYTE) && sbus_footer_valid(sbus_frame[24])) { sbus_decode_frame(sbus_frame); sbus_frame_pos = 0u; }
        else { sbus_frames_bad++; sbus_parser_recover(); }
    }
}
static void sbus_service_receive_restart(void) {
    if (sbus_rx_rearm_pending == 0u || huart1.RxState != HAL_UART_STATE_READY) return;
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF);
    if (HAL_UARTEx_ReceiveToIdle_IT(&huart1, sbus_rx_buffer, sizeof(sbus_rx_buffer)) == HAL_OK) sbus_rx_rearm_pending = 0u;
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
    if (huart == &huart1) {
        if (size > sizeof(sbus_rx_buffer)) size = sizeof(sbus_rx_buffer);
        for (uint16_t i = 0u; i < size; ++i) sbus_feed_byte(sbus_rx_buffer[i]);
        sbus_rx_rearm_pending = 1u; return;
    }
    MT06_RxEventCallback(huart, size);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    UAVConfig_OnRxComplete(huart);
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) { sbus_uart_errors++; sbus_frame_pos = 0u; sbus_rx_rearm_pending = 1u; return; }
    if (huart == &hlpuart1) {
        UAVConfig_OnError(huart);
        if (!UAVTelemetry_OnTxError(huart) && log_tx_inflight) log_tx_errors++;
        log_tx_inflight = 0u;
        return;
    }
    MT06_ErrorCallback(huart);
}

/* ----------------------------- DShot Logic ---------------------------- */
static void dshot_set_compare_zero(void) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0u); __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0u);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0u); __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0u);
}
static void dshot_prepare_buffer(const uint16_t command[PWM_OUTPUT_COUNT]) {
    uint16_t packet[PWM_OUTPUT_COUNT];
    for (uint8_t motor = 0u; motor < PWM_OUTPUT_COUNT; ++motor) packet[motor] = dshot_make_packet(command[motor]);
    for (uint8_t bit = 0u; bit < DSHOT_PACKET_BITS; ++bit) {
        for (uint8_t motor = 0u; motor < PWM_OUTPUT_COUNT; ++motor) {
            uint8_t one = (uint8_t)((packet[motor] >> (15u - bit)) & 1u);
            dshot_dma_buffer[bit][motor] = one ? DSHOT_T1H_TICKS : DSHOT_T0H_TICKS;
        }
    }
    for (uint8_t motor = 0u; motor < PWM_OUTPUT_COUNT; ++motor) dshot_dma_buffer[DSHOT_PACKET_BITS][motor] = 0u;
}
static bool dshot_send_all(const uint16_t command[PWM_OUTPUT_COUNT]) {
    DMA_HandleTypeDef *dma = &hdma_tim3_up; HAL_StatusTypeDef status;
    __HAL_TIM_DISABLE_DMA(&htim3, TIM_DMA_UPDATE);
    if (HAL_DMA_GetState(dma) != HAL_DMA_STATE_READY) { (void)HAL_DMA_Abort(dma); dshot_set_compare_zero(); dshot_tx_errors++; return false; }
    dshot_prepare_buffer(command); dshot_set_compare_zero(); __DMB();
    dma->XferCpltCallback = dshot_dma_tx_complete; dma->XferErrorCallback = dshot_dma_tx_error;
    status = HAL_DMA_Start_IT(dma, (uint32_t)(uintptr_t)&dshot_dma_buffer[0][0], (uint32_t)(uintptr_t)&htim3.Instance->DMAR, DSHOT_DMA_CELLS * PWM_OUTPUT_COUNT);
    if (status != HAL_OK) { (void)HAL_DMA_Abort(dma); dshot_set_compare_zero(); dshot_tx_errors++; return false; }
    __HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_UPDATE); return true;
}

/* ----------------------------- Motor control -------------------------- */
static uint32_t boot_ms;
static bool armed, arm_ready, spinning, link_prev, throttle_filter_initialized, low_tracking, alt_hold_active;
static bool uav_config_allowed;
static UAVCageGuard cage_guard;
static uint32_t bb_spinning_since;
static bool bb_prev_cage_powered;
static bool bb_prev_cage_latched;
static UAVConfigGateState uav_config_gate;
static bool arm_switch_prev;
static uint32_t link_up_ms, low_since_ms;
static uint16_t win_min, win_max, throttle_zero;
static float throttle_filtered, throttle_ratio;
static uint32_t debug_age_ms, debug_frames_ok, debug_frames_bad, debug_uart_errors;
static uint16_t debug_throttle, debug_roll, debug_pitch, debug_arm, debug_kill, debug_alt_ch;
static uint16_t debug_yaw;
static uint8_t debug_sbus_flags;
static MC7RE_Guard mc7re_guard;
static uint8_t debug_rf_guard_reason;
static bool debug_rf_preset, debug_rf_latched;
static bool debug_link_ok, debug_imu_ok, debug_throttle_cal_ok;
/* V337: diagnostics ONLY: why ARM is not granted. No safety gate bypass. */
static uint16_t debug_arm_blockers;
static bool debug_arm_ready, debug_low_hold, debug_link_stable, debug_level_ok, debug_neutral_ok;
static float debug_pid_roll, debug_pid_pitch, debug_pid_yaw, debug_pid_alt;
static FlightPIDSampleHold pid_sample_hold;
static float debug_mix[PWM_OUTPUT_COUNT];
static uint16_t debug_requested[PWM_OUTPUT_COUNT]; /* mixer request before final safety gates */
static float debug_base_command;
static float debug_mixer_scale;
static float debug_mixer_offset;
static float debug_throttle_ratio;
static uint16_t output_dshot[PWM_OUTPUT_COUNT] = {0u, 0u, 0u, 0u};
static bool bench_pulse_pending, bench_pulse_active, bench_pulse_consumed, bench_arm_prev;
static uint32_t bench_prep_start_ms, bench_pulse_start_ms;
static bool bench_prep_tracking;
static uint8_t bench_result_state; /* 0=none, 3=complete, 4=aborted/insufficient DMA */
static uint8_t debug_bench_state;  /* 0=idle 1=ready 2=active 3=complete 4=abort */
static uint16_t debug_bench_command; /* Actual motor test command, 0 when not active */
static uint8_t debug_bench_motor; /* 0=none/gap, 1..4 = TIM3 CH1..4 */
static uint32_t bench_tx_start_done, bench_last_dma_frames;
/* YQ: 0=no directional evidence; 1=CW passed; 2=CCW passed;
 * 3=both signed body-yaw response directions passed; 4=sign mismatch.
 * This cannot verify motor reaction torque or flight gains. */
static uint16_t yaw_check_cw_count, yaw_check_ccw_count;
static uint8_t yaw_check_status;


static void outputs_force_zero(void) { for (uint8_t i = 0u; i < PWM_OUTPUT_COUNT; ++i) output_dshot[i] = 0u; dshot_set_compare_zero(); }

static void sbus_snapshot(uint32_t *frames_ok, uint32_t *last_valid_ms, uint32_t *frames_bad, uint32_t *uart_errors, uint8_t *flags, uint16_t channels[16]) {
    uint32_t primask = __get_PRIMASK(); __disable_irq();
    *frames_ok = sbus_frames_ok; *last_valid_ms = sbus_last_valid_ms; *frames_bad = sbus_frames_bad; *uart_errors = sbus_uart_errors; *flags = sbus_flags;
    for (uint8_t i = 0u; i < 16u; ++i) channels[i] = sbus_channels[i];
    if (primask == 0u) __enable_irq();
}
static bool throttle_calibration_valid(void) {
    return (SBUS_THR_HIGH_RAW > SBUS_THR_LOW_RAW) &&
           ((SBUS_THR_HIGH_RAW - SBUS_THR_LOW_RAW) >= 100u);
}

static void motor_step(void)
{
    uint32_t now = HAL_GetTick(); uint32_t elapsed = now - boot_ms;
    uint32_t frames_ok, last_valid_ms, frames_bad, uart_errors, age_ms; uint8_t flags; uint16_t channels[16];
    uint16_t ch_thr, ch_arm, ch_kill, ch_alt; bool link_ok, link_stable, throttle_low, stick_stable_low;
    bool arm_switch_on, kill_switch_on, imu_fresh, can_arm, throttle_cal_ok, alt_switch_on;
    float threshold = 0.0f, denominator, target_ratio = 0.0f;
    float pid_roll = 0.0f, pid_pitch = 0.0f, pid_yaw = 0.0f, pid_alt = 0.0f;

    sbus_snapshot(&frames_ok, &last_valid_ms, &frames_bad, &uart_errors, &flags, channels);

    if (dshot_dma_error_pending != 0u) {
        dshot_dma_error_pending = 0u; armed = false; arm_ready = false; spinning = false; throttle_ratio = 0.0f; outputs_force_zero();
    }
    /* DMA may glitch once while disarmed. An earlier error is retained in DE
     * but can be recovered after 600 consecutive successful frames. If the
     * errors persist, this never reaches the required clean streak. A fresh
     * fault still immediately disarms in callback / dshot_send_all. */
    if (dshot_tx_errors != dshot_faults_observed) {
        dshot_faults_observed = dshot_tx_errors;
        dshot_tx_done_at_fault = dshot_tx_done;
    }
    dshot_tx_recovered =
        ((uint32_t)(dshot_tx_done - dshot_tx_done_at_fault) >= UAV_DSHOT_REARM_GOOD_FRAMES) &&
        (dshot_dma_error_pending == 0u);

    ch_thr = channels[CH_THR]; ch_arm = channels[CH_ARM]; ch_kill = channels[CH_KILL]; ch_alt = channels[CH_ALT_HOLD];
    age_ms = now - last_valid_ms;

    /* MC7RE can retransmit a preset of 1000/1000/1000, THR=40, with
     * SBUS flags unchanged even after RF loss.  Guard the inferred pattern
     * separately from actual SBUS transport status; never call it proof of RF.
     * A latched event revokes link_ok and disarms without automatic re-arm. */
    {
        uint8_t raw_reason = 0u;
        bool raw_link;
        bool safe_arm_off, safe_kill_on, rc_at_neutral, safe_throttle_low;
        if ((frames_ok == 0u) || (age_ms >= SBUS_LINK_TIMEOUT_MS)) raw_reason = 1u;
        else if ((flags & SBUS_FLAG_FAILSAFE) != 0u) raw_reason = 3u;
        else if ((flags & SBUS_FLAG_FRAME_LOST) != 0u) raw_reason = 2u;
        raw_link = (raw_reason == 0u);
#if ARM_SWITCH_ACTIVE_HIGH
        safe_arm_off = (ch_arm < ARM_SWITCH_THRESHOLD);
#else
        safe_arm_off = (ch_arm > ARM_SWITCH_THRESHOLD);
#endif
#if KILL_SWITCH_ACTIVE_HIGH
        safe_kill_on = (ch_kill >= KILL_SWITCH_THRESHOLD);
#else
        safe_kill_on = (ch_kill <= KILL_SWITCH_THRESHOLD);
#endif
        rc_at_neutral = mc7re_near(channels[CH_ROLL], RC_ROLL_CENTER, MC7RE_NEUTRAL_TOL_RAW) &&
                        mc7re_near(channels[CH_PITCH], RC_PITCH_CENTER, MC7RE_NEUTRAL_TOL_RAW) &&
                        mc7re_near(channels[CH_YAW], RC_YAW_CENTER, MC7RE_NEUTRAL_TOL_RAW);
        safe_throttle_low = RC_ThrottleIsLow(ch_thr);
        mc7re_guard_update(&mc7re_guard, now, raw_reason,
                           channels[CH_ROLL], channels[CH_PITCH],
                           channels[CH_YAW], ch_thr,
                           safe_arm_off && safe_kill_on && rc_at_neutral && safe_throttle_low);
        link_ok = raw_link && !mc7re_guard.latched && !mc7re_guard.candidate;
        debug_rf_guard_reason = mc7re_guard.reason;
        debug_rf_preset = mc7re_guard.candidate;
        debug_rf_latched = mc7re_guard.latched;
    }
    if (link_ok && !link_prev) link_up_ms = now;
    link_prev = link_ok; link_stable = link_ok && ((now - link_up_ms) >= LINK_STABLE_MS);

    if (frames_ok > 0u) {
        /* Follow current SBUS throttle input without a low-pass ramp. */
        throttle_filtered = (float)ch_thr;
        throttle_filter_initialized = true;
    } else { throttle_filter_initialized = false; low_tracking = false; }

#if ARM_SWITCH_ACTIVE_HIGH
    arm_switch_on = (ch_arm >= ARM_SWITCH_THRESHOLD);
#else
    arm_switch_on = (ch_arm <= ARM_SWITCH_THRESHOLD);
#endif
#if KILL_SWITCH_ACTIVE_HIGH
    kill_switch_on = (ch_kill >= KILL_SWITCH_THRESHOLD);
#else
    kill_switch_on = (ch_kill <= KILL_SWITCH_THRESHOLD);
#endif
    const bool arm_rising = arm_switch_on && !arm_switch_prev;
    arm_switch_prev = arm_switch_on;
    /* V338 SIMPLE ARM: use measured SBUS endpoints from rc_input.h.
     * No full-high throttle sweep required on every boot.
     * Retain strict low-throttle, valid raw range, IMU, link, KILL and
     * explicit ARM-OFF -> ARM-ON checks below. */
    throttle_cal_ok = throttle_calibration_valid() &&
                      RC_ThrottleRawValid(ch_thr) &&
                      (UAV_THROTTLE_MAPPING_VERIFIED == 1u);
    alt_switch_on = (ch_alt >= ALT_HOLD_THRESHOLD); /* [THÊM MỚI] Đọc công tắc bám cây */

    throttle_low = link_ok && throttle_cal_ok && RC_ThrottleIsLow(ch_thr);
    if (!throttle_low) { low_tracking = false; low_since_ms = 0u; }
    else if (!low_tracking) { low_tracking = true; low_since_ms = now; win_min = ch_thr; win_max = ch_thr; }
    else {
        if (ch_thr < win_min) win_min = ch_thr;
        if (ch_thr > win_max) win_max = ch_thr;
        if ((uint16_t)(win_max - win_min) > THR_STABLE_BAND) {
            low_since_ms = now; win_min = ch_thr; win_max = ch_thr;
        }
    }
    stick_stable_low = low_tracking && ((now - low_since_ms) >= ARM_LOW_HOLD_MS);

    imu_fresh = imu_ready && imu_calibrated && imu_control_valid &&
                attitude_initialized && ((now - imu_last_good_ms) <= IMU_MAX_AGE_MS);
/* The only permitted release is IMU-feedback mode with fresh sensors,
     * a steady near-level craft and zero-rate neutral sticks at ARM time.
     * Failsafe and KILL remain independent disarm gates below. */
    can_arm = imu_fresh && throttle_cal_ok && (UAV_BENCH_ARM_LOCK == 0u) &&
              (UAV_EXPERIMENTAL_ARM_ENABLE == 1u) &&
              (UAV_YAW_DIAGNOSTIC_ARM_LOCK == 0u) &&
              (!UAV_CAGE_TEST_ARM_ENABLE || !cage_guard.latched);
    const bool level_for_arm = isfinite(roll_angle) && isfinite(pitch_angle) &&
              fabsf(roll_angle) <= UAV_ARM_MAX_ANGLE_DEG &&
              fabsf(pitch_angle) <= UAV_ARM_MAX_ANGLE_DEG &&
              fabsf(body_roll_rate_dps()) <= UAV_ARM_MAX_RATE_DPS &&
              fabsf(body_pitch_rate_dps()) <= UAV_ARM_MAX_RATE_DPS &&
              fabsf(body_yaw_rate_dps()) <= UAV_ARM_MAX_RATE_DPS;
    const bool neutral_for_arm =
              mc7re_near(channels[CH_ROLL], RC_ROLL_CENTER, MC7RE_NEUTRAL_TOL_RAW) &&
              mc7re_near(channels[CH_PITCH], RC_PITCH_CENTER, MC7RE_NEUTRAL_TOL_RAW) &&
              mc7re_near(channels[CH_YAW], RC_YAW_CENTER, MC7RE_NEUTRAL_TOL_RAW);

    /* V338: independent condition monitor, before the actual state transition.
     * BKL mask:
     * 0x001 ESC init, 0x002 invalid RC link, 0x004 KILL engaged,
     * 0x008 IMU stale/calibration, 0x010 throttle raw out of validated range,
     * 0x020 link stabilization, 0x040 throttle not low/stable 500ms,
     * 0x080 not level/still, 0x100 RC sticks off-center,
     * 0x200 MC7RE failsafe guard, 0x400 DShot has not recovered after a fault.
     * 0x800 ARM switch off (normal, not an error); 0x2000 yaw diagnostic lock.
     * This diagnostic does not change can_arm or motor output logic. */
    debug_arm_blockers = 0u;
    if (elapsed < ESC_INIT_MS) debug_arm_blockers |= 0x001u;
    if (!link_ok) debug_arm_blockers |= 0x002u;
    if (kill_switch_on) debug_arm_blockers |= 0x004u;
    if (!imu_fresh) debug_arm_blockers |= 0x008u;
    if (!throttle_cal_ok) debug_arm_blockers |= 0x010u;
    if (!link_stable) debug_arm_blockers |= 0x020u;
    if (!stick_stable_low) debug_arm_blockers |= 0x040u;
    if (!level_for_arm) debug_arm_blockers |= 0x080u;
    if (!neutral_for_arm) debug_arm_blockers |= 0x100u;
    if (mc7re_guard.latched || mc7re_guard.candidate) debug_arm_blockers |= 0x200u;
    if (!dshot_tx_recovered) debug_arm_blockers |= 0x400u;
    if (!arm_switch_on) debug_arm_blockers |= 0x800u;
    if (UAV_BENCH_ARM_LOCK != 0u) debug_arm_blockers |= 0x1000u;
    if (UAV_YAW_DIAGNOSTIC_ARM_LOCK != 0u) debug_arm_blockers |= 0x2000u;
    if (UAV_CAGE_TEST_ARM_ENABLE && cage_guard.latched) debug_arm_blockers |= 0x4000u;
    debug_link_stable = link_stable;
    debug_low_hold = stick_stable_low;
    debug_level_ok = level_for_arm;
    debug_neutral_ok = neutral_for_arm;

    if ((elapsed < ESC_INIT_MS) || !link_ok || kill_switch_on || !can_arm) {
        armed = false; arm_ready = false;
    } else if (!arm_switch_on) {
        armed = false; arm_ready = true;
    } else if (!armed && arm_rising && arm_ready && link_stable && stick_stable_low &&
               ((UAV_CONFIG_REQUIRE_CH8 == 0u) ||
                (channels[CH_CONFIG] < CONFIG_SWITCH_THRESHOLD)) &&
               level_for_arm && neutral_for_arm &&
               !mc7re_guard.latched && !mc7re_guard.candidate &&
               dshot_tx_recovered) {
        armed = true; throttle_zero = win_max; throttle_ratio = 0.0f; spinning = false;
        FlightPID_Reset(&flight_pid); FlightPIDSampleHold_Reset(&pid_sample_hold); alt_integral = 0.0f; alt_last_error = 0.0f;
        last_pid_ms = now; pid_last_sample_sequence = imu_sample_sequence;
    }

    /* Consume every ON attempt, including a failed pre-arm. A held ON
     * switch must never arm later when a condition becomes true. */
    if (arm_switch_on) arm_ready = false;

    /* Kiểm tra điều kiện an toàn cho chế độ Bám ngọn cây */
    if ((UAV_ENABLE_ALT_HOLD != 0u) && armed && alt_switch_on && current_range_valid) {
        alt_hold_active = true;
    } else {
        alt_hold_active = false; /* Tự động ngắt nếu mất cảm biến hoặc gạt công tắc xuống */
    }

    debug_arm_ready = arm_ready;
    if (!armed) {
        spinning = false; throttle_ratio = 0.0f;
        FlightPID_Reset(&flight_pid); alt_integral = 0.0f;
    } else {
        /* [THÊM MỚI] TÍNH TOÁN PID ĐỘ CAO */
        if (alt_hold_active) {
            float error = ALT_TARGET_CM - (float)current_vertical_range_cm;
            float dt = DSHOT_CONTROL_PERIOD_MS / 1000.0f;

            alt_integral += error * dt;
            if (alt_integral > 50.0f) alt_integral = 50.0f;
            if (alt_integral < -50.0f) alt_integral = -50.0f;

            float derivative = (error - alt_last_error) / dt;
            alt_last_error = error;

            pid_alt = clampf_local((PID_ALT_KP * error) + (PID_ALT_KI * alt_integral) + (PID_ALT_KD * derivative), -300.0f, 300.0f);
        } else {
            alt_integral = 0.0f; /* Reset nếu tắt chế độ */
        }

        /* Nếu bám cây thì ga tự động, nếu không thì ga theo tay điều khiển */
        if (alt_hold_active) {
            spinning = true; // Bám cây thì luôn coi như đang quay
        } else {
            threshold = (float)throttle_zero + THR_DEADBAND;
            if (!spinning && throttle_filtered > (threshold + THR_START_HYST)) spinning = true;
            else if (spinning && throttle_filtered < (threshold - THR_STOP_HYST)) spinning = false;

            if (spinning) {
                denominator = (float)SBUS_THR_HIGH_RAW - threshold;
                if (denominator > 1.0f) target_ratio = (throttle_filtered - threshold) / denominator;
                target_ratio = (target_ratio < 0.0f) ? 0.0f : ((target_ratio > 1.0f) ? 1.0f : target_ratio);
            }
            /* Direct mapping: no per-second software throttle-rise cap. */
            throttle_ratio = target_ratio;
        }
    }

/* Calculate PID only when new IMU sample exists. Between samples retain
       the latest correction, but never use it after the IMU age limit. */
#if ENABLE_ATTITUDE_PID
    if (imu_fresh && link_ok) {
        uint32_t dt_ms = now - last_pid_ms;
        bool new_imu_sample = (imu_sample_sequence != pid_last_sample_sequence);
        /* Millisecond timestamps can coincide when SPI crosses a tick.
         * Keep the held output until time advances; dt==0 is not a dropout. */
        if (new_imu_sample && dt_ms > 0u) {
            pid_last_sample_sequence = imu_sample_sequence;
            last_pid_ms = now;
            if (dt_ms > 0u && dt_ms <= IMU_MAX_AGE_MS) {
                FlightPIDInput in = {
                    .roll_set_deg = stick_to_angle_deg(channels[CH_ROLL], RC_ROLL_CENTER),
                    .pitch_set_deg = stick_to_angle_deg(channels[CH_PITCH], RC_PITCH_CENTER),
                    .yaw_rate_set_dps = RC_StickNormalized(channels[CH_YAW], RC_YAW_CENTER)*PID_YAW_RATE_LIMIT_DPS,
                    .roll_deg = control_roll_angle(),
                    .pitch_deg = control_pitch_angle(),
                    .gyro_roll_dps = body_roll_rate_dps(),
                    .gyro_pitch_dps = body_pitch_rate_dps(),
                    /* Body convention: positive yaw clockwise viewed from above.
                     * Preserve V328 manually observed negative sensor-Z sign. */
                    .gyro_yaw_dps = body_yaw_rate_dps(),
                    .dt_s = (float)dt_ms/1000.0f,
                    /* A restraint can hold the craft at a false steady angle.
                     * Integrate ONLY near level, neutral RC, moderate gyro,
                     * with the motors already spinning. Never wind against
                     * the cage/tether or a strong transient. */
                    .reset_integrator = !armed || !spinning,
                    .allow_integrator = armed && spinning &&
                        (fabsf(roll_angle) < 6.0f) &&
                        (fabsf(pitch_angle) < 6.0f) &&
                        (fabsf(control_roll_angle()) < 6.0f) &&
                        (fabsf(control_pitch_angle()) < 6.0f) &&
                        (fabsf(body_roll_rate_dps()) < 35.0f) &&
                        (fabsf(body_pitch_rate_dps()) < 35.0f) &&
                        (fabsf(body_yaw_rate_dps()) < 35.0f) &&
                        (fabsf(RC_StickNormalized(channels[CH_ROLL], RC_ROLL_CENTER)) < 0.02f) &&
                        (fabsf(RC_StickNormalized(channels[CH_PITCH], RC_PITCH_CENTER)) < 0.02f) &&
                        (fabsf(RC_StickNormalized(channels[CH_YAW], RC_YAW_CENTER)) < 0.02f)
                };
                bb_pid_dt_us = dt_ms * 1000u;
                FlightPIDOutput out;
                if (FlightPID_Step(&flight_pid, &in, &out)) {
                    (void)FlightPIDSampleHold_Store(&pid_sample_hold, &out, now);
                    /* Evaluate only a HAND-ROTATED, nearly level frame, with
                     * throttle low and explicit ARM OFF + KILL ON. Never affect
                     * output or arming on the basis of this diagnostic. */
                    if (yaw_check_status != 4u && link_ok && !arm_switch_on &&
                        kill_switch_on && RC_ThrottleIsLow(ch_thr) &&
                        mc7re_near(channels[CH_ROLL], RC_ROLL_CENTER, MC7RE_NEUTRAL_TOL_RAW) &&
                        mc7re_near(channels[CH_PITCH], RC_PITCH_CENTER, MC7RE_NEUTRAL_TOL_RAW) &&
                        mc7re_near(channels[CH_YAW], RC_YAW_CENTER, MC7RE_NEUTRAL_TOL_RAW) &&
                        (fabsf(roll_angle) < 12.0f) && (fabsf(pitch_angle) < 12.0f) &&
                        (fabsf(in.gyro_roll_dps) < 35.0f) &&
                        (fabsf(in.gyro_pitch_dps) < 35.0f) &&
                        (fabsf(in.gyro_yaw_dps) >= YAW_CHECK_RATE_MIN_DPS) &&
                        (fabsf(in.gyro_yaw_dps) <= YAW_CHECK_RATE_MAX_DPS)) {
                        bool opposes_rotation = ((in.gyro_yaw_dps > 0.0f) && (out.yaw < -4.0f)) ||
                                                ((in.gyro_yaw_dps < 0.0f) && (out.yaw > 4.0f));
                        if (!opposes_rotation) {
                            yaw_check_status = 4u;
                        } else {
                            if (in.gyro_yaw_dps > 0.0f && yaw_check_cw_count < YAW_CHECK_COUNT_TARGET)
                                yaw_check_cw_count++;
                            else if (in.gyro_yaw_dps < 0.0f && yaw_check_ccw_count < YAW_CHECK_COUNT_TARGET)
                                yaw_check_ccw_count++;
                            yaw_check_status = (uint8_t)((yaw_check_cw_count >= YAW_CHECK_COUNT_TARGET ? 1u : 0u) |
                                                         (yaw_check_ccw_count >= YAW_CHECK_COUNT_TARGET ? 2u : 0u));
                        }
                    }
                } else {
                    FlightPIDSampleHold_Reset(&pid_sample_hold);
                }
            } else {
                FlightPID_Reset(&flight_pid);
                FlightPIDSampleHold_Reset(&pid_sample_hold);
            }
        }
    } else {
        FlightPID_Reset(&flight_pid);
        FlightPIDSampleHold_Reset(&pid_sample_hold);
    }
    {
        FlightPIDOutput held;
        if (FlightPIDSampleHold_Read(&pid_sample_hold, now, IMU_MAX_AGE_MS, &held)) {
            pid_roll = held.roll;
            pid_pitch = held.pitch;
            pid_yaw = held.yaw;
        } else if (armed && spinning) {
            /* Never silently fall back to equal open-loop motor commands
             * after PID validation fails or its held result expires. */
            armed = false; arm_ready = false; spinning = false;
            throttle_ratio = 0.0f;
            outputs_force_zero();
        }
    }
#endif

    /* QUAD-X mixer with desaturation by collective shift. The old algorithm
     * multiplied every PID correction by a headroom ratio. At base=2047,
     * high_headroom became zero and all attitude corrections collapsed to 0.
     * This version preserves differential torque authority and shifts the
     * complete motor set into the legal DShot range. */
    {
        float base = 0.0f;
        float mixed[PWM_OUTPUT_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
        float scale = 0.0f;
        float offset = 0.0f;
        if (armed && spinning) {
            if (alt_hold_active) {
                base = ALT_HOVER_THROTTLE + pid_alt;
            } else {
                base = (float)DSHOT_THROTTLE_MIN + throttle_ratio *
                       (float)(DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN);
            }
            base = clampf_local(base, (float)DSHOT_THROTTLE_MIN,
                                (float)DSHOT_THROTTLE_MAX);
        }

        float correction[PWM_OUTPUT_COUNT];
        FlightPID_MixX(pid_roll, pid_pitch, pid_yaw, correction);
        for (uint8_t i = 0u; i < PWM_OUTPUT_COUNT; ++i) debug_mix[i] = correction[i];

        const bool mix_ok = (armed && spinning) &&
            FlightPID_DesaturateMixer(base, correction,
                (float)DSHOT_THROTTLE_MIN, (float)UAV_EXPERIMENTAL_MAX_DSHOT,
                mixed, &scale, &offset);

        debug_base_command = base;
        debug_mixer_scale = mix_ok ? scale : 0.0f;
        debug_mixer_offset = mix_ok ? offset : 0.0f;
        debug_throttle_ratio = throttle_ratio;

        for (uint8_t i = 0u; i < PWM_OUTPUT_COUNT; ++i) {
            if (mix_ok && ((TEST_MOTOR_MASK >> i) & 1u) != 0u) {
                const uint16_t command = (uint16_t)(mixed[i] + 0.5f);
                debug_requested[i] = command;
                output_dshot[i] = command;
            } else {
                debug_requested[i] = 0u;
                output_dshot[i] = 0u;
            }
        }
    }

    /* Flight mixer is released ONLY while the already-guarded state machine
     * holds ARM+SPIN and live IMU+SBUS.  Kill and inferred MC7RE failsafe
     * revoke armed above; this last barrier is defense in depth. */
    if (!armed || !spinning || !imu_fresh || !link_ok || kill_switch_on ||
        mc7re_guard.latched || mc7re_guard.candidate) {
        outputs_force_zero();
    }

    /* PROPS-OFF TEST ONLY, flight ARM stays compile-time locked.
     * Two bounded low DShot stages to let a sensorless ESC spool up.
     * Never infer motor rotation from E=3. E=3 means timer + DMA callbacks
     * completed; it does NOT confirm actual ESC reception or rotation.
     * Trigger: KILL ON + ARM OFF + THR LOW for 1s, then KILL OFF,
     * then a fresh ARM rising edge while keeping THR LOW.
     * Fail closed: KILL ON / link lost / high throttle / IMU lost / DMA error.
     */
#if UAV_PROPS_OFF_ESC_TEST
    {
        bool arm_rising = arm_switch_on && !bench_arm_prev;
        bool neutral_sticks =
            mc7re_near(channels[CH_ROLL], RC_ROLL_CENTER, MC7RE_NEUTRAL_TOL_RAW) &&
            mc7re_near(channels[CH_PITCH], RC_PITCH_CENTER, MC7RE_NEUTRAL_TOL_RAW) &&
            mc7re_near(channels[CH_YAW], RC_YAW_CENTER, MC7RE_NEUTRAL_TOL_RAW);
        bool safe_inputs = link_stable && imu_fresh && throttle_cal_ok &&
                           !mc7re_guard.latched && !mc7re_guard.candidate &&
                           RC_ThrottleIsLow(ch_thr) && neutral_sticks &&
                           (fabsf(roll_angle) < 10.0f) && (fabsf(pitch_angle) < 10.0f) &&
                           (elapsed >= ESC_INIT_MS) &&
                           (dshot_tx_errors == 0u) &&
                           (dshot_dma_error_pending == 0u);
        uint32_t pulse_age;
        bench_arm_prev = arm_switch_on;

        /* Revoke test immediately if any safety precondition fails. */
        if (bench_pulse_active &&
            (!safe_inputs || kill_switch_on || !arm_switch_on)) {
            bench_pulse_active = false;
            bench_result_state = 4u;
            bench_last_dma_frames = dshot_tx_done - bench_tx_start_done;
        }
        if (bench_pulse_active) {
            pulse_age = now - bench_pulse_start_ms;
            if (pulse_age >= UAV_BENCH_TEST_TOTAL_MS) {
                bench_pulse_active = false;
                bench_last_dma_frames = dshot_tx_done - bench_tx_start_done;
                bench_result_state =
                    (bench_last_dma_frames >= UAV_BENCH_MIN_DMA_FRAMES &&
                     dshot_tx_errors == 0u) ? 3u : 4u;
            }
        }

        if (safe_inputs && kill_switch_on && !arm_switch_on) {
            if (!bench_prep_tracking) {
                bench_prep_tracking = true;
                bench_prep_start_ms = now;
            }
            if ((now - bench_prep_start_ms) >= UAV_BENCH_PREP_HOLD_MS) {
                bench_pulse_pending = true;
                bench_pulse_consumed = false;
                bench_result_state = 0u;
                bench_last_dma_frames = 0u;
            }
        } else {
            bench_prep_tracking = false;
        }
        if (!safe_inputs) bench_pulse_pending = false;

        if (safe_inputs && !kill_switch_on && arm_rising &&
            bench_pulse_pending && !bench_pulse_consumed && !bench_pulse_active) {
            bench_pulse_active = true;
            bench_pulse_consumed = true;
            bench_pulse_pending = false;
            bench_pulse_start_ms = now;
            bench_tx_start_done = dshot_tx_done;
            bench_result_state = 0u;
            bench_last_dma_frames = 0u;
        }
        debug_bench_command = 0u;
        debug_bench_motor = 0u;
        if (bench_pulse_active) {
            /* Scan CH1..CH4 sequentially, zero command between channels. */
            const uint32_t elapsed_test = now - bench_pulse_start_ms;
            const uint32_t slot = elapsed_test / UAV_BENCH_TEST_SEGMENT_MS;
            const uint32_t phase = elapsed_test % UAV_BENCH_TEST_SEGMENT_MS;
            outputs_force_zero();
            if (slot < UAV_BENCH_TEST_MOTOR_COUNT &&
                phase < UAV_BENCH_TEST_MOTOR_WINDOW_MS) {
                debug_bench_motor = (uint8_t)(slot + 1u);
                debug_bench_command = (phase < UAV_BENCH_TEST_STAGE1_MS) ?
                    UAV_BENCH_TEST_STAGE1_DSHOT : UAV_BENCH_TEST_STAGE2_DSHOT;
                output_dshot[slot] = debug_bench_command;
            }
        }
        debug_bench_state = bench_pulse_active ? 2u :
                            bench_pulse_pending ? 1u :
                            bench_pulse_consumed ? bench_result_state : 0u;
    }
#else
    debug_bench_state = 0u;
    debug_bench_command = 0u;
    debug_bench_motor = 0u;
#endif

    /* R3 diagnosis: independent LAST-LINE motor-output interlock.
     * No bench scan or flight output is permitted in this diagnostic build,
     * including if another part of the state machine ever sets armed=true.
     * This is deliberately independent from SBUS ARM, KILL and CONFIG. */
#if UAV_YAW_DIAGNOSTIC_ARM_LOCK
    armed = false;
    spinning = false;
    arm_ready = false;
    throttle_ratio = 0.0f;
    outputs_force_zero();
#endif
#if UAV_CAGE_TEST_ARM_ENABLE
    /* Independent FINAL-LINE captive test guard; cannot be bypassed by
     * PID, configuration writes, motor mixing or the ARM state machine.
     * Once a run ends/trips, reset power before another cage experiment. */
    const UAVCageInputs cage_in = {
        .now_ms=now, .armed=armed, .spinning=spinning,
        .pilot_stopped=link_ok && (kill_switch_on || !arm_switch_on),
        .input_ok=imu_fresh && link_ok && !kill_switch_on &&
                  !mc7re_guard.latched && !mc7re_guard.candidate &&
                  dshot_dma_error_pending==0u,
        .yaw_dps=body_yaw_rate_dps(),
        .roll_dps=body_roll_rate_dps(),
        .pitch_dps=body_pitch_rate_dps(),
        .roll_deg=roll_angle, .pitch_deg=pitch_angle
    };
    if (UAVCage_Update(&cage_guard, &cage_in)) {
        armed=false; spinning=false; arm_ready=false;
        throttle_ratio=0.0f; outputs_force_zero();
    }
    /* A normal pilot stop / 5.5s timeout is a *completed* test, not an
     * unsafe flight fault. Allow another attempt ONLY after a deliberate
     * disarmed safe hold. Never clear FAST_YAW / ATTITUDE / RATE / INPUT faults. */
    const bool cage_safe_clear = UAVCage_IsBenignStop(&cage_guard) &&
        !arm_switch_on && kill_switch_on && throttle_low &&
        link_stable && imu_fresh && level_for_arm && neutral_for_arm &&
        dshot_dma_error_pending == 0u && dshot_tx_recovered &&
        !mc7re_guard.latched && !mc7re_guard.candidate &&
        !armed && !spinning &&
        ((output_dshot[0] | output_dshot[1] |
          output_dshot[2] | output_dshot[3]) == 0u);
    /* Wait for the buffered PID capture to finish downloading before clearing
     * the latched fault, so the Dashboard can still show the stop reason. */
    (void)UAVCage_TryClearBenign(&cage_guard, now,
                                cage_safe_clear && !UAVBB_Dumping());
#endif
    /* V335: guarded pilot-throttle attitude mixer.  Neither SBUS transport
     * nor this code proves RF is present when MC7RE repeats old channels. */
    if (!dshot_send_all(output_dshot)) {
        armed = false; arm_ready = false; spinning = false; throttle_ratio = 0.0f; outputs_force_zero();
#if UAV_PROPS_OFF_ESC_TEST
        if (bench_pulse_active) {
            bench_pulse_active = false;
            bench_result_state = 4u;
            bench_last_dma_frames = dshot_tx_done - bench_tx_start_done;
            debug_bench_state = 4u;
            debug_bench_command = 0u;
            debug_bench_motor = 0u;
        }
#endif
    }

    /* MC6C six-channel safe PID write permission (no nonexistent CH8).
     * Require continuous RC safety for >= 2 seconds; immediately revoke on
     * KILL OFF, ARM ON, throttle increase, RF loss or any motor output.
     * The existing Wi-Fi authentication/physical props-off confirmation are
     * additional independent restrictions, not replacements for this gate. */
    const UAVConfigGateInputs config_inputs = {
        .link_stable = link_stable,
        .rf_guard_ok = !mc7re_guard.latched && !mc7re_guard.candidate,
        .config_switch_on = (UAV_CONFIG_REQUIRE_CH8 == 0u) ||
                            (channels[CH_CONFIG] >= CONFIG_SWITCH_THRESHOLD),
        .kill_on = kill_switch_on,
        .arm_on = arm_switch_on,
        .throttle_low = throttle_low,
        .armed = armed,
        .spinning = spinning,
        .bench_in_progress = bench_pulse_active || bench_pulse_pending,
    };
    uav_config_allowed = UAVConfigGate_Update(&uav_config_gate, now, &config_inputs);

    debug_age_ms = age_ms; debug_frames_ok = frames_ok; debug_frames_bad = frames_bad; debug_uart_errors = uart_errors;
    debug_link_ok = link_ok; debug_imu_ok = imu_fresh; debug_throttle_cal_ok = throttle_cal_ok;
    debug_roll = channels[CH_ROLL]; debug_pitch = channels[CH_PITCH]; debug_yaw = channels[CH_YAW]; debug_sbus_flags = flags;
    debug_pid_roll = pid_roll; debug_pid_pitch = pid_pitch; debug_pid_yaw = pid_yaw; debug_pid_alt = pid_alt;
    debug_throttle = ch_thr; debug_arm = ch_arm; debug_kill = ch_kill; debug_alt_ch = ch_alt;
}

/* --------------------------- Telemetry publisher -------------------------- */
#define UAV_TELEMETRY_PERIOD_MS 20u /* 50 Hz snapshot; non-blocking UART IT */

static int16_t telemetry_i16_scaled(float value, float scale)
{
    if (!isfinite(value)) return 0;
    value *= scale;
    if (value > 32767.0f) value = 32767.0f;
    if (value < -32768.0f) value = -32768.0f;
    return (int16_t)lrintf(value);
}

static uint16_t telemetry_u16_scaled(float value, float scale)
{
    if (!isfinite(value) || value <= 0.0f) return 0u;
    value *= scale;
    if (value > 65535.0f) value = 65535.0f;
    return (uint16_t)lrintf(value);
}

static uint16_t telemetry_u16_from_u32(uint32_t value)
{
    return (value > 65535u) ? 65535u : (uint16_t)value;
}

static void telemetry_publish(uint32_t now_ms)
{
    UAVTelemetrySample sample = {0};
    bool kill_active = (debug_kill >= KILL_SWITCH_THRESHOLD);

    sample.timestamp_ms = now_ms;
    if (debug_link_ok) sample.flags |= UAV_TELEM_FLAG_LINK_OK;
    if (((debug_sbus_flags & (SBUS_FLAG_FAILSAFE | SBUS_FLAG_FRAME_LOST)) != 0u) ||
        debug_rf_latched || debug_rf_preset) sample.flags |= UAV_TELEM_FLAG_FAILSAFE;
    if (armed) sample.flags |= UAV_TELEM_FLAG_ARMED;
    if (spinning) sample.flags |= UAV_TELEM_FLAG_SPINNING;
    if (debug_imu_ok) sample.flags |= UAV_TELEM_FLAG_IMU_FRESH;
    if (uav_config_allowed) sample.flags |= UAV_TELEM_FLAG_CONFIG_READY;
    if (kill_active) sample.flags |= UAV_TELEM_FLAG_KILL_ACTIVE;
    if (alt_hold_active) sample.flags |= UAV_TELEM_FLAG_ALT_HOLD;
    if (imu_ready && imu_calibrated) sample.flags |= UAV_TELEM_FLAG_IMU_CAL;
    if (current_range_valid) sample.flags |= UAV_TELEM_FLAG_RANGE_VALID;
    if (dshot_dma_error_pending != 0u) sample.flags |= UAV_TELEM_FLAG_DMA_ERROR;
#if UAV_CAGE_TEST_ARM_ENABLE
    sample.flags |= ((uint16_t)cage_guard.fault & 7u) << UAV_TELEM_CAGE_FAULT_SHIFT;
    if (cage_guard.latched) sample.flags |= UAV_TELEM_FLAG_CAGE_LATCHED;
#endif

    sample.rc_throttle = debug_throttle;
    sample.rc_roll = debug_roll;
    sample.rc_pitch = debug_pitch;
    sample.rc_yaw = debug_yaw;

    sample.roll_cdeg = telemetry_i16_scaled(control_roll_angle(), 100.0f);
    sample.pitch_cdeg = telemetry_i16_scaled(control_pitch_angle(), 100.0f);
    sample.gyro_roll_dps_x10 = telemetry_i16_scaled(body_roll_rate_dps(), 10.0f);
    sample.gyro_pitch_dps_x10 = telemetry_i16_scaled(body_pitch_rate_dps(), 10.0f);
    sample.gyro_yaw_dps_x10 = telemetry_i16_scaled(body_yaw_rate_dps(), 10.0f);
    sample.accel_x_raw = accel_x;
    sample.accel_y_raw = accel_y;
    sample.accel_z_raw = accel_z;

    sample.pid_roll_x10 = telemetry_i16_scaled(debug_pid_roll, 10.0f);
    sample.pid_pitch_x10 = telemetry_i16_scaled(debug_pid_pitch, 10.0f);
    sample.pid_yaw_x10 = telemetry_i16_scaled(debug_pid_yaw, 10.0f);
    sample.base_dshot = telemetry_u16_scaled(debug_base_command, 1.0f);
    sample.mixer_scale_x1000 = telemetry_u16_scaled(debug_mixer_scale, 1000.0f);
    sample.mixer_offset = telemetry_i16_scaled(debug_mixer_offset, 1.0f);

    for (uint8_t i = 0u; i < PWM_OUTPUT_COUNT; ++i) {
        sample.mix_x10[i] = telemetry_i16_scaled(debug_mix[i], 10.0f);
        sample.requested_dshot[i] = debug_requested[i];
        sample.output_dshot[i] = output_dshot[i];
        if (output_dshot[i] != 0u) sample.flags |= UAV_TELEM_FLAG_OUTPUT_ACTIVE;
    }

    /* The current project has no validated battery voltage/current ADC path.
     * 0xFFFF is an explicit unavailable sentinel, not a fabricated reading. */
    sample.battery_mv = 0xFFFFu;
    sample.current_ma = 0xFFFFu;
    sample.range_cm = current_range_valid ? current_vertical_range_cm : 0xFFFFu;
    sample.imu_dt_max_ms = telemetry_u16_from_u32(imu_dt_max_ms);
    sample.motor_dt_max_ms = telemetry_u16_from_u32(motor_dt_max_ms);
    sample.dshot_errors = telemetry_u16_from_u32(dshot_tx_errors);
    sample.rx_age_ms = telemetry_u16_from_u32(debug_age_ms);

    (void)UAVTelemetry_Send(&hlpuart1, &sample);
}

/* R5 high-rate blackbox: sample real motor outputs while spinning in cage.
 * Ring is written in the control loop; UART transfer is deferred until
 * disarmed, KILL ON and all outputs have stopped. */
static void bb_record_if_due(uint32_t now_ms, bool kill_on)
{
    /* A motor shutdown may coincide with an invalid/stale IMU or occur less
     * than 5 ms after the last Blackbox sample. Preserve the final powered
     * window even when the normal sample-rate gate rejects this tick. */
    const bool motor_now_on = armed && spinning && !kill_on &&
        ((output_dshot[0] | output_dshot[1] |
          output_dshot[2] | output_dshot[3]) != 0u);
    if (bb_prev_cage_powered && !motor_now_on) {
        UAVIMURaw_MotorStopped();
        UAVBB_FreezeNow();
        bb_prev_cage_powered = false;
    }
    if (!imu_control_valid || imu_sample_sequence == bb_last_imu_seq ||
        (uint32_t)(now_ms - bb_last_sample_ms) < UAV_BB_PERIOD_MS) return;
    bb_last_imu_seq = imu_sample_sequence;
    bb_last_sample_ms = now_ms;
    UAVRateSample s = {0};
    s.stamp_ms = now_ms;
    s.pid_dt_us = (bb_pid_dt_us > 65535u) ? 65535u : (uint16_t)bb_pid_dt_us;
    s.motor_dt_us = bb_motor_interval_us;
    for (unsigned a = 0u; a < 3u; ++a) {
        s.gyro_x10[a] = telemetry_i16_scaled(
            a == 0u ? body_roll_rate_dps() : a == 1u ? body_pitch_rate_dps() : body_yaw_rate_dps(),10.0f);
        s.set_x10[a] = telemetry_i16_scaled(flight_pid.rate_set_dps[a], 10.0f);
        s.p_x10[a] = telemetry_i16_scaled(flight_pid.pid_p[a], 10.0f);
        s.i_x10[a] = telemetry_i16_scaled(flight_pid.pid_i[a], 10.0f);
        s.d_x10[a] = telemetry_i16_scaled(flight_pid.pid_d[a], 10.0f);
    }
    for (unsigned m = 0u; m < 4u; ++m) s.motor[m] = output_dshot[m];
    s.base = telemetry_u16_scaled(debug_base_command, 1.0f);
    s.mixer_scale_x1000 = telemetry_u16_scaled(debug_mixer_scale, 1000.0f);
    s.imu_seq_lsb = (uint16_t)imu_sample_sequence;
    if (armed) s.flags |= 1u;
    if (spinning) s.flags |= 2u;
    if (kill_on) s.flags |= 4u;
    if (dshot_dma_error_pending) s.flags |= 8u;
    FlightPIDGains gains;
    FlightPID_GetGains(&gains);
    if (fabsf(debug_pid_yaw) >= gains.correction_limit - 0.5f) s.flags |= 16u;
    if (armed && debug_mixer_scale < 0.999f) s.flags |= 32u;
    const float mag = fmaxf(fabsf(body_yaw_rate_dps()),
                           fmaxf(fabsf(body_roll_rate_dps()), fabsf(body_pitch_rate_dps())));
    const bool motors_on = (s.motor[0] | s.motor[1] | s.motor[2] | s.motor[3]) != 0u;
    /* Record every NORMAL powered run, not just cage-mode sessions.
     * Capture a rolling IMU window during flight and freeze it at pilot stop
     * or any disarm; transmit only after motors are zero and KILL is ON. */
    const bool motors_powered = armed && spinning && motors_on && !kill_on;
    if (motors_powered && bb_spinning_since==0u) bb_spinning_since=now_ms;
    const bool motor_just_started = !bb_prev_cage_powered && motors_powered;
    const bool motor_just_stopped = bb_prev_cage_powered && !motors_powered;
    if (motor_just_started) UAVIMURaw_MotorStarted();
    if (motor_just_stopped) UAVIMURaw_MotorStopped();
    bb_prev_cage_powered = motors_powered;
    if (!motors_powered) bb_spinning_since=0u;
    const bool passive_trigger = !armed && !spinning &&
          kill_on && !motors_on && mag>=45.0f;
    UAVBB_Push(&s, passive_trigger);
    if (motor_just_stopped) UAVBB_FreezeNow();
}

/* ----------------------------- Public API ----------------------------- */
void uav_test_init(void)
{
    __HAL_TIM_DISABLE_DMA(&htim3, TIM_DMA_UPDATE); __HAL_TIM_SET_PRESCALER(&htim3, 0u); __HAL_TIM_SET_AUTORELOAD(&htim3, DSHOT_TIMER_PERIOD_TICKS - 1u);
    SET_BIT(htim3.Instance->CCMR1, TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE); SET_BIT(htim3.Instance->CCMR2, TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE);
    htim3.Instance->DCR = TIM_DMABASE_CCR1 | TIM_DMABURSTLENGTH_4TRANSFERS;
    __HAL_TIM_SET_COUNTER(&htim3, 0u); outputs_force_zero(); htim3.Instance->EGR = TIM_EGR_UG; __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);

    mc7re_guard_init(&mc7re_guard);
    debug_rf_guard_reason = 5u; debug_rf_preset = false; debug_rf_latched = true;
    armed = false; arm_ready = false; arm_switch_prev = false; spinning = false; uav_config_allowed = false;
    UAVConfigGate_Reset(&uav_config_gate); link_prev = false; throttle_filter_initialized = false; low_tracking = false;
    imu_control_valid = false;
    imu_gap_count = 0u; imu_dt_max_ms = 0u; motor_dt_max_ms = 0u; motor_late_count = 0u;
    throttle_ratio = 0.0f; imu_read_errors = 0u; imu_last_good_ms = 0u; imu_sample_sequence = 0u; pid_last_sample_sequence = 0u;
    dshot_tx_errors = 0u; dshot_tx_done = 0u; dshot_dma_error_pending = 0u;
    dshot_faults_observed = 0u; dshot_tx_done_at_fault = 0u; dshot_tx_recovered = false;
    sbus_frame_pos = 0u; sbus_rx_rearm_pending = 1u;
    FlightPID_Reset(&flight_pid); FlightPIDSampleHold_Reset(&pid_sample_hold);
    UAVTelemetry_Reset();
    UAVBB_Reset();
    UAVIMURaw_Reset();
    imu_raw_capture_ready = false;
    UAVCage_Init(&cage_guard);
    bb_spinning_since=0u; bb_prev_cage_powered=false;
    bb_prev_cage_latched=false;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    bb_cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000u;
    if (bb_cycles_per_us == 0u) bb_cycles_per_us = 1u;
    bb_prev_motor_cycle = bb_last_sample_ms = bb_last_imu_seq = bb_last_tx_ms = bb_pid_dt_us = 0u;
    bb_motor_interval_us = 0u;
    UAVConfig_Init(&hlpuart1);
    for (uint8_t i = 0u; i < PWM_OUTPUT_COUNT; ++i) debug_requested[i] = 0u;
    bench_pulse_pending = false; bench_pulse_active = false;
    bench_pulse_consumed = false; bench_prep_tracking = false;
    bench_arm_prev = false;
    bench_result_state = 0u; debug_bench_state = 0u;
    debug_bench_command = 0u;
    debug_bench_motor = 0u;
    bench_tx_start_done = 0u; bench_last_dma_frames = 0u;
    yaw_check_cw_count = 0u; yaw_check_ccw_count = 0u; yaw_check_status = 0u;
    alt_integral = 0.0f; roll_angle = 0.0f; pitch_angle = 0.0f;
    debug_base_command = 0.0f; debug_mixer_scale = 0.0f;
    debug_mixer_offset = 0.0f; debug_throttle_ratio = 0.0f;
    sensor_roll_angle = 0.0f; sensor_pitch_angle = 0.0f; attitude_initialized = false; alt_hold_active = false;

    barometer_ready = DPS310_Init(&barometer, &hi2c1, DPS310_I2C_ADDR_DEFAULT);
    if (!barometer_ready) barometer_ready = DPS310_Init(&barometer, &hi2c1, DPS310_I2C_ADDR_ALT);
    rangefinder_ready = MT06_Init(&rangefinder, &huart3);

    imu_ready = mpu6500_init_spi(); imu_calibrated = false;
    if (imu_ready) imu_calibrated = mpu6500_calibrate_gyro();
    /* Only capture after the 3-second gyro-bias calibration, not during it. */
    imu_raw_capture_ready = imu_ready && imu_calibrated;
    last_attitude_update_ms = HAL_GetTick();

#if ENABLE_ATTITUDE_PID
    if (!imu_ready || !imu_calibrated) log_async("FAULT: PID locked; IMU init/calibration failed reason=%u\r\n", imu_cal_failure_reason);
#else
    if (!imu_ready) log_async("WARN: IMU unavailable; PID disabled; bench mode only\r\n");
    else if (!imu_calibrated) log_async("WARN: gyro calibration failed; PID must remain disabled\r\n");
#endif

    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

    boot_ms = HAL_GetTick(); sbus_service_receive_restart();
    log_async("YAW_R5_1: CAGE=%u DIAG_LOCK=%u GYRO_FS=2000dps YSIGN=%ld ARM_ENABLE=%u ARM_LOCK=%u SCAN=%u ALT_HOLD=%u IMU=%u CAL=%u\r\n",
              (unsigned)UAV_CAGE_TEST_ARM_ENABLE,
              (unsigned)UAV_YAW_DIAGNOSTIC_ARM_LOCK,
              (long)UAV_BODY_YAW_SENSOR_SIGN,
              (unsigned)UAV_EXPERIMENTAL_ARM_ENABLE,
              (unsigned)UAV_BENCH_ARM_LOCK,
              (unsigned)UAV_PROPS_OFF_ESC_TEST,
              (unsigned)UAV_ENABLE_ALT_HOLD,
              imu_ready ? 1u : 0u, imu_calibrated ? 1u : 0u);
}

void uav_test_loop(void)
{
    static uint32_t last_motor_update_ms;
    static uint32_t last_log_ms;
    static uint32_t last_imu_ms;
    static uint32_t last_telemetry_ms;
    uint32_t now = HAL_GetTick(); DPS310_Data baro_data = {0}; MT06_Data range_data = {0};

    sbus_service_receive_restart();
    if (rangefinder_ready) MT06_Update(&rangefinder);

    if (imu_ready && ((now - last_imu_ms) >= IMU_SAMPLE_PERIOD_MS)) {
        last_imu_ms = now;
        if (mpu6500_read_spi() && imu_calibrated) {
            uint32_t sample_now = HAL_GetTick(); uint32_t attitude_dt = sample_now - last_attitude_update_ms;
            if (attitude_dt > imu_dt_max_ms) imu_dt_max_ms = attitude_dt;
            if ((attitude_dt > 0u) && (attitude_dt <= IMU_MAX_AGE_MS)) {
                update_attitude(attitude_dt);
                imu_control_valid = attitude_initialized && isfinite(roll_angle) && isfinite(pitch_angle);
            } else if (attitude_dt > IMU_MAX_AGE_MS) {
                /* Mark this recovery sample invalid BEFORE motor_step can
                 * observe the newly refreshed SPI timestamp. Reinitialize on
                 * the next timely sample; arming still requires a new edge. */
                imu_control_valid = false;
                attitude_initialized = false;
                FlightPID_Reset(&flight_pid);
                FlightPIDSampleHold_Reset(&pid_sample_hold);
                imu_gap_count++;
            }
            last_attitude_update_ms = sample_now;
        }
    }

    /* [CẬP NHẬT] Tính toán khoảng cách thẳng đứng thực tế bù trừ góc nghiêng */
    (void)MT06_GetData(&rangefinder, &range_data);
    current_range_valid = false;
    current_vertical_range_cm = 0;

    now = HAL_GetTick(); /* SPI may have crossed a tick; avoid unsigned age underflow. */
    if (range_data.valid && imu_ready && imu_calibrated && imu_control_valid && ((now - imu_last_good_ms) <= IMU_MAX_AGE_MS)) {
        const float deg_to_rad = 0.01745329251994329577f;
        const float tilt_factor = cosf(roll_angle * deg_to_rad) * cosf(pitch_angle * deg_to_rad);
        if ((tilt_factor > 0.70f) && (tilt_factor <= 1.01f)) {
            float vertical_cm = (float)range_data.distance_cm * tilt_factor;
            if (vertical_cm > 65535.0f) vertical_cm = 65535.0f;
            current_vertical_range_cm = (uint16_t)(vertical_cm + 0.5f);
            current_range_valid = true;
        }
    }

    now = HAL_GetTick();
    if ((now - last_motor_update_ms) >= DSHOT_CONTROL_PERIOD_MS) {
        /* Ignore the first interval from boot/calibration. Millisecond
         * diagnostics detect missed ticks, not sub-ms timing jitter. */
        if (last_motor_update_ms != 0u) {
            const uint32_t interval = now - last_motor_update_ms;
            if (interval > motor_dt_max_ms) motor_dt_max_ms = interval;
            if (interval > DSHOT_CONTROL_PERIOD_MS) motor_late_count++;
        }
        last_motor_update_ms = now;
        const uint32_t motor_cycle = DWT->CYCCNT;
        if (bb_prev_motor_cycle != 0u) {
            const uint32_t delta_us = (motor_cycle - bb_prev_motor_cycle) / bb_cycles_per_us;
            bb_motor_interval_us = delta_us > 65535u ? 65535u : (uint16_t)delta_us;
        }
        bb_prev_motor_cycle = motor_cycle;
        motor_step();
        /* Capture only after final motor interlock and DShot output selection. */
        bb_record_if_due(now, debug_kill >= KILL_SWITCH_THRESHOLD);
#if UAV_CAGE_TEST_ARM_ENABLE
        /* A trip can occur before the Blackbox post-trigger window ends,
         * or while the IMU is invalid. Freeze pre-event samples immediately.
         * Transmission still waits until disarmed, KILL ON and motors zero. */
        const bool cage_trip_now = cage_guard.latched && !bb_prev_cage_latched;
        bb_prev_cage_latched = cage_guard.latched;
        if (cage_trip_now) UAVBB_FreezeNow();
#endif
    }
    /* This command service never runs in an IRQ and never waits for Wi-Fi.
     * RC data is still read by the normal SBUS failsafe path. */
    /* Observe a full 2-second stationary pose before an explicit capture.
     * Even a CONFIG request cannot sample while a motor is commanded. */
    const bool level_ref_safe = uav_config_allowed && imu_ready && imu_calibrated &&
        imu_control_valid && attitude_initialized &&
        (uint32_t)(now - imu_last_good_ms) <= IMU_MAX_AGE_MS &&
        !armed && !spinning &&
        (output_dshot[0] | output_dshot[1] | output_dshot[2] | output_dshot[3]) == 0u;
    UAVConfig_LevelObserve(roll_angle, pitch_angle,
        body_roll_rate_dps(), body_pitch_rate_dps(), body_yaw_rate_dps(),
        accel_tilt_confidence((float)accel_x,(float)accel_y,(float)accel_z),
        level_ref_safe);
    UAVConfig_Service(uav_config_allowed);
    UAVConfig_ServiceBoot(uav_config_allowed);

    if (barometer_ready) (void)DPS310_Update(&barometer);
    now = HAL_GetTick();
    (void)DPS310_GetData(&barometer, &baro_data);

    /* R5.7 flight telemetry: O is the actual four-channel DShot vector.
     * All PID200 and IMU1000 capture is retained after pilot DISARM + KILL.
     */
    if (UAV_SHARED_UART_PERIODIC_ASCII && !UAVBB_Dumping() &&
        (now - last_log_ms) >= LOG_PERIOD_MS) {
        const bool rc_neutral = debug_link_ok &&
            ((debug_roll > RC_ROLL_CENTER ? debug_roll - RC_ROLL_CENTER : RC_ROLL_CENTER - debug_roll) <= RC_STICK_DEADBAND) &&
            ((debug_pitch > RC_PITCH_CENTER ? debug_pitch - RC_PITCH_CENTER : RC_PITCH_CENTER - debug_pitch) <= RC_STICK_DEADBAND) &&
            ((debug_yaw > RC_YAW_CENTER ? debug_yaw - RC_YAW_CENTER : RC_YAW_CENTER - debug_yaw) <= RC_STICK_DEADBAND);
        const bool level_and_still = imu_ready && imu_calibrated &&
            ((now - imu_last_good_ms) <= IMU_MAX_AGE_MS) &&
            (fabsf(roll_angle) <= 2.0f) && (fabsf(pitch_angle) <= 2.0f) &&
            (fabsf(gyro_x_rate_dps()) <= 3.0f) &&
            (fabsf(gyro_y_rate_dps()) <= 3.0f) &&
            (fabsf(gyro_z_rate_dps()) <= 3.0f);
        /* L reflects valid SBUS transport and status flags, NOT direct RF telemetry.
         * A receiver that retransmits preset channels without flags cannot be
         * distinguished from live RC by the FC over SBUS alone.  PRE is only a
         * diagnostic hint based on the observed 1000/1000/1000 channel preset.
         * The inferred preset now REVOKES link_ok after debounce and LATCHES
         * the output lock; this is still NOT reliable RF telemetry. */
        const bool rx_fresh = (debug_frames_ok > 0u) && (debug_age_ms < SBUS_LINK_TIMEOUT_MS);
        const bool lost_flag = ((debug_sbus_flags & SBUS_FLAG_FRAME_LOST) != 0u);
        const bool failsafe_flag = ((debug_sbus_flags & SBUS_FLAG_FAILSAFE) != 0u);
        const uint8_t fs_reason = failsafe_flag ? 3u :
                                  (lost_flag ? 2u : (rx_fresh ? 0u : 1u));
        const bool preset_triplet = debug_rf_preset;
        /* FS=4 is a persistent inferred-pattern latch, FS=5 is startup latch.
         * Flags 0x04/0x08 and timeout remain independently visible. */
        const uint8_t fs_guard = debug_rf_latched ? debug_rf_guard_reason : fs_reason;
        last_log_ms = now;
        log_async("V%lu RX=%u L=%u FS=%u F=%02X AGE=%lu PRE=%u LAT=%u A=%u K=%u T=%u RC=%u/%u/%u N=%u S=%ld/%ld R=%ld P=%ld LV=%u G=%ld/%ld/%ld PID=%ld/%ld/%ld MIX=%ld/%ld/%ld/%ld O=%u/%u/%u/%u B=%u M=%u CMD=%u TX=%lu DE=%lu DH=%u LK=%u AR=%u SP=%u CAP=%u YQ=%u BLK=%03X RD=%u MAP=%u LS=%u LOW=%u FL=%u NE=%u IM=%u TZ=%u RAT=%ld BASE=%ld MS=%ld MO=%ld IDT=%lu MDT=%lu LATE=%lu IGAP=%lu IERR=%lu TLM=%lu TLD=%lu RT=%ld/%ld/%ld RE=%ld/%ld/%ld IT=%ld/%ld/%ld\r\n",
                  (unsigned long)(++log_sequence),
                  rx_fresh ? 1u : 0u, debug_link_ok ? 1u : 0u,
                  (unsigned)fs_guard, (unsigned)debug_sbus_flags, (unsigned long)debug_age_ms,
                  preset_triplet ? 1u : 0u, debug_rf_latched ? 1u : 0u,
                  (unsigned)debug_arm, (unsigned)debug_kill, (unsigned)debug_throttle,
                  (unsigned)debug_roll, (unsigned)debug_pitch, (unsigned)debug_yaw,
                  rc_neutral ? 1u : 0u,
                  (long)(sensor_roll_angle * 100.0f), (long)(sensor_pitch_angle * 100.0f),
                  (long)(roll_angle * 100.0f), (long)(pitch_angle * 100.0f),
                  level_and_still ? 1u : 0u,
                  (long)(body_roll_rate_dps()*10.0f),
                  (long)(body_pitch_rate_dps()*10.0f),
                  (long)(body_yaw_rate_dps()*10.0f),
                  (long)debug_pid_roll, (long)debug_pid_pitch, (long)debug_pid_yaw,
                  (long)debug_mix[0], (long)debug_mix[1],
                  (long)debug_mix[2], (long)debug_mix[3],
                  (unsigned)output_dshot[0], (unsigned)output_dshot[1],
                  (unsigned)output_dshot[2], (unsigned)output_dshot[3],
                  (unsigned)debug_bench_state, (unsigned)debug_bench_motor,
                  (unsigned)debug_bench_command, (unsigned long)dshot_tx_done,
                  (unsigned long)dshot_tx_errors, dshot_tx_recovered ? 1u : 0u,
                  UAV_BENCH_ARM_LOCK, armed ? 1u : 0u, spinning ? 1u : 0u,
                  (unsigned)UAV_EXPERIMENTAL_MAX_DSHOT,
                  (unsigned)yaw_check_status,
                  (unsigned)debug_arm_blockers, debug_arm_ready ? 1u : 0u,
                  debug_throttle_cal_ok ? 1u : 0u,
                  debug_link_stable ? 1u : 0u,
                  debug_low_hold ? 1u : 0u,
                  debug_level_ok ? 1u : 0u,
                  debug_neutral_ok ? 1u : 0u,
                  debug_imu_ok ? 1u : 0u,
                  (unsigned)throttle_zero,
                  (long)(debug_throttle_ratio * 1000.0f),
                  (long)debug_base_command,
                  (long)(debug_mixer_scale * 1000.0f),
                  (long)debug_mixer_offset,
                  (unsigned long)imu_dt_max_ms, (unsigned long)motor_dt_max_ms,
                  (unsigned long)motor_late_count, (unsigned long)imu_gap_count,
                  (unsigned long)imu_read_errors,
                  (unsigned long)UAVTelemetry_GetCompleted(),
                  (unsigned long)UAVTelemetry_GetDropped(),
                  (long)(flight_pid.rate_set_dps[0] * 10.0f),
                  (long)(flight_pid.rate_set_dps[1] * 10.0f),
                  (long)(flight_pid.rate_set_dps[2] * 10.0f),
                  (long)(flight_pid.rate_error_dps[0] * 10.0f),
                  (long)(flight_pid.rate_error_dps[1] * 10.0f),
                  (long)(flight_pid.rate_error_dps[2] * 10.0f),
                  (long)(flight_pid.pid_i[0] * 10.0f),
                  (long)(flight_pid.pid_i[1] * 10.0f),
                  (long)(flight_pid.pid_i[2] * 10.0f));
    }

    /* UART service: no waits in 1 ms PID/motor loop.
     * After PID200, upload IMU1000 (type 3) at up to 2 of every 3 slots;
     * retain a flight telemetry heartbeat in the third slot.
     * During PID200 Blackbox upload continue type-1 flight telemetry at
     * least every 60 ms (one out of 3 x 20 ms slots), and send type-2
     * records in the other slots. This prevents the Ground Station from
     * reporting an apparent disconnect during the ~15.4 s bulk download.
     * Both packet types are non-blocking and 88 bytes at 115200 baud. */
    now = HAL_GetTick();
    UAVBB_Tick(now);
    if ((uint32_t)(now - last_telemetry_ms) >= UAV_TELEMETRY_PERIOD_MS) {
        static uint8_t bb_upload_slot = 0u;
        last_telemetry_ms = now;
        UAVRateSample record;
        uint16_t record_index, record_count, capture_id;
        /* Never upload Blackbox while motor power is commanded.
         * In cage mode a latched guard prevents rearming during transfer;
         * normal disarmed download requires KILL ON. */
        const bool bb_download_safe = !armed && !spinning &&
          (output_dshot[0] | output_dshot[1] |
           output_dshot[2] | output_dshot[3]) == 0u &&
          ((debug_kill >= KILL_SWITCH_THRESHOLD) ||
           (UAV_CAGE_TEST_ARM_ENABLE && cage_guard.latched));
        if (bb_download_safe && UAVBB_Peek(&record, &record_index,
                                           &record_count, &capture_id)) {
            bb_upload_slot++;
            if (bb_upload_slot >= 3u) {
                bb_upload_slot = 0u;
                telemetry_publish(now);
            } else if (UAVTelemetry_SendBlackbox(&hlpuart1, &record,
                                                  record_index, record_count,
                                                  capture_id)) {
                UAVBB_Sent();
            }
        } else if (bb_download_safe) {
            UAVIMURawSample imu_record;
            uint16_t raw_idx, raw_count, raw_capture, raw_kind;
            if (UAVIMURaw_Peek(&imu_record, &raw_idx, &raw_count,
                               &raw_capture, &raw_kind)) {
                bb_upload_slot++;
                if (bb_upload_slot >= 3u) {
                    bb_upload_slot = 0u;
                    telemetry_publish(now);
                } else if (UAVTelemetry_SendIMURaw(&hlpuart1, &imu_record,
                            raw_idx, raw_count, raw_capture, raw_kind,
                            (uint16_t)(HAL_RCC_GetHCLKFreq()/1000000u))) {
                    UAVIMURaw_Sent();
                }
            } else {
                bb_upload_slot = 0u;
                telemetry_publish(now);
            }
        } else {
            bb_upload_slot = 0u;
            telemetry_publish(now);
        }
    }
}
