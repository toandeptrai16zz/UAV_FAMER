/*
 * uav_test.c
 * SBUS receiver -> four-channel PWM output at 400 Hz, plus MPU6500 SPI readout.
 *
 * IMPORTANT:
 * - This file outputs conventional PWM, NOT DShot.
 * - Remove all propellers during bench tests.
 * - CH_THR / CH_ARM / CH_KILL are zero-based SBUS channel indexes.
 * - Verify channel mapping and switch polarity before connecting motor power.
 */

#include "main.h"
#include "uav_test.h"

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef hlpuart1;
extern TIM_HandleTypeDef htim3;
extern SPI_HandleTypeDef hspi2;

/* ----------------------------- PWM config ----------------------------- */
#define PWM_FREQ_HZ                 400u
#define TIM3_CLK_HZ                 170000000u
#define PWM_TICK_HZ                 1000000u
#define PWM_MIN_US                  1000.0f
#define PWM_MAX_US                  1900.0f
#define PWM_OUTPUT_COUNT            4u
#define TEST_MOTOR_MASK             0x0Fu

/* Increase from zero equally on all motors. Tune only after bench validation. */
#define THR_FILTER_ALPHA            0.30f
#define SLEW_UP_RATIO_PER_S          5.0f

/* ----------------------------- SBUS config ---------------------------- */
#define SBUS_FRAME_SIZE             25u
#define SBUS_RX_BUFFER_SIZE         64u
#define SBUS_START_BYTE             0x0Fu
#define SBUS_LINK_TIMEOUT_MS        100u
#define SBUS_FLAG_FRAME_LOST        0x04u
#define SBUS_FLAG_FAILSAFE          0x08u

/* Preserve the input calibration from the supplied source. Adjust to measured endpoints. */
#define SBUS_MIN                    8u
#define SBUS_MAX                    1606u
#define SBUS_MID                    ((SBUS_MIN + SBUS_MAX) / 2u)

/* Zero-based channel indexes: index 2 = receiver channel 3, etc. */
#define CH_THR                      2u
#define CH_ARM                      4u
#define CH_KILL                     5u
#define ARM_SWITCH_THRESHOLD        SBUS_MID
#define KILL_SWITCH_THRESHOLD       1500u

/* ----------------------------- Arm safety ----------------------------- */
#define THR_DEADBAND                40.0f
#define THR_START_HYST              5.0f
#define THR_STOP_HYST               15.0f
#define THR_STABLE_BAND             30u
#define ESC_INIT_MS                 2500u
#define ARM_LOW_HOLD_MS             500u
#define LINK_STABLE_MS              1000u
#define LOG_PERIOD_MS               200u
#define IMU_SAMPLE_PERIOD_MS        1u   /* sensor ODR = 1 kHz */
#define SPI_TIMEOUT_MS              5u

/* Gyro +/-500 dps -> 65.5 LSB/dps; accel +/-4 g -> 8192 LSB/g */
#define GYRO_MDPS(raw_minus_bias)   (((int32_t)(raw_minus_bias) * 2000) / 131)
#define ACCEL_MG(raw)               (((int32_t)(raw) * 125) / 1024)

#define CS_LOW()  HAL_GPIO_WritePin(IMU_GPIO_Port, IMU_Pin, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(IMU_GPIO_Port, IMU_Pin, GPIO_PIN_SET)

/* ----------------------------- Logging -------------------------------- */
static char log_buffer[256];

static void log_async(const char *fmt, ...)
{
    int n;
    size_t tx_len;
    va_list ap;

    if (hlpuart1.gState != HAL_UART_STATE_READY) {
        return;
    }

    va_start(ap, fmt);
    n = vsnprintf(log_buffer, sizeof(log_buffer), fmt, ap);
    va_end(ap);

    if (n <= 0) {
        return;
    }

    /* vsnprintf returns the length that would have been written; clamp it. */
    tx_len = (size_t)n;
    if (tx_len >= sizeof(log_buffer)) {
        tx_len = sizeof(log_buffer) - 1u;
    }

    (void)HAL_UART_Transmit_IT(&hlpuart1, (uint8_t *)log_buffer, (uint16_t)tx_len);
}

/* ----------------------------- MPU6500 -------------------------------- */
static int16_t accel_x, accel_y, accel_z;
static int16_t gyro_x, gyro_y, gyro_z;
static bool imu_ready;
static bool gyro_cal_ok;
static uint32_t imu_read_errors;

static HAL_StatusTypeDef mpu_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {(uint8_t)(reg & 0x7Fu), value};
    HAL_StatusTypeDef status;

    CS_LOW();
    status = HAL_SPI_Transmit(&hspi2, tx, sizeof(tx), SPI_TIMEOUT_MS);
    CS_HIGH();
    return status;
}

static HAL_StatusTypeDef mpu_read_reg(uint8_t reg, uint8_t *value)
{
    uint8_t tx[2] = {(uint8_t)(reg | 0x80u), 0x00u};
    uint8_t rx[2] = {0u, 0u};
    HAL_StatusTypeDef status;

    if (value == NULL) {
        return HAL_ERROR;
    }

    CS_LOW();
    status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, sizeof(tx), SPI_TIMEOUT_MS);
    CS_HIGH();

    if (status == HAL_OK) {
        *value = rx[1];
    }
    return status;
}

static HAL_StatusTypeDef mpu_read_regs(uint8_t reg, uint8_t *buffer, uint16_t len)
{
    /* One CS-low transaction: address byte + len data bytes. */
    uint8_t tx[15] = {0};
    uint8_t rx[15] = {0};
    HAL_StatusTypeDef status;

    if ((buffer == NULL) || (len == 0u) || (len > 14u)) {
        return HAL_ERROR;
    }

    tx[0] = (uint8_t)(reg | 0x80u);

    CS_LOW();
    status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, (uint16_t)(len + 1u), SPI_TIMEOUT_MS);
    CS_HIGH();

    if (status == HAL_OK) {
        memcpy(buffer, &rx[1], len);
    }
    return status;
}

static bool mpu6500_init_spi(void)
{
    uint8_t who_am_i = 0u;

    CS_HIGH();
    HAL_Delay(10u);

    if (mpu_read_reg(0x75u, &who_am_i) != HAL_OK) {
        return false;
    }

    /* MPU-6500 WHO_AM_I is 0x70. Do not silently accept a different device. */
    if (who_am_i != 0x70u) {
        log_async("IMU ERROR: WHO_AM_I=0x%02X, expected 0x70\r\n", who_am_i);
        return false;
    }

    if (mpu_write_reg(0x6Bu, 0x80u) != HAL_OK) { /* Device reset */
        return false;
    }
    HAL_Delay(100u);

    /* SPI-mode recommendation: reset signal paths and disable the I2C interface. */
    if (mpu_write_reg(0x6Au, 0x11u) != HAL_OK) { /* I2C_IF_DIS | SIG_COND_RST */
        return false;
    }
    HAL_Delay(100u);

    if ((mpu_write_reg(0x6Bu, 0x01u) != HAL_OK) || /* PLL clock, wake up */
        (mpu_write_reg(0x6Cu, 0x00u) != HAL_OK) || /* all axes enabled */
        (mpu_write_reg(0x19u, 0x00u) != HAL_OK) || /* 1 kHz / (1+0) */
        (mpu_write_reg(0x1Au, 0x03u) != HAL_OK) || /* gyro DLPF 41 Hz */
        (mpu_write_reg(0x1Bu, 0x08u) != HAL_OK) || /* gyro +/-500 dps */
        (mpu_write_reg(0x1Cu, 0x08u) != HAL_OK) || /* accel +/-4 g */
        (mpu_write_reg(0x1Du, 0x03u) != HAL_OK))   /* accel DLPF 41 Hz */
    {
        return false;
    }

    /* Read back to make sure the writes really landed. */
    {
        uint8_t gcfg = 0u, acfg = 0u;
        if ((mpu_read_reg(0x1Bu, &gcfg) != HAL_OK) ||
            (mpu_read_reg(0x1Cu, &acfg) != HAL_OK) ||
            (gcfg != 0x08u) || (acfg != 0x08u)) {
            return false;
        }
    }

    HAL_Delay(10u);
    return true;
}

static bool mpu6500_read_spi(void)
{
    uint8_t buffer[14];

    if (mpu_read_regs(0x3Bu, buffer, sizeof(buffer)) != HAL_OK) {
        imu_read_errors++;
        return false;
    }

    accel_x = (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
    accel_y = (int16_t)(((uint16_t)buffer[2] << 8) | buffer[3]);
    accel_z = (int16_t)(((uint16_t)buffer[4] << 8) | buffer[5]);
    /* buffer[6..7] contains temperature. */
    gyro_x  = (int16_t)(((uint16_t)buffer[8]  << 8) | buffer[9]);
    gyro_y  = (int16_t)(((uint16_t)buffer[10] << 8) | buffer[11]);
    gyro_z  = (int16_t)(((uint16_t)buffer[12] << 8) | buffer[13]);

    return true;
}
#define GYRO_CAL_SAMPLES 1500u

static int32_t gyro_bias_x_raw;
static int32_t gyro_bias_y_raw;
static int32_t gyro_bias_z_raw;

#define GYRO_CAL_MAX_SPREAD_RAW 300  /* ~4.5 dps peak-to-peak: board was moved */

static bool mpu6500_calibrate_gyro(void)
{
    int64_t sum[3] = {0, 0, 0};
    int16_t vmin[3] = {INT16_MAX, INT16_MAX, INT16_MAX};
    int16_t vmax[3] = {INT16_MIN, INT16_MIN, INT16_MIN};
    uint32_t valid_samples = 0;
    uint8_t a;

    /* Board must be perfectly still; let the reset-button jolt settle. */
    HAL_Delay(1000u);

    for (uint32_t i = 0; i < GYRO_CAL_SAMPLES; i++) {
        if (mpu6500_read_spi()) {
            int16_t v[3] = {gyro_x, gyro_y, gyro_z};
            for (a = 0u; a < 3u; a++) {
                sum[a] += v[a];
                if (v[a] < vmin[a]) { vmin[a] = v[a]; }
                if (v[a] > vmax[a]) { vmax[a] = v[a]; }
            }
            valid_samples++;
        }
        HAL_Delay(2u);
    }

    if (valid_samples < (GYRO_CAL_SAMPLES * 95u) / 100u) {
        return false;
    }
    for (a = 0u; a < 3u; a++) {
        if ((vmax[a] - vmin[a]) > GYRO_CAL_MAX_SPREAD_RAW) {
            return false;
        }
    }

    gyro_bias_x_raw = (int32_t)(sum[0] / valid_samples);
    gyro_bias_y_raw = (int32_t)(sum[1] / valid_samples);
    gyro_bias_z_raw = (int32_t)(sum[2] / valid_samples);
    return true;
}

static bool mpu6500_calibrate_gyro_retry(void)
{
    uint8_t attempt;

    for (attempt = 0u; attempt < 3u; attempt++) {
        if (mpu6500_calibrate_gyro()) {
            return true;
        }
    }
    return false;
}

/* ----------------------------- SBUS ----------------------------------- */
static uint8_t sbus_rx_buffer[SBUS_RX_BUFFER_SIZE];
static uint8_t sbus_frame[SBUS_FRAME_SIZE];
static uint8_t sbus_frame_pos;
static volatile uint16_t sbus_channels[16];
static volatile uint8_t sbus_flags;
static volatile uint32_t sbus_frames_ok;
static volatile uint32_t sbus_frames_bad;
static volatile uint32_t sbus_uart_errors;
static volatile uint32_t sbus_last_valid_ms;
static volatile uint8_t sbus_rx_rearm_pending = 1u;

static bool sbus_footer_valid(uint8_t footer)
{
    /* SBUS1: 0x00; common SBUS2 footers: 0x04, 0x14, 0x24, 0x34.
       Some SBUS2/12-channel receivers use 0x08. */
    return (footer == 0x00u) ||
           (footer == 0x08u) ||
           ((footer & 0x0Fu) == 0x04u);
}

static void sbus_decode_frame(const uint8_t frame[SBUS_FRAME_SIZE])
{
    uint32_t accumulator = 0u;
    uint8_t available_bits = 0u;
    uint8_t index = 1u;
    uint8_t channel;

    for (channel = 0u; channel < 16u; ++channel) {
        while (available_bits < 11u) {
            accumulator |= ((uint32_t)frame[index++]) << available_bits;
            available_bits = (uint8_t)(available_bits + 8u);
        }
        sbus_channels[channel] = (uint16_t)(accumulator & 0x07FFu);
        accumulator >>= 11;
        available_bits = (uint8_t)(available_bits - 11u);
    }

    sbus_flags = frame[23];
    sbus_last_valid_ms = HAL_GetTick();
    sbus_frames_ok++;
}

static void sbus_parser_recover(void)
{
    uint8_t i;

    for (i = 1u; i < SBUS_FRAME_SIZE; ++i) {
        if (sbus_frame[i] == SBUS_START_BYTE) {
            uint8_t remaining = (uint8_t)(SBUS_FRAME_SIZE - i);
            memmove(sbus_frame, &sbus_frame[i], remaining);
            sbus_frame_pos = remaining;
            return;
        }
    }

    sbus_frame_pos = 0u;
}

static void sbus_feed_byte(uint8_t byte)
{
    if (sbus_frame_pos == 0u) {
        if (byte == SBUS_START_BYTE) {
            sbus_frame[0] = byte;
            sbus_frame_pos = 1u;
        }
        return;
    }

    sbus_frame[sbus_frame_pos++] = byte;
    if (sbus_frame_pos >= SBUS_FRAME_SIZE) {
        if ((sbus_frame[0] == SBUS_START_BYTE) && sbus_footer_valid(sbus_frame[24])) {
            sbus_decode_frame(sbus_frame);
            sbus_frame_pos = 0u;
        } else {
            sbus_frames_bad++;
            sbus_parser_recover();
        }
    }
}

static void sbus_service_receive_restart(void)
{
    if (sbus_rx_rearm_pending == 0u) {
        return;
    }
    if (huart1.RxState != HAL_UART_STATE_READY) {
        return;
    }

    __HAL_UART_CLEAR_FLAG(&huart1,
                          UART_CLEAR_PEF | UART_CLEAR_FEF |
                          UART_CLEAR_NEF | UART_CLEAR_OREF);

    if (HAL_UARTEx_ReceiveToIdle_IT(&huart1, sbus_rx_buffer,
                                    sizeof(sbus_rx_buffer)) == HAL_OK) {
        sbus_rx_rearm_pending = 0u;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    uint16_t i;

    if (huart != &huart1) {
        return;
    }

    if (size > sizeof(sbus_rx_buffer)) {
        size = sizeof(sbus_rx_buffer);
    }

    for (i = 0u; i < size; ++i) {
        sbus_feed_byte(sbus_rx_buffer[i]);
    }

    /* ReceiveToIdle_IT ends this transaction; restart it in the main loop. */
    sbus_rx_rearm_pending = 1u;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart1) {
        return;
    }

    sbus_uart_errors++;
    sbus_frame_pos = 0u;
    sbus_rx_rearm_pending = 1u;
}

/* ----------------------------- Motor control -------------------------- */
static uint32_t boot_ms;
static bool armed;
static bool arm_ready;
static bool spinning;
static bool link_prev;
static bool throttle_filter_initialized;
static bool low_tracking;
static uint32_t link_up_ms;
static uint32_t low_since_ms;
static uint16_t win_min;
static uint16_t win_max;
static uint16_t throttle_zero;
static float throttle_filtered;
static float throttle_ratio;
static float output_us[PWM_OUTPUT_COUNT] = {
    PWM_MIN_US, PWM_MIN_US, PWM_MIN_US, PWM_MIN_US
};

static uint32_t debug_age_ms;
static uint32_t debug_frames_ok;
static uint32_t debug_frames_bad;
static uint32_t debug_uart_errors;
static bool debug_link_ok;
static uint16_t debug_throttle;
static uint16_t debug_arm;
static uint16_t debug_kill;

static uint32_t pulse_us_to_ccr(float pulse_us)
{
    if (pulse_us < PWM_MIN_US) {
        pulse_us = PWM_MIN_US;
    } else if (pulse_us > PWM_MAX_US) {
        pulse_us = PWM_MAX_US;
    }
    return (uint32_t)(pulse_us + 0.5f);
}

static void sbus_snapshot(uint32_t *frames_ok,
                          uint32_t *last_valid_ms,
                          uint32_t *frames_bad,
                          uint32_t *uart_errors,
                          uint8_t *flags,
                          uint16_t channels[16])
{
    uint32_t primask = __get_PRIMASK();
    uint8_t i;

    __disable_irq();
    *frames_ok = sbus_frames_ok;
    *last_valid_ms = sbus_last_valid_ms;
    *frames_bad = sbus_frames_bad;
    *uart_errors = sbus_uart_errors;
    *flags = sbus_flags;
    for (i = 0u; i < 16u; ++i) {
        channels[i] = sbus_channels[i];
    }
    if (primask == 0u) {
        __enable_irq();
    }
}

static void set_all_motor_pulses(float pulse_us)
{
    uint32_t ccr = pulse_us_to_ccr(pulse_us);
    uint8_t i;

    for (i = 0u; i < PWM_OUTPUT_COUNT; ++i) {
        output_us[i] = (float)ccr;
    }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ccr);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ccr);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ccr);
}

static void motor_step(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - boot_ms;
    uint32_t frames_ok;
    uint32_t last_valid_ms;
    uint32_t frames_bad;
    uint32_t uart_errors;
    uint32_t age_ms;
    uint8_t flags;
    uint16_t channels[16];
    uint16_t ch_thr;
    uint16_t ch_arm;
    uint16_t ch_kill;
    bool link_ok;
    bool link_stable;
    bool throttle_low;
    bool stick_stable_low;
    bool arm_switch_on;
    bool kill_switch_on;
    float threshold;
    float denominator;
    float target_ratio = 0.0f;
    float step_up = SLEW_UP_RATIO_PER_S / (float)PWM_FREQ_HZ;
    float pulse_us;
    uint8_t i;

    sbus_snapshot(&frames_ok, &last_valid_ms, &frames_bad,
                  &uart_errors, &flags, channels);

    ch_thr = channels[CH_THR];
    ch_arm = channels[CH_ARM];
    ch_kill = channels[CH_KILL];
    age_ms = now - last_valid_ms;

    /* Stop on stale data, receiver failsafe, or a reported lost frame. */
    link_ok = (frames_ok > 0u) && (age_ms < SBUS_LINK_TIMEOUT_MS) &&
              ((flags & (SBUS_FLAG_FRAME_LOST | SBUS_FLAG_FAILSAFE)) == 0u);

    if (link_ok && !link_prev) {
        link_up_ms = now;
    }
    link_prev = link_ok;
    link_stable = link_ok && ((now - link_up_ms) >= LINK_STABLE_MS);

    if (!throttle_filter_initialized) {
        throttle_filtered = (float)ch_thr;
        throttle_filter_initialized = true;
    }
    throttle_filtered += THR_FILTER_ALPHA * ((float)ch_thr - throttle_filtered);

    throttle_low = (ch_thr <= (SBUS_MIN + ((SBUS_MAX - SBUS_MIN) / 5u)));
    if (!throttle_low) {
        low_tracking = false;
        low_since_ms = 0u;
    } else if (!low_tracking) {
        low_tracking = true;
        low_since_ms = now;
        win_min = ch_thr;
        win_max = ch_thr;
    } else {
        if (ch_thr < win_min) {
            win_min = ch_thr;
        }
        if (ch_thr > win_max) {
            win_max = ch_thr;
        }
        if ((uint16_t)(win_max - win_min) > THR_STABLE_BAND) {
            low_since_ms = now;
            win_min = ch_thr;
            win_max = ch_thr;
        }
    }
    stick_stable_low = low_tracking && ((now - low_since_ms) >= ARM_LOW_HOLD_MS);

    arm_switch_on = (ch_arm >= ARM_SWITCH_THRESHOLD);
    /* This project's switch convention: kill is ACTIVE when CH_KILL is high. */
    kill_switch_on = (ch_kill >= KILL_SWITCH_THRESHOLD);

    /* Mandatory switch sequence: arm switch LOW first, then HIGH to arm. */
    if ((elapsed < ESC_INIT_MS) || !link_ok || kill_switch_on) {
        armed = false;
        arm_ready = false;
    } else if (!arm_switch_on) {
        armed = false;
        arm_ready = true;
    } else if (!armed && arm_ready && link_stable && stick_stable_low) {
        armed = true;
        throttle_zero = win_max;
        throttle_ratio = 0.0f;
        spinning = false;
    }

    if (!armed) {
        spinning = false;
        throttle_ratio = 0.0f;
    } else {
        threshold = (float)throttle_zero + THR_DEADBAND;
        if (!spinning && throttle_filtered > threshold + THR_START_HYST) {
            spinning = true;
        } else if (spinning && throttle_filtered < threshold - THR_STOP_HYST) {
            spinning = false;
        }

        if (spinning) {
            denominator = (float)SBUS_MAX - threshold;
            if (denominator > 1.0f) {
                target_ratio = (throttle_filtered - threshold) / denominator;
            }
            if (target_ratio < 0.0f) {
                target_ratio = 0.0f;
            } else if (target_ratio > 1.0f) {
                target_ratio = 1.0f;
            }
        }

        /* Limit throttle ramp-up; reductions apply immediately. */
        if (target_ratio > throttle_ratio + step_up) {
            throttle_ratio += step_up;
        } else {
            throttle_ratio = target_ratio;
        }
    }

    /* Apply the same command to every enabled output; no per-motor offsets. */
    pulse_us = PWM_MIN_US + throttle_ratio * (PWM_MAX_US - PWM_MIN_US);
    for (i = 0u; i < PWM_OUTPUT_COUNT; ++i) {
        if (armed && spinning && ((TEST_MOTOR_MASK >> i) & 1u) != 0u) {
            output_us[i] = pulse_us;
        } else {
            output_us[i] = PWM_MIN_US;
        }
    }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_us_to_ccr(output_us[0]));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse_us_to_ccr(output_us[1]));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse_us_to_ccr(output_us[2]));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulse_us_to_ccr(output_us[3]));

    debug_age_ms = age_ms;
    debug_frames_ok = frames_ok;
    debug_frames_bad = frames_bad;
    debug_uart_errors = uart_errors;
    debug_link_ok = link_ok;
    debug_throttle = ch_thr;
    debug_arm = ch_arm;
    debug_kill = ch_kill;
}

/* ----------------------------- Public API ----------------------------- */
void uav_test_init(void)
{
    HAL_StatusTypeDef status;

    /* Start with the timer stopped and all compare values at minimum pulse. */
    __HAL_TIM_SET_PRESCALER(&htim3, (TIM3_CLK_HZ / PWM_TICK_HZ) - 1u);
    __HAL_TIM_SET_AUTORELOAD(&htim3, (PWM_TICK_HZ / PWM_FREQ_HZ) - 1u);
    __HAL_TIM_SET_COUNTER(&htim3, 0u);
    set_all_motor_pulses(PWM_MIN_US);
    htim3.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);

    /* Arm safety is switch-based; do not arm automatically on low throttle. */
    armed = false;
    arm_ready = false;
    spinning = false;
    link_prev = false;
    throttle_filter_initialized = false;
    low_tracking = false;
    throttle_ratio = 0.0f;
    sbus_frame_pos = 0u;
    sbus_rx_rearm_pending = 1u;

    /* Initialize IMU while PWM is still stopped. The ESC initialization timer
       starts only when valid minimum PWM pulses are about to be emitted. */
    imu_ready = mpu6500_init_spi();
    if (imu_ready) {
        gyro_cal_ok = mpu6500_calibrate_gyro_retry();
    }

    boot_ms = HAL_GetTick();
    status = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    if (status != HAL_OK) {
        Error_Handler();
    }
    status = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    if (status != HAL_OK) {
        set_all_motor_pulses(PWM_MIN_US);
        Error_Handler();
    }
    status = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    if (status != HAL_OK) {
        set_all_motor_pulses(PWM_MIN_US);
        Error_Handler();
    }
    status = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    if (status != HAL_OK) {
        set_all_motor_pulses(PWM_MIN_US);
        Error_Handler();
    }

    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    sbus_service_receive_restart();

    /* Combine startup status into one transmission: log_async intentionally
       refuses a second message while the previous interrupt TX is active. */
    if (imu_ready) {
        log_async("IMU OK cal=%u bias=%ld/%ld/%ld | READY: PWM %u Hz\r\n",
                  gyro_cal_ok ? 1u : 0u,
                  (long)gyro_bias_x_raw, (long)gyro_bias_y_raw, (long)gyro_bias_z_raw,
                  (unsigned)PWM_FREQ_HZ);
    } else {
        log_async("IMU ERROR: check SPI2/CS PB12/power/wiring | READY: PWM %u Hz\r\n",
                  (unsigned)PWM_FREQ_HZ);
    }
}

void uav_test_loop(void)
{
    static uint32_t last_log_ms;
    static uint32_t last_imu_ms;
    static uint8_t log_phase;
    static uint8_t log_sent_phase;
    uint32_t now;

    sbus_service_receive_restart();

    /* Timer update flag serves as the 400 Hz control-loop tick. */
    if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
        motor_step();
    }

    now = HAL_GetTick();
    if (imu_ready && ((now - last_imu_ms) >= IMU_SAMPLE_PERIOD_MS)) {
        last_imu_ms = now;
        (void)mpu6500_read_spi();
    }

    if ((now - last_log_ms) >= LOG_PERIOD_MS) {
        last_log_ms = now;
        log_phase ^= 1u;
    }

    /* One shared TX buffer: alternate the two lines on each tick so neither is dropped. */
    if (log_phase != log_sent_phase) {
        log_sent_phase = log_phase;
      if (log_phase == 0u) {
        log_async("LINK=%u age=%lu thr=%u arm=%u kill=%u ARM=%u R=%u "
                  "PWM=%u/%u/%u/%u frames=%lu bad=%lu uartErr=%lu\r\n",
                  debug_link_ok ? 1u : 0u,
                  (unsigned long)debug_age_ms,
                  (unsigned)debug_throttle,
                  (unsigned)debug_arm,
                  (unsigned)debug_kill,
                  armed ? 1u : 0u,
                  (unsigned)(throttle_ratio * 100.0f),
                  (unsigned)(output_us[0] + 0.5f),
                  (unsigned)(output_us[1] + 0.5f),
                  (unsigned)(output_us[2] + 0.5f),
                  (unsigned)(output_us[3] + 0.5f),
                  (unsigned long)debug_frames_ok,
                  (unsigned long)debug_frames_bad,
                  (unsigned long)debug_uart_errors);
      } else if (imu_ready) {
        log_async("IMU cal=%u gyro[mdps] %ld %ld %ld | accel[mg] %ld %ld %ld | readErr=%lu\r\n",
                  gyro_cal_ok ? 1u : 0u,
                  (long)GYRO_MDPS(gyro_x - gyro_bias_x_raw),
                  (long)GYRO_MDPS(gyro_y - gyro_bias_y_raw),
                  (long)GYRO_MDPS(gyro_z - gyro_bias_z_raw),
                  (long)ACCEL_MG(accel_x), (long)ACCEL_MG(accel_y), (long)ACCEL_MG(accel_z),
                  (unsigned long)imu_read_errors);
      }
    }
}
