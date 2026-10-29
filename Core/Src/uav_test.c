/*
 * uav_test.c
 * SBUS -> 4-channel DShot300 (DMA-burst) + MPU6500 SPI + optional roll/pitch PID
 */

#include "main.h"
#include "uav_test.h"
#include "dshot_protocol.h"

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h> // Thêm thư viện toán học cho PID

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef hlpuart1;
extern TIM_HandleTypeDef htim3;
extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_tim3_up;

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

#define THR_FILTER_ALPHA            0.30f
#define SLEW_UP_RATIO_PER_S          1.0f  /* limit rise to full scale over at least ~1 s */

/* ----------------------------- SBUS config ---------------------------- */
#define SBUS_FRAME_SIZE             25u
#define SBUS_RX_BUFFER_SIZE         64u
#define SBUS_START_BYTE             0x0Fu
#define SBUS_LINK_TIMEOUT_MS        100u
#define SBUS_FLAG_FRAME_LOST        0x04u
#define SBUS_FLAG_FAILSAFE          0x08u

/*
 * Throttle calibration defaults are based on the values seen in the user's UART log.
 * Re-measure true stick endpoints on the actual receiver and update these constants.
 */
#define SBUS_THR_LOW_RAW             1115u
#define SBUS_THR_HIGH_RAW            1640u
#define SBUS_THR_LOW_FRACTION_PCT    20u

/* Standard raw SBUS defaults for roll/pitch stick mapping; calibrate if your RX differs. */
#define SBUS_STICK_MIN_RAW           172u
#define SBUS_STICK_MID_RAW           992u
#define SBUS_STICK_MAX_RAW           1811u

/* Zero-based SBUS channel indexes. Verify these against the transmitter monitor. */
#define CH_ROLL                      0u
#define CH_PITCH                     1u
#define CH_THR                       2u
#define CH_YAW                       3u
#define CH_ARM                       4u
#define CH_KILL                      5u

/* Thresholds retain the mapping used by the supplied project/log. Verify switch polarity. */
#define ARM_SWITCH_THRESHOLD         1320u  /* log shows 1000 in one state; verify ON/OFF raw values */
#define KILL_SWITCH_THRESHOLD        1500u

/* ----------------------------- Arm safety ----------------------------- */
#define THR_DEADBAND                40.0f
#define THR_START_HYST              5.0f
#define THR_STOP_HYST               15.0f
#define THR_STABLE_BAND             30u
#define ESC_INIT_MS                 2500u
#define ARM_LOW_HOLD_MS             500u
#define LINK_STABLE_MS              1000u
#define LOG_PERIOD_MS               200u
#define IMU_SAMPLE_PERIOD_MS        1u
#define IMU_MAX_AGE_MS               20u
#define SPI_TIMEOUT_MS              5u

/*
 * Keep attitude PID disabled for the first motor/ESC bench verification.
 * Set to 1 only after IMU axes, motor order/directions and gains are verified.
 */
#define ENABLE_ATTITUDE_PID          0u
#define PID_ANGLE_LIMIT_DEG          25.0f
#define PID_KP_ROLL                  8.0f   /* DShot counts per degree */
#define PID_KI_ROLL                  0.0f
#define PID_KD_ROLL                  2.0f   /* DShot counts per (degree/second) */
#define PID_KP_PITCH                 8.0f
#define PID_KI_PITCH                 0.0f
#define PID_KD_PITCH                 2.0f
#define PID_I_LIMIT                  20.0f
#define PID_OUTPUT_LIMIT             300.0f
#define GYRO_LSB_PER_DPS              65.5f /* MPU6500 configured for +/-500 dps */

/* Complementary filter coefficient used by the optional attitude controller. */
#define COMP_FILTER_ALPHA           0.98f

#define CS_LOW()  HAL_GPIO_WritePin(IMU_GPIO_Port, IMU_Pin, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(IMU_GPIO_Port, IMU_Pin, GPIO_PIN_SET)

/* --------------------------- DShot DMA state -------------------------- */
static uint16_t dshot_dma_buffer[DSHOT_DMA_CELLS][PWM_OUTPUT_COUNT];
static volatile uint32_t dshot_tx_errors;
static volatile uint32_t dshot_tx_done;
static volatile uint8_t dshot_dma_error_pending;

static void dshot_set_compare_zero(void);

static void dshot_dma_tx_complete(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    __HAL_TIM_DISABLE_DMA(&htim3, TIM_DMA_UPDATE);
    dshot_tx_done++;
}

static void dshot_dma_tx_error(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    __HAL_TIM_DISABLE_DMA(&htim3, TIM_DMA_UPDATE);
    dshot_set_compare_zero();
    dshot_tx_errors++;
    dshot_dma_error_pending = 1u;
}

/* ----------------------------- Logging -------------------------------- */
static char log_buffer[384];

static void log_async(const char *fmt, ...)
{
    int n;
    size_t length;
    va_list ap;

    if (hlpuart1.gState != HAL_UART_STATE_READY) return;

    va_start(ap, fmt);
    n = vsnprintf(log_buffer, sizeof(log_buffer), fmt, ap);
    va_end(ap);
    if (n <= 0) return;

    length = (size_t)n;
    if (length >= sizeof(log_buffer)) length = sizeof(log_buffer) - 1u;
    (void)HAL_UART_Transmit_IT(&hlpuart1, (uint8_t *)log_buffer, (uint16_t)length);
}

/* ----------------------------- MPU6500 -------------------------------- */
static int16_t accel_x, accel_y, accel_z;
static int16_t gyro_x, gyro_y, gyro_z;
static bool imu_ready;
static bool imu_calibrated;
static uint32_t imu_read_errors;
static uint32_t imu_last_good_ms;
static uint32_t imu_sample_sequence;
static uint32_t pid_last_sample_sequence;

static int32_t gyro_bias_x_raw, gyro_bias_y_raw, gyro_bias_z_raw;
static float roll_angle = 0.0f, pitch_angle = 0.0f;
static float roll_integral = 0.0f, pitch_integral = 0.0f;
static uint32_t last_pid_ms = 0;
static uint32_t last_attitude_update_ms = 0;

static HAL_StatusTypeDef mpu_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {(uint8_t)(reg & 0x7Fu), value};
    HAL_StatusTypeDef status;
    CS_LOW();
    status = HAL_SPI_Transmit(&hspi2, tx, sizeof(tx), SPI_TIMEOUT_MS);
    CS_HIGH();
    return status;
}

static HAL_StatusTypeDef mpu_read_regs(uint8_t reg, uint8_t *buffer, uint16_t len)
{
    HAL_StatusTypeDef status;
    uint8_t address = (uint8_t)(reg | 0x80u);
    uint8_t dummy_tx[14] = {0u};
    if ((buffer == NULL) || (len == 0u) || (len > sizeof(dummy_tx))) return HAL_ERROR;

    CS_LOW();
    status = HAL_SPI_Transmit(&hspi2, &address, 1u, SPI_TIMEOUT_MS);
    if (status == HAL_OK) {
        status = HAL_SPI_TransmitReceive(&hspi2, dummy_tx, buffer, len, SPI_TIMEOUT_MS);
    }
    CS_HIGH();
    return status;
}

static bool mpu6500_init_spi(void)
{
    uint8_t who_am_i = 0u;
    CS_HIGH();
    HAL_Delay(10u);
    if (mpu_read_regs(0x75u, &who_am_i, 1u) != HAL_OK) return false;
    if (who_am_i != 0x70u) return false;

    if (mpu_write_reg(0x6Bu, 0x80u) != HAL_OK) return false; /* reset */
    HAL_Delay(100u);
    if (mpu_write_reg(0x6Bu, 0x01u) != HAL_OK) return false; /* wake, gyro PLL */
    if (mpu_write_reg(0x19u, 0x00u) != HAL_OK) return false; /* divider 0 -> 1 kHz when DLPF is enabled */
    if (mpu_write_reg(0x1Au, 0x03u) != HAL_OK) return false; /* DLPF */
    if (mpu_write_reg(0x1Bu, 0x08u) != HAL_OK) return false; /* gyro +/-500 dps */
    if (mpu_write_reg(0x1Cu, 0x08u) != HAL_OK) return false; /* accel +/-4 g */
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
    gyro_x  = (int16_t)(((uint16_t)buffer[8]  << 8) | buffer[9]);
    gyro_y  = (int16_t)(((uint16_t)buffer[10] << 8) | buffer[11]);
    gyro_z  = (int16_t)(((uint16_t)buffer[12] << 8) | buffer[13]);
    imu_last_good_ms = HAL_GetTick();
    imu_sample_sequence++;
    return true;
}

static bool mpu6500_calibrate_gyro(void)
{
    int64_t sum_x = 0, sum_y = 0, sum_z = 0;
    uint32_t valid_samples = 0u;
    HAL_Delay(2000u); /* keep the board completely still during this period */

    for (uint32_t i = 0u; i < 1500u; ++i) {
        if (mpu6500_read_spi()) {
            sum_x += gyro_x;
            sum_y += gyro_y;
            sum_z += gyro_z;
            valid_samples++;
        }
        HAL_Delay(2u);
    }

    if (valid_samples < 1425u) return false; /* require >=95% successful samples */
    gyro_bias_x_raw = (int32_t)(sum_x / (int64_t)valid_samples);
    gyro_bias_y_raw = (int32_t)(sum_y / (int64_t)valid_samples);
    gyro_bias_z_raw = (int32_t)(sum_z / (int64_t)valid_samples);
    return true;
}

static float gyro_x_rate_dps(void)
{
    return (float)(gyro_x - gyro_bias_x_raw) / GYRO_LSB_PER_DPS;
}

static float gyro_y_rate_dps(void)
{
    return (float)(gyro_y - gyro_bias_y_raw) / GYRO_LSB_PER_DPS;
}

static void update_attitude(uint32_t dt_ms)
{
    float dt = (float)dt_ms / 1000.0f;
    float accel_roll;
    float accel_pitch;
    if (dt <= 0.0f) return;

    accel_roll = atan2f((float)accel_y, (float)accel_z) * 57.2957795f;
    accel_pitch = atan2f(-(float)accel_x,
                         sqrtf((float)accel_y * (float)accel_y +
                               (float)accel_z * (float)accel_z)) * 57.2957795f;

    /* MPU6500 FS_SEL=01 (+/-500 dps) -> 65.5 LSB/(degree/second). */
    roll_angle = COMP_FILTER_ALPHA * (roll_angle + gyro_x_rate_dps() * dt) +
                 (1.0f - COMP_FILTER_ALPHA) * accel_roll;
    pitch_angle = COMP_FILTER_ALPHA * (pitch_angle + gyro_y_rate_dps() * dt) +
                  (1.0f - COMP_FILTER_ALPHA) * accel_pitch;
}

#if ENABLE_ATTITUDE_PID
static float clampf_local(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float stick_to_angle_deg(uint16_t raw)
{
    if (raw >= SBUS_STICK_MID_RAW) {
        float den = (float)(SBUS_STICK_MAX_RAW - SBUS_STICK_MID_RAW);
        if (den < 1.0f) return 0.0f;
        return clampf_local(((float)raw - (float)SBUS_STICK_MID_RAW) / den,
                            0.0f, 1.0f) * PID_ANGLE_LIMIT_DEG;
    }
    {
        float den = (float)(SBUS_STICK_MID_RAW - SBUS_STICK_MIN_RAW);
        if (den < 1.0f) return 0.0f;
        return -clampf_local(((float)SBUS_STICK_MID_RAW - (float)raw) / den,
                             0.0f, 1.0f) * PID_ANGLE_LIMIT_DEG;
    }
}
#endif

/* ----------------------------- SBUS ----------------------------------- */
static uint8_t sbus_rx_buffer[SBUS_RX_BUFFER_SIZE];
static uint8_t sbus_frame[SBUS_FRAME_SIZE];
static uint8_t sbus_frame_pos;
static volatile uint16_t sbus_channels[16];
static volatile uint8_t sbus_flags;
static volatile uint32_t sbus_frames_ok, sbus_frames_bad, sbus_uart_errors, sbus_last_valid_ms;
static volatile uint8_t sbus_rx_rearm_pending = 1u;

static bool sbus_footer_valid(uint8_t footer) {
    return (footer == 0x00u) || (footer == 0x08u) || ((footer & 0x0Fu) == 0x04u);
}

static void sbus_decode_frame(const uint8_t frame[SBUS_FRAME_SIZE]) {
    uint32_t acc = 0; uint8_t avail = 0, idx = 1;
    for (uint8_t ch = 0; ch < 16; ++ch) {
        while (avail < 11) { acc |= ((uint32_t)frame[idx++]) << avail; avail += 8; }
        sbus_channels[ch] = (uint16_t)(acc & 0x07FFu);
        acc >>= 11; avail -= 11;
    }
    sbus_flags = frame[23];
    sbus_last_valid_ms = HAL_GetTick();
    sbus_frames_ok++;
}

static void sbus_parser_recover(void) {
    for (uint8_t i = 1u; i < SBUS_FRAME_SIZE; ++i) {
        if (sbus_frame[i] == SBUS_START_BYTE) {
            uint8_t rem = (uint8_t)(SBUS_FRAME_SIZE - i);
            memmove(sbus_frame, &sbus_frame[i], rem);
            sbus_frame_pos = rem; return;
        }
    }
    sbus_frame_pos = 0u;
}

static void sbus_feed_byte(uint8_t byte) {
    if (sbus_frame_pos == 0u) {
        if (byte == SBUS_START_BYTE) { sbus_frame[0] = byte; sbus_frame_pos = 1u; }
        return;
    }
    sbus_frame[sbus_frame_pos++] = byte;
    if (sbus_frame_pos >= SBUS_FRAME_SIZE) {
        if ((sbus_frame[0] == SBUS_START_BYTE) && sbus_footer_valid(sbus_frame[24])) {
            sbus_decode_frame(sbus_frame); sbus_frame_pos = 0u;
        } else { sbus_frames_bad++; sbus_parser_recover(); }
    }
}

static void sbus_service_receive_restart(void) {
    if (sbus_rx_rearm_pending == 0u || huart1.RxState != HAL_UART_STATE_READY) return;
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF);
    if (HAL_UARTEx_ReceiveToIdle_IT(&huart1, sbus_rx_buffer, sizeof(sbus_rx_buffer)) == HAL_OK) {
        sbus_rx_rearm_pending = 0u;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
    if (huart != &huart1) return;
    if (size > sizeof(sbus_rx_buffer)) size = sizeof(sbus_rx_buffer);
    for (uint16_t i = 0u; i < size; ++i) sbus_feed_byte(sbus_rx_buffer[i]);
    sbus_rx_rearm_pending = 1u;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart != &huart1) return;
    sbus_uart_errors++; sbus_frame_pos = 0u; sbus_rx_rearm_pending = 1u;
}

/* ----------------------------- DShot Logic ---------------------------- */
static void dshot_set_compare_zero(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0u);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0u);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0u);
}

static void dshot_prepare_buffer(const uint16_t command[PWM_OUTPUT_COUNT])
{
    uint16_t packet[PWM_OUTPUT_COUNT];
    for (uint8_t motor = 0u; motor < PWM_OUTPUT_COUNT; ++motor) {
        packet[motor] = dshot_make_packet(command[motor]);
    }

    /* Row = one DShot bit cell; four halfwords = CCR1..CCR4 DMA burst. */
    for (uint8_t bit = 0u; bit < DSHOT_PACKET_BITS; ++bit) {
        for (uint8_t motor = 0u; motor < PWM_OUTPUT_COUNT; ++motor) {
            uint8_t one = (uint8_t)((packet[motor] >> (15u - bit)) & 1u);
            dshot_dma_buffer[bit][motor] = one ? DSHOT_T1H_TICKS : DSHOT_T0H_TICKS;
        }
    }
    for (uint8_t motor = 0u; motor < PWM_OUTPUT_COUNT; ++motor) {
        dshot_dma_buffer[DSHOT_PACKET_BITS][motor] = 0u; /* trailing low cell */
    }
}

static bool dshot_send_all(const uint16_t command[PWM_OUTPUT_COUNT])
{
    DMA_HandleTypeDef *dma = &hdma_tim3_up;
    HAL_StatusTypeDef status;

    __HAL_TIM_DISABLE_DMA(&htim3, TIM_DMA_UPDATE);
    if (HAL_DMA_GetState(dma) != HAL_DMA_STATE_READY) {
        (void)HAL_DMA_Abort(dma);
        dshot_set_compare_zero();
        dshot_tx_errors++;
        return false;
    }

    dshot_prepare_buffer(command);
    dshot_set_compare_zero();
    __DMB(); /* complete SRAM writes before starting DMA on Cortex-M4 */

    dma->XferCpltCallback = dshot_dma_tx_complete;
    dma->XferErrorCallback = dshot_dma_tx_error;
    status = HAL_DMA_Start_IT(dma,
                              (uint32_t)(uintptr_t)&dshot_dma_buffer[0][0],
                              (uint32_t)(uintptr_t)&htim3.Instance->DMAR,
                              DSHOT_DMA_CELLS * PWM_OUTPUT_COUNT);
    if (status != HAL_OK) {
        (void)HAL_DMA_Abort(dma);
        dshot_set_compare_zero();
        dshot_tx_errors++;
        return false;
    }

    __HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_UPDATE);
    return true;
}

/* ----------------------------- Motor control -------------------------- */
static uint32_t boot_ms;
static bool armed, arm_ready, spinning, link_prev, throttle_filter_initialized, low_tracking;
static uint32_t link_up_ms, low_since_ms;
static uint16_t win_min, win_max, throttle_zero;
static float throttle_filtered, throttle_ratio;
static uint32_t debug_age_ms, debug_frames_ok, debug_frames_bad, debug_uart_errors;
static uint16_t debug_throttle, debug_arm, debug_kill;
static bool debug_link_ok, debug_imu_ok;
static uint16_t output_dshot[PWM_OUTPUT_COUNT] = {0u, 0u, 0u, 0u};

static void outputs_force_zero(void)
{
    for (uint8_t i = 0u; i < PWM_OUTPUT_COUNT; ++i) output_dshot[i] = 0u;
    dshot_set_compare_zero();
}

static void sbus_snapshot(uint32_t *frames_ok, uint32_t *last_valid_ms,
                          uint32_t *frames_bad, uint32_t *uart_errors,
                          uint8_t *flags, uint16_t channels[16])
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *frames_ok = sbus_frames_ok;
    *last_valid_ms = sbus_last_valid_ms;
    *frames_bad = sbus_frames_bad;
    *uart_errors = sbus_uart_errors;
    *flags = sbus_flags;
    for (uint8_t i = 0u; i < 16u; ++i) channels[i] = sbus_channels[i];
    if (primask == 0u) __enable_irq();
}

static bool throttle_calibration_valid(void)
{
    return (SBUS_THR_HIGH_RAW > SBUS_THR_LOW_RAW) &&
           ((SBUS_THR_HIGH_RAW - SBUS_THR_LOW_RAW) >= 100u);
}

static void motor_step(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - boot_ms;
    uint32_t frames_ok, last_valid_ms, frames_bad, uart_errors, age_ms;
    uint8_t flags;
    uint16_t channels[16];
    uint16_t ch_thr, ch_arm, ch_kill;
    bool link_ok, link_stable, throttle_low, stick_stable_low;
    bool arm_switch_on, kill_switch_on, imu_fresh, can_arm;
    float threshold = 0.0f, denominator, target_ratio = 0.0f;
    float step_up = SLEW_UP_RATIO_PER_S * ((float)DSHOT_CONTROL_PERIOD_MS / 1000.0f);
    float pid_roll = 0.0f, pid_pitch = 0.0f;

    sbus_snapshot(&frames_ok, &last_valid_ms, &frames_bad, &uart_errors, &flags, channels);

    if (dshot_dma_error_pending != 0u) {
        dshot_dma_error_pending = 0u;
        armed = false;
        arm_ready = false;
        spinning = false;
        throttle_ratio = 0.0f;
        outputs_force_zero();
    }

    ch_thr = channels[CH_THR];
    ch_arm = channels[CH_ARM];
    ch_kill = channels[CH_KILL];
    age_ms = now - last_valid_ms;

    link_ok = (frames_ok > 0u) && (age_ms < SBUS_LINK_TIMEOUT_MS) &&
              ((flags & (SBUS_FLAG_FRAME_LOST | SBUS_FLAG_FAILSAFE)) == 0u);
    if (link_ok && !link_prev) link_up_ms = now;
    link_prev = link_ok;
    link_stable = link_ok && ((now - link_up_ms) >= LINK_STABLE_MS);

    if (!throttle_calibration_valid()) {
        /* Invalid endpoints are a configuration fault: refuse to arm. */
        link_ok = false;
    }

    if (frames_ok > 0u) {
        if (!throttle_filter_initialized) {
            throttle_filtered = (float)ch_thr;
            throttle_filter_initialized = true;
        } else {
            throttle_filtered += THR_FILTER_ALPHA * ((float)ch_thr - throttle_filtered);
        }
    } else {
        throttle_filter_initialized = false;
        low_tracking = false;
    }

    throttle_low = (frames_ok > 0u) && link_ok &&
        ((uint32_t)ch_thr <= ((uint32_t)SBUS_THR_LOW_RAW +
          (((uint32_t)SBUS_THR_HIGH_RAW - (uint32_t)SBUS_THR_LOW_RAW) *
           SBUS_THR_LOW_FRACTION_PCT / 100u)));

    if (!throttle_low) {
        low_tracking = false;
        low_since_ms = 0u;
    } else if (!low_tracking) {
        low_tracking = true;
        low_since_ms = now;
        win_min = ch_thr;
        win_max = ch_thr;
    } else {
        if (ch_thr < win_min) win_min = ch_thr;
        if (ch_thr > win_max) win_max = ch_thr;
        if ((uint16_t)(win_max - win_min) > THR_STABLE_BAND) {
            low_since_ms = now;
            win_min = ch_thr;
            win_max = ch_thr;
        }
    }
    stick_stable_low = low_tracking && ((now - low_since_ms) >= ARM_LOW_HOLD_MS);

    arm_switch_on = (ch_arm >= ARM_SWITCH_THRESHOLD);
    kill_switch_on = (ch_kill >= KILL_SWITCH_THRESHOLD);
    imu_fresh = imu_ready && imu_calibrated && ((now - imu_last_good_ms) <= IMU_MAX_AGE_MS);
#if ENABLE_ATTITUDE_PID
    can_arm = imu_fresh;
#else
    can_arm = true; /* bench throttle test; PID is deliberately disabled by default */
#endif

    if ((elapsed < ESC_INIT_MS) || !link_ok || kill_switch_on || !can_arm) {
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
        roll_integral = 0.0f;
        pitch_integral = 0.0f;
        last_pid_ms = now;
        pid_last_sample_sequence = imu_sample_sequence;
    }

    if (!armed) {
        spinning = false;
        throttle_ratio = 0.0f;
        roll_integral = 0.0f;
        pitch_integral = 0.0f;
    } else {
        threshold = (float)throttle_zero + THR_DEADBAND;
        if (!spinning && throttle_filtered > (threshold + THR_START_HYST)) spinning = true;
        else if (spinning && throttle_filtered < (threshold - THR_STOP_HYST)) spinning = false;

        if (spinning) {
            denominator = (float)SBUS_THR_HIGH_RAW - threshold;
            if (denominator > 1.0f) target_ratio = (throttle_filtered - threshold) / denominator;
            target_ratio = (target_ratio < 0.0f) ? 0.0f : ((target_ratio > 1.0f) ? 1.0f : target_ratio);
        }
        if (target_ratio > throttle_ratio + step_up) throttle_ratio += step_up;
        else throttle_ratio = target_ratio;
    }

#if ENABLE_ATTITUDE_PID
    if (armed && imu_fresh) {
        uint32_t dt_ms = now - last_pid_ms;
        bool new_imu_sample = (imu_sample_sequence != pid_last_sample_sequence);
        float dt = 0.0f;
        if (new_imu_sample) {
            pid_last_sample_sequence = imu_sample_sequence;
            if (dt_ms > 0u && dt_ms <= 20u) {
                dt = (float)dt_ms / 1000.0f;
            }
            /* Rebase after a scheduling gap so the integrator can recover. */
            last_pid_ms = now;
        }
        {
            float roll_sp = stick_to_angle_deg(channels[CH_ROLL]);
            float pitch_sp = stick_to_angle_deg(channels[CH_PITCH]);
            float error_roll = roll_sp - roll_angle;
            float error_pitch = pitch_sp - pitch_angle;
            if (dt > 0.0f) {
                roll_integral = clampf_local(roll_integral + error_roll * dt, -PID_I_LIMIT, PID_I_LIMIT);
                pitch_integral = clampf_local(pitch_integral + error_pitch * dt, -PID_I_LIMIT, PID_I_LIMIT);
            }
            pid_roll = clampf_local(PID_KP_ROLL * error_roll + PID_KI_ROLL * roll_integral -
                                    PID_KD_ROLL * gyro_x_rate_dps(), -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
            pid_pitch = clampf_local(PID_KP_PITCH * error_pitch + PID_KI_PITCH * pitch_integral -
                                      PID_KD_PITCH * gyro_y_rate_dps(), -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
        }
    } else {
        roll_integral = 0.0f;
        pitch_integral = 0.0f;
    }
#endif

    /* QUAD-X mixer. Motor geometry, signs and motor order MUST be verified physically. */
    {
        float base = spinning && armed ?
            ((float)DSHOT_THROTTLE_MIN + throttle_ratio *
             (float)(DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN)) : 0.0f;
        float mixed[PWM_OUTPUT_COUNT];
        mixed[0] = base + pid_pitch - pid_roll;
        mixed[1] = base - pid_pitch - pid_roll;
        mixed[2] = base - pid_pitch + pid_roll;
        mixed[3] = base + pid_pitch + pid_roll;

        for (uint8_t i = 0u; i < PWM_OUTPUT_COUNT; ++i) {
            if (armed && spinning && ((TEST_MOTOR_MASK >> i) & 1u) != 0u) {
                float v = mixed[i];
                if (v < (float)DSHOT_THROTTLE_MIN) v = (float)DSHOT_THROTTLE_MIN;
                if (v > (float)DSHOT_THROTTLE_MAX) v = (float)DSHOT_THROTTLE_MAX;
                output_dshot[i] = (uint16_t)(v + 0.5f);
            } else {
                output_dshot[i] = 0u; /* DShot zero = stop */
            }
        }
    }

    if (!dshot_send_all(output_dshot)) {
        armed = false;
        arm_ready = false;
        spinning = false;
        throttle_ratio = 0.0f;
        outputs_force_zero();
    }

    debug_age_ms = age_ms;
    debug_frames_ok = frames_ok;
    debug_frames_bad = frames_bad;
    debug_uart_errors = uart_errors;
    debug_link_ok = link_ok;
    debug_imu_ok = imu_fresh;
    debug_throttle = ch_thr;
    debug_arm = ch_arm;
    debug_kill = ch_kill;
}

/* ----------------------------- Public API ----------------------------- */
void uav_test_init(void)
{
    __HAL_TIM_DISABLE_DMA(&htim3, TIM_DMA_UPDATE);
    __HAL_TIM_SET_PRESCALER(&htim3, 0u);
    __HAL_TIM_SET_AUTORELOAD(&htim3, DSHOT_TIMER_PERIOD_TICKS - 1u);

    /* Enable CCR preload and configure one TIM3_UP DMA burst over CCR1..CCR4. */
    SET_BIT(htim3.Instance->CCMR1, TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
    SET_BIT(htim3.Instance->CCMR2, TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE);
    htim3.Instance->DCR = TIM_DMABASE_CCR1 | TIM_DMABURSTLENGTH_4TRANSFERS;
    __HAL_TIM_SET_COUNTER(&htim3, 0u);
    outputs_force_zero();
    htim3.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);

    armed = false;
    arm_ready = false;
    spinning = false;
    link_prev = false;
    throttle_filter_initialized = false;
    low_tracking = false;
    throttle_ratio = 0.0f;
    imu_read_errors = 0u;
    imu_last_good_ms = 0u;
    imu_sample_sequence = 0u;
    pid_last_sample_sequence = 0u;
    dshot_tx_errors = 0u;
    dshot_tx_done = 0u;
    dshot_dma_error_pending = 0u;
    sbus_frame_pos = 0u;
    sbus_rx_rearm_pending = 1u;
    roll_integral = 0.0f;
    pitch_integral = 0.0f;
    roll_angle = 0.0f;
    pitch_angle = 0.0f;

    imu_ready = mpu6500_init_spi();
    imu_calibrated = false;
    if (imu_ready) imu_calibrated = mpu6500_calibrate_gyro();
    last_attitude_update_ms = HAL_GetTick();

#if ENABLE_ATTITUDE_PID
    if (!imu_ready || !imu_calibrated) {
        log_async("FAULT: PID locked; IMU init/calibration failed\r\n");
    }
#else
    if (!imu_ready) log_async("WARN: IMU unavailable; PID disabled; bench mode only\r\n");
    else if (!imu_calibrated) log_async("WARN: gyro calibration failed; PID must remain disabled\r\n");
#endif

    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

    boot_ms = HAL_GetTick();
    sbus_service_receive_restart();
    log_async("READY: SBUS -> 4x DShot300; PID=%u; wait ESC init\r\n", ENABLE_ATTITUDE_PID);
}

void uav_test_loop(void)
{
    static uint32_t last_motor_update_ms;
    static uint32_t last_log_ms;
    static uint32_t last_imu_ms;
    static uint32_t last_imu_log_ms = 500u;
    uint32_t now = HAL_GetTick();

    sbus_service_receive_restart();

    /* Sample the IMU before the control step so the PID sees the freshest valid sample. */
    if (imu_ready && ((now - last_imu_ms) >= IMU_SAMPLE_PERIOD_MS)) {
        last_imu_ms = now;
        if (mpu6500_read_spi() && imu_calibrated) {
            uint32_t sample_now = HAL_GetTick();
            uint32_t attitude_dt = sample_now - last_attitude_update_ms;
            if ((attitude_dt > 0u) && (attitude_dt <= IMU_MAX_AGE_MS)) {
                update_attitude(attitude_dt);
            }
            last_attitude_update_ms = sample_now;
        }
    }

    now = HAL_GetTick();
    if ((now - last_motor_update_ms) >= DSHOT_CONTROL_PERIOD_MS) {
        last_motor_update_ms = now;
        motor_step();
    }

    if ((now - last_log_ms) >= LOG_PERIOD_MS) {
        last_log_ms = now;
        log_async("LINK=%u age=%lu thr=%u armCH=%u killCH=%u ARM=%u R=%u "
                  "DSHOT=%u/%u/%u/%u txDone=%lu txErr=%lu frames=%lu bad=%lu "
                  "uartErr=%lu IMU=%u CAL=%u imuFresh=%u imuAge=%lu imuErr=%lu PID=%u\r\n",
                  debug_link_ok ? 1u : 0u,
                  (unsigned long)debug_age_ms,
                  (unsigned)debug_throttle,
                  (unsigned)debug_arm,
                  (unsigned)debug_kill,
                  armed ? 1u : 0u,
                  (unsigned)(throttle_ratio * 100.0f),
                  (unsigned)output_dshot[0], (unsigned)output_dshot[1],
                  (unsigned)output_dshot[2], (unsigned)output_dshot[3],
                  (unsigned long)dshot_tx_done,
                  (unsigned long)dshot_tx_errors,
                  (unsigned long)debug_frames_ok,
                  (unsigned long)debug_frames_bad,
                  (unsigned long)debug_uart_errors,
                  imu_ready ? 1u : 0u,
                  imu_calibrated ? 1u : 0u,
                  debug_imu_ok ? 1u : 0u,
                  (unsigned long)(imu_ready ? (now - imu_last_good_ms) : 0xFFFFFFFFu),
                  (unsigned long)imu_read_errors,
                  (unsigned)ENABLE_ATTITUDE_PID);
    }

    if (imu_ready && ((now - last_imu_log_ms) >= 1000u)) {
        last_imu_log_ms = now;
        log_async("ANGLE R=%d P=%d gyroRateX=%d gyroRateY=%d\r\n",
                  (int)roll_angle, (int)pitch_angle,
                  (int)gyro_x_rate_dps(), (int)gyro_y_rate_dps());
    }
}
