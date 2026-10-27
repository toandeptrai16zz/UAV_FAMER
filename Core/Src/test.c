/*
 * test_4.c  -  SBUS -> PWM 400 Hz (Thuật toán Shared Slope - Chuẩn bị cho PID)
 */

#include "main.h"
#include "uav_test.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef hlpuart1;
extern TIM_HandleTypeDef  htim3;

/* ========================= CẤU HÌNH ========================= */
#define TEST_MOTOR_MASK       0x0F

#define PWM_FREQ_HZ           400u
#define TIM3_CLK_HZ           170000000u
#define PWM_TICK_HZ           1000000u

#define PWM_MIN_US            1000.0f   /* Dừng hoàn toàn */
#define PWM_LIMIT_US          1900.0f   /* TRẦN 90% CÔNG SUẤT */

/* =====================================================================
 * HIỆU CHUẨN ĐIỂM CHẾT (MOTOR_START_US)
 * Đây là xung tối thiểu để từng motor bắt đầu nhích lên quay.
 * BẠN HÃY CHỈNH 4 CON SỐ NÀY CHO ĐẾN KHI CẢ 4 CON CÙNG NHÍCH LÊN MỘT LÚC.
 * (Tăng số của con nào đang quay sớm hơn, giảm số của con quay muộn hơn).
 * ===================================================================== */
static const float MOTOR_START_US[4] = {
    1180.0f,  /* M1 - PA6 */
    1160.0f,  /* M2 - PA4 */
    1100.0f,  /* M3 - PB0 */
    1140.0f   /* M4 - PB1 */
};

#define THR_FILTER_ALPHA      0.3f
#define SLEW_UP_RATIO_PER_S   5.0f

/* SBUS */
#define SBUS_FRAME_SIZE       25u
#define SBUS_RX_BUFFER_SIZE   64u
#define SBUS_START_BYTE       0x0Fu
#define SBUS_LINK_TIMEOUT_MS  100u
#define SBUS_FLAG_FAILSAFE    0x08u
#define SBUS_MIN              8u
#define SBUS_MAX              1606u
#define SBUS_MID              ((SBUS_MIN + SBUS_MAX) / 2u)

#define CH_THR                2u
#define CH_ARM                4u
#define CH_KILL               5u
#define USE_ARM_KILL_SWITCHES 1

/* Arm */
#define THR_DEADBAND          40.0f
#define THR_START_HYST        5.0f
#define THR_STOP_HYST         15.0f
#define THR_STABLE_BAND       30u
#define ESC_INIT_MS           2500u
#define ARM_LOW_HOLD_MS       500u
#define LINK_STABLE_MS        1000u
#define LOG_PERIOD_MS         200u

/* ========================= LOG ========================= */
static char log_buffer[256];
static void log_async(const char *fmt, ...) {
    if (hlpuart1.gState != HAL_UART_STATE_READY) return;
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(log_buffer, sizeof(log_buffer), fmt, ap);
    va_end(ap);
    if (n > 0) HAL_UART_Transmit_IT(&hlpuart1, (uint8_t *)log_buffer, (uint16_t)n);
}

/* ========================= SBUS ========================= */
static uint8_t sbus_rx_buffer[SBUS_RX_BUFFER_SIZE];
static uint8_t sbus_frame[SBUS_FRAME_SIZE];
static uint8_t sbus_frame_pos = 0;
static volatile uint16_t sbus_channels[16];
static volatile uint8_t  sbus_flags;
static volatile uint32_t sbus_frames_ok = 0, sbus_uart_errors = 0, sbus_last_valid_ms = 0;
static volatile uint8_t  sbus_rx_rearm_pending = 1;

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
    for (uint8_t i = 1; i < SBUS_FRAME_SIZE; ++i) {
        if (sbus_frame[i] == SBUS_START_BYTE) {
            uint8_t rem = (uint8_t)(SBUS_FRAME_SIZE - i);
            memmove(sbus_frame, &sbus_frame[i], rem);
            sbus_frame_pos = rem; return;
        }
    }
    sbus_frame_pos = 0;
}

static void sbus_feed_byte(uint8_t byte) {
    if (sbus_frame_pos == 0) {
        if (byte == SBUS_START_BYTE) { sbus_frame[0] = byte; sbus_frame_pos = 1; }
        return;
    }
    sbus_frame[sbus_frame_pos++] = byte;
    if (sbus_frame_pos >= SBUS_FRAME_SIZE) {
        if (sbus_frame[0] == SBUS_START_BYTE) { sbus_decode_frame(sbus_frame); sbus_frame_pos = 0; }
        else { sbus_parser_recover(); }
    }
}

static void sbus_service_receive_restart(void) {
    if (sbus_rx_rearm_pending == 0) return;
    if (huart1.RxState != HAL_UART_STATE_READY) return;
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_PEF|UART_CLEAR_FEF|UART_CLEAR_NEF|UART_CLEAR_OREF);
    if (HAL_UARTEx_ReceiveToIdle_IT(&huart1, sbus_rx_buffer, sizeof(sbus_rx_buffer)) == HAL_OK) sbus_rx_rearm_pending = 0;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
    if (huart != &huart1) return;
    if (size > sizeof(sbus_rx_buffer)) size = sizeof(sbus_rx_buffer);
    for (uint16_t i = 0; i < size; ++i) sbus_feed_byte(sbus_rx_buffer[i]);
    sbus_rx_rearm_pending = 1;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart != &huart1) return;
    sbus_uart_errors++;
    (void)HAL_UART_AbortReceive(&huart1);
    sbus_rx_rearm_pending = 1;
}

/* ========================= MOTOR ========================= */
static uint32_t boot_ms;
static bool     armed, arm_ready, spinning, link_prev, thr_f_init;
static uint32_t link_up_ms, low_since_ms;
static uint16_t win_min, win_max, thr_zero;
static float    thr_f, ratio_cur;
static float    out_us[4] = { PWM_MIN_US, PWM_MIN_US, PWM_MIN_US, PWM_MIN_US };

static int32_t  dbg_age;
static bool     dbg_link;
static uint16_t dbg_ch_thr, dbg_ch_arm, dbg_ch_kill;

static uint32_t us_to_ccr(float us) {
    if (us < PWM_MIN_US) return (uint32_t)PWM_MIN_US;
    if (us > PWM_LIMIT_US) return (uint32_t)PWM_LIMIT_US;
    return (uint32_t)(us + 0.5f);
}

static void motor_step(void) {
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - boot_ms;

    uint32_t ok_cnt  = sbus_frames_ok;
    uint32_t last_ms = sbus_last_valid_ms;
    uint8_t  flags   = sbus_flags;
    uint16_t ch_thr  = sbus_channels[CH_THR];
    uint16_t ch_arm  = sbus_channels[CH_ARM];
    uint16_t ch_kill = sbus_channels[CH_KILL];

    int32_t age = (int32_t)(now - last_ms);
    if (age < 0) age = 0;
    bool link_ok = (ok_cnt > 0u) && (age < (int32_t)SBUS_LINK_TIMEOUT_MS) && ((flags & SBUS_FLAG_FAILSAFE) == 0u);
    if (link_ok && !link_prev) link_up_ms = now;
    link_prev = link_ok;
    bool link_stable = link_ok && ((now - link_up_ms) >= LINK_STABLE_MS);

    if (!thr_f_init) { thr_f = (float)ch_thr; thr_f_init = true; }
    thr_f += THR_FILTER_ALPHA * ((float)ch_thr - thr_f);

    bool in_low_zone = (ch_thr < (SBUS_MIN + (SBUS_MAX - SBUS_MIN) / 5u));
    if (!in_low_zone) low_since_ms = 0;
    else if (low_since_ms == 0u) { low_since_ms = now; win_min = win_max = ch_thr; }
    else {
        if (ch_thr < win_min) win_min = ch_thr;
        if (ch_thr > win_max) win_max = ch_thr;
        if ((uint16_t)(win_max - win_min) > THR_STABLE_BAND) { low_since_ms = now; win_min = win_max = ch_thr; }
    }
    bool stick_stable_low = (low_since_ms != 0u) && ((now - low_since_ms) >= ARM_LOW_HOLD_MS);

#if USE_ARM_KILL_SWITCHES
    bool arm_sw  = (ch_arm  > SBUS_MID);
    bool kill_sw = (ch_kill > 1500);
    if (elapsed < ESC_INIT_MS || !link_ok || kill_sw) {
        armed = false; arm_ready = false;
    } else if (!arm_sw) {
        armed = false; arm_ready = true;
    } else if (!armed && arm_ready && link_stable && stick_stable_low) {
        armed = true; thr_zero = win_max;
    }
#else
    if (elapsed < ESC_INIT_MS || !link_ok) armed = false;
    else if (!armed && link_stable && stick_stable_low) { armed = true; thr_zero = win_max; }
#endif

    if (!armed) {
        spinning = false;
        ratio_cur = 0.0f;
    } else {
        float lo = (float)thr_zero + THR_DEADBAND;
        if (!spinning && thr_f > lo + THR_START_HYST) spinning = true;
        else if (spinning && thr_f < lo - THR_STOP_HYST) spinning = false;

        float target = 0.0f;
        if (spinning) {
            target = (thr_f - lo) / ((float)SBUS_MAX - lo);
            if (target < 0.0f) target = 0.0f;
            if (target > 1.0f) target = 1.0f;
        }

        float up = SLEW_UP_RATIO_PER_S / (float)PWM_FREQ_HZ;
        if (target > ratio_cur + up) ratio_cur += up;
        else ratio_cur = target;
    }

    /*
     * THUẬT TOÁN SHARED SLOPE (ĐỘ DỐC CHUNG)
     * Tìm điểm chết lớn nhất để làm mốc tính độ dốc.
     */
    float max_start = MOTOR_START_US[0];
    for (int i = 1; i < 4; i++) {
        if (MOTOR_START_US[i] > max_start) max_start = MOTOR_START_US[i];
    }

    // Độ dốc chung = Trần - Điểm chết lớn nhất
    float shared_slope = PWM_LIMIT_US - max_start;

    for (uint32_t i = 0; i < 4; i++) {
        if (armed && spinning && ((TEST_MOTOR_MASK >> i) & 1u)) {
            // Công thức: Điểm chết riêng + (Tỷ lệ * Độ dốc chung)
            out_us[i] = MOTOR_START_US[i] + (ratio_cur * shared_slope);

            // Chống tràn trần
            if (out_us[i] > PWM_LIMIT_US) out_us[i] = PWM_LIMIT_US;
        } else {
            out_us[i] = PWM_MIN_US;
        }
    }

    TIM3->CCR1 = us_to_ccr(out_us[0]);
    TIM3->CCR2 = us_to_ccr(out_us[1]);
    TIM3->CCR3 = us_to_ccr(out_us[2]);
    TIM3->CCR4 = us_to_ccr(out_us[3]);

    dbg_age = age; dbg_link = link_ok; dbg_ch_thr = ch_thr; dbg_ch_arm = ch_arm; dbg_ch_kill = ch_kill;
}

/* ========================= API ========================= */
void uav_test_init(void) {
    HAL_NVIC_SetPriority(SysTick_IRQn, 3, 0);
    HAL_NVIC_SetPriority(LPUART1_IRQn, 3, 0);
    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);

    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_AF_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_VERY_HIGH; g.Alternate = GPIO_AF2_TIM3;
    g.Pin = GPIO_PIN_4 | GPIO_PIN_6; HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_0 | GPIO_PIN_1; HAL_GPIO_Init(GPIOB, &g);

    __HAL_TIM_SET_PRESCALER(&htim3, TIM3_CLK_HZ / PWM_TICK_HZ - 1u);
    __HAL_TIM_SET_AUTORELOAD(&htim3, PWM_TICK_HZ / PWM_FREQ_HZ - 1u);

    uint32_t c_min = us_to_ccr(PWM_MIN_US);
    TIM3->CCR1 = c_min; TIM3->CCR2 = c_min; TIM3->CCR3 = c_min; TIM3->CCR4 = c_min;
    TIM3->EGR  = TIM_EGR_UG;

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);

    boot_ms = HAL_GetTick();
    sbus_frame_pos = 0; sbus_rx_rearm_pending = 1; sbus_service_receive_restart();
    log_async("READY: Shared Slope Mode (Ready for PID)\r\n");
}

void uav_test_loop(void) {
    static uint32_t t_log = 0;
    sbus_service_receive_restart();

    if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE)) {
        __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
        motor_step();
    }

    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - t_log) >= LOG_PERIOD_MS) {
        t_log = now;
        log_async("ARM=%u R=%u M1=%u M2=%u M3=%u M4=%u\r\n",
                  armed ? 1u : 0u, (unsigned)(ratio_cur * 100.0f),
                  (unsigned)(out_us[0] + 0.5f), (unsigned)(out_us[1] + 0.5f),
                  (unsigned)(out_us[2] + 0.5f), (unsigned)(out_us[3] + 0.5f));
    }
}
