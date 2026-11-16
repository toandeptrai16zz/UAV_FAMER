#include "uav_imu_raw.h"
#include <stddef.h>

_Static_assert(sizeof(UAVIMURawSample) == 20u, "Unexpected raw IMU sample size");

static UAVIMURawSample ring[UAV_IMU_RAW_SAMPLES];
static uint16_t head, valid, tx_count, tx_index, capture_id, capture_kind;
static bool boot_capture, frozen, motor_seen;

void UAVIMURaw_Reset(void)
{
    head = valid = tx_count = tx_index = capture_id = capture_kind = 0u;
    boot_capture = true;
    frozen = motor_seen = false;
}

void UAVIMURaw_Push(const UAVIMURawSample *s)
{
    if (!s || frozen) return;
    ring[head] = *s;
    head = (uint16_t)((head + 1u) % UAV_IMU_RAW_SAMPLES);
    if (valid < UAV_IMU_RAW_SAMPLES) ++valid;
    /* Initial stationary reference: collects one second of real MPU reads.
     * If motors start first, OnMotorStarted cancels this reference capture. */
    if (boot_capture && valid == UAV_IMU_RAW_SAMPLES) {
        frozen = true;
        tx_count = valid;
        tx_index = 0u;
        capture_kind = UAV_IMU_RAW_CAPTURE_BOOT;
        ++capture_id;
    }
}

void UAVIMURaw_MotorStarted(void)
{
    if (motor_seen) return;
    motor_seen = true;
    /* Motor record takes priority over incomplete/pending boot reference.
     * Do not consume UART bandwidth while motors are active. */
    frozen = false;
    boot_capture = false;
    head = valid = tx_count = tx_index = 0u;
}

void UAVIMURaw_MotorStopped(void)
{
    if (!motor_seen) return;
    motor_seen = false;
    boot_capture = false;
    if (valid == 0u) return;
    frozen = true;
    tx_count = valid;
    tx_index = 0u;
    capture_kind = UAV_IMU_RAW_CAPTURE_MOTOR;
    ++capture_id;
}

bool UAVIMURaw_Peek(UAVIMURawSample *s, uint16_t *index,
                    uint16_t *total, uint16_t *id, uint16_t *kind)
{
    if (!s || !index || !total || !id || !kind || !frozen ||
        tx_index >= tx_count) return false;
    const uint16_t oldest = (uint16_t)((head + UAV_IMU_RAW_SAMPLES - tx_count) % UAV_IMU_RAW_SAMPLES);
    *s = ring[(oldest + tx_index) % UAV_IMU_RAW_SAMPLES];
    *index = tx_index;
    *total = tx_count;
    *id = capture_id;
    *kind = capture_kind;
    return true;
}

void UAVIMURaw_Sent(void)
{
    if (!frozen || tx_index >= tx_count) return;
    ++tx_index;
    if (tx_index == tx_count) {
        frozen = false;
        boot_capture = false;
        head = valid = tx_index = tx_count = 0u;
    }
}

bool UAVIMURaw_Dumping(void) { return frozen && tx_index < tx_count; }
uint16_t UAVIMURaw_Total(void) { return tx_count; }
