#include "uav_rate_blackbox.h"
#include <stddef.h>
/* One producer: main loop; one consumer: same main loop.
 * 512 x 54 bytes ~27 KiB, SRAM only, no IRQ races.
 * Background continuous ring; freeze ~0.8 s after passive hand-motion trigger. */
static UAVRateSample samples[UAV_BB_SAMPLES];
static uint16_t head, valid_count, transfer_count, transfer_index, capture_id;
static uint16_t post_remaining;
static uint32_t cooldown_start_ms;
static enum { BB_COLLECT, BB_POST, BB_FROZEN, BB_COOLDOWN } state;

void UAVBB_Reset(void) {
    head = valid_count = transfer_count = transfer_index = 0u;
    post_remaining = 0u; capture_id = 0u; cooldown_start_ms = 0u;
    state = BB_COLLECT;
}
void UAVBB_Push(const UAVRateSample *s, bool trigger) {
    if (!s || (state != BB_COLLECT && state != BB_POST)) return;
    samples[head] = *s;
    head = (uint16_t)((head + 1u) % UAV_BB_SAMPLES);
    if (valid_count < UAV_BB_SAMPLES) ++valid_count;
    if (state == BB_COLLECT && trigger && valid_count >= 40u) {
        state = BB_POST;
        post_remaining = UAV_BB_POST_TRIGGER;
    } else if (state == BB_POST) {
        if (post_remaining > 0u) --post_remaining;
        if (!post_remaining) {
            state = BB_FROZEN;
            transfer_count = valid_count;
            transfer_index = 0u;
            ++capture_id;
        }
    }
}
void UAVBB_FreezeNow(void) {
    if ((state != BB_COLLECT && state != BB_POST) || valid_count == 0u) return;
    state = BB_FROZEN;
    transfer_count = valid_count;
    transfer_index = 0u;
    post_remaining = 0u;
    ++capture_id;
}

bool UAVBB_Peek(UAVRateSample *s, uint16_t *index, uint16_t *count, uint16_t *id) {
    if (!s || !index || !count || !id || state != BB_FROZEN ||
        transfer_index >= transfer_count) return false;
    uint16_t oldest = (uint16_t)((head + UAV_BB_SAMPLES - transfer_count) % UAV_BB_SAMPLES);
    *s = samples[(oldest + transfer_index) % UAV_BB_SAMPLES];
    *index = transfer_index;
    *count = transfer_count;
    *id = capture_id;
    return true;
}
void UAVBB_Sent(void) {
    if (state != BB_FROZEN || transfer_index >= transfer_count) return;
    ++transfer_index;
    if (transfer_index == transfer_count) {
        state = BB_COOLDOWN;
        cooldown_start_ms = samples[(head + UAV_BB_SAMPLES - 1u) % UAV_BB_SAMPLES].stamp_ms;
    }
}
void UAVBB_Tick(uint32_t now_ms) {
    if (state == BB_COOLDOWN && (uint32_t)(now_ms - cooldown_start_ms) >= 15000u) {
        /* User can initiate a new movement after cool-down; old capture was sent. */
        state = BB_COLLECT;
        head = valid_count = transfer_count = transfer_index = 0u;
        post_remaining = 0u;
    }
}
bool UAVBB_Dumping(void) { return state == BB_FROZEN; }
uint16_t UAVBB_Total(void) { return transfer_count; }
