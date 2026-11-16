#include "uav_config_gate.h"
#include <assert.h>
#include <stdio.h>

static UAVConfigGateInputs good(void) {
    return (UAVConfigGateInputs){ .link_stable = true, .rf_guard_ok = true,
        .config_switch_on = true, .kill_on = true, .arm_on = false,
        .throttle_low = true, .armed = false, .spinning = false,
        .bench_in_progress = false };
}

int main(void) {
    UAVConfigGateState s = {0};
    UAVConfigGateInputs v = good();
    assert(!UAVConfigGate_Update(&s, 100u, &v));
    assert(!UAVConfigGate_Update(&s, 2099u, &v));
    assert(UAVConfigGate_Update(&s, 2100u, &v));

    /* Each hazard revokes write authorization immediately; two seconds
       of stable safe state are needed again afterwards. */
    for (int hazard=0; hazard<8; ++hazard) {
        v = good();
        switch (hazard) {
            case 0: v.link_stable = false; break;
            case 1: v.rf_guard_ok = false; break;
            case 2: v.config_switch_on = false; break;
            case 3: v.kill_on = false; break;
            case 4: v.arm_on = true; break;
            case 5: v.throttle_low = false; break;
            case 6: v.armed = true; break;
            case 7: v.spinning = true; break;
        }
        assert(!UAVConfigGate_Update(&s, 2200u, &v));
        v = good();
        assert(!UAVConfigGate_Update(&s, 2201u, &v));
        assert(!UAVConfigGate_Update(&s, 4200u, &v));
        assert(UAVConfigGate_Update(&s, 4201u, &v));
    }
    v = good(); v.bench_in_progress = true;
    assert(!UAVConfigGate_Update(&s, 4300u, &v));
    v = good();
    /* timestamp rollover: arithmetic is intentionally uint32_t modulo. */
    assert(!UAVConfigGate_Update(&s, 0xfffffff0u, &v));
    assert(!UAVConfigGate_Update(&s, 0x000007bf, &v));
    assert(UAVConfigGate_Update(&s, 0x000007c0, &v));
    assert(!UAVConfigGate_Update(0, 0u, &v));
    assert(!UAVConfigGate_Update(&s, 0u, 0));
    printf("six-channel RC CONFIG gate PASS\n");
    return 0;
}
