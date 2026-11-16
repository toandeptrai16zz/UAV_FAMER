#include "uav_rate_blackbox.h"
#include <assert.h>
#include <stdio.h>
#include <stddef.h>

int main(void) {
    assert(sizeof(UAVRateSample) == 56u);
    UAVBB_Reset();
    UAVRateSample s = {0}, out;
    uint16_t idx, count, id;
    for (unsigned i=0;i<400u;i++) {
        s.stamp_ms = i*5u;
        s.gyro_x10[2] = 5;
        UAVBB_Push(&s, false);
    }
    assert(!UAVBB_Peek(&out,&idx,&count,&id));
    s.stamp_ms = 2000; s.gyro_x10[2] = 501;
    UAVBB_Push(&s, true);
    for (unsigned i=0;i<UAV_BB_POST_TRIGGER;i++) {
        s.stamp_ms += 5u;
        UAVBB_Push(&s, false);
    }
    assert(UAVBB_Dumping());
    unsigned n=0;
    uint32_t prev=0;
    while (UAVBB_Peek(&out,&idx,&count,&id)) {
        assert(count == 512u);
        assert(idx == n && id==1u);
        if (n) assert(out.stamp_ms >= prev);
        prev = out.stamp_ms;
        UAVBB_Sent();
        n++;
    }
    assert(n == 512u);
    assert(!UAVBB_Dumping());
    UAVBB_Tick(s.stamp_ms + 16000u);
    for (unsigned i=0;i<60u;i++) { s.stamp_ms+=5u; UAVBB_Push(&s,i==42); }
    assert(!UAVBB_Dumping()); /* still in post trigger */
    /* R5.2 regression: early abnormal cutoff must still generate a capture,
     * even before the normal 160 post-trigger samples arrive. */
    UAVBB_Reset();
    for (unsigned i=0u;i<80u;i++) {
        s.stamp_ms = 20000u + 5u*i;
        s.motor[0] = (i>=70u) ? 400u : 0u;
        UAVBB_Push(&s, i==70u);
    }
    assert(!UAVBB_Dumping());
    UAVBB_FreezeNow();
    assert(UAVBB_Dumping());
    assert(UAVBB_Peek(&out,&idx,&count,&id));
    assert(count==80u && id==1u && idx==0u);
    assert(out.stamp_ms==20000u);
    unsigned motor_records=0u;
    do {
        if (out.motor[0]!=0u) motor_records++;
        UAVBB_Sent();
    } while (UAVBB_Peek(&out,&idx,&count,&id));
    assert(motor_records==10u);
    assert(!UAVBB_Dumping());
    /* A guard trip while IMU sampling stops should still freeze pre-event
     * data without needing another sample. */
    UAVBB_Reset();
    s.stamp_ms=40000u;
    UAVBB_Push(&s,false);
    UAVBB_FreezeNow();
    assert(UAVBB_Peek(&out,&idx,&count,&id));
    assert(count==1u && id==1u);
    puts("PASS blackbox 200 Hz ring, early-cutout freeze, pre/post capture, transfer and rearm");
    return 0;
}
