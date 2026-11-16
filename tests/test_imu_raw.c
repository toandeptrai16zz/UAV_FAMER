#include "uav_imu_raw.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
int main(void) {
    UAVIMURawSample s = {0}, got = {0};
    uint16_t idx,total,cid,kind;
    UAVIMURaw_Reset();
    for (unsigned i=0;i<1024;i++) {
        s.tick_ms = i;
        s.cycle_count = i*170000u;
        s.accel[0] = (int16_t)i;
        UAVIMURaw_Push(&s);
    }
    assert(UAVIMURaw_Dumping());
    assert(UAVIMURaw_Total()==1024);
    assert(UAVIMURaw_Peek(&got,&idx,&total,&cid,&kind));
    assert(idx==0 && got.accel[0]==0 && kind==UAV_IMU_RAW_CAPTURE_BOOT);
    for(unsigned i=0;i<1024;i++) {
        assert(UAVIMURaw_Peek(&got,&idx,&total,&cid,&kind));
        assert(idx==i && got.accel[0]==(int16_t)i && kind==UAV_IMU_RAW_CAPTURE_BOOT);
        UAVIMURaw_Sent();
    }
    assert(!UAVIMURaw_Dumping());
    /* overrun retains last 1024 of 1500 motor samples */
    UAVIMURaw_MotorStarted();
    for (unsigned i=0;i<1500;i++) {s.tick_ms=i; s.accel[0]=i; UAVIMURaw_Push(&s);}
    UAVIMURaw_MotorStopped();
    assert(UAVIMURaw_Peek(&got,&idx,&total,&cid,&kind));
    assert(got.accel[0] == 476 && total==1024 && kind==UAV_IMU_RAW_CAPTURE_MOTOR);
    for(unsigned i=0;i<1024;i++) UAVIMURaw_Sent();
    assert(!UAVIMURaw_Dumping());
    /* motor start preempts unfinished or pending static reference */
    UAVIMURaw_Reset();
    for(unsigned i=0;i<20;i++) UAVIMURaw_Push(&s);
    UAVIMURaw_MotorStarted();
    s.accel[0]=77; UAVIMURaw_Push(&s); UAVIMURaw_MotorStopped();
    assert(UAVIMURaw_Peek(&got,&idx,&total,&cid,&kind));
    assert(total==1 && got.accel[0]==77 && kind==UAV_IMU_RAW_CAPTURE_MOTOR);
    puts("IMU raw buffer tests PASS");
    return 0;
}
