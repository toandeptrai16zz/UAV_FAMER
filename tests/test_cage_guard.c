#include "uav_cage_guard.h"
#include <assert.h>
#include <stdio.h>
#include <math.h>
static UAVCageInputs valid(void) {
 UAVCageInputs in={0}; in.now_ms=10000u;in.armed=true;in.spinning=true;in.input_ok=true;
 return in;
}
int main(void) {
 UAVCageGuard g;UAVCageInputs x=valid();
 UAVCage_Init(&g);assert(!UAVCage_Update(&g,&x));
 x.now_ms+=5000u;assert(!UAVCage_Update(&g,&x));
 x.now_ms+=500u;assert(UAVCage_Update(&g,&x));assert(g.latched&&g.fault==UAV_CAGE_TIMEOUT);
 x.now_ms+=10u; x.armed=false;assert(UAVCage_Update(&g,&x));
 UAVCage_Init(&g);x=valid();x.yaw_dps=111.0f;
 assert(!UAVCage_Update(&g,&x));x.now_ms+=79u;assert(!UAVCage_Update(&g,&x));
 x.now_ms++;assert(UAVCage_Update(&g,&x));assert(g.fault==UAV_CAGE_FAST_YAW);
 UAVCage_Init(&g);x=valid();x.roll_deg=35.0f;assert(UAVCage_Update(&g,&x));assert(g.fault==UAV_CAGE_ATTITUDE);
 UAVCage_Init(&g);x=valid();x.pitch_deg=23.0f;assert(!UAVCage_Update(&g,&x));
 x.now_ms+=80u;assert(UAVCage_Update(&g,&x));assert(g.fault==UAV_CAGE_ATTITUDE);
 UAVCage_Init(&g);x=valid();x.yaw_dps=240.0f;
 assert(UAVCage_Update(&g,&x));assert(g.fault==UAV_CAGE_RATE_HARD);
 UAVCage_Init(&g);x=valid();x.yaw_dps=NAN;
 assert(UAVCage_Update(&g,&x));assert(g.fault==UAV_CAGE_INPUT_FAULT);
 UAVCage_Init(&g);x=valid();assert(!UAVCage_Update(&g,&x));
 x.armed=false;x.spinning=false;assert(UAVCage_Update(&g,&x));assert(g.latched);
 UAVCage_Init(&g);x=valid();x.spinning=false;
 assert(!UAVCage_Update(&g,&x));assert(!g.active);
 x.armed=false;assert(!UAVCage_Update(&g,&x));assert(!g.latched);
 puts("PASS R5 captive one-shot timeout, yaw/attitude limits, input fault, no auto-rearm");
 return 0;
}
