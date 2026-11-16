#define main test_original_r561_main
#include "test_flight.c"
#undef main
static void test_no_cage_stop_and_kill(void) {
    assert(UAV_CAGE_TEST_ARM_ENABLE==0);
    assert(UAV_YAW_DIAGNOSTIC_ARM_LOCK==0);
    assert(UAV_PROPS_OFF_ESC_TEST==0);
    assert(UAV_EXPERIMENTAL_MAX_DSHOT==DSHOT_THROTTLE_MAX);
    arm_and_spin();
    for (unsigned i=0;i<10000;i++) {
        step();
        assert(armed && spinning);
        assert(output_dshot[0] && output_dshot[1] && output_dshot[2] && output_dshot[3]);
    }
    assert(!cage_guard.latched);
    /* Strong yaw by itself must not trip the legacy 110 deg/s cage guard. */
    gyro_z = (int16_t)(-130.0f*GYRO_LSB_PER_DPS);
    for (unsigned i=0;i<250;i++) { step(); assert(armed && spinning); }
    gyro_z = 0;
    /* Pilot KILL must always disarm all outputs, even after 10 s. */
    sbus_channels[CH_KILL]=1800; step();
    assert(!armed && !spinning);zero();
    puts("PASS R5.7: no 5.5sec stop / no cage overspeed trip / manual KILL works");
}
static void test_manual_stop_blackbox(void) {
    arm_and_spin();
    UAVBB_Reset();UAVIMURaw_Reset();
    bb_prev_cage_powered=false;bb_last_imu_seq=0;bb_last_sample_ms=0;
    for (unsigned i=0;i<600;i++) {for (int j=0;j<5;j++) step(); bb_record_if_due(tick,false);}
    assert(bb_prev_cage_powered);
    sbus_channels[CH_KILL]=1800;step();bb_record_if_due(tick,true);
    assert(UAVBB_Dumping());
    assert(UAVBB_Total()==512);
    assert(UAVIMURaw_Dumping()==false); /* raw samples require IMU producer */
    puts("PASS R5.7: PID200 captures last samples when pilot stops");
}
int main(void) {
    test_original_r561_main();
    test_no_cage_stop_and_kill();
    test_manual_stop_blackbox();
    return 0;
}
