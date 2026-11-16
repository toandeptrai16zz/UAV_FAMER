#include "uav_telemetry.h"
#include <stdio.h>
#include <assert.h>
#include <string.h>
static uint8_t last_packet[88];
int HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, const uint8_t *data, uint16_t size) {
    (void)uart; assert(size == 88); memcpy(last_packet, data, size);
    return HAL_OK;
}
int main(int argc,char **argv) {
    UART_HandleTypeDef uart={.gState = HAL_UART_STATE_READY};
    UAVIMURawSample s = {0};
    s.tick_ms=1234; s.cycle_count=0xffffff00u;
    s.accel[0]=100; s.accel[1]=-200; s.accel[2]=8180;
    s.gyro[0]=20; s.gyro[1]=-40; s.gyro[2]=99;
    assert(UAVTelemetry_SendIMURaw(&uart,&s,7,1024,2,UAV_IMU_RAW_CAPTURE_MOTOR,170));
    assert(argc==2);
    FILE *fp=fopen(argv[1],"wb"); assert(fp);
    assert(fwrite(last_packet,1,88,fp)==88); fclose(fp);
    puts("IMU raw C wire test PASS");
    return 0;
}
