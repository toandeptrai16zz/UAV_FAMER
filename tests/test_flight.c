/* Execute the actual application state machine with mocked peripherals.
 * This is a software regression test, NOT an ESC/IMU/airframe simulator. */
#include <assert.h>
#include <stdlib.h>
#include "../Core/Src/uav_test.c"
CoreDebug_Type mock_coredebug;
DWT_Type mock_dwt;
uint32_t HAL_RCC_GetHCLKFreq(void) { return 170000000u; }
static uint32_t tick;
static HAL_StatusTypeDef spi_result=HAL_OK;
static uint8_t mock_mpu_reg=0u;
static uint8_t mock_mpu_gyro_cfg=0u;
static bool mock_ignore_gyro_cfg_write=false;
static TIM_TypeDef timer;
UART_HandleTypeDef huart1,hlpuart1,huart3;
TIM_HandleTypeDef htim3={&timer};
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_tim3_up;
I2C_HandleTypeDef hi2c1;
/* UART bridge mocks: independent protocol module is tested separately. */
void UAVTelemetry_Reset(void) {}
bool UAVTelemetry_SendBlackbox(UART_HandleTypeDef *u, const UAVRateSample *s, uint16_t i, uint16_t n, uint16_t c) {
    (void)u; (void)s; (void)i; (void)n; (void)c; return true;
}
bool UAVTelemetry_SendIMURaw(UART_HandleTypeDef *u, const UAVIMURawSample *s,
                             uint16_t i, uint16_t n, uint16_t c, uint16_t k, uint16_t mhz) {
 (void)u; (void)s; (void)i; (void)n; (void)c; (void)k; (void)mhz; return true;
}
bool UAVTelemetry_Send(UART_HandleTypeDef *u, const UAVTelemetrySample *s) {(void)u;(void)s;return true;}
void UAVTelemetry_OnTxComplete(UART_HandleTypeDef *u) {(void)u;}
bool UAVTelemetry_OnTxError(UART_HandleTypeDef *u) {(void)u;return false;}
uint32_t UAVTelemetry_GetDropped(void) {return 0;}
uint32_t UAVTelemetry_GetCompleted(void) {return 0;}
void UAVConfig_Init(UART_HandleTypeDef *u) {(void)u;}
void UAVConfig_OnRxComplete(UART_HandleTypeDef *u) {(void)u;}
void UAVConfig_OnTxComplete(UART_HandleTypeDef *u) {(void)u;}
void UAVConfig_OnError(UART_HandleTypeDef *u) {(void)u;}
void UAVConfig_Service(bool safe) {(void)safe;}
void UAVConfig_ServiceBoot(bool safe) {(void)safe;}
/* Reference-pose config mocks (covered independently by config unit tests). */
float UAVConfig_LevelRoll(void) { return 0.0f; }
float UAVConfig_LevelPitch(void) { return 0.0f; }
void UAVConfig_LevelGet(float *r, float *p, bool *valid) { if(r)*r=0; if(p)*p=0; if(valid)*valid=false; }
void UAVConfig_LevelObserve(float r, float p, float gr, float gp, float gy, float conf, bool safe)
{ (void)r; (void)p; (void)gr; (void)gp; (void)gy; (void)conf; (void)safe; }

uint32_t HAL_GetTick(void){return tick;}
void HAL_Delay(uint32_t x){tick+=x;}
void HAL_GPIO_WritePin(int p,int n,int v){(void)p;(void)n;(void)v;}
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef*h,uint8_t*p,uint16_t n,uint32_t t){
 (void)h;(void)t;
 if(n==1) mock_mpu_reg=(uint8_t)(p[0]&0x7fu);
 else if(n==2 && p[0]==0x1bu && !mock_ignore_gyro_cfg_write) mock_mpu_gyro_cfg=p[1];
 return spi_result;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef*h,uint8_t*a,uint8_t*b,uint16_t n,uint32_t t){
 (void)h;(void)a;(void)t;memset(b,0,n);
 if(n==14)b[4]=0x20;
 if(n==1 && mock_mpu_reg==0x75u)b[0]=0x70u;
 if(n==1 && mock_mpu_reg==0x1bu)b[0]=mock_mpu_gyro_cfg;
 return spi_result;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef*h,uint8_t*p,uint16_t n){(void)h;(void)p;(void)n;return HAL_OK;}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef*h,uint8_t*p,uint16_t n){(void)h;(void)p;(void)n;return HAL_OK;}
int HAL_DMA_GetState(DMA_HandleTypeDef*h){(void)h;return HAL_DMA_STATE_READY;}
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef*h){(void)h;return HAL_OK;}
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef*h,uint32_t a,uint32_t b,uint32_t n){(void)a;(void)b;assert(n==68);h->XferCpltCallback(h);return HAL_OK;}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef*h,uint32_t c){(void)h;(void)c;return HAL_OK;}
void Error_Handler(void){abort();}
bool DPS310_Init(DPS310_HandleTypeDef*d,I2C_HandleTypeDef*h,uint8_t a){(void)d;(void)h;(void)a;return false;}
bool DPS310_Update(DPS310_HandleTypeDef*d){(void)d;return false;}
bool DPS310_GetData(const DPS310_HandleTypeDef*d,DPS310_Data*b){(void)d;memset(b,0,sizeof(*b));return false;}
bool MT06_Init(MT06_HandleTypeDef*d,UART_HandleTypeDef*h){(void)d;(void)h;return false;}
void MT06_Update(MT06_HandleTypeDef*d){(void)d;}
void MT06_RxEventCallback(UART_HandleTypeDef*h,uint16_t n){(void)h;(void)n;}
void MT06_ErrorCallback(UART_HandleTypeDef*h){(void)h;}
bool MT06_GetData(const MT06_HandleTypeDef*d,MT06_Data*b){(void)d;memset(b,0,sizeof(*b));return false;}
static void setup(void){
 tick=10000;boot_ms=0;armed=false;spinning=false;arm_ready=false;arm_switch_prev=false;
 UAVCage_Init(&cage_guard);bb_spinning_since=0u;bb_prev_cage_powered=false;
 UAVConfigGate_Reset(&uav_config_gate);uav_config_allowed=false;
 imu_ready=imu_calibrated=imu_control_valid=attitude_initialized=true;
 roll_angle=pitch_angle=0;gyro_x=gyro_y=gyro_z=0;
 gyro_bias_x_raw=gyro_bias_y_raw=gyro_bias_z_raw=0;
 imu_last_good_ms=tick;imu_sample_sequence=1;pid_last_sample_sequence=0;last_pid_ms=tick-1;
 link_prev=true;link_up_ms=tick-2000;low_tracking=true;low_since_ms=tick-1000;win_min=win_max=40;
 sbus_frames_ok=100;sbus_flags=0;sbus_last_valid_ms=tick;
 memset((void*)sbus_channels,0,sizeof(sbus_channels));
 sbus_channels[CH_ROLL]=RC_ROLL_CENTER;sbus_channels[CH_PITCH]=RC_PITCH_CENTER;sbus_channels[CH_YAW]=RC_YAW_CENTER;
 sbus_channels[CH_THR]=40;sbus_channels[CH_ARM]=1000;sbus_channels[CH_KILL]=1000;
 mc7re_guard_init(&mc7re_guard);mc7re_guard.latched=false;
 dshot_tx_done=1000;dshot_tx_errors=dshot_faults_observed=dshot_tx_done_at_fault=0;dshot_dma_error_pending=0;
 FlightPID_Reset(&flight_pid);FlightPIDSampleHold_Reset(&pid_sample_hold);
 outputs_force_zero();
}
static void step(void){tick++;imu_last_good_ms=tick;imu_sample_sequence++;sbus_last_valid_ms=tick;motor_step();}
static void arm_and_spin(void){setup();step();assert(arm_ready);sbus_channels[CH_ARM]=1800;step();assert(armed&&!spinning);sbus_channels[CH_THR]=800;step();assert(armed&&spinning);}
static void zero(void){for(int i=0;i<4;i++)assert(output_dshot[i]==0);}
static void test_arming(void){
 setup();sbus_channels[CH_ARM]=1800;step();assert(!armed);
 setup();step();sbus_channels[CH_ARM]=1800;roll_angle=20;step();assert(!armed);
 roll_angle=0;step();assert(!armed); /* regression: held ON used to arm here */
 sbus_channels[CH_ARM]=1000;step();sbus_channels[CH_ARM]=1800;step();assert(armed);
 arm_and_spin();sbus_channels[CH_KILL]=1800;step();assert(!armed);zero();
 sbus_channels[CH_KILL]=1000;step();assert(!armed);zero();
 puts("PASS ARM edge, failed attempt, normal spin, KILL, no automatic rearm");
}
static void test_faults(void){
 arm_and_spin();sbus_flags=SBUS_FLAG_FAILSAFE;step();assert(!armed&&mc7re_guard.latched);zero();
 arm_and_spin();tick+=101;imu_last_good_ms=tick;motor_step();assert(!armed);zero();
 arm_and_spin();sbus_channels[CH_ROLL]=sbus_channels[CH_PITCH]=sbus_channels[CH_YAW]=1000;sbus_channels[CH_THR]=40;step();assert(!armed);zero();
 arm_and_spin();dshot_dma_error_pending=1;dshot_tx_errors++;step();assert(!armed);zero();
 arm_and_spin();roll_angle=NAN;step();assert(!armed);zero();
 arm_and_spin();tick+=21;sbus_last_valid_ms=tick;motor_step();assert(!armed);zero();
 arm_and_spin();last_pid_ms=tick-30;step();assert(!armed);zero();
 puts("PASS SBUS failsafe/timeout/preset, DMA fault, invalid PID, PID time gap");
}
static void test_same_tick(void){
 arm_and_spin();last_pid_ms=tick;imu_sample_sequence++;motor_step();
 assert(armed&&spinning); /* zero dt keeps previous valid output */
 tick++;sbus_last_valid_ms=tick;last_attitude_update_ms=tick;
 uav_test_loop();assert(imu_control_valid&&armed&&spinning);
 puts("PASS same-ms sample timestamp retains control without false dropout");
}
static void test_imu_gap(void){
 arm_and_spin();last_attitude_update_ms=tick-30;tick++;sbus_last_valid_ms=tick;
 uav_test_loop();assert(!imu_control_valid&&!armed);zero();
 tick++;sbus_last_valid_ms=tick;uav_test_loop();assert(imu_control_valid&&!armed);zero();
 puts("PASS fresh SPI after >20 ms gap cannot mask stale attitude or rearm");
}
static void test_filter(void){
 accel_x=accel_y=0;accel_z=8192;gyro_x=gyro_y=gyro_z=0;gyro_bias_x_raw=gyro_bias_y_raw=gyro_bias_z_raw=0;
 attitude_initialized=true;sensor_roll_angle=10;sensor_pitch_angle=0;update_attitude(2);float v=sensor_roll_angle;
 sensor_roll_angle=10;update_attitude(1);update_attitude(1);assert(fabsf(v-sensor_roll_angle)<0.0001f);
 puts("PASS complementary decay equivalence at 1/2 ms for static gyro input");
}
static void test_mixer(void){
 float c[4],out[4],scale,offset;
 FlightPID_MixX(-60,0,0,c);assert(c[1]>c[0]&&c[3]>c[2]);
 FlightPID_MixX(0,60,0,c);assert(c[0]>c[2]&&c[1]>c[3]);
 for(int base=48;base<=2047;base+=7)for(int r=-200;r<=200;r+=40)for(int p=-200;p<=200;p+=40){
  FlightPID_MixX(r,p,170,c);assert(FlightPID_DesaturateMixer(base,c,48,2047,out,&scale,&offset));
  for(int i=0;i<4;i++){assert(out[i]>=48&&out[i]<=2047);for(int j=0;j<4;j++)assert(fabsf((out[i]-out[j])-(c[i]-c[j])*scale)<0.001f);}
 }
 c[0]=NAN;assert(!FlightPID_DesaturateMixer(100,c,48,2047,out,0,0));
 puts("PASS mixer directions, 34,606 saturation cases, invalid input");
}
static void test_protocol(void){
 for(unsigned value=0;value<=2047;value++){
  unsigned mapped=value?(value<48?48:value):0;
  unsigned packet=dshot_make_packet(value),payload=packet>>4;
  assert((payload>>1)==mapped && !(payload&1));
  assert(((packet>>12)^(packet>>8)^(packet>>4)^packet)%16==0);
 }
 assert(RC_ThrottleIsLow(40));assert(!RC_ThrottleIsLow(1640));
 assert(RC_StickNormalized(RC_ROLL_CENTER,RC_ROLL_CENTER)==0);
 uint8_t frame[25]={0};frame[0]=0x0f;
 uint16_t expected[16];
 for(unsigned ch=0;ch<16;ch++){
  expected[ch]=(uint16_t)((ch*131+40)&2047);
  for(unsigned b=0;b<11;b++)if(expected[ch]&(1u<<b))frame[1+(ch*11+b)/8]|=(uint8_t)(1u<<((ch*11+b)%8));
 }
 frame[23]=8;sbus_frame_pos=0;
 for(unsigned i=0;i<25;i++)sbus_feed_byte(frame[i]);
 for(unsigned ch=0;ch<16;ch++)assert(sbus_channels[ch]==expected[ch]);
 assert(sbus_flags==8);
 puts("PASS all DShot payloads/CRC, SBUS 16-channel decode, RC neutral/low throttle");
}
static void test_config_mode(void){
 /* No CH8 on the MC6C. CH5 ARM, CH6 KILL are the only safety switches. */
 setup();sbus_channels[CH_CONFIG]=1000;sbus_channels[CH_KILL]=1800;step();
 assert(!uav_config_allowed && !armed && !spinning); /* safe hold begins */
 tick+=2000;step();assert(uav_config_allowed);
 sbus_channels[CH_CONFIG]=1800;step();assert(uav_config_allowed); /* CH8 ignored in six-channel mode */
 sbus_channels[CH_KILL]=1000;step();assert(!uav_config_allowed);
 sbus_channels[CH_KILL]=1800;step();assert(!uav_config_allowed); /* 2s restart */
 tick+=2000;step();assert(uav_config_allowed);
 sbus_channels[CH_ARM]=1800;step();assert(!uav_config_allowed && !armed);
 sbus_channels[CH_ARM]=1000;step();assert(!uav_config_allowed);
 tick+=2000;step();assert(uav_config_allowed);
 sbus_channels[CH_THR]=800;step();assert(!uav_config_allowed);
 setup();step();sbus_channels[CH_ARM]=1800;step();assert(armed);
 sbus_channels[CH_CONFIG]=1800;step();assert(armed && !uav_config_allowed); /* no midflight disarm */
 puts("PASS six-channel PID config gate with 2s hold, safety locks and normal ARM");
}
static void test_20261105_ground_log_zero(void) {
 /* Known-level hypothesis from 2,283 stationary samples. Account for the
  * crossed sensor/airframe axis mapping, not the raw accel tilt names. */
 const float log_roll_mean = -1.2988f, log_pitch_mean = -2.81026f;
 const float corrected_roll = log_roll_mean + (IMU_PITCH_MOUNT_TRIM_DEG - (-1.06f));
 const float corrected_pitch = log_pitch_mean + (IMU_ROLL_MOUNT_TRIM_DEG - (-2.13f));
 assert(fabsf(corrected_roll) < 0.02f);
 assert(fabsf(corrected_pitch) < 0.02f);
 FlightPIDState state={0}; FlightPIDOutput o={0};
 FlightPIDInput in={0};
 in.roll_deg=corrected_roll;in.pitch_deg=corrected_pitch;
 in.gyro_roll_dps=-0.26636f;in.gyro_pitch_dps=0.06075f;
 in.gyro_yaw_dps=0.01297f;in.dt_s=0.001f;
 FlightPID_Reset(&state);
 assert(FlightPID_Step(&state,&in,&o));
 assert(fabsf(o.roll) < 1.0f && fabsf(o.pitch) < 1.0f && fabsf(o.yaw) < 1.0f);
 puts("PASS static zero correction & small stationary PID for remounted IMU");
}
static void test_runtime_gains(void){
 FlightPIDGains baseline,g;
 FlightPID_DefaultGains(&baseline); FlightPID_GetGains(&g);
 assert(fabsf(g.rate_kp[0]-1.5f)<0.001f && fabsf(g.angle_kp-3.0f)<0.001f);
 g.rate_kp[0] = 1.2f; assert(FlightPID_SetGains(&g));
 g.rate_ki[1] = NAN; assert(!FlightPID_SetGains(&g));
 FlightPID_GetGains(&g); assert(fabsf(g.rate_kp[0]-1.2f)<0.001f);
 assert(FlightPID_SetGains(&baseline));
 puts("PASS runtime PID defaults, bounds, NAN rejection, atomic apply");
}
static void test_acceleration_rejection(void){
    gyro_x=gyro_y=gyro_z=0;
    gyro_bias_x_raw=gyro_bias_y_raw=gyro_bias_z_raw=0;
    attitude_initialized=true;
    sensor_roll_angle=0.0f;sensor_pitch_angle=0.0f;
    /* 1.5g/30deg acceleration: must not be interpreted as tilt. */
    accel_x=0;accel_y=6144;accel_z=10642;
    assert(accel_tilt_confidence((float)accel_x,(float)accel_y,(float)accel_z)<0.001f);
    for (unsigned i=0;i<100;i++)update_attitude(1u);
    assert(fabsf(sensor_roll_angle)<0.001f);
    /* Real gravity-only 30-degree lean should converge without a 50ms snap. */
    accel_y=4096;accel_z=7094;
    assert(accel_tilt_confidence((float)accel_x,(float)accel_y,(float)accel_z)>0.99f);
    update_attitude(1u);
    assert(sensor_roll_angle>0.04f && sensor_roll_angle<0.10f);
    for (unsigned i=0;i<1000;i++)update_attitude(1u);
    assert(sensor_roll_angle>25.0f && sensor_roll_angle<30.5f);
    /* Invalid acceleration should not initialize attitude at boot. */
    attitude_initialized=false;accel_y=0;accel_z=15000;
    update_attitude(1u);assert(!attitude_initialized);
    puts("PASS acceleration norm rejection, slow gravity convergence, safe initialization");
}
static void test_rate_integral_near_level(void){
 FlightPIDState state={0}; FlightPIDInput in={0}; FlightPIDOutput o={0};
 FlightPIDGains g; FlightPID_DefaultGains(&g);
 assert(fabsf(g.rate_ki[0]-0.20f)<0.0001f);
 assert(fabsf(g.rate_ki[1]-0.20f)<0.0001f);
 assert(fabsf(g.rate_ki[2]-0.10f)<0.0001f);
 assert(fabsf(g.angle_kp-3.0f)<0.0001f);
 assert(FlightPID_SetGains(&g));
 in.dt_s=.001f; in.roll_deg=2.0f; in.allow_integrator=true;
 FlightPID_Reset(&state);
 for(int i=0;i<15000;i++) assert(FlightPID_Step(&state,&in,&o));
 assert(state.integrator[0] < -29.9f && state.integrator[0] >= -30.0f);
 assert(fabsf(state.pid_i[0]+6.0f)<0.001f);
 assert(fabsf(state.rate_set_dps[0]+6.0f)<0.001f);
 assert(fabsf(state.rate_error_dps[0]+6.0f)<0.001f);
 /* Fast transient must freeze the bias instead of deleting it. */
 in.allow_integrator=false; assert(FlightPID_Step(&state,&in,&o));
 assert(state.integrator[0] < -29.9f && fabsf(state.pid_i[0]+6.0f)<0.01f);
 /* Motor-off condition explicitly resets the stored correction. */
 in.reset_integrator=true; assert(FlightPID_Step(&state,&in,&o));
 assert(state.integrator[0]==0.0f && state.pid_i[0]==0.0f);
 in.reset_integrator=false;
 in.allow_integrator=true; in.roll_deg=0.0f;
 FlightPID_Reset(&state); assert(FlightPID_Step(&state,&in,&o));
 assert(fabsf(o.roll)<0.001f);
 puts("PASS tiny bounded near-level I, setpoint/error telemetry, freeze and motor-off reset");
}
static void test_rate_saturation_and_derivative_dt(void){
 FlightPIDGains baseline,g; FlightPID_DefaultGains(&baseline);
 g=baseline; g.rate_kp[0]=3.0f;g.rate_ki[0]=1.0f;g.rate_kd[0]=0.0f;
 assert(FlightPID_SetGains(&g));
 FlightPIDState s={0};FlightPIDInput in={0};FlightPIDOutput out={0};
 FlightPID_Reset(&s);in.dt_s=.001f;in.allow_integrator=true;
 in.gyro_roll_dps=-200.0f;
 for(int j=0;j<1000;j++)assert(FlightPID_Step(&s,&in,&out));
 assert(s.integrator[0]==0.0f && out.roll==g.correction_limit);
 in.gyro_roll_dps=0.0f;in.roll_deg=2.0f;
 for(int j=0;j<10;j++)assert(FlightPID_Step(&s,&in,&out));
 assert(s.integrator[0]<0.0f);
 FlightPID_Reset(&s);
 assert(s.integrator[0]==0.0f && !s.has_prev);
 /* D-path timing invariance for a linearly increasing gyro, 1 kHz vs 500 Hz. */
 g=baseline;g.rate_kd[0]=0.01f;g.rate_kp[0]=0.0f;g.rate_ki[0]=0.0f;
 assert(FlightPID_SetGains(&g));
 FlightPIDState a={0},b={0};FlightPIDOutput oa={0},ob={0};
 FlightPIDInput x={0},y={0};x.dt_s=.001f;y.dt_s=.002f;
 for(int i=0;i<1000;i++){
   x.gyro_roll_dps=(float)i*0.1f;
   assert(FlightPID_Step(&a,&x,&oa));
 }
 for(int i=0;i<500;i++){
   y.gyro_roll_dps=(float)i*0.2f;
   assert(FlightPID_Step(&b,&y,&ob));
 }
 assert(fabsf(a.gyro_deriv_filt[0]-b.gyro_deriv_filt[0])<1.0f);
 assert(fabsf(a.gyro_deriv_filt[0]-100.0f)<2.0f);
 assert(FlightPID_SetGains(&baseline));
 puts("PASS saturation antiwindup and dt-invariant D lowpass");
}

static void test_yaw_coupled_attitude(void){
 /* At 10 deg roll and 8 deg pitch, a +120 dps BODY yaw causes nonzero
  * Euler roll/pitch rate even when body p/q are zero.  Refuse gravity
  * correction for this synthetic sample; integrate exactly one 10ms tick. */
 const float rad=0.01745329251994329577f;
 const float initial_roll=10.0f, initial_pitch=8.0f, yaw=120.0f;
 accel_x=0;accel_y=0;accel_z=15000; /* |a|>1.3g -> reject accel */
 gyro_bias_x_raw=gyro_bias_y_raw=gyro_bias_z_raw=0;
 gyro_x=gyro_y=0;gyro_z=(int16_t)(-yaw*GYRO_LSB_PER_DPS);
 attitude_initialized=true;
 sensor_pitch_angle=IMU_PITCH_MOUNT_TRIM_DEG-initial_roll;
 sensor_roll_angle=IMU_ROLL_MOUNT_TRIM_DEG-initial_pitch;
 const float expected_roll=initial_roll + yaw*cosf(initial_roll*rad)*tanf(initial_pitch*rad)*0.010f;
 const float expected_pitch=initial_pitch - yaw*sinf(initial_roll*rad)*0.010f;
 update_attitude(10);
 assert(fabsf(roll_angle-expected_roll)<0.001f);
 assert(fabsf(pitch_angle-expected_pitch)<0.001f);
 assert(fabsf(sensor_pitch_angle-(IMU_PITCH_MOUNT_TRIM_DEG-roll_angle))<0.0001f);
 assert(fabsf(sensor_roll_angle-(IMU_ROLL_MOUNT_TRIM_DEG-pitch_angle))<0.0001f);
 puts("PASS yaw-coupled 3D attitude propagation and legacy sensor-axis mapping");
}
static void test_yaw_coupled_rate_setpoints(void){
 FlightPIDGains baseline;FlightPID_DefaultGains(&baseline);
 assert(FlightPID_SetGains(&baseline));
 FlightPIDState state={0};FlightPIDInput in={0};FlightPIDOutput out={0};
 in.dt_s=0.001f;in.roll_deg=10.0f;in.pitch_deg=8.0f;in.gyro_yaw_dps=120.0f;
 assert(FlightPID_Step(&state,&in,&out));
 const float phi=in.roll_deg*0.01745329251994329577f;
 const float theta=in.pitch_deg*0.01745329251994329577f;
 const float phi_dot=-baseline.angle_kp*in.roll_deg;
 const float theta_dot=-baseline.angle_kp*in.pitch_deg;
 const float q=(theta_dot+in.gyro_yaw_dps*sinf(phi))/cosf(phi);
 const float p=phi_dot-(q*sinf(phi)+in.gyro_yaw_dps*cosf(phi))*tanf(theta);
 assert(fabsf(state.rate_set_dps[0]-p)<0.01f);
 assert(fabsf(state.rate_set_dps[1]-q)<0.01f);
 assert(fabsf(state.rate_set_dps[2])<0.01f);
 /* With zero yaw and a level airframe, retain legacy rate setpoints. */
 in.roll_deg=0;in.pitch_deg=0;in.gyro_yaw_dps=0;
 assert(FlightPID_Step(&state,&in,&out));
 assert(fabsf(state.rate_set_dps[0])<0.001f);
 assert(fabsf(state.rate_set_dps[1])<0.001f);
 puts("PASS yaw-coupled Euler-target to body-rate mapping and level regression");
}
static void test_gyro_2000dps_fsr_and_yaw_mapping(void) {
 assert(MPU_GYRO_CONFIG_2000DPS == 0x18u);
 assert(mpu6500_init_spi());
 assert(mock_mpu_gyro_cfg == 0x18u);
 mock_mpu_gyro_cfg=0x08u;
 mock_ignore_gyro_cfg_write=true;
 assert(!mpu6500_init_spi()); /* rejected stale hardware FSR */
 mock_ignore_gyro_cfg_write=false;
 assert(mpu6500_init_spi());
 assert(GYRO_LSB_PER_DPS > 16.3f && GYRO_LSB_PER_DPS < 16.5f);
 assert(GYRO_CAL_STILLNESS_MAX_DPS == 4.0f);
 mock_ignore_gyro_cfg_write=false;mock_mpu_gyro_cfg=0x08u;
 assert(mpu6500_init_spi()); /* Valid WHO_AM_I and readback after write. */
 assert((mock_mpu_gyro_cfg & 0x18u)==0x18u);
 mock_ignore_gyro_cfg_write=true;mock_mpu_gyro_cfg=0x08u;
 assert(!mpu6500_init_spi()); /* Configuration mismatch must refuse IMU. */
 mock_ignore_gyro_cfg_write=false;
 gyro_bias_z_raw=0;
 gyro_z=(int16_t)(-1000.0f*GYRO_LSB_PER_DPS);
 assert(fabsf(body_yaw_rate_dps()-1000.0f) < 0.2f);
 gyro_z=(int16_t)(+1000.0f*GYRO_LSB_PER_DPS);
 assert(fabsf(body_yaw_rate_dps()+1000.0f) < 0.2f);
 gyro_z=0;
 puts("PASS MPU6500 +/-2000dps register/scale and nominal yaw sign conversion");
}
#if UAV_CAGE_TEST_ARM_ENABLE
static void test_r5_cage_motor_timeout_and_overspeed(void) {
 arm_and_spin();
 for (unsigned i=0; i<UAV_CAGE_MAX_ACTIVE_MS+20u && armed; i++) step();
 assert(!armed && cage_guard.latched && cage_guard.fault==UAV_CAGE_TIMEOUT);zero();
 sbus_channels[CH_ARM]=1000;step();sbus_channels[CH_ARM]=1800;step();
 assert(!armed && (debug_arm_blockers & 0x4000u));zero();
 arm_and_spin();
 gyro_z=(int16_t)(-125.0f*GYRO_LSB_PER_DPS);
 for(unsigned i=0;i<UAV_CAGE_SOFT_HOLD_MS+10u && armed;i++) step();
 assert(!armed && cage_guard.latched && cage_guard.fault==UAV_CAGE_FAST_YAW);zero();
 puts("PASS R5 motor command cutoff: timer / yaw limit and no repeated ARM");
}
static void test_r5_blackbox_short_powered_cutout(void) {
 arm_and_spin();
 UAVBB_Reset();bb_last_imu_seq=0u;bb_last_sample_ms=0u;
 /* Turn throttle on, then KILL: capture should still trigger at motor-off edge
  * even if no yaw excursion and test duration less than 1.2 s. */
 for (int j=0;j<70;j++) {
   for(int i=0;i<5;i++) step();
   bb_record_if_due(tick,false);
 }
 assert(bb_prev_cage_powered);
 sbus_channels[CH_KILL]=1800;step();
 bb_record_if_due(tick,true);
 assert(!armed && !spinning && cage_guard.latched);
 for(int j=0;j<UAV_BB_POST_TRIGGER+3;j++) {
   for(int i=0;i<5;i++) step();
   bb_record_if_due(tick,true);
 }
 assert(UAVBB_Dumping());
 UAVRateSample record;uint16_t i,n,id;bool powered=false,unpowered=false;
 while(UAVBB_Peek(&record,&i,&n,&id)) {
   if(record.flags&2u) powered=true;
   if(!(record.motor[0]|record.motor[1]|record.motor[2]|record.motor[3])) unpowered=true;
   UAVBB_Sent();
 }
 assert(powered && unpowered);
 puts("PASS R5 motor-on DShot history, short run cutout, deferred captured dump");
}
#endif
int main(void){
 test_gyro_2000dps_fsr_and_yaw_mapping();
#if UAV_YAW_DIAGNOSTIC_ARM_LOCK
 setup();step();sbus_channels[CH_ARM]=1800;sbus_channels[CH_THR]=800;
 for(int i=0;i<100;i++)step();
 assert(!armed && !spinning);zero();
 assert((debug_arm_blockers & 0x2000u) != 0u);
 puts("PASS diagnostic lock prevents arming and all motor output, independent of RC");
#else
 test_yaw_coupled_attitude(); test_yaw_coupled_rate_setpoints();
 test_rate_integral_near_level(); test_rate_saturation_and_derivative_dt();
 test_runtime_gains(); test_20261105_ground_log_zero(); test_config_mode();
 test_protocol();test_arming();test_faults();test_imu_gap();test_same_tick();
 test_filter();test_acceleration_rejection();test_mixer();
 assert(UAV_EXPERIMENTAL_MAX_DSHOT == (UAV_CAGE_TEST_ARM_ENABLE ? 1550u : DSHOT_THROTTLE_MAX));
 puts("PASS R5 captive motor ceiling and normal safety regression");
#if UAV_CAGE_TEST_ARM_ENABLE
 test_r5_cage_motor_timeout_and_overspeed();
 test_r5_blackbox_short_powered_cutout();
#endif
#endif
 return 0;
}
