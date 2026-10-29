#ifndef UAV_TEST_H
#define UAV_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* SBUS/DShot300 bench controller + MPU6500; PID is disabled by default. */
void uav_test_init(void);
void uav_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif /* UAV_TEST_H */
