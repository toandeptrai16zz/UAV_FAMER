UAV_DATN2 — FLIGHT CANDIDATE

Mo project: UAV_DATN/UAV_.ioc (STM32CubeIDE / CubeMX)
Doc truoc: UAV_DATN/FLIGHT_CANDIDATE_VI.md

Ban nay da chuyen tu BENCH_PROPS_OFF sang FLIGHT_CANDIDATE:
  UAV_EXPERIMENTAL_ARM_ENABLE = 1
  UAV_BENCH_ARM_LOCK          = 0
  UAV_PROPS_OFF_ESC_TEST      = 0
  UAV_ENABLE_ALT_HOLD         = 0

ARM bay duoc mo; scanner da tat. Khong dong nghia PID da tuning hoac may da duoc chung nhan bay.
Phai kiem tra IMU/mixer sign va RF failsafe voi ca 4 canh quat thao roi moi lap canh.
