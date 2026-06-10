#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#define HW_CONTROL_PERIOD_US                  5000U
#define HW_CONTROL_PERIOD_MS                  5U
#define HW_CONTROL_PERIOD_S                   0.005f

#define HW_MOTOR_DUTY_MIN                     0
#define HW_MOTOR_DUTY_MAX                     10000

#define HW_REMOTE_TIMEOUT_MS                  150U
#define HW_NAV_TIMEOUT_MS                     120U

#define HW_ENCODER_DISPLAY_DEADBAND           3

#define HW_WHEEL_RADIUS_M                     0.048f
#define HW_ENCODER_COUNTS_PER_WHEEL_REV       2048.0f

#define HW_MAX_LINEAR_SPEED_MPS               1.5f
#define HW_MAX_LINEAR_ACCEL_MPS2              3.5f
#define HW_MAX_YAW_RATE_RADPS                 2.4f
#define HW_MAX_WHEEL_SPEED_MPS                1.9f

#define HW_MANUAL_MAX_DUTY                    2000
#define HW_MANUAL_MAX_LINEAR_SPEED_MPS        0.35f
#define HW_MANUAL_MAX_YAW_RATE_RADPS          1.20f

#define HW_WHEEL_TUNE_TARGET_MPS              0.12f
#define HW_WHEEL_TUNE_KP_STEP                 10.0f
#define HW_WHEEL_TUNE_KI_STEP                 5.0f
#define HW_WHEEL_TUNE_KD_STEP                 0.05f

// Temporary bring-up switch:
// 1 = manual mode bypasses wheel PID and drives motors in open loop.
// 0 = manual mode uses wheel PID closed loop.
#define HW_MANUAL_OPEN_LOOP_TEST              1

#define HW_MOTOR_FF_KS_DUTY                   250.0f
#define HW_MOTOR_FF_KV_DUTY_PER_MPS           1200.0f

#endif
