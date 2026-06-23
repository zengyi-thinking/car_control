#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#define HW_CONTROL_PERIOD_US                  5000U
#define HW_CONTROL_PERIOD_MS                  5U
#define HW_CONTROL_PERIOD_S                   0.005f

#define HW_MOTOR_DUTY_MIN                     0
#define HW_MOTOR_DUTY_MAX                     10000

#define HW_REMOTE_TIMEOUT_MS                  150U
#define HW_DRONE_TIMEOUT_MS                   150U

#define HW_ENCODER_DISPLAY_DEADBAND           3

#define HW_WHEEL_RADIUS_M                     0.048f
#define HW_ENCODER_COUNTS_PER_WHEEL_REV       2048.0f

#define HW_MAX_LINEAR_SPEED_MPS               1.5f
#define HW_MAX_LINEAR_ACCEL_MPS2              3.5f
#define HW_MAX_YAW_RATE_RADPS                 2.4f
#define HW_MAX_WHEEL_SPEED_MPS                1.9f

#define HW_MANUAL_MAX_DUTY                    6000
#define HW_MANUAL_MAX_LINEAR_SPEED_MPS        0.80f
#define HW_MANUAL_MAX_LATERAL_SPEED_MPS       0.95f
#define HW_MANUAL_MAX_YAW_RATE_RADPS          1.20f

#define HW_DRONE_DEADBAND_PX                  4.0f
#define HW_DRONE_PX_NORM                      160.0f
#define HW_DRONE_MAX_FWD_MPS                  0.80f
#define HW_DRONE_MAX_RIGHT_MPS                0.80f
#define HW_DRONE_FWD_SIGN                     (1.0f)
/* Car reference frame for the drone link:
 *   +Y = forward, +X = right.
 * A4 err_fwd drives +Y, err_right drives +X. */
#define HW_DRONE_RIGHT_SIGN                   (1.0f)
#define HW_DRONE_REMOTE_OVERRIDE_RAW          260
/* Beacon pass-over coast: when the beacon disappears (move_valid 1->0) but the
 * drone link is still fresh, the car was physically over the beacon. Hold the
 * last velocity briefly so the car clears the beacon and extinguishes it. */
#define HW_BEACON_COAST_MS                    200U
#define HW_BEACON_COAST_MIN_VEL_MPS           0.10f

#define HW_WHEEL_TUNE_TARGET_MPS              0.12f
#define HW_WHEEL_TUNE_KP_STEP                 10.0f
#define HW_WHEEL_TUNE_KI_STEP                 5.0f
#define HW_WHEEL_TUNE_KD_STEP                 0.05f

#define HW_POSITION_EDIT_RATE_MPS             0.25f
#define HW_POSITION_EDIT_MAX_OFFSET_M         2.00f
#define HW_POSITION_TARGET_TOL_M              0.03f

/* 轻量静摩擦补偿，避免低速纯 PID 起步太软 */
#define HW_MOTOR_FF_KS_DUTY                   220.0f
#define HW_MOTOR_FF_KV_DUTY_PER_MPS           1200.0f
#define HW_MOTOR_FF_FADE_MPS                  0.05f   /* static-friction fade-in threshold */

/* =========================
 * IMU backend selection
 * ========================= */
#define HW_IMU_MODEL_ICM45686                 0U
#define HW_IMU_MODEL_IMU660RB                 1U

/* Default to the ICM45686 backend. */
#ifndef HW_IMU_MODEL
#define HW_IMU_MODEL                          HW_IMU_MODEL_ICM45686
#endif

/*
 * RB axis remap defaults.
 * Keep these as compile-time knobs so we can correct installation orientation
 * without touching AHRS or chassis code.
 */
#define HW_IMU660RB_ACC_X_SIGN                (1.0f)
#define HW_IMU660RB_ACC_Y_SIGN                (1.0f)
#define HW_IMU660RB_ACC_Z_SIGN                (1.0f)
#define HW_IMU660RB_GYRO_X_SIGN               (1.0f)
#define HW_IMU660RB_GYRO_Y_SIGN               (1.0f)
#define HW_IMU660RB_GYRO_Z_SIGN               (1.0f)

/* =========================
 * IMU drift suppression
 * ========================= */
#define HW_IMU_STATIONARY_GYRO_DPS_THRESHOLD   (1.20f)
#define HW_IMU_STATIONARY_ACC_DEV_G_THRESHOLD  (0.10f)
#define HW_IMU_STATIONARY_DWELL_S              (0.60f)

#define HW_IMU_ACC_WEIGHT_FULL_DEV_G           (0.06f)
#define HW_IMU_ACC_WEIGHT_MIN_DEV_G            (0.25f)
#define HW_IMU_ACC_WEIGHT_MIN                  (0.25f)

/* During straight translation, let the IMU keep trimming tiny z-rate bias
 * using encoder/odometry confirmation that the chassis is not actually turning.
 */
#define HW_IMU_MOTION_TRIM_TRANSLATION_MPS     (0.08f)
#define HW_IMU_MOTION_TRIM_OMEGA_RADPS         (0.05f)
#define HW_IMU_MOTION_TRIM_RATE_DEADBAND_DPS   (0.03f)
#define HW_IMU_MOTION_TRIM_GAIN                (0.25f)
#define HW_IMU_MOTION_TRIM_LIMIT_DPS           (0.40f)

#endif
