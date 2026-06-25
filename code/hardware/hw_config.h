#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#define HW_CONTROL_PERIOD_US                  5000U
#define HW_CONTROL_PERIOD_MS                  5U
#define HW_CONTROL_PERIOD_S                   0.005f

#define HW_MOTOR_DUTY_MIN                     0
#define HW_MOTOR_DUTY_MAX                     10000

#define HW_REMOTE_TIMEOUT_MS                  150U
#define HW_DRONE_TIMEOUT_MS                   280U

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

#define HW_DRONE_DEADBAND_PX                  5.0f
#define HW_DRONE_PX_NORM                      150.0f
#define HW_DRONE_MIN_FWD_MPS                  0.375f
#define HW_DRONE_MIN_RIGHT_MPS                0.375f
#define HW_DRONE_MAX_FWD_MPS                  0.94f
#define HW_DRONE_MAX_RIGHT_MPS                0.94f
#define HW_DRONE_TRACK_FILTER_ALPHA           0.50f
/* PD position loop gains for drone beacon tracking (no I term: mecanum has no
 * steady-state error to null, and integral windup risks overrunning the beacon).
 *   P : distance -> speed curve (see chassis_drone_distance_to_speed).
 *   D : -KD * d(pixel_err)/dt, damps overshoot and guarantees the V-beacon pixel
 *       distance decreases monotonically (brakes when the error starts growing). */
#define HW_DRONE_TRACK_KD_MPS_PER_PXPS        0.012f
#define HW_DRONE_NEAR_GAIN_BOOST_PX           40.0f
#define HW_DRONE_NEAR_GAIN_MULT               1.4f
#define HW_DRONE_FWD_SIGN                     (1.0f)
/* Car reference frame for the drone link:
 *   +Y = forward, +X = right.
 * A4 carries raw V/B pixels; VBX drives +X and VBY drives +Y. */
#define HW_DRONE_RIGHT_SIGN                   (1.0f)
#define HW_DRONE_REMOTE_OVERRIDE_RAW          260
/* Beacon pass-over coast: when the beacon disappears (move_valid 1->0) but the
 * drone link is still fresh, the car was physically over the beacon. Hold the
 * last closed-loop body velocity (no slew decay) so the car keeps moving to
 * clear and extinguish the beacon. Direction is the last closed-loop command,
 * not a frozen strike direction. Only coast if the car was actually moving.
 * If the held speed is below the dash floor, scale it up along its direction so
 * the car has enough momentum to overrun the lamp housing (avoids the chassis
 * getting caught on the lamp cover at low speed). */
#define HW_BEACON_COAST_MS                    750U
#define HW_BEACON_COAST_MIN_VEL_MPS           0.06f
#define HW_BEACON_COAST_DASH_FLOOR_MPS        0.60f
#define HW_BEACON_STRIKE_CROSS_ZONE_PX        64.0f

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

/* NOTE: 平移时学习陀螺z零偏的 HW_IMU_MOTION_TRIM_* 机制已移除 — 它与 IMU 自身的静止ZRU
 * 冲突造成双重扣零偏, 回静止后引发 yaw 匀速漂移(最高0.40dps≈80°/200s).
 * 无磁力计时z零偏仅在静止可观测, 完全依赖 hw_imu_update_bias() 的静止标定即可. */

#endif
