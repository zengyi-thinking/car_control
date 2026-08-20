#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#define HW_CONTROL_PERIOD_US                  5000U
#define HW_CONTROL_PERIOD_MS                  5U
#define HW_CONTROL_PERIOD_S                   0.005f

#define HW_MOTOR_DUTY_MIN                     0
#define HW_MOTOR_DUTY_MAX                     10000

#define HW_REMOTE_TIMEOUT_MS                  150U
#define HW_DRONE_TIMEOUT_MS                   280U
/* 视觉短时丢帧租约。租约内保持上一可靠方向，随后才平滑停车。 */
#define HW_DRONE_TRACK_LOST_GRACE_MS          1000U

#define HW_ENCODER_DISPLAY_DEADBAND           3

#define HW_WHEEL_RADIUS_M                     0.048f
#define HW_ENCODER_COUNTS_PER_WHEEL_REV       2048.0f

#define HW_MAX_LINEAR_SPEED_MPS               1.5f
/* 阶段2+ 反馈调优: 降低底盘前后加速度, 避免无人机因惯性被信号线拽住.
 * 原 3.5 m/s² → 现 2.0 m/s²: 从 0 到 1.2 m/s 需要 600ms (原 340ms),
 * 给无人机足够的跟随时间, 信号线不会被突然拉紧. */
#define HW_MAX_LINEAR_ACCEL_MPS2              2.0f
#define HW_MAX_YAW_RATE_RADPS                 2.4f
#define HW_MAX_WHEEL_SPEED_MPS                2.2f

#define HW_MANUAL_MAX_DUTY                    9000
#define HW_DRONE_TRACK_MAX_DUTY               9000
#define HW_DRONE_TRACK_DUTY_SLEW_PER_TICK     500.0f
#define HW_MANUAL_MAX_LINEAR_SPEED_MPS        1.10f
#define HW_MANUAL_MAX_LATERAL_SPEED_MPS       1.50f
#define HW_MANUAL_MAX_YAW_RATE_RADPS          1.20f

/* 自动追灯快速连续档。速度和加速度只恢复到旧完赛参数与当前保守档
 * 之间，先缩短起步迟滞，同时保留无人机追踪余量。 */
#define HW_DRONE_MAX_FWD_MPS                  1.40f
#define HW_DRONE_MAX_RIGHT_MPS                2.00f
#define HW_DRONE_TRACK_FILTER_ALPHA           0.35f
#define HW_DRONE_TRACK_SPEED_MPS              1.20f
#define HW_DRONE_TRACK_MIN_SPEED_MPS          0.52f
#define HW_DRONE_TRACK_NEAR_PX                35.0f
#define HW_DRONE_TRACK_FWD_SCALE              0.65f
#define HW_DRONE_TRACK_RIGHT_SCALE            1.00f
#define HW_DRONE_TRACK_ACCEL_MPS2             1.2f
/* 前后方向仍比横向温和，避免小车突然前冲把无人机拖出视场。 */
#define HW_DRONE_TRACK_FWD_ACCEL_MPS2         1.0f
#define HW_DRONE_NEAR_ENTER_PX                8.0f
#define HW_DRONE_NEAR_EXIT_PX                 12.0f
#define HW_DRONE_NEAR_PRESS_SPEED_MPS         0.45f
#define HW_DRONE_NEAR_PRESS_ACCEL_MPS2        0.80f
#define HW_DRONE_NEAR_PRESS_DIR_ALPHA         0.40f
#define HW_DRONE_NEAR_PRESS_DIR_MIN_PX        2.5f
#define HW_DRONE_NEAR_PRESS_DIR_MIN_DOT       0.15f
#define HW_DRONE_NEAR_PRESS_LOST_MS           250U
#define HW_DRONE_NEAR_PRESS_MAX_MS            1200U
/* 边缘信标丢失后的续航总时长，末段衰减包含在总时长内。 */
#define HW_DRONE_LOST_HOLD_MS                 HW_DRONE_TRACK_LOST_GRACE_MS
#define HW_DRONE_LOST_HOLD_SPEED_SCALE        1.00f
#define HW_DRONE_LOST_HOLD_FADE_MS            300U
#define HW_DRONE_V_CENTER_X_PX                94.0f
#define HW_DRONE_V_CENTER_Y_PX                60.0f
#define HW_DRONE_FWD_SIGN                     (1.0f)
/* Car reference frame for the drone link:
 *   +Y = forward, +X = right.
 * A4 carries raw V/B pixels; VBX drives +X and VBY drives +Y. */
#define HW_DRONE_RIGHT_SIGN                   (1.0f)
#define HW_DRONE_YAW_COMP_ENABLE             1U
#define HW_DRONE_YAW_COMP_SIGN               (1.0f)

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
