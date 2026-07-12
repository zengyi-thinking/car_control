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
#define HW_MAX_WHEEL_SPEED_MPS                2.2f

#define HW_MANUAL_MAX_DUTY                    9000
#define HW_DRONE_TRACK_MAX_DUTY               9000
#define HW_MANUAL_MAX_LINEAR_SPEED_MPS        1.10f
#define HW_MANUAL_MAX_LATERAL_SPEED_MPS       1.50f
#define HW_MANUAL_MAX_YAW_RATE_RADPS          1.20f

/* 前后速度提高到 1.2m/s, 侧向保留更高余量补偿麦轮横移效率。 */
#define HW_DRONE_MAX_FWD_MPS                  1.20f
#define HW_DRONE_MAX_RIGHT_MPS                2.00f
#define HW_DRONE_TRACK_FILTER_ALPHA           0.62f
/* 信标追踪采用外圈快追、近圈降速、中心短促压灯。 */
#define HW_DRONE_TRACK_SPEED_MPS              1.20f
#define HW_DRONE_TRACK_MIN_SPEED_MPS          0.55f
#define HW_DRONE_TRACK_NEAR_PX                35.0f
#define HW_DRONE_TRACK_FWD_SCALE              0.80f
#define HW_DRONE_TRACK_RIGHT_SCALE            1.12f
#define HW_DRONE_TRACK_ACCEL_MPS2             6.0f
#define HW_DRONE_CENTER_HOLD_PX               8.0f
/* 踩灭中心信标后沿最后确认方向快速离开，避免低速起步卡在灯上。 */
#define HW_DRONE_CENTER_BURST_SPEED_MPS       0.95f
#define HW_DRONE_CENTER_BURST_MS              380U
#define HW_DRONE_LOST_HOLD_MS                 2400U
#define HW_DRONE_LOST_HOLD_START_SPEED_SCALE  0.75f
/* 阶段1 微调: hold 结束时保留 20% 速度, 避免最后一帧速度直接归零产生硬切台阶.
 * 真正解决 hold 退出硬停需要阶段2的 fade-out 逻辑, 此处只作为即时过渡. */
#define HW_DRONE_LOST_HOLD_END_SPEED_SCALE    0.20f
/* 阶段2 新增: hold 结束后追加一段平滑减速窗口.
 * 在这段时间内保持 DRONE_TRACK 模式并沿 hold 方向逐渐减速到 0,
 * 避免信标接力场景下"踩灭 A -> 等待 B 出现"期间的硬停台阶.
 * fade 期间如果新信标被识别, 自然接管追踪 (slew 当前值即"惯性速度", 无缝接续). */
#define HW_DRONE_LOST_HOLD_FADE_MS            600U
#define HW_DRONE_V_CENTER_X_PX                94.0f
#define HW_DRONE_V_CENTER_Y_PX                60.0f
#define HW_DRONE_FWD_SIGN                     (1.0f)
/* Car reference frame for the drone link:
 *   +Y = forward, +X = right.
 * A4 carries raw V/B pixels; VBX drives +X and VBY drives +Y. */
#define HW_DRONE_RIGHT_SIGN                   (1.0f)
#define HW_DRONE_YAW_COMP_ENABLE             1U
#define HW_DRONE_YAW_COMP_SIGN               (1.0f)
#define HW_DRONE_REMOTE_OVERRIDE_RAW          260

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
