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

#define HW_MANUAL_MAX_DUTY                    9000
#define HW_DRONE_TRACK_MAX_DUTY               9000
#define HW_MANUAL_MAX_LINEAR_SPEED_MPS        1.10f
#define HW_MANUAL_MAX_LATERAL_SPEED_MPS       1.50f
#define HW_MANUAL_MAX_YAW_RATE_RADPS          1.20f

#define HW_DRONE_DEADBAND_PX                  5.0f
#define HW_DRONE_PX_NORM                      150.0f
#define HW_DRONE_MIN_FWD_MPS                  0.375f
#define HW_DRONE_MIN_RIGHT_MPS                0.375f
/* 前后速度保持保守, 侧向给到轮速上限, 用来补偿麦轮横移效率偏低。 */
#define HW_DRONE_MAX_FWD_MPS                  0.80f
#define HW_DRONE_MAX_RIGHT_MPS                1.90f
#define HW_DRONE_TRACK_FILTER_ALPHA           0.30f
/* PD position loop gains for drone beacon tracking (no I term: mecanum has no
 * steady-state error to null, and integral windup risks overrunning the beacon).
 *   P : distance -> speed curve (see chassis_drone_distance_to_speed).
 *   D : -KD * d(pixel_err)/dt, damps overshoot and guarantees the V-beacon pixel
 *       distance decreases monotonically (brakes when the error starts growing). */
#define HW_DRONE_TRACK_KD_MPS_PER_PXPS        0.012f
#define HW_DRONE_NEAR_GAIN_BOOST_PX           40.0f
#define HW_DRONE_NEAR_GAIN_MULT               1.4f
/* 末端不减速地板: 进入 NEAR_GAIN_BOOST 区 (5~40px) 后速度下限不再降到 MIN_*,
 * 而是保持在 max_speed * NEAR_FLOOR_FRAC, 让小车到达信标正上方时仍有满动量
 * 一次性压过灯罩, 不再依赖后续 COAST/dash floor 补救 (那已经是事后补救)。
 * 这是"到达"问题不是"调节"问题: 麦轮无静差, I 项只会 windup 冲过震荡,
 * 真正瓶颈是末端减速吃掉了冲过所需的动量。0.85 对应约 0.80 m/s 末端最低速度。 */
#define HW_DRONE_NEAR_FLOOR_FRAC              0.85f
/* 信标冲刺阶段: 信标进入 DASH_ZONE(复用 NEAR_GAIN_BOOST_PX=40px) 内, 沿 PD 实时
 * 方向把速度提到 DASH_SPEED, 但前后轴仍受 MAX_FWD 限制, 避免正前方信标过冲。
 * 只放大不缩小, 不扭曲方向 (冲刺方向 = PD 单位误差向量)。
 * 冲刺仍走 slew 平滑加速; 信标消失后 COAST 继续沿锁定方向压过去。 */
#define HW_DRONE_DASH_SPEED_MPS               1.45f
#define HW_DRONE_DASH_ZONE_PX                 (HW_DRONE_NEAR_GAIN_BOOST_PX)
#define HW_DRONE_FWD_SIGN                     (1.0f)
/* Car reference frame for the drone link:
 *   +Y = forward, +X = right.
 * A4 carries raw V/B pixels; VBX drives +X and VBY drives +Y. */
#define HW_DRONE_RIGHT_SIGN                   (1.0f)
#define HW_DRONE_REMOTE_OVERRIDE_RAW          260
/* 信标消失后短时沿锁定方向滑行, 保证底盘越过灯罩, 避免边缘来回抖动。 */
#define HW_BEACON_COAST_MS                    350U
#define HW_BEACON_COAST_MIN_VEL_MPS           0.06f
#define HW_BEACON_COAST_DASH_FLOOR_MPS        0.80f
/* 主动击灭承诺区: 当信标进入此像素距离内, 且 V-B 单位向量方向翻转
 * (信标刚从小车下方穿到另一侧), 锁定穿越瞬间的 body 速度方向继续滑行,
 * 不让 PD 立即反向追。打破"冲过头->反向追->又冲过头"的来回震荡。
 * 64px 约对应 1m 高度下信标在小车下方附近的几何尺度。 */
#define HW_BEACON_STRIKE_CROSS_ZONE_PX        64.0f
/* 穿越承诺锁定最长持续时间(ms): 超时强制退出, 防止永久锁死。
 * 正常情况下信标会在远小于此时间内被踩灭(move_valid 掉->进 COAST)。 */
#define HW_BEACON_CROSS_COMMIT_TIMEOUT_MS     800U
/* 穿越锁定期间若 V-B 距离重新变大到此阈值以上, 认为确实没踩中, 退出锁定让 PD 重新接管。 */
#define HW_BEACON_CROSS_COMMIT_ESCAPE_PX      (HW_BEACON_STRIKE_CROSS_ZONE_PX * 1.5f)
/* 距离冲刺触发 (吸收"早期方向"想法的正确落地): 除了"方向翻转"触发 (偏晚),
 * 当距离首次进入此阈值内且在持续缩小 (逼近非抖动) 时, 也锁定当前稳定方向全力冲刺。
 * 用进入近距区那一刻的稳定方向做冲刺方向, 不依赖近距像素方向 (5px抖动=几十度方向噪声)。
 * 取 25px: 比方向翻转触发的几何尺度更早建立动量, 又足够近让方向已收敛稳定。 */
#define HW_BEACON_DASH_TRIGGER_PX             25.0f

/* 卡住检测 + 前后轴自救 (麦轮侧向碾灯罩必卡, 软件事后自救):
 * 物理依据: mecanum 纯 vx(前后轴)让4轮同向主动驱动(最有力); 纯 vy(侧向)靠滚子被动(无力易卡)。
 * 检测: 命令速度>STUCK_CMD_MIN 但实测平移<STUCK_MEAS_MAX 持续 STUCK_CONFIRM_MS -> 判定卡住。
 * 自救: 沿车体前后轴强驱动(纯vx), omega 仍跑 yaw_hold(锁头不破), 持续 ESCAPE_MS 后回PD重试。
 * 防死循环: 同信标卡住超 STUCK_MAX_RETRY 次 -> 进 SAFE_STOP 停车保护。 */
#define HW_BEACON_STUCK_CMD_MIN_MPS           0.30f
#define HW_BEACON_STUCK_MEAS_MAX_MPS          0.08f
#define HW_BEACON_STUCK_CONFIRM_MS            250U
#define HW_BEACON_ESCAPE_SPEED_MPS            0.50f
#define HW_BEACON_ESCAPE_MS                   400U
#define HW_BEACON_STUCK_MAX_RETRY             2U
/* 信标重新远距出现(距离增大到此以上)时清零重试计数, 认为是新一次接近而非同一卡住循环。 */
#define HW_BEACON_STUCK_RETRY_RESET_PX        (HW_BEACON_STRIKE_CROSS_ZONE_PX * 2.0f)

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
