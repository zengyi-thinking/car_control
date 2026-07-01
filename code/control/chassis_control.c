/*******************************************************************************
 * 麦轮底盘控制
 *
 * 控制模式:
 * 1. SAFE_STOP: 电机停转
 * 2. MANUAL: LoRa 遥控摇杆直控 + IMU 锁头
 * 3. DRONE_TRACK: 0xA4 无人机 V/B 像素坐标 + 车体系速度控制 + IMU 锁头
 ******************************************************************************/

#include "control/chassis_control.h"

#include "control/mecanum.h"
#include "control/odometry.h"
#include "control/pid.h"
#include "hardware/hw_config.h"

#include <math.h>
#include <string.h>

#define CHASSIS_PI                          3.14159265358979323846f
#define CHASSIS_DEG_TO_RAD                  (CHASSIS_PI / 180.0f)
#define CHASSIS_RAD_TO_DEG                  (180.0f / CHASSIS_PI)

#define CHASSIS_POS_KP                      1.8f
#define CHASSIS_VEL_DAMP_K                  0.55f
#define CHASSIS_POS_DEADBAND_M              0.03f
#define CHASSIS_ARRIVE_VEL_MPS              0.08f

/* yaw 保持控制器参数. 这组参数针对「电机疲劳造成四轮不对称 → 持续旋转扰动」设计:
 * KP 提供即时回正力矩; KI + I_LIMIT 提供足够积分权限去"吃掉"恒定不对称扰动
 * (这是无磁力计麦轮航向锁定的关键 — 持续扰动只能靠积分抵消, 见 IEEE 11301709);
 * KD 直接阻尼陀螺角速率, 抑制超调与振荡. */
#define CHASSIS_HEADING_KP                  3.5f
#define CHASSIS_HEADING_KI                  0.18f
#define CHASSIS_HEADING_KD                  0.38f
#define CHASSIS_HEADING_I_LIMIT             0.60f
/* 仅作为「静止防抖」死区: 只有当车体几乎不动(平移速度<阈值)时才生效,
 * 避免在移动中把积分清零导致「攒→清→攒」周期性摆动. 移动中小偏差必须持续修正,
 * 否则长距离直线行驶会因小幅偏差累积而缓慢偏航. */
#define CHASSIS_HEADING_ERR_DEADBAND_DEG    0.15f
#define CHASSIS_HEADING_GYRO_DEADBAND_RADPS 0.010f
#define CHASSIS_HEADING_STATIC_VEL_MPS      0.05f   /* 低于此平移速度视为静止 */

#define CHASSIS_STARTUP_LOCK_TIME_S         1.0f
#define CHASSIS_STARTUP_YAW_STABLE_DEG      3.0f

#define CHASSIS_REMOTE_AXIS_MAX             1600.0f
#define CHASSIS_REMOTE_DEADBAND             100.0f
#define CHASSIS_MANUAL_AXIS_SNAP_MIN        0.18f
#define CHASSIS_MANUAL_AXIS_SNAP_RATIO      0.35f

#define CHASSIS_WHEEL_PID_OUT_MAX           3000.0f
#define CHASSIS_WHEEL_PID_I_MAX             800.0f

static const float s_wheel_pid_kp[4] =
{
    700.0f,   /* LF: 四轮一致起步，消除旧的"右侧补强"偏置 */
    700.0f,   /* RF */
    700.0f,   /* LB */
    700.0f,   /* RB */
};

static const float s_wheel_pid_ki[4] =
{
    170.0f,   /* LF: 四轮一致 */
    170.0f,   /* RF */
    170.0f,   /* LB */
    170.0f,   /* RB */
};

static const float s_wheel_pid_kd[4] =
{
    8.0f,    /* LF: light damping to suppress Kp-induced buzz */
    8.0f,    /* RF */
    8.0f,    /* LB */
    8.0f,    /* RB */
};

#define CHASSIS_TUNE_AXIS_ON                0.55f
#define CHASSIS_TUNE_AXIS_OFF               0.25f

#define CHASSIS_POSITION_EDIT_AXIS_ON       0.45f
#define CHASSIS_POSITION_EDIT_AXIS_OFF      0.20f
#define CHASSIS_POSITION_EDIT_DEADBAND      0.08f
#define CHASSIS_POSITION_ARRIVE_VEL_MPS     0.08f

volatile uint8 g_chassis_wheel_pid_tune_enabled = 0U;
volatile uint8 g_chassis_wheel_pid_tune_wheel = 0U;
volatile uint8 g_chassis_wheel_pid_tune_param = 0U;
volatile float g_chassis_wheel_pid_tune_target_mps = HW_WHEEL_TUNE_TARGET_MPS;

typedef enum
{
    CHASSIS_DRONE_TRACK_APPROACH = 0,
    CHASSIS_DRONE_TRACK_COAST = 1,
    CHASSIS_DRONE_TRACK_STUCK_ESCAPE = 2,   /* 卡住自救: 沿前后轴强驱动挣脱 */
} chassis_drone_track_state_t;

static const float s_wheel_ff_ks_duty[4] =
{
    300.0f,   /* LF: 四轮一致起步，消除旧的"右侧补强"偏置 */
    300.0f,   /* RF */
    300.0f,   /* LB */
    300.0f,   /* RB */
};

typedef struct
{
    proto_remote_data_t remote;
    proto_drone_data_t drone;
    odometry_state_t odom;
    chassis_mode_t last_mode;
    uint8 yaw_lock_valid;
    float yaw_lock_deg;
    float heading_integral;
    float last_yaw_deg;
    float cmd_vx_field_state;
    float cmd_vy_field_state;
    float cmd_vx_body_state;
    float cmd_vy_body_state;
    float gyro_z_bias_radps;
    float gyro_z_bias_inflight_radps;  /* 运动中在线估计的陀螺z零偏(温漂补偿) */
    float motion_yaw_trim_dps;
    float startup_reference_yaw_deg;
    float startup_reference_yaw_min_deg;
    float startup_reference_yaw_max_deg;
    float startup_gyro_sum_radps;
    float startup_lock_elapsed_s;
    uint16 startup_lock_samples;
    uint8 startup_lock_done;
    uint8 startup_reference_valid;
    uint8 yaw_continuous_valid;
    uint8 position_move_state;
    uint8 position_press_armed;
    uint8 heading_reset_armed;
    uint8 tune_wheel_axis_armed;
    uint8 tune_param_axis_armed;
    uint8 tune_value_axis_armed;
    float position_edit_dx_m;
    float position_edit_dy_m;
    float position_target_forward_m;
    float position_target_right_m;
    float last_attitude_yaw_deg;
    float yaw_lock_continuous_deg;
    uint16 coast_remaining_ms;   /* beacon pass-over coast window (ms) */
    uint8 drone_track_state;
    uint8 drone_error_filter_valid;
    uint8 drone_last_error_valid;
    float drone_error_x_px;       /* filtered pixel error (P + filter state) */
    float drone_error_y_px;
    float drone_last_error_x_px;  /* previous-tick filtered error (D term) */
    float drone_last_error_y_px;
    /* D 项抗噪: 视觉帧约 30Hz 但控制环 200Hz, 若 D 项用控制周期 (5ms) 做微分,
     * 同一帧被复用 6-7 拍, 帧切换瞬间 d(err)/dt 会被放大 ~6.6 倍产生尖峰刹车。
     * 用 drone_frame_dt_s 记录真实帧间隔 (新帧到达时锁定), D 项改用它做 dt,
     * 消除采样率不匹配导致的微分尖峰。 */
    float drone_frame_dt_s;        /* 真实帧间隔, 仅在新帧到达拍更新 */
    uint16 drone_age_accum_ms;     /* 自上次新帧起的累积控制拍时间 */
    uint8 drone_frame_fresh;       /* 本拍是否有新视觉帧 (边沿信号) */
    /* 主动击灭承诺区: 当信标进入 CROSS_ZONE_PX 内且 V-B 单位向量方向翻转
     * (信标刚从小车下方穿到另一侧), 锁定穿越瞬间的 body 速度方向继续滑行,
     * 不让 PD 立即反向追, 打破"冲过头->反向追->又冲过头"的来回震荡。
     * 锁定期间冻结 last_vb 用于退出判定; 由 move_valid 掉(COAST)/距离重新变大/超时退出。 */
    float drone_last_vb_x_px;      /* 上一拍滤波后的 V-B 像素误差 (穿越检测用) */
    float drone_last_vb_y_px;
    uint8 drone_vb_valid;          /* 是否有有效的上一拍 vb (首次无 last_error 时为0) */
    uint8 drone_cross_committed;   /* 当前是否处于穿越承诺锁定状态 */
    uint16 drone_cross_commit_ms;  /* 锁定已持续时间(ms), 超时退出防永久锁死 */
    /* 卡住检测 + 前后轴自救 (麦轮侧向碾灯罩必卡, 软件事后自救, 不依赖无人机)。
     * stuck_detect_ms: 命令大但实测小(堵转)的持续累积时间, 达 STUCK_CONFIRM_MS 触发自救。
     * stuck_escape_ms: 自救剩余时间, 倒计时到0回PD; escape_dir 取卡住前 cmd_vx 符号。 */
    uint16 drone_stuck_detect_ms;
    uint16 drone_stuck_escape_ms;
    float drone_stuck_escape_dir;  /* 自救前后轴方向: +1前进 / -1后退 */
    uint8 drone_stuck_retry_count; /* 同信标卡住重试计数, 超 STUCK_MAX_RETRY 放弃进SAFE_STOP */
} chassis_runtime_t;

static chassis_runtime_t g_runtime;
static chassis_status_t g_status;
pid_t g_wheel_pid[4];

static float chassis_clamp(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

static float chassis_wrap_deg(float deg);

/*
 * 坐标约定:
 * - IMU/编辑参考系: +X 左, +Y 前
 * - 底盘车体系:      forward 为前进, right 为右移
 * - 里程计/场地系:   x/y 为世界平面坐标, 由当前 yaw 决定和车体的夹角
 *
 * 关系分两步:
 * 1) IMU编辑量 -> 车体位移: forward = +Y, right = -X
 * 2) 车体位移   -> 场地位移: 按当前 yaw 旋转到世界坐标
 */
static void chassis_imu_to_body_move(float dx_left_m, float dy_front_m, float *forward_m, float *right_m)
{
    if (forward_m)
    {
        *forward_m = dy_front_m;
    }
    if (right_m)
    {
        *right_m = -dx_left_m;
    }
}

static void chassis_body_to_field_move(float forward_m, float right_m, float yaw_deg, float *field_x_m, float *field_y_m)
{
    const float yaw_rad = yaw_deg * CHASSIS_DEG_TO_RAD;
    const float cos_yaw = cosf(yaw_rad);
    const float sin_yaw = sinf(yaw_rad);

    if (field_x_m)
    {
        *field_x_m = (cos_yaw * forward_m) - (sin_yaw * right_m);
    }
    if (field_y_m)
    {
        *field_y_m = (sin_yaw * forward_m) + (cos_yaw * right_m);
    }
}

static void chassis_field_to_imu_error(float field_x_err_m,
                                       float field_y_err_m,
                                       float yaw_deg,
                                       float *left_err_m,
                                       float *front_err_m)
{
    const float yaw_rad = yaw_deg * CHASSIS_DEG_TO_RAD;
    const float cos_yaw = cosf(yaw_rad);
    const float sin_yaw = sinf(yaw_rad);
    const float forward_err_m = (cos_yaw * field_x_err_m) + (sin_yaw * field_y_err_m);
    const float right_err_m = (-sin_yaw * field_x_err_m) + (cos_yaw * field_y_err_m);

    if (left_err_m)
    {
        *left_err_m = -right_err_m;
    }
    if (front_err_m)
    {
        *front_err_m = forward_err_m;
    }
}

static float chassis_tune_param_step(uint8 param_index)
{
    switch (param_index)
    {
        case CHASSIS_TUNE_PARAM_KP: return HW_WHEEL_TUNE_KP_STEP;
        case CHASSIS_TUNE_PARAM_KI: return HW_WHEEL_TUNE_KI_STEP;
        case CHASSIS_TUNE_PARAM_KD: return HW_WHEEL_TUNE_KD_STEP;
        default: return 0.0f;
    }
}

static void chassis_tune_reset_latch(void)
{
    g_runtime.tune_wheel_axis_armed = 0U;
    g_runtime.tune_param_axis_armed = 0U;
    g_runtime.tune_value_axis_armed = 0U;
}

static void chassis_tune_wrap_selection(void)
{
    g_chassis_wheel_pid_tune_wheel %= 4U;
    g_chassis_wheel_pid_tune_param %= CHASSIS_TUNE_PARAM_COUNT;
}

static void chassis_reset_command_states(void)
{
    g_runtime.cmd_vx_field_state = 0.0f;
    g_runtime.cmd_vy_field_state = 0.0f;
    g_runtime.cmd_vx_body_state = 0.0f;
    g_runtime.cmd_vy_body_state = 0.0f;
    g_runtime.drone_track_state = CHASSIS_DRONE_TRACK_APPROACH;
    g_runtime.drone_error_filter_valid = 0U;
    g_runtime.drone_last_error_valid = 0U;
    g_runtime.drone_error_x_px = 0.0f;
    g_runtime.drone_error_y_px = 0.0f;
    g_runtime.drone_last_error_x_px = 0.0f;
    g_runtime.drone_last_error_y_px = 0.0f;
    g_runtime.drone_frame_dt_s = 0.033f;       /* 默认假设 30Hz 视觉帧 */
    g_runtime.drone_age_accum_ms = 0U;
    g_runtime.drone_frame_fresh = 0U;
    g_runtime.drone_last_vb_x_px = 0.0f;
    g_runtime.drone_last_vb_y_px = 0.0f;
    g_runtime.drone_vb_valid = 0U;
    g_runtime.drone_cross_committed = 0U;
    g_runtime.drone_cross_commit_ms = 0U;
    g_runtime.drone_stuck_detect_ms = 0U;
    g_runtime.drone_stuck_escape_ms = 0U;
    g_runtime.drone_stuck_escape_dir = 1.0f;
    g_runtime.drone_stuck_retry_count = 0U;
}

static void chassis_seed_command_states_from_odometry(void)
{
    g_runtime.cmd_vx_field_state = g_runtime.odom.vx_field_mps;
    g_runtime.cmd_vy_field_state = g_runtime.odom.vy_field_mps;
    g_runtime.cmd_vx_body_state = g_runtime.odom.vx_body_mps;
    g_runtime.cmd_vy_body_state = g_runtime.odom.vy_body_mps;
}

static void chassis_tune_adjust_selected_pid(float delta)
{
    pid_t *pid;

    g_chassis_wheel_pid_tune_wheel %= 4U;
    g_chassis_wheel_pid_tune_param %= CHASSIS_TUNE_PARAM_COUNT;

    pid = &g_wheel_pid[g_chassis_wheel_pid_tune_wheel];
    switch (g_chassis_wheel_pid_tune_param)
    {
        case CHASSIS_TUNE_PARAM_KP:
            pid->kp = chassis_clamp(pid->kp + delta, 0.0f, 5000.0f);
            break;
        case CHASSIS_TUNE_PARAM_KI:
            pid->ki = chassis_clamp(pid->ki + delta, 0.0f, 5000.0f);
            break;
        case CHASSIS_TUNE_PARAM_KD:
            pid->kd = chassis_clamp(pid->kd + delta, 0.0f, 5000.0f);
            break;
        default:
            break;
    }
}

static float chassis_feedforward_duty(uint8 wheel, float target_speed_mps)
{
    float abs_speed;
    float duty;
    float ff_scale;

    if (fabsf(target_speed_mps) < 0.001f)
    {
        return 0.0f;
    }

    abs_speed = fabsf(target_speed_mps);
    /* Fade the static-friction term in below HW_MOTOR_FF_FADE_MPS so the
     * feedforward is continuous through zero instead of jumping by the full
     * static-friction duty. Removes the "thunk" at deadband entry/exit. */
    ff_scale = (abs_speed < HW_MOTOR_FF_FADE_MPS) ? (abs_speed / HW_MOTOR_FF_FADE_MPS) : 1.0f;
    duty = (s_wheel_ff_ks_duty[wheel] * ff_scale) + (HW_MOTOR_FF_KV_DUTY_PER_MPS * abs_speed);

    return (target_speed_mps > 0.0f) ? duty : -duty;
}

static void chassis_startup_lock_reset(void)
{
    g_runtime.gyro_z_bias_radps = 0.0f;
    g_runtime.gyro_z_bias_inflight_radps = 0.0f;
    g_runtime.startup_reference_yaw_deg = 0.0f;
    g_runtime.startup_reference_yaw_min_deg = 0.0f;
    g_runtime.startup_reference_yaw_max_deg = 0.0f;
    g_runtime.startup_gyro_sum_radps = 0.0f;
    g_runtime.startup_lock_elapsed_s = 0.0f;
    g_runtime.startup_lock_samples = 0U;
    g_runtime.startup_lock_done = 0U;
    g_runtime.startup_reference_valid = 0U;
    g_runtime.yaw_continuous_valid = 0U;
    g_status.startup_lock_done = 0U;
    g_status.startup_lock_elapsed_s = 0.0f;
    g_status.gyro_z_bias_radps = 0.0f;
    g_runtime.yaw_lock_continuous_deg = 0.0f;
    g_runtime.yaw_lock_deg = 0.0f;
    g_runtime.heading_reset_armed = 0U;
    g_runtime.motion_yaw_trim_dps = 0.0f;
    g_status.target_yaw_deg = chassis_wrap_deg(g_runtime.last_yaw_deg);
    g_status.yaw_error_deg = 0.0f;
    chassis_reset_command_states();
}

static void chassis_startup_lock_step(float yaw_deg, float gyro_z_radps, float dt_s, uint8 attitude_valid)
{
    float yaw_delta_deg;

    if (g_runtime.startup_lock_done || (dt_s <= 0.0f) || !attitude_valid)
    {
        g_status.startup_lock_done = g_runtime.startup_lock_done;
        g_status.startup_lock_elapsed_s = g_runtime.startup_lock_elapsed_s;
        g_status.gyro_z_bias_radps = g_runtime.gyro_z_bias_radps;
        g_status.gyro_z_motion_trim_radps = g_runtime.motion_yaw_trim_dps * CHASSIS_DEG_TO_RAD;
        return;
    }

    if (!g_runtime.startup_reference_valid)
    {
        g_runtime.startup_reference_valid = 1U;
        g_runtime.startup_reference_yaw_deg = yaw_deg;
        g_runtime.startup_reference_yaw_min_deg = yaw_deg;
        g_runtime.startup_reference_yaw_max_deg = yaw_deg;
        g_runtime.startup_lock_elapsed_s = 0.0f;
        g_runtime.startup_gyro_sum_radps = 0.0f;
        g_runtime.startup_lock_samples = 0U;
    }

    yaw_delta_deg = fabsf(chassis_wrap_deg(yaw_deg - g_runtime.startup_reference_yaw_deg));
    if (yaw_delta_deg > CHASSIS_STARTUP_YAW_STABLE_DEG)
    {
        g_runtime.startup_reference_yaw_deg = yaw_deg;
        g_runtime.startup_reference_yaw_min_deg = yaw_deg;
        g_runtime.startup_reference_yaw_max_deg = yaw_deg;
        g_runtime.startup_lock_elapsed_s = 0.0f;
        g_runtime.startup_gyro_sum_radps = 0.0f;
        g_runtime.startup_lock_samples = 0U;
        g_status.startup_lock_done = 0U;
        g_status.startup_lock_elapsed_s = 0.0f;
        g_status.gyro_z_bias_radps = 0.0f;
        return;
    }

    if (yaw_deg < g_runtime.startup_reference_yaw_min_deg)
    {
        g_runtime.startup_reference_yaw_min_deg = yaw_deg;
    }
    if (yaw_deg > g_runtime.startup_reference_yaw_max_deg)
    {
        g_runtime.startup_reference_yaw_max_deg = yaw_deg;
    }

    g_runtime.startup_lock_elapsed_s += dt_s;
    g_runtime.startup_gyro_sum_radps += gyro_z_radps;
    g_runtime.startup_lock_samples++;

    if (g_runtime.startup_lock_elapsed_s >= CHASSIS_STARTUP_LOCK_TIME_S)
    {
        /* 锚定到「窗口结束时的当前 yaw」(已收敛值), 而非第一帧的 startup_reference_yaw_deg.
         * 上电瞬间 AHRS 互补滤波尚未收敛, 第一帧 yaw 是漂移的暂态值; 若锚定到它,
         * 1 秒后 IMU 收敛到真实航向, 目标却还停在旧值 -> 上电即出现 ~8° (≈STABLE 阈值) 偏差.
         * startup_reference_yaw_deg 仅继续用于上面的稳定性判定 (相对首帧漂移是否超阈值). */
        g_runtime.yaw_lock_continuous_deg = yaw_deg;
        g_runtime.yaw_lock_deg = chassis_wrap_deg(g_runtime.yaw_lock_continuous_deg);
        g_runtime.gyro_z_bias_radps = g_runtime.startup_gyro_sum_radps / ((g_runtime.startup_lock_samples > 0U) ? (float)g_runtime.startup_lock_samples : 1.0f);
        g_runtime.yaw_lock_valid = 1U;
        g_runtime.heading_integral = 0.0f;
        g_runtime.startup_lock_done = 1U;
    }

    g_status.startup_lock_done = g_runtime.startup_lock_done;
    g_status.startup_lock_elapsed_s = g_runtime.startup_lock_elapsed_s;
    g_status.gyro_z_bias_radps = g_runtime.gyro_z_bias_radps;
    g_status.gyro_z_motion_trim_radps = g_runtime.motion_yaw_trim_dps * CHASSIS_DEG_TO_RAD;
}

static float chassis_yaw_error_deg(float yaw_deg)
{
    if (!g_runtime.yaw_lock_valid)
    {
        return 0.0f;
    }

    /*
     * Yaw is continuous internally to avoid ±180° jumps, but the hold target
     * lives on a circle. Always use the shortest angular distance here so
     * 0° and 360° stay equivalent after multiple full turns.
     */
    return chassis_wrap_deg(yaw_deg - g_runtime.yaw_lock_continuous_deg);
}

/* 运动中在线陀螺z零偏估计. 无磁力计麦轮的关键: 上电静止标定的零偏会因温漂变化,
 * 运动中无法重新静止标定, 故用 yaw 积分方向作为可观测量缓慢学习.
 * 原理: 若存在残余零偏 b, yaw 会以 b 速率持续单向漂移 → yaw_hold 积分朝修正方向
 * 累积. 当积分已接近饱和(说明扰动持续存在), 把一小部分积分"迁移"成零偏估计,
 * 这样零偏慢慢吸收恒定扰动, 积分留余量应对瞬态. 学习极慢(增益0.001)避免误学真实转动. */
#define CHASSIS_BIAS_LEARN_GAIN              0.0015f
#define CHASSIS_BIAS_LEARN_MIN_INTEGRAL_FRAC 0.5f   /* 积分>50%饱和时才学零偏 */
#define CHASSIS_BIAS_LEARN_LIMIT_RADPS        0.020f /* 在线零偏估计限幅(~1.1dps) */

/* 统一的 yaw 保持控制器. strict=1 时完全不设死区(用于位置控制, 任何偏差都要修);
 * strict=0 时只在车体静止时启用防抖死区, 移动中持续修正(避免长直线摆动).
 * 注意: 不再因死区清空 heading_integral, 仅在静止+小偏差时停止累加并输出0,
 * 这样移动中累积的修正量在短暂静止后能立即复用, 保持更稳定. */
static float chassis_yaw_hold_core(float yaw_deg, float gyro_z_radps, float transl_speed_mps,
                                   float dt_s, uint8 strict)
{
    float yaw_err_deg;
    float yaw_err_rad;
    float integral_candidate;
    float unsat_output;
    float omega_cmd;
    float gyro_z_corrected_radps;
    float total_bias_radps;

    if (!g_runtime.yaw_lock_valid || (dt_s <= 0.0f))
    {
        g_status.yaw_error_deg = 0.0f;
        return 0.0f;
    }

    /* 总零偏 = 上电静止标定零偏 + 运动中在线零偏估计(温漂补偿). */
    total_bias_radps = g_runtime.gyro_z_bias_radps + g_runtime.gyro_z_bias_inflight_radps;
    gyro_z_corrected_radps = gyro_z_radps - total_bias_radps;
    yaw_err_deg = chassis_yaw_error_deg(yaw_deg);

    /* 静止防抖死区: 仅 strict=0 且车体静止时, 对极小偏差+低转速输出0.
     * 不清空 integral — 移动中攒的修正量保留, 下次有偏差可继续修正. */
    if ((strict == 0U) &&
        (transl_speed_mps < CHASSIS_HEADING_STATIC_VEL_MPS) &&
        (fabsf(yaw_err_deg) < CHASSIS_HEADING_ERR_DEADBAND_DEG) &&
        (fabsf(gyro_z_corrected_radps) < CHASSIS_HEADING_GYRO_DEADBAND_RADPS))
    {
        g_status.yaw_error_deg = yaw_err_deg;
        return 0.0f;
    }

    yaw_err_rad = yaw_err_deg * CHASSIS_DEG_TO_RAD;

    integral_candidate = g_runtime.heading_integral + (yaw_err_rad * CHASSIS_HEADING_KI * dt_s);
    integral_candidate = chassis_clamp(integral_candidate, -CHASSIS_HEADING_I_LIMIT, CHASSIS_HEADING_I_LIMIT);

    /* 条件积分(防积分饱和): 仅当未饱和或饱和方向与误差符号相反时才提交积分. */
    unsat_output = CHASSIS_HEADING_KP * yaw_err_rad + integral_candidate - CHASSIS_HEADING_KD * gyro_z_corrected_radps;
    if ((fabsf(unsat_output) <= HW_MAX_YAW_RATE_RADPS) ||
        ((unsat_output > HW_MAX_YAW_RATE_RADPS) && (yaw_err_rad < 0.0f)) ||
        ((unsat_output < -HW_MAX_YAW_RATE_RADPS) && (yaw_err_rad > 0.0f)))
    {
        g_runtime.heading_integral = integral_candidate;
    }

    /* 在线零偏学习: 当积分已接近饱和(>I_LIMIT 的阈值比例), 说明存在持续单向扰动,
     * 极缓慢地把残余零偏从积分迁移到 bias_inflight. 仅移动中学习(transl_speed>静态阈值). */
    if ((strict == 0U) &&
        (transl_speed_mps >= CHASSIS_HEADING_STATIC_VEL_MPS) &&
        (fabsf(g_runtime.heading_integral) > (CHASSIS_HEADING_I_LIMIT * CHASSIS_BIAS_LEARN_MIN_INTEGRAL_FRAC)))
    {
        /* 积分符号指示了偏移方向: 积分为负=车在CCW漂(需CW修正), 意味着 gyro 残余为正. */
        const float bias_delta = -g_runtime.heading_integral * CHASSIS_BIAS_LEARN_GAIN * dt_s;
        g_runtime.gyro_z_bias_inflight_radps =
            chassis_clamp(g_runtime.gyro_z_bias_inflight_radps + bias_delta,
                          -CHASSIS_BIAS_LEARN_LIMIT_RADPS, CHASSIS_BIAS_LEARN_LIMIT_RADPS);
        /* 同步释放一部分积分(已迁移到零偏), 防止积分长期顶在饱和值. */
        g_runtime.heading_integral += bias_delta;
    }

    omega_cmd = CHASSIS_HEADING_KP * yaw_err_rad + g_runtime.heading_integral - CHASSIS_HEADING_KD * gyro_z_corrected_radps;
    omega_cmd = chassis_clamp(omega_cmd, -HW_MAX_YAW_RATE_RADPS, HW_MAX_YAW_RATE_RADPS);
    g_status.yaw_error_deg = yaw_err_deg;
    return omega_cmd;
}

static float chassis_yaw_hold(float yaw_deg, float gyro_z_radps, float transl_speed_mps, float dt_s)
{
    return chassis_yaw_hold_core(yaw_deg, gyro_z_radps, transl_speed_mps, dt_s, 0U);
}

static float chassis_yaw_hold_strict(float yaw_deg, float gyro_z_radps, float transl_speed_mps, float dt_s)
{
    return chassis_yaw_hold_core(yaw_deg, gyro_z_radps, transl_speed_mps, dt_s, 1U);
}

static float chassis_limit_translation_scale(const float translation_wheel[4], const float rotation_wheel[4], float max_speed)
{
    float low = 0.0f;
    float high = 1.0f;
    float best = 0.0f;

    for (uint8 iter = 0U; iter < 18U; ++iter)
    {
        const float mid = 0.5f * (low + high);
        float max_abs = 0.0f;

        for (uint8 i = 0U; i < 4U; ++i)
        {
            const float wheel = (translation_wheel[i] * mid) + rotation_wheel[i];
            const float abs_wheel = fabsf(wheel);

            if (abs_wheel > max_abs)
            {
                max_abs = abs_wheel;
            }
        }

        if (max_abs > max_speed)
        {
            high = mid;
        }
        else
        {
            best = mid;
            low = mid;
        }
    }

    return best;
}

static void chassis_tune_apply_remote_input(float left_x, float left_y, float right_y)
{
    const uint8 select_wheel = (uint8)(fabsf(left_y) > CHASSIS_TUNE_AXIS_ON);
    const uint8 select_param = (uint8)(fabsf(left_x) > CHASSIS_TUNE_AXIS_ON);
    const uint8 adjust_value = (uint8)(fabsf(right_y) > CHASSIS_TUNE_AXIS_ON);

    if (fabsf(left_y) < CHASSIS_TUNE_AXIS_OFF)
    {
        g_runtime.tune_wheel_axis_armed = 0U;
    }
    if (fabsf(left_x) < CHASSIS_TUNE_AXIS_OFF)
    {
        g_runtime.tune_param_axis_armed = 0U;
    }
    if (fabsf(right_y) < CHASSIS_TUNE_AXIS_OFF)
    {
        g_runtime.tune_value_axis_armed = 0U;
    }

    if (select_wheel && !g_runtime.tune_wheel_axis_armed)
    {
        if (left_y > 0.0f)
        {
            g_chassis_wheel_pid_tune_wheel = (uint8)((g_chassis_wheel_pid_tune_wheel + 1U) % 4U);
        }
        else
        {
            g_chassis_wheel_pid_tune_wheel = (uint8)((g_chassis_wheel_pid_tune_wheel + 3U) % 4U);
        }
        g_runtime.tune_wheel_axis_armed = 1U;
    }

    if (select_param && !g_runtime.tune_param_axis_armed)
    {
        if (left_x > 0.0f)
        {
            g_chassis_wheel_pid_tune_param = (uint8)((g_chassis_wheel_pid_tune_param + 1U) % CHASSIS_TUNE_PARAM_COUNT);
        }
        else
        {
            g_chassis_wheel_pid_tune_param = (uint8)((g_chassis_wheel_pid_tune_param + CHASSIS_TUNE_PARAM_COUNT - 1U) % CHASSIS_TUNE_PARAM_COUNT);
        }
        g_runtime.tune_param_axis_armed = 1U;
    }

    if (adjust_value && !g_runtime.tune_value_axis_armed)
    {
        const float step = chassis_tune_param_step(g_chassis_wheel_pid_tune_param);
        const float delta = (right_y > 0.0f) ? step : -step;
        chassis_tune_adjust_selected_pid(delta);
        g_runtime.tune_value_axis_armed = 1U;
    }

    chassis_tune_wrap_selection();
}

static float chassis_wrap_deg(float deg)
{
    while (deg > 180.0f)
    {
        deg -= 360.0f;
    }
    while (deg < -180.0f)
    {
        deg += 360.0f;
    }
    return deg;
}

static float chassis_move_toward(float current, float target, float max_step)
{
    if (target > (current + max_step))
    {
        return current + max_step;
    }
    if (target < (current - max_step))
    {
        return current - max_step;
    }
    return target;
}

static void chassis_limit_vector(float *x, float *y, float max_norm)
{
    float norm;
    float scale;

    if ((x == 0) || (y == 0) || (max_norm <= 0.0f))
    {
        return;
    }

    norm = sqrtf((*x * *x) + (*y * *y));
    if (norm <= max_norm)
    {
        return;
    }

    scale = max_norm / norm;
    *x *= scale;
    *y *= scale;
}

static float chassis_normalize_remote_axis(int16 value)
{
    float axis = (float)value;

    if (fabsf(axis) <= CHASSIS_REMOTE_DEADBAND)
    {
        return 0.0f;
    }

    axis = chassis_clamp(axis / CHASSIS_REMOTE_AXIS_MAX, -1.0f, 1.0f);
    return axis;
}

static uint8 chassis_remote_button_pressed(const proto_remote_data_t *remote, uint8 key_index, uint8 *armed)
{
    uint8 pressed = 0U;

    if ((remote == 0) || (key_index >= 4U) || (armed == 0))
    {
        return 0U;
    }

    pressed = (remote->key[key_index] != 0U) ? 1U : 0U;
    if ((pressed != 0U) && (*armed == 0U))
    {
        *armed = 1U;
        return 1U;
    }

    if (pressed == 0U)
    {
        *armed = 0U;
    }

    return 0U;
}

static float chassis_drone_error_distance(float err_x_px, float err_y_px)
{
    return sqrtf((err_x_px * err_x_px) + (err_y_px * err_y_px));
}

static float chassis_drone_distance_to_speed(float dist_px)
{
    const float min_speed_mps = 0.5f * (HW_DRONE_MIN_FWD_MPS + HW_DRONE_MIN_RIGHT_MPS);
    /* Ceiling 用前后/侧向较大值, 让各向异性由末尾椭圆 clamp 控制;
     * 若取平均, 侧向 (MAX_RIGHT) 会被软件拉低, 与麦轮侧向需要更强命令相悖。 */
    const float max_speed_mps = (HW_DRONE_MAX_FWD_MPS > HW_DRONE_MAX_RIGHT_MPS)
                              ? HW_DRONE_MAX_FWD_MPS : HW_DRONE_MAX_RIGHT_MPS;
    float span_px;
    float progress;
    float near_max_speed_mps;

    if ((dist_px <= HW_DRONE_DEADBAND_PX) || (max_speed_mps <= 0.0f))
    {
        return 0.0f;
    }

    /* Near-zone gain boost: between DEADBAND and NEAR_GAIN_BOOST_PX, scale the
     * top of the curve by NEAR_GAIN_MULT so the car keeps closing in fast at
     * close range instead of stalling. The boost only raises the *target* of
     * the near segment; the far segment is clamped to the global max.
     * 末端地板修正 (一次性踩灭): 原曲线下限用 min_speed (0.375), 导致到达正上方时
     * 只剩 0.375 m/s 动量, 不足以让底盘整个压过灯罩, 依赖后续 COAST/dash floor 补救。
     * 改为下限 = max_speed * NEAR_FLOOR_FRAC (~0.80), 让到达时仍有满动量冲过。
     * 麦轮无静差, 不需要 I 项追精度; 瓶颈是末端动量, 不是稳态误差。 */
    near_max_speed_mps = chassis_clamp(max_speed_mps * HW_DRONE_NEAR_GAIN_MULT,
                                       min_speed_mps, max_speed_mps);
    {
        const float near_floor_mps = chassis_clamp(max_speed_mps * HW_DRONE_NEAR_FLOOR_FRAC,
                                                   min_speed_mps, near_max_speed_mps);
        if (dist_px <= HW_DRONE_NEAR_GAIN_BOOST_PX)
        {
            span_px = HW_DRONE_NEAR_GAIN_BOOST_PX - HW_DRONE_DEADBAND_PX;
            if (span_px < 1.0f)
            {
                span_px = 1.0f;
            }
            progress = (dist_px - HW_DRONE_DEADBAND_PX) / span_px;
            progress = chassis_clamp(progress, 0.0f, 1.0f);
            return near_floor_mps + ((near_max_speed_mps - near_floor_mps) * progress);
        }
    }

    span_px = HW_DRONE_PX_NORM - HW_DRONE_NEAR_GAIN_BOOST_PX;
    if (span_px < 1.0f)
    {
        span_px = 1.0f;
    }
    progress = (dist_px - HW_DRONE_NEAR_GAIN_BOOST_PX) / span_px;
    progress = chassis_clamp(progress, 0.0f, 1.0f);

    return near_max_speed_mps + ((max_speed_mps - near_max_speed_mps) * progress);
}

static void chassis_drone_filter_error(float raw_x_px, float raw_y_px, float *out_x_px, float *out_y_px)
{
    const float alpha = chassis_clamp(HW_DRONE_TRACK_FILTER_ALPHA, 0.0f, 1.0f);

    if (g_runtime.drone_error_filter_valid == 0U)
    {
        g_runtime.drone_error_x_px = raw_x_px;
        g_runtime.drone_error_y_px = raw_y_px;
        g_runtime.drone_error_filter_valid = 1U;
    }
    else
    {
        g_runtime.drone_error_x_px += alpha * (raw_x_px - g_runtime.drone_error_x_px);
        g_runtime.drone_error_y_px += alpha * (raw_y_px - g_runtime.drone_error_y_px);
    }

    if (out_x_px != 0)
    {
        *out_x_px = g_runtime.drone_error_x_px;
    }
    if (out_y_px != 0)
    {
        *out_y_px = g_runtime.drone_error_y_px;
    }
}

static void chassis_drone_store_last_error(float err_x_px, float err_y_px)
{
    g_runtime.drone_last_error_x_px = err_x_px;
    g_runtime.drone_last_error_y_px = err_y_px;
    g_runtime.drone_last_error_valid = 1U;
}

/* 卡住检测: 用上一拍命令速度 (g_status.cmd_vx/vy_body_mps, 上一拍末尾写入) 与当前实测平移速度
 * 比较。命令大但实测小 = 堵转, 持续 STUCK_CONFIRM_MS 判定卡住, 返回1并启动自救。
 * 自救期间(escape_ms>0) 不再累积检测, 由调用者倒计时退出。
 * move_invalid 或实测恢复 -> 清检测计数, 避免瞬时打滑/静止误判。 */
static uint8 chassis_drone_update_stuck_detection(uint8 move_valid)
{
    if ((move_valid == 0U) || (g_runtime.drone_stuck_escape_ms > 0U))
    {
        g_runtime.drone_stuck_detect_ms = 0U;
        return 0U;
    }

    {
        const float cmd_norm = sqrtf((g_status.cmd_vx_body_mps * g_status.cmd_vx_body_mps)
                                   + (g_status.cmd_vy_body_mps * g_status.cmd_vy_body_mps));
        const float meas_norm = sqrtf((g_runtime.odom.vx_body_mps * g_runtime.odom.vx_body_mps)
                                    + (g_runtime.odom.vy_body_mps * g_runtime.odom.vy_body_mps));

        if ((cmd_norm > HW_BEACON_STUCK_CMD_MIN_MPS) && (meas_norm < HW_BEACON_STUCK_MEAS_MAX_MPS))
        {
            if (g_runtime.drone_stuck_detect_ms < 0xFFFFU)
            {
                g_runtime.drone_stuck_detect_ms = (uint16)(g_runtime.drone_stuck_detect_ms + HW_CONTROL_PERIOD_MS);
            }
            if (g_runtime.drone_stuck_detect_ms >= HW_BEACON_STUCK_CONFIRM_MS)
            {
                /* 触发卡住: 锁定自救方向(取上一拍 cmd_vx 符号, 几近静止默认前进)并启动自救窗口。 */
                const float vx_sign = (g_status.cmd_vx_body_mps >= 0.0f) ? 1.0f : -1.0f;
                g_runtime.drone_stuck_escape_dir = vx_sign;
                g_runtime.drone_stuck_escape_ms = HW_BEACON_ESCAPE_MS;
                g_runtime.drone_stuck_detect_ms = 0U;
                return 1U;
            }
        }
        else
        {
            g_runtime.drone_stuck_detect_ms = 0U;   /* 实测恢复或命令不足, 清零 */
        }
    }
    return 0U;
}

/* PD closed-loop beacon tracking command (no STRIKE open-loop dash).
 *   P : speed magnitude from distance (chassis_drone_distance_to_speed),
 *       direction = unit pixel-error vector, updated every tick.
 *   D : -KD * d(filtered_error)/dt, brakes when the pixel distance stops
 *       shrinking (overshoot / sideways drift), guaranteeing monotonic
 *       convergence toward the beacon.
 * The summed P+D velocity is clamped to the global max, then mapped to body
 * forward/right with the hardware sign conventions. */
static void chassis_drone_track_compute_command(float err_x_px,
                                                float err_y_px,
                                                float dist_px,
                                                float dt_s,
                                                float *vx_body_cmd,
                                                float *vy_body_cmd)
{
    float speed_p;
    float dir_x;
    float dir_y;
    float vp_x;
    float vp_y;
    float vd_x;
    float vd_y;
    float v_x;
    float v_y;
    float norm_ell;

    if ((vx_body_cmd == 0) || (vy_body_cmd == 0) || (dt_s <= 0.0f))
    {
        return;
    }

    if (dist_px <= HW_DRONE_DEADBAND_PX)
    {
        /* 信标进入正下方 deadband 后不归零, 而是沿最后方向保持最低巡航速度,
         * 让车平滑冲过信标正下方, 避免"减速到0→信标丢失→被COAST拽起"的顿挫。
         * 方向取进入 deadband 前最后的单位误差向量 (无误差向量时退化为纯前进,
         * 这是信标在小车正下方最常见的几何), 速度取前后/侧向最低速度的合成。 */
        float last_dir_x = 0.0f;
        float last_dir_y = 1.0f;
        const float last_err_mag = sqrtf((g_runtime.drone_last_error_x_px * g_runtime.drone_last_error_x_px)
                                       + (g_runtime.drone_last_error_y_px * g_runtime.drone_last_error_y_px));
        if (last_err_mag > 0.5f)
        {
            last_dir_x = g_runtime.drone_last_error_x_px / last_err_mag;
            last_dir_y = g_runtime.drone_last_error_y_px / last_err_mag;
        }
        const float min_fwd = HW_DRONE_MIN_FWD_MPS * fabsf(last_dir_y);
        const float min_right = HW_DRONE_MIN_RIGHT_MPS * fabsf(last_dir_x);
        const float min_speed = sqrtf((min_fwd * min_fwd) + (min_right * min_right));
        v_x = last_dir_x * min_speed;
        v_y = last_dir_y * min_speed;
        *vx_body_cmd = HW_DRONE_FWD_SIGN * v_y;
        *vy_body_cmd = HW_DRONE_RIGHT_SIGN * v_x;
        return;
    }

    /* P term: unit direction * speed-from-distance. */
    speed_p = chassis_drone_distance_to_speed(dist_px);
    dir_x = err_x_px / dist_px;
    dir_y = err_y_px / dist_px;
    vp_x = dir_x * speed_p;
    vp_y = dir_y * speed_p;

    /* D term: derivative of the filtered pixel error. Only active once a
     * previous sample exists; sign is negative so a growing error brakes.
     * 抗噪: 只在真实视觉帧到达拍 (drone_frame_fresh) 计算微分, 且 dt 用真实帧间隔
     * (drone_frame_dt_s) 而非控制周期。同一帧被多个控制拍复用时不更新 D 项,
     * 消除 200Hz 控制环对 30Hz 信号做微分产生的尖峰刹车。 */
    vd_x = 0.0f;
    vd_y = 0.0f;
    if ((g_runtime.drone_last_error_valid != 0U) && (g_runtime.drone_frame_fresh != 0U)
        && (g_runtime.drone_frame_dt_s > 0.001f))
    {
        const float frame_dt = g_runtime.drone_frame_dt_s;
        vd_x = -HW_DRONE_TRACK_KD_MPS_PER_PXPS * ((err_x_px - g_runtime.drone_last_error_x_px) / frame_dt);
        vd_y = -HW_DRONE_TRACK_KD_MPS_PER_PXPS * ((err_y_px - g_runtime.drone_last_error_y_px) / frame_dt);
    }

    v_x = vp_x + vd_x;
    v_y = vp_y + vd_y;

    /* 各向异性椭圆 clamp: (v_x/MAX_RIGHT)^2 + (v_y/MAX_FWD)^2 <= 1。
     * v_x 是像素侧向误差->body 侧向 (right), 受 MAX_RIGHT 约束;
     * v_y 是像素前后误差->body 前后 (forward), 受 MAX_FWD 约束。
     * 用椭圆而非各向同性圆, 让侧向能吃满 MAX_RIGHT, 前后被压到 MAX_FWD。
     * D 项可刹车但不能把车推过 P 曲线 ceiling, 椭圆对 D 项同样有效。 */
    norm_ell = (v_x / HW_DRONE_MAX_RIGHT_MPS) * (v_x / HW_DRONE_MAX_RIGHT_MPS)
             + (v_y / HW_DRONE_MAX_FWD_MPS) * (v_y / HW_DRONE_MAX_FWD_MPS);
    if (norm_ell > 1.0f)
    {
        const float scale = 1.0f / sqrtf(norm_ell);
        v_x *= scale;
        v_y *= scale;
    }

    /* 信标冲刺阶段: 信标进入 DASH_ZONE(40px) 内, 沿 PD 方向把速度提到 DASH_SPEED,
     * 但前后轴仍受 MAX_FWD 限制, 避免正前方信标过冲出界。
     * 只放大不缩小: 若 PD 输出已 > DASH_SPEED 则不降速。纯缩放不扭曲方向,
     * 冲刺方向 = PD 单位误差向量 (实时跟随, 非冻结)。仍走后续 slew 平滑加速。 */
    if (dist_px <= HW_DRONE_DASH_ZONE_PX)
    {
        const float speed_now = sqrtf((v_x * v_x) + (v_y * v_y));
        if ((speed_now > 0.001f) && (speed_now < HW_DRONE_DASH_SPEED_MPS))
        {
            const float dash_scale = HW_DRONE_DASH_SPEED_MPS / speed_now;
            v_x *= dash_scale;
            v_y *= dash_scale;
        }
        v_x = chassis_clamp(v_x, -HW_DRONE_MAX_RIGHT_MPS, HW_DRONE_MAX_RIGHT_MPS);
        v_y = chassis_clamp(v_y, -HW_DRONE_MAX_FWD_MPS, HW_DRONE_MAX_FWD_MPS);
    }

    /* Map pixel-frame (x=right, y=forward) to body frame (vy>0 right, vx>0 fwd). */
    *vx_body_cmd = HW_DRONE_FWD_SIGN * v_y;
    *vy_body_cmd = HW_DRONE_RIGHT_SIGN * v_x;
}

static void chassis_drone_lock_strike_velocity(float err_x_px, float err_y_px)
{
    float vx = g_runtime.cmd_vx_body_state;
    float vy = g_runtime.cmd_vy_body_state;
    float speed = sqrtf((vx * vx) + (vy * vy));

    if (speed < HW_BEACON_COAST_MIN_VEL_MPS)
    {
        float dist = sqrtf((err_x_px * err_x_px) + (err_y_px * err_y_px));
        if ((dist <= HW_DRONE_DEADBAND_PX) && (g_runtime.drone_vb_valid != 0U))
        {
            err_x_px = g_runtime.drone_last_vb_x_px;
            err_y_px = g_runtime.drone_last_vb_y_px;
            dist = sqrtf((err_x_px * err_x_px) + (err_y_px * err_y_px));
        }

        if (dist > HW_DRONE_DEADBAND_PX)
        {
            vx = HW_DRONE_FWD_SIGN * (err_y_px / dist);
            vy = HW_DRONE_RIGHT_SIGN * (err_x_px / dist);
            speed = 1.0f;
        }
        else
        {
            vx = HW_DRONE_FWD_SIGN;
            vy = 0.0f;
            speed = 1.0f;
        }
    }

    if (speed < HW_DRONE_DASH_SPEED_MPS)
    {
        const float scale = HW_DRONE_DASH_SPEED_MPS / speed;
        vx *= scale;
        vy *= scale;
    }

    vx = chassis_clamp(vx, -HW_DRONE_MAX_FWD_MPS, HW_DRONE_MAX_FWD_MPS);
    vy = chassis_clamp(vy, -HW_DRONE_MAX_RIGHT_MPS, HW_DRONE_MAX_RIGHT_MPS);

    g_runtime.cmd_vx_body_state = vx;
    g_runtime.cmd_vy_body_state = vy;
}

static void chassis_position_move_reset(void)
{
    g_runtime.position_move_state = CHASSIS_POSITION_MOVE_IDLE;
    g_runtime.position_press_armed = 0U;
    g_runtime.position_edit_dx_m = 0.0f;
    g_runtime.position_edit_dy_m = 0.0f;
    g_runtime.position_target_forward_m = 0.0f;
    g_runtime.position_target_right_m = 0.0f;
    g_status.position_move_state = CHASSIS_POSITION_MOVE_IDLE;
}

static void chassis_position_move_set_state(chassis_position_move_state_t state)
{
    g_runtime.position_move_state = (uint8)state;
    g_status.position_move_state = (uint8)state;
}

static void chassis_position_move_abort(void)
{
    chassis_reset_command_states();
    g_status.pos_error_x_m = 0.0f;
    g_status.pos_error_y_m = 0.0f;
    g_status.distance_to_target_m = 0.0f;
    chassis_position_move_set_state(CHASSIS_POSITION_MOVE_DONE);
}

static void chassis_position_move_commit(float yaw_deg)
{
    float target_forward_m = 0.0f;
    float target_right_m = 0.0f;
    float target_field_x_m = 0.0f;
    float target_field_y_m = 0.0f;

    chassis_imu_to_body_move(g_runtime.position_edit_dx_m,
                             g_runtime.position_edit_dy_m,
                             &target_forward_m,
                             &target_right_m);
    chassis_body_to_field_move(target_forward_m,
                               target_right_m,
                               yaw_deg,
                               &target_field_x_m,
                               &target_field_y_m);

    g_runtime.position_target_forward_m = g_status.pose_x_m + target_field_x_m;
    g_runtime.position_target_right_m = g_status.pose_y_m + target_field_y_m;

    g_status.target_x_m = g_runtime.position_edit_dx_m;
    g_status.target_y_m = g_runtime.position_edit_dy_m;
    g_status.target_valid = 1U;
    g_status.target_yaw_deg = g_runtime.yaw_lock_deg;
    g_status.heading_valid = g_runtime.yaw_lock_valid ? 1U : 0U;
    g_status.pos_error_x_m = 0.0f;
    g_status.pos_error_y_m = 0.0f;
    g_status.distance_to_target_m = 0.0f;

    g_runtime.position_edit_dx_m = 0.0f;
    g_runtime.position_edit_dy_m = 0.0f;
    g_runtime.cmd_vx_field_state = g_runtime.odom.vx_field_mps;
    g_runtime.cmd_vy_field_state = g_runtime.odom.vy_field_mps;
    chassis_position_move_set_state(CHASSIS_POSITION_MOVE_RUN);
}

static void chassis_apply_body_command_slew(float *vx_body_cmd, float *vy_body_cmd, float dt_s)
{
    const float max_step = HW_MAX_LINEAR_ACCEL_MPS2 * dt_s;
    float cur_vx;
    float cur_vy;
    float dv_x;
    float dv_y;
    float dv_norm;
    float max_dv_norm;

    if ((vx_body_cmd == 0) || (vy_body_cmd == 0) || (dt_s <= 0.0f))
    {
        return;
    }

    /* Vector-magnitude slew instead of per-axis slew.
     * Per-axis slew throttles a single axis when the required direction changes
     * (e.g. beacon moves from ahead to the right), making the car trace an arc /
     * detour instead of heading straight at the beacon. Limiting the MAGNITUDE of
     * the velocity delta vector (|target - current|) lets the direction follow the
     * live P-term unit vector immediately while still capping the acceleration, so
     * the chassis accelerates/brakes smoothly but no longer lags on axis changes.
     * max_dv_norm uses sqrt(2)*max_step so pure speed-up (both axes grow) keeps the
     * same per-axis ceiling as before for motor-safety parity. */
    cur_vx = g_runtime.cmd_vx_body_state;
    cur_vy = g_runtime.cmd_vy_body_state;
    dv_x = *vx_body_cmd - cur_vx;
    dv_y = *vy_body_cmd - cur_vy;
    dv_norm = sqrtf((dv_x * dv_x) + (dv_y * dv_y));
    max_dv_norm = 1.4142f * max_step;
    if (dv_norm > max_dv_norm)
    {
        const float scale = max_dv_norm / dv_norm;
        dv_x *= scale;
        dv_y *= scale;
    }
    g_runtime.cmd_vx_body_state = cur_vx + dv_x;
    g_runtime.cmd_vy_body_state = cur_vy + dv_y;
    *vx_body_cmd = g_runtime.cmd_vx_body_state;
    *vy_body_cmd = g_runtime.cmd_vy_body_state;
}

/* Right-stick position-edit workflow. In DRONE_TRACK mode this function is
 * skipped entirely (see the mode gate above), so the right stick is retired
 * from automatic control. Kept for MANUAL mode parity and future use. */
static void chassis_position_move_update(const proto_remote_data_t *remote, float yaw_deg, float dt_s)
{
    float right_x;
    float right_y;
    uint8 right_x_active;
    uint8 right_y_active;
    uint8 commit_pressed;

    if (dt_s <= 0.0f)
    {
        return;
    }

    if ((g_status.mode == CHASSIS_MODE_SAFE_STOP) || (g_status.mode == CHASSIS_MODE_WHEEL_TUNE))
    {
        g_runtime.position_press_armed = 0U;
        if (g_runtime.position_move_state == CHASSIS_POSITION_MOVE_RUN)
        {
            chassis_position_move_abort();
        }
        else if (g_runtime.position_move_state != CHASSIS_POSITION_MOVE_IDLE)
        {
            chassis_position_move_set_state(CHASSIS_POSITION_MOVE_IDLE);
        }
        return;
    }

    if ((remote == 0) || (g_status.remote_fresh == 0U))
    {
        g_runtime.position_press_armed = 0U;
        return;
    }

    right_x = chassis_normalize_remote_axis(remote->joystick[2]);
    right_y = chassis_normalize_remote_axis(remote->joystick[3]);
    right_x_active = (uint8)(fabsf(right_x) >= CHASSIS_POSITION_EDIT_AXIS_ON);
    right_y_active = (uint8)(fabsf(right_y) >= CHASSIS_POSITION_EDIT_AXIS_ON);

    if (!right_x_active && !right_y_active)
    {
        if ((fabsf(right_x) <= CHASSIS_POSITION_EDIT_AXIS_OFF) &&
            (fabsf(right_y) <= CHASSIS_POSITION_EDIT_AXIS_OFF))
        {
            g_runtime.position_press_armed = 0U;
        }
    }

    if (g_runtime.position_move_state != CHASSIS_POSITION_MOVE_RUN)
    {
        if (right_x_active || right_y_active)
        {
            chassis_position_move_set_state(CHASSIS_POSITION_MOVE_EDIT);
        }

        if (fabsf(right_x) > CHASSIS_POSITION_EDIT_DEADBAND)
        {
            g_runtime.position_edit_dx_m += (-right_x) * HW_POSITION_EDIT_RATE_MPS * dt_s;
        }
        if (fabsf(right_y) > CHASSIS_POSITION_EDIT_DEADBAND)
        {
            g_runtime.position_edit_dy_m += right_y * HW_POSITION_EDIT_RATE_MPS * dt_s;
        }

        g_runtime.position_edit_dx_m = chassis_clamp(g_runtime.position_edit_dx_m,
                                                      -HW_POSITION_EDIT_MAX_OFFSET_M,
                                                      HW_POSITION_EDIT_MAX_OFFSET_M);
        g_runtime.position_edit_dy_m = chassis_clamp(g_runtime.position_edit_dy_m,
                                                      -HW_POSITION_EDIT_MAX_OFFSET_M,
                                                      HW_POSITION_EDIT_MAX_OFFSET_M);

        commit_pressed = chassis_remote_button_pressed(remote, REMOTE_RIGHT_STICK_PRESS_KEY, &g_runtime.position_press_armed);
        if (commit_pressed != 0U)
        {
            if (g_status.pose_valid && g_runtime.yaw_lock_valid)
            {
                chassis_position_move_commit(yaw_deg);
            }
        }

        if (g_runtime.position_move_state != CHASSIS_POSITION_MOVE_RUN)
        {
            g_status.pos_error_x_m = 0.0f;
            g_status.pos_error_y_m = 0.0f;
            g_status.distance_to_target_m = 0.0f;
        }
    }
    else
    {
        const float err_field_x = g_runtime.position_target_forward_m - g_status.pose_x_m;
        const float err_field_y = g_runtime.position_target_right_m - g_status.pose_y_m;
        const float distance_m = sqrtf(err_field_x * err_field_x + err_field_y * err_field_y);
        const float speed_mps = sqrtf((g_runtime.odom.vx_field_mps * g_runtime.odom.vx_field_mps) +
                                      (g_runtime.odom.vy_field_mps * g_runtime.odom.vy_field_mps));
        float err_left_m = 0.0f;
        float err_front_m = 0.0f;

        chassis_field_to_imu_error(err_field_x,
                                   err_field_y,
                                   yaw_deg,
                                   &err_left_m,
                                   &err_front_m);
        g_status.pos_error_x_m = err_left_m;
        g_status.pos_error_y_m = err_front_m;
        g_status.distance_to_target_m = distance_m;

        if ((distance_m <= HW_POSITION_TARGET_TOL_M) &&
            (speed_mps <= CHASSIS_POSITION_ARRIVE_VEL_MPS))
        {
            chassis_position_move_abort();
        }
    }
}

static void chassis_reset_pid_and_output(void)
{
    for (uint8 i = 0; i < 4U; ++i)
    {
        pid_reset(&g_wheel_pid[i]);
        g_status.wheel_target_mps[i] = 0.0f;
        g_status.wheel_measured_mps[i] = 0.0f;
        g_status.wheel_error_mps[i] = 0.0f;
        g_status.motor_duty[i] = 0.0f;
        g_status.wheel_output_saturated[i] = 0U;
    }
}

static void chassis_reset_heading_controller(float yaw_deg)
{
    g_runtime.yaw_lock_continuous_deg = yaw_deg;
    g_runtime.yaw_lock_deg = chassis_wrap_deg(yaw_deg);
    g_runtime.yaw_lock_valid = 1U;
    g_runtime.heading_integral = 0.0f;
    g_status.target_yaw_deg = g_runtime.yaw_lock_deg;
    g_status.heading_valid = 1U;
}

static uint8 chassis_stationary_hint_from_status(void)
{
    const float wheel_speed_limit = 0.03f;
    const float motor_duty_limit = 180.0f;
    const float cmd_speed_limit = 0.03f;
    const float cmd_omega_limit = 0.05f;

    if (g_status.position_move_state == CHASSIS_POSITION_MOVE_RUN)
    {
        return 0U;
    }

    if ((fabsf(g_status.cmd_vx_body_mps) > cmd_speed_limit) ||
        (fabsf(g_status.cmd_vy_body_mps) > cmd_speed_limit) ||
        (fabsf(g_status.cmd_omega_radps) > cmd_omega_limit))
    {
        return 0U;
    }

    for (uint8 i = 0U; i < 4U; ++i)
    {
        if (fabsf(g_status.wheel_measured_mps[i]) > wheel_speed_limit)
        {
            return 0U;
        }

        if (fabsf(g_status.motor_duty[i]) > motor_duty_limit)
        {
            return 0U;
        }
    }

    return 1U;
}

/* 注意: 原实现 chssäis_update_motion_yaw_trim() 会在平移时把陀螺z速率积分进零偏,
 * 与 IMU 自身的静止ZRU(hw_imu_update_bias)冲突, 造成"双重扣零偏" ->
 * 一旦学到一个非零trim(平移时无法区分真实转动与零偏), 回静止后 gyro_corr[2]≈-trim
 * 成为恒定误差, AHRS匀速漂移, yaw_hold跟转, 最终车体转80°+. 无磁力计时z零偏
 * 仅在静止可观测, 故此学习器是有害设计, 已移除. 保留函数体(被周期调用)用于持续
 * 冲刷trim清零, 确保任何残留坏值不会在运行中被意外恢复. */
static void chassis_update_motion_yaw_trim(float gyro_z_radps, float dt_s)
{
    (void)gyro_z_radps;
    (void)dt_s;

    if (g_runtime.motion_yaw_trim_dps != 0.0f)
    {
        g_runtime.motion_yaw_trim_dps = 0.0f;
        hw_imu_set_gyro_z_trim(0.0f);
    }
}

static void chassis_manual_snap_axes(float *joy_x, float *joy_y)
{
    float abs_x;
    float abs_y;

    if ((joy_x == 0) || (joy_y == 0))
    {
        return;
    }

    abs_x = fabsf(*joy_x);
    abs_y = fabsf(*joy_y);

    if ((abs_y >= CHASSIS_MANUAL_AXIS_SNAP_MIN) && (abs_x <= (abs_y * CHASSIS_MANUAL_AXIS_SNAP_RATIO)))
    {
        *joy_x = 0.0f;
    }
    else if ((abs_x >= CHASSIS_MANUAL_AXIS_SNAP_MIN) && (abs_y <= (abs_x * CHASSIS_MANUAL_AXIS_SNAP_RATIO)))
    {
        *joy_y = 0.0f;
    }
}

static uint8 chassis_run_position_control(float yaw_deg,
                                          float gyro_z_radps,
                                          float dt_s,
                                          float *vx_field_cmd,
                                          float *vy_field_cmd,
                                          float *vx_body_cmd,
                                          float *vy_body_cmd,
                                          float *omega_cmd)
{
    const float err_field_x = g_runtime.position_target_forward_m - g_status.pose_x_m;
    const float err_field_y = g_runtime.position_target_right_m - g_status.pose_y_m;
    const float distance_m = sqrtf((err_field_x * err_field_x) + (err_field_y * err_field_y));
    const float braking_speed_limit = sqrtf(2.0f * HW_MAX_LINEAR_ACCEL_MPS2 * distance_m);
    const float max_step = HW_MAX_LINEAR_ACCEL_MPS2 * dt_s;
    const float yaw_rad = yaw_deg * CHASSIS_DEG_TO_RAD;
    const float transl_speed_mps = sqrtf((g_runtime.odom.vx_field_mps * g_runtime.odom.vx_field_mps) +
                                         (g_runtime.odom.vy_field_mps * g_runtime.odom.vy_field_mps));

    *vx_field_cmd = CHASSIS_POS_KP * err_field_x - CHASSIS_VEL_DAMP_K * g_runtime.odom.vx_field_mps;
    *vy_field_cmd = CHASSIS_POS_KP * err_field_y - CHASSIS_VEL_DAMP_K * g_runtime.odom.vy_field_mps;

    if (fabsf(err_field_x) < CHASSIS_POS_DEADBAND_M)
    {
        *vx_field_cmd = 0.0f;
    }
    if (fabsf(err_field_y) < CHASSIS_POS_DEADBAND_M)
    {
        *vy_field_cmd = 0.0f;
    }

    chassis_limit_vector(vx_field_cmd, vy_field_cmd, HW_MAX_LINEAR_SPEED_MPS);
    if (braking_speed_limit < HW_MAX_LINEAR_SPEED_MPS)
    {
        chassis_limit_vector(vx_field_cmd, vy_field_cmd, braking_speed_limit);
    }

    if ((distance_m < CHASSIS_POS_DEADBAND_M) &&
        (fabsf(g_runtime.odom.vx_field_mps) < CHASSIS_ARRIVE_VEL_MPS) &&
        (fabsf(g_runtime.odom.vy_field_mps) < CHASSIS_ARRIVE_VEL_MPS))
    {
        *vx_field_cmd = 0.0f;
        *vy_field_cmd = 0.0f;
    }

    g_runtime.cmd_vx_field_state = chassis_move_toward(g_runtime.cmd_vx_field_state, *vx_field_cmd, max_step);
    g_runtime.cmd_vy_field_state = chassis_move_toward(g_runtime.cmd_vy_field_state, *vy_field_cmd, max_step);

    *vx_field_cmd = g_runtime.cmd_vx_field_state;
    *vy_field_cmd = g_runtime.cmd_vy_field_state;
    *vx_body_cmd = cosf(yaw_rad) * (*vx_field_cmd) + sinf(yaw_rad) * (*vy_field_cmd);
    *vy_body_cmd = -sinf(yaw_rad) * (*vx_field_cmd) + cosf(yaw_rad) * (*vy_field_cmd);
    *omega_cmd = chassis_yaw_hold_strict(yaw_deg, gyro_z_radps, transl_speed_mps, dt_s);

    chassis_field_to_imu_error(err_field_x,
                               err_field_y,
                               yaw_deg,
                               &g_status.pos_error_x_m,
                               &g_status.pos_error_y_m);
    g_status.distance_to_target_m = distance_m;
    g_status.target_yaw_deg = g_runtime.yaw_lock_deg;
    g_status.heading_valid = g_runtime.yaw_lock_valid ? 1U : 0U;

    return ((distance_m <= HW_POSITION_TARGET_TOL_M) &&
            (fabsf(g_runtime.odom.vx_field_mps) < CHASSIS_POSITION_ARRIVE_VEL_MPS) &&
            (fabsf(g_runtime.odom.vy_field_mps) < CHASSIS_POSITION_ARRIVE_VEL_MPS)) ? 1U : 0U;
}

void chassis_control_init(void)
{
    memset(&g_runtime, 0, sizeof(g_runtime));
    memset(&g_status, 0, sizeof(g_status));
    g_runtime.last_mode = CHASSIS_MODE_SAFE_STOP;
    g_status.mode = CHASSIS_MODE_SAFE_STOP;
    g_runtime.yaw_lock_deg = 0.0f;
    g_runtime.yaw_lock_continuous_deg = 0.0f;
    g_runtime.yaw_lock_valid = 0U;
    g_runtime.yaw_continuous_valid = 0U;
    g_runtime.last_attitude_yaw_deg = 0.0f;
    g_runtime.heading_reset_armed = 0U;
    g_runtime.motion_yaw_trim_dps = 0.0f;
    g_chassis_wheel_pid_tune_target_mps = HW_WHEEL_TUNE_TARGET_MPS;
    g_chassis_wheel_pid_tune_wheel = 0U;
    g_chassis_wheel_pid_tune_param = 0U;
    g_chassis_wheel_pid_tune_enabled = 0U;
    hw_imu_set_gyro_z_trim(0.0f);
    chassis_startup_lock_reset();
    chassis_position_move_reset();
    chassis_reset_command_states();

    for (uint8 i = 0; i < 4U; ++i)
    {
        pid_init(&g_wheel_pid[i],
                 s_wheel_pid_kp[i],
                 s_wheel_pid_ki[i],
                 s_wheel_pid_kd[i],
                 CHASSIS_WHEEL_PID_OUT_MAX,
                 CHASSIS_WHEEL_PID_I_MAX);
    }
}

void chassis_control_reset(void)
{
    odometry_reset(&g_runtime.odom);
    g_runtime.yaw_lock_valid = 0U;
    g_runtime.yaw_lock_deg = 0.0f;
    g_runtime.yaw_lock_continuous_deg = 0.0f;
    g_runtime.heading_integral = 0.0f;
    g_runtime.last_yaw_deg = 0.0f;
    g_runtime.last_attitude_yaw_deg = 0.0f;
    g_runtime.yaw_continuous_valid = 0U;
    g_runtime.cmd_vx_field_state = 0.0f;
    g_runtime.cmd_vy_field_state = 0.0f;
    g_runtime.gyro_z_bias_radps = 0.0f;
    g_runtime.gyro_z_bias_inflight_radps = 0.0f;
    g_runtime.motion_yaw_trim_dps = 0.0f;
    g_runtime.startup_gyro_sum_radps = 0.0f;
    g_runtime.startup_lock_elapsed_s = 0.0f;
    g_runtime.startup_lock_samples = 0U;
    g_runtime.startup_lock_done = 0U;
    g_runtime.heading_reset_armed = 0U;
    hw_imu_set_gyro_z_trim(0.0f);
    chassis_tune_reset_latch();
    chassis_position_move_reset();

    memset(&g_status, 0, sizeof(g_status));
    g_status.mode = CHASSIS_MODE_SAFE_STOP;
    g_runtime.last_mode = CHASSIS_MODE_SAFE_STOP;
    chassis_startup_lock_reset();
    chassis_reset_pid_and_output();
    chassis_reset_command_states();
}

void chassis_control_set_mode(chassis_mode_t mode)
{
    g_status.mode = mode;
}

void chassis_control_set_remote(const proto_remote_data_t *remote, uint8 fresh, uint16 age_ms)
{
    if (remote != 0)
    {
        g_runtime.remote = *remote;
    }

    g_status.remote_fresh = fresh ? 1U : 0U;
    g_status.remote_age_ms = age_ms;
}

void chassis_control_set_drone(const proto_drone_data_t *drone, uint8 fresh, uint16 age_ms)
{
    if (drone != 0)
    {
        g_runtime.drone = *drone;
    }

    /* 帧边沿检测: app 层在新帧到达时把 age_ms 归零。利用这一信号锁定真实帧间隔,
     * 供 D 项微分使用, 避免 200Hz 控制环对 30Hz 视觉做微分产生尖峰。 */
    if ((fresh != 0U) && (age_ms == 0U))
    {
        const uint16 frame_dt_ms = (uint16)(g_runtime.drone_age_accum_ms + HW_CONTROL_PERIOD_MS);
        g_runtime.drone_frame_dt_s = (float)frame_dt_ms * 0.001f;
        g_runtime.drone_age_accum_ms = 0U;
        g_runtime.drone_frame_fresh = 1U;
    }
    else
    {
        if (g_runtime.drone_age_accum_ms < 0xFFFFU)
        {
            g_runtime.drone_age_accum_ms = (uint16)(g_runtime.drone_age_accum_ms + HW_CONTROL_PERIOD_MS);
        }
        g_runtime.drone_frame_fresh = 0U;
    }

    g_status.drone_fresh = fresh ? 1U : 0U;
    g_status.drone_age_ms = age_ms;
    g_status.drone_flags = g_runtime.drone.flags;
    g_status.drone_move_valid = ((fresh != 0U) &&
                                 ((g_runtime.drone.flags & PROTO_DRONE_FLAG_MOVE_VALID) != 0U)) ? 1U : 0U;
    g_status.drone_v_x_px = g_runtime.drone.v_x_px;
    g_status.drone_v_y_px = g_runtime.drone.v_y_px;
    g_status.drone_beacon_x_px = g_runtime.drone.beacon_x_px;
    g_status.drone_beacon_y_px = g_runtime.drone.beacon_y_px;
    g_status.drone_vb_x_px = g_runtime.drone.vb_x_px;
    g_status.drone_vb_y_px = g_runtime.drone.vb_y_px;
    g_status.drone_v_conf_permille = g_runtime.drone.v_conf_permille;
    g_status.drone_beacon_conf_permille = g_runtime.drone.beacon_conf_permille;
}

void chassis_control_set_coast(uint16 remaining_ms)
{
    g_runtime.coast_remaining_ms = remaining_ms;
}

void chassis_control_step(const int16 enc_counts[4], float dt_s, const flight_attitude_t *attitude)
{
    float yaw_deg = g_runtime.last_yaw_deg;
    float gyro_z_radps = g_runtime.odom.omega_radps;
    float vx_field_cmd = 0.0f;
    float vy_field_cmd = 0.0f;
    float vx_body_cmd = 0.0f;
    float vy_body_cmd = 0.0f;
    float omega_cmd = 0.0f;
    float wheel_target_mps[4] = {0};
    float transl_speed_mps = 0.0f;   /* 车体平移速度模, 喂给 yaw_hold 做静止防抖判据 */
    uint8 command_active = 0U;
    uint8 attitude_valid = 0U;
    float left_x = 0.0f;
    float left_y = 0.0f;
    uint8 manual_override = 0U;

    if ((enc_counts == 0) || (dt_s <= 0.0f))
    {
        return;
    }

    if ((attitude != 0) && (attitude->valid != 0U))
    {
        if (!g_runtime.yaw_continuous_valid)
        {
            g_runtime.last_yaw_deg = attitude->yaw;
            g_runtime.last_attitude_yaw_deg = attitude->yaw;
            g_runtime.yaw_continuous_valid = 1U;
        }
        else
        {
            g_runtime.last_yaw_deg += chassis_wrap_deg(attitude->yaw - g_runtime.last_attitude_yaw_deg);
            g_runtime.last_attitude_yaw_deg = attitude->yaw;
        }
        yaw_deg = g_runtime.last_yaw_deg;
        gyro_z_radps = attitude->gyro_dps[2] * CHASSIS_DEG_TO_RAD;
        attitude_valid = 1U;
    }
    else
    {
        g_runtime.last_yaw_deg += g_runtime.odom.omega_radps * dt_s * CHASSIS_RAD_TO_DEG;
        yaw_deg = g_runtime.last_yaw_deg;
        gyro_z_radps = g_runtime.odom.omega_radps;
    }
    g_runtime.last_yaw_deg = yaw_deg;

    odometry_update(&g_runtime.odom, enc_counts, dt_s, yaw_deg);
    g_status.odom_x_m = g_runtime.odom.x_m;
    g_status.odom_y_m = g_runtime.odom.y_m;
    g_status.pose_yaw_deg = chassis_wrap_deg(yaw_deg);
    g_status.vel_x_field_mps = g_runtime.odom.vx_field_mps;
    g_status.vel_y_field_mps = g_runtime.odom.vy_field_mps;
    transl_speed_mps = sqrtf((g_runtime.odom.vx_field_mps * g_runtime.odom.vx_field_mps) +
                             (g_runtime.odom.vy_field_mps * g_runtime.odom.vy_field_mps));

    for (uint8 i = 0; i < 4U; ++i)
    {
        g_status.wheel_measured_mps[i] = g_runtime.odom.wheel_speed_mps[i];
    }

    chassis_startup_lock_step(yaw_deg, gyro_z_radps, dt_s, attitude_valid);
    if (!g_runtime.startup_lock_done)
    {
        chassis_reset_pid_and_output();
        g_status.target_yaw_deg = chassis_wrap_deg(yaw_deg);
        g_status.heading_valid = 0U;
        g_status.enabled = 0U;
        g_status.cmd_vx_field_mps = 0.0f;
        g_status.cmd_vy_field_mps = 0.0f;
        g_status.cmd_vx_body_mps = 0.0f;
        g_status.cmd_vy_body_mps = 0.0f;
        g_status.cmd_omega_radps = 0.0f;
        g_status.yaw_error_deg = 0.0f;
        return;
    }

    if (attitude_valid)
    {
        chassis_update_motion_yaw_trim(gyro_z_radps, dt_s);
    }

    if (g_runtime.odom.valid)
    {
        if (!g_status.pose_valid)
        {
            g_status.pose_x_m = g_runtime.odom.x_m;
            g_status.pose_y_m = g_runtime.odom.y_m;
            g_status.pose_valid = 1U;
        }
        else
        {
            g_status.pose_x_m += g_runtime.odom.delta_x_field_m;
            g_status.pose_y_m += g_runtime.odom.delta_y_field_m;
        }
    }

    if (g_status.mode != g_runtime.last_mode)
    {
        if ((g_runtime.last_mode == CHASSIS_MODE_DRONE_TRACK) &&
            (g_status.mode != CHASSIS_MODE_DRONE_TRACK))
        {
            g_runtime.drone_track_state = CHASSIS_DRONE_TRACK_APPROACH;
            g_runtime.drone_error_filter_valid = 0U;
            g_runtime.drone_last_error_valid = 0U;
            g_runtime.drone_cross_committed = 0U;
            g_runtime.drone_cross_commit_ms = 0U;
            g_runtime.drone_vb_valid = 0U;
            g_runtime.drone_stuck_detect_ms = 0U;
            g_runtime.drone_stuck_escape_ms = 0U;
            g_runtime.drone_stuck_retry_count = 0U;
        }

        if (g_status.mode == CHASSIS_MODE_SAFE_STOP)
        {
            chassis_reset_command_states();
            chassis_reset_pid_and_output();
        }
        else if (g_status.mode == CHASSIS_MODE_WHEEL_TUNE)
        {
            chassis_tune_reset_latch();
        }
        else if (g_status.mode == CHASSIS_MODE_MANUAL)
        {
            chassis_seed_command_states_from_odometry();
        }
        else if (g_status.mode == CHASSIS_MODE_DRONE_TRACK)
        {
            chassis_seed_command_states_from_odometry();
            g_runtime.drone_track_state = CHASSIS_DRONE_TRACK_APPROACH;
            g_runtime.drone_error_filter_valid = 0U;
            g_runtime.drone_last_error_valid = 0U;
            g_runtime.drone_cross_committed = 0U;
            g_runtime.drone_cross_commit_ms = 0U;
            g_runtime.drone_vb_valid = 0U;
            g_runtime.drone_stuck_detect_ms = 0U;
            g_runtime.drone_stuck_escape_ms = 0U;
            g_runtime.drone_stuck_retry_count = 0U;
            if (!g_runtime.yaw_lock_valid)
            {
                chassis_reset_heading_controller(yaw_deg);
            }
        }
        else if (!g_runtime.yaw_lock_valid)
        {
            chassis_reset_heading_controller(yaw_deg);
        }

        g_runtime.last_mode = g_status.mode;
    }

    if (g_status.mode == CHASSIS_MODE_DRONE_TRACK)
    {
        g_runtime.position_press_armed = 0U;
        g_runtime.position_move_state = CHASSIS_POSITION_MOVE_IDLE;
        g_status.position_move_state = CHASSIS_POSITION_MOVE_IDLE;
    }
    else
    {
        chassis_position_move_update(&g_runtime.remote, yaw_deg, dt_s);
    }
    g_status.position_move_state = g_runtime.position_move_state;
    g_status.position_edit_dx_m = g_runtime.position_edit_dx_m;
    g_status.position_edit_dy_m = g_runtime.position_edit_dy_m;

    if (g_status.remote_fresh)
    {
        if (chassis_remote_button_pressed(&g_runtime.remote, REMOTE_HEADING_RESET_KEY, &g_runtime.heading_reset_armed))
        {
            if (g_hw_imu_stationary_confirmed != 0U)
            {
                chassis_reset_heading_controller(yaw_deg);
            }
        }

        left_x = chassis_normalize_remote_axis(g_runtime.remote.joystick[0]);
        left_y = chassis_normalize_remote_axis(g_runtime.remote.joystick[1]);
        chassis_manual_snap_axes(&left_x, &left_y);
        manual_override = (uint8)((fabsf(left_x) > CHASSIS_POSITION_EDIT_AXIS_ON) ||
                                  (fabsf(left_y) > CHASSIS_POSITION_EDIT_AXIS_ON));
    }

    if ((g_runtime.position_move_state == CHASSIS_POSITION_MOVE_RUN) && manual_override)
    {
        chassis_position_move_abort();
        g_runtime.position_move_state = CHASSIS_POSITION_MOVE_IDLE;
        g_status.position_move_state = CHASSIS_POSITION_MOVE_IDLE;
    }

    g_status.joy_x_norm = 0.0f;
    g_status.joy_y_norm = 0.0f;
    g_status.user_cmd_x_mps = 0.0f;
    g_status.user_cmd_y_mps = 0.0f;
    g_status.tune_active = 0U;
    g_status.tune_wheel_index = 0U;
    g_status.tune_param_index = 0U;
    g_status.tune_target_mps = 0.0f;
    g_status.tune_selected_value = 0.0f;
    g_status.tune_selected_step = 0.0f;

    if (g_runtime.position_move_state == CHASSIS_POSITION_MOVE_RUN)
    {
        command_active = 1U;
        g_status.position_move_state = CHASSIS_POSITION_MOVE_RUN;
        if (g_status.pose_valid && g_runtime.yaw_lock_valid)
        {
            const uint8 arrived = chassis_run_position_control(yaw_deg,
                                                               gyro_z_radps,
                                                               dt_s,
                                                               &vx_field_cmd,
                                                               &vy_field_cmd,
                                                               &vx_body_cmd,
                                                               &vy_body_cmd,
                                                               &omega_cmd);
            if (arrived != 0U)
            {
                chassis_position_move_abort();
                vx_field_cmd = 0.0f;
                vy_field_cmd = 0.0f;
                vx_body_cmd = 0.0f;
                vy_body_cmd = 0.0f;
                omega_cmd = 0.0f;
                command_active = 0U;
            }
        }
        else
        {
            chassis_position_move_abort();
            vx_field_cmd = 0.0f;
            vy_field_cmd = 0.0f;
            vx_body_cmd = 0.0f;
            vy_body_cmd = 0.0f;
            omega_cmd = 0.0f;
            command_active = 0U;
        }
    }
    else switch (g_status.mode)
    {
        case CHASSIS_MODE_WHEEL_TUNE:
        {
            float tune_target = g_chassis_wheel_pid_tune_target_mps;

            if (g_status.remote_fresh)
            {
                float left_x = chassis_normalize_remote_axis(g_runtime.remote.joystick[0]);
                float left_y = chassis_normalize_remote_axis(g_runtime.remote.joystick[1]);
                float right_y = chassis_normalize_remote_axis(g_runtime.remote.joystick[3]);

                chassis_tune_apply_remote_input(left_x, left_y, right_y);
            }

            tune_target = chassis_clamp(tune_target, -HW_MAX_WHEEL_SPEED_MPS, HW_MAX_WHEEL_SPEED_MPS);
            g_chassis_wheel_pid_tune_target_mps = tune_target;

            vx_body_cmd = tune_target;
            vy_body_cmd = 0.0f;
            omega_cmd = 0.0f;
            command_active = (fabsf(tune_target) > 0.0001f);

            g_status.tune_active = 1U;
            g_status.tune_wheel_index = g_chassis_wheel_pid_tune_wheel % 4U;
            g_status.tune_param_index = g_chassis_wheel_pid_tune_param % CHASSIS_TUNE_PARAM_COUNT;
            g_status.tune_target_mps = g_chassis_wheel_pid_tune_target_mps;
            g_status.tune_selected_step = chassis_tune_param_step(g_status.tune_param_index);

            switch (g_status.tune_param_index)
            {
                case CHASSIS_TUNE_PARAM_KP:
                    g_status.tune_selected_value = g_wheel_pid[g_status.tune_wheel_index].kp;
                    break;
                case CHASSIS_TUNE_PARAM_KI:
                    g_status.tune_selected_value = g_wheel_pid[g_status.tune_wheel_index].ki;
                    break;
                case CHASSIS_TUNE_PARAM_KD:
                    g_status.tune_selected_value = g_wheel_pid[g_status.tune_wheel_index].kd;
                    break;
                default:
                    g_status.tune_selected_value = 0.0f;
                    break;
            }
        } break;

        case CHASSIS_MODE_MANUAL:
        {
            if (g_status.remote_fresh)
            {
                float joy_x = left_x;
                float joy_y = left_y;

                if (!manual_override)
                {
                    joy_x = chassis_normalize_remote_axis(g_runtime.remote.joystick[0]);
                    joy_y = chassis_normalize_remote_axis(g_runtime.remote.joystick[1]);
                    chassis_manual_snap_axes(&joy_x, &joy_y);
                }

                g_status.joy_x_norm = joy_x;
                g_status.joy_y_norm = joy_y;
                g_status.user_cmd_x_mps = joy_x * HW_MANUAL_MAX_LATERAL_SPEED_MPS;
                g_status.user_cmd_y_mps = joy_y * HW_MANUAL_MAX_LINEAR_SPEED_MPS;
                vx_body_cmd = g_status.user_cmd_y_mps;
                vy_body_cmd = joy_x * HW_MANUAL_MAX_LATERAL_SPEED_MPS;
                chassis_apply_body_command_slew(&vx_body_cmd, &vy_body_cmd, dt_s);
                omega_cmd = chassis_yaw_hold(yaw_deg, gyro_z_radps, transl_speed_mps, dt_s);
                command_active = 1U;
                g_status.target_yaw_deg = g_runtime.yaw_lock_deg;
                g_status.heading_valid = g_runtime.yaw_lock_valid ? 1U : 0U;
            }
        } break;

        case CHASSIS_MODE_DRONE_TRACK:
        {
            /* 卡住检测 (move_valid 期间持续监控): 命令大但实测小持续确认窗口 -> 触发前后轴自救。
             * 自救优先级最高: 触发或仍在自救窗口内时, 覆盖命令为纯前后轴强驱动, 跳过PD/COAST。 */
            (void)chassis_drone_update_stuck_detection(g_status.drone_move_valid);

            if (g_runtime.drone_stuck_escape_ms > 0U)
            {
                uint8 escape_done = 0U;
                g_runtime.drone_stuck_escape_ms = (g_runtime.drone_stuck_escape_ms > HW_CONTROL_PERIOD_MS)
                                                ? (uint16)(g_runtime.drone_stuck_escape_ms - HW_CONTROL_PERIOD_MS)
                                                : 0U;
                if (g_runtime.drone_stuck_escape_ms == 0U)
                {
                    /* 自救窗口结束: 重试计数+1, 超限放弃(停车不再尝试); 否则回PD重新追踪。 */
                    if (g_runtime.drone_stuck_retry_count < 0xFFU)
                    {
                        g_runtime.drone_stuck_retry_count++;
                    }
                    if (g_runtime.drone_stuck_retry_count > HW_BEACON_STUCK_MAX_RETRY)
                    {
                        escape_done = 2U;   /* 超限: 停车放弃, 不再驱动 */
                    }
                    else
                    {
                        escape_done = 1U;   /* 正常退出: 回PD, 清cross-commit/滤波重追 */
                        g_runtime.drone_cross_committed = 0U;
                        g_runtime.drone_cross_commit_ms = 0U;
                        g_runtime.drone_error_filter_valid = 0U;
                        g_runtime.drone_last_error_valid = 0U;
                        g_runtime.drone_track_state = CHASSIS_DRONE_TRACK_APPROACH;
                    }
                }

                if (escape_done != 2U)
                {
                    /* 自救动作: 纯前后轴(纯vx)强驱动, omega跑yaw_hold锁头不破, vy=0不走侧向无力轴。 */
                    g_runtime.drone_track_state = CHASSIS_DRONE_TRACK_STUCK_ESCAPE;
                    vx_body_cmd = g_runtime.drone_stuck_escape_dir * HW_BEACON_ESCAPE_SPEED_MPS;
                    vy_body_cmd = 0.0f;
                    /* 自救速度直接写入 state (不走 slew, 保证全力挣脱)。 */
                    g_runtime.cmd_vx_body_state = vx_body_cmd;
                    g_runtime.cmd_vy_body_state = vy_body_cmd;
                    omega_cmd = chassis_yaw_hold(yaw_deg, gyro_z_radps, transl_speed_mps, dt_s);
                    command_active = 1U;
                    g_status.target_yaw_deg = g_runtime.yaw_lock_deg;
                    g_status.heading_valid = g_runtime.yaw_lock_valid ? 1U : 0U;
                }
                else
                {
                    /* 超重试次数: 停车放弃。清速度命令, command_active=0 让外层失能电机。
                     * 不再驱动避免无限来回卡。yaw_hold 仍跑维持锁头。 */
                    g_runtime.cmd_vx_body_state = 0.0f;
                    g_runtime.cmd_vy_body_state = 0.0f;
                    vx_body_cmd = 0.0f;
                    vy_body_cmd = 0.0f;
                    omega_cmd = chassis_yaw_hold(yaw_deg, gyro_z_radps, transl_speed_mps, dt_s);
                    command_active = 1U;   /* 保持 yaw_lock 活跃, 仅平移为零 */
                    g_status.target_yaw_deg = g_runtime.yaw_lock_deg;
                    g_status.heading_valid = g_runtime.yaw_lock_valid ? 1U : 0U;
                }
            }
            else if (g_status.drone_move_valid != 0U)
            {
                float vb_x_px;
                float vb_y_px;
                float vb_dist_px;

                chassis_drone_filter_error((float)g_runtime.drone.vb_x_px,
                                           (float)g_runtime.drone.vb_y_px,
                                           &vb_x_px,
                                           &vb_y_px);
                vb_dist_px = chassis_drone_error_distance(vb_x_px, vb_y_px);

                /* 信标重新远距出现(距离远超卡住区): 认为是新一次接近, 清零卡住重试计数。 */
                if (vb_dist_px > HW_BEACON_STUCK_RETRY_RESET_PX)
                {
                    g_runtime.drone_stuck_retry_count = 0U;
                }

                if (g_runtime.drone_track_state == CHASSIS_DRONE_TRACK_COAST)
                {
                    g_runtime.drone_track_state = CHASSIS_DRONE_TRACK_APPROACH;
                }

                /* 主动击灭承诺区 (move_valid 仍为1期间生效):
                 * - 未锁定: 跑 PD 闭环, 然后做穿越检测 (V-B 距离<CROSS_ZONE 且与上一拍 vb 点积<=0
                 *   = 信标刚从小车下方穿到另一侧 -> 锁定此刻 body 速度方向继续滑行)。
                 * - 已锁定: 冻结 PD 反向追踪, 保持锁定瞬间的 body 速度方向继续冲 (不减速不反向),
                 *   直到 move_valid 掉(进COAST完成击灭) / 距离重新变大(没踩中) / 超时(防永久锁死)。
                 *   锁定期间不走 slew 衰减, 直接复用 cmd_vx/vy_body_state (锁定瞬间的实际速度)。 */
                if (g_runtime.drone_cross_committed == 0U)
                {
                    chassis_drone_track_compute_command(vb_x_px,
                                                         vb_y_px,
                                                         vb_dist_px,
                                                         dt_s,
                                                         &vx_body_cmd,
                                                         &vy_body_cmd);
                    if (g_runtime.drone_frame_fresh != 0U)
                    {
                        chassis_drone_store_last_error(vb_x_px, vb_y_px);
                    }
                    chassis_apply_body_command_slew(&vx_body_cmd, &vy_body_cmd, dt_s);

                    /* 击灭承诺触发 (两路并行, 任一满足即锁定当前 body 速度方向继续冲刺):
                     * A) 方向翻转触发: V-B 距离<CROSS_ZONE 且与上一拍 vb 点积<=0
                     *    (信标刚从小车下方穿到另一侧)。几何尺度偏晚。
                     * B) 距离冲刺触发 (吸收早期方向): 距离<DASH_TRIGGER 且在持续缩小
                     *    (last vb 距离>当前距离 = 真在逼近非抖动)。用进入近距区那一刻的
                     *    稳定方向做冲刺方向, 不依赖近距像素方向 (5px抖动=几十度方向噪声)。
                     * 仅在真实视觉帧到达拍判定, 避免复用帧误触发。 */
                    if ((g_runtime.drone_frame_fresh != 0U) && (g_runtime.drone_vb_valid != 0U))
                    {
                        uint8 commit_now = 0U;
                        if (vb_dist_px < HW_BEACON_STRIKE_CROSS_ZONE_PX)
                        {
                            const float dot = (vb_x_px * g_runtime.drone_last_vb_x_px)
                                            + (vb_y_px * g_runtime.drone_last_vb_y_px);
                            if (dot <= 0.0f)
                            {
                                commit_now = 1U;   /* A) 方向翻转 */
                            }
                        }
                        if ((commit_now == 0U) && (vb_dist_px < HW_BEACON_DASH_TRIGGER_PX))
                        {
                            const float last_dist = sqrtf((g_runtime.drone_last_vb_x_px * g_runtime.drone_last_vb_x_px)
                                                        + (g_runtime.drone_last_vb_y_px * g_runtime.drone_last_vb_y_px));
                            /* 上一拍距离比当前大 = 在逼近 (排除原地抖动误触发)。 */
                            if (last_dist > vb_dist_px)
                            {
                                commit_now = 1U;   /* B) 距离冲刺触发 */
                            }
                        }
                        if (commit_now != 0U)
                        {
                            g_runtime.drone_cross_committed = 1U;
                            g_runtime.drone_cross_commit_ms = 0U;
                            chassis_drone_lock_strike_velocity(vb_x_px, vb_y_px);
                            vx_body_cmd = g_runtime.cmd_vx_body_state;
                            vy_body_cmd = g_runtime.cmd_vy_body_state;
                        }
                    }
                    /* 更新上一拍 vb (用本拍滤波后误差) 供下拍穿越检测。 */
                    g_runtime.drone_last_vb_x_px = vb_x_px;
                    g_runtime.drone_last_vb_y_px = vb_y_px;
                    g_runtime.drone_vb_valid = 1U;
                }
                else
                {
                    /* 已锁定: 检查退出条件。退出条件任一满足即解除锁定, 回到 PD 闭环。 */
                    uint8 exit_commit = 0U;
                    g_runtime.drone_cross_commit_ms = (uint16)(g_runtime.drone_cross_commit_ms + HW_CONTROL_PERIOD_MS);
                    if (g_runtime.drone_cross_commit_ms >= HW_BEACON_CROSS_COMMIT_TIMEOUT_MS)
                    {
                        exit_commit = 1U;   /* 超时防永久锁死 */
                    }
                    else if (vb_dist_px > HW_BEACON_CROSS_COMMIT_ESCAPE_PX)
                    {
                        exit_commit = 1U;   /* 距离重新变大 = 没踩中, 让 PD 重新接管追踪 */
                    }
                    else
                    {
                        /* 仍锁定: 保持锁定瞬间的 body 速度方向继续冲, 不走 slew 衰减。 */
                        vx_body_cmd = g_runtime.cmd_vx_body_state;
                        vy_body_cmd = g_runtime.cmd_vy_body_state;
                    }

                    if (exit_commit != 0U)
                    {
                        g_runtime.drone_cross_committed = 0U;
                        g_runtime.drone_cross_commit_ms = 0U;
                        /* 解锁后本拍仍跑一次 PD, 避免空拍丢指令。 */
                        chassis_drone_track_compute_command(vb_x_px,
                                                             vb_y_px,
                                                             vb_dist_px,
                                                             dt_s,
                                                             &vx_body_cmd,
                                                             &vy_body_cmd);
                        chassis_apply_body_command_slew(&vx_body_cmd, &vy_body_cmd, dt_s);
                    }
                    /* 锁定期间同步更新上一拍 vb, 保持穿越检测基准连续 (解锁后下拍可用)。 */
                    g_runtime.drone_last_vb_x_px = vb_x_px;
                    g_runtime.drone_last_vb_y_px = vb_y_px;
                    g_runtime.drone_vb_valid = 1U;
                }
                omega_cmd = chassis_yaw_hold(yaw_deg, gyro_z_radps, transl_speed_mps, dt_s);
                command_active = 1U;
                g_status.target_yaw_deg = g_runtime.yaw_lock_deg;
                g_status.heading_valid = g_runtime.yaw_lock_valid ? 1U : 0U;
            }
            else if (g_runtime.coast_remaining_ms > 0U)
            {
                /* Beacon disappeared after the pass-over moment. Keep moving
                 * along the locked strike command so the chassis clears the
                 * lamp cover instead of dithering around the beacon edge. */
                g_runtime.drone_track_state = CHASSIS_DRONE_TRACK_COAST;
                /* 信标已消失 (无论是否踩灭), 穿越承诺任务结束, 清除锁定让后续重捕获从新 PD 开始。
                 * 卡住检测也清零 (信标已不在, 不再算"驱向信标却不动")。 */
                g_runtime.drone_cross_committed = 0U;
                g_runtime.drone_cross_commit_ms = 0U;
                g_runtime.drone_stuck_detect_ms = 0U;
                {
                    float coast_speed = sqrtf((g_runtime.cmd_vx_body_state * g_runtime.cmd_vx_body_state) +
                                              (g_runtime.cmd_vy_body_state * g_runtime.cmd_vy_body_state));
                    if (coast_speed >= HW_BEACON_COAST_MIN_VEL_MPS)
                    {
                        float cvx = g_runtime.cmd_vx_body_state;
                        float cvy = g_runtime.cmd_vy_body_state;
                        if (coast_speed < HW_BEACON_COAST_DASH_FLOOR_MPS)
                        {
                            const float scale = HW_BEACON_COAST_DASH_FLOOR_MPS / coast_speed;
                            cvx *= scale;
                            cvy *= scale;
                        }
                        vx_body_cmd = cvx;
                        vy_body_cmd = cvy;
                        omega_cmd = chassis_yaw_hold(yaw_deg, gyro_z_radps, transl_speed_mps, dt_s);
                        command_active = 1U;
                        g_status.target_yaw_deg = g_runtime.yaw_lock_deg;
                        g_status.heading_valid = g_runtime.yaw_lock_valid ? 1U : 0U;
                    }
                }
            }
            else
            {
                g_runtime.drone_track_state = CHASSIS_DRONE_TRACK_APPROACH;
                g_runtime.drone_error_filter_valid = 0U;
                g_runtime.drone_last_error_valid = 0U;
                g_runtime.drone_cross_committed = 0U;
                g_runtime.drone_cross_commit_ms = 0U;
                g_runtime.drone_vb_valid = 0U;
                g_runtime.drone_stuck_detect_ms = 0U;
            }
        } break;

        case CHASSIS_MODE_SAFE_STOP:
        default:
        {
            g_runtime.cmd_vx_field_state = 0.0f;
            g_runtime.cmd_vy_field_state = 0.0f;
            g_status.joy_x_norm = 0.0f;
            g_status.joy_y_norm = 0.0f;
            g_status.pos_error_x_m = 0.0f;
            g_status.pos_error_y_m = 0.0f;
            g_status.distance_to_target_m = 0.0f;
            g_status.tune_active = 0U;
            g_status.tune_wheel_index = 0U;
            g_status.tune_param_index = 0U;
            g_status.tune_target_mps = 0.0f;
            g_status.tune_selected_value = 0.0f;
            g_status.tune_selected_step = 0.0f;

            /* Yaw hold stays active even in SAFE_STOP so the car auto-returns
             * to its locked heading when disturbed (e.g. hand-rotated). This is
             * required for accurate drone-remote movement: the heading lock
             * must persist from power-on, in motion and at rest. Only rotation
             * correction is allowed here; translation stays zero so the car
             * never drifts without an explicit command. */
            if (g_runtime.yaw_lock_valid)
            {
                vx_body_cmd = 0.0f;
                vy_body_cmd = 0.0f;
                omega_cmd = chassis_yaw_hold(yaw_deg, gyro_z_radps, transl_speed_mps, dt_s);
                command_active = 1U;
                g_status.target_yaw_deg = g_runtime.yaw_lock_deg;
                g_status.heading_valid = 1U;
            }
        } break;
    }

    if (command_active)
    {
        float translation_wheel[4];
        float rotation_wheel[4];
        float translation_scale;

        mecanum_inverse(vx_body_cmd, vy_body_cmd, 0.0f, translation_wheel);
        mecanum_inverse(0.0f, 0.0f, omega_cmd, rotation_wheel);
        translation_scale = chassis_limit_translation_scale(translation_wheel, rotation_wheel, HW_MAX_WHEEL_SPEED_MPS);

        for (uint8 i = 0U; i < 4U; ++i)
        {
            wheel_target_mps[i] = (translation_wheel[i] * translation_scale) + rotation_wheel[i];
        }

        mecanum_desaturate(wheel_target_mps, HW_MAX_WHEEL_SPEED_MPS);

        for (uint8 i = 0; i < 4U; ++i)
        {
            float duty_limit;
            float duty_raw;
            float duty_ff;

            g_status.wheel_target_mps[i] = wheel_target_mps[i];

            duty_limit = (g_runtime.position_move_state == CHASSIS_POSITION_MOVE_RUN) ?
                         (float)HW_MOTOR_DUTY_MAX :
                         ((g_status.mode == CHASSIS_MODE_DRONE_TRACK) ?
                          (float)HW_DRONE_TRACK_MAX_DUTY :
                          (float)HW_MANUAL_MAX_DUTY);
            duty_ff = chassis_feedforward_duty(i, wheel_target_mps[i]);
            duty_raw = duty_ff + pid_compute(&g_wheel_pid[i], wheel_target_mps[i], g_runtime.odom.wheel_speed_mps[i], dt_s);
            g_status.motor_duty[i] = chassis_clamp(duty_raw, -duty_limit, duty_limit);

            g_status.wheel_error_mps[i] = g_status.wheel_target_mps[i] - g_status.wheel_measured_mps[i];
            g_status.wheel_output_saturated[i] = (fabsf(g_status.motor_duty[i]) >= (duty_limit - 0.5f)) ? 1U : 0U;
        }
    }
    else
    {
        chassis_reset_pid_and_output();
        memset(wheel_target_mps, 0, sizeof(wheel_target_mps));
    }

    for (uint8 i = 0; i < 4U; ++i)
    {
        g_status.encoder_counts[i] = enc_counts[i];
    }

    g_status.cmd_vx_field_mps = vx_field_cmd;
    g_status.cmd_vy_field_mps = vy_field_cmd;
    g_status.cmd_vx_body_mps = vx_body_cmd;
    g_status.cmd_vy_body_mps = vy_body_cmd;
    g_status.cmd_omega_radps = omega_cmd;
    g_status.startup_lock_done = g_runtime.startup_lock_done;
    g_status.startup_lock_elapsed_s = g_runtime.startup_lock_elapsed_s;
    g_status.gyro_z_bias_radps = g_runtime.gyro_z_bias_radps;
    g_status.gyro_z_motion_trim_radps = g_runtime.motion_yaw_trim_dps * CHASSIS_DEG_TO_RAD;
    g_status.heading_valid = g_runtime.yaw_lock_valid ? 1U : 0U;
    g_status.target_yaw_deg = g_runtime.yaw_lock_deg;
    g_status.enabled = (g_runtime.startup_lock_done != 0U) &&
                       (((g_status.mode == CHASSIS_MODE_MANUAL) && g_status.remote_fresh) ||
                       ((g_status.mode == CHASSIS_MODE_WHEEL_TUNE) && command_active) ||
                        (g_runtime.position_move_state == CHASSIS_POSITION_MOVE_RUN) ||
                        ((g_status.mode == CHASSIS_MODE_DRONE_TRACK) &&
                         g_status.drone_fresh &&
                         (g_status.drone_move_valid || (g_runtime.coast_remaining_ms > 0U))) ||
                        ((g_status.mode == CHASSIS_MODE_SAFE_STOP) &&
                         g_runtime.yaw_lock_valid &&
                         command_active));

    hw_imu_set_stationary_hint(chassis_stationary_hint_from_status());
}

const chassis_status_t *chassis_control_get_status(void)
{
    return &g_status;
}

void chassis_control_get_motor_duty(int16 duty_out[4])
{
    if (duty_out == 0)
    {
        return;
    }

    for (uint8 i = 0; i < 4U; ++i)
    {
        duty_out[i] = (int16)g_status.motor_duty[i];
    }
}
