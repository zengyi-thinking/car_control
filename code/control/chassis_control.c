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

static float chassis_drone_error_to_norm(int16 err_px)
{
    /* 比例 + 软饱和, 取代旧的 bang-bang (全速/全停).
     * 控制曲线:
     *   |err| <= deadband(4px)           : 0      (静止, 抗像素噪声抖动)
     *   deadband < |err| <= PX_NORM(160) : 线性比例 0->1.0 (接近目标减速, 避免过冲)
     *   |err| > PX_NORM                  : 软饱和 1.0      (远处全速接近)
     * 收益: 接近 beacon 时减速而非全速冲, 大幅减少过冲振荡; 无人机视觉~100ms延迟下
     * 比例控制远比 bang-bang 平滑, 电机不再频繁正反转切换. */
    const float err = (float)err_px;
    const float abs_err = fabsf(err);

    if (abs_err <= HW_DRONE_DEADBAND_PX)
    {
        return 0.0f;
    }

    if (abs_err >= HW_DRONE_PX_NORM)
    {
        return (err > 0.0f) ? 1.0f : -1.0f;
    }

    /* 线性比例段: 从 deadband 处的 0 线性升到 PX_NORM 处的 1.0. */
    return ((abs_err - HW_DRONE_DEADBAND_PX) / (HW_DRONE_PX_NORM - HW_DRONE_DEADBAND_PX))
           * ((err > 0.0f) ? 1.0f : -1.0f);
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

    if ((vx_body_cmd == 0) || (vy_body_cmd == 0) || (dt_s <= 0.0f))
    {
        return;
    }

    g_runtime.cmd_vx_body_state = chassis_move_toward(g_runtime.cmd_vx_body_state, *vx_body_cmd, max_step);
    g_runtime.cmd_vy_body_state = chassis_move_toward(g_runtime.cmd_vy_body_state, *vy_body_cmd, max_step);
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
            if (g_status.drone_move_valid != 0U)
            {
                const float fwd_norm = chassis_drone_error_to_norm(g_runtime.drone.vb_y_px);
                const float right_norm = chassis_drone_error_to_norm(g_runtime.drone.vb_x_px);

                vx_body_cmd = HW_DRONE_FWD_SIGN * fwd_norm * HW_DRONE_MAX_FWD_MPS;
                vy_body_cmd = HW_DRONE_RIGHT_SIGN * right_norm * HW_DRONE_MAX_RIGHT_MPS;
                chassis_apply_body_command_slew(&vx_body_cmd, &vy_body_cmd, dt_s);
                omega_cmd = chassis_yaw_hold(yaw_deg, gyro_z_radps, transl_speed_mps, dt_s);
                command_active = 1U;
                g_status.target_yaw_deg = g_runtime.yaw_lock_deg;
                g_status.heading_valid = g_runtime.yaw_lock_valid ? 1U : 0U;
            }
            else if (g_runtime.coast_remaining_ms > 0U)
            {
                /* Beacon pass-over coast: the beacon went under the car.
                 * Hold the last body velocity (no slew decay) so the car keeps
                 * moving to clear and extinguish the beacon. Only coast if the
                 * car was actually moving; a stopped car has nothing to coast. */
                const float coast_speed = sqrtf((g_runtime.cmd_vx_body_state * g_runtime.cmd_vx_body_state) +
                                                (g_runtime.cmd_vy_body_state * g_runtime.cmd_vy_body_state));
                if (coast_speed >= HW_BEACON_COAST_MIN_VEL_MPS)
                {
                    vx_body_cmd = g_runtime.cmd_vx_body_state;
                    vy_body_cmd = g_runtime.cmd_vy_body_state;
                    omega_cmd = chassis_yaw_hold(yaw_deg, gyro_z_radps, transl_speed_mps, dt_s);
                    command_active = 1U;
                    g_status.target_yaw_deg = g_runtime.yaw_lock_deg;
                    g_status.heading_valid = g_runtime.yaw_lock_valid ? 1U : 0U;
                }
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
                         (float)HW_MANUAL_MAX_DUTY;
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
