/*******************************************************************************
 * 麦轮底盘控制
 *
 * 控制模式:
 * 1. SAFE_STOP: 电机停转
 * 2. MANUAL: LoRa 遥控摇杆直控 + IMU 锁头
 * 3. NAV: 0xA2 / 0xA1 导航输入 + 编码器里程计 + IMU 航向保持
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

#define CHASSIS_POSE_BLEND_ALPHA            0.35f
#define CHASSIS_POSE_CORR_MAX_STEP_M        0.20f
#define CHASSIS_POSE_JUMP_REJECT_M          1.20f

#define CHASSIS_POS_KP                      1.8f
#define CHASSIS_VEL_DAMP_K                  0.55f
#define CHASSIS_POS_DEADBAND_M              0.03f
#define CHASSIS_ARRIVE_VEL_MPS              0.08f

#define CHASSIS_HEADING_KP                  3.2f
#define CHASSIS_HEADING_KI                  0.75f
#define CHASSIS_HEADING_KD                  0.12f
#define CHASSIS_HEADING_I_LIMIT             0.40f

#define CHASSIS_REMOTE_AXIS_MAX             1600.0f
#define CHASSIS_REMOTE_DEADBAND             100.0f
#define CHASSIS_REMOTE_YAW_HOLD_THRESHOLD   0.08f

#define CHASSIS_MANUAL_AXIS_SNAP_MIN        0.18f
#define CHASSIS_MANUAL_AXIS_SNAP_RATIO      0.35f

#define CHASSIS_MANUAL_HEADING_KP           2.6f
#define CHASSIS_MANUAL_HEADING_KI           0.22f
#define CHASSIS_MANUAL_HEADING_KD           0.10f
#define CHASSIS_MANUAL_HEADING_ERR_DEG      1.5f
#define CHASSIS_MANUAL_HEADING_I_LIMIT      0.18f
#define CHASSIS_MANUAL_HEADING_MAX_RADPS    0.35f

#define CHASSIS_WHEEL_PID_OUT_MAX           3000.0f
#define CHASSIS_WHEEL_PID_I_MAX             800.0f

static const float s_wheel_pid_kp[4] =
{
    650.0f,   /* LF */
    720.0f,   /* RF: 右前起步稍强 */
    650.0f,   /* LB */
    720.0f,   /* RB: 右后起步稍强 */
};

static const float s_wheel_pid_ki[4] =
{
    160.0f,   /* LF */
    180.0f,   /* RF */
    160.0f,   /* LB */
    180.0f,   /* RB */
};

static const float s_wheel_pid_kd[4] =
{
    0.0f,
    0.0f,
    0.0f,
    0.0f,
};

#define CHASSIS_TUNE_AXIS_ON                0.55f
#define CHASSIS_TUNE_AXIS_OFF               0.25f

volatile uint8 g_chassis_wheel_pid_tune_enabled = 0U;
volatile uint8 g_chassis_wheel_pid_tune_wheel = 0U;
volatile uint8 g_chassis_wheel_pid_tune_param = 0U;
volatile float g_chassis_wheel_pid_tune_target_mps = HW_WHEEL_TUNE_TARGET_MPS;

static const float s_wheel_ff_ks_duty[4] =
{
    260.0f,   /* LF */
    400.0f,   /* RF: 右侧补一点静摩擦 */
    250.0f,   /* LB */
    420.0f,   /* RB */
};

typedef struct
{
    proto_remote_data_t remote;
    proto_nav_data_t nav;
    odometry_state_t odom;
    chassis_mode_t last_mode;
    uint8 yaw_lock_valid;
    float yaw_lock_deg;
    float heading_integral;
    float last_yaw_deg;
    float cmd_vx_field_state;
    float cmd_vy_field_state;
    uint8 tune_wheel_axis_armed;
    uint8 tune_param_axis_armed;
    uint8 tune_value_axis_armed;
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

    if (fabsf(target_speed_mps) < 0.001f)
    {
        return 0.0f;
    }

    abs_speed = fabsf(target_speed_mps);
    duty = s_wheel_ff_ks_duty[wheel] + (HW_MOTOR_FF_KV_DUTY_PER_MPS * abs_speed);

    return (target_speed_mps > 0.0f) ? duty : -duty;
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
    g_runtime.yaw_lock_deg = yaw_deg;
    g_runtime.yaw_lock_valid = 1U;
    g_runtime.heading_integral = 0.0f;
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

static float chassis_manual_heading_hold(float yaw_deg, float gyro_z_radps, float dt_s)
{
    float yaw_err_deg;
    float yaw_err_rad;
    float integral_candidate;
    float omega_cmd;

    if (!g_runtime.yaw_lock_valid || (dt_s <= 0.0f))
    {
        return 0.0f;
    }

    yaw_err_deg = chassis_wrap_deg(g_runtime.yaw_lock_deg - yaw_deg);
    if (fabsf(yaw_err_deg) < CHASSIS_MANUAL_HEADING_ERR_DEG)
    {
        yaw_err_deg = 0.0f;
    }

    yaw_err_rad = yaw_err_deg * CHASSIS_DEG_TO_RAD;
    integral_candidate = g_runtime.heading_integral + yaw_err_rad * dt_s;
    g_runtime.heading_integral = chassis_clamp(integral_candidate,
                                               -CHASSIS_MANUAL_HEADING_I_LIMIT,
                                               CHASSIS_MANUAL_HEADING_I_LIMIT);

    omega_cmd = CHASSIS_MANUAL_HEADING_KP * yaw_err_rad
              + CHASSIS_MANUAL_HEADING_KI * g_runtime.heading_integral
              - CHASSIS_MANUAL_HEADING_KD * gyro_z_radps;
    return chassis_clamp(omega_cmd, -CHASSIS_MANUAL_HEADING_MAX_RADPS, CHASSIS_MANUAL_HEADING_MAX_RADPS);
}

static void chassis_apply_pose_measurement(const proto_nav_data_t *nav)
{
    float meas_x_m;
    float meas_y_m;
    float dx;
    float dy;
    float innovation_norm;
    float corr_x;
    float corr_y;
    float corr_norm;
    float corr_scale;

    if ((nav == 0) || ((nav->flags & PROTO_NAV_FLAG_CURRENT_VALID) == 0U))
    {
        return;
    }

    meas_x_m = 0.001f * (float)nav->current_x_mm;
    meas_y_m = 0.001f * (float)nav->current_y_mm;
    g_status.pose_meas_x_m = meas_x_m;
    g_status.pose_meas_y_m = meas_y_m;

    if (!g_status.pose_valid)
    {
        g_status.pose_x_m = meas_x_m;
        g_status.pose_y_m = meas_y_m;
        g_status.pose_valid = 1U;
        return;
    }

    dx = meas_x_m - g_status.pose_x_m;
    dy = meas_y_m - g_status.pose_y_m;
    innovation_norm = sqrtf(dx * dx + dy * dy);

    if (innovation_norm > CHASSIS_POSE_JUMP_REJECT_M)
    {
        if (!g_status.nav_fresh)
        {
            g_status.pose_x_m = meas_x_m;
            g_status.pose_y_m = meas_y_m;
        }
        return;
    }

    corr_x = CHASSIS_POSE_BLEND_ALPHA * dx;
    corr_y = CHASSIS_POSE_BLEND_ALPHA * dy;
    corr_norm = sqrtf(corr_x * corr_x + corr_y * corr_y);

    if (corr_norm > CHASSIS_POSE_CORR_MAX_STEP_M)
    {
        corr_scale = CHASSIS_POSE_CORR_MAX_STEP_M / corr_norm;
        corr_x *= corr_scale;
        corr_y *= corr_scale;
    }

    g_status.pose_x_m += corr_x;
    g_status.pose_y_m += corr_y;
}

static void chassis_update_target(const proto_nav_data_t *nav, float current_yaw_deg)
{
    if (nav == 0)
    {
        return;
    }

    if ((nav->flags & PROTO_NAV_FLAG_TARGET_VALID) != 0U)
    {
        g_status.target_x_m = 0.001f * (float)nav->target_x_mm;
        g_status.target_y_m = 0.001f * (float)nav->target_y_mm;
        g_status.target_valid = 1U;
    }

    if ((nav->flags & PROTO_NAV_FLAG_HEADING_VALID) != 0U)
    {
        g_runtime.yaw_lock_deg = 0.1f * (float)nav->heading_deg_x10;
        g_runtime.yaw_lock_valid = 1U;
        g_runtime.heading_integral = 0.0f;
        g_status.target_yaw_deg = g_runtime.yaw_lock_deg;
        g_status.heading_valid = 1U;
    }
    else if (!g_runtime.yaw_lock_valid)
    {
        g_runtime.yaw_lock_deg = current_yaw_deg;
        g_runtime.yaw_lock_valid = 1U;
        g_status.target_yaw_deg = g_runtime.yaw_lock_deg;
        g_status.heading_valid = 1U;
    }
}

static float chassis_heading_hold(float yaw_deg, float gyro_z_radps, float dt_s)
{
    float yaw_err_deg;
    float yaw_err_rad;
    float integral_candidate;
    float unsat_output;
    float omega_cmd;

    if (!g_runtime.yaw_lock_valid || (dt_s <= 0.0f))
    {
        return 0.0f;
    }

    yaw_err_deg = chassis_wrap_deg(g_runtime.yaw_lock_deg - yaw_deg);
    yaw_err_rad = yaw_err_deg * CHASSIS_DEG_TO_RAD;

    integral_candidate = g_runtime.heading_integral + (yaw_err_rad * CHASSIS_HEADING_KI * dt_s);
    integral_candidate = chassis_clamp(integral_candidate, -CHASSIS_HEADING_I_LIMIT, CHASSIS_HEADING_I_LIMIT);

    unsat_output = CHASSIS_HEADING_KP * yaw_err_rad + integral_candidate - CHASSIS_HEADING_KD * gyro_z_radps;
    if ((fabsf(unsat_output) <= HW_MAX_YAW_RATE_RADPS) ||
        ((unsat_output > HW_MAX_YAW_RATE_RADPS) && (yaw_err_rad < 0.0f)) ||
        ((unsat_output < -HW_MAX_YAW_RATE_RADPS) && (yaw_err_rad > 0.0f)))
    {
        g_runtime.heading_integral = integral_candidate;
    }

    omega_cmd = CHASSIS_HEADING_KP * yaw_err_rad + g_runtime.heading_integral - CHASSIS_HEADING_KD * gyro_z_radps;
    return chassis_clamp(omega_cmd, -HW_MAX_YAW_RATE_RADPS, HW_MAX_YAW_RATE_RADPS);
}

void chassis_control_init(void)
{
    memset(&g_runtime, 0, sizeof(g_runtime));
    memset(&g_status, 0, sizeof(g_status));
    g_runtime.last_mode = CHASSIS_MODE_SAFE_STOP;
    g_status.mode = CHASSIS_MODE_SAFE_STOP;
    g_chassis_wheel_pid_tune_target_mps = HW_WHEEL_TUNE_TARGET_MPS;
    g_chassis_wheel_pid_tune_wheel = 0U;
    g_chassis_wheel_pid_tune_param = 0U;
    g_chassis_wheel_pid_tune_enabled = 0U;

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
    g_runtime.heading_integral = 0.0f;
    g_runtime.last_yaw_deg = 0.0f;
    g_runtime.cmd_vx_field_state = 0.0f;
    g_runtime.cmd_vy_field_state = 0.0f;
    chassis_tune_reset_latch();

    memset(&g_status, 0, sizeof(g_status));
    g_status.mode = CHASSIS_MODE_SAFE_STOP;
    g_runtime.last_mode = CHASSIS_MODE_SAFE_STOP;
    chassis_reset_pid_and_output();
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

void chassis_control_set_nav(const proto_nav_data_t *nav, uint8 fresh, uint16 age_ms)
{
    if (nav != 0)
    {
        g_runtime.nav = *nav;
        g_status.using_legacy_mode = (nav->mode == PROTO_NAV_MODE_LEGACY_ERROR) ? 1U : 0U;
    }
    else
    {
        g_status.using_legacy_mode = 0U;
    }

    g_status.nav_fresh = fresh ? 1U : 0U;
    g_status.nav_age_ms = age_ms;
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
    uint8 command_active = 0U;

    if ((enc_counts == 0) || (dt_s <= 0.0f))
    {
        return;
    }

    if ((attitude != 0) && (attitude->valid != 0U))
    {
        yaw_deg = attitude->yaw;
        gyro_z_radps = attitude->gyro_dps[2] * CHASSIS_DEG_TO_RAD;
    }
    else
    {
        yaw_deg = chassis_wrap_deg(yaw_deg + g_runtime.odom.omega_radps * dt_s * CHASSIS_RAD_TO_DEG);
        gyro_z_radps = g_runtime.odom.omega_radps;
    }
    g_runtime.last_yaw_deg = yaw_deg;

    odometry_update(&g_runtime.odom, enc_counts, dt_s, yaw_deg);
    g_status.odom_x_m = g_runtime.odom.x_m;
    g_status.odom_y_m = g_runtime.odom.y_m;
    g_status.pose_yaw_deg = yaw_deg;
    g_status.vel_x_field_mps = g_runtime.odom.vx_field_mps;
    g_status.vel_y_field_mps = g_runtime.odom.vy_field_mps;

    for (uint8 i = 0; i < 4U; ++i)
    {
        g_status.wheel_measured_mps[i] = g_runtime.odom.wheel_speed_mps[i];
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

    if (g_status.nav_fresh)
    {
        chassis_apply_pose_measurement(&g_runtime.nav);
        chassis_update_target(&g_runtime.nav, yaw_deg);
    }

    if (g_status.mode != g_runtime.last_mode)
    {
        if (g_status.mode == CHASSIS_MODE_SAFE_STOP)
        {
            g_runtime.cmd_vx_field_state = 0.0f;
            g_runtime.cmd_vy_field_state = 0.0f;
            chassis_reset_pid_and_output();
        }
        else if (g_status.mode == CHASSIS_MODE_MANUAL)
        {
            chassis_reset_heading_controller(yaw_deg);
        }
        else if (g_status.mode == CHASSIS_MODE_WHEEL_TUNE)
        {
            chassis_reset_heading_controller(yaw_deg);
            chassis_tune_reset_latch();
        }
        else if ((g_status.mode == CHASSIS_MODE_NAV) && !g_runtime.yaw_lock_valid)
        {
            chassis_reset_heading_controller(yaw_deg);
        }

        g_runtime.last_mode = g_status.mode;
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

    switch (g_status.mode)
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
                float joy_x = chassis_normalize_remote_axis(g_runtime.remote.joystick[0]);
                float joy_y = chassis_normalize_remote_axis(g_runtime.remote.joystick[1]);
                float joy_yaw = chassis_normalize_remote_axis(g_runtime.remote.joystick[2]);
                uint8 translation_active;
                uint8 yaw_active;

                chassis_manual_snap_axes(&joy_x, &joy_y);
                translation_active = (fabsf(joy_x) > 0.001f) || (fabsf(joy_y) > 0.001f);
                yaw_active = (fabsf(joy_yaw) > CHASSIS_REMOTE_YAW_HOLD_THRESHOLD);

                g_status.joy_x_norm = joy_x;
                g_status.joy_y_norm = joy_y;
                g_status.user_cmd_x_mps = joy_x * HW_MANUAL_MAX_LINEAR_SPEED_MPS;
                g_status.user_cmd_y_mps = joy_y * HW_MANUAL_MAX_LINEAR_SPEED_MPS;
                vx_body_cmd = g_status.user_cmd_y_mps;
                vy_body_cmd = g_status.user_cmd_x_mps;

                if (translation_active || yaw_active)
                {
                    command_active = 1U;

                    if (yaw_active)
                    {
                        // Right stick X actively commands yaw; refresh lock target so
                        // heading hold resumes from the current heading after release.
                        chassis_reset_heading_controller(yaw_deg);
                        omega_cmd = joy_yaw * HW_MANUAL_MAX_YAW_RATE_RADPS;
                    }
                    else
                    {
                        omega_cmd = chassis_manual_heading_hold(yaw_deg, gyro_z_radps, dt_s);
                    }
                }
                else
                {
                    vx_body_cmd = 0.0f;
                    vy_body_cmd = 0.0f;
                    omega_cmd = 0.0f;
                    chassis_reset_heading_controller(yaw_deg);
                    chassis_reset_pid_and_output();
                }

                g_status.target_yaw_deg = g_runtime.yaw_lock_deg;
                g_status.heading_valid = g_runtime.yaw_lock_valid ? 1U : 0U;
            }
        } break;

        case CHASSIS_MODE_NAV:
        {
            if (g_status.nav_fresh && (g_runtime.nav.mode == PROTO_NAV_MODE_TARGET) &&
                g_status.pose_valid && g_status.target_valid)
            {
                const float err_x = g_status.target_x_m - g_status.pose_x_m;
                const float err_y = g_status.target_y_m - g_status.pose_y_m;
                const float distance_m = sqrtf(err_x * err_x + err_y * err_y);
                const float braking_speed_limit = sqrtf(2.0f * HW_MAX_LINEAR_ACCEL_MPS2 * distance_m);
                const float max_step = HW_MAX_LINEAR_ACCEL_MPS2 * dt_s;
                const float yaw_rad = yaw_deg * CHASSIS_DEG_TO_RAD;

                vx_field_cmd = CHASSIS_POS_KP * err_x - CHASSIS_VEL_DAMP_K * g_runtime.odom.vx_field_mps;
                vy_field_cmd = CHASSIS_POS_KP * err_y - CHASSIS_VEL_DAMP_K * g_runtime.odom.vy_field_mps;

                if (fabsf(err_x) < CHASSIS_POS_DEADBAND_M)
                {
                    vx_field_cmd = 0.0f;
                }
                if (fabsf(err_y) < CHASSIS_POS_DEADBAND_M)
                {
                    vy_field_cmd = 0.0f;
                }

                chassis_limit_vector(&vx_field_cmd, &vy_field_cmd, HW_MAX_LINEAR_SPEED_MPS);
                if (braking_speed_limit < HW_MAX_LINEAR_SPEED_MPS)
                {
                    chassis_limit_vector(&vx_field_cmd, &vy_field_cmd, braking_speed_limit);
                }

                if ((distance_m < CHASSIS_POS_DEADBAND_M) &&
                    (fabsf(g_runtime.odom.vx_field_mps) < CHASSIS_ARRIVE_VEL_MPS) &&
                    (fabsf(g_runtime.odom.vy_field_mps) < CHASSIS_ARRIVE_VEL_MPS))
                {
                    vx_field_cmd = 0.0f;
                    vy_field_cmd = 0.0f;
                }

                g_runtime.cmd_vx_field_state = chassis_move_toward(g_runtime.cmd_vx_field_state, vx_field_cmd, max_step);
                g_runtime.cmd_vy_field_state = chassis_move_toward(g_runtime.cmd_vy_field_state, vy_field_cmd, max_step);

                vx_field_cmd = g_runtime.cmd_vx_field_state;
                vy_field_cmd = g_runtime.cmd_vy_field_state;

                vx_body_cmd = cosf(yaw_rad) * vx_field_cmd + sinf(yaw_rad) * vy_field_cmd;
                vy_body_cmd = -sinf(yaw_rad) * vx_field_cmd + cosf(yaw_rad) * vy_field_cmd;
                omega_cmd = chassis_heading_hold(yaw_deg, gyro_z_radps, dt_s);
                command_active = (fabsf(vx_body_cmd) > 0.001f) || (fabsf(vy_body_cmd) > 0.001f) || (fabsf(omega_cmd) > 0.001f);

                g_status.pos_error_x_m = err_x;
                g_status.pos_error_y_m = err_y;
                g_status.distance_to_target_m = distance_m;
                g_status.target_yaw_deg = g_runtime.yaw_lock_deg;
                g_status.heading_valid = g_runtime.yaw_lock_valid ? 1U : 0U;
            }
            else if (g_status.nav_fresh && (g_runtime.nav.mode == PROTO_NAV_MODE_LEGACY_ERROR))
            {
                vx_body_cmd = chassis_clamp(0.001f * (float)g_runtime.nav.err_fwd, -0.8f, 0.8f);
                vy_body_cmd = chassis_clamp(-0.001f * (float)g_runtime.nav.err_lat, -0.8f, 0.8f);
                command_active = (fabsf(vx_body_cmd) > 0.001f) || (fabsf(vy_body_cmd) > 0.001f);
                g_status.pos_error_x_m = 0.0f;
                g_status.pos_error_y_m = 0.0f;
                g_status.distance_to_target_m = 0.0f;
            }
            else
            {
                g_runtime.cmd_vx_field_state = 0.0f;
                g_runtime.cmd_vy_field_state = 0.0f;
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
        } break;
    }

    if (command_active)
    {
        mecanum_inverse(vx_body_cmd, vy_body_cmd, omega_cmd, wheel_target_mps);
        mecanum_desaturate(wheel_target_mps, HW_MAX_WHEEL_SPEED_MPS);

        for (uint8 i = 0; i < 4U; ++i)
        {
            float duty_limit;
            float duty_raw;
            float duty_ff;

            g_status.wheel_target_mps[i] = wheel_target_mps[i];

            duty_limit = (g_status.mode == CHASSIS_MODE_NAV) ? (float)HW_MOTOR_DUTY_MAX : (float)HW_MANUAL_MAX_DUTY;
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
    g_status.enabled = ((g_status.mode == CHASSIS_MODE_MANUAL) &&
                        g_status.remote_fresh &&
                        command_active) ||
                       ((g_status.mode == CHASSIS_MODE_WHEEL_TUNE) &&
                        command_active) ||
                       ((g_status.mode == CHASSIS_MODE_NAV) &&
                        g_status.nav_fresh &&
                        ((g_status.target_valid && g_status.pose_valid) || g_status.using_legacy_mode) &&
                        command_active);
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
