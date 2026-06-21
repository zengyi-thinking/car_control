#include "hw_imu.h"
#include "hw_icm45686.h"
#include "zf_device_imu660rb.h"
#include "../hw_config.h"

#include <math.h>
#include <string.h>

#include "zf_common_headfile.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define HW_IMU_DT_S                     (((float)HW_CONTROL_PERIOD_US) * 0.000001f)
#define HW_IMU_AHRS_KP_DEFAULT          (0.75f)
#define HW_IMU_AHRS_KI_DEFAULT          (0.004f)
#define HW_IMU_GYRO_WINDOW              (100U)
#define HW_IMU_GYRO_VARIANCE_THRESHOLD  (0.005f)

typedef struct
{
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float x1;
    float x2;
    float y1;
    float y2;
} hw_biquad_t;

static const float g_acc_b0 = 0.046132f;
static const float g_acc_b1 = 0.092264f;
static const float g_acc_b2 = 0.046132f;
static const float g_acc_a1 = -1.307285f;
static const float g_acc_a2 = 0.491813f;

static const float g_gyro_b0 = 0.206572f;
static const float g_gyro_b1 = 0.413144f;
static const float g_gyro_b2 = 0.206572f;
static const float g_gyro_a1 = -0.369527f;
static const float g_gyro_a2 = 0.195816f;

static hw_biquad_t g_acc_filter[3];
static hw_biquad_t g_gyro_filter[3];

static float g_filtered_acc_g[3];
static float g_filtered_gyro_dps[3];
static float g_gyro_bias_dps[3];
static float g_gyro_window[3][HW_IMU_GYRO_WINDOW];
static float g_gyro_sum[3];
static float g_gyro_sum_sq[3];
static unsigned short g_gyro_window_count = 0U;
static unsigned short g_gyro_window_head = 0U;
static unsigned short g_gyro_cal_count = 0U;

static float g_q0 = 1.0f;
static float g_q1 = 0.0f;
static float g_q2 = 0.0f;
static float g_q3 = 0.0f;
static float g_ex_int = 0.0f;
static float g_ey_int = 0.0f;
static float g_ez_int = 0.0f;
static float g_ahrs_kp = HW_IMU_AHRS_KP_DEFAULT;
static float g_ahrs_ki = HW_IMU_AHRS_KI_DEFAULT;

static flight_attitude_t g_local_attitude;
static flight_world_accel_t g_local_world_accel;

static unsigned char g_imu_ready = 0U;

#if (HW_IMU_MODEL != HW_IMU_MODEL_ICM45686) && (HW_IMU_MODEL != HW_IMU_MODEL_IMU660RB)
#error "Unsupported HW_IMU_MODEL value."
#endif

static unsigned char hw_imu_backend_init(void)
{
#if (HW_IMU_MODEL == HW_IMU_MODEL_IMU660RB)
    return (imu660rb_init() == 0U) ? 1U : 0U;
#else
    hw_icm45686_init();
    return (unsigned char)hw_icm45686_ready();
#endif
}

static unsigned char hw_imu_backend_read(float accel_g[3], float gyro_dps[3])
{
    if((accel_g == 0) || (gyro_dps == 0))
    {
        return 0U;
    }

    if(!g_imu_ready)
    {
        return 0U;
    }

#if (HW_IMU_MODEL == HW_IMU_MODEL_IMU660RB)
    imu660rb_get_acc();
    imu660rb_get_gyro();
    accel_g[0] = HW_IMU660RB_ACC_X_SIGN * imu660rb_acc_transition(imu660rb_acc_x);
    accel_g[1] = HW_IMU660RB_ACC_Y_SIGN * imu660rb_acc_transition(imu660rb_acc_y);
    accel_g[2] = HW_IMU660RB_ACC_Z_SIGN * imu660rb_acc_transition(imu660rb_acc_z);
    gyro_dps[0] = HW_IMU660RB_GYRO_X_SIGN * imu660rb_gyro_transition(imu660rb_gyro_x);
    gyro_dps[1] = HW_IMU660RB_GYRO_Y_SIGN * imu660rb_gyro_transition(imu660rb_gyro_y);
    gyro_dps[2] = HW_IMU660RB_GYRO_Z_SIGN * imu660rb_gyro_transition(imu660rb_gyro_z);
    return 1U;
#else
    const flight_imu_raw_t *imu_raw;

    hw_icm45686_update();
    imu_raw = hw_icm45686_get_local_raw();
    if((imu_raw == 0) || (imu_raw->valid == 0U))
    {
        return 0U;
    }

    hw_icm45686_read_float(accel_g, gyro_dps);
    return 1U;
#endif
}

static float hw_biquad_update(hw_biquad_t *filter, float input)
{
    float output = filter->b0 * input
                 + filter->b1 * filter->x1
                 + filter->b2 * filter->x2
                 - filter->a1 * filter->y1
                 - filter->a2 * filter->y2;

    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
}

static void hw_biquad_init(hw_biquad_t *filter, float b0, float b1, float b2, float a1, float a2)
{
    filter->b0 = b0;
    filter->b1 = b1;
    filter->b2 = b2;
    filter->a1 = a1;
    filter->a2 = a2;
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
}

static float hw_imu_inv_sqrt(float value)
{
    if(value <= 0.0f)
    {
        return 0.0f;
    }
    return 1.0f / sqrtf(value);
}

static void hw_imu_reset_outputs(void)
{
    g_local_attitude.quat[0] = 1.0f;
    g_local_attitude.quat[1] = 0.0f;
    g_local_attitude.quat[2] = 0.0f;
    g_local_attitude.quat[3] = 0.0f;
    g_local_attitude.roll = 0.0f;
    g_local_attitude.pitch = 0.0f;
    g_local_attitude.yaw = 0.0f;
    g_local_attitude.gyro_dps[0] = 0.0f;
    g_local_attitude.gyro_dps[1] = 0.0f;
    g_local_attitude.gyro_dps[2] = 0.0f;
    g_local_attitude.valid = 0U;

    g_local_world_accel.north = 0.0f;
    g_local_world_accel.east = 0.0f;
    g_local_world_accel.up = 0.0f;
    g_local_world_accel.valid = 0U;

    g_filtered_acc_g[0] = 0.0f;
    g_filtered_acc_g[1] = 0.0f;
    g_filtered_acc_g[2] = 0.0f;
    g_filtered_gyro_dps[0] = 0.0f;
    g_filtered_gyro_dps[1] = 0.0f;
    g_filtered_gyro_dps[2] = 0.0f;
}

static void hw_imu_reset_bias_state(void)
{
    memset(g_gyro_window, 0, sizeof(g_gyro_window));
    memset(g_gyro_sum, 0, sizeof(g_gyro_sum));
    memset(g_gyro_sum_sq, 0, sizeof(g_gyro_sum_sq));
    g_gyro_window_count = 0U;
    g_gyro_window_head = 0U;
    g_gyro_cal_count = 0U;
    g_gyro_bias_dps[0] = 0.0f;
    g_gyro_bias_dps[1] = 0.0f;
    g_gyro_bias_dps[2] = 0.0f;
    g_ex_int = 0.0f;
    g_ey_int = 0.0f;
    g_ez_int = 0.0f;
}

static void hw_imu_update_bias(const float gyro_dps[3])
{
    unsigned short slot;
    unsigned short axis;

    if(g_gyro_window_count < HW_IMU_GYRO_WINDOW)
    {
        slot = g_gyro_window_count;
        g_gyro_window_count++;
    }
    else
    {
        slot = g_gyro_window_head;
        for(axis = 0U; axis < 3U; axis++)
        {
            g_gyro_sum[axis] -= g_gyro_window[axis][slot];
            g_gyro_sum_sq[axis] -= g_gyro_window[axis][slot] * g_gyro_window[axis][slot];
        }
    }

    for(axis = 0U; axis < 3U; axis++)
    {
        g_gyro_window[axis][slot] = gyro_dps[axis];
        g_gyro_sum[axis] += gyro_dps[axis];
        g_gyro_sum_sq[axis] += gyro_dps[axis] * gyro_dps[axis];
    }

    g_gyro_window_head = (unsigned short)((slot + 1U) % HW_IMU_GYRO_WINDOW);

    if(g_gyro_window_count == HW_IMU_GYRO_WINDOW)
    {
        float mean[3];
        float variance[3];
        unsigned char stable = 1U;
        float sample_count = (float)HW_IMU_GYRO_WINDOW;

        for(axis = 0U; axis < 3U; axis++)
        {
            mean[axis] = g_gyro_sum[axis] / sample_count;
            variance[axis] = (g_gyro_sum_sq[axis] - (g_gyro_sum[axis] * g_gyro_sum[axis] / sample_count)) / sample_count;
            if(variance[axis] >= HW_IMU_GYRO_VARIANCE_THRESHOLD)
            {
                stable = 0U;
            }
        }

        if(stable && (g_gyro_cal_count >= (HW_IMU_GYRO_WINDOW - 1U)))
        {
            g_gyro_bias_dps[0] = mean[0];
            g_gyro_bias_dps[1] = mean[1];
            g_gyro_bias_dps[2] = mean[2];
            g_ex_int = 0.0f;
            g_ey_int = 0.0f;
            g_ez_int = 0.0f;
            g_gyro_cal_count = 0U;
        }
        else if(g_gyro_cal_count < HW_IMU_GYRO_WINDOW)
        {
            g_gyro_cal_count++;
        }
    }
}

static void hw_imu_update_ahrs(float gx,
                               float gy,
                               float gz,
                               float ax,
                               float ay,
                               float az)
{
    float acc_norm;
    float q0q0;
    float q1q1;
    float q2q2;
    float q3q3;
    float vx;
    float vy;
    float vz;
    float ex;
    float ey;
    float ez;
    float q0;
    float q1;
    float q2;
    float q3;
    float half_dt;

    acc_norm = ax * ax + ay * ay + az * az;
    if(acc_norm <= 0.000001f)
    {
        return;
    }
    acc_norm = hw_imu_inv_sqrt(acc_norm);
    if(acc_norm <= 0.0f)
    {
        return;
    }

    ax *= acc_norm;
    ay *= acc_norm;
    az *= acc_norm;

    q0q0 = g_q0 * g_q0;
    q1q1 = g_q1 * g_q1;
    q2q2 = g_q2 * g_q2;
    q3q3 = g_q3 * g_q3;

    vx = 2.0f * (g_q1 * g_q3 - g_q0 * g_q2);
    vy = 2.0f * (g_q0 * g_q1 + g_q2 * g_q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    if((ex != 0.0f) || (ey != 0.0f) || (ez != 0.0f))
    {
        g_ex_int += ex * g_ahrs_ki * (0.5f * HW_IMU_DT_S);
        g_ey_int += ey * g_ahrs_ki * (0.5f * HW_IMU_DT_S);
        g_ez_int += ez * g_ahrs_ki * (0.5f * HW_IMU_DT_S);

        gx += g_ahrs_kp * ex + g_ex_int;
        gy += g_ahrs_kp * ey + g_ey_int;
        gz += g_ahrs_kp * ez + g_ez_int;
    }

    half_dt = 0.5f * HW_IMU_DT_S;
    q0 = g_q0 + (-g_q1 * gx - g_q2 * gy - g_q3 * gz) * half_dt;
    q1 = g_q1 + (g_q0 * gx + g_q2 * gz - g_q3 * gy) * half_dt;
    q2 = g_q2 + (g_q0 * gy - g_q1 * gz + g_q3 * gx) * half_dt;
    q3 = g_q3 + (g_q0 * gz + g_q1 * gy - g_q2 * gx) * half_dt;

    acc_norm = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
    if(acc_norm > 0.0f)
    {
        acc_norm = hw_imu_inv_sqrt(acc_norm);
        if(acc_norm > 0.0f)
        {
            g_q0 = q0 * acc_norm;
            g_q1 = q1 * acc_norm;
            g_q2 = q2 * acc_norm;
            g_q3 = q3 * acc_norm;
        }
    }
}

static void hw_imu_refresh_attitude_and_world_accel(void)
{
    float r11;
    float r12;
    float r13;
    float r21;
    float r22;
    float r23;
    float r31;
    float r32;
    float r33;
    float a_north;
    float a_east;
    float a_down;

    g_local_attitude.quat[0] = g_q0;
    g_local_attitude.quat[1] = g_q1;
    g_local_attitude.quat[2] = g_q2;
    g_local_attitude.quat[3] = g_q3;

    g_local_attitude.yaw = -atan2f(2.0f * g_q1 * g_q2 + 2.0f * g_q0 * g_q3,
                                   -2.0f * g_q2 * g_q2 - 2.0f * g_q3 * g_q3 + 1.0f) * 180.0f / (float)M_PI;
    g_local_attitude.pitch = -asinf(-2.0f * g_q1 * g_q3 + 2.0f * g_q0 * g_q2) * 180.0f / (float)M_PI;
    g_local_attitude.roll = atan2f(2.0f * g_q2 * g_q3 + 2.0f * g_q0 * g_q1,
                                   -2.0f * g_q1 * g_q1 - 2.0f * g_q2 * g_q2 + 1.0f) * 180.0f / (float)M_PI;
    g_local_attitude.gyro_dps[0] = g_filtered_gyro_dps[0] - g_gyro_bias_dps[0];
    g_local_attitude.gyro_dps[1] = g_filtered_gyro_dps[1] - g_gyro_bias_dps[1];
    g_local_attitude.gyro_dps[2] = g_filtered_gyro_dps[2] - g_gyro_bias_dps[2];
    g_local_attitude.valid = 1U;

    r11 = 1.0f - 2.0f * (g_q2 * g_q2 + g_q3 * g_q3);
    r12 = 2.0f * (g_q1 * g_q2 - g_q0 * g_q3);
    r13 = 2.0f * (g_q1 * g_q3 + g_q0 * g_q2);
    r21 = 2.0f * (g_q1 * g_q2 + g_q0 * g_q3);
    r22 = 1.0f - 2.0f * (g_q1 * g_q1 + g_q3 * g_q3);
    r23 = 2.0f * (g_q2 * g_q3 - g_q0 * g_q1);
    r31 = 2.0f * (g_q1 * g_q3 - g_q0 * g_q2);
    r32 = 2.0f * (g_q2 * g_q3 + g_q0 * g_q1);
    r33 = 1.0f - 2.0f * (g_q1 * g_q1 + g_q2 * g_q2);

    a_north = r11 * g_filtered_acc_g[0] + r12 * g_filtered_acc_g[1] + r13 * g_filtered_acc_g[2];
    a_east  = r21 * g_filtered_acc_g[0] + r22 * g_filtered_acc_g[1] + r23 * g_filtered_acc_g[2];
    a_down  = r31 * g_filtered_acc_g[0] + r32 * g_filtered_acc_g[1] + r33 * g_filtered_acc_g[2];

    g_local_world_accel.north = a_north;
    g_local_world_accel.east = a_east;
    /* Unconditional up update — matches reference imu.c Get_World_Accel():
     * w_acc.up = (a_Down - 1.0f) * 9780; always executed. */
    g_local_world_accel.up = (a_down - 1.0f) * 9780.0f;
    g_local_world_accel.valid = 1U;
}

void hw_imu_init(void)
{
    unsigned short axis;

    for(axis = 0U; axis < 3U; axis++)
    {
        hw_biquad_init(&g_acc_filter[axis], g_acc_b0, g_acc_b1, g_acc_b2, g_acc_a1, g_acc_a2);
        hw_biquad_init(&g_gyro_filter[axis], g_gyro_b0, g_gyro_b1, g_gyro_b2, g_gyro_a1, g_gyro_a2);
    }

    hw_imu_reset_outputs();
    hw_imu_reset_bias_state();
    g_ahrs_kp = HW_IMU_AHRS_KP_DEFAULT;
    g_ahrs_ki = HW_IMU_AHRS_KI_DEFAULT;
    g_imu_ready = hw_imu_backend_init();
}

void hw_imu_set_ahrs_gains(float kp, float ki)
{
    __disable_irq();
    g_ahrs_kp = kp;
    g_ahrs_ki = ki;
    __enable_irq();
}

void hw_imu_update(void)
{
    float accel_raw[3];
    float gyro_raw[3];
    float gyro_corr[3];
    unsigned short axis;

    if(!hw_imu_backend_read(accel_raw, gyro_raw))
    {
        g_local_attitude.valid = 0U;
        g_local_world_accel.valid = 0U;
        return;
    }

    for(axis = 0U; axis < 3U; axis++)
    {
        g_filtered_acc_g[axis] = hw_biquad_update(&g_acc_filter[axis], accel_raw[axis]);
        g_filtered_gyro_dps[axis] = hw_biquad_update(&g_gyro_filter[axis], gyro_raw[axis]);
    }

    hw_imu_update_bias(g_filtered_gyro_dps);

    gyro_corr[0] = (g_filtered_gyro_dps[0] - g_gyro_bias_dps[0]) * ((float)M_PI / 180.0f);
    gyro_corr[1] = (g_filtered_gyro_dps[1] - g_gyro_bias_dps[1]) * ((float)M_PI / 180.0f);
    gyro_corr[2] = (g_filtered_gyro_dps[2] - g_gyro_bias_dps[2]) * ((float)M_PI / 180.0f);
    hw_imu_update_ahrs(gyro_corr[0],
                       gyro_corr[1],
                       gyro_corr[2],
                       g_filtered_acc_g[0],
                       g_filtered_acc_g[1],
                       g_filtered_acc_g[2]);

    hw_imu_refresh_attitude_and_world_accel();
}

void hw_imu_irq_callback(void)
{
    /* Both backends are polled in hw_imu_update() right now. */
}

void hw_imu_read_attitude(flight_attitude_t *attitude)
{
    if(attitude)
    {
        __disable_irq();
        *attitude = g_local_attitude;
        __enable_irq();
    }
}

void hw_imu_read_world_accel(flight_world_accel_t *world_accel)
{
    if(world_accel)
    {
        __disable_irq();
        *world_accel = g_local_world_accel;
        __enable_irq();
    }
}

void hw_imu_read_float(float accel_g[3], float gyro_dps[3])
{
    __disable_irq();
    if(accel_g)
    {
        accel_g[0] = g_filtered_acc_g[0];
        accel_g[1] = g_filtered_acc_g[1];
        accel_g[2] = g_filtered_acc_g[2];
    }

    if(gyro_dps)
    {
        gyro_dps[0] = g_filtered_gyro_dps[0] - g_gyro_bias_dps[0];
        gyro_dps[1] = g_filtered_gyro_dps[1] - g_gyro_bias_dps[1];
        gyro_dps[2] = g_filtered_gyro_dps[2] - g_gyro_bias_dps[2];
    }
    __enable_irq();
}
