/*******************************************************************************
 * 通用 PID 控制器
 *
 * 采用位置式 PID，方便给轮速环叠加前馈。
 * 输出和积分都做对称限幅，避免高速段积分堆积。
 ******************************************************************************/

#include "control/pid.h"
#include "string.h"

void pid_init(pid_t *pid, float kp, float ki, float kd, float out_max, float int_max)
{
    memset(pid, 0, sizeof(pid_t));
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_min = -out_max;
    pid->output_max = out_max;
    pid->integral_min = -int_max;
    pid->integral_max = int_max;
    pid->initialized = 0;
}

float pid_compute(pid_t *pid, float setpoint, float measurement, float dt_s)
{
    float error = setpoint - measurement;
    float derivative = 0.0f;
    float output;

    if ((pid == 0) || (dt_s <= 0.0f))
    {
        return 0.0f;
    }

    if (!pid->initialized)
    {
        pid->initialized = 1;
        pid->prev_error = error;
        pid->prev_measurement = measurement;
    }

    pid->integral += error * dt_s;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < pid->integral_min) pid->integral = pid->integral_min;

    derivative = (measurement - pid->prev_measurement) / dt_s;

    output = pid->kp * error
           + pid->ki * pid->integral
           - pid->kd * derivative;

    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    pid->prev_error = error;
    pid->prev_measurement = measurement;
    pid->output = output;

    return output;
}

void pid_reset(pid_t *pid)
{
    pid->integral = 0;
    pid->prev_error = 0;
    pid->prev_measurement = 0;
    pid->output = 0;
    pid->initialized = 0;
}
