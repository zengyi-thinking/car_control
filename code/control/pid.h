#ifndef _pid_h_
#define _pid_h_

#include "zf_common_typedef.h"

typedef struct
{
    float   kp;
    float   ki;
    float   kd;
    float   integral;
    float   integral_min;
    float   integral_max;
    float   prev_error;
    float   prev_measurement;
    float   output;
    float   output_min;
    float   output_max;
    uint8   initialized;
} pid_t;

void    pid_init(pid_t *pid, float kp, float ki, float kd, float out_max, float int_max);
float   pid_compute(pid_t *pid, float setpoint, float measurement, float dt_s);
void    pid_reset(pid_t *pid);

#endif
