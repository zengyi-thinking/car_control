#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "zf_common_typedef.h"

typedef struct
{
    float x_m;
    float y_m;
    float yaw_deg;

    float delta_x_field_m;
    float delta_y_field_m;

    float vx_body_mps;  /* 历史字段名：物理含义为前向速度，小车 +Y。 */
    float vy_body_mps;  /* 历史字段名：物理含义为右向速度，小车 +X。 */
    float omega_radps;
    float vx_field_mps;
    float vy_field_mps;

    float wheel_speed_mps[4];
    float wheel_delta_m[4];

    uint8 valid;
} odometry_state_t;

void odometry_reset(odometry_state_t *state);
void odometry_update(odometry_state_t *state, const int16 enc_counts[4], float dt_s, float yaw_deg);

#endif
