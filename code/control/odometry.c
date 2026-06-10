#include "control/odometry.h"

#include "control/mecanum.h"
#include "hardware/hw_config.h"
#include "hardware/hw_encoder.h"

#include <math.h>
#include <string.h>

#define ODOMETRY_PI                        3.14159265358979323846f
#define ODOMETRY_DEG_TO_RAD                (ODOMETRY_PI / 180.0f)

void odometry_reset(odometry_state_t *state)
{
    if (state == 0)
    {
        return;
    }

    memset(state, 0, sizeof(*state));
}

void odometry_update(odometry_state_t *state, const int16 enc_counts[4], float dt_s, float yaw_deg)
{
    float meters_per_count;
    float dx_body_m = 0.0f;
    float dy_body_m = 0.0f;
    float yaw_rad;

    if ((state == 0) || (enc_counts == 0) || (dt_s <= 0.0f))
    {
        return;
    }

    state->yaw_deg = yaw_deg;
    state->delta_x_field_m = 0.0f;
    state->delta_y_field_m = 0.0f;

    hw_encoder_counts_to_speed_mps(enc_counts, dt_s, state->wheel_speed_mps);
    mecanum_forward(state->wheel_speed_mps, &state->vx_body_mps, &state->vy_body_mps, &state->omega_radps);

    meters_per_count = (2.0f * ODOMETRY_PI * HW_WHEEL_RADIUS_M) / HW_ENCODER_COUNTS_PER_WHEEL_REV;
    for (uint8 i = 0; i < 4U; ++i)
    {
        state->wheel_delta_m[i] = (float)enc_counts[i] * meters_per_count;
    }

    mecanum_forward(state->wheel_delta_m, &dx_body_m, &dy_body_m, 0);

    yaw_rad = yaw_deg * ODOMETRY_DEG_TO_RAD;
    state->delta_x_field_m = cosf(yaw_rad) * dx_body_m - sinf(yaw_rad) * dy_body_m;
    state->delta_y_field_m = sinf(yaw_rad) * dx_body_m + cosf(yaw_rad) * dy_body_m;

    state->x_m += state->delta_x_field_m;
    state->y_m += state->delta_y_field_m;

    state->vx_field_mps = cosf(yaw_rad) * state->vx_body_mps - sinf(yaw_rad) * state->vy_body_mps;
    state->vy_field_mps = sinf(yaw_rad) * state->vx_body_mps + cosf(yaw_rad) * state->vy_body_mps;
    state->valid = 1U;
}
