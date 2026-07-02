#include "app/app_car.h"

#include "hardware/hw_config.h"
#include "hardware/hw_display.h"
#include "hardware/hw_encoder.h"
#include "hardware/hw_indicator.h"
#include "hardware/hw_motor_drv8701e.h"
#include "hardware/sensors/hw_imu.h"
#include "protocol/proto_drone.h"
#include "protocol/proto_remote.h"

#include <string.h>

extern volatile uint8 g_chassis_wheel_pid_tune_enabled;

typedef struct
{
    proto_remote_data_t remote;
    proto_drone_data_t drone;
    uint8 has_remote;
    uint8 has_drone;
    uint16 remote_age_ms;
    uint16 drone_age_ms;
    chassis_mode_t mode;
} app_car_runtime_t;

static app_car_runtime_t g_app_runtime;

static uint8 app_remote_fresh(void)
{
    return g_app_runtime.has_remote && (g_app_runtime.remote_age_ms <= HW_REMOTE_TIMEOUT_MS);
}

static uint8 app_drone_fresh(void)
{
    return g_app_runtime.has_drone && (g_app_runtime.drone_age_ms <= HW_DRONE_TIMEOUT_MS);
}

static uint8 app_drone_move_valid(void)
{
    return app_drone_fresh() &&
           ((g_app_runtime.drone.flags & PROTO_DRONE_FLAG_MOVE_VALID) != 0U);
}

static uint8 app_drone_strike_hold_valid(void)
{
    return app_drone_fresh() &&
           ((g_app_runtime.drone.flags & PROTO_DRONE_FLAG_MOVE_VALID) == 0U) &&
           ((g_app_runtime.drone.flags & PROTO_DRONE_FLAG_HAS_VLAMP) != 0U) &&
           (chassis_control_drone_strike_hold_ready() != 0U);
}

static uint16 app_abs_i16(int16 value)
{
    return (value < 0) ? (uint16)(-value) : (uint16)value;
}

static uint8 app_remote_manual_override(void)
{
    if(!app_remote_fresh())
    {
        return 0U;
    }

    return ((app_abs_i16(g_app_runtime.remote.joystick[0]) >= (uint16)HW_DRONE_REMOTE_OVERRIDE_RAW) ||
            (app_abs_i16(g_app_runtime.remote.joystick[1]) >= (uint16)HW_DRONE_REMOTE_OVERRIDE_RAW)) ? 1U : 0U;
}

static void app_update_mode(void)
{
    const uint8 remote_override = app_remote_manual_override();
    const uint8 drone_move_valid = app_drone_move_valid();
    const uint8 drone_hold_valid = app_drone_strike_hold_valid();

    if (g_chassis_wheel_pid_tune_enabled)
    {
        g_app_runtime.mode = CHASSIS_MODE_WHEEL_TUNE;
        return;
    }

    if (remote_override)
    {
        g_app_runtime.mode = CHASSIS_MODE_MANUAL;
    }
    else if ((drone_move_valid != 0U) || (drone_hold_valid != 0U))
    {
        g_app_runtime.mode = CHASSIS_MODE_DRONE_TRACK;
    }
    else
    {
        g_app_runtime.mode = CHASSIS_MODE_SAFE_STOP;
    }
}

void app_car_init(void)
{
    memset(&g_app_runtime, 0, sizeof(g_app_runtime));

    hw_encoder_init();
    hw_motor_drv8701e_init();
    hw_indicator_init();
    hw_display_init();
    hw_imu_init();
    proto_drone_init();
    proto_remote_init();
    chassis_control_init();
    chassis_control_set_mode(CHASSIS_MODE_SAFE_STOP);

    hw_indicator_off();
    hw_motor_drv8701e_stop_all();
}

void app_car_step(void)
{
    int16 enc_counts[4] = {0};
    int16 motor_duty[4] = {0};
    flight_attitude_t attitude;
    const chassis_status_t *status;

    memset(&attitude, 0, sizeof(attitude));

    hw_imu_update();
    hw_imu_read_attitude(&attitude);
    hw_encoder_read_counts(enc_counts);

    if (proto_remote_update(&g_app_runtime.remote))
    {
        g_app_runtime.has_remote = 1U;
        g_app_runtime.remote_age_ms = 0U;
    }
    else if (g_app_runtime.remote_age_ms < 0xFFFFU)
    {
        g_app_runtime.remote_age_ms = (uint16)(g_app_runtime.remote_age_ms + HW_CONTROL_PERIOD_MS);
    }

    if (proto_drone_update(&g_app_runtime.drone))
    {
        g_app_runtime.has_drone = 1U;
        g_app_runtime.drone_age_ms = 0U;
    }
    else if (g_app_runtime.drone_age_ms < 0xFFFFU)
    {
        g_app_runtime.drone_age_ms = (uint16)(g_app_runtime.drone_age_ms + HW_CONTROL_PERIOD_MS);
    }

    app_update_mode();

    chassis_control_set_mode(g_app_runtime.mode);
    chassis_control_set_remote(&g_app_runtime.remote, app_remote_fresh(), g_app_runtime.remote_age_ms);
    chassis_control_set_drone(&g_app_runtime.drone, app_drone_fresh(), g_app_runtime.drone_age_ms);
    chassis_control_step(enc_counts, HW_CONTROL_PERIOD_S, &attitude);

    status = chassis_control_get_status();
    if (status->enabled)
    {
        chassis_control_get_motor_duty(motor_duty);
        hw_motor_drv8701e_set_all(motor_duty);
    }
    else
    {
        hw_motor_drv8701e_stop_all();
    }

    if (status->remote_fresh || status->drone_fresh || status->tune_active || (status->position_move_state == CHASSIS_POSITION_MOVE_RUN))
    {
        hw_indicator_on();
    }
    else
    {
        hw_indicator_off();
    }

    hw_display_update(status);
}

const app_car_status_t *app_car_get_status(void)
{
    return chassis_control_get_status();
}
