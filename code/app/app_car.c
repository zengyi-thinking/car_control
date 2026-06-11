#include "app/app_car.h"

#include "hardware/hw_config.h"
#include "hardware/hw_display.h"
#include "hardware/hw_encoder.h"
#include "hardware/hw_indicator.h"
#include "hardware/hw_motor_drv8701e.h"
#include "hardware/sensors/hw_imu.h"
#include "protocol/proto_nav.h"
#include "protocol/proto_remote.h"

#include <string.h>

extern volatile uint8 g_chassis_wheel_pid_tune_enabled;

typedef struct
{
    proto_remote_data_t remote;
    proto_nav_data_t nav;
    uint8 has_remote;
    uint8 has_nav;
    uint16 remote_age_ms;
    uint16 nav_age_ms;
    chassis_mode_t mode;
} app_car_runtime_t;

static app_car_runtime_t g_app_runtime;

static uint8 app_remote_fresh(void)
{
    return g_app_runtime.has_remote && (g_app_runtime.remote_age_ms <= HW_REMOTE_TIMEOUT_MS);
}

static uint8 app_nav_fresh(void)
{
    return g_app_runtime.has_nav && (g_app_runtime.nav_age_ms <= HW_NAV_TIMEOUT_MS);
}

static void app_update_mode(void)
{
    const uint8 remote_fresh = app_remote_fresh();

    if (g_chassis_wheel_pid_tune_enabled)
    {
        g_app_runtime.mode = CHASSIS_MODE_WHEEL_TUNE;
        return;
    }

    switch (g_app_runtime.mode)
    {
        case CHASSIS_MODE_MANUAL:
        {
            if (!remote_fresh)
            {
                g_app_runtime.mode = CHASSIS_MODE_SAFE_STOP;
            }
        } break;

        case CHASSIS_MODE_SAFE_STOP:
        default:
        {
            if (remote_fresh)
            {
                g_app_runtime.mode = CHASSIS_MODE_MANUAL;
            }
            else
            {
                g_app_runtime.mode = CHASSIS_MODE_SAFE_STOP;
            }
        } break;
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
    proto_nav_init();
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

    if (proto_nav_update(&g_app_runtime.nav))
    {
        g_app_runtime.has_nav = 1U;
        g_app_runtime.nav_age_ms = 0U;
    }
    else if (g_app_runtime.nav_age_ms < 0xFFFFU)
    {
        g_app_runtime.nav_age_ms = (uint16)(g_app_runtime.nav_age_ms + HW_CONTROL_PERIOD_MS);
    }

    app_update_mode();

    chassis_control_set_mode(g_app_runtime.mode);
    chassis_control_set_remote(&g_app_runtime.remote, app_remote_fresh(), g_app_runtime.remote_age_ms);
    chassis_control_set_nav(&g_app_runtime.nav, app_nav_fresh(), g_app_runtime.nav_age_ms);
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

    if (status->remote_fresh || status->nav_fresh || status->tune_active || (status->position_move_state == CHASSIS_POSITION_MOVE_RUN))
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
