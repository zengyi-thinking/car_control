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
    /* 阶段1 微调: DRONE_TRACK 进入/退出迟滞计数器, 抗信标抖动.
     * 5ms 控制周期下, 默认 3 帧退出迟滞 = 15ms 抗抖动窗口. */
    uint8 drone_track_enter_pending;
    uint8 drone_track_exit_pending;
} app_car_runtime_t;

/* 阶段1: DRONE_TRACK 退出迟滞. 连续 N 帧信标无效才真正退出, 抗信标抖动/短暂丢失.
 * 进入保持 1 帧有效即可, 保证起步响应. */
#define APP_DRONE_TRACK_EXIT_HOLD_FRAMES     3U

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

static uint8 app_drone_lost_hold_valid(void)
{
    return app_drone_fresh() &&
           ((g_app_runtime.drone.flags & PROTO_DRONE_FLAG_MOVE_VALID) == 0U) &&
           ((g_app_runtime.drone.flags & PROTO_DRONE_FLAG_HAS_VLAMP) != 0U) &&
           (chassis_control_drone_lost_hold_ready() != 0U);
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
    const uint8 drone_lost_hold_valid = app_drone_lost_hold_valid();
    const uint8 drone_track_valid = ((drone_move_valid != 0U) || (drone_lost_hold_valid != 0U)) ? 1U : 0U;

    if (g_chassis_wheel_pid_tune_enabled)
    {
        g_app_runtime.mode = CHASSIS_MODE_WHEEL_TUNE;
        g_app_runtime.drone_track_enter_pending = 0U;
        g_app_runtime.drone_track_exit_pending = 0U;
        return;
    }

    if (remote_override)
    {
        g_app_runtime.mode = CHASSIS_MODE_MANUAL;
        g_app_runtime.drone_track_enter_pending = 0U;
        g_app_runtime.drone_track_exit_pending = 0U;
        return;
    }

    /* DRONE_TRACK 进入/退出迟滞:
     * 进入: 1 帧有效即可 (反应要快, 否则起步慢)
     * 退出: 连续 N 帧无效才真正退出 (抗信标抖动/短暂丢失).
     * 信标真实长时间丢失仍会快速响应, 只是不再被单帧抖动打断. */
    if (drone_track_valid != 0U)
    {
        if (g_app_runtime.mode == CHASSIS_MODE_DRONE_TRACK)
        {
            /* 已经在 DRONE_TRACK, 保持并清零计数器 */
            g_app_runtime.drone_track_enter_pending = 0U;
            g_app_runtime.drone_track_exit_pending = 0U;
        }
        else
        {
            /* 从其他模式进 DRONE_TRACK: 1 帧有效即可 */
            g_app_runtime.drone_track_enter_pending = 1U;
            g_app_runtime.drone_track_exit_pending = 0U;
            g_app_runtime.mode = CHASSIS_MODE_DRONE_TRACK;
        }
    }
    else
    {
        if (g_app_runtime.mode == CHASSIS_MODE_DRONE_TRACK)
        {
            /* 当前在 DRONE_TRACK 但本帧无效: 累加退出计数 */
            g_app_runtime.drone_track_exit_pending++;
            g_app_runtime.drone_track_enter_pending = 0U;
            if (g_app_runtime.drone_track_exit_pending >= APP_DRONE_TRACK_EXIT_HOLD_FRAMES)
            {
                /* 连续 N 帧无效, 才真正退出 */
                g_app_runtime.mode = CHASSIS_MODE_SAFE_STOP;
                g_app_runtime.drone_track_exit_pending = 0U;
            }
            /* 否则保持 DRONE_TRACK: 让 lost_hold 等机制有时间响应,
             * 避免信标抖动 1~2 帧就触发模式切换 */
        }
        else
        {
            /* 既不在 DRONE_TRACK, 本帧又无效: SAFE_STOP */
            g_app_runtime.mode = CHASSIS_MODE_SAFE_STOP;
            g_app_runtime.drone_track_enter_pending = 0U;
            g_app_runtime.drone_track_exit_pending = 0U;
        }
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
