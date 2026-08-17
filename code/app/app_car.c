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

typedef struct
{
    proto_remote_data_t remote;
    proto_remote_diag_t remote_diag;
    proto_drone_data_t drone;
    uint8 has_remote;
    uint8 has_drone;
    uint16 remote_age_ms;
    uint16 drone_age_ms;
    chassis_mode_t mode;
    /* 当前追踪模式的视觉无效累计时间。短时无效不切走模式，
     * 让 chassis_control 的上一可靠方向保持逻辑连续工作。 */
    uint16 drone_invalid_ms;
    uint8 mission_stop_latched;
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

static void app_update_mode(void)
{
    const uint8 remote_fresh = app_remote_fresh();
    const uint8 drone_move_valid = app_drone_move_valid();

    if(g_app_runtime.mission_stop_latched != 0U)
    {
        g_app_runtime.mode = CHASSIS_MODE_SAFE_STOP;
        g_app_runtime.drone_invalid_ms = 0U;
        return;
    }

    if (drone_move_valid != 0U)
    {
        /* 新鲜有效帧重新获得租约，立即恢复追踪且不清空底层方向缓存。 */
        g_app_runtime.drone_invalid_ms = 0U;
        g_app_runtime.mode = CHASSIS_MODE_DRONE_TRACK;
        return;
    }

    if (g_app_runtime.mode == CHASSIS_MODE_DRONE_TRACK)
    {
        /* A4仍在线但本帧没有可用B1时，先让底层沿上一可靠方向运行。
         * 若链路本身已经超时，则不再延长租约。 */
        if ((g_app_runtime.has_drone != 0U) &&
            (g_app_runtime.drone_age_ms <= HW_DRONE_TIMEOUT_MS) &&
            (g_app_runtime.drone_invalid_ms < HW_DRONE_TRACK_LOST_GRACE_MS))
        {
            if (g_app_runtime.drone_invalid_ms <=
                (uint16)(HW_DRONE_TRACK_LOST_GRACE_MS - HW_CONTROL_PERIOD_MS))
            {
                g_app_runtime.drone_invalid_ms = (uint16)(g_app_runtime.drone_invalid_ms +
                                                          HW_CONTROL_PERIOD_MS);
                return;
            }
        }

        /* 租约耗尽后才允许退出自动追踪，避免每次短丢帧都重置滤波器。 */
        g_app_runtime.drone_invalid_ms = 0U;
        g_app_runtime.mode = (remote_fresh != 0U) ? CHASSIS_MODE_MANUAL : CHASSIS_MODE_HEADING_HOLD;
        return;
    }

    g_app_runtime.drone_invalid_ms = 0U;
    /* 未曾建立自动追踪时，保持原有遥控/航向保持优先级。 */
    g_app_runtime.mode = (remote_fresh != 0U) ? CHASSIS_MODE_MANUAL : CHASSIS_MODE_HEADING_HOLD;
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
    uint8 drone_frame_new = 0U;

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
        drone_frame_new = 1U;
        if((g_app_runtime.drone.flags & PROTO_DRONE_FLAG_MISSION_STOP) != 0U)
        {
            g_app_runtime.mission_stop_latched = 1U;
        }
    }
    else if (g_app_runtime.drone_age_ms < 0xFFFFU)
    {
        g_app_runtime.drone_age_ms = (uint16)(g_app_runtime.drone_age_ms + HW_CONTROL_PERIOD_MS);
    }
    proto_remote_get_diag(&g_app_runtime.remote_diag);

    /* 任务停车只由无人机 A4 的 MISSION_STOP 标志触发(降落停桨后小车锁存停车),
     * 不再有本地上电计时自动停车。 */

    app_update_mode();

    chassis_control_set_mode(g_app_runtime.mode);
    chassis_control_set_remote(&g_app_runtime.remote,
                               app_remote_fresh(),
                               g_app_runtime.remote_age_ms,
                               &g_app_runtime.remote_diag);
    chassis_control_set_drone(&g_app_runtime.drone, app_drone_fresh(), g_app_runtime.drone_age_ms);
    chassis_control_step(enc_counts, HW_CONTROL_PERIOD_S, &attitude);

    status = chassis_control_get_status();

    /* 收到新 A4 帧即回传车体实测速度(编码器里程计), 与无人机 20ms
     * 节拍天然同步, 作为追踪前馈的真实车速来源。 */
    if((drone_frame_new != 0U) && (status != 0))
    {
        proto_drone_send_speed((int16)(status->vel_vx_body_mps * 1000.0f),
                               (int16)(status->vel_vy_body_mps * 1000.0f));
    }

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

}

void app_car_service_display(void)
{
    hw_display_update(chassis_control_get_status());
}

const app_car_status_t *app_car_get_status(void)
{
    return chassis_control_get_status();
}
