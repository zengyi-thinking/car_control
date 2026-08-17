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
    /* 阶段1 微调: DRONE_TRACK 进入/退出迟滞计数器, 抗信标抖动.
     * 5ms 控制周期下, 默认 3 帧退出迟滞 = 15ms 抗抖动窗口. */
    uint8 drone_track_enter_pending;
    uint8 drone_track_exit_pending;
    uint8 mission_stop_latched;
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

static uint8 app_drone_follow_valid(void)
{
    return chassis_control_drone_follow_ready();
}

static void app_update_mode(void)
{
    const uint8 remote_fresh = app_remote_fresh();
    const uint8 drone_move_valid = app_drone_move_valid();
    const uint8 drone_follow_valid = app_drone_follow_valid();
    const uint8 drone_track_valid = ((drone_move_valid != 0U) || (drone_follow_valid != 0U)) ? 1U : 0U;

    if(g_app_runtime.mission_stop_latched != 0U)
    {
        g_app_runtime.mode = CHASSIS_MODE_SAFE_STOP;
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
                /* 连续 N 帧无效后，下一优先级才允许遥控接管。 */
                g_app_runtime.mode = (remote_fresh != 0U) ? CHASSIS_MODE_MANUAL : CHASSIS_MODE_HEADING_HOLD;
                g_app_runtime.drone_track_exit_pending = 0U;
            }
            /* 否则保持 DRONE_TRACK: 让续航状态机有时间响应,
             * 避免信标抖动 1~2 帧就触发模式切换 */
        }
        else
        {
            /* 最初遥控语义：收到新鲜A3帧即进入MANUAL，无需额外摇杆门限。 */
            g_app_runtime.mode = (remote_fresh != 0U) ? CHASSIS_MODE_MANUAL : CHASSIS_MODE_HEADING_HOLD;
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
