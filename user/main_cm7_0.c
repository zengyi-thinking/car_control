#include "zf_common_headfile.h"

#include "app/app_car.h"
#include "hardware/hw_config.h"

int main(void)
{
    uint32 debug_div = 0U;

    clock_init(SYSTEM_CLOCK_250M);
    debug_init();
    app_car_init();

    printf("[CAR] rebuilt DRV8701E mecanum control started\r\n");

    while (1)
    {
        const app_car_status_t *status;

        system_delay_ms(HW_CONTROL_PERIOD_MS);
        app_car_step();

        if (++debug_div >= 20U)
        {
            debug_div = 0U;
            status = app_car_get_status();
            printf("[CAR] mode:%u remote:%u nav:%u pose:(%.2f,%.2f,%.1f) target:(%.2f,%.2f,%.1f) cmd:(%.2f,%.2f,%.2f)\r\n",
                   (uint8)status->mode,
                   status->remote_fresh,
                   status->nav_fresh,
                   status->pose_x_m,
                   status->pose_y_m,
                   status->pose_yaw_deg,
                   status->target_x_m,
                   status->target_y_m,
                   status->target_yaw_deg,
                   status->cmd_vx_body_mps,
                   status->cmd_vy_body_mps,
                   status->cmd_omega_radps);
        }
    }
}
