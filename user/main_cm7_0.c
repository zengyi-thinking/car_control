#include "zf_common_headfile.h"

#include "app/app_car.h"
#include "hardware/hw_config.h"

int main(void)
{
    extern volatile uint8 g_car_control_tick_pending;

    clock_init(SYSTEM_CLOCK_250M);
    app_car_init();
    pit_ms_init(PIT_CH0, HW_CONTROL_PERIOD_MS);

    while (1)
    {
        if (g_car_control_tick_pending == 0U)
        {
            continue;
        }

        __disable_irq();
        g_car_control_tick_pending = 0U;
        __enable_irq();

        app_car_step();

        /* Display work is best-effort.  Never start an LCD transfer when the
         * next 200 Hz control tick is already waiting. */
        if (g_car_control_tick_pending == 0U)
        {
            app_car_service_display();
        }
    }
}
