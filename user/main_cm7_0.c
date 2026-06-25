#include "zf_common_headfile.h"

#include "app/app_car.h"
#include "hardware/hw_config.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    app_car_init();

    while (1)
    {
        system_delay_ms(HW_CONTROL_PERIOD_MS);
        app_car_step();
    }
}
