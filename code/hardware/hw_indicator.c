#include "hardware/hw_indicator.h"

#include "hardware/hw_board_pins.h"
#include "zf_common_headfile.h"

void hw_indicator_init(void)
{
    gpio_init(HW_INDICATOR_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

void hw_indicator_on(void)
{
    gpio_set_level(HW_INDICATOR_PIN, GPIO_HIGH);
}

void hw_indicator_off(void)
{
    gpio_set_level(HW_INDICATOR_PIN, GPIO_LOW);
}
