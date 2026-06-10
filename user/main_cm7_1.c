#include "zf_common_headfile.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();

    printf("[CAR] CM7_1 idle\r\n");

    while (1)
    {
        system_delay_ms(500);
    }
}
