#include "zf_common_headfile.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);

    while (1)
    {
        system_delay_ms(500);
    }
}
