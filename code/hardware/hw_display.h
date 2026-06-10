#ifndef HW_DISPLAY_H
#define HW_DISPLAY_H

#include "control/chassis_control.h"

void hw_display_init(void);
void hw_display_update(const chassis_status_t *status);

#endif
