#ifndef HW_MOTOR_DRV8701E_H
#define HW_MOTOR_DRV8701E_H

#include "zf_common_headfile.h"

void hw_motor_drv8701e_init(void);
void hw_motor_drv8701e_set(uint8 motor_id, int16 duty);
void hw_motor_drv8701e_set_all(const int16 duty[4]);
void hw_motor_drv8701e_stop_all(void);

#endif
