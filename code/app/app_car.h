#ifndef APP_CAR_H
#define APP_CAR_H

#include "control/chassis_control.h"

typedef chassis_status_t app_car_status_t;

void app_car_init(void);
void app_car_step(void);
const app_car_status_t *app_car_get_status(void);

#endif
