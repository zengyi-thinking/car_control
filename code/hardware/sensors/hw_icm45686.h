#ifndef HW_ICM45686_H
#define HW_ICM45686_H

#include "../hw_types.h"

void hw_icm45686_init(void);
void hw_icm45686_update(void);
void hw_icm45686_irq_callback(void);
void hw_icm45686_read(flight_imu_raw_t *imu_raw);
const flight_imu_raw_t *hw_icm45686_get_local_raw(void);

int hw_icm45686_get_raw(float accel_g[3], float gyro_dps[3], float *temp);
void hw_icm45686_read_float(float accel_g[3], float gyro_dps[3]);
int hw_icm45686_ready(void);

#endif
