#ifndef HW_IMU_H
#define HW_IMU_H

#include "../hw_types.h"
#include "zf_common_headfile.h"

typedef flight_attitude_t car_attitude_t;

void hw_imu_init(void);
void hw_imu_update(void);
void hw_imu_irq_callback(void);
void hw_imu_read_attitude(flight_attitude_t *attitude);
void hw_imu_read_world_accel(flight_world_accel_t *world_accel);
void hw_imu_set_ahrs_gains(float kp, float ki);
void hw_imu_set_gyro_z_trim(float trim_dps);
void hw_imu_set_stationary_hint(uint8 stationary);

/* 读取经过滤波与零偏补偿后的加速度(g)和角速度(dps)。 */
void hw_imu_read_float(float accel_g[3], float gyro_dps[3]);

extern volatile uint8 g_hw_imu_stationary_hint;
extern volatile uint8 g_hw_imu_stationary_confirmed;
extern volatile float g_hw_imu_stationary_elapsed_s;
extern volatile float g_hw_imu_acc_norm_g;
extern volatile float g_hw_imu_acc_weight;

#endif
