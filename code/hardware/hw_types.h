#ifndef HW_TYPES_H
#define HW_TYPES_H

#include <stdint.h>

typedef struct
{
    int acc_x;
    int acc_y;
    int acc_z;
    int gyro_x;
    int gyro_y;
    int gyro_z;
    float dt;
    unsigned char valid;
} flight_imu_raw_t;

typedef struct
{
    float quat[4];
    float roll;
    float pitch;
    float yaw;
    float gyro_dps[3];
    unsigned char valid;
} flight_attitude_t;

typedef struct
{
    float north;
    float east;
    float up;
    unsigned char valid;
} flight_world_accel_t;

typedef struct
{
    unsigned short distance_mm;
    float height_mm;
    unsigned char valid;
    unsigned char fresh;
    unsigned short age_ms;
    float sample_dt;
} flight_tof_raw_t;

typedef struct
{
    int motor[4];
} flight_motor_output_t;

typedef struct
{
    int16_t pixel_flow_x_integral;
    int16_t pixel_flow_y_integral;
    uint32_t integration_timespan;
    uint16_t ground_distance;
    uint8_t qual;
    uint8_t valid;
} flight_flow_raw_t;

typedef struct
{
    float x;
    float y;
    uint32_t dt;
    uint8_t qual;
    uint8_t update;
} flight_flow_float_t;

#endif
