#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "hardware/sensors/hw_imu.h"
#include "control/pid.h"
#include "protocol/proto_nav.h"
#include "protocol/proto_remote.h"
#include "zf_common_typedef.h"

typedef enum
{
    CHASSIS_MODE_SAFE_STOP = 0,
    CHASSIS_MODE_MANUAL = 1,
    CHASSIS_MODE_NAV = 2,
    CHASSIS_MODE_WHEEL_TUNE = 3,
} chassis_mode_t;

typedef enum
{
    CHASSIS_TUNE_PARAM_KP = 0,
    CHASSIS_TUNE_PARAM_KI = 1,
    CHASSIS_TUNE_PARAM_KD = 2,
    CHASSIS_TUNE_PARAM_COUNT = 3,
} chassis_tune_param_t;

typedef struct
{
    chassis_mode_t mode;

    float odom_x_m;
    float odom_y_m;
    float pose_x_m;
    float pose_y_m;
    float pose_yaw_deg;
    float pose_meas_x_m;
    float pose_meas_y_m;

    float target_x_m;
    float target_y_m;
    float target_yaw_deg;

    float vel_x_field_mps;
    float vel_y_field_mps;
    float cmd_vx_field_mps;
    float cmd_vy_field_mps;
    float cmd_vx_body_mps;
    float cmd_vy_body_mps;
    float cmd_omega_radps;
    float yaw_error_deg;
    float gyro_z_bias_radps;
    float startup_lock_elapsed_s;
    float joy_x_norm;
    float joy_y_norm;
    float user_cmd_x_mps;
    float user_cmd_y_mps;
    float pos_error_x_m;
    float pos_error_y_m;
    float distance_to_target_m;

    float wheel_target_mps[4];
    float wheel_measured_mps[4];
    float wheel_error_mps[4];
    float motor_duty[4];
    uint8 wheel_output_saturated[4];
    int16 encoder_counts[4];

    uint16 remote_age_ms;
    uint16 nav_age_ms;
    uint8 remote_fresh;
    uint8 nav_fresh;
    uint8 pose_valid;
    uint8 target_valid;
    uint8 heading_valid;
    uint8 startup_lock_done;
    uint8 using_legacy_mode;
    uint8 enabled;
    uint8 tune_active;
    uint8 tune_wheel_index;
    uint8 tune_param_index;
    float tune_target_mps;
    float tune_selected_value;
    float tune_selected_step;
} chassis_status_t;

extern volatile uint8 g_chassis_wheel_pid_tune_enabled;
extern volatile uint8 g_chassis_wheel_pid_tune_wheel;
extern volatile uint8 g_chassis_wheel_pid_tune_param;
extern volatile float g_chassis_wheel_pid_tune_target_mps;
extern pid_t g_wheel_pid[4];

void chassis_control_init(void);
void chassis_control_reset(void);
void chassis_control_set_mode(chassis_mode_t mode);
void chassis_control_set_remote(const proto_remote_data_t *remote, uint8 fresh, uint16 age_ms);
void chassis_control_set_nav(const proto_nav_data_t *nav, uint8 fresh, uint16 age_ms);
void chassis_control_step(const int16 enc_counts[4], float dt_s, const flight_attitude_t *attitude);
const chassis_status_t *chassis_control_get_status(void);
void chassis_control_get_motor_duty(int16 duty_out[4]);

#endif
