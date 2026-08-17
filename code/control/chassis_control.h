#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "hardware/sensors/hw_imu.h"
#include "control/pid.h"
#include "protocol/proto_drone.h"
#include "protocol/proto_remote.h"
#include "zf_common_typedef.h"

typedef enum
{
    CHASSIS_MODE_SAFE_STOP = 0,
    CHASSIS_MODE_MANUAL = 1,
    CHASSIS_MODE_HEADING_HOLD = 2,
    CHASSIS_MODE_WHEEL_TUNE = 3,
    CHASSIS_MODE_DRONE_TRACK = 4,
} chassis_mode_t;

typedef enum
{
    CHASSIS_POSITION_MOVE_IDLE = 0,
    CHASSIS_POSITION_MOVE_EDIT = 1,
    CHASSIS_POSITION_MOVE_RUN = 2,
    CHASSIS_POSITION_MOVE_DONE = 3,
} chassis_position_move_state_t;

typedef enum
{
    CHASSIS_BEACON_FOLLOW_WAIT_NEXT_EDGE = 0,
    CHASSIS_BEACON_FOLLOW_EDGE_TRACK = 1,
    CHASSIS_BEACON_FOLLOW_EDGE_CONTINUE = 2,
    CHASSIS_BEACON_FOLLOW_NEAR_PRESS = 3,
} chassis_beacon_follow_state_t;

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
    float vel_vx_body_mps;  /* 历史字段名：前向速度，小车 +Y。 */
    float vel_vy_body_mps;  /* 历史字段名：右向速度，小车 +X。 */
    float cmd_vx_field_mps;
    float cmd_vy_field_mps;
    float cmd_vx_body_mps;  /* 历史字段名：前向命令，小车 +Y。 */
    float cmd_vy_body_mps;  /* 历史字段名：右向命令，小车 +X。 */
    float cmd_omega_radps;
    float yaw_error_deg;
    float gyro_z_bias_radps;
    float gyro_z_motion_trim_radps;
    float startup_lock_elapsed_s;
    float joy_x_norm;
    float joy_y_norm;
    float user_cmd_x_mps;
    float user_cmd_y_mps;
    float pos_error_x_m;
    float pos_error_y_m;
    float distance_to_target_m;
    float position_edit_dx_m;
    float position_edit_dy_m;

    float wheel_target_mps[4];
    float wheel_measured_mps[4];
    float wheel_error_mps[4];
    float motor_duty[4];
    uint8 wheel_output_saturated[4];
    int16 encoder_counts[4];

    uint16 remote_age_ms;
    uint16 drone_age_ms;
    uint32 remote_rx_byte_count;
    uint32 remote_valid_frame_count;
    uint32 remote_checksum_error_count;
    uint8 remote_fresh;
    uint8 drone_fresh;
    uint8 drone_move_valid;
    chassis_beacon_follow_state_t drone_follow_state;
    uint16 drone_follow_remaining_ms;
    uint8 drone_hold_dir_valid;
    uint8 pose_valid;
    uint8 target_valid;
    uint8 heading_valid;
    uint8 startup_lock_done;
    uint8 enabled;
    uint8 tune_active;
    uint8 tune_wheel_index;
    uint8 tune_param_index;
    uint8 position_move_state;
    float tune_target_mps;
    float tune_selected_value;
    float tune_selected_step;
    int16 drone_v_x_px;
    int16 drone_v_y_px;
    int16 drone_beacon_x_px;
    int16 drone_beacon_y_px;
    int16 drone_vb_x_px;  /* Yaw补偿后：正值表示信标在小车右方(+X)。 */
    int16 drone_vb_y_px;  /* Yaw补偿后：正值表示信标在小车前方(+Y)。 */
    int16 drone_yaw_err_cdeg;
    uint16 drone_v_conf_permille;
    uint16 drone_beacon_conf_permille;
    uint8 drone_flags;
} chassis_status_t;

extern volatile uint8 g_chassis_wheel_pid_tune_enabled;
extern volatile uint8 g_chassis_wheel_pid_tune_wheel;
extern volatile uint8 g_chassis_wheel_pid_tune_param;
extern volatile float g_chassis_wheel_pid_tune_target_mps;
extern pid_t g_wheel_pid[4];

void chassis_control_init(void);
void chassis_control_reset(void);
void chassis_control_set_mode(chassis_mode_t mode);
void chassis_control_set_remote(const proto_remote_data_t *remote,
                                uint8 fresh,
                                uint16 age_ms,
                                const proto_remote_diag_t *diag);
void chassis_control_set_drone(const proto_drone_data_t *drone, uint8 fresh, uint16 age_ms);
uint8 chassis_control_drone_follow_ready(void);
uint8 chassis_control_drone_lost_hold_ready(void);
void chassis_control_step(const int16 enc_counts[4], float dt_s, const flight_attitude_t *attitude);
const chassis_status_t *chassis_control_get_status(void);
void chassis_control_get_motor_duty(int16 duty_out[4]);

#endif
