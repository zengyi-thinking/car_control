#ifndef PROTO_DRONE_H
#define PROTO_DRONE_H

#include "zf_common_headfile.h"

#define DRONE_FRAME_HEAD                 0xA4U
#define DRONE_FRAME_TAIL                 0x55U
#define DRONE_FRAME_SIZE                 18U
#define DRONE_FRAME_SIZE_MAX             DRONE_FRAME_SIZE

#define PROTO_DRONE_FLAG_HAS_VLAMP       (1U << 0)
#define PROTO_DRONE_FLAG_HAS_BEACON      (1U << 1)
#define PROTO_DRONE_FLAG_MOVE_VALID      (1U << 2)
#define PROTO_DRONE_FLAG_HAS_YAW_ERR     (1U << 3)
#define PROTO_DRONE_FLAG_FOLLOW_OK       (1U << 4)
#define PROTO_DRONE_FLAG_MISSION_STOP    (1U << 5)

typedef struct
{
    int16 v_x_px;
    int16 v_y_px;
    int16 beacon_x_px;
    int16 beacon_y_px;
    int16 vb_x_px;
    int16 vb_y_px;
    int16 yaw_err_cdeg;
    uint16 v_conf_permille;
    uint16 beacon_conf_permille;
    uint8 flags;
    uint8 fresh;
} proto_drone_data_t;

void proto_drone_init(void);
void proto_drone_on_byte(uint8 byte);
uint8 proto_drone_update(proto_drone_data_t *out);

#endif
