#ifndef PROTO_REMOTE_H
#define PROTO_REMOTE_H

#include "zf_common_headfile.h"

#define REMOTE_FRAME_HEAD            0xA3U
#define REMOTE_FRAME_SIZE            18U
#define REMOTE_HEADING_RESET_KEY     0U
#define REMOTE_RIGHT_STICK_PRESS_KEY  1U
#define REMOTE_NAV_ENTER_KEY         2U
#define REMOTE_NAV_EXIT_KEY          3U

typedef struct
{
    int16 joystick[4];
    uint8 key[4];
    uint8 switch_key[4];
    uint8 fresh;
} proto_remote_data_t;

typedef struct
{
    uint32 rx_byte_count;
    uint32 valid_frame_count;
    uint32 checksum_error_count;
} proto_remote_diag_t;

void proto_remote_init(void);
void proto_remote_on_byte(uint8 byte);
uint8 proto_remote_update(proto_remote_data_t *out);
void proto_remote_get_diag(proto_remote_diag_t *out);

#endif
