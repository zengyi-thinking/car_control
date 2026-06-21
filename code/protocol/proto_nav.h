#ifndef PROTO_NAV_H
#define PROTO_NAV_H

#include "zf_common_headfile.h"

// ============================================
// 协议定义
// ============================================
#define NAV_FRAME_HEAD_TARGET          0xA2U
#define NAV_FRAME_TAIL                 0x55U
#define NAV_FRAME_SIZE_TARGET          14U
#define NAV_FRAME_SIZE_MAX             NAV_FRAME_SIZE_TARGET

// flags bit definition for nav frame
#define PROTO_NAV_FLAG_CURRENT_VALID   (1U << 0)
#define PROTO_NAV_FLAG_TARGET_VALID    (1U << 1)
#define PROTO_NAV_FLAG_HEADING_VALID   (1U << 2)
#define PROTO_NAV_FLAG_HEADING_HOLD    (1U << 3)

typedef struct
{
    // A2 场地坐标, 单位: mm / 0.1 deg
    // 约定: +X = 左, +Y = 前
    int16   current_x_mm;
    int16   current_y_mm;
    int16   target_x_mm;
    int16   target_y_mm;
    int16   heading_deg_x10;

    uint8   flags;
    uint8   fresh;
} proto_nav_data_t;

// ============================================
// 函数声明
// ============================================
void proto_nav_init(void);
void proto_nav_on_byte(uint8 byte);
uint8 proto_nav_update(proto_nav_data_t *out);

#endif
