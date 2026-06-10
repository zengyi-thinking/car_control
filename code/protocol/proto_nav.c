/*******************************************************************************
 * 通信层
 *
 * 兼容两种帧:
 * 1. 0xA1 旧版误差帧
 *    0xA1 | err_fwd_H | err_fwd_L | err_lat_H | err_lat_L | flags | xor | 0x55
 *
 * 2. 0xA2 新版导航帧
 *    0xA2 | cur_x_H | cur_x_L | cur_y_H | cur_y_L |
 *          tgt_x_H | tgt_x_L | tgt_y_H | tgt_y_L |
 *          yaw_H   | yaw_L   | flags   | xor | 0x55
 *
 * 其中 yaw 单位为 0.1 deg，坐标单位为 mm。
 ******************************************************************************/

#include "protocol/proto_nav.h"
#include "hardware/hw_board_pins.h"
#include "zf_driver_uart.h"

static uint8 rx_buf[NAV_FRAME_SIZE_MAX];
static uint8 rx_idx;
static uint8 rx_expected_size;
static proto_nav_data_t g_latest_nav;
static uint8 g_nav_pending = 0U;

static uint8 proto_nav_checksum(const uint8 *buf, uint8 len_without_checksum_and_tail)
{
    uint8 xor_val = 0;

    for (uint8 i = 0; i < len_without_checksum_and_tail; i++)
    {
        xor_val ^= buf[i];
    }
    return xor_val;
}

static void proto_nav_reset_parser(void)
{
    rx_idx = 0;
    rx_expected_size = 0;
}

static uint8 proto_nav_try_parse(proto_nav_data_t *out)
{
    if ((out == 0) || (rx_expected_size == 0) || (rx_idx < rx_expected_size))
    {
        return 0;
    }

    if (rx_buf[rx_expected_size - 1U] != NAV_FRAME_TAIL)
    {
        proto_nav_reset_parser();
        return 0;
    }

    if (proto_nav_checksum(rx_buf, (uint8)(rx_expected_size - 2U)) != rx_buf[rx_expected_size - 2U])
    {
        proto_nav_reset_parser();
        return 0;
    }

    if (rx_buf[0] == NAV_FRAME_HEAD_LEGACY)
    {
        out->mode = PROTO_NAV_MODE_LEGACY_ERROR;
        out->err_fwd = (int16)(((uint16)rx_buf[1] << 8) | rx_buf[2]);
        out->err_lat = (int16)(((uint16)rx_buf[3] << 8) | rx_buf[4]);
        out->flags = rx_buf[5];
        out->fresh = 1U;
    }
    else if (rx_buf[0] == NAV_FRAME_HEAD_TARGET)
    {
        out->mode = PROTO_NAV_MODE_TARGET;
        out->current_x_mm = (int16)(((uint16)rx_buf[1] << 8) | rx_buf[2]);
        out->current_y_mm = (int16)(((uint16)rx_buf[3] << 8) | rx_buf[4]);
        out->target_x_mm = (int16)(((uint16)rx_buf[5] << 8) | rx_buf[6]);
        out->target_y_mm = (int16)(((uint16)rx_buf[7] << 8) | rx_buf[8]);
        out->heading_deg_x10 = (int16)(((uint16)rx_buf[9] << 8) | rx_buf[10]);
        out->flags = rx_buf[11];
        out->fresh = 1U;
    }
    else
    {
        proto_nav_reset_parser();
        return 0;
    }

    proto_nav_reset_parser();
    return 1U;
}

void proto_nav_init(void)
{
    uart_init(HW_NAV_UART, HW_NAV_UART_BAUD, HW_NAV_UART_TX_PIN, HW_NAV_UART_RX_PIN);
    uart_rx_interrupt(HW_NAV_UART, 1);
    proto_nav_reset_parser();
    g_nav_pending = 0U;
}

void proto_nav_on_byte(uint8 byte)
{
    if (rx_idx == 0U)
    {
        if (byte == NAV_FRAME_HEAD_LEGACY)
        {
            rx_expected_size = NAV_FRAME_SIZE_LEGACY;
        }
        else if (byte == NAV_FRAME_HEAD_TARGET)
        {
            rx_expected_size = NAV_FRAME_SIZE_TARGET;
        }
        else
        {
            return;
        }
        rx_buf[rx_idx++] = byte;
        return;
    }

    if (rx_idx >= NAV_FRAME_SIZE_MAX)
    {
        proto_nav_reset_parser();
        return;
    }

    rx_buf[rx_idx++] = byte;
    if (rx_idx < rx_expected_size)
    {
        return;
    }

    if (proto_nav_try_parse(&g_latest_nav))
    {
        g_nav_pending = 1U;
    }
}

uint8 proto_nav_update(proto_nav_data_t *out)
{
    if ((out == 0) || (g_nav_pending == 0U))
    {
        return 0U;
    }

    *out = g_latest_nav;
    g_nav_pending = 0U;
    return 1U;
}
