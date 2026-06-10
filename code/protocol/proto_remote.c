/*******************************************************************************
 * LoRa3A22 遥控接收协议
 *
 * 遥控器发送帧结构（小端）:
 * head(0xA3) | sum_check | joystick[4] int16 | key[4] | switch_key[4]
 *
 * 说明:
 * - 发送端在计算校验和时，会先把 sum_check 置 0，再对整帧所有字节求和。
 * - 本接收端按同样方式验证。
 ******************************************************************************/

#include "protocol/proto_remote.h"
#include "hardware/hw_board_pins.h"
#include "zf_driver_uart.h"
#include "string.h"

typedef struct
{
    uint8 head;
    uint8 sum_check;
    int16 joystick[4];
    uint8 key[4];
    uint8 switch_key[4];
} remote_frame_t;

static uint8 g_rx_buf[REMOTE_FRAME_SIZE];
static uint8 g_rx_idx = 0U;
static proto_remote_data_t g_latest_remote;
static uint8 g_remote_pending = 0U;

static uint8 proto_remote_calc_sum(const uint8 *buf, uint8 len)
{
    uint8 sum = 0U;

    for (uint8 i = 0; i < len; i++)
    {
        sum = (uint8)(sum + buf[i]);
    }
    return sum;
}

static void proto_remote_reset(void)
{
    g_rx_idx = 0U;
}

static uint8 proto_remote_try_parse(proto_remote_data_t *out)
{
    remote_frame_t frame;
    uint8 sum;

    if ((out == 0) || (g_rx_idx < REMOTE_FRAME_SIZE))
    {
        return 0U;
    }

    if (g_rx_buf[0] != REMOTE_FRAME_HEAD)
    {
        proto_remote_reset();
        return 0U;
    }

    memcpy(&frame, g_rx_buf, sizeof(frame));
    frame.sum_check = 0U;
    sum = proto_remote_calc_sum((const uint8 *)&frame, (uint8)sizeof(frame));
    if (sum != g_rx_buf[1])
    {
        proto_remote_reset();
        return 0U;
    }

    memcpy(out->joystick, ((remote_frame_t *)g_rx_buf)->joystick, sizeof(out->joystick));
    memcpy(out->key, ((remote_frame_t *)g_rx_buf)->key, sizeof(out->key));
    memcpy(out->switch_key, ((remote_frame_t *)g_rx_buf)->switch_key, sizeof(out->switch_key));
    out->fresh = 1U;

    proto_remote_reset();
    return 1U;
}

void proto_remote_init(void)
{
    uart_init(HW_REMOTE_UART, HW_REMOTE_UART_BAUD, HW_REMOTE_UART_TX_PIN, HW_REMOTE_UART_RX_PIN);
    uart_rx_interrupt(HW_REMOTE_UART, 1);
    proto_remote_reset();
    memset(&g_latest_remote, 0, sizeof(g_latest_remote));
    g_remote_pending = 0U;
}

void proto_remote_on_byte(uint8 byte)
{
    if (g_rx_idx == 0U)
    {
        if (byte != REMOTE_FRAME_HEAD)
        {
            return;
        }
        g_rx_buf[g_rx_idx++] = byte;
        return;
    }

    if (g_rx_idx >= REMOTE_FRAME_SIZE)
    {
        proto_remote_reset();
        if (byte == REMOTE_FRAME_HEAD)
        {
            g_rx_buf[g_rx_idx++] = byte;
        }
        return;
    }

    g_rx_buf[g_rx_idx++] = byte;
    if (g_rx_idx >= REMOTE_FRAME_SIZE)
    {
        if (proto_remote_try_parse(&g_latest_remote))
        {
            g_remote_pending = 1U;
        }
    }
}

uint8 proto_remote_update(proto_remote_data_t *out)
{
    if ((out == 0) || (g_remote_pending == 0U))
    {
        return 0U;
    }

    *out = g_latest_remote;
    g_remote_pending = 0U;
    return 1U;
}
