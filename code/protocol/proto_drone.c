#include "protocol/proto_drone.h"

#include "hardware/hw_config.h"
#include "hardware/hw_board_pins.h"
#include "scb/cy_scb_uart.h"
#include "zf_driver_uart.h"

static uint8 rx_buf[DRONE_FRAME_SIZE_MAX];
static uint8 rx_idx;
static uint8 rx_expected_size;
static proto_drone_data_t g_latest_drone;
static uint8 g_drone_pending = 0U;
static uint8 g_car_motion_tx_seq = 0U;

#define PROTO_CAR_MOTION_FRAME_SIZE       13U
#define PROTO_CAR_MOTION_CHECKSUM_INDEX   11U

static uint8 proto_drone_checksum(const uint8 *buf, uint8 len_without_checksum_and_tail)
{
    uint8 xor_val = 0U;

    for(uint8 i = 0U; i < len_without_checksum_and_tail; ++i)
    {
        xor_val ^= buf[i];
    }

    return xor_val;
}

static int16 proto_drone_read_i16(uint8 high, uint8 low)
{
    return (int16)(((uint16)high << 8) | low);
}

static uint16 proto_drone_read_u16(uint8 high, uint8 low)
{
    return (uint16)(((uint16)high << 8) | low);
}

static void proto_drone_put_i16(uint8 *buf, uint8 index, int16 value)
{
    const uint16 raw = (uint16)value;

    buf[index] = (uint8)(raw >> 8);
    buf[(uint8)(index + 1U)] = (uint8)raw;
}

static uint8 proto_drone_try_write_car_motion(const uint8 *frame)
{
    volatile stc_SCB_t *scb = get_scb_module(HW_DRONE_UART);
    uint32 fifo_free;

    if((frame == 0) || (scb == 0))
    {
        return 0U;
    }

    fifo_free = Cy_SCB_GetFifoSize(scb) - Cy_SCB_UART_GetNumInTxFifo(scb);
    if((Cy_SCB_UART_IsTxComplete(scb) == 0) || (fifo_free < PROTO_CAR_MOTION_FRAME_SIZE))
    {
        return 0U;
    }

    return (Cy_SCB_UART_PutArray(scb, (void *)frame, PROTO_CAR_MOTION_FRAME_SIZE)
            == PROTO_CAR_MOTION_FRAME_SIZE) ? 1U : 0U;
}

static void proto_drone_reset_parser(void)
{
    rx_idx = 0U;
    rx_expected_size = 0U;
}

static uint8 proto_drone_try_parse(proto_drone_data_t *out)
{
    if((out == 0) || (rx_expected_size == 0U) || (rx_idx < rx_expected_size))
    {
        return 0U;
    }

    if(rx_buf[rx_expected_size - 1U] != DRONE_FRAME_TAIL)
    {
        proto_drone_reset_parser();
        return 0U;
    }

    if(proto_drone_checksum(rx_buf, (uint8)(rx_expected_size - 2U)) != rx_buf[rx_expected_size - 2U])
    {
        proto_drone_reset_parser();
        return 0U;
    }

    if(rx_buf[0] != DRONE_FRAME_HEAD)
    {
        proto_drone_reset_parser();
        return 0U;
    }

    out->v_x_px = proto_drone_read_i16(rx_buf[1], rx_buf[2]);
    out->v_y_px = proto_drone_read_i16(rx_buf[3], rx_buf[4]);
    out->beacon_x_px = proto_drone_read_i16(rx_buf[5], rx_buf[6]);
    out->beacon_y_px = proto_drone_read_i16(rx_buf[7], rx_buf[8]);
    out->v_conf_permille = proto_drone_read_u16(rx_buf[9], rx_buf[10]);
    out->beacon_conf_permille = proto_drone_read_u16(rx_buf[11], rx_buf[12]);
    out->flags = rx_buf[13];
    out->yaw_err_cdeg = proto_drone_read_i16(rx_buf[14], rx_buf[15]);
    if((out->flags & PROTO_DRONE_FLAG_HAS_YAW_ERR) == 0U)
    {
        out->yaw_err_cdeg = 0;
    }
    if((out->flags & PROTO_DRONE_FLAG_MOVE_VALID) != 0U)
    {
        out->vb_x_px = (int16)(out->beacon_x_px - out->v_x_px);
        out->vb_y_px = (int16)(out->beacon_y_px - out->v_y_px);
    }
    else
    {
        out->vb_x_px = 0;
        out->vb_y_px = 0;
    }
    out->fresh = 1U;

    proto_drone_reset_parser();
    return 1U;
}

void proto_drone_init(void)
{
    uart_init(HW_DRONE_UART, HW_DRONE_UART_BAUD, HW_DRONE_UART_TX_PIN, HW_DRONE_UART_RX_PIN);
    uart_rx_interrupt(HW_DRONE_UART, 1U);
    proto_drone_reset_parser();
    g_drone_pending = 0U;
    g_car_motion_tx_seq = 0U;
}

void proto_drone_on_byte(uint8 byte)
{
    if(rx_idx == 0U)
    {
        if(byte == DRONE_FRAME_HEAD)
        {
            rx_expected_size = DRONE_FRAME_SIZE;
        }
        else
        {
            return;
        }
        rx_buf[rx_idx++] = byte;
        return;
    }

    if(rx_idx >= DRONE_FRAME_SIZE_MAX)
    {
        proto_drone_reset_parser();
        return;
    }

    rx_buf[rx_idx++] = byte;
    if(rx_idx < rx_expected_size)
    {
        return;
    }

    if(proto_drone_try_parse(&g_latest_drone) != 0U)
    {
        g_drone_pending = 1U;
    }
}

uint8 proto_drone_update(proto_drone_data_t *out)
{
    if((out == 0) || (g_drone_pending == 0U))
    {
        return 0U;
    }

    *out = g_latest_drone;
    g_drone_pending = 0U;
    return 1U;
}

void proto_drone_send_car_motion(int16 cmd_vx_body_mmps,
                                 int16 cmd_vy_body_mmps,
                                 int16 measured_vx_body_mmps,
                                 int16 measured_vy_body_mmps,
                                 uint8 flags)
{
#if HW_DRONE_MOTION_STATUS_ENABLE != 0U
    uint8 frame[PROTO_CAR_MOTION_FRAME_SIZE] = {0U};

    frame[0] = PROTO_CAR_MOTION_FRAME_HEAD;
    proto_drone_put_i16(frame, 1U, cmd_vx_body_mmps);
    proto_drone_put_i16(frame, 3U, cmd_vy_body_mmps);
    proto_drone_put_i16(frame, 5U, measured_vx_body_mmps);
    proto_drone_put_i16(frame, 7U, measured_vy_body_mmps);
    frame[9] = flags;
    frame[10] = g_car_motion_tx_seq;
    frame[PROTO_CAR_MOTION_CHECKSUM_INDEX] =
        proto_drone_checksum(frame, PROTO_CAR_MOTION_CHECKSUM_INDEX);
    frame[PROTO_CAR_MOTION_FRAME_SIZE - 1U] = PROTO_CAR_MOTION_FRAME_TAIL;

    if(proto_drone_try_write_car_motion(frame) != 0U)
    {
        g_car_motion_tx_seq++;
    }
#else
    (void)cmd_vx_body_mmps;
    (void)cmd_vy_body_mmps;
    (void)measured_vx_body_mmps;
    (void)measured_vy_body_mmps;
    (void)flags;
#endif
}
