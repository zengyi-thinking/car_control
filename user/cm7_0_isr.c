#include "zf_common_headfile.h"
#include "hardware/hw_board_pins.h"
#include "protocol/proto_drone.h"
#include "protocol/proto_remote.h"

static void app_uart_service(uart_index_enum uart_index)
{
    uint8 byte;

    if (!uart_isr_mask(uart_index))
    {
        return;
    }

    while (uart_query_byte(uart_index, &byte))
    {
        if (uart_index == HW_REMOTE_UART)
        {
            proto_remote_on_byte(byte);
        }
        else if (uart_index == HW_DRONE_UART)
        {
            proto_drone_on_byte(byte);
        }
    }
}

void pit0_ch0_isr(void)  { pit_isr_flag_clear(PIT_CH0); }
void pit0_ch1_isr(void)  { pit_isr_flag_clear(PIT_CH1); }
void pit0_ch2_isr(void)  { pit_isr_flag_clear(PIT_CH2); }
void pit0_ch10_isr(void) { pit_isr_flag_clear(PIT_CH10); }
void pit0_ch11_isr(void) { pit_isr_flag_clear(PIT_CH11); }
void pit0_ch12_isr(void) { pit_isr_flag_clear(PIT_CH12); }
void pit0_ch13_isr(void) { pit_isr_flag_clear(PIT_CH13); }
void pit0_ch14_isr(void) { pit_isr_flag_clear(PIT_CH14); }
void pit0_ch15_isr(void) { pit_isr_flag_clear(PIT_CH15); }
void pit0_ch16_isr(void) { pit_isr_flag_clear(PIT_CH16); }
void pit0_ch17_isr(void) { pit_isr_flag_clear(PIT_CH17); }
void pit0_ch18_isr(void) { pit_isr_flag_clear(PIT_CH18); }
void pit0_ch19_isr(void) { pit_isr_flag_clear(PIT_CH19); }
void pit0_ch20_isr(void) { pit_isr_flag_clear(PIT_CH20); }
void pit0_ch21_isr(void) { pit_isr_flag_clear(PIT_CH21); }

void uart0_isr(void)
{
    if (uart_isr_mask(UART_0))
    {
#if DEBUG_UART_USE_INTERRUPT
        debug_interrupr_handler();
#endif
    }
}

void uart1_isr(void) { app_uart_service(UART_1); }
void uart2_isr(void) { app_uart_service(UART_2); }
void uart3_isr(void) { app_uart_service(UART_3); }
void uart4_isr(void) { app_uart_service(UART_4); }
void uart5_isr(void) { app_uart_service(UART_5); }
void uart6_isr(void) { app_uart_service(UART_6); }
