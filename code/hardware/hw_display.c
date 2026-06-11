#include "hardware/hw_display.h"

#include "hardware/hw_config.h"
#include "zf_common_headfile.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define DISPLAY_LINE_H               8U
#define DISPLAY_TEXT_COLS            18U
#define DISPLAY_LINE_BUF_LEN         32U
#define DISPLAY_REFRESH_TICKS        10U
#define DISPLAY_LINE_COUNT           20U

static uint8 g_display_ready = 0U;
static uint8 g_display_tick = 0U;
static uint8 g_layout_drawn = 0U;
static char g_last_lines[DISPLAY_LINE_COUNT][DISPLAY_LINE_BUF_LEN];

static const char *hw_display_position_state_name(uint8 state)
{
    switch (state)
    {
        case CHASSIS_POSITION_MOVE_EDIT: return "EDIT";
        case CHASSIS_POSITION_MOVE_RUN:  return "RUN";
        case CHASSIS_POSITION_MOVE_DONE: return "DONE";
        default:                         return "IDLE";
    }
}

static const char *hw_display_wheel_name(uint8 wheel_index)
{
    switch (wheel_index)
    {
        case 0U: return "LF";
        case 1U: return "RF";
        case 2U: return "LB";
        case 3U: return "RB";
        default: return "??";
    }
}

static void hw_display_pad_line(char *line)
{
    uint8 len;

    if (line == 0)
    {
        return;
    }

    len = (uint8)strlen(line);
    if (len > DISPLAY_TEXT_COLS)
    {
        len = DISPLAY_TEXT_COLS;
        line[len] = '\0';
    }
    while ((len < DISPLAY_TEXT_COLS) && (len < (DISPLAY_LINE_BUF_LEN - 1U)))
    {
        line[len++] = ' ';
    }
    line[len] = '\0';
}

static void hw_display_make_line(char *dst, const char *fmt, ...)
{
    va_list args;

    if ((dst == 0) || (fmt == 0))
    {
        return;
    }

    va_start(args, fmt);
    vsnprintf(dst, DISPLAY_LINE_BUF_LEN, fmt, args);
    va_end(args);
    hw_display_pad_line(dst);
}

static void hw_display_draw_line(uint16 line, uint16 color, const char *text)
{
    if ((text == 0) || !g_display_ready)
    {
        return;
    }

    tft180_set_color(color, RGB565_BLACK);
    tft180_show_string(0, (uint16)(line * DISPLAY_LINE_H), text);
}

static uint16 hw_display_line_color(uint8 line, const chassis_status_t *status)
{
    uint8 wheel_index;

    if (line == 0U)
    {
        return RGB565_CYAN;
    }

    if ((line >= 1U) && (line <= 5U))
    {
        if (status->tune_active != 0U)
        {
            return RGB565_YELLOW;
        }
        if (status->position_move_state == CHASSIS_POSITION_MOVE_RUN)
        {
            return (line == 5U) ? RGB565_CYAN : RGB565_WHITE;
        }
        if (status->position_move_state == CHASSIS_POSITION_MOVE_EDIT)
        {
            return RGB565_YELLOW;
        }
        return RGB565_WHITE;
    }

    if ((line >= 6U) && (line <= 13U))
    {
        wheel_index = (uint8)((line - 6U) / 2U);
        if ((status->tune_active != 0U) && (wheel_index == status->tune_wheel_index))
        {
            return ((line & 1U) == 1U) ? RGB565_YELLOW : RGB565_CYAN;
        }
        return (((line & 1U) == 0U) && (status->wheel_output_saturated[wheel_index] != 0U)) ? RGB565_PINK : RGB565_WHITE;
    }

    if (line == 14U)
    {
        return (status->tune_active != 0U) ? RGB565_YELLOW : RGB565_WHITE;
    }

    return RGB565_WHITE;
}

static void hw_display_draw_layout(void)
{
    if (!g_display_ready)
    {
        return;
    }

    tft180_full(RGB565_BLACK);
    tft180_draw_line(0, 10, 127, 10, RGB565_CYAN);
    tft180_draw_line(0, 26, 127, 26, RGB565_GRAY);
    tft180_draw_line(0, 44, 127, 44, RGB565_GRAY);
    tft180_draw_line(0, 62, 127, 62, RGB565_GRAY);

    memset(g_last_lines, 0, sizeof(g_last_lines));
    g_layout_drawn = 1U;
}

void hw_display_init(void)
{
    tft180_set_dir(TFT180_PORTAIT);
    tft180_init();
    tft180_set_font(TFT180_6X8_FONT);
    tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    g_display_ready = 1U;
    g_display_tick = 0U;
    g_layout_drawn = 0U;
    hw_display_draw_layout();
}

static void hw_display_make_wheel_lines(char *line_a, char *line_b, const chassis_status_t *status, uint8 wheel_index)
{
    const float target = status->wheel_target_mps[wheel_index];
    const float measured = status->wheel_measured_mps[wheel_index];
    const float error = status->wheel_error_mps[wheel_index];
    const int16 duty = (int16)status->motor_duty[wheel_index];
    const char *name = hw_display_wheel_name(wheel_index);
    const char suffix = (status->wheel_output_saturated[wheel_index] != 0U) ? '!' : ' ';

    hw_display_make_line(line_a, "%s T%+.2f M%+.2f", name, target, measured);
    hw_display_make_line(line_b, "E%+.2f D%+4d%c", error, duty, suffix);
}

void hw_display_update(const chassis_status_t *status)
{
    char lines[DISPLAY_LINE_COUNT][DISPLAY_LINE_BUF_LEN];
    uint8 i;

    if (!g_display_ready || (status == 0))
    {
        return;
    }

    if (!g_layout_drawn)
    {
        hw_display_draw_layout();
    }

    if (++g_display_tick < DISPLAY_REFRESH_TICKS)
    {
        return;
    }
    g_display_tick = 0U;

    memset(lines, 0, sizeof(lines));

    hw_display_make_line(lines[0], "PID+POS");
    hw_display_make_line(lines[1], "Y:%+5.1f E:%+5.1f", status->pose_yaw_deg, status->yaw_error_deg);
    hw_display_make_line(lines[2], "DXm:%+5.2f", status->position_edit_dx_m);
    hw_display_make_line(lines[3], "DYm:%+5.2f", status->position_edit_dy_m);
    hw_display_make_line(lines[4], "TXm:%+5.2f", status->target_x_m);
    hw_display_make_line(lines[5], "TYm:%+5.2f", status->target_y_m);

    hw_display_make_wheel_lines(lines[6], lines[7], status, 0U);
    hw_display_make_wheel_lines(lines[8], lines[9], status, 1U);
    hw_display_make_wheel_lines(lines[10], lines[11], status, 2U);
    hw_display_make_wheel_lines(lines[12], lines[13], status, 3U);

    for (i = 0U; i < DISPLAY_LINE_COUNT; ++i)
    {
        if ((lines[i][0] != '\0') && (strcmp(lines[i], g_last_lines[i]) != 0))
        {
            hw_display_draw_line(i, hw_display_line_color(i, status), lines[i]);
            strcpy(g_last_lines[i], lines[i]);
        }
    }
}
