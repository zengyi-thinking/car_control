#include "hardware/hw_display.h"

#include "hardware/hw_config.h"
#include "zf_common_headfile.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define DISPLAY_LINE_H               8U
#define DISPLAY_TEXT_COLS            21U
#define DISPLAY_LINE_BUF_LEN         32U
#define DISPLAY_REFRESH_TICKS        10U
#define DISPLAY_LINE_COUNT           20U

static uint8 g_display_ready = 0U;
static uint8 g_display_tick = 0U;
static uint8 g_layout_drawn = 0U;
static char g_last_lines[DISPLAY_LINE_COUNT][DISPLAY_LINE_BUF_LEN];

static const char *hw_display_mode_name(chassis_mode_t mode)
{
    switch (mode)
    {
        case CHASSIS_MODE_MANUAL:     return "MANUAL";
        case CHASSIS_MODE_NAV:        return "NAV";
        case CHASSIS_MODE_WHEEL_TUNE: return "TUNE";
        default:                      return "STOP";
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

static const char *hw_display_tune_param_name(uint8 param_index)
{
    switch (param_index)
    {
        case CHASSIS_TUNE_PARAM_KP: return "KP";
        case CHASSIS_TUNE_PARAM_KI: return "KI";
        case CHASSIS_TUNE_PARAM_KD: return "KD";
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

    if ((line == 1U) || (line == 2U) || (line == 3U))
    {
        return (status->tune_active != 0U) ? RGB565_YELLOW : RGB565_WHITE;
    }

    if ((line >= 5U) && (line <= 12U))
    {
        wheel_index = (uint8)((line - 5U) / 2U);
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
    tft180_draw_line(0, 42, 127, 42, RGB565_GRAY);
    tft180_draw_line(0, 74, 127, 74, RGB565_GRAY);
    tft180_draw_line(0, 106, 127, 106, RGB565_GRAY);

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
    const char *mode_name;
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
    mode_name = hw_display_mode_name(status->mode);

    hw_display_make_line(lines[0], "WHEEL PID CONTROL");
    if (status->startup_lock_done == 0U)
    {
        const float remain_s = (status->startup_lock_elapsed_s < 2.0f) ? (2.0f - status->startup_lock_elapsed_s) : 0.0f;
        hw_display_make_line(lines[1], "MODE:%-5s BOOT:%3.1fs", mode_name, remain_s);
    }
    else
    {
        hw_display_make_line(lines[1], "MODE:%-5s LOCK:OK", mode_name);
    }

    if (status->tune_active != 0U)
    {
        hw_display_make_line(lines[2], "SEL:%s P:%-2s V:%7.2f", hw_display_wheel_name(status->tune_wheel_index), hw_display_tune_param_name(status->tune_param_index), status->tune_selected_value);
        hw_display_make_line(lines[3], "TGT:%+.2f STEP:%5.2f", status->tune_target_mps, status->tune_selected_step);
    }
    else
    {
        hw_display_make_line(lines[2], "Y:%+6.1f TY:%+6.1f", status->pose_yaw_deg, status->target_yaw_deg);
        hw_display_make_line(lines[3], "E:%+5.1f O:%+5.2f", status->yaw_error_deg, status->cmd_omega_radps);
    }

    hw_display_make_wheel_lines(lines[5], lines[6], status, 0U);
    hw_display_make_wheel_lines(lines[7], lines[8], status, 1U);
    hw_display_make_wheel_lines(lines[9], lines[10], status, 2U);
    hw_display_make_wheel_lines(lines[11], lines[12], status, 3U);

    if (status->tune_active != 0U)
    {
        hw_display_make_line(lines[14], "VAL:%7.2f", status->tune_selected_value);
    }
    else
    {
        hw_display_make_line(lines[14], "DUTY CAP:%4u", (uint16)HW_MANUAL_MAX_DUTY);
    }

    for (i = 0U; i < DISPLAY_LINE_COUNT; ++i)
    {
        if ((lines[i][0] != '\0') && (strcmp(lines[i], g_last_lines[i]) != 0))
        {
            hw_display_draw_line(i, hw_display_line_color(i, status), lines[i]);
            strcpy(g_last_lines[i], lines[i]);
        }
    }
}
