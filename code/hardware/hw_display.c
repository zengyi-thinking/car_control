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
#define DISPLAY_USED_LINE_COUNT      6U

static uint8 g_display_ready = 0U;
static uint8 g_display_tick = 0U;
static uint8 g_layout_drawn = 0U;
static char g_last_lines[DISPLAY_USED_LINE_COUNT][DISPLAY_LINE_BUF_LEN];

static const char *hw_display_mode_name(chassis_mode_t mode)
{
    switch (mode)
    {
        case CHASSIS_MODE_SAFE_STOP: return "STOP";
        case CHASSIS_MODE_MANUAL: return "MAN";
        case CHASSIS_MODE_WHEEL_TUNE: return "TUNE";
        case CHASSIS_MODE_DRONE_TRACK: return "DRON";
        default: return "UNK";
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

    return RGB565_WHITE;
}

static void hw_display_draw_layout(void)
{
    if (!g_display_ready)
    {
        return;
    }

    /* 清屏并重置差分缓冲。不再画分隔横线——原横线坐标(10/26/44/62)不是
     * 行高(8)的倍数，会穿过并遮挡文字行，导致数据显示残缺/看不见。
     * 当前屏幕只保留无人机 A4 链路和小车实际速度命令。 */
    tft180_full(RGB565_BLACK);

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

void hw_display_update(const chassis_status_t *status)
{
    char lines[DISPLAY_USED_LINE_COUNT][DISPLAY_LINE_BUF_LEN];
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

    hw_display_make_line(lines[0], "%s %s",
                         hw_display_mode_name(status->mode),
                         ((status->mode == CHASSIS_MODE_DRONE_TRACK) || (status->drone_fresh != 0U)) ? "A4" : "--");
    hw_display_make_line(lines[1], "Y:%+5.1f E:%+5.1f", status->pose_yaw_deg, status->yaw_error_deg);
    /* line2: drone pixel error (BVf/BVr = beacon-vs-Vlamp forward/right).
     * Mirrors the drone LCD line3 labels so values can be cross-checked. */
    if(status->drone_fresh != 0U)
    {
        hw_display_make_line(lines[2], "BVf:%+4d BVr:%+4d",
                             status->drone_err_fwd_px, status->drone_err_right_px);
    }
    else
    {
        hw_display_make_line(lines[2], "BVf:---- BVr:----");
    }
    /* line3: actual body-frame velocity command sent to mecanum inverse. */
    hw_display_make_line(lines[3], "VX:%+.2f VY:%+.2f", status->cmd_vx_body_mps, status->cmd_vy_body_mps);
    /* line4: drone link flags (V=has_vlamp B=has_beacon M=move_valid) + age ms. */
    hw_display_make_line(lines[4], "F:%u%u%u AG:%3u",
                         (uint16)((status->drone_flags & 0x01U) != 0U ? 1U : 0U),
                         (uint16)((status->drone_flags & 0x02U) != 0U ? 1U : 0U),
                         (uint16)((status->drone_flags & 0x04U) != 0U ? 1U : 0U),
                         status->drone_age_ms);
    /* line5: yaw rate command (omega), completes the VX/VY/omega triplet. */
    hw_display_make_line(lines[5], "W:%+5.2f", status->cmd_omega_radps);

    /* 只刷新实际有内容的行。 */
    for (i = 0U; i < DISPLAY_USED_LINE_COUNT; ++i)
    {
        if (strcmp(lines[i], g_last_lines[i]) != 0)
        {
            hw_display_draw_line(i, hw_display_line_color(i, status), lines[i]);
            strcpy(g_last_lines[i], lines[i]);
        }
    }
}
