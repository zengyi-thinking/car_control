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
#define DISPLAY_USED_LINE_COUNT      5U

#define DISPLAY_CAMERA_W             188U
#define DISPLAY_CAMERA_H             120U
#define DISPLAY_GRAPH_X              2U
#define DISPLAY_GRAPH_Y              44U
#define DISPLAY_GRAPH_W              124U
#define DISPLAY_GRAPH_H              114U
#define DISPLAY_GRAPH_PIXELS         (DISPLAY_GRAPH_W * DISPLAY_GRAPH_H)

static uint8 g_display_ready = 0U;
static uint8 g_display_tick = 0U;
static uint8 g_layout_drawn = 0U;
static char g_last_lines[DISPLAY_USED_LINE_COUNT][DISPLAY_LINE_BUF_LEN];
static uint16 g_graph[DISPLAY_GRAPH_PIXELS];

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

static void hw_display_draw_line_text(uint16 line, uint16 color, const char *text)
{
    if ((text == 0) || !g_display_ready)
    {
        return;
    }

    tft180_set_color(color, RGB565_BLACK);
    tft180_show_string(0, (uint16)(line * DISPLAY_LINE_H), text);
}

static uint16 hw_display_line_color(uint8 line)
{
    switch(line)
    {
        case 0U: return RGB565_CYAN;
        case 1U: return RGB565_RED;
        case 2U: return RGB565_GREEN;
        case 3U: return RGB565_YELLOW;
        case 4U: return RGB565_CYAN;
        default: return RGB565_WHITE;
    }
}

static int16 hw_display_clamp_i16(int16 value, int16 min_value, int16 max_value)
{
    if(value < min_value)
    {
        return min_value;
    }
    if(value > max_value)
    {
        return max_value;
    }
    return value;
}

static void hw_display_graph_put(int16 x, int16 y, uint16 color)
{
    if((x < 0) || (y < 0) || (x >= (int16)DISPLAY_GRAPH_W) || (y >= (int16)DISPLAY_GRAPH_H))
    {
        return;
    }
    g_graph[((uint32)y * (uint32)DISPLAY_GRAPH_W) + (uint32)x] = color;
}

static void hw_display_graph_line(int16 x0, int16 y0, int16 x1, int16 y1, uint16 color)
{
    int16 dx = (x1 >= x0) ? (int16)(x1 - x0) : (int16)(x0 - x1);
    int16 sx = (x0 < x1) ? 1 : -1;
    int16 dy = (y1 >= y0) ? (int16)(y0 - y1) : (int16)(y1 - y0);
    int16 sy = (y0 < y1) ? 1 : -1;
    int16 err = (int16)(dx + dy);

    for(;;)
    {
        int16 e2;

        hw_display_graph_put(x0, y0, color);
        if((x0 == x1) && (y0 == y1))
        {
            break;
        }

        e2 = (int16)(2 * err);
        if(e2 >= dy)
        {
            err = (int16)(err + dy);
            x0 = (int16)(x0 + sx);
        }
        if(e2 <= dx)
        {
            err = (int16)(err + dx);
            y0 = (int16)(y0 + sy);
        }
    }
}

static void hw_display_graph_cross(int16 x, int16 y, int16 half, uint16 color)
{
    hw_display_graph_line((int16)(x - half), y, (int16)(x + half), y, color);
    hw_display_graph_line(x, (int16)(y - half), x, (int16)(y + half), color);
}

static int16 hw_display_map_x(int16 camera_x)
{
    int32 x = hw_display_clamp_i16(camera_x, 0, (int16)(DISPLAY_CAMERA_W - 1U));
    x = (x * (int32)(DISPLAY_GRAPH_W - 1U)) / (int32)(DISPLAY_CAMERA_W - 1U);
    return (int16)x;
}

static int16 hw_display_map_y(int16 camera_y)
{
    int32 y = hw_display_clamp_i16(camera_y, 0, (int16)(DISPLAY_CAMERA_H - 1U));
    y = (y * (int32)(DISPLAY_GRAPH_H - 1U)) / (int32)(DISPLAY_CAMERA_H - 1U);
    return (int16)y;
}

static void hw_display_graph_clear(void)
{
    for(uint32 i = 0U; i < (uint32)DISPLAY_GRAPH_PIXELS; ++i)
    {
        g_graph[i] = RGB565_BLACK;
    }
}

static void hw_display_graph_border(void)
{
    const int16 x0 = 0;
    const int16 y0 = 0;
    const int16 x1 = (int16)(DISPLAY_GRAPH_W - 1U);
    const int16 y1 = (int16)(DISPLAY_GRAPH_H - 1U);
    const int16 cx = hw_display_map_x((int16)(DISPLAY_CAMERA_W / 2U));
    const int16 cy = hw_display_map_y((int16)(DISPLAY_CAMERA_H / 2U));

    hw_display_graph_line(x0, y0, x1, y0, RGB565_GRAY);
    hw_display_graph_line(x0, y1, x1, y1, RGB565_GRAY);
    hw_display_graph_line(x0, y0, x0, y1, RGB565_GRAY);
    hw_display_graph_line(x1, y0, x1, y1, RGB565_GRAY);

    hw_display_graph_line(2, cy, (int16)(DISPLAY_GRAPH_W - 4U), cy, RGB565_39C5BB);
    hw_display_graph_line(cx, 2, cx, (int16)(DISPLAY_GRAPH_H - 4U), RGB565_39C5BB);
    hw_display_graph_line((int16)(DISPLAY_GRAPH_W - 8U), (int16)(cy - 3), (int16)(DISPLAY_GRAPH_W - 3U), cy, RGB565_39C5BB);
    hw_display_graph_line((int16)(DISPLAY_GRAPH_W - 8U), (int16)(cy + 3), (int16)(DISPLAY_GRAPH_W - 3U), cy, RGB565_39C5BB);
    hw_display_graph_line((int16)(cx - 3), (int16)(DISPLAY_GRAPH_H - 8U), cx, (int16)(DISPLAY_GRAPH_H - 3U), RGB565_39C5BB);
    hw_display_graph_line((int16)(cx + 3), (int16)(DISPLAY_GRAPH_H - 8U), cx, (int16)(DISPLAY_GRAPH_H - 3U), RGB565_39C5BB);
    hw_display_graph_cross(cx, cy, 3, RGB565_WHITE);
}

static void hw_display_graph_target(int16 x, int16 y, uint16 color)
{
    const int16 gx = hw_display_map_x(x);
    const int16 gy = hw_display_map_y(y);

    hw_display_graph_cross(gx, gy, 4, color);
    hw_display_graph_put((int16)(gx - 1), (int16)(gy - 1), color);
    hw_display_graph_put((int16)(gx + 1), (int16)(gy - 1), color);
    hw_display_graph_put((int16)(gx - 1), (int16)(gy + 1), color);
    hw_display_graph_put((int16)(gx + 1), (int16)(gy + 1), color);
}

static void hw_display_draw_graph(const chassis_status_t *status)
{
    const uint8 has_v = ((status->drone_fresh != 0U) &&
                         ((status->drone_flags & PROTO_DRONE_FLAG_HAS_VLAMP) != 0U)) ? 1U : 0U;
    const uint8 has_b = ((status->drone_fresh != 0U) &&
                         ((status->drone_flags & PROTO_DRONE_FLAG_HAS_BEACON) != 0U)) ? 1U : 0U;

    hw_display_graph_clear();
    hw_display_graph_border();

    if((has_v != 0U) && (has_b != 0U))
    {
        hw_display_graph_line(hw_display_map_x(status->drone_v_x_px),
                              hw_display_map_y(status->drone_v_y_px),
                              hw_display_map_x(status->drone_beacon_x_px),
                              hw_display_map_y(status->drone_beacon_y_px),
                              RGB565_YELLOW);
    }
    if(has_b != 0U)
    {
        hw_display_graph_target(status->drone_beacon_x_px, status->drone_beacon_y_px, RGB565_RED);
    }
    if(has_v != 0U)
    {
        hw_display_graph_target(status->drone_v_x_px, status->drone_v_y_px, RGB565_GREEN);
    }

    tft180_show_rgb565_image(DISPLAY_GRAPH_X,
                             DISPLAY_GRAPH_Y,
                             g_graph,
                             DISPLAY_GRAPH_W,
                             DISPLAY_GRAPH_H,
                             DISPLAY_GRAPH_W,
                             DISPLAY_GRAPH_H,
                             0U);
    tft180_set_color(RGB565_CYAN, RGB565_BLACK);
    tft180_show_string((uint16)(DISPLAY_GRAPH_X + DISPLAY_GRAPH_W - 15U),
                       (uint16)(DISPLAY_GRAPH_Y + hw_display_map_y((int16)(DISPLAY_CAMERA_H / 2U)) - 8U),
                       "+X");
    tft180_show_string((uint16)(DISPLAY_GRAPH_X + hw_display_map_x((int16)(DISPLAY_CAMERA_W / 2U)) + 3U),
                       (uint16)(DISPLAY_GRAPH_Y + DISPLAY_GRAPH_H - 10U),
                       "+Y");
}

static void hw_display_draw_layout(void)
{
    if (!g_display_ready)
    {
        return;
    }

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
    const uint8 has_v = ((status != 0) && (status->drone_fresh != 0U) &&
                         ((status->drone_flags & PROTO_DRONE_FLAG_HAS_VLAMP) != 0U)) ? 1U : 0U;
    const uint8 has_b = ((status != 0) && (status->drone_fresh != 0U) &&
                         ((status->drone_flags & PROTO_DRONE_FLAG_HAS_BEACON) != 0U)) ? 1U : 0U;

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

    if(has_b != 0U)
    {
        hw_display_make_line(lines[1], "B F1 X%03d Y%03d C%03u",
                             (int)status->drone_beacon_x_px,
                             (int)status->drone_beacon_y_px,
                             (unsigned int)status->drone_beacon_conf_permille);
    }
    else
    {
        hw_display_make_line(lines[1], "B F0 X--- Y--- C---");
    }

    if(has_v != 0U)
    {
        hw_display_make_line(lines[2], "V F1 X%03d Y%03d C%03u",
                             (int)status->drone_v_x_px,
                             (int)status->drone_v_y_px,
                             (unsigned int)status->drone_v_conf_permille);
    }
    else
    {
        hw_display_make_line(lines[2], "V F0 X--- Y--- C---");
    }

    if(status->drone_move_valid != 0U)
    {
        hw_display_make_line(lines[3], "VBX %+4d VBY %+4d",
                             (int)status->drone_vb_x_px,
                             (int)status->drone_vb_y_px);
    }
    else
    {
        hw_display_make_line(lines[3], "VBX ---- VBY ----");
    }

    hw_display_make_line(lines[4], "VX %+1.2f VY %+1.2f",
                         status->cmd_vy_body_mps,
                         status->cmd_vx_body_mps);

    for (i = 0U; i < DISPLAY_USED_LINE_COUNT; ++i)
    {
        if (strcmp(lines[i], g_last_lines[i]) != 0)
        {
            hw_display_draw_line_text(i, hw_display_line_color(i), lines[i]);
            strcpy(g_last_lines[i], lines[i]);
        }
    }

    hw_display_draw_graph(status);
}
