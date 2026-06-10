#include "hardware/hw_encoder.h"

#include "hardware/hw_board_pins.h"
#include "hardware/hw_config.h"

#define HW_ENCODER_PI                       3.14159265358979323846f

static const encoder_index_enum g_encoder_index[HW_WHEEL_COUNT] =
{
    HW_ENCODER0_INDEX,
    HW_ENCODER1_INDEX,
    HW_ENCODER2_INDEX,
    HW_ENCODER3_INDEX,
};

static const encoder_channel1_enum g_encoder_ch_a[HW_WHEEL_COUNT] =
{
    HW_ENCODER0_CH_A,
    HW_ENCODER1_CH_A,
    HW_ENCODER2_CH_A,
    HW_ENCODER3_CH_A,
};

static const encoder_channel2_enum g_encoder_ch_b[HW_WHEEL_COUNT] =
{
    HW_ENCODER0_CH_B,
    HW_ENCODER1_CH_B,
    HW_ENCODER2_CH_B,
    HW_ENCODER3_CH_B,
};

static const int8 g_encoder_sign[HW_WHEEL_COUNT] =
{
    HW_ENCODER0_SIGN,
    HW_ENCODER1_SIGN,
    HW_ENCODER2_SIGN,
    HW_ENCODER3_SIGN,
};

void hw_encoder_init(void)
{
    for (uint8 i = 0; i < HW_WHEEL_COUNT; ++i)
    {
        encoder_quad_init(g_encoder_index[i], g_encoder_ch_a[i], g_encoder_ch_b[i]);
        encoder_clear_count(g_encoder_index[i]);
        (void)encoder_get_count(g_encoder_index[i]);
        encoder_clear_count(g_encoder_index[i]);
    }
}

void hw_encoder_read_counts(int16 counts[4])
{
    if (counts == 0)
    {
        return;
    }

    for (uint8 i = 0; i < HW_WHEEL_COUNT; ++i)
    {
        counts[i] = (int16)(g_encoder_sign[i] * encoder_get_count(g_encoder_index[i]));
        encoder_clear_count(g_encoder_index[i]);
    }
}

void hw_encoder_counts_to_speed_mps(const int16 counts[4], float dt_s, float speed_mps[4])
{
    float meters_per_count;

    if ((counts == 0) || (speed_mps == 0) || (dt_s <= 0.0f) || (HW_ENCODER_COUNTS_PER_WHEEL_REV <= 0.0f))
    {
        return;
    }

    meters_per_count = (2.0f * HW_ENCODER_PI * HW_WHEEL_RADIUS_M) / HW_ENCODER_COUNTS_PER_WHEEL_REV;
    for (uint8 i = 0; i < HW_WHEEL_COUNT; ++i)
    {
        speed_mps[i] = ((float)counts[i] * meters_per_count) / dt_s;
    }
}
