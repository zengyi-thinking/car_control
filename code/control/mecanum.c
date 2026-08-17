/*******************************************************************************
 * 麦轮运动学
 *
 * 轮序号固定为:
 * 0 = 左前 (LF)
 * 1 = 右前 (RF)
 * 2 = 左后 (LB)
 * 3 = 右后 (RB)
 *
 * 坐标约定:
 * forward_y > 0: 前进, 对应小车 +Y
 * right_x > 0: 右移, 对应小车 +X
 * omega > 0: 逆时针
 ******************************************************************************/

#include "control/mecanum.h"
#include "math.h"

void mecanum_inverse(float forward_y, float right_x, float omega, float wheel[4])
{
    float k = MECANUM_GEOMETRY_K_M;

    wheel[0] = forward_y - right_x - omega * k;
    wheel[1] = forward_y + right_x + omega * k;
    wheel[2] = forward_y + right_x - omega * k;
    wheel[3] = forward_y - right_x + omega * k;
}

void mecanum_forward(const float wheel[4], float *forward_y, float *right_x, float *omega)
{
    float k = MECANUM_GEOMETRY_K_M;

    if (forward_y)
    {
        *forward_y = 0.25f * (wheel[0] + wheel[1] + wheel[2] + wheel[3]);
    }

    if (right_x)
    {
        *right_x = 0.25f * (-wheel[0] + wheel[1] + wheel[2] - wheel[3]);
    }

    if (omega)
    {
        if (k > 0.0f)
        {
            *omega = 0.25f * (-wheel[0] + wheel[1] - wheel[2] + wheel[3]) / k;
        }
        else
        {
            *omega = 0.0f;
        }
    }
}

void mecanum_desaturate(float wheel[4], float max_abs_speed)
{
    float max_found = 0.0f;
    float scale;
    uint8 i;

    if ((wheel == 0) || (max_abs_speed <= 0.0f))
    {
        return;
    }

    for (i = 0; i < 4; i++)
    {
        float abs_value = fabsf(wheel[i]);
        if (abs_value > max_found)
        {
            max_found = abs_value;
        }
    }

    if (max_found <= max_abs_speed)
    {
        return;
    }

    scale = max_abs_speed / max_found;
    for (i = 0; i < 4; i++)
    {
        wheel[i] *= scale;
    }
}

void mecanum_solve(float forward_y, float right_x, float omega, float wheel[4])
{
    mecanum_inverse(forward_y, right_x, omega, wheel);
}
