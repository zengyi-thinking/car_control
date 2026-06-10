#include "hardware/hw_motor_drv8701e.h"

#include "hardware/hw_board_pins.h"
#include "hardware/hw_config.h"

static const gpio_pin_enum g_motor_dir_pin[HW_WHEEL_COUNT] =
{
    HW_MOTOR0_DIR_PIN,
    HW_MOTOR1_DIR_PIN,
    HW_MOTOR2_DIR_PIN,
    HW_MOTOR3_DIR_PIN,
};

static const pwm_channel_enum g_motor_pwm_pin[HW_WHEEL_COUNT] =
{
    HW_MOTOR0_PWM_PIN,
    HW_MOTOR1_PWM_PIN,
    HW_MOTOR2_PWM_PIN,
    HW_MOTOR3_PWM_PIN,
};

static const int8 g_motor_sign[HW_WHEEL_COUNT] =
{
    HW_MOTOR0_SIGN,
    HW_MOTOR1_SIGN,
    HW_MOTOR2_SIGN,
    HW_MOTOR3_SIGN,
};

static int16 hw_motor_clamp_duty(int32 duty)
{
    if (duty > HW_MOTOR_DUTY_MAX)
    {
        return HW_MOTOR_DUTY_MAX;
    }
    if (duty < -HW_MOTOR_DUTY_MAX)
    {
        return -HW_MOTOR_DUTY_MAX;
    }
    return (int16)duty;
}

void hw_motor_drv8701e_init(void)
{
    for (uint8 i = 0; i < HW_WHEEL_COUNT; ++i)
    {
        gpio_init(g_motor_dir_pin[i], GPO, GPIO_HIGH, GPO_PUSH_PULL);
        pwm_init(g_motor_pwm_pin[i], HW_MOTOR_PWM_FREQ_HZ, 0);
    }
}

void hw_motor_drv8701e_set(uint8 motor_id, int16 duty)
{
    int32 signed_duty;
    uint32 pwm_duty;

    if (motor_id >= HW_WHEEL_COUNT)
    {
        return;
    }

    signed_duty = (int32)g_motor_sign[motor_id] * duty;
    duty = hw_motor_clamp_duty(signed_duty);

    if (duty >= 0)
    {
        gpio_set_level(g_motor_dir_pin[motor_id], GPIO_HIGH);
        pwm_duty = (uint32)duty;
    }
    else
    {
        gpio_set_level(g_motor_dir_pin[motor_id], GPIO_LOW);
        pwm_duty = (uint32)(-duty);
    }

    pwm_set_duty(g_motor_pwm_pin[motor_id], pwm_duty);
}

void hw_motor_drv8701e_set_all(const int16 duty[4])
{
    if (duty == 0)
    {
        return;
    }

    for (uint8 i = 0; i < HW_WHEEL_COUNT; ++i)
    {
        hw_motor_drv8701e_set(i, duty[i]);
    }
}

void hw_motor_drv8701e_stop_all(void)
{
    for (uint8 i = 0; i < HW_WHEEL_COUNT; ++i)
    {
        pwm_set_duty(g_motor_pwm_pin[i], 0);
    }
}
