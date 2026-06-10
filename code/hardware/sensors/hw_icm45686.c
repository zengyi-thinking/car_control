#include "hw_icm45686.h"
#include "../hw_board_pins.h"
#include "../hw_config.h"
#include "inv_imu/inv_imu_driver.h"
#include "zf_common_headfile.h"

//====================================================硬件 SPI 配置====================================================
#define ICM45686_SPI_SPEED          (10 * 1000 * 1000)
#define ICM45686_SPI                (HW_IMU_SPI)
#define ICM45686_SPC_PIN            (HW_IMU_SPI_CLK_PIN)
#define ICM45686_SDI_PIN            (HW_IMU_SPI_MOSI_PIN)
#define ICM45686_SDO_PIN            (HW_IMU_SPI_MISO_PIN)
#define ICM45686_CS_PIN             (HW_IMU_CS_PIN)
#define ICM45686_CS(x)              (gpio_set_level(ICM45686_CS_PIN, (x) ? GPIO_HIGH : GPIO_LOW))
#define ICM45686_INT2_PIN           (HW_IMU_DRDY_PIN)
//====================================================硬件 SPI 配置====================================================

static inv_imu_device_t g_icm45686_dev;
static uint8_t g_icm45686_ready = 0;
static uint8_t g_icm45686_whoami = 0;

static float g_accel_g[3];
static float g_gyro_dps[3];
static float g_temp_degc;

static flight_imu_raw_t g_local_imu_raw;

static int icm45686_read_regs(uint8_t reg, uint8_t *buf, uint32_t len)
{
    uint8_t reg_addr = (uint8_t)(reg & 0x7F) | 0x80;
    ICM45686_CS(0);
    spi_read_8bit_registers(ICM45686_SPI, reg_addr, buf, len);
    ICM45686_CS(1);
    return 0;
}

static int icm45686_write_regs(uint8_t reg, const uint8_t *buf, uint32_t len)
{
    uint8_t reg_addr = (uint8_t)(reg & 0x7F);
    ICM45686_CS(0);
    spi_write_8bit_registers(ICM45686_SPI, reg_addr, (uint8_t *)buf, len);
    ICM45686_CS(1);
    return 0;
}

static int icm45686_setup(void)
{
    int rc = 0;
    inv_imu_int_pin_config_t int_pin_config;
    inv_imu_int_state_t int_config;
    drive_config0_t drive_config0;

    spi_init(ICM45686_SPI, SPI_MODE0, ICM45686_SPI_SPEED,
             ICM45686_SPC_PIN, ICM45686_SDI_PIN, ICM45686_SDO_PIN, SPI_CS_NULL);
    gpio_init(ICM45686_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    g_icm45686_dev.transport.read_reg   = icm45686_read_regs;
    g_icm45686_dev.transport.write_reg  = icm45686_write_regs;
    g_icm45686_dev.transport.sleep_us   = system_delay_us;
    g_icm45686_dev.transport.serif_type = UI_SPI4;

    if (g_icm45686_dev.transport.serif_type == UI_SPI3 || g_icm45686_dev.transport.serif_type == UI_SPI4)
    {
        drive_config0.pads_spi_slew = DRIVE_CONFIG0_PADS_SPI_SLEW_TYP_10NS;
        rc |= inv_imu_write_reg(&g_icm45686_dev, DRIVE_CONFIG0, 1, (uint8_t *)&drive_config0);
        if(rc != 0) return rc;
        system_delay_us(2);
    }

    rc |= inv_imu_get_who_am_i(&g_icm45686_dev, &g_icm45686_whoami);
    if(rc != 0) return rc;
    if(g_icm45686_whoami != INV_IMU_WHOAMI)
    {
        return -1;
    }

    rc |= inv_imu_soft_reset(&g_icm45686_dev);
    system_delay_ms(2);

    int_pin_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
    int_pin_config.int_mode     = INTX_CONFIG2_INTX_MODE_PULSE;
    int_pin_config.int_drive    = INTX_CONFIG2_INTX_DRIVE_PP;
    rc |= inv_imu_set_pin_config_int(&g_icm45686_dev, INV_IMU_INT1, &int_pin_config);

    memset(&int_config, INV_IMU_DISABLE, sizeof(int_config));
    int_config.INV_UI_DRDY = INV_IMU_ENABLE;
    rc |= inv_imu_set_config_int(&g_icm45686_dev, INV_IMU_INT1, &int_config);

    rc |= inv_imu_set_accel_fsr(&g_icm45686_dev, ACCEL_CONFIG0_ACCEL_UI_FS_SEL_8_G);
    rc |= inv_imu_set_gyro_fsr(&g_icm45686_dev, GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS);

    rc |= inv_imu_set_accel_frequency(&g_icm45686_dev, ACCEL_CONFIG0_ACCEL_ODR_800_HZ);
    rc |= inv_imu_set_gyro_frequency(&g_icm45686_dev, GYRO_CONFIG0_GYRO_ODR_800_HZ);

    rc |= inv_imu_set_accel_ln_bw(&g_icm45686_dev, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4);
    rc |= inv_imu_set_gyro_ln_bw(&g_icm45686_dev, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4);

    rc |= inv_imu_select_accel_lp_clk(&g_icm45686_dev, SMC_CONTROL_0_ACCEL_LP_CLK_RCOSC);

    rc |= inv_imu_set_accel_mode(&g_icm45686_dev, PWR_MGMT0_ACCEL_MODE_LN);
    rc |= inv_imu_set_gyro_mode(&g_icm45686_dev, PWR_MGMT0_GYRO_MODE_LN);

    return rc;
}

int hw_icm45686_get_raw(float accel_g[3], float gyro_dps[3], float *temp)
{
    inv_imu_sensor_data_t d;
    int rc = inv_imu_get_register_data(&g_icm45686_dev, &d);
    if (rc != 0) return rc;

    accel_g[0] = (float)(d.accel_data[0] * 8.0f / 32768.0f);
    accel_g[1] = (float)(d.accel_data[1] * 8.0f / 32768.0f);
    accel_g[2] = (float)(d.accel_data[2] * 8.0f / 32768.0f);

    gyro_dps[0] = (float)(d.gyro_data[0] * 2000.0f / 32768.0f);
    gyro_dps[1] = (float)(d.gyro_data[1] * 2000.0f / 32768.0f);
    gyro_dps[2] = (float)(d.gyro_data[2] * 2000.0f / 32768.0f);

    *temp = (float)(25.0f + (d.temp_data / 128.0f));

    return 0;
}

int hw_icm45686_ready(void)
{
    return g_icm45686_ready;
}

void hw_icm45686_init(void)
{
    g_icm45686_ready = (0 == icm45686_setup()) ? 1 : 0;

    g_local_imu_raw.acc_x = 0;
    g_local_imu_raw.acc_y = 0;
    g_local_imu_raw.acc_z = 0;
    g_local_imu_raw.gyro_x = 0;
    g_local_imu_raw.gyro_y = 0;
    g_local_imu_raw.gyro_z = 0;
    g_local_imu_raw.dt = ((float)HW_CONTROL_PERIOD_US) * 0.000001f;
    g_local_imu_raw.valid = 0;
    g_accel_g[0] = 0.0f;
    g_accel_g[1] = 0.0f;
    g_accel_g[2] = 0.0f;
    g_gyro_dps[0] = 0.0f;
    g_gyro_dps[1] = 0.0f;
    g_gyro_dps[2] = 0.0f;
    g_temp_degc = 0.0f;
}

void hw_icm45686_update(void)
{
    int rc;

    if (!g_icm45686_ready) return;

    rc = hw_icm45686_get_raw(g_accel_g, g_gyro_dps, &g_temp_degc);
    if(rc != 0)
    {
        g_accel_g[0] = 0.0f;
        g_accel_g[1] = 0.0f;
        g_accel_g[2] = 0.0f;
        g_gyro_dps[0] = 0.0f;
        g_gyro_dps[1] = 0.0f;
        g_gyro_dps[2] = 0.0f;
        g_local_imu_raw.acc_x = 0;
        g_local_imu_raw.acc_y = 0;
        g_local_imu_raw.acc_z = 0;
        g_local_imu_raw.gyro_x = 0;
        g_local_imu_raw.gyro_y = 0;
        g_local_imu_raw.gyro_z = 0;
        g_local_imu_raw.dt = ((float)HW_CONTROL_PERIOD_US) * 0.000001f;
        g_local_imu_raw.valid = 0;
        return;
    }

    g_local_imu_raw.acc_x = (int)(g_accel_g[0] * 4096.0f);
    g_local_imu_raw.acc_y = (int)(g_accel_g[1] * 4096.0f);
    g_local_imu_raw.acc_z = (int)(g_accel_g[2] * 4096.0f);
    g_local_imu_raw.gyro_x = (int)(g_gyro_dps[0] * 16.384f);
    g_local_imu_raw.gyro_y = (int)(g_gyro_dps[1] * 16.384f);
    g_local_imu_raw.gyro_z = (int)(g_gyro_dps[2] * 16.384f);
    g_local_imu_raw.dt = ((float)HW_CONTROL_PERIOD_US) * 0.000001f;
    g_local_imu_raw.valid = 1;
}

void hw_icm45686_irq_callback(void)
{
    /* 预留给外部数据就绪中断，当前硬件层采用周期轮询读取。 */
}

void hw_icm45686_read(flight_imu_raw_t *imu_raw)
{
    if (imu_raw)
    {
        __disable_irq();
        *imu_raw = g_local_imu_raw;
        __enable_irq();
    }
}

void hw_icm45686_read_float(float accel_g[3], float gyro_dps[3])
{
    __disable_irq();
    if(!g_local_imu_raw.valid)
    {
        if(accel_g)
        {
            accel_g[0] = 0.0f;
            accel_g[1] = 0.0f;
            accel_g[2] = 0.0f;
        }
        if(gyro_dps)
        {
            gyro_dps[0] = 0.0f;
            gyro_dps[1] = 0.0f;
            gyro_dps[2] = 0.0f;
        }
        __enable_irq();
        return;
    }

    if(accel_g)
    {
        accel_g[0] = g_accel_g[0];
        accel_g[1] = g_accel_g[1];
        accel_g[2] = g_accel_g[2];
    }
    if(gyro_dps)
    {
        gyro_dps[0] = g_gyro_dps[0];
        gyro_dps[1] = g_gyro_dps[1];
        gyro_dps[2] = g_gyro_dps[2];
    }
    __enable_irq();
}

const flight_imu_raw_t *hw_icm45686_get_local_raw(void)
{
    return &g_local_imu_raw;
}
