#ifndef HW_BOARD_PINS_H
#define HW_BOARD_PINS_H

#include "zf_common_headfile.h"

#define HW_WHEEL_COUNT                      4U

/*
 * Unified wheel order for the whole car project:
 *   0 = LF  (left-front)
 *   1 = RF  (right-front)
 *   2 = LB  (left-back)
 *   3 = RB  (right-back)
 *
 * Encoder mapping verified by hand-spin:
 *   LF = encoder4
 *   RF = encoder3
 *   LB = encoder1
 *   RB = encoder2
 *
 * Motor mapping is aligned to the observed strafe behavior:
 *   LF = front-left motor channel
 *   RF = front-right motor channel
 *   LB = left-back motor channel
 *   RB = right-back motor channel
 *
 * The current bring-up keeps abstract wheel-positive direction aligned with
 * the user's mecanum table directly in the kinematic layer, so all motor/encoder
 * signs stay positive here. If a single wheel later proves electrically reversed,
 * only that wheel's sign should be changed here.
 */

/* =========================
 * DRV8701E brushed motors
 * ========================= */
#define HW_MOTOR0_DIR_PIN                  P05_1
#define HW_MOTOR0_PWM_PIN                  TCPWM_CH09_P05_0
#define HW_MOTOR0_SIGN                     1

#define HW_MOTOR1_DIR_PIN                  P05_3
#define HW_MOTOR1_PWM_PIN                  TCPWM_CH11_P05_2
#define HW_MOTOR1_SIGN                     1

#define HW_MOTOR2_DIR_PIN                  P10_3
#define HW_MOTOR2_PWM_PIN                  TCPWM_CH30_P10_2
#define HW_MOTOR2_SIGN                     1

#define HW_MOTOR3_DIR_PIN                  P09_1
#define HW_MOTOR3_PWM_PIN                  TCPWM_CH24_P09_0
#define HW_MOTOR3_SIGN                     1

#define HW_MOTOR_PWM_FREQ_HZ               17000U

/* =========================
 * Quadrature encoders
 * ========================= */
#define HW_ENCODER0_INDEX                  TC_CH20_ENCODER
#define HW_ENCODER0_CH_A                   TC_CH20_ENCODER_CH1_P08_1
#define HW_ENCODER0_CH_B                   TC_CH20_ENCODER_CH2_P08_2
#define HW_ENCODER0_SIGN                   (-1)

#define HW_ENCODER1_INDEX                  TC_CH07_ENCODER
#define HW_ENCODER1_CH_A                   TC_CH07_ENCODER_CH1_P07_6
#define HW_ENCODER1_CH_B                   TC_CH07_ENCODER_CH2_P07_7
#define HW_ENCODER1_SIGN                   1

#define HW_ENCODER2_INDEX                  TC_CH58_ENCODER
#define HW_ENCODER2_CH_A                   TC_CH58_ENCODER_CH1_P17_3
#define HW_ENCODER2_CH_B                   TC_CH58_ENCODER_CH2_P17_4
#define HW_ENCODER2_SIGN                   (-1)

#define HW_ENCODER3_INDEX                  TC_CH27_ENCODER
#define HW_ENCODER3_CH_A                   TC_CH27_ENCODER_CH1_P19_2
#define HW_ENCODER3_CH_B                   TC_CH27_ENCODER_CH2_P19_3
#define HW_ENCODER3_SIGN                   1

/* =========================
 * Communication
 * ========================= */
#define HW_REMOTE_UART                     UART_1
#define HW_REMOTE_UART_BAUD                115200U
#define HW_REMOTE_UART_TX_PIN              UART1_TX_P04_1
#define HW_REMOTE_UART_RX_PIN              UART1_RX_P04_0

#define HW_DRONE_UART                      UART_5
#define HW_DRONE_UART_BAUD                 115200U
#define HW_DRONE_UART_TX_PIN               UART5_TX_P02_1
#define HW_DRONE_UART_RX_PIN               UART5_RX_P02_0

/* =========================
 * Indicator
 * ========================= */
#define HW_INDICATOR_PIN                   P01_1

/* =========================
 * IMU (ICM45686 on SPI2)
 * ========================= */
#define HW_IMU_SPI                         SPI_2
#define HW_IMU_SPI_CLK_PIN                 SPI2_CLK_P15_2
#define HW_IMU_SPI_MOSI_PIN                SPI2_MOSI_P15_1
#define HW_IMU_SPI_MISO_PIN                SPI2_MISO_P15_0
#define HW_IMU_CS_PIN                      P15_3
#define HW_IMU_DRDY_PIN                    P06_7

#endif
