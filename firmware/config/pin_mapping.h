/*
 * REI Drone - Hardware Pin Mapping
 * Rootcastle Engineering & Innovation
 *
 * Pin assignments for Pixhawk 6C / Cube Orange flight controller.
 * Maps physical hardware connections to logical names used
 * throughout the firmware.
 *
 * Author: Batuhan Ayribas, M.Sc.
 */

#ifndef PIN_MAPPING_H
#define PIN_MAPPING_H

/* ---------------------------------------------------------------
 * PWM Output Channels (Motor ESCs and Tilt Servos)
 * --------------------------------------------------------------- */
#define PWM_OUT_MOTOR1              0   /* Front Left motor */
#define PWM_OUT_MOTOR2              1   /* Front Right motor */
#define PWM_OUT_MOTOR3              2   /* Rear Left motor */
#define PWM_OUT_MOTOR4              3   /* Rear Right motor */
#define PWM_OUT_TILT_FL             4   /* Front Left tilt servo */
#define PWM_OUT_TILT_FR             5   /* Front Right tilt servo */
#define PWM_OUT_TILT_RL             6   /* Rear Left tilt servo (if equipped) */
#define PWM_OUT_TILT_RR             7   /* Rear Right tilt servo (if equipped) */
#define PWM_OUT_GIMBAL_PITCH        8   /* Gimbal pitch servo */
#define PWM_OUT_GIMBAL_ROLL         9   /* Gimbal roll servo */

/* ---------------------------------------------------------------
 * UART Interfaces
 * --------------------------------------------------------------- */
#define UART_TELEMETRY              1   /* UART1: 915 MHz telemetry radio */
#define UART_GPS                    2   /* UART2: GPS module */
#define UART_COMPANION              3   /* UART3: Companion computer (optional) */
#define UART_DEBUG                  6   /* UART6: Debug console */

/* ---------------------------------------------------------------
 * SPI Interfaces (IMU)
 * --------------------------------------------------------------- */
#define SPI_IMU_PRIMARY             1   /* SPI1: Primary IMU (ICM-42688-P) */
#define SPI_IMU_SECONDARY           2   /* SPI2: Secondary IMU (ICM-42688-P) */

/* ---------------------------------------------------------------
 * I2C Interfaces
 * --------------------------------------------------------------- */
#define I2C_EXTERNAL                1   /* I2C1: External compass, barometer */
#define I2C_INTERNAL                2   /* I2C2: Internal barometer (MS5611) */

/* ---------------------------------------------------------------
 * ADC Channels
 * --------------------------------------------------------------- */
#define ADC_BATTERY_VOLTAGE         0   /* Battery voltage sense */
#define ADC_BATTERY_CURRENT         1   /* Battery current sense */
#define ADC_5V_RAIL                 2   /* 5V rail monitor */

/* ---------------------------------------------------------------
 * RC Input
 * --------------------------------------------------------------- */
#define RC_INPUT_SBUS               4   /* UART4: SBUS input from receiver */
#define RC_INPUT_PPM                0   /* Timer capture: PPM sum input */

/* ---------------------------------------------------------------
 * LED and Buzzer
 * --------------------------------------------------------------- */
#define LED_STATUS_RED              0
#define LED_STATUS_GREEN            1
#define LED_STATUS_BLUE             2
#define BUZZER_PIN                  3

/* ---------------------------------------------------------------
 * Safety Switch
 * --------------------------------------------------------------- */
#define SAFETY_SWITCH_PIN           4
#define ARM_SWITCH_PIN              5

#endif /* PIN_MAPPING_H */
