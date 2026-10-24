/* ============================================================================
 * board_accif435.h — конфигурация пинов/AF/UART для платы ACCIF435 (AT32F435G).
 *
 * Единственное место, где описано назначение пинов. Все остальные модули
 * ссылаются на константы из этого файла.
 *
 * ВНИМАНИЕ: номера пинов и AF нужно сверить с реальной схемой платы ACCIF435
 * перед первым включением. Ниже — типичное расположение для quad-контроллера
 * на AT32F435G; помечены TODO там, где требует точной сверки.
 * ============================================================================ */
#pragma once
#include "radun_fix/radun_hw.h"

/* ---------- LEDs ---------- */
#define LED_PORT  GPIOC
#define LED_PIN   PIN_14

/* ---------- IMU BMI270 (SPI1) ---------- */
#define GYRO_SPI   SPI1
#define GYRO_CS_PORT  GPIOA
#define GYRO_CS_PIN   PIN_4
/* SPI1 SCK/MISO/MOSI — PB3/PB4/PB5, AF=5 (сверять с платой) */
#define SPI1_SCK_PIN  PIN_3
#define SPI1_SCK_GPIO GPIOB
#define SPI1_MISO_PIN PIN_4
#define SPI1_MISO_GPIO GPIOB
#define SPI1_MOSI_PIN PIN_5
#define SPI1_MOSI_GPIO GPIOB
#define SPI1_AF       5

/* ---------- Baro BMP280 (I2C1) ---------- */
#define BARO_I2C   I2C1
/* I2C1 SCL/SDA — PB6/PB7, AF=4 (сверять с платой) */
#define I2C1_SCL_PIN  PIN_6
#define I2C1_SCL_GPIO GPIOB
#define I2C1_SDA_PIN  PIN_7
#define I2C1_SDA_GPIO GPIOB
#define I2C1_AF       4

/* ---------- CRSF RX (USART6) ---------- */
#define CRSF_USART   USART6
#define CRSF_TX_PORT  GPIOC
#define CRSF_TX_PIN   PIN_7   /* PC7, AF=8 (сверять) */
#define CRSF_RX_PORT  GPIOC
#define CRSF_RX_PIN   PIN_6   /* PC6, AF=8 (сверять) */
#define CRSF_AF       8
#define CRSF_BAUD     420000U

/* ---------- Motors (DShot600) ---------- */
/* MOTOR1: PA09 (TIM1 CH2), MOTOR2: PA08 (TIM1 CH1) */
#define MOTOR1_PORT GPIOA
#define MOTOR1_PIN  PIN_9
#define MOTOR2_PORT GPIOA
#define MOTOR2_PIN  PIN_8
/* MOTOR3: PC09 (TIM8 CH4), MOTOR4: PC08 (TIM8 CH3) — TODO(плата): TMR1 vs TMR8 */
#define MOTOR3_PORT GPIOC
#define MOTOR3_PIN  PIN_9
#define MOTOR4_PORT GPIOC
#define MOTOR4_PIN  PIN_8
#define MOTOR_AF_TMR1 1
#define MOTOR_AF_TMR8 3

/* ---------- ADC (VBAT / current) ---------- */
#define ADC_CH_VBAT   11
#define ADC_CH_CURRENT 12   /* TODO(плата): уточнить канал датчика тока */

/* ---------- Flash (SPI2) ---------- */
#define FLASH_SPI   SPI2
#define FLASH_CS_PORT   GPIOB
#define FLASH_CS_PIN    PIN_12
/* SPI2 SCK/MISO/MOSI — PB13/PB14/PB15, AF=5 (сверять) */
#define SPI2_SCK_PIN  PIN_13
#define SPI2_MISO_PIN PIN_14
#define SPI2_MOSI_PIN PIN_15
#define SPI2_AF       5
