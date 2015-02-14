/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for STMicroelectronics STM32F4-Discovery board.
 */

/*
 * Board identifier.
 */
#define BOARD_NAME              "Striso v1.1"

/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#define STM32_LSECLK                0
#define STM32_HSECLK                8000000

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   300

/*
 * MCU type as defined in the ST header.
 */
#define STM32F40_41xxx

/*
 * USB VBUSSENS pin is not connected.
 */
#define BOARD_OTG_NOVBUSSENS

/*
 * IO pins assignments.
 */
#define GPIOA_ADC0                  0
#define GPIOA_ADC1                  1
#define GPIOA_ADC2                  2
#define GPIOA_ADC3                  3
#define GPIOA_PIN4                  4
#define GPIOA_PIN5                  5
#define GPIOA_PIN6                  6
#define GPIOA_PIN7                  7
#define GPIOA_I2C3_SCL              8
#define GPIOA_UART1_TX              9
#define GPIOA_UART1_RX              10
#define GPIOA_OTG_FS_DM             11
#define GPIOA_OTG_FS_DP             12
#define GPIOA_SWDIO                 13
#define GPIOA_SWCLK                 14
#define GPIOA_PIN15                 15

#define GPIOB_PIN0                  0
#define GPIOB_PIN1                  1
#define GPIOB_PIN2                  2
#define GPIOB_PIN3                  3
#define GPIOB_PIN4                  4
#define GPIOB_PIN5                  5
#define GPIOB_PIN6                  6
#define GPIOB_PIN7                  7
#define GPIOB_PIN8                  8
#define GPIOB_PIN9                  9
#define GPIOB_PIN10                 10
#define GPIOB_PIN11                 11
#define GPIOB_PIN12                 12
#define GPIOB_PIN13                 13
#define GPIOB_PIN14                 14
#define GPIOB_PIN15                 15

#define GPIOC_ADC10                 0
#define GPIOC_ADC11                 1
#define GPIOC_ADC12                 2
#define GPIOC_ADC13                 3
#define GPIOC_PIN4                  4
#define GPIOC_PIN5                  5
#define GPIOC_PIN6                  6
#define GPIOC_PIN7                  7
#define GPIOC_PIN8                  8
#define GPIOC_I2C3_SDA              9
#define GPIOC_PIN10                 10
#define GPIOC_PIN11                 11
#define GPIOC_PIN12                 12
#define GPIOC_PIN13                 13
#define GPIOC_PIN14                 14
#define GPIOC_PIN15                 15

#define GPIOD_PIN0                  0
#define GPIOD_PIN1                  1
#define GPIOD_PIN2                  2
#define GPIOD_PIN3                  3
#define GPIOD_PIN4                  4
#define GPIOD_PIN5                  5
#define GPIOD_PIN6                  6
#define GPIOD_PIN7                  7
#define GPIOD_PIN8                  8
#define GPIOD_PIN9                  9
#define GPIOD_PIN10                 10
#define GPIOD_PIN11                 11
#define GPIOD_PIN12                 12
#define GPIOD_PIN13                 13
#define GPIOD_PIN14                 14
#define GPIOD_PIN15                 15

#define GPIOE_PIN0                  0
#define GPIOE_PIN1                  1
#define GPIOE_PIN2                  2
#define GPIOE_PIN3                  3
#define GPIOE_PIN4                  4
#define GPIOE_PIN5                  5
#define GPIOE_PIN6                  6
#define GPIOE_PIN7                  7
#define GPIOE_PIN8                  8
#define GPIOE_PIN9                  9
#define GPIOE_PIN10                 10
#define GPIOE_PIN11                 11
#define GPIOE_PIN12                 12
#define GPIOE_PIN13                 13
#define GPIOE_PIN14                 14
#define GPIOE_PIN15                 15

#define GPIOF_PIN0                  0
#define GPIOF_PIN1                  1
#define GPIOF_PIN2                  2
#define GPIOF_PIN3                  3
#define GPIOF_PIN4                  4
#define GPIOF_PIN5                  5
#define GPIOF_PIN6                  6
#define GPIOF_PIN7                  7
#define GPIOF_PIN8                  8
#define GPIOF_PIN9                  9
#define GPIOF_PIN10                 10
#define GPIOF_PIN11                 11
#define GPIOF_PIN12                 12
#define GPIOF_PIN13                 13
#define GPIOF_PIN14                 14
#define GPIOF_PIN15                 15

#define GPIOG_PIN0                  0
#define GPIOG_PIN1                  1
#define GPIOG_PIN2                  2
#define GPIOG_PIN3                  3
#define GPIOG_PIN4                  4
#define GPIOG_PIN5                  5
#define GPIOG_PIN6                  6
#define GPIOG_PIN7                  7
#define GPIOG_PIN8                  8
#define GPIOG_PIN9                  9
#define GPIOG_PIN10                 10
#define GPIOG_PIN11                 11
#define GPIOG_PIN12                 12
#define GPIOG_PIN13                 13
#define GPIOG_PIN14                 14
#define GPIOG_PIN15                 15

#define GPIOH_OSC_IN                0
#define GPIOH_OSC_OUT               1
#define GPIOH_PIN2                  2
#define GPIOH_PIN3                  3
#define GPIOH_PIN4                  4
#define GPIOH_PIN5                  5
#define GPIOH_PIN6                  6
#define GPIOH_PIN7                  7
#define GPIOH_PIN8                  8
#define GPIOH_PIN9                  9
#define GPIOH_PIN10                 10
#define GPIOH_PIN11                 11
#define GPIOH_PIN12                 12
#define GPIOH_PIN13                 13
#define GPIOH_PIN14                 14
#define GPIOH_LED1                  15

#define GPIOI_PIN0                  0
#define GPIOI_PIN1                  1
#define GPIOI_PIN2                  2
#define GPIOI_PIN3                  3
#define GPIOI_PIN4                  4
#define GPIOI_PIN5                  5
#define GPIOI_PIN6                  6
#define GPIOI_PIN7                  7
#define GPIOI_PIN8                  8
#define GPIOI_PIN9                  9
#define GPIOI_PIN10                 10
#define GPIOI_PIN11                 11
#define GPIOI_PIN12                 12
#define GPIOI_PIN13                 13
#define GPIOI_PIN14                 14
#define GPIOI_PIN15                 15

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_2M(n)            (0U << ((n) * 2))
#define PIN_OSPEED_25M(n)           (1U << ((n) * 2))
#define PIN_OSPEED_50M(n)           (2U << ((n) * 2))
#define PIN_OSPEED_100M(n)          (3U << ((n) * 2))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2))
#define PIN_AFIO_AF(n, v)           ((v##U) << ((n % 8) * 4))

/*
 * GPIOA setup:
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_ADC0) | \
                                     PIN_MODE_INPUT(GPIOA_ADC1) | \
                                     PIN_MODE_INPUT(GPIOA_ADC2) | \
                                     PIN_MODE_INPUT(GPIOA_ADC3) | \
                                     PIN_MODE_INPUT(GPIOA_PIN4) | \
                                     PIN_MODE_INPUT(GPIOA_PIN5) | \
                                     PIN_MODE_INPUT(GPIOA_PIN6) | \
                                     PIN_MODE_INPUT(GPIOA_PIN7) | \
                                     PIN_MODE_ALTERNATE(GPIOA_I2C3_SCL) | \
                                     PIN_MODE_ALTERNATE(GPIOA_UART1_TX) | \
                                     PIN_MODE_ALTERNATE(GPIOA_UART1_RX) | \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DM) | \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DP) | \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) | \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) | \
                                     PIN_MODE_INPUT(GPIOA_PIN15))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_ADC0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ADC1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ADC2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ADC3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN4) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN5) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN6) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN7) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_I2C3_SCL) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART1_TX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART1_RX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DM) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DP) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN15))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_100M(GPIOA_ADC0) | \
                                     PIN_OSPEED_100M(GPIOA_ADC1) | \
                                     PIN_OSPEED_100M(GPIOA_ADC2) | \
                                     PIN_OSPEED_100M(GPIOA_ADC3) | \
                                     PIN_OSPEED_100M(GPIOA_PIN4) | \
                                     PIN_OSPEED_100M(GPIOA_PIN5) | \
                                     PIN_OSPEED_100M(GPIOA_PIN6) | \
                                     PIN_OSPEED_100M(GPIOA_PIN7) | \
                                     PIN_OSPEED_100M(GPIOA_I2C3_SCL) | \
                                     PIN_OSPEED_100M(GPIOA_UART1_TX) | \
                                     PIN_OSPEED_100M(GPIOA_UART1_RX) | \
                                     PIN_OSPEED_100M(GPIOA_OTG_FS_DM) | \
                                     PIN_OSPEED_100M(GPIOA_OTG_FS_DP) | \
                                     PIN_OSPEED_100M(GPIOA_SWDIO) | \
                                     PIN_OSPEED_100M(GPIOA_SWCLK) | \
                                     PIN_OSPEED_100M(GPIOA_PIN15))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_ADC0) | \
                                     PIN_PUPDR_FLOATING(GPIOA_ADC1) | \
                                     PIN_PUPDR_FLOATING(GPIOA_ADC2) | \
                                     PIN_PUPDR_FLOATING(GPIOA_ADC3) | \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN4) | \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN5) | \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN6) | \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN7) | \
                                     PIN_PUPDR_FLOATING(GPIOA_I2C3_SCL) | \
                                     PIN_PUPDR_FLOATING(GPIOA_UART1_TX) | \
                                     PIN_PUPDR_FLOATING(GPIOA_UART1_RX) | \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DM) | \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DP) | \
                                     PIN_PUPDR_FLOATING(GPIOA_SWDIO) | \
                                     PIN_PUPDR_FLOATING(GPIOA_SWCLK) | \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN15))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_ADC0) | \
                                     PIN_ODR_HIGH(GPIOA_ADC1) | \
                                     PIN_ODR_HIGH(GPIOA_ADC2) | \
                                     PIN_ODR_HIGH(GPIOA_ADC3) | \
                                     PIN_ODR_HIGH(GPIOA_PIN4) | \
                                     PIN_ODR_HIGH(GPIOA_PIN5) | \
                                     PIN_ODR_HIGH(GPIOA_PIN6) | \
                                     PIN_ODR_HIGH(GPIOA_PIN7) | \
                                     PIN_ODR_HIGH(GPIOA_I2C3_SCL) | \
                                     PIN_ODR_HIGH(GPIOA_UART1_TX) | \
                                     PIN_ODR_HIGH(GPIOA_UART1_RX) | \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DM) | \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DP) | \
                                     PIN_ODR_HIGH(GPIOA_SWDIO) | \
                                     PIN_ODR_HIGH(GPIOA_SWCLK) | \
                                     PIN_ODR_HIGH(GPIOA_PIN15))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_ADC0, 0) | \
                                     PIN_AFIO_AF(GPIOA_ADC1, 0) | \
                                     PIN_AFIO_AF(GPIOA_ADC2, 0) | \
                                     PIN_AFIO_AF(GPIOA_ADC3, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN4, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN5, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN6, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN7, 0))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_I2C3_SCL, 0) | \
                                     PIN_AFIO_AF(GPIOA_UART1_TX, 7) | \
                                     PIN_AFIO_AF(GPIOA_UART1_RX, 7) | \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DM, 10) | \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DP, 10) | \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0) | \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN15, 0))

/*
 * GPIOB setup:
 */
#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(GPIOB_PIN0) | \
                                     PIN_MODE_INPUT(GPIOB_PIN1) | \
                                     PIN_MODE_INPUT(GPIOB_PIN2) | \
                                     PIN_MODE_INPUT(GPIOB_PIN3) | \
                                     PIN_MODE_INPUT(GPIOB_PIN4) | \
                                     PIN_MODE_INPUT(GPIOB_PIN5) | \
                                     PIN_MODE_INPUT(GPIOB_PIN6) | \
                                     PIN_MODE_INPUT(GPIOB_PIN7) | \
                                     PIN_MODE_INPUT(GPIOB_PIN8) | \
                                     PIN_MODE_INPUT(GPIOB_PIN9) | \
                                     PIN_MODE_INPUT(GPIOB_PIN10) | \
                                     PIN_MODE_INPUT(GPIOB_PIN11) | \
                                     PIN_MODE_INPUT(GPIOB_PIN12) | \
                                     PIN_MODE_INPUT(GPIOB_PIN13) | \
                                     PIN_MODE_INPUT(GPIOB_PIN14) | \
                                     PIN_MODE_INPUT(GPIOB_PIN15))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_PIN0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN4) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN5) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN6) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN7) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN9) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN11) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN12) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN13) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN14) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN15))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_100M(GPIOB_PIN0) | \
                                     PIN_OSPEED_100M(GPIOB_PIN1) | \
                                     PIN_OSPEED_100M(GPIOB_PIN2) | \
                                     PIN_OSPEED_100M(GPIOB_PIN3) | \
                                     PIN_OSPEED_100M(GPIOB_PIN4) | \
                                     PIN_OSPEED_100M(GPIOB_PIN5) | \
                                     PIN_OSPEED_100M(GPIOB_PIN6) | \
                                     PIN_OSPEED_100M(GPIOB_PIN7) | \
                                     PIN_OSPEED_100M(GPIOB_PIN8) | \
                                     PIN_OSPEED_100M(GPIOB_PIN9) | \
                                     PIN_OSPEED_100M(GPIOB_PIN10) | \
                                     PIN_OSPEED_100M(GPIOB_PIN11) | \
                                     PIN_OSPEED_100M(GPIOB_PIN12) | \
                                     PIN_OSPEED_100M(GPIOB_PIN13) | \
                                     PIN_OSPEED_100M(GPIOB_PIN14) | \
                                     PIN_OSPEED_100M(GPIOB_PIN15))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_PIN0) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN1) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN2) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN3) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN4) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN5) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN6) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN7) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN8) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN9) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN10) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN11) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN12) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN13) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN14) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN15))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_PIN0) | \
                                     PIN_ODR_HIGH(GPIOB_PIN1) | \
                                     PIN_ODR_HIGH(GPIOB_PIN2) | \
                                     PIN_ODR_HIGH(GPIOB_PIN3) | \
                                     PIN_ODR_HIGH(GPIOB_PIN4) | \
                                     PIN_ODR_HIGH(GPIOB_PIN5) | \
                                     PIN_ODR_HIGH(GPIOB_PIN6) | \
                                     PIN_ODR_HIGH(GPIOB_PIN7) | \
                                     PIN_ODR_HIGH(GPIOB_PIN8) | \
                                     PIN_ODR_HIGH(GPIOB_PIN9) | \
                                     PIN_ODR_HIGH(GPIOB_PIN10) | \
                                     PIN_ODR_HIGH(GPIOB_PIN11) | \
                                     PIN_ODR_HIGH(GPIOB_PIN12) | \
                                     PIN_ODR_HIGH(GPIOB_PIN13) | \
                                     PIN_ODR_HIGH(GPIOB_PIN14) | \
                                     PIN_ODR_HIGH(GPIOB_PIN15))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_PIN0, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN1, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN2, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN3, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN4, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN5, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN6, 4) | \
                                     PIN_AFIO_AF(GPIOB_PIN7, 0))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_PIN8, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN9, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN10, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN11, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN12, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN13, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN14, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN15, 0))

/*
 * GPIOC setup:
 */
#define VAL_GPIOC_MODER             (PIN_MODE_ANALOG(GPIOC_ADC10) | \
                                     PIN_MODE_ANALOG(GPIOC_ADC11) | \
                                     PIN_MODE_ANALOG(GPIOC_ADC12) | \
                                     PIN_MODE_ANALOG(GPIOC_ADC13) | \
                                     PIN_MODE_INPUT(GPIOC_PIN4) | \
                                     PIN_MODE_INPUT(GPIOC_PIN5) | \
                                     PIN_MODE_INPUT(GPIOC_PIN6) | \
                                     PIN_MODE_INPUT(GPIOC_PIN7) | \
                                     PIN_MODE_INPUT(GPIOC_PIN8) | \
                                     PIN_MODE_INPUT(GPIOC_I2C3_SDA) | \
                                     PIN_MODE_INPUT(GPIOC_PIN10) | \
                                     PIN_MODE_INPUT(GPIOC_PIN11) | \
                                     PIN_MODE_INPUT(GPIOC_PIN12) | \
                                     PIN_MODE_INPUT(GPIOC_PIN13) | \
                                     PIN_MODE_INPUT(GPIOC_PIN14) | \
                                     PIN_MODE_INPUT(GPIOC_PIN15))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_ADC10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ADC11) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ADC12) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ADC13) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN4) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN5) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN6) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN7) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_I2C3_SDA) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN11) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN12) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN13) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN14) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN15))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_100M(GPIOC_ADC10) | \
                                     PIN_OSPEED_100M(GPIOC_ADC11) | \
                                     PIN_OSPEED_100M(GPIOC_ADC12) | \
                                     PIN_OSPEED_100M(GPIOC_ADC13) | \
                                     PIN_OSPEED_100M(GPIOC_PIN4) | \
                                     PIN_OSPEED_100M(GPIOC_PIN5) | \
                                     PIN_OSPEED_100M(GPIOC_PIN6) | \
                                     PIN_OSPEED_100M(GPIOC_PIN7) | \
                                     PIN_OSPEED_100M(GPIOC_PIN8) | \
                                     PIN_OSPEED_100M(GPIOC_I2C3_SDA) | \
                                     PIN_OSPEED_100M(GPIOC_PIN10) | \
                                     PIN_OSPEED_100M(GPIOC_PIN11) | \
                                     PIN_OSPEED_100M(GPIOC_PIN12) | \
                                     PIN_OSPEED_100M(GPIOC_PIN13) | \
                                     PIN_OSPEED_100M(GPIOC_PIN14) | \
                                     PIN_OSPEED_100M(GPIOC_PIN15))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_ADC10) | \
                                     PIN_PUPDR_FLOATING(GPIOC_ADC11) | \
                                     PIN_PUPDR_FLOATING(GPIOC_ADC12) | \
                                     PIN_PUPDR_FLOATING(GPIOC_ADC13) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN4) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN5) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN6) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN7) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN8) | \
                                     PIN_PUPDR_FLOATING(GPIOC_I2C3_SDA) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN10) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN11) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN12) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN13) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN14) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN15))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_ADC10) | \
                                     PIN_ODR_HIGH(GPIOC_ADC11) | \
                                     PIN_ODR_HIGH(GPIOC_ADC12) | \
                                     PIN_ODR_HIGH(GPIOC_ADC13) | \
                                     PIN_ODR_HIGH(GPIOC_PIN4) | \
                                     PIN_ODR_HIGH(GPIOC_PIN5) | \
                                     PIN_ODR_HIGH(GPIOC_PIN6) | \
                                     PIN_ODR_HIGH(GPIOC_PIN7) | \
                                     PIN_ODR_HIGH(GPIOC_PIN8) | \
                                     PIN_ODR_HIGH(GPIOC_I2C3_SDA) | \
                                     PIN_ODR_HIGH(GPIOC_PIN10) | \
                                     PIN_ODR_HIGH(GPIOC_PIN11) | \
                                     PIN_ODR_HIGH(GPIOC_PIN12) | \
                                     PIN_ODR_HIGH(GPIOC_PIN13) | \
                                     PIN_ODR_HIGH(GPIOC_PIN14) | \
                                     PIN_ODR_HIGH(GPIOC_PIN15))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_ADC10, 0) | \
                                     PIN_AFIO_AF(GPIOC_ADC11, 0) | \
                                     PIN_AFIO_AF(GPIOC_ADC12, 0) | \
                                     PIN_AFIO_AF(GPIOC_ADC13, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN4, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN5, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN6, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN7, 6))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_PIN8, 0) | \
                                     PIN_AFIO_AF(GPIOC_I2C3_SDA, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN10, 6) | \
                                     PIN_AFIO_AF(GPIOC_PIN11, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN12, 6) | \
                                     PIN_AFIO_AF(GPIOC_PIN13, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN14, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN15, 0))

/*
 * GPIOD setup:
 */
#define VAL_GPIOD_MODER             (PIN_MODE_INPUT(GPIOD_PIN0) | \
                                     PIN_MODE_INPUT(GPIOD_PIN1) | \
                                     PIN_MODE_INPUT(GPIOD_PIN2) | \
                                     PIN_MODE_INPUT(GPIOD_PIN3) | \
                                     PIN_MODE_INPUT(GPIOD_PIN4) | \
                                     PIN_MODE_INPUT(GPIOD_PIN5) | \
                                     PIN_MODE_INPUT(GPIOD_PIN6) | \
                                     PIN_MODE_INPUT(GPIOD_PIN7) | \
                                     PIN_MODE_INPUT(GPIOD_PIN8) | \
                                     PIN_MODE_INPUT(GPIOD_PIN9) | \
                                     PIN_MODE_INPUT(GPIOD_PIN10) | \
                                     PIN_MODE_INPUT(GPIOD_PIN11) | \
                                     PIN_MODE_INPUT(GPIOD_PIN12) | \
                                     PIN_MODE_INPUT(GPIOD_PIN13) | \
                                     PIN_MODE_INPUT(GPIOD_PIN14) | \
                                     PIN_MODE_INPUT(GPIOD_PIN15))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_PIN0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN4) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN5) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN6) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN7) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN9) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN11) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN12) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN13) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN14) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN15))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_100M(GPIOD_PIN0) | \
                                     PIN_OSPEED_100M(GPIOD_PIN1) | \
                                     PIN_OSPEED_100M(GPIOD_PIN2) | \
                                     PIN_OSPEED_100M(GPIOD_PIN3) | \
                                     PIN_OSPEED_100M(GPIOD_PIN4) | \
                                     PIN_OSPEED_100M(GPIOD_PIN5) | \
                                     PIN_OSPEED_100M(GPIOD_PIN6) | \
                                     PIN_OSPEED_100M(GPIOD_PIN7) | \
                                     PIN_OSPEED_100M(GPIOD_PIN8) | \
                                     PIN_OSPEED_100M(GPIOD_PIN9) | \
                                     PIN_OSPEED_100M(GPIOD_PIN10) | \
                                     PIN_OSPEED_100M(GPIOD_PIN11) | \
                                     PIN_OSPEED_100M(GPIOD_PIN12) | \
                                     PIN_OSPEED_100M(GPIOD_PIN13) | \
                                     PIN_OSPEED_100M(GPIOD_PIN14) | \
                                     PIN_OSPEED_100M(GPIOD_PIN15))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(GPIOD_PIN0) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN1) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN2) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN3) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN4) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN5) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN6) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN7) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN8) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN9) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN10) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN11) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN12) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN13) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN14) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN15))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_PIN0) | \
                                     PIN_ODR_HIGH(GPIOD_PIN1) | \
                                     PIN_ODR_HIGH(GPIOD_PIN2) | \
                                     PIN_ODR_HIGH(GPIOD_PIN3) | \
                                     PIN_ODR_HIGH(GPIOD_PIN4) | \
                                     PIN_ODR_HIGH(GPIOD_PIN5) | \
                                     PIN_ODR_HIGH(GPIOD_PIN6) | \
                                     PIN_ODR_HIGH(GPIOD_PIN7) | \
                                     PIN_ODR_HIGH(GPIOD_PIN8) | \
                                     PIN_ODR_HIGH(GPIOD_PIN9) | \
                                     PIN_ODR_HIGH(GPIOD_PIN10) | \
                                     PIN_ODR_HIGH(GPIOD_PIN11) | \
                                     PIN_ODR_HIGH(GPIOD_PIN12) | \
                                     PIN_ODR_HIGH(GPIOD_PIN13) | \
                                     PIN_ODR_HIGH(GPIOD_PIN14) | \
                                     PIN_ODR_HIGH(GPIOD_PIN15))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_PIN0, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN1, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN2, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN3, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN4, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN5, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN6, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN7, 0))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_PIN8, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN9, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN10, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN11, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN12, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN13, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN14, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN15, 0))

/*
 * GPIOE setup:
 */
#define VAL_GPIOE_MODER             (PIN_MODE_INPUT(GPIOE_PIN0) | \
                                     PIN_MODE_INPUT(GPIOE_PIN1) | \
                                     PIN_MODE_INPUT(GPIOE_PIN2) | \
                                     PIN_MODE_INPUT(GPIOE_PIN3) | \
                                     PIN_MODE_INPUT(GPIOE_PIN4) | \
                                     PIN_MODE_INPUT(GPIOE_PIN5) | \
                                     PIN_MODE_INPUT(GPIOE_PIN6) | \
                                     PIN_MODE_INPUT(GPIOE_PIN7) | \
                                     PIN_MODE_INPUT(GPIOE_PIN8) | \
                                     PIN_MODE_INPUT(GPIOE_PIN9) | \
                                     PIN_MODE_INPUT(GPIOE_PIN10) | \
                                     PIN_MODE_INPUT(GPIOE_PIN11) | \
                                     PIN_MODE_INPUT(GPIOE_PIN12) | \
                                     PIN_MODE_INPUT(GPIOE_PIN13) | \
                                     PIN_MODE_INPUT(GPIOE_PIN14) | \
                                     PIN_MODE_INPUT(GPIOE_PIN15))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_PIN0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN4) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN5) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN6) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN7) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN9) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN11) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN12) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN13) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN14) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN15))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_100M(GPIOE_PIN0) | \
                                     PIN_OSPEED_100M(GPIOE_PIN1) | \
                                     PIN_OSPEED_100M(GPIOE_PIN2) | \
                                     PIN_OSPEED_100M(GPIOE_PIN3) | \
                                     PIN_OSPEED_100M(GPIOE_PIN4) | \
                                     PIN_OSPEED_100M(GPIOE_PIN5) | \
                                     PIN_OSPEED_100M(GPIOE_PIN6) | \
                                     PIN_OSPEED_100M(GPIOE_PIN7) | \
                                     PIN_OSPEED_100M(GPIOE_PIN8) | \
                                     PIN_OSPEED_100M(GPIOE_PIN9) | \
                                     PIN_OSPEED_100M(GPIOE_PIN10) | \
                                     PIN_OSPEED_100M(GPIOE_PIN11) | \
                                     PIN_OSPEED_100M(GPIOE_PIN12) | \
                                     PIN_OSPEED_100M(GPIOE_PIN13) | \
                                     PIN_OSPEED_100M(GPIOE_PIN14) | \
                                     PIN_OSPEED_100M(GPIOE_PIN15))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_FLOATING(GPIOE_PIN0) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN1) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN2) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN3) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN4) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN5) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN6) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN7) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN8) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN9) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN10) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN11) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN12) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN13) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN14) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN15))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_PIN0) | \
                                     PIN_ODR_HIGH(GPIOE_PIN1) | \
                                     PIN_ODR_HIGH(GPIOE_PIN2) | \
                                     PIN_ODR_HIGH(GPIOE_PIN3) | \
                                     PIN_ODR_HIGH(GPIOE_PIN4) | \
                                     PIN_ODR_HIGH(GPIOE_PIN5) | \
                                     PIN_ODR_HIGH(GPIOE_PIN6) | \
                                     PIN_ODR_HIGH(GPIOE_PIN7) | \
                                     PIN_ODR_HIGH(GPIOE_PIN8) | \
                                     PIN_ODR_HIGH(GPIOE_PIN9) | \
                                     PIN_ODR_HIGH(GPIOE_PIN10) | \
                                     PIN_ODR_HIGH(GPIOE_PIN11) | \
                                     PIN_ODR_HIGH(GPIOE_PIN12) | \
                                     PIN_ODR_HIGH(GPIOE_PIN13) | \
                                     PIN_ODR_HIGH(GPIOE_PIN14) | \
                                     PIN_ODR_HIGH(GPIOE_PIN15))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_PIN0, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN1, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN2, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN3, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN4, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN5, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN6, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN7, 0))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_PIN8, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN9, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN10, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN11, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN12, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN13, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN14, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN15, 0))

/*
 * GPIOF setup:
 */
#define VAL_GPIOF_MODER             (PIN_MODE_INPUT(GPIOF_PIN0) | \
                                     PIN_MODE_INPUT(GPIOF_PIN1) | \
                                     PIN_MODE_INPUT(GPIOF_PIN2) | \
                                     PIN_MODE_INPUT(GPIOF_PIN3) | \
                                     PIN_MODE_INPUT(GPIOF_PIN4) | \
                                     PIN_MODE_INPUT(GPIOF_PIN5) | \
                                     PIN_MODE_INPUT(GPIOF_PIN6) | \
                                     PIN_MODE_INPUT(GPIOF_PIN7) | \
                                     PIN_MODE_INPUT(GPIOF_PIN8) | \
                                     PIN_MODE_INPUT(GPIOF_PIN9) | \
                                     PIN_MODE_INPUT(GPIOF_PIN10) | \
                                     PIN_MODE_INPUT(GPIOF_PIN11) | \
                                     PIN_MODE_INPUT(GPIOF_PIN12) | \
                                     PIN_MODE_INPUT(GPIOF_PIN13) | \
                                     PIN_MODE_INPUT(GPIOF_PIN14) | \
                                     PIN_MODE_INPUT(GPIOF_PIN15))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_PIN0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN4) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN5) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN6) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN7) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN9) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN11) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN12) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN13) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN14) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN15))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_100M(GPIOF_PIN0) | \
                                     PIN_OSPEED_100M(GPIOF_PIN1) | \
                                     PIN_OSPEED_100M(GPIOF_PIN2) | \
                                     PIN_OSPEED_100M(GPIOF_PIN3) | \
                                     PIN_OSPEED_100M(GPIOF_PIN4) | \
                                     PIN_OSPEED_100M(GPIOF_PIN5) | \
                                     PIN_OSPEED_100M(GPIOF_PIN6) | \
                                     PIN_OSPEED_100M(GPIOF_PIN7) | \
                                     PIN_OSPEED_100M(GPIOF_PIN8) | \
                                     PIN_OSPEED_100M(GPIOF_PIN9) | \
                                     PIN_OSPEED_100M(GPIOF_PIN10) | \
                                     PIN_OSPEED_100M(GPIOF_PIN11) | \
                                     PIN_OSPEED_100M(GPIOF_PIN12) | \
                                     PIN_OSPEED_100M(GPIOF_PIN13) | \
                                     PIN_OSPEED_100M(GPIOF_PIN14) | \
                                     PIN_OSPEED_100M(GPIOF_PIN15))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_PIN0) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN1) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN2) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN3) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN4) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN5) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN6) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN7) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN8) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN9) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN10) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN11) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN12) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN13) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN14) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN15))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_PIN0) | \
                                     PIN_ODR_HIGH(GPIOF_PIN1) | \
                                     PIN_ODR_HIGH(GPIOF_PIN2) | \
                                     PIN_ODR_HIGH(GPIOF_PIN3) | \
                                     PIN_ODR_HIGH(GPIOF_PIN4) | \
                                     PIN_ODR_HIGH(GPIOF_PIN5) | \
                                     PIN_ODR_HIGH(GPIOF_PIN6) | \
                                     PIN_ODR_HIGH(GPIOF_PIN7) | \
                                     PIN_ODR_HIGH(GPIOF_PIN8) | \
                                     PIN_ODR_HIGH(GPIOF_PIN9) | \
                                     PIN_ODR_HIGH(GPIOF_PIN10) | \
                                     PIN_ODR_HIGH(GPIOF_PIN11) | \
                                     PIN_ODR_HIGH(GPIOF_PIN12) | \
                                     PIN_ODR_HIGH(GPIOF_PIN13) | \
                                     PIN_ODR_HIGH(GPIOF_PIN14) | \
                                     PIN_ODR_HIGH(GPIOF_PIN15))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_PIN0, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN1, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN2, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN3, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN4, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN5, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN6, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN7, 0))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_PIN8, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN9, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN10, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN11, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN12, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN13, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN14, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN15, 0))

/*
 * GPIOG setup:
 */
#define VAL_GPIOG_MODER             (PIN_MODE_INPUT(GPIOG_PIN0) | \
                                     PIN_MODE_INPUT(GPIOG_PIN1) | \
                                     PIN_MODE_INPUT(GPIOG_PIN2) | \
                                     PIN_MODE_INPUT(GPIOG_PIN3) | \
                                     PIN_MODE_INPUT(GPIOG_PIN4) | \
                                     PIN_MODE_INPUT(GPIOG_PIN5) | \
                                     PIN_MODE_INPUT(GPIOG_PIN6) | \
                                     PIN_MODE_INPUT(GPIOG_PIN7) | \
                                     PIN_MODE_INPUT(GPIOG_PIN8) | \
                                     PIN_MODE_INPUT(GPIOG_PIN9) | \
                                     PIN_MODE_INPUT(GPIOG_PIN10) | \
                                     PIN_MODE_INPUT(GPIOG_PIN11) | \
                                     PIN_MODE_INPUT(GPIOG_PIN12) | \
                                     PIN_MODE_INPUT(GPIOG_PIN13) | \
                                     PIN_MODE_INPUT(GPIOG_PIN14) | \
                                     PIN_MODE_INPUT(GPIOG_PIN15))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOG_PIN0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN4) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN5) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN6) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN7) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN9) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN11) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN12) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN13) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN14) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN15))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_100M(GPIOG_PIN0) | \
                                     PIN_OSPEED_100M(GPIOG_PIN1) | \
                                     PIN_OSPEED_100M(GPIOG_PIN2) | \
                                     PIN_OSPEED_100M(GPIOG_PIN3) | \
                                     PIN_OSPEED_100M(GPIOG_PIN4) | \
                                     PIN_OSPEED_100M(GPIOG_PIN5) | \
                                     PIN_OSPEED_100M(GPIOG_PIN6) | \
                                     PIN_OSPEED_100M(GPIOG_PIN7) | \
                                     PIN_OSPEED_100M(GPIOG_PIN8) | \
                                     PIN_OSPEED_100M(GPIOG_PIN9) | \
                                     PIN_OSPEED_100M(GPIOG_PIN10) | \
                                     PIN_OSPEED_100M(GPIOG_PIN11) | \
                                     PIN_OSPEED_100M(GPIOG_PIN12) | \
                                     PIN_OSPEED_100M(GPIOG_PIN13) | \
                                     PIN_OSPEED_100M(GPIOG_PIN14) | \
                                     PIN_OSPEED_100M(GPIOG_PIN15))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_FLOATING(GPIOG_PIN0) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN1) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN2) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN3) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN4) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN5) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN6) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN7) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN8) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN9) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN10) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN11) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN12) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN13) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN14) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN15))
#define VAL_GPIOG_ODR               (PIN_ODR_HIGH(GPIOG_PIN0) | \
                                     PIN_ODR_HIGH(GPIOG_PIN1) | \
                                     PIN_ODR_HIGH(GPIOG_PIN2) | \
                                     PIN_ODR_HIGH(GPIOG_PIN3) | \
                                     PIN_ODR_HIGH(GPIOG_PIN4) | \
                                     PIN_ODR_HIGH(GPIOG_PIN5) | \
                                     PIN_ODR_HIGH(GPIOG_PIN6) | \
                                     PIN_ODR_HIGH(GPIOG_PIN7) | \
                                     PIN_ODR_HIGH(GPIOG_PIN8) | \
                                     PIN_ODR_HIGH(GPIOG_PIN9) | \
                                     PIN_ODR_HIGH(GPIOG_PIN10) | \
                                     PIN_ODR_HIGH(GPIOG_PIN11) | \
                                     PIN_ODR_HIGH(GPIOG_PIN12) | \
                                     PIN_ODR_HIGH(GPIOG_PIN13) | \
                                     PIN_ODR_HIGH(GPIOG_PIN14) | \
                                     PIN_ODR_HIGH(GPIOG_PIN15))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_PIN0, 0) | \
                                     PIN_AFIO_AF(GPIOG_PIN1, 0) | \
                                     PIN_AFIO_AF(GPIOG_PIN2, 0) | \
                                     PIN_AFIO_AF(GPIOG_PIN3, 0) | \
                                     PIN_AFIO_AF(GPIOG_PIN4, 0) | \
                                     PIN_AFIO_AF(GPIOG_PIN5, 0) | \
                                     PIN_AFIO_AF(GPIOG_PIN6, 0) | \
                                     PIN_AFIO_AF(GPIOG_PIN7, 0))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_PIN8, 0) | \
                                     PIN_AFIO_AF(GPIOG_PIN9, 0) | \
                                     PIN_AFIO_AF(GPIOG_PIN10, 0) | \
                                     PIN_AFIO_AF(GPIOG_PIN11, 0) | \
                                     PIN_AFIO_AF(GPIOG_PIN12, 0) | \
                                     PIN_AFIO_AF(GPIOG_PIN13, 0) | \
                                     PIN_AFIO_AF(GPIOG_PIN14, 0) | \
                                     PIN_AFIO_AF(GPIOG_PIN15, 0))

/*
 * GPIOH setup:
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) | \
                                     PIN_MODE_INPUT(GPIOH_OSC_OUT) | \
                                     PIN_MODE_INPUT(GPIOH_PIN2) | \
                                     PIN_MODE_INPUT(GPIOH_PIN3) | \
                                     PIN_MODE_INPUT(GPIOH_PIN4) | \
                                     PIN_MODE_INPUT(GPIOH_PIN5) | \
                                     PIN_MODE_INPUT(GPIOH_PIN6) | \
                                     PIN_MODE_INPUT(GPIOH_PIN7) | \
                                     PIN_MODE_INPUT(GPIOH_PIN8) | \
                                     PIN_MODE_INPUT(GPIOH_PIN9) | \
                                     PIN_MODE_INPUT(GPIOH_PIN10) | \
                                     PIN_MODE_INPUT(GPIOH_PIN11) | \
                                     PIN_MODE_INPUT(GPIOH_PIN12) | \
                                     PIN_MODE_INPUT(GPIOH_PIN13) | \
                                     PIN_MODE_INPUT(GPIOH_PIN14) | \
                                     PIN_MODE_OUTPUT(GPIOH_LED1))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_OSC_IN) | \
                                     PIN_OTYPE_PUSHPULL(GPIOH_OSC_OUT) | \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN4) | \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN5) | \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN6) | \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN7) | \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN9) | \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN11) | \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN12) | \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN13) | \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN14) | \
                                     PIN_OTYPE_PUSHPULL(GPIOH_LED1))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_100M(GPIOH_OSC_IN) | \
                                     PIN_OSPEED_100M(GPIOH_OSC_OUT) | \
                                     PIN_OSPEED_100M(GPIOH_PIN2) | \
                                     PIN_OSPEED_100M(GPIOH_PIN3) | \
                                     PIN_OSPEED_100M(GPIOH_PIN4) | \
                                     PIN_OSPEED_100M(GPIOH_PIN5) | \
                                     PIN_OSPEED_100M(GPIOH_PIN6) | \
                                     PIN_OSPEED_100M(GPIOH_PIN7) | \
                                     PIN_OSPEED_100M(GPIOH_PIN8) | \
                                     PIN_OSPEED_100M(GPIOH_PIN9) | \
                                     PIN_OSPEED_100M(GPIOH_PIN10) | \
                                     PIN_OSPEED_100M(GPIOH_PIN11) | \
                                     PIN_OSPEED_100M(GPIOH_PIN12) | \
                                     PIN_OSPEED_100M(GPIOH_PIN13) | \
                                     PIN_OSPEED_100M(GPIOH_PIN14) | \
                                     PIN_OSPEED_100M(GPIOH_LED1))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_FLOATING(GPIOH_OSC_IN) | \
                                     PIN_PUPDR_FLOATING(GPIOH_OSC_OUT) | \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN2) | \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN3) | \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN4) | \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN5) | \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN6) | \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN7) | \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN8) | \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN9) | \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN10) | \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN11) | \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN12) | \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN13) | \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN14) | \
                                     PIN_PUPDR_FLOATING(GPIOH_LED1))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_OSC_IN) | \
                                     PIN_ODR_HIGH(GPIOH_OSC_OUT) | \
                                     PIN_ODR_HIGH(GPIOH_PIN2) | \
                                     PIN_ODR_HIGH(GPIOH_PIN3) | \
                                     PIN_ODR_HIGH(GPIOH_PIN4) | \
                                     PIN_ODR_HIGH(GPIOH_PIN5) | \
                                     PIN_ODR_HIGH(GPIOH_PIN6) | \
                                     PIN_ODR_HIGH(GPIOH_PIN7) | \
                                     PIN_ODR_HIGH(GPIOH_PIN8) | \
                                     PIN_ODR_HIGH(GPIOH_PIN9) | \
                                     PIN_ODR_HIGH(GPIOH_PIN10) | \
                                     PIN_ODR_HIGH(GPIOH_PIN11) | \
                                     PIN_ODR_HIGH(GPIOH_PIN12) | \
                                     PIN_ODR_HIGH(GPIOH_PIN13) | \
                                     PIN_ODR_HIGH(GPIOH_PIN14) | \
                                     PIN_ODR_LOW(GPIOH_LED1))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_OSC_IN, 0) | \
                                     PIN_AFIO_AF(GPIOH_OSC_OUT, 0) | \
                                     PIN_AFIO_AF(GPIOH_PIN2, 0) | \
                                     PIN_AFIO_AF(GPIOH_PIN3, 0) | \
                                     PIN_AFIO_AF(GPIOH_PIN4, 0) | \
                                     PIN_AFIO_AF(GPIOH_PIN5, 0) | \
                                     PIN_AFIO_AF(GPIOH_PIN6, 0) | \
                                     PIN_AFIO_AF(GPIOH_PIN7, 0))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_PIN8, 0) | \
                                     PIN_AFIO_AF(GPIOH_PIN9, 0) | \
                                     PIN_AFIO_AF(GPIOH_PIN10, 0) | \
                                     PIN_AFIO_AF(GPIOH_PIN11, 0) | \
                                     PIN_AFIO_AF(GPIOH_PIN12, 0) | \
                                     PIN_AFIO_AF(GPIOH_PIN13, 0) | \
                                     PIN_AFIO_AF(GPIOH_PIN14, 0) | \
                                     PIN_AFIO_AF(GPIOH_LED1, 0))

/*
 * GPIOI setup:
 */
#define VAL_GPIOI_MODER             (PIN_MODE_INPUT(GPIOI_PIN0) | \
                                     PIN_MODE_INPUT(GPIOI_PIN1) | \
                                     PIN_MODE_INPUT(GPIOI_PIN2) | \
                                     PIN_MODE_INPUT(GPIOI_PIN3) | \
                                     PIN_MODE_INPUT(GPIOI_PIN4) | \
                                     PIN_MODE_INPUT(GPIOI_PIN5) | \
                                     PIN_MODE_INPUT(GPIOI_PIN6) | \
                                     PIN_MODE_INPUT(GPIOI_PIN7) | \
                                     PIN_MODE_INPUT(GPIOI_PIN8) | \
                                     PIN_MODE_INPUT(GPIOI_PIN9) | \
                                     PIN_MODE_INPUT(GPIOI_PIN10) | \
                                     PIN_MODE_INPUT(GPIOI_PIN11) | \
                                     PIN_MODE_INPUT(GPIOI_PIN12) | \
                                     PIN_MODE_INPUT(GPIOI_PIN13) | \
                                     PIN_MODE_INPUT(GPIOI_PIN14) | \
                                     PIN_MODE_INPUT(GPIOI_PIN15))
#define VAL_GPIOI_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOI_PIN0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN4) | \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN5) | \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN6) | \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN7) | \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN9) | \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN11) | \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN12) | \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN13) | \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN14) | \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN15))
#define VAL_GPIOI_OSPEEDR           (PIN_OSPEED_100M(GPIOI_PIN0) | \
                                     PIN_OSPEED_100M(GPIOI_PIN1) | \
                                     PIN_OSPEED_100M(GPIOI_PIN2) | \
                                     PIN_OSPEED_100M(GPIOI_PIN3) | \
                                     PIN_OSPEED_100M(GPIOI_PIN4) | \
                                     PIN_OSPEED_100M(GPIOI_PIN5) | \
                                     PIN_OSPEED_100M(GPIOI_PIN6) | \
                                     PIN_OSPEED_100M(GPIOI_PIN7) | \
                                     PIN_OSPEED_100M(GPIOI_PIN8) | \
                                     PIN_OSPEED_100M(GPIOI_PIN9) | \
                                     PIN_OSPEED_100M(GPIOI_PIN10) | \
                                     PIN_OSPEED_100M(GPIOI_PIN11) | \
                                     PIN_OSPEED_100M(GPIOI_PIN12) | \
                                     PIN_OSPEED_100M(GPIOI_PIN13) | \
                                     PIN_OSPEED_100M(GPIOI_PIN14) | \
                                     PIN_OSPEED_100M(GPIOI_PIN15))
#define VAL_GPIOI_PUPDR             (PIN_PUPDR_FLOATING(GPIOI_PIN0) | \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN1) | \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN2) | \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN3) | \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN4) | \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN5) | \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN6) | \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN7) | \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN8) | \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN9) | \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN10) | \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN11) | \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN12) | \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN13) | \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN14) | \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN15))
#define VAL_GPIOI_ODR               (PIN_ODR_HIGH(GPIOI_PIN0) | \
                                     PIN_ODR_HIGH(GPIOI_PIN1) | \
                                     PIN_ODR_HIGH(GPIOI_PIN2) | \
                                     PIN_ODR_HIGH(GPIOI_PIN3) | \
                                     PIN_ODR_HIGH(GPIOI_PIN4) | \
                                     PIN_ODR_HIGH(GPIOI_PIN5) | \
                                     PIN_ODR_HIGH(GPIOI_PIN6) | \
                                     PIN_ODR_HIGH(GPIOI_PIN7) | \
                                     PIN_ODR_HIGH(GPIOI_PIN8) | \
                                     PIN_ODR_HIGH(GPIOI_PIN9) | \
                                     PIN_ODR_HIGH(GPIOI_PIN10) | \
                                     PIN_ODR_HIGH(GPIOI_PIN11) | \
                                     PIN_ODR_HIGH(GPIOI_PIN12) | \
                                     PIN_ODR_HIGH(GPIOI_PIN13) | \
                                     PIN_ODR_HIGH(GPIOI_PIN14) | \
                                     PIN_ODR_HIGH(GPIOI_PIN15))
#define VAL_GPIOI_AFRL              (PIN_AFIO_AF(GPIOI_PIN0, 0) | \
                                     PIN_AFIO_AF(GPIOI_PIN1, 0) | \
                                     PIN_AFIO_AF(GPIOI_PIN2, 0) | \
                                     PIN_AFIO_AF(GPIOI_PIN3, 0) | \
                                     PIN_AFIO_AF(GPIOI_PIN4, 0) | \
                                     PIN_AFIO_AF(GPIOI_PIN5, 0) | \
                                     PIN_AFIO_AF(GPIOI_PIN6, 0) | \
                                     PIN_AFIO_AF(GPIOI_PIN7, 0))
#define VAL_GPIOI_AFRH              (PIN_AFIO_AF(GPIOI_PIN8, 0) | \
                                     PIN_AFIO_AF(GPIOI_PIN9, 0) | \
                                     PIN_AFIO_AF(GPIOI_PIN10, 0) | \
                                     PIN_AFIO_AF(GPIOI_PIN11, 0) | \
                                     PIN_AFIO_AF(GPIOI_PIN12, 0) | \
                                     PIN_AFIO_AF(GPIOI_PIN13, 0) | \
                                     PIN_AFIO_AF(GPIOI_PIN14, 0) | \
                                     PIN_AFIO_AF(GPIOI_PIN15, 0))


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
