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
#define GPIOA_DIS01                 7
#define GPIOA_I2C3_SCL              8
#define GPIOA_UART1_TX              9
#define GPIOA_UART1_RX              10
#define GPIOA_OTG_FS_DM             11
#define GPIOA_OTG_FS_DP             12
#define GPIOA_SWDIO                 13
#define GPIOA_SWCLK                 14
#define GPIOA_BAS01                 15

#define GPIOB_DIS04                 0
#define GPIOB_DIS05                 1
#define GPIOB_PIN2                  2
#define GPIOB_BAS20                 3
#define GPIOB_BAS21                 4
#define GPIOB_BAS22                 5
#define GPIOB_BAS23                 6
#define GPIOB_BAS24                 7
#define GPIOB_BAS25                 8
#define GPIOB_BAS26                 9
#define GPIOB_DIS22                 10
#define GPIOB_DIS23                 11
#define GPIOB_PIN12                 12
#define GPIOB_DIS31                 13
#define GPIOB_DIS32                 14
#define GPIOB_DIS33                 15

#define GPIOC_ADC10                 0
#define GPIOC_ADC11                 1
#define GPIOC_ADC12                 2
#define GPIOC_ADC13                 3
#define GPIOC_DIS02                 4
#define GPIOC_DIS03                 5
#define GPIOC_DIS49                 6
#define GPIOC_DIS50                 7
#define GPIOC_DIS51                 8
#define GPIOC_I2C3_SDA              9
#define GPIOC_BAS02                 10
#define GPIOC_BAS03                 11
#define GPIOC_BAS04                 12
#define GPIOC_PIN13                 13
#define GPIOC_PIN14                 14
#define GPIOC_PIN15                 15

#define GPIOD_BAS05                 0
#define GPIOD_BAS06                 1
#define GPIOD_BAS07                 2
#define GPIOD_BAS08                 3
#define GPIOD_BAS09                 4
#define GPIOD_BAS10                 5
#define GPIOD_BAS11                 6
#define GPIOD_BAS12                 7
#define GPIOD_DIS34                 8
#define GPIOD_DIS35                 9
#define GPIOD_DIS36                 10
#define GPIOD_DIS37                 11
#define GPIOD_DIS38                 12
#define GPIOD_DIS39                 13
#define GPIOD_DIS40                 14
#define GPIOD_DIS41                 15

#define GPIOE_BAS27                 0
#define GPIOE_BAS28                 1
#define GPIOE_BAS33                 2
#define GPIOE_BAS34                 3
#define GPIOE_BAS35                 4
#define GPIOE_BAS36                 5
#define GPIOE_BAS37                 6
#define GPIOE_DIS13                 7
#define GPIOE_DIS14                 8
#define GPIOE_DIS15                 9
#define GPIOE_DIS16                 10
#define GPIOE_DIS17                 11
#define GPIOE_DIS18                 12
#define GPIOE_DIS19                 13
#define GPIOE_DIS20                 14
#define GPIOE_DIS21                 15

#define GPIOF_BAS41                 0
#define GPIOF_BAS42                 1
#define GPIOF_BAS43                 2
#define GPIOF_BAS44                 3
#define GPIOF_BAS45                 4
#define GPIOF_BAS46                 5
#define GPIOF_BAS47                 6
#define GPIOF_BAS48                 7
#define GPIOF_BAS49                 8
#define GPIOF_BAS50                 9
#define GPIOF_BAS51                 10
#define GPIOF_DIS06                 11
#define GPIOF_DIS07                 12
#define GPIOF_DIS08                 13
#define GPIOF_DIS09                 14
#define GPIOF_DIS10                 15

#define GPIOG_DIS11                 0
#define GPIOG_DIS12                 1
#define GPIOG_DIS42                 2
#define GPIOG_DIS43                 3
#define GPIOG_DIS44                 4
#define GPIOG_DIS45                 5
#define GPIOG_DIS46                 6
#define GPIOG_DIS47                 7
#define GPIOG_DIS48                 8
#define GPIOG_BAS13                 9
#define GPIOG_BAS14                 10
#define GPIOG_BAS15                 11
#define GPIOG_BAS16                 12
#define GPIOG_BAS17                 13
#define GPIOG_BAS18                 14
#define GPIOG_BAS19                 15

#define GPIOH_OSC_IN                0
#define GPIOH_OSC_OUT               1
#define GPIOH_PIN2                  2
#define GPIOH_PIN3                  3
#define GPIOH_PIN4                  4
#define GPIOH_PIN5                  5
#define GPIOH_DIS24                 6
#define GPIOH_DIS25                 7
#define GPIOH_DIS26                 8
#define GPIOH_DIS27                 9
#define GPIOH_DIS28                 10
#define GPIOH_DIS29                 11
#define GPIOH_DIS30                 12
#define GPIOH_PIN13                 13
#define GPIOH_PIN14                 14
#define GPIOH_LED1                  15

#define GPIOI_PIN0                  0
#define GPIOI_PIN1                  1
#define GPIOI_PIN2                  2
#define GPIOI_PIN3                  3
#define GPIOI_BAS29                 4
#define GPIOI_BAS30                 5
#define GPIOI_BAS31                 6
#define GPIOI_BAS32                 7
#define GPIOI_PIN8                  8
#define GPIOI_BAS38                 9
#define GPIOI_BAS39                 10
#define GPIOI_BAS40                 11
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

#define PIN_OTYPE_BUTTONOUT PIN_OTYPE_OPENDRAIN

/*
 * GPIOA setup:
 */
#define VAL_GPIOA_MODER             (PIN_MODE_ANALOG(GPIOA_ADC0) | \
                                     PIN_MODE_ANALOG(GPIOA_ADC1) | \
                                     PIN_MODE_ANALOG(GPIOA_ADC2) | \
                                     PIN_MODE_ANALOG(GPIOA_ADC3) | \
                                     PIN_MODE_INPUT(GPIOA_PIN4) | \
                                     PIN_MODE_INPUT(GPIOA_PIN5) | \
                                     PIN_MODE_INPUT(GPIOA_PIN6) | \
                                     PIN_MODE_OUTPUT(GPIOA_DIS01) | \
                                     PIN_MODE_ALTERNATE(GPIOA_I2C3_SCL) | \
                                     PIN_MODE_ALTERNATE(GPIOA_UART1_TX) | \
                                     PIN_MODE_ALTERNATE(GPIOA_UART1_RX) | \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DM) | \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DP) | \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) | \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) | \
                                     PIN_MODE_OUTPUT(GPIOA_BAS01))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_ADC0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ADC1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ADC2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ADC3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN4) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN5) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN6) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOA_DIS01) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOA_I2C3_SCL) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART1_TX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART1_RX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DM) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DP) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOA_BAS01))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_100M(GPIOA_ADC0) | \
                                     PIN_OSPEED_100M(GPIOA_ADC1) | \
                                     PIN_OSPEED_100M(GPIOA_ADC2) | \
                                     PIN_OSPEED_100M(GPIOA_ADC3) | \
                                     PIN_OSPEED_100M(GPIOA_PIN4) | \
                                     PIN_OSPEED_100M(GPIOA_PIN5) | \
                                     PIN_OSPEED_100M(GPIOA_PIN6) | \
                                     PIN_OSPEED_100M(GPIOA_DIS01) | \
                                     PIN_OSPEED_100M(GPIOA_I2C3_SCL) | \
                                     PIN_OSPEED_100M(GPIOA_UART1_TX) | \
                                     PIN_OSPEED_100M(GPIOA_UART1_RX) | \
                                     PIN_OSPEED_100M(GPIOA_OTG_FS_DM) | \
                                     PIN_OSPEED_100M(GPIOA_OTG_FS_DP) | \
                                     PIN_OSPEED_100M(GPIOA_SWDIO) | \
                                     PIN_OSPEED_100M(GPIOA_SWCLK) | \
                                     PIN_OSPEED_100M(GPIOA_BAS01))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_ADC0) | \
                                     PIN_PUPDR_FLOATING(GPIOA_ADC1) | \
                                     PIN_PUPDR_FLOATING(GPIOA_ADC2) | \
                                     PIN_PUPDR_FLOATING(GPIOA_ADC3) | \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN4) | \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN5) | \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN6) | \
                                     PIN_PUPDR_FLOATING(GPIOA_DIS01) | \
                                     PIN_PUPDR_FLOATING(GPIOA_I2C3_SCL) | \
                                     PIN_PUPDR_FLOATING(GPIOA_UART1_TX) | \
                                     PIN_PUPDR_FLOATING(GPIOA_UART1_RX) | \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DM) | \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DP) | \
                                     PIN_PUPDR_FLOATING(GPIOA_SWDIO) | \
                                     PIN_PUPDR_FLOATING(GPIOA_SWCLK) | \
                                     PIN_PUPDR_FLOATING(GPIOA_BAS01))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_ADC0) | \
                                     PIN_ODR_HIGH(GPIOA_ADC1) | \
                                     PIN_ODR_HIGH(GPIOA_ADC2) | \
                                     PIN_ODR_HIGH(GPIOA_ADC3) | \
                                     PIN_ODR_HIGH(GPIOA_PIN4) | \
                                     PIN_ODR_HIGH(GPIOA_PIN5) | \
                                     PIN_ODR_HIGH(GPIOA_PIN6) | \
                                     PIN_ODR_HIGH(GPIOA_DIS01) | \
                                     PIN_ODR_HIGH(GPIOA_I2C3_SCL) | \
                                     PIN_ODR_HIGH(GPIOA_UART1_TX) | \
                                     PIN_ODR_HIGH(GPIOA_UART1_RX) | \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DM) | \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DP) | \
                                     PIN_ODR_HIGH(GPIOA_SWDIO) | \
                                     PIN_ODR_HIGH(GPIOA_SWCLK) | \
                                     PIN_ODR_HIGH(GPIOA_BAS01))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_ADC0, 0) | \
                                     PIN_AFIO_AF(GPIOA_ADC1, 0) | \
                                     PIN_AFIO_AF(GPIOA_ADC2, 0) | \
                                     PIN_AFIO_AF(GPIOA_ADC3, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN4, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN5, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN6, 0) | \
                                     PIN_AFIO_AF(GPIOA_DIS01, 0))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_I2C3_SCL, 4) | \
                                     PIN_AFIO_AF(GPIOA_UART1_TX, 7) | \
                                     PIN_AFIO_AF(GPIOA_UART1_RX, 7) | \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DM, 10) | \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DP, 10) | \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0) | \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0) | \
                                     PIN_AFIO_AF(GPIOA_BAS01, 0))

/*
 * GPIOB setup:
 */
#define VAL_GPIOB_MODER             (PIN_MODE_OUTPUT(GPIOB_DIS04) | \
                                     PIN_MODE_OUTPUT(GPIOB_DIS05) | \
                                     PIN_MODE_INPUT(GPIOB_PIN2) | \
                                     PIN_MODE_OUTPUT(GPIOB_BAS20) | \
                                     PIN_MODE_OUTPUT(GPIOB_BAS21) | \
                                     PIN_MODE_OUTPUT(GPIOB_BAS22) | \
                                     PIN_MODE_OUTPUT(GPIOB_BAS23) | \
                                     PIN_MODE_OUTPUT(GPIOB_BAS24) | \
                                     PIN_MODE_OUTPUT(GPIOB_BAS25) | \
                                     PIN_MODE_OUTPUT(GPIOB_BAS26) | \
                                     PIN_MODE_OUTPUT(GPIOB_DIS22) | \
                                     PIN_MODE_OUTPUT(GPIOB_DIS23) | \
                                     PIN_MODE_INPUT(GPIOB_PIN12) | \
                                     PIN_MODE_OUTPUT(GPIOB_DIS31) | \
                                     PIN_MODE_OUTPUT(GPIOB_DIS32) | \
                                     PIN_MODE_OUTPUT(GPIOB_DIS33))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOB_DIS04) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_DIS05) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN2) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOB_BAS20) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOB_BAS21) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOB_BAS22) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOB_BAS23) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOB_BAS24) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOB_BAS25) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOB_BAS26) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_DIS22) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_DIS23) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN12) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_DIS31) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_DIS32) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_DIS33))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_100M(GPIOB_DIS04) | \
                                     PIN_OSPEED_100M(GPIOB_DIS05) | \
                                     PIN_OSPEED_100M(GPIOB_PIN2) | \
                                     PIN_OSPEED_100M(GPIOB_BAS20) | \
                                     PIN_OSPEED_100M(GPIOB_BAS21) | \
                                     PIN_OSPEED_100M(GPIOB_BAS22) | \
                                     PIN_OSPEED_100M(GPIOB_BAS23) | \
                                     PIN_OSPEED_100M(GPIOB_BAS24) | \
                                     PIN_OSPEED_100M(GPIOB_BAS25) | \
                                     PIN_OSPEED_100M(GPIOB_BAS26) | \
                                     PIN_OSPEED_100M(GPIOB_DIS22) | \
                                     PIN_OSPEED_100M(GPIOB_DIS23) | \
                                     PIN_OSPEED_100M(GPIOB_PIN12) | \
                                     PIN_OSPEED_100M(GPIOB_DIS31) | \
                                     PIN_OSPEED_100M(GPIOB_DIS32) | \
                                     PIN_OSPEED_100M(GPIOB_DIS33))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_DIS04) | \
                                     PIN_PUPDR_FLOATING(GPIOB_DIS05) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN2) | \
                                     PIN_PUPDR_FLOATING(GPIOB_BAS20) | \
                                     PIN_PUPDR_FLOATING(GPIOB_BAS21) | \
                                     PIN_PUPDR_FLOATING(GPIOB_BAS22) | \
                                     PIN_PUPDR_FLOATING(GPIOB_BAS23) | \
                                     PIN_PUPDR_FLOATING(GPIOB_BAS24) | \
                                     PIN_PUPDR_FLOATING(GPIOB_BAS25) | \
                                     PIN_PUPDR_FLOATING(GPIOB_BAS26) | \
                                     PIN_PUPDR_FLOATING(GPIOB_DIS22) | \
                                     PIN_PUPDR_FLOATING(GPIOB_DIS23) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN12) | \
                                     PIN_PUPDR_FLOATING(GPIOB_DIS31) | \
                                     PIN_PUPDR_FLOATING(GPIOB_DIS32) | \
                                     PIN_PUPDR_FLOATING(GPIOB_DIS33))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_DIS04) | \
                                     PIN_ODR_HIGH(GPIOB_DIS05) | \
                                     PIN_ODR_HIGH(GPIOB_PIN2) | \
                                     PIN_ODR_HIGH(GPIOB_BAS20) | \
                                     PIN_ODR_HIGH(GPIOB_BAS21) | \
                                     PIN_ODR_HIGH(GPIOB_BAS22) | \
                                     PIN_ODR_HIGH(GPIOB_BAS23) | \
                                     PIN_ODR_HIGH(GPIOB_BAS24) | \
                                     PIN_ODR_HIGH(GPIOB_BAS25) | \
                                     PIN_ODR_HIGH(GPIOB_BAS26) | \
                                     PIN_ODR_HIGH(GPIOB_DIS22) | \
                                     PIN_ODR_HIGH(GPIOB_DIS23) | \
                                     PIN_ODR_HIGH(GPIOB_PIN12) | \
                                     PIN_ODR_HIGH(GPIOB_DIS31) | \
                                     PIN_ODR_HIGH(GPIOB_DIS32) | \
                                     PIN_ODR_HIGH(GPIOB_DIS33))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_DIS04, 0) | \
                                     PIN_AFIO_AF(GPIOB_DIS05, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN2, 0) | \
                                     PIN_AFIO_AF(GPIOB_BAS20, 0) | \
                                     PIN_AFIO_AF(GPIOB_BAS21, 0) | \
                                     PIN_AFIO_AF(GPIOB_BAS22, 0) | \
                                     PIN_AFIO_AF(GPIOB_BAS23, 4) | \
                                     PIN_AFIO_AF(GPIOB_BAS24, 0))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_BAS25, 0) | \
                                     PIN_AFIO_AF(GPIOB_BAS26, 0) | \
                                     PIN_AFIO_AF(GPIOB_DIS22, 0) | \
                                     PIN_AFIO_AF(GPIOB_DIS23, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN12, 0) | \
                                     PIN_AFIO_AF(GPIOB_DIS31, 0) | \
                                     PIN_AFIO_AF(GPIOB_DIS32, 0) | \
                                     PIN_AFIO_AF(GPIOB_DIS33, 0))

/*
 * GPIOC setup:
 */
#define VAL_GPIOC_MODER             (PIN_MODE_ANALOG(GPIOC_ADC10) | \
                                     PIN_MODE_ANALOG(GPIOC_ADC11) | \
                                     PIN_MODE_ANALOG(GPIOC_ADC12) | \
                                     PIN_MODE_ANALOG(GPIOC_ADC13) | \
                                     PIN_MODE_OUTPUT(GPIOC_DIS02) | \
                                     PIN_MODE_OUTPUT(GPIOC_DIS03) | \
                                     PIN_MODE_OUTPUT(GPIOC_DIS49) | \
                                     PIN_MODE_OUTPUT(GPIOC_DIS50) | \
                                     PIN_MODE_OUTPUT(GPIOC_DIS51) | \
                                     PIN_MODE_OUTPUT(GPIOC_I2C3_SDA) | \
                                     PIN_MODE_OUTPUT(GPIOC_BAS02) | \
                                     PIN_MODE_OUTPUT(GPIOC_BAS03) | \
                                     PIN_MODE_OUTPUT(GPIOC_BAS04) | \
                                     PIN_MODE_INPUT(GPIOC_PIN13) | \
                                     PIN_MODE_INPUT(GPIOC_PIN14) | \
                                     PIN_MODE_INPUT(GPIOC_PIN15))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_ADC10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ADC11) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ADC12) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ADC13) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_DIS02) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_DIS03) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_DIS49) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_DIS50) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_DIS51) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_I2C3_SDA) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOC_BAS02) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOC_BAS03) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOC_BAS04) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN13) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN14) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN15))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_100M(GPIOC_ADC10) | \
                                     PIN_OSPEED_100M(GPIOC_ADC11) | \
                                     PIN_OSPEED_100M(GPIOC_ADC12) | \
                                     PIN_OSPEED_100M(GPIOC_ADC13) | \
                                     PIN_OSPEED_100M(GPIOC_DIS02) | \
                                     PIN_OSPEED_100M(GPIOC_DIS03) | \
                                     PIN_OSPEED_100M(GPIOC_DIS49) | \
                                     PIN_OSPEED_100M(GPIOC_DIS50) | \
                                     PIN_OSPEED_100M(GPIOC_DIS51) | \
                                     PIN_OSPEED_100M(GPIOC_I2C3_SDA) | \
                                     PIN_OSPEED_100M(GPIOC_BAS02) | \
                                     PIN_OSPEED_100M(GPIOC_BAS03) | \
                                     PIN_OSPEED_100M(GPIOC_BAS04) | \
                                     PIN_OSPEED_100M(GPIOC_PIN13) | \
                                     PIN_OSPEED_100M(GPIOC_PIN14) | \
                                     PIN_OSPEED_100M(GPIOC_PIN15))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_ADC10) | \
                                     PIN_PUPDR_FLOATING(GPIOC_ADC11) | \
                                     PIN_PUPDR_FLOATING(GPIOC_ADC12) | \
                                     PIN_PUPDR_FLOATING(GPIOC_ADC13) | \
                                     PIN_PUPDR_FLOATING(GPIOC_DIS02) | \
                                     PIN_PUPDR_FLOATING(GPIOC_DIS03) | \
                                     PIN_PUPDR_FLOATING(GPIOC_DIS49) | \
                                     PIN_PUPDR_FLOATING(GPIOC_DIS50) | \
                                     PIN_PUPDR_FLOATING(GPIOC_DIS51) | \
                                     PIN_PUPDR_FLOATING(GPIOC_I2C3_SDA) | \
                                     PIN_PUPDR_FLOATING(GPIOC_BAS02) | \
                                     PIN_PUPDR_FLOATING(GPIOC_BAS03) | \
                                     PIN_PUPDR_FLOATING(GPIOC_BAS04) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN13) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN14) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN15))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_ADC10) | \
                                     PIN_ODR_HIGH(GPIOC_ADC11) | \
                                     PIN_ODR_HIGH(GPIOC_ADC12) | \
                                     PIN_ODR_HIGH(GPIOC_ADC13) | \
                                     PIN_ODR_HIGH(GPIOC_DIS02) | \
                                     PIN_ODR_HIGH(GPIOC_DIS03) | \
                                     PIN_ODR_HIGH(GPIOC_DIS49) | \
                                     PIN_ODR_HIGH(GPIOC_DIS50) | \
                                     PIN_ODR_HIGH(GPIOC_DIS51) | \
                                     PIN_ODR_HIGH(GPIOC_I2C3_SDA) | \
                                     PIN_ODR_HIGH(GPIOC_BAS02) | \
                                     PIN_ODR_HIGH(GPIOC_BAS03) | \
                                     PIN_ODR_HIGH(GPIOC_BAS04) | \
                                     PIN_ODR_HIGH(GPIOC_PIN13) | \
                                     PIN_ODR_HIGH(GPIOC_PIN14) | \
                                     PIN_ODR_HIGH(GPIOC_PIN15))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_ADC10, 0) | \
                                     PIN_AFIO_AF(GPIOC_ADC11, 0) | \
                                     PIN_AFIO_AF(GPIOC_ADC12, 0) | \
                                     PIN_AFIO_AF(GPIOC_ADC13, 0) | \
                                     PIN_AFIO_AF(GPIOC_DIS02, 0) | \
                                     PIN_AFIO_AF(GPIOC_DIS03, 0) | \
                                     PIN_AFIO_AF(GPIOC_DIS49, 0) | \
                                     PIN_AFIO_AF(GPIOC_DIS50, 6))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_DIS51, 0) | \
                                     PIN_AFIO_AF(GPIOC_I2C3_SDA, 4) | \
                                     PIN_AFIO_AF(GPIOC_BAS02, 6) | \
                                     PIN_AFIO_AF(GPIOC_BAS03, 0) | \
                                     PIN_AFIO_AF(GPIOC_BAS04, 6) | \
                                     PIN_AFIO_AF(GPIOC_PIN13, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN14, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN15, 0))

/*
 * GPIOD setup:
 */
#define VAL_GPIOD_MODER             (PIN_MODE_OUTPUT(GPIOD_BAS05) | \
                                     PIN_MODE_OUTPUT(GPIOD_BAS06) | \
                                     PIN_MODE_OUTPUT(GPIOD_BAS07) | \
                                     PIN_MODE_OUTPUT(GPIOD_BAS08) | \
                                     PIN_MODE_OUTPUT(GPIOD_BAS09) | \
                                     PIN_MODE_OUTPUT(GPIOD_BAS10) | \
                                     PIN_MODE_OUTPUT(GPIOD_BAS11) | \
                                     PIN_MODE_OUTPUT(GPIOD_BAS12) | \
                                     PIN_MODE_OUTPUT(GPIOD_DIS34) | \
                                     PIN_MODE_OUTPUT(GPIOD_DIS35) | \
                                     PIN_MODE_OUTPUT(GPIOD_DIS36) | \
                                     PIN_MODE_OUTPUT(GPIOD_DIS37) | \
                                     PIN_MODE_OUTPUT(GPIOD_DIS38) | \
                                     PIN_MODE_OUTPUT(GPIOD_DIS39) | \
                                     PIN_MODE_OUTPUT(GPIOD_DIS40) | \
                                     PIN_MODE_OUTPUT(GPIOD_DIS41))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_BUTTONOUT(GPIOD_BAS05) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOD_BAS06) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOD_BAS07) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOD_BAS08) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOD_BAS09) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOD_BAS10) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOD_BAS11) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOD_BAS12) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOD_DIS34) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOD_DIS35) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOD_DIS36) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOD_DIS37) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOD_DIS38) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOD_DIS39) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOD_DIS40) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOD_DIS41))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_100M(GPIOD_BAS05) | \
                                     PIN_OSPEED_100M(GPIOD_BAS06) | \
                                     PIN_OSPEED_100M(GPIOD_BAS07) | \
                                     PIN_OSPEED_100M(GPIOD_BAS08) | \
                                     PIN_OSPEED_100M(GPIOD_BAS09) | \
                                     PIN_OSPEED_100M(GPIOD_BAS10) | \
                                     PIN_OSPEED_100M(GPIOD_BAS11) | \
                                     PIN_OSPEED_100M(GPIOD_BAS12) | \
                                     PIN_OSPEED_100M(GPIOD_DIS34) | \
                                     PIN_OSPEED_100M(GPIOD_DIS35) | \
                                     PIN_OSPEED_100M(GPIOD_DIS36) | \
                                     PIN_OSPEED_100M(GPIOD_DIS37) | \
                                     PIN_OSPEED_100M(GPIOD_DIS38) | \
                                     PIN_OSPEED_100M(GPIOD_DIS39) | \
                                     PIN_OSPEED_100M(GPIOD_DIS40) | \
                                     PIN_OSPEED_100M(GPIOD_DIS41))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(GPIOD_BAS05) | \
                                     PIN_PUPDR_FLOATING(GPIOD_BAS06) | \
                                     PIN_PUPDR_FLOATING(GPIOD_BAS07) | \
                                     PIN_PUPDR_FLOATING(GPIOD_BAS08) | \
                                     PIN_PUPDR_FLOATING(GPIOD_BAS09) | \
                                     PIN_PUPDR_FLOATING(GPIOD_BAS10) | \
                                     PIN_PUPDR_FLOATING(GPIOD_BAS11) | \
                                     PIN_PUPDR_FLOATING(GPIOD_BAS12) | \
                                     PIN_PUPDR_FLOATING(GPIOD_DIS34) | \
                                     PIN_PUPDR_FLOATING(GPIOD_DIS35) | \
                                     PIN_PUPDR_FLOATING(GPIOD_DIS36) | \
                                     PIN_PUPDR_FLOATING(GPIOD_DIS37) | \
                                     PIN_PUPDR_FLOATING(GPIOD_DIS38) | \
                                     PIN_PUPDR_FLOATING(GPIOD_DIS39) | \
                                     PIN_PUPDR_FLOATING(GPIOD_DIS40) | \
                                     PIN_PUPDR_FLOATING(GPIOD_DIS41))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_BAS05) | \
                                     PIN_ODR_HIGH(GPIOD_BAS06) | \
                                     PIN_ODR_HIGH(GPIOD_BAS07) | \
                                     PIN_ODR_HIGH(GPIOD_BAS08) | \
                                     PIN_ODR_HIGH(GPIOD_BAS09) | \
                                     PIN_ODR_HIGH(GPIOD_BAS10) | \
                                     PIN_ODR_HIGH(GPIOD_BAS11) | \
                                     PIN_ODR_HIGH(GPIOD_BAS12) | \
                                     PIN_ODR_HIGH(GPIOD_DIS34) | \
                                     PIN_ODR_HIGH(GPIOD_DIS35) | \
                                     PIN_ODR_HIGH(GPIOD_DIS36) | \
                                     PIN_ODR_HIGH(GPIOD_DIS37) | \
                                     PIN_ODR_HIGH(GPIOD_DIS38) | \
                                     PIN_ODR_HIGH(GPIOD_DIS39) | \
                                     PIN_ODR_HIGH(GPIOD_DIS40) | \
                                     PIN_ODR_HIGH(GPIOD_DIS41))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_BAS05, 0) | \
                                     PIN_AFIO_AF(GPIOD_BAS06, 0) | \
                                     PIN_AFIO_AF(GPIOD_BAS07, 0) | \
                                     PIN_AFIO_AF(GPIOD_BAS08, 0) | \
                                     PIN_AFIO_AF(GPIOD_BAS09, 0) | \
                                     PIN_AFIO_AF(GPIOD_BAS10, 0) | \
                                     PIN_AFIO_AF(GPIOD_BAS11, 0) | \
                                     PIN_AFIO_AF(GPIOD_BAS12, 0))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_DIS34, 0) | \
                                     PIN_AFIO_AF(GPIOD_DIS35, 0) | \
                                     PIN_AFIO_AF(GPIOD_DIS36, 0) | \
                                     PIN_AFIO_AF(GPIOD_DIS37, 0) | \
                                     PIN_AFIO_AF(GPIOD_DIS38, 0) | \
                                     PIN_AFIO_AF(GPIOD_DIS39, 0) | \
                                     PIN_AFIO_AF(GPIOD_DIS40, 0) | \
                                     PIN_AFIO_AF(GPIOD_DIS41, 0))

/*
 * GPIOE setup:
 */
#define VAL_GPIOE_MODER             (PIN_MODE_OUTPUT(GPIOE_BAS27) | \
                                     PIN_MODE_OUTPUT(GPIOE_BAS28) | \
                                     PIN_MODE_OUTPUT(GPIOE_BAS33) | \
                                     PIN_MODE_OUTPUT(GPIOE_BAS34) | \
                                     PIN_MODE_OUTPUT(GPIOE_BAS35) | \
                                     PIN_MODE_OUTPUT(GPIOE_BAS36) | \
                                     PIN_MODE_OUTPUT(GPIOE_BAS37) | \
                                     PIN_MODE_OUTPUT(GPIOE_DIS13) | \
                                     PIN_MODE_OUTPUT(GPIOE_DIS14) | \
                                     PIN_MODE_OUTPUT(GPIOE_DIS15) | \
                                     PIN_MODE_OUTPUT(GPIOE_DIS16) | \
                                     PIN_MODE_OUTPUT(GPIOE_DIS17) | \
                                     PIN_MODE_OUTPUT(GPIOE_DIS18) | \
                                     PIN_MODE_OUTPUT(GPIOE_DIS19) | \
                                     PIN_MODE_OUTPUT(GPIOE_DIS20) | \
                                     PIN_MODE_OUTPUT(GPIOE_DIS21))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_BUTTONOUT(GPIOE_BAS27) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOE_BAS28) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOE_BAS33) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOE_BAS34) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOE_BAS35) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOE_BAS36) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOE_BAS37) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOE_DIS13) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOE_DIS14) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOE_DIS15) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOE_DIS16) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOE_DIS17) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOE_DIS18) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOE_DIS19) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOE_DIS20) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOE_DIS21))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_100M(GPIOE_BAS27) | \
                                     PIN_OSPEED_100M(GPIOE_BAS28) | \
                                     PIN_OSPEED_100M(GPIOE_BAS33) | \
                                     PIN_OSPEED_100M(GPIOE_BAS34) | \
                                     PIN_OSPEED_100M(GPIOE_BAS35) | \
                                     PIN_OSPEED_100M(GPIOE_BAS36) | \
                                     PIN_OSPEED_100M(GPIOE_BAS37) | \
                                     PIN_OSPEED_100M(GPIOE_DIS13) | \
                                     PIN_OSPEED_100M(GPIOE_DIS14) | \
                                     PIN_OSPEED_100M(GPIOE_DIS15) | \
                                     PIN_OSPEED_100M(GPIOE_DIS16) | \
                                     PIN_OSPEED_100M(GPIOE_DIS17) | \
                                     PIN_OSPEED_100M(GPIOE_DIS18) | \
                                     PIN_OSPEED_100M(GPIOE_DIS19) | \
                                     PIN_OSPEED_100M(GPIOE_DIS20) | \
                                     PIN_OSPEED_100M(GPIOE_DIS21))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_FLOATING(GPIOE_BAS27) | \
                                     PIN_PUPDR_FLOATING(GPIOE_BAS28) | \
                                     PIN_PUPDR_FLOATING(GPIOE_BAS33) | \
                                     PIN_PUPDR_FLOATING(GPIOE_BAS34) | \
                                     PIN_PUPDR_FLOATING(GPIOE_BAS35) | \
                                     PIN_PUPDR_FLOATING(GPIOE_BAS36) | \
                                     PIN_PUPDR_FLOATING(GPIOE_BAS37) | \
                                     PIN_PUPDR_FLOATING(GPIOE_DIS13) | \
                                     PIN_PUPDR_FLOATING(GPIOE_DIS14) | \
                                     PIN_PUPDR_FLOATING(GPIOE_DIS15) | \
                                     PIN_PUPDR_FLOATING(GPIOE_DIS16) | \
                                     PIN_PUPDR_FLOATING(GPIOE_DIS17) | \
                                     PIN_PUPDR_FLOATING(GPIOE_DIS18) | \
                                     PIN_PUPDR_FLOATING(GPIOE_DIS19) | \
                                     PIN_PUPDR_FLOATING(GPIOE_DIS20) | \
                                     PIN_PUPDR_FLOATING(GPIOE_DIS21))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_BAS27) | \
                                     PIN_ODR_HIGH(GPIOE_BAS28) | \
                                     PIN_ODR_HIGH(GPIOE_BAS33) | \
                                     PIN_ODR_HIGH(GPIOE_BAS34) | \
                                     PIN_ODR_HIGH(GPIOE_BAS35) | \
                                     PIN_ODR_HIGH(GPIOE_BAS36) | \
                                     PIN_ODR_HIGH(GPIOE_BAS37) | \
                                     PIN_ODR_HIGH(GPIOE_DIS13) | \
                                     PIN_ODR_HIGH(GPIOE_DIS14) | \
                                     PIN_ODR_HIGH(GPIOE_DIS15) | \
                                     PIN_ODR_HIGH(GPIOE_DIS16) | \
                                     PIN_ODR_HIGH(GPIOE_DIS17) | \
                                     PIN_ODR_HIGH(GPIOE_DIS18) | \
                                     PIN_ODR_HIGH(GPIOE_DIS19) | \
                                     PIN_ODR_HIGH(GPIOE_DIS20) | \
                                     PIN_ODR_HIGH(GPIOE_DIS21))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_BAS27, 0) | \
                                     PIN_AFIO_AF(GPIOE_BAS28, 0) | \
                                     PIN_AFIO_AF(GPIOE_BAS33, 0) | \
                                     PIN_AFIO_AF(GPIOE_BAS34, 0) | \
                                     PIN_AFIO_AF(GPIOE_BAS35, 0) | \
                                     PIN_AFIO_AF(GPIOE_BAS36, 0) | \
                                     PIN_AFIO_AF(GPIOE_BAS37, 0) | \
                                     PIN_AFIO_AF(GPIOE_DIS13, 0))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_DIS14, 0) | \
                                     PIN_AFIO_AF(GPIOE_DIS15, 0) | \
                                     PIN_AFIO_AF(GPIOE_DIS16, 0) | \
                                     PIN_AFIO_AF(GPIOE_DIS17, 0) | \
                                     PIN_AFIO_AF(GPIOE_DIS18, 0) | \
                                     PIN_AFIO_AF(GPIOE_DIS19, 0) | \
                                     PIN_AFIO_AF(GPIOE_DIS20, 0) | \
                                     PIN_AFIO_AF(GPIOE_DIS21, 0))

/*
 * GPIOF setup:
 */
#define VAL_GPIOF_MODER             (PIN_MODE_OUTPUT(GPIOF_BAS41) | \
                                     PIN_MODE_OUTPUT(GPIOF_BAS42) | \
                                     PIN_MODE_OUTPUT(GPIOF_BAS43) | \
                                     PIN_MODE_OUTPUT(GPIOF_BAS44) | \
                                     PIN_MODE_OUTPUT(GPIOF_BAS45) | \
                                     PIN_MODE_OUTPUT(GPIOF_BAS46) | \
                                     PIN_MODE_OUTPUT(GPIOF_BAS47) | \
                                     PIN_MODE_OUTPUT(GPIOF_BAS48) | \
                                     PIN_MODE_OUTPUT(GPIOF_BAS49) | \
                                     PIN_MODE_OUTPUT(GPIOF_BAS50) | \
                                     PIN_MODE_OUTPUT(GPIOF_BAS51) | \
                                     PIN_MODE_OUTPUT(GPIOF_DIS06) | \
                                     PIN_MODE_OUTPUT(GPIOF_DIS07) | \
                                     PIN_MODE_OUTPUT(GPIOF_DIS08) | \
                                     PIN_MODE_OUTPUT(GPIOF_DIS09) | \
                                     PIN_MODE_OUTPUT(GPIOF_DIS10))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_BUTTONOUT(GPIOF_BAS41) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOF_BAS42) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOF_BAS43) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOF_BAS44) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOF_BAS45) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOF_BAS46) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOF_BAS47) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOF_BAS48) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOF_BAS49) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOF_BAS50) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOF_BAS51) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_DIS06) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_DIS07) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_DIS08) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_DIS09) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_DIS10))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_100M(GPIOF_BAS41) | \
                                     PIN_OSPEED_100M(GPIOF_BAS42) | \
                                     PIN_OSPEED_100M(GPIOF_BAS43) | \
                                     PIN_OSPEED_100M(GPIOF_BAS44) | \
                                     PIN_OSPEED_100M(GPIOF_BAS45) | \
                                     PIN_OSPEED_100M(GPIOF_BAS46) | \
                                     PIN_OSPEED_100M(GPIOF_BAS47) | \
                                     PIN_OSPEED_100M(GPIOF_BAS48) | \
                                     PIN_OSPEED_100M(GPIOF_BAS49) | \
                                     PIN_OSPEED_100M(GPIOF_BAS50) | \
                                     PIN_OSPEED_100M(GPIOF_BAS51) | \
                                     PIN_OSPEED_100M(GPIOF_DIS06) | \
                                     PIN_OSPEED_100M(GPIOF_DIS07) | \
                                     PIN_OSPEED_100M(GPIOF_DIS08) | \
                                     PIN_OSPEED_100M(GPIOF_DIS09) | \
                                     PIN_OSPEED_100M(GPIOF_DIS10))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_BAS41) | \
                                     PIN_PUPDR_FLOATING(GPIOF_BAS42) | \
                                     PIN_PUPDR_FLOATING(GPIOF_BAS43) | \
                                     PIN_PUPDR_FLOATING(GPIOF_BAS44) | \
                                     PIN_PUPDR_FLOATING(GPIOF_BAS45) | \
                                     PIN_PUPDR_FLOATING(GPIOF_BAS46) | \
                                     PIN_PUPDR_FLOATING(GPIOF_BAS47) | \
                                     PIN_PUPDR_FLOATING(GPIOF_BAS48) | \
                                     PIN_PUPDR_FLOATING(GPIOF_BAS49) | \
                                     PIN_PUPDR_FLOATING(GPIOF_BAS50) | \
                                     PIN_PUPDR_FLOATING(GPIOF_BAS51) | \
                                     PIN_PUPDR_FLOATING(GPIOF_DIS06) | \
                                     PIN_PUPDR_FLOATING(GPIOF_DIS07) | \
                                     PIN_PUPDR_FLOATING(GPIOF_DIS08) | \
                                     PIN_PUPDR_FLOATING(GPIOF_DIS09) | \
                                     PIN_PUPDR_FLOATING(GPIOF_DIS10))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_BAS41) | \
                                     PIN_ODR_HIGH(GPIOF_BAS42) | \
                                     PIN_ODR_HIGH(GPIOF_BAS43) | \
                                     PIN_ODR_HIGH(GPIOF_BAS44) | \
                                     PIN_ODR_HIGH(GPIOF_BAS45) | \
                                     PIN_ODR_HIGH(GPIOF_BAS46) | \
                                     PIN_ODR_HIGH(GPIOF_BAS47) | \
                                     PIN_ODR_HIGH(GPIOF_BAS48) | \
                                     PIN_ODR_HIGH(GPIOF_BAS49) | \
                                     PIN_ODR_HIGH(GPIOF_BAS50) | \
                                     PIN_ODR_HIGH(GPIOF_BAS51) | \
                                     PIN_ODR_HIGH(GPIOF_DIS06) | \
                                     PIN_ODR_HIGH(GPIOF_DIS07) | \
                                     PIN_ODR_HIGH(GPIOF_DIS08) | \
                                     PIN_ODR_HIGH(GPIOF_DIS09) | \
                                     PIN_ODR_HIGH(GPIOF_DIS10))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_BAS41, 0) | \
                                     PIN_AFIO_AF(GPIOF_BAS42, 0) | \
                                     PIN_AFIO_AF(GPIOF_BAS43, 0) | \
                                     PIN_AFIO_AF(GPIOF_BAS44, 0) | \
                                     PIN_AFIO_AF(GPIOF_BAS45, 0) | \
                                     PIN_AFIO_AF(GPIOF_BAS46, 0) | \
                                     PIN_AFIO_AF(GPIOF_BAS47, 0) | \
                                     PIN_AFIO_AF(GPIOF_BAS48, 0))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_BAS49, 0) | \
                                     PIN_AFIO_AF(GPIOF_BAS50, 0) | \
                                     PIN_AFIO_AF(GPIOF_BAS51, 0) | \
                                     PIN_AFIO_AF(GPIOF_DIS06, 0) | \
                                     PIN_AFIO_AF(GPIOF_DIS07, 0) | \
                                     PIN_AFIO_AF(GPIOF_DIS08, 0) | \
                                     PIN_AFIO_AF(GPIOF_DIS09, 0) | \
                                     PIN_AFIO_AF(GPIOF_DIS10, 0))

/*
 * GPIOG setup:
 */
#define VAL_GPIOG_MODER             (PIN_MODE_OUTPUT(GPIOG_DIS11) | \
                                     PIN_MODE_OUTPUT(GPIOG_DIS12) | \
                                     PIN_MODE_OUTPUT(GPIOG_DIS42) | \
                                     PIN_MODE_OUTPUT(GPIOG_DIS43) | \
                                     PIN_MODE_OUTPUT(GPIOG_DIS44) | \
                                     PIN_MODE_OUTPUT(GPIOG_DIS45) | \
                                     PIN_MODE_OUTPUT(GPIOG_DIS46) | \
                                     PIN_MODE_OUTPUT(GPIOG_DIS47) | \
                                     PIN_MODE_OUTPUT(GPIOG_DIS48) | \
                                     PIN_MODE_OUTPUT(GPIOG_BAS13) | \
                                     PIN_MODE_OUTPUT(GPIOG_BAS14) | \
                                     PIN_MODE_OUTPUT(GPIOG_BAS15) | \
                                     PIN_MODE_OUTPUT(GPIOG_BAS16) | \
                                     PIN_MODE_OUTPUT(GPIOG_BAS17) | \
                                     PIN_MODE_OUTPUT(GPIOG_BAS18) | \
                                     PIN_MODE_OUTPUT(GPIOG_BAS19))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOG_DIS11) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOG_DIS12) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOG_DIS42) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOG_DIS43) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOG_DIS44) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOG_DIS45) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOG_DIS46) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOG_DIS47) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOG_DIS48) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOG_BAS13) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOG_BAS14) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOG_BAS15) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOG_BAS16) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOG_BAS17) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOG_BAS18) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOG_BAS19))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_100M(GPIOG_DIS11) | \
                                     PIN_OSPEED_100M(GPIOG_DIS12) | \
                                     PIN_OSPEED_100M(GPIOG_DIS42) | \
                                     PIN_OSPEED_100M(GPIOG_DIS43) | \
                                     PIN_OSPEED_100M(GPIOG_DIS44) | \
                                     PIN_OSPEED_100M(GPIOG_DIS45) | \
                                     PIN_OSPEED_100M(GPIOG_DIS46) | \
                                     PIN_OSPEED_100M(GPIOG_DIS47) | \
                                     PIN_OSPEED_100M(GPIOG_DIS48) | \
                                     PIN_OSPEED_100M(GPIOG_BAS13) | \
                                     PIN_OSPEED_100M(GPIOG_BAS14) | \
                                     PIN_OSPEED_100M(GPIOG_BAS15) | \
                                     PIN_OSPEED_100M(GPIOG_BAS16) | \
                                     PIN_OSPEED_100M(GPIOG_BAS17) | \
                                     PIN_OSPEED_100M(GPIOG_BAS18) | \
                                     PIN_OSPEED_100M(GPIOG_BAS19))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_FLOATING(GPIOG_DIS11) | \
                                     PIN_PUPDR_FLOATING(GPIOG_DIS12) | \
                                     PIN_PUPDR_FLOATING(GPIOG_DIS42) | \
                                     PIN_PUPDR_FLOATING(GPIOG_DIS43) | \
                                     PIN_PUPDR_FLOATING(GPIOG_DIS44) | \
                                     PIN_PUPDR_FLOATING(GPIOG_DIS45) | \
                                     PIN_PUPDR_FLOATING(GPIOG_DIS46) | \
                                     PIN_PUPDR_FLOATING(GPIOG_DIS47) | \
                                     PIN_PUPDR_FLOATING(GPIOG_DIS48) | \
                                     PIN_PUPDR_FLOATING(GPIOG_BAS13) | \
                                     PIN_PUPDR_FLOATING(GPIOG_BAS14) | \
                                     PIN_PUPDR_FLOATING(GPIOG_BAS15) | \
                                     PIN_PUPDR_FLOATING(GPIOG_BAS16) | \
                                     PIN_PUPDR_FLOATING(GPIOG_BAS17) | \
                                     PIN_PUPDR_FLOATING(GPIOG_BAS18) | \
                                     PIN_PUPDR_FLOATING(GPIOG_BAS19))
#define VAL_GPIOG_ODR               (PIN_ODR_HIGH(GPIOG_DIS11) | \
                                     PIN_ODR_HIGH(GPIOG_DIS12) | \
                                     PIN_ODR_HIGH(GPIOG_DIS42) | \
                                     PIN_ODR_HIGH(GPIOG_DIS43) | \
                                     PIN_ODR_HIGH(GPIOG_DIS44) | \
                                     PIN_ODR_HIGH(GPIOG_DIS45) | \
                                     PIN_ODR_HIGH(GPIOG_DIS46) | \
                                     PIN_ODR_HIGH(GPIOG_DIS47) | \
                                     PIN_ODR_HIGH(GPIOG_DIS48) | \
                                     PIN_ODR_HIGH(GPIOG_BAS13) | \
                                     PIN_ODR_HIGH(GPIOG_BAS14) | \
                                     PIN_ODR_HIGH(GPIOG_BAS15) | \
                                     PIN_ODR_HIGH(GPIOG_BAS16) | \
                                     PIN_ODR_HIGH(GPIOG_BAS17) | \
                                     PIN_ODR_HIGH(GPIOG_BAS18) | \
                                     PIN_ODR_HIGH(GPIOG_BAS19))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_DIS11, 0) | \
                                     PIN_AFIO_AF(GPIOG_DIS12, 0) | \
                                     PIN_AFIO_AF(GPIOG_DIS42, 0) | \
                                     PIN_AFIO_AF(GPIOG_DIS43, 0) | \
                                     PIN_AFIO_AF(GPIOG_DIS44, 0) | \
                                     PIN_AFIO_AF(GPIOG_DIS45, 0) | \
                                     PIN_AFIO_AF(GPIOG_DIS46, 0) | \
                                     PIN_AFIO_AF(GPIOG_DIS47, 0))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_DIS48, 0) | \
                                     PIN_AFIO_AF(GPIOG_BAS13, 0) | \
                                     PIN_AFIO_AF(GPIOG_BAS14, 0) | \
                                     PIN_AFIO_AF(GPIOG_BAS15, 0) | \
                                     PIN_AFIO_AF(GPIOG_BAS16, 0) | \
                                     PIN_AFIO_AF(GPIOG_BAS17, 0) | \
                                     PIN_AFIO_AF(GPIOG_BAS18, 0) | \
                                     PIN_AFIO_AF(GPIOG_BAS19, 0))

/*
 * GPIOH setup:
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) | \
                                     PIN_MODE_INPUT(GPIOH_OSC_OUT) | \
                                     PIN_MODE_INPUT(GPIOH_PIN2) | \
                                     PIN_MODE_INPUT(GPIOH_PIN3) | \
                                     PIN_MODE_INPUT(GPIOH_PIN4) | \
                                     PIN_MODE_INPUT(GPIOH_PIN5) | \
                                     PIN_MODE_OUTPUT(GPIOH_DIS24) | \
                                     PIN_MODE_OUTPUT(GPIOH_DIS25) | \
                                     PIN_MODE_OUTPUT(GPIOH_DIS26) | \
                                     PIN_MODE_OUTPUT(GPIOH_DIS27) | \
                                     PIN_MODE_OUTPUT(GPIOH_DIS28) | \
                                     PIN_MODE_OUTPUT(GPIOH_DIS29) | \
                                     PIN_MODE_OUTPUT(GPIOH_DIS30) | \
                                     PIN_MODE_INPUT(GPIOH_PIN13) | \
                                     PIN_MODE_INPUT(GPIOH_PIN14) | \
                                     PIN_MODE_OUTPUT(GPIOH_LED1))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_OSC_IN) | \
                                     PIN_OTYPE_PUSHPULL(GPIOH_OSC_OUT) | \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN4) | \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN5) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_DIS24) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_DIS25) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_DIS26) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_DIS27) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_DIS28) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_DIS29) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_DIS30) | \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN13) | \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN14) | \
                                     PIN_OTYPE_PUSHPULL(GPIOH_LED1))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_100M(GPIOH_OSC_IN) | \
                                     PIN_OSPEED_100M(GPIOH_OSC_OUT) | \
                                     PIN_OSPEED_100M(GPIOH_PIN2) | \
                                     PIN_OSPEED_100M(GPIOH_PIN3) | \
                                     PIN_OSPEED_100M(GPIOH_PIN4) | \
                                     PIN_OSPEED_100M(GPIOH_PIN5) | \
                                     PIN_OSPEED_100M(GPIOH_DIS24) | \
                                     PIN_OSPEED_100M(GPIOH_DIS25) | \
                                     PIN_OSPEED_100M(GPIOH_DIS26) | \
                                     PIN_OSPEED_100M(GPIOH_DIS27) | \
                                     PIN_OSPEED_100M(GPIOH_DIS28) | \
                                     PIN_OSPEED_100M(GPIOH_DIS29) | \
                                     PIN_OSPEED_100M(GPIOH_DIS30) | \
                                     PIN_OSPEED_100M(GPIOH_PIN13) | \
                                     PIN_OSPEED_100M(GPIOH_PIN14) | \
                                     PIN_OSPEED_100M(GPIOH_LED1))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_FLOATING(GPIOH_OSC_IN) | \
                                     PIN_PUPDR_FLOATING(GPIOH_OSC_OUT) | \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN2) | \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN3) | \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN4) | \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN5) | \
                                     PIN_PUPDR_FLOATING(GPIOH_DIS24) | \
                                     PIN_PUPDR_FLOATING(GPIOH_DIS25) | \
                                     PIN_PUPDR_FLOATING(GPIOH_DIS26) | \
                                     PIN_PUPDR_FLOATING(GPIOH_DIS27) | \
                                     PIN_PUPDR_FLOATING(GPIOH_DIS28) | \
                                     PIN_PUPDR_FLOATING(GPIOH_DIS29) | \
                                     PIN_PUPDR_FLOATING(GPIOH_DIS30) | \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN13) | \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN14) | \
                                     PIN_PUPDR_FLOATING(GPIOH_LED1))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_OSC_IN) | \
                                     PIN_ODR_HIGH(GPIOH_OSC_OUT) | \
                                     PIN_ODR_HIGH(GPIOH_PIN2) | \
                                     PIN_ODR_HIGH(GPIOH_PIN3) | \
                                     PIN_ODR_HIGH(GPIOH_PIN4) | \
                                     PIN_ODR_HIGH(GPIOH_PIN5) | \
                                     PIN_ODR_HIGH(GPIOH_DIS24) | \
                                     PIN_ODR_HIGH(GPIOH_DIS25) | \
                                     PIN_ODR_HIGH(GPIOH_DIS26) | \
                                     PIN_ODR_HIGH(GPIOH_DIS27) | \
                                     PIN_ODR_HIGH(GPIOH_DIS28) | \
                                     PIN_ODR_HIGH(GPIOH_DIS29) | \
                                     PIN_ODR_HIGH(GPIOH_DIS30) | \
                                     PIN_ODR_HIGH(GPIOH_PIN13) | \
                                     PIN_ODR_HIGH(GPIOH_PIN14) | \
                                     PIN_ODR_LOW(GPIOH_LED1))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_OSC_IN, 0) | \
                                     PIN_AFIO_AF(GPIOH_OSC_OUT, 0) | \
                                     PIN_AFIO_AF(GPIOH_PIN2, 0) | \
                                     PIN_AFIO_AF(GPIOH_PIN3, 0) | \
                                     PIN_AFIO_AF(GPIOH_PIN4, 0) | \
                                     PIN_AFIO_AF(GPIOH_PIN5, 0) | \
                                     PIN_AFIO_AF(GPIOH_DIS24, 0) | \
                                     PIN_AFIO_AF(GPIOH_DIS25, 0))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_DIS26, 0) | \
                                     PIN_AFIO_AF(GPIOH_DIS27, 0) | \
                                     PIN_AFIO_AF(GPIOH_DIS28, 0) | \
                                     PIN_AFIO_AF(GPIOH_DIS29, 0) | \
                                     PIN_AFIO_AF(GPIOH_DIS30, 0) | \
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
                                     PIN_MODE_OUTPUT(GPIOI_BAS29) | \
                                     PIN_MODE_OUTPUT(GPIOI_BAS30) | \
                                     PIN_MODE_OUTPUT(GPIOI_BAS31) | \
                                     PIN_MODE_OUTPUT(GPIOI_BAS32) | \
                                     PIN_MODE_INPUT(GPIOI_PIN8) | \
                                     PIN_MODE_OUTPUT(GPIOI_BAS38) | \
                                     PIN_MODE_OUTPUT(GPIOI_BAS39) | \
                                     PIN_MODE_OUTPUT(GPIOI_BAS40) | \
                                     PIN_MODE_INPUT(GPIOI_PIN12) | \
                                     PIN_MODE_INPUT(GPIOI_PIN13) | \
                                     PIN_MODE_INPUT(GPIOI_PIN14) | \
                                     PIN_MODE_INPUT(GPIOI_PIN15))
#define VAL_GPIOI_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOI_PIN0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN3) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOI_BAS29) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOI_BAS30) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOI_BAS31) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOI_BAS32) | \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN8) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOI_BAS38) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOI_BAS39) | \
                                     PIN_OTYPE_BUTTONOUT(GPIOI_BAS40) | \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN12) | \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN13) | \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN14) | \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN15))
#define VAL_GPIOI_OSPEEDR           (PIN_OSPEED_100M(GPIOI_PIN0) | \
                                     PIN_OSPEED_100M(GPIOI_PIN1) | \
                                     PIN_OSPEED_100M(GPIOI_PIN2) | \
                                     PIN_OSPEED_100M(GPIOI_PIN3) | \
                                     PIN_OSPEED_100M(GPIOI_BAS29) | \
                                     PIN_OSPEED_100M(GPIOI_BAS30) | \
                                     PIN_OSPEED_100M(GPIOI_BAS31) | \
                                     PIN_OSPEED_100M(GPIOI_BAS32) | \
                                     PIN_OSPEED_100M(GPIOI_PIN8) | \
                                     PIN_OSPEED_100M(GPIOI_BAS38) | \
                                     PIN_OSPEED_100M(GPIOI_BAS39) | \
                                     PIN_OSPEED_100M(GPIOI_BAS40) | \
                                     PIN_OSPEED_100M(GPIOI_PIN12) | \
                                     PIN_OSPEED_100M(GPIOI_PIN13) | \
                                     PIN_OSPEED_100M(GPIOI_PIN14) | \
                                     PIN_OSPEED_100M(GPIOI_PIN15))
#define VAL_GPIOI_PUPDR             (PIN_PUPDR_FLOATING(GPIOI_PIN0) | \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN1) | \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN2) | \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN3) | \
                                     PIN_PUPDR_FLOATING(GPIOI_BAS29) | \
                                     PIN_PUPDR_FLOATING(GPIOI_BAS30) | \
                                     PIN_PUPDR_FLOATING(GPIOI_BAS31) | \
                                     PIN_PUPDR_FLOATING(GPIOI_BAS32) | \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN8) | \
                                     PIN_PUPDR_FLOATING(GPIOI_BAS38) | \
                                     PIN_PUPDR_FLOATING(GPIOI_BAS39) | \
                                     PIN_PUPDR_FLOATING(GPIOI_BAS40) | \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN12) | \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN13) | \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN14) | \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN15))
#define VAL_GPIOI_ODR               (PIN_ODR_HIGH(GPIOI_PIN0) | \
                                     PIN_ODR_HIGH(GPIOI_PIN1) | \
                                     PIN_ODR_HIGH(GPIOI_PIN2) | \
                                     PIN_ODR_HIGH(GPIOI_PIN3) | \
                                     PIN_ODR_HIGH(GPIOI_BAS29) | \
                                     PIN_ODR_HIGH(GPIOI_BAS30) | \
                                     PIN_ODR_HIGH(GPIOI_BAS31) | \
                                     PIN_ODR_HIGH(GPIOI_BAS32) | \
                                     PIN_ODR_HIGH(GPIOI_PIN8) | \
                                     PIN_ODR_HIGH(GPIOI_BAS38) | \
                                     PIN_ODR_HIGH(GPIOI_BAS39) | \
                                     PIN_ODR_HIGH(GPIOI_BAS40) | \
                                     PIN_ODR_HIGH(GPIOI_PIN12) | \
                                     PIN_ODR_HIGH(GPIOI_PIN13) | \
                                     PIN_ODR_HIGH(GPIOI_PIN14) | \
                                     PIN_ODR_HIGH(GPIOI_PIN15))
#define VAL_GPIOI_AFRL              (PIN_AFIO_AF(GPIOI_PIN0, 0) | \
                                     PIN_AFIO_AF(GPIOI_PIN1, 0) | \
                                     PIN_AFIO_AF(GPIOI_PIN2, 0) | \
                                     PIN_AFIO_AF(GPIOI_PIN3, 0) | \
                                     PIN_AFIO_AF(GPIOI_BAS29, 0) | \
                                     PIN_AFIO_AF(GPIOI_BAS30, 0) | \
                                     PIN_AFIO_AF(GPIOI_BAS31, 0) | \
                                     PIN_AFIO_AF(GPIOI_BAS32, 0))
#define VAL_GPIOI_AFRH              (PIN_AFIO_AF(GPIOI_PIN8, 0) | \
                                     PIN_AFIO_AF(GPIOI_BAS38, 0) | \
                                     PIN_AFIO_AF(GPIOI_BAS39, 0) | \
                                     PIN_AFIO_AF(GPIOI_BAS40, 0) | \
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
