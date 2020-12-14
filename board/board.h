/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

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

/*
 * This file has been automatically generated using ChibiStudio board
 * generator plugin. Do not edit manually.
 */

#ifndef BOARD_H
#define BOARD_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*
 * Setup for Striso dis v2.0.2 H743 board.
 */

/*
 * Board identifier.
 */
#define BOARD_STRISO_H743
#define BOARD_NAME                  "Striso dis v2.0.2 H743"

/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0U
#endif

#define STM32_LSEDRV                (3U << 3U)

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                8000000U
#endif

/*
 * MCU type as defined in the ST header.
 */
#define STM32H743xx

/*
 * IO pins assignments.
 */
#define GPIOA_PIN0                  0U
#define GPIOA_PIN1                  1U
#define GPIOA_PIN2                  2U
#define GPIOA_ADC4                  3U
#define GPIOA_ADC3                  4U
#define GPIOA_ADC2                  5U
#define GPIOA_ADC1                  6U
#define GPIOA_DIS01                 7U
#define GPIOA_I2C3_SCL              8U
#define GPIOA_LED_DOWN2             9U
#define GPIOA_LED_UP2               10U
#define GPIOA_OTG_FS_DM             11U
#define GPIOA_OTG_FS_DP             12U
#define GPIOA_SWDIO                 13U
#define GPIOA_SWCLK                 14U
#define GPIOA_PIN15                 15U

#define GPIOB_DIS04                 0U
#define GPIOB_DIS05                 1U
#define GPIOB_PIN2                  2U
#define GPIOB_PIN3                  3U
#define GPIOB_LED_DOWN              4U
#define GPIOB_LED_UP                5U
#define GPIOB_LED_R                 6U
#define GPIOB_LED_B                 7U
#define GPIOB_LED_G                 8U
#define GPIOB_PIN9                  9U
#define GPIOB_DIS22                 10U
#define GPIOB_DIS23                 11U
#define GPIOB_PIN12                 12U
#define GPIOB_DIS31                 13U
#define GPIOB_DIS32                 14U
#define GPIOB_DIS33                 15U

#define GPIOC_AUX_ADC_T             0U
#define GPIOC_AUX_ADC_R             1U
#define GPIOC_PIN2                  2U
#define GPIOC_PIN3                  3U
#define GPIOC_DIS02                 4U
#define GPIOC_DIS03                 5U
#define GPIOC_DIS49                 6U
#define GPIOC_DIS50                 7U
#define GPIOC_DIS51                 8U
#define GPIOC_I2C3_SDA              9U
#define GPIOC_UART3_TX              10U
#define GPIOC_UART3_RX              11U
#define GPIOC_PIN12                 12U
#define GPIOC_PIN13                 13U
#define GPIOC_PIN14                 14U
#define GPIOC_PIN15                 15U

#define GPIOD_PIN0                  0U
#define GPIOD_PIN1                  1U
#define GPIOD_PIN2                  2U
#define GPIOD_JACK_DETECT           3U
#define GPIOD_AUX_JACK_DETECT       4U
#define GPIOD_AUX_UART2_TX          5U
#define GPIOD_AUX_UART2_RX          6U
#define GPIOD_SPI1_MOSI             7U
#define GPIOD_DIS34                 8U
#define GPIOD_DIS35                 9U
#define GPIOD_DIS36                 10U
#define GPIOD_DIS37                 11U
#define GPIOD_DIS38                 12U
#define GPIOD_DIS39                 13U
#define GPIOD_DIS40                 14U
#define GPIOD_DIS41                 15U

#define GPIOE_AUX_VDD               0U
#define GPIOE_PIN1                  1U
#define GPIOE_SAI1_MCLK_A           2U
#define GPIOE_SAI1_SD_B             3U
#define GPIOE_SAI1_FS_A             4U
#define GPIOE_SAI1_SCK_A            5U
#define GPIOE_SAI1_SD_A             6U
#define GPIOE_DIS13                 7U
#define GPIOE_DIS14                 8U
#define GPIOE_DIS15                 9U
#define GPIOE_DIS16                 10U
#define GPIOE_DIS17                 11U
#define GPIOE_DIS18                 12U
#define GPIOE_DIS19                 13U
#define GPIOE_DIS20                 14U
#define GPIOE_DIS21                 15U

#define GPIOF_I2C2_SDA              0U
#define GPIOF_I2C2_SCL              1U
#define GPIOF_PIN2                  2U
#define GPIOF_PIN3                  3U
#define GPIOF_PIN4                  4U
#define GPIOF_PIN5                  5U
#define GPIOF_ADC_EX2               6U
#define GPIOF_ADC_EX1               7U
#define GPIOF_PIN8                  8U
#define GPIOF_PIN9                  9U
#define GPIOF_PIN10                 10U
#define GPIOF_DIS06                 11U
#define GPIOF_DIS07                 12U
#define GPIOF_DIS08                 13U
#define GPIOF_DIS09                 14U
#define GPIOF_DIS10                 15U

#define GPIOG_DIS11                 0U
#define GPIOG_DIS12                 1U
#define GPIOG_DIS42                 2U
#define GPIOG_DIS43                 3U
#define GPIOG_DIS44                 4U
#define GPIOG_DIS45                 5U
#define GPIOG_DIS46                 6U
#define GPIOG_DIS47                 7U
#define GPIOG_DIS48                 8U
#define GPIOG_SPI1_MISO             9U
#define GPIOG_SPI1_NSS              10U
#define GPIOG_SPI1_SCK              11U
#define GPIOG_VDD1                  12U
#define GPIOG_VDD2                  13U
#define GPIOG_PIN14                 14U
#define GPIOG_PIN15                 15U

#define GPIOH_PIN0                  0U
#define GPIOH_PIN1                  1U
#define GPIOH_PIN2                  2U
#define GPIOH_PIN3                  3U
#define GPIOH_PIN4                  4U
#define GPIOH_PIN5                  5U
#define GPIOH_DIS24                 6U
#define GPIOH_DIS25                 7U
#define GPIOH_DIS26                 8U
#define GPIOH_DIS27                 9U
#define GPIOH_DIS28                 10U
#define GPIOH_DIS29                 11U
#define GPIOH_DIS30                 12U
#define GPIOH_MOTION_INT1           13U
#define GPIOH_PIN14                 14U
#define GPIOH_PIN15                 15U

#define GPIOI_BUTTON_ALT            0U
#define GPIOI_BUTTON_DOWN           1U
#define GPIOI_BUTTON_UP             2U
#define GPIOI_BUTTON_PORT           3U
#define GPIOI_PIN4                  4U
#define GPIOI_PIN5                  5U
#define GPIOI_HP_EN                 6U
#define GPIOI_CODEC_EN              7U
#define GPIOI_PIN8                  8U
#define GPIOI_LED1                  9U
#define GPIOI_PIN10                 10U
#define GPIOI_PIN11                 11U
#define GPIOI_PIN12                 12U
#define GPIOI_PIN13                 13U
#define GPIOI_PIN14                 14U
#define GPIOI_PIN15                 15U

#define GPIOJ_PIN0                  0U
#define GPIOJ_PIN1                  1U
#define GPIOJ_PIN2                  2U
#define GPIOJ_PIN3                  3U
#define GPIOJ_PIN4                  4U
#define GPIOJ_PIN5                  5U
#define GPIOJ_PIN6                  6U
#define GPIOJ_PIN7                  7U
#define GPIOJ_PIN8                  8U
#define GPIOJ_PIN9                  9U
#define GPIOJ_PIN10                 10U
#define GPIOJ_PIN11                 11U
#define GPIOJ_PIN12                 12U
#define GPIOJ_PIN13                 13U
#define GPIOJ_PIN14                 14U
#define GPIOJ_PIN15                 15U

#define GPIOK_PIN0                  0U
#define GPIOK_PIN1                  1U
#define GPIOK_PIN2                  2U
#define GPIOK_PIN3                  3U
#define GPIOK_PIN4                  4U
#define GPIOK_PIN5                  5U
#define GPIOK_PIN6                  6U
#define GPIOK_PIN7                  7U
#define GPIOK_PIN8                  8U
#define GPIOK_PIN9                  9U
#define GPIOK_PIN10                 10U
#define GPIOK_PIN11                 11U
#define GPIOK_PIN12                 12U
#define GPIOK_PIN13                 13U
#define GPIOK_PIN14                 14U
#define GPIOK_PIN15                 15U

/*
 * IO lines assignments.
 */
#define LINE_ADC4                   PAL_LINE(GPIOA, 3U)
#define LINE_ADC3                   PAL_LINE(GPIOA, 4U)
#define LINE_ADC2                   PAL_LINE(GPIOA, 5U)
#define LINE_ADC1                   PAL_LINE(GPIOA, 6U)
#define LINE_DIS01                  PAL_LINE(GPIOA, 7U)
#define LINE_I2C3_SCL               PAL_LINE(GPIOA, 8U)
#define LINE_LED_DOWN2              PAL_LINE(GPIOA, 9U)
#define LINE_LED_UP2                PAL_LINE(GPIOA, 10U)
#define LINE_OTG_FS_DM              PAL_LINE(GPIOA, 11U)
#define LINE_OTG_FS_DP              PAL_LINE(GPIOA, 12U)
#define LINE_SWDIO                  PAL_LINE(GPIOA, 13U)
#define LINE_SWCLK                  PAL_LINE(GPIOA, 14U)
#define LINE_DIS04                  PAL_LINE(GPIOB, 0U)
#define LINE_DIS05                  PAL_LINE(GPIOB, 1U)
#define LINE_LED_DOWN               PAL_LINE(GPIOB, 4U)
#define LINE_LED_UP                 PAL_LINE(GPIOB, 5U)
#define LINE_LED_R                  PAL_LINE(GPIOB, 6U)
#define LINE_LED_B                  PAL_LINE(GPIOB, 7U)
#define LINE_LED_G                  PAL_LINE(GPIOB, 8U)
#define LINE_DIS22                  PAL_LINE(GPIOB, 10U)
#define LINE_DIS23                  PAL_LINE(GPIOB, 11U)
#define LINE_DIS31                  PAL_LINE(GPIOB, 13U)
#define LINE_DIS32                  PAL_LINE(GPIOB, 14U)
#define LINE_DIS33                  PAL_LINE(GPIOB, 15U)
#define LINE_AUX_ADC_T              PAL_LINE(GPIOC, 0U)
#define LINE_AUX_ADC_R              PAL_LINE(GPIOC, 1U)
#define LINE_DIS02                  PAL_LINE(GPIOC, 4U)
#define LINE_DIS03                  PAL_LINE(GPIOC, 5U)
#define LINE_DIS49                  PAL_LINE(GPIOC, 6U)
#define LINE_DIS50                  PAL_LINE(GPIOC, 7U)
#define LINE_DIS51                  PAL_LINE(GPIOC, 8U)
#define LINE_I2C3_SDA               PAL_LINE(GPIOC, 9U)
#define LINE_UART3_TX               PAL_LINE(GPIOC, 10U)
#define LINE_UART3_RX               PAL_LINE(GPIOC, 11U)
#define LINE_JACK_DETECT            PAL_LINE(GPIOD, 3U)
#define LINE_AUX_JACK_DETECT        PAL_LINE(GPIOD, 4U)
#define LINE_AUX_UART2_TX           PAL_LINE(GPIOD, 5U)
#define LINE_AUX_UART2_RX           PAL_LINE(GPIOD, 6U)
#define LINE_SPI1_MOSI              PAL_LINE(GPIOD, 7U)
#define LINE_DIS34                  PAL_LINE(GPIOD, 8U)
#define LINE_DIS35                  PAL_LINE(GPIOD, 9U)
#define LINE_DIS36                  PAL_LINE(GPIOD, 10U)
#define LINE_DIS37                  PAL_LINE(GPIOD, 11U)
#define LINE_DIS38                  PAL_LINE(GPIOD, 12U)
#define LINE_DIS39                  PAL_LINE(GPIOD, 13U)
#define LINE_DIS40                  PAL_LINE(GPIOD, 14U)
#define LINE_DIS41                  PAL_LINE(GPIOD, 15U)
#define LINE_AUX_VDD                PAL_LINE(GPIOE, 0U)
#define LINE_SAI1_MCLK_A            PAL_LINE(GPIOE, 2U)
#define LINE_SAI1_SD_B              PAL_LINE(GPIOE, 3U)
#define LINE_SAI1_FS_A              PAL_LINE(GPIOE, 4U)
#define LINE_SAI1_SCK_A             PAL_LINE(GPIOE, 5U)
#define LINE_SAI1_SD_A              PAL_LINE(GPIOE, 6U)
#define LINE_DIS13                  PAL_LINE(GPIOE, 7U)
#define LINE_DIS14                  PAL_LINE(GPIOE, 8U)
#define LINE_DIS15                  PAL_LINE(GPIOE, 9U)
#define LINE_DIS16                  PAL_LINE(GPIOE, 10U)
#define LINE_DIS17                  PAL_LINE(GPIOE, 11U)
#define LINE_DIS18                  PAL_LINE(GPIOE, 12U)
#define LINE_DIS19                  PAL_LINE(GPIOE, 13U)
#define LINE_DIS20                  PAL_LINE(GPIOE, 14U)
#define LINE_DIS21                  PAL_LINE(GPIOE, 15U)
#define LINE_I2C2_SDA               PAL_LINE(GPIOF, 0U)
#define LINE_I2C2_SCL               PAL_LINE(GPIOF, 1U)
#define LINE_ADC_EX2                PAL_LINE(GPIOF, 6U)
#define LINE_ADC_EX1                PAL_LINE(GPIOF, 7U)
#define LINE_DIS06                  PAL_LINE(GPIOF, 11U)
#define LINE_DIS07                  PAL_LINE(GPIOF, 12U)
#define LINE_DIS08                  PAL_LINE(GPIOF, 13U)
#define LINE_DIS09                  PAL_LINE(GPIOF, 14U)
#define LINE_DIS10                  PAL_LINE(GPIOF, 15U)
#define LINE_DIS11                  PAL_LINE(GPIOG, 0U)
#define LINE_DIS12                  PAL_LINE(GPIOG, 1U)
#define LINE_DIS42                  PAL_LINE(GPIOG, 2U)
#define LINE_DIS43                  PAL_LINE(GPIOG, 3U)
#define LINE_DIS44                  PAL_LINE(GPIOG, 4U)
#define LINE_DIS45                  PAL_LINE(GPIOG, 5U)
#define LINE_DIS46                  PAL_LINE(GPIOG, 6U)
#define LINE_DIS47                  PAL_LINE(GPIOG, 7U)
#define LINE_DIS48                  PAL_LINE(GPIOG, 8U)
#define LINE_SPI1_MISO              PAL_LINE(GPIOG, 9U)
#define LINE_SPI1_NSS               PAL_LINE(GPIOG, 10U)
#define LINE_SPI1_SCK               PAL_LINE(GPIOG, 11U)
#define LINE_VDD1                   PAL_LINE(GPIOG, 12U)
#define LINE_VDD2                   PAL_LINE(GPIOG, 13U)
#define LINE_DIS24                  PAL_LINE(GPIOH, 6U)
#define LINE_DIS25                  PAL_LINE(GPIOH, 7U)
#define LINE_DIS26                  PAL_LINE(GPIOH, 8U)
#define LINE_DIS27                  PAL_LINE(GPIOH, 9U)
#define LINE_DIS28                  PAL_LINE(GPIOH, 10U)
#define LINE_DIS29                  PAL_LINE(GPIOH, 11U)
#define LINE_DIS30                  PAL_LINE(GPIOH, 12U)
#define LINE_MOTION_INT1            PAL_LINE(GPIOH, 13U)
#define LINE_BUTTON_ALT             PAL_LINE(GPIOI, 0U)
#define LINE_BUTTON_DOWN            PAL_LINE(GPIOI, 1U)
#define LINE_BUTTON_UP              PAL_LINE(GPIOI, 2U)
#define LINE_BUTTON_PORT            PAL_LINE(GPIOI, 3U)
#define LINE_HP_EN                  PAL_LINE(GPIOI, 6U)
#define LINE_CODEC_EN               PAL_LINE(GPIOI, 7U)
#define LINE_LED1                   PAL_LINE(GPIOI, 9U)

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

/*
 * GPIOA setup:
 *
 * PA0  - PIN0                      (input pullup).
 * PA1  - PIN1                      (input pullup).
 * PA2  - PIN2                      (input pullup).
 * PA3  - ADC4                      (analog).
 * PA4  - ADC3                      (analog).
 * PA5  - ADC2                      (analog).
 * PA6  - ADC1                      (analog).
 * PA7  - DIS01                     (input pullup).
 * PA8  - I2C3_SCL                  (input floating).
 * PA9  - LED_DOWN2                 (output pushpull minimum).
 * PA10 - LED_UP2                   (output pushpull minimum).
 * PA11 - OTG_FS_DM                 (alternate 10).
 * PA12 - OTG_FS_DP                 (alternate 10).
 * PA13 - SWDIO                     (alternate 0).
 * PA14 - SWCLK                     (alternate 0).
 * PA15 - PIN15                     (input pullup).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOA_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOA_PIN2) |           \
                                     PIN_MODE_ANALOG(GPIOA_ADC4) |          \
                                     PIN_MODE_ANALOG(GPIOA_ADC3) |          \
                                     PIN_MODE_ANALOG(GPIOA_ADC2) |          \
                                     PIN_MODE_ANALOG(GPIOA_ADC1) |          \
                                     PIN_MODE_INPUT(GPIOA_DIS01) |          \
                                     PIN_MODE_INPUT(GPIOA_I2C3_SCL) |       \
                                     PIN_MODE_OUTPUT(GPIOA_LED_DOWN2) |     \
                                     PIN_MODE_OUTPUT(GPIOA_LED_UP2) |       \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DM) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DP) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |      \
                                     PIN_MODE_INPUT(GPIOA_PIN15))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ADC4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ADC3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ADC2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ADC1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_DIS01) |      \
                                     PIN_OTYPE_OPENDRAIN(GPIOA_I2C3_SCL) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LED_DOWN2) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LED_UP2) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DM) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DP) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN15))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOA_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOA_PIN1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOA_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOA_ADC4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOA_ADC3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOA_ADC2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOA_ADC1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOA_DIS01) |      \
                                     PIN_OSPEED_VERYLOW(GPIOA_I2C3_SCL) |   \
                                     PIN_OSPEED_VERYLOW(GPIOA_LED_DOWN2) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_LED_UP2) |    \
                                     PIN_OSPEED_HIGH(GPIOA_OTG_FS_DM) |     \
                                     PIN_OSPEED_HIGH(GPIOA_OTG_FS_DP) |     \
                                     PIN_OSPEED_HIGH(GPIOA_SWDIO) |         \
                                     PIN_OSPEED_HIGH(GPIOA_SWCLK) |         \
                                     PIN_OSPEED_VERYLOW(GPIOA_PIN15))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_PULLUP(GPIOA_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN2) |         \
                                     PIN_PUPDR_FLOATING(GPIOA_ADC4) |       \
                                     PIN_PUPDR_FLOATING(GPIOA_ADC3) |       \
                                     PIN_PUPDR_FLOATING(GPIOA_ADC2) |       \
                                     PIN_PUPDR_FLOATING(GPIOA_ADC1) |       \
                                     PIN_PUPDR_PULLUP(GPIOA_DIS01) |        \
                                     PIN_PUPDR_FLOATING(GPIOA_I2C3_SCL) |   \
                                     PIN_PUPDR_FLOATING(GPIOA_LED_DOWN2) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_LED_UP2) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DM) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DP) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_SWDIO) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_SWCLK) |      \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN15))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOA_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOA_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOA_ADC4) |             \
                                     PIN_ODR_HIGH(GPIOA_ADC3) |             \
                                     PIN_ODR_HIGH(GPIOA_ADC2) |             \
                                     PIN_ODR_HIGH(GPIOA_ADC1) |             \
                                     PIN_ODR_HIGH(GPIOA_DIS01) |            \
                                     PIN_ODR_HIGH(GPIOA_I2C3_SCL) |         \
                                     PIN_ODR_LOW(GPIOA_LED_DOWN2) |         \
                                     PIN_ODR_LOW(GPIOA_LED_UP2) |           \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DM) |        \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DP) |        \
                                     PIN_ODR_HIGH(GPIOA_SWDIO) |            \
                                     PIN_ODR_HIGH(GPIOA_SWCLK) |            \
                                     PIN_ODR_HIGH(GPIOA_PIN15))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_ADC4, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_ADC3, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_ADC2, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_ADC1, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_DIS01, 0U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_I2C3_SCL, 4U) |      \
                                     PIN_AFIO_AF(GPIOA_LED_DOWN2, 0U) |     \
                                     PIN_AFIO_AF(GPIOA_LED_UP2, 0U) |       \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DM, 10U) |    \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DP, 10U) |    \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_PIN15, 0U))

/*
 * GPIOB setup:
 *
 * PB0  - DIS04                     (input pullup).
 * PB1  - DIS05                     (input pullup).
 * PB2  - PIN2                      (input pullup).
 * PB3  - PIN3                      (input pullup).
 * PB4  - LED_DOWN                  (output pushpull minimum).
 * PB5  - LED_UP                    (output pushpull minimum).
 * PB6  - LED_R                     (output pushpull minimum).
 * PB7  - LED_B                     (output pushpull minimum).
 * PB8  - LED_G                     (output pushpull minimum).
 * PB9  - PIN9                      (input pullup).
 * PB10 - DIS22                     (input pullup).
 * PB11 - DIS23                     (input pullup).
 * PB12 - PIN12                     (input pullup).
 * PB13 - DIS31                     (input pullup).
 * PB14 - DIS32                     (input pullup).
 * PB15 - DIS33                     (input pullup).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(GPIOB_DIS04) |          \
                                     PIN_MODE_INPUT(GPIOB_DIS05) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN3) |           \
                                     PIN_MODE_OUTPUT(GPIOB_LED_DOWN) |      \
                                     PIN_MODE_OUTPUT(GPIOB_LED_UP) |        \
                                     PIN_MODE_OUTPUT(GPIOB_LED_R) |         \
                                     PIN_MODE_OUTPUT(GPIOB_LED_B) |         \
                                     PIN_MODE_OUTPUT(GPIOB_LED_G) |         \
                                     PIN_MODE_INPUT(GPIOB_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOB_DIS22) |          \
                                     PIN_MODE_INPUT(GPIOB_DIS23) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOB_DIS31) |          \
                                     PIN_MODE_INPUT(GPIOB_DIS32) |          \
                                     PIN_MODE_INPUT(GPIOB_DIS33))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_DIS04) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_DIS05) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LED_DOWN) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LED_UP) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LED_R) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LED_B) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LED_G) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_DIS22) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_DIS23) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_DIS31) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_DIS32) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_DIS33))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOB_DIS04) |      \
                                     PIN_OSPEED_VERYLOW(GPIOB_DIS05) |      \
                                     PIN_OSPEED_VERYLOW(GPIOB_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOB_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOB_LED_DOWN) |   \
                                     PIN_OSPEED_VERYLOW(GPIOB_LED_UP) |     \
                                     PIN_OSPEED_VERYLOW(GPIOB_LED_R) |      \
                                     PIN_OSPEED_VERYLOW(GPIOB_LED_B) |      \
                                     PIN_OSPEED_VERYLOW(GPIOB_LED_G) |      \
                                     PIN_OSPEED_VERYLOW(GPIOB_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOB_DIS22) |      \
                                     PIN_OSPEED_VERYLOW(GPIOB_DIS23) |      \
                                     PIN_OSPEED_VERYLOW(GPIOB_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOB_DIS31) |      \
                                     PIN_OSPEED_VERYLOW(GPIOB_DIS32) |      \
                                     PIN_OSPEED_VERYLOW(GPIOB_DIS33))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLUP(GPIOB_DIS04) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_DIS05) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN3) |         \
                                     PIN_PUPDR_FLOATING(GPIOB_LED_DOWN) |   \
                                     PIN_PUPDR_FLOATING(GPIOB_LED_UP) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_LED_R) |      \
                                     PIN_PUPDR_FLOATING(GPIOB_LED_B) |      \
                                     PIN_PUPDR_FLOATING(GPIOB_LED_G) |      \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOB_DIS22) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_DIS23) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_DIS31) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_DIS32) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_DIS33))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_DIS04) |            \
                                     PIN_ODR_HIGH(GPIOB_DIS05) |            \
                                     PIN_ODR_HIGH(GPIOB_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOB_PIN3) |             \
                                     PIN_ODR_LOW(GPIOB_LED_DOWN) |          \
                                     PIN_ODR_LOW(GPIOB_LED_UP) |            \
                                     PIN_ODR_HIGH(GPIOB_LED_R) |            \
                                     PIN_ODR_HIGH(GPIOB_LED_B) |            \
                                     PIN_ODR_HIGH(GPIOB_LED_G) |            \
                                     PIN_ODR_HIGH(GPIOB_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOB_DIS22) |            \
                                     PIN_ODR_HIGH(GPIOB_DIS23) |            \
                                     PIN_ODR_HIGH(GPIOB_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOB_DIS31) |            \
                                     PIN_ODR_HIGH(GPIOB_DIS32) |            \
                                     PIN_ODR_HIGH(GPIOB_DIS33))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_DIS04, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_DIS05, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOB_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOB_LED_DOWN, 0U) |      \
                                     PIN_AFIO_AF(GPIOB_LED_UP, 0U) |        \
                                     PIN_AFIO_AF(GPIOB_LED_R, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_LED_B, 0U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_LED_G, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOB_DIS22, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_DIS23, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_DIS31, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_DIS32, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_DIS33, 0U))

/*
 * GPIOC setup:
 *
 * PC0  - AUX_ADC_T                 (analog).
 * PC1  - AUX_ADC_R                 (analog).
 * PC2  - PIN2                      (input pullup).
 * PC3  - PIN3                      (input pullup).
 * PC4  - DIS02                     (input pullup).
 * PC5  - DIS03                     (input pullup).
 * PC6  - DIS49                     (input pullup).
 * PC7  - DIS50                     (input pullup).
 * PC8  - DIS51                     (input pullup).
 * PC9  - I2C3_SDA                  (input floating).
 * PC10 - UART3_TX                  (input pullup).
 * PC11 - UART3_RX                  (input pullup).
 * PC12 - PIN12                     (input pullup).
 * PC13 - PIN13                     (input pullup).
 * PC14 - PIN14                     (input pullup).
 * PC15 - PIN15                     (input pullup).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_ANALOG(GPIOC_AUX_ADC_T) |     \
                                     PIN_MODE_ANALOG(GPIOC_AUX_ADC_R) |     \
                                     PIN_MODE_INPUT(GPIOC_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOC_DIS02) |          \
                                     PIN_MODE_INPUT(GPIOC_DIS03) |          \
                                     PIN_MODE_INPUT(GPIOC_DIS49) |          \
                                     PIN_MODE_INPUT(GPIOC_DIS50) |          \
                                     PIN_MODE_INPUT(GPIOC_DIS51) |          \
                                     PIN_MODE_INPUT(GPIOC_I2C3_SDA) |       \
                                     PIN_MODE_INPUT(GPIOC_UART3_TX) |       \
                                     PIN_MODE_INPUT(GPIOC_UART3_RX) |       \
                                     PIN_MODE_INPUT(GPIOC_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOC_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOC_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOC_PIN15))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_AUX_ADC_T) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_AUX_ADC_R) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_DIS02) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_DIS03) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_DIS49) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_DIS50) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_DIS51) |      \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_I2C3_SDA) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_UART3_TX) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_UART3_RX) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN15))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOC_AUX_ADC_T) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_AUX_ADC_R) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOC_DIS02) |      \
                                     PIN_OSPEED_VERYLOW(GPIOC_DIS03) |      \
                                     PIN_OSPEED_VERYLOW(GPIOC_DIS49) |      \
                                     PIN_OSPEED_VERYLOW(GPIOC_DIS50) |      \
                                     PIN_OSPEED_VERYLOW(GPIOC_DIS51) |      \
                                     PIN_OSPEED_VERYLOW(GPIOC_I2C3_SDA) |   \
                                     PIN_OSPEED_VERYLOW(GPIOC_UART3_TX) |   \
                                     PIN_OSPEED_VERYLOW(GPIOC_UART3_RX) |   \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN15))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_AUX_ADC_T) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_AUX_ADC_R) |  \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_DIS02) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_DIS03) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_DIS49) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_DIS50) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_DIS51) |        \
                                     PIN_PUPDR_FLOATING(GPIOC_I2C3_SDA) |   \
                                     PIN_PUPDR_PULLUP(GPIOC_UART3_TX) |     \
                                     PIN_PUPDR_PULLUP(GPIOC_UART3_RX) |     \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN15))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_AUX_ADC_T) |        \
                                     PIN_ODR_HIGH(GPIOC_AUX_ADC_R) |        \
                                     PIN_ODR_HIGH(GPIOC_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOC_DIS02) |            \
                                     PIN_ODR_HIGH(GPIOC_DIS03) |            \
                                     PIN_ODR_HIGH(GPIOC_DIS49) |            \
                                     PIN_ODR_HIGH(GPIOC_DIS50) |            \
                                     PIN_ODR_HIGH(GPIOC_DIS51) |            \
                                     PIN_ODR_HIGH(GPIOC_I2C3_SDA) |         \
                                     PIN_ODR_HIGH(GPIOC_UART3_TX) |         \
                                     PIN_ODR_HIGH(GPIOC_UART3_RX) |         \
                                     PIN_ODR_HIGH(GPIOC_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOC_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOC_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOC_PIN15))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_AUX_ADC_T, 0U) |     \
                                     PIN_AFIO_AF(GPIOC_AUX_ADC_R, 0U) |     \
                                     PIN_AFIO_AF(GPIOC_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_DIS02, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_DIS03, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_DIS49, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_DIS50, 0U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_DIS51, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_I2C3_SDA, 4U) |      \
                                     PIN_AFIO_AF(GPIOC_UART3_TX, 0U) |      \
                                     PIN_AFIO_AF(GPIOC_UART3_RX, 0U) |      \
                                     PIN_AFIO_AF(GPIOC_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_PIN15, 0U))

/*
 * GPIOD setup:
 *
 * PD0  - PIN0                      (input pullup).
 * PD1  - PIN1                      (input pullup).
 * PD2  - PIN2                      (input pullup).
 * PD3  - JACK_DETECT               (input pullup).
 * PD4  - AUX_JACK_DETECT           (input pullup).
 * PD5  - AUX_UART2_TX              (output opendrain minimum).
 * PD6  - AUX_UART2_RX              (input floating).
 * PD7  - SPI1_MOSI                 (input pullup).
 * PD8  - DIS34                     (input pullup).
 * PD9  - DIS35                     (input pullup).
 * PD10 - DIS36                     (input pullup).
 * PD11 - DIS37                     (input pullup).
 * PD12 - DIS38                     (input pullup).
 * PD13 - DIS39                     (input pullup).
 * PD14 - DIS40                     (input pullup).
 * PD15 - DIS41                     (input pullup).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_INPUT(GPIOD_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOD_JACK_DETECT) |    \
                                     PIN_MODE_INPUT(GPIOD_AUX_JACK_DETECT) |\
                                     PIN_MODE_OUTPUT(GPIOD_AUX_UART2_TX) |  \
                                     PIN_MODE_INPUT(GPIOD_AUX_UART2_RX) |   \
                                     PIN_MODE_INPUT(GPIOD_SPI1_MOSI) |      \
                                     PIN_MODE_INPUT(GPIOD_DIS34) |          \
                                     PIN_MODE_INPUT(GPIOD_DIS35) |          \
                                     PIN_MODE_INPUT(GPIOD_DIS36) |          \
                                     PIN_MODE_INPUT(GPIOD_DIS37) |          \
                                     PIN_MODE_INPUT(GPIOD_DIS38) |          \
                                     PIN_MODE_INPUT(GPIOD_DIS39) |          \
                                     PIN_MODE_INPUT(GPIOD_DIS40) |          \
                                     PIN_MODE_INPUT(GPIOD_DIS41))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_JACK_DETECT) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_AUX_JACK_DETECT) |\
                                     PIN_OTYPE_OPENDRAIN(GPIOD_AUX_UART2_TX) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_AUX_UART2_RX) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_SPI1_MOSI) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_DIS34) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_DIS35) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_DIS36) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_DIS37) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_DIS38) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_DIS39) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_DIS40) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_DIS41))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOD_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_JACK_DETECT) |\
                                     PIN_OSPEED_VERYLOW(GPIOD_AUX_JACK_DETECT) |\
                                     PIN_OSPEED_VERYLOW(GPIOD_AUX_UART2_TX) |\
                                     PIN_OSPEED_VERYLOW(GPIOD_AUX_UART2_RX) |\
                                     PIN_OSPEED_VERYLOW(GPIOD_SPI1_MOSI) |  \
                                     PIN_OSPEED_VERYLOW(GPIOD_DIS34) |      \
                                     PIN_OSPEED_VERYLOW(GPIOD_DIS35) |      \
                                     PIN_OSPEED_VERYLOW(GPIOD_DIS36) |      \
                                     PIN_OSPEED_VERYLOW(GPIOD_DIS37) |      \
                                     PIN_OSPEED_VERYLOW(GPIOD_DIS38) |      \
                                     PIN_OSPEED_VERYLOW(GPIOD_DIS39) |      \
                                     PIN_OSPEED_VERYLOW(GPIOD_DIS40) |      \
                                     PIN_OSPEED_VERYLOW(GPIOD_DIS41))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_PULLUP(GPIOD_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_JACK_DETECT) |  \
                                     PIN_PUPDR_PULLUP(GPIOD_AUX_JACK_DETECT) |\
                                     PIN_PUPDR_FLOATING(GPIOD_AUX_UART2_TX) |\
                                     PIN_PUPDR_FLOATING(GPIOD_AUX_UART2_RX) |\
                                     PIN_PUPDR_PULLUP(GPIOD_SPI1_MOSI) |    \
                                     PIN_PUPDR_PULLUP(GPIOD_DIS34) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_DIS35) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_DIS36) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_DIS37) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_DIS38) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_DIS39) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_DIS40) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_DIS41))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOD_JACK_DETECT) |      \
                                     PIN_ODR_HIGH(GPIOD_AUX_JACK_DETECT) |  \
                                     PIN_ODR_LOW(GPIOD_AUX_UART2_TX) |      \
                                     PIN_ODR_HIGH(GPIOD_AUX_UART2_RX) |     \
                                     PIN_ODR_HIGH(GPIOD_SPI1_MOSI) |        \
                                     PIN_ODR_HIGH(GPIOD_DIS34) |            \
                                     PIN_ODR_HIGH(GPIOD_DIS35) |            \
                                     PIN_ODR_HIGH(GPIOD_DIS36) |            \
                                     PIN_ODR_HIGH(GPIOD_DIS37) |            \
                                     PIN_ODR_HIGH(GPIOD_DIS38) |            \
                                     PIN_ODR_HIGH(GPIOD_DIS39) |            \
                                     PIN_ODR_HIGH(GPIOD_DIS40) |            \
                                     PIN_ODR_HIGH(GPIOD_DIS41))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_JACK_DETECT, 0U) |   \
                                     PIN_AFIO_AF(GPIOD_AUX_JACK_DETECT, 0U) |\
                                     PIN_AFIO_AF(GPIOD_AUX_UART2_TX, 0U) |  \
                                     PIN_AFIO_AF(GPIOD_AUX_UART2_RX, 0U) |  \
                                     PIN_AFIO_AF(GPIOD_SPI1_MOSI, 0U))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_DIS34, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_DIS35, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_DIS36, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_DIS37, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_DIS38, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_DIS39, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_DIS40, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_DIS41, 0U))

/*
 * GPIOE setup:
 *
 * PE0  - AUX_VDD                   (output opendrain minimum).
 * PE1  - PIN1                      (input pullup).
 * PE2  - SAI1_MCLK_A               (input pullup).
 * PE3  - SAI1_SD_B                 (input pullup).
 * PE4  - SAI1_FS_A                 (input pullup).
 * PE5  - SAI1_SCK_A                (input pullup).
 * PE6  - SAI1_SD_A                 (input pullup).
 * PE7  - DIS13                     (input pullup).
 * PE8  - DIS14                     (input pullup).
 * PE9  - DIS15                     (input pullup).
 * PE10 - DIS16                     (input pullup).
 * PE11 - DIS17                     (input pullup).
 * PE12 - DIS18                     (input pullup).
 * PE13 - DIS19                     (input pullup).
 * PE14 - DIS20                     (input pullup).
 * PE15 - DIS21                     (input pullup).
 */
#define VAL_GPIOE_MODER             (PIN_MODE_OUTPUT(GPIOE_AUX_VDD) |       \
                                     PIN_MODE_INPUT(GPIOE_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOE_SAI1_MCLK_A) |    \
                                     PIN_MODE_INPUT(GPIOE_SAI1_SD_B) |      \
                                     PIN_MODE_INPUT(GPIOE_SAI1_FS_A) |      \
                                     PIN_MODE_INPUT(GPIOE_SAI1_SCK_A) |     \
                                     PIN_MODE_INPUT(GPIOE_SAI1_SD_A) |      \
                                     PIN_MODE_INPUT(GPIOE_DIS13) |          \
                                     PIN_MODE_INPUT(GPIOE_DIS14) |          \
                                     PIN_MODE_INPUT(GPIOE_DIS15) |          \
                                     PIN_MODE_INPUT(GPIOE_DIS16) |          \
                                     PIN_MODE_INPUT(GPIOE_DIS17) |          \
                                     PIN_MODE_INPUT(GPIOE_DIS18) |          \
                                     PIN_MODE_INPUT(GPIOE_DIS19) |          \
                                     PIN_MODE_INPUT(GPIOE_DIS20) |          \
                                     PIN_MODE_INPUT(GPIOE_DIS21))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOE_AUX_VDD) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SAI1_MCLK_A) |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_SAI1_SD_B) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SAI1_FS_A) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SAI1_SCK_A) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SAI1_SD_A) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_DIS13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_DIS14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_DIS15) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_DIS16) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_DIS17) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_DIS18) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_DIS19) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_DIS20) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_DIS21))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOE_AUX_VDD) |    \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_SAI1_MCLK_A) |\
                                     PIN_OSPEED_VERYLOW(GPIOE_SAI1_SD_B) |  \
                                     PIN_OSPEED_VERYLOW(GPIOE_SAI1_FS_A) |  \
                                     PIN_OSPEED_VERYLOW(GPIOE_SAI1_SCK_A) | \
                                     PIN_OSPEED_VERYLOW(GPIOE_SAI1_SD_A) |  \
                                     PIN_OSPEED_VERYLOW(GPIOE_DIS13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_DIS14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_DIS15) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_DIS16) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_DIS17) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_DIS18) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_DIS19) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_DIS20) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_DIS21))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_FLOATING(GPIOE_AUX_VDD) |    \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_SAI1_MCLK_A) |  \
                                     PIN_PUPDR_PULLUP(GPIOE_SAI1_SD_B) |    \
                                     PIN_PUPDR_PULLUP(GPIOE_SAI1_FS_A) |    \
                                     PIN_PUPDR_PULLUP(GPIOE_SAI1_SCK_A) |   \
                                     PIN_PUPDR_PULLUP(GPIOE_SAI1_SD_A) |    \
                                     PIN_PUPDR_PULLUP(GPIOE_DIS13) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_DIS14) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_DIS15) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_DIS16) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_DIS17) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_DIS18) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_DIS19) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_DIS20) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_DIS21))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_AUX_VDD) |          \
                                     PIN_ODR_HIGH(GPIOE_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOE_SAI1_MCLK_A) |      \
                                     PIN_ODR_HIGH(GPIOE_SAI1_SD_B) |        \
                                     PIN_ODR_HIGH(GPIOE_SAI1_FS_A) |        \
                                     PIN_ODR_HIGH(GPIOE_SAI1_SCK_A) |       \
                                     PIN_ODR_HIGH(GPIOE_SAI1_SD_A) |        \
                                     PIN_ODR_HIGH(GPIOE_DIS13) |            \
                                     PIN_ODR_HIGH(GPIOE_DIS14) |            \
                                     PIN_ODR_HIGH(GPIOE_DIS15) |            \
                                     PIN_ODR_HIGH(GPIOE_DIS16) |            \
                                     PIN_ODR_HIGH(GPIOE_DIS17) |            \
                                     PIN_ODR_HIGH(GPIOE_DIS18) |            \
                                     PIN_ODR_HIGH(GPIOE_DIS19) |            \
                                     PIN_ODR_HIGH(GPIOE_DIS20) |            \
                                     PIN_ODR_HIGH(GPIOE_DIS21))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_AUX_VDD, 0U) |       \
                                     PIN_AFIO_AF(GPIOE_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_SAI1_MCLK_A, 0U) |   \
                                     PIN_AFIO_AF(GPIOE_SAI1_SD_B, 0U) |     \
                                     PIN_AFIO_AF(GPIOE_SAI1_FS_A, 0U) |     \
                                     PIN_AFIO_AF(GPIOE_SAI1_SCK_A, 0U) |    \
                                     PIN_AFIO_AF(GPIOE_SAI1_SD_A, 0U) |     \
                                     PIN_AFIO_AF(GPIOE_DIS13, 0U))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_DIS14, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_DIS15, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_DIS16, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_DIS17, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_DIS18, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_DIS19, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_DIS20, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_DIS21, 0U))

/*
 * GPIOF setup:
 *
 * PF0  - I2C2_SDA                  (input floating).
 * PF1  - I2C2_SCL                  (input floating).
 * PF2  - PIN2                      (input pullup).
 * PF3  - PIN3                      (input pullup).
 * PF4  - PIN4                      (input pullup).
 * PF5  - PIN5                      (input pullup).
 * PF6  - ADC_EX2                   (input pullup).
 * PF7  - ADC_EX1                   (input pullup).
 * PF8  - PIN8                      (input pullup).
 * PF9  - PIN9                      (input pullup).
 * PF10 - PIN10                     (input pullup).
 * PF11 - DIS06                     (input pullup).
 * PF12 - DIS07                     (input pullup).
 * PF13 - DIS08                     (input pullup).
 * PF14 - DIS09                     (input pullup).
 * PF15 - DIS10                     (input pullup).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_INPUT(GPIOF_I2C2_SDA) |       \
                                     PIN_MODE_INPUT(GPIOF_I2C2_SCL) |       \
                                     PIN_MODE_INPUT(GPIOF_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOF_ADC_EX2) |        \
                                     PIN_MODE_INPUT(GPIOF_ADC_EX1) |        \
                                     PIN_MODE_INPUT(GPIOF_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOF_DIS06) |          \
                                     PIN_MODE_INPUT(GPIOF_DIS07) |          \
                                     PIN_MODE_INPUT(GPIOF_DIS08) |          \
                                     PIN_MODE_INPUT(GPIOF_DIS09) |          \
                                     PIN_MODE_INPUT(GPIOF_DIS10))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOF_I2C2_SDA) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_I2C2_SCL) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_ADC_EX2) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOF_ADC_EX1) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_DIS06) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_DIS07) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_DIS08) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_DIS09) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_DIS10))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOF_I2C2_SDA) |   \
                                     PIN_OSPEED_VERYLOW(GPIOF_I2C2_SCL) |   \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_ADC_EX2) |    \
                                     PIN_OSPEED_VERYLOW(GPIOF_ADC_EX1) |    \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_DIS06) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_DIS07) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_DIS08) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_DIS09) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_DIS10))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_I2C2_SDA) |   \
                                     PIN_PUPDR_FLOATING(GPIOF_I2C2_SCL) |   \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_ADC_EX2) |      \
                                     PIN_PUPDR_PULLUP(GPIOF_ADC_EX1) |      \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_DIS06) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_DIS07) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_DIS08) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_DIS09) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_DIS10))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_I2C2_SDA) |         \
                                     PIN_ODR_HIGH(GPIOF_I2C2_SCL) |         \
                                     PIN_ODR_HIGH(GPIOF_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOF_ADC_EX2) |          \
                                     PIN_ODR_HIGH(GPIOF_ADC_EX1) |          \
                                     PIN_ODR_HIGH(GPIOF_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOF_DIS06) |            \
                                     PIN_ODR_HIGH(GPIOF_DIS07) |            \
                                     PIN_ODR_HIGH(GPIOF_DIS08) |            \
                                     PIN_ODR_HIGH(GPIOF_DIS09) |            \
                                     PIN_ODR_HIGH(GPIOF_DIS10))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_I2C2_SDA, 4U) |      \
                                     PIN_AFIO_AF(GPIOF_I2C2_SCL, 4U) |      \
                                     PIN_AFIO_AF(GPIOF_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_ADC_EX2, 0U) |       \
                                     PIN_AFIO_AF(GPIOF_ADC_EX1, 0U))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_DIS06, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_DIS07, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_DIS08, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_DIS09, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_DIS10, 0U))

/*
 * GPIOG setup:
 *
 * PG0  - DIS11                     (input pullup).
 * PG1  - DIS12                     (input pullup).
 * PG2  - DIS42                     (input pullup).
 * PG3  - DIS43                     (input pullup).
 * PG4  - DIS44                     (input pullup).
 * PG5  - DIS45                     (input pullup).
 * PG6  - DIS46                     (input pullup).
 * PG7  - DIS47                     (input pullup).
 * PG8  - DIS48                     (input pullup).
 * PG9  - SPI1_MISO                 (input pullup).
 * PG10 - SPI1_NSS                  (input pullup).
 * PG11 - SPI1_SCK                  (input pullup).
 * PG12 - VDD1                      (input floating).
 * PG13 - VDD2                      (input floating).
 * PG14 - PIN14                     (input pullup).
 * PG15 - PIN15                     (input pullup).
 */
#define VAL_GPIOG_MODER             (PIN_MODE_INPUT(GPIOG_DIS11) |          \
                                     PIN_MODE_INPUT(GPIOG_DIS12) |          \
                                     PIN_MODE_INPUT(GPIOG_DIS42) |          \
                                     PIN_MODE_INPUT(GPIOG_DIS43) |          \
                                     PIN_MODE_INPUT(GPIOG_DIS44) |          \
                                     PIN_MODE_INPUT(GPIOG_DIS45) |          \
                                     PIN_MODE_INPUT(GPIOG_DIS46) |          \
                                     PIN_MODE_INPUT(GPIOG_DIS47) |          \
                                     PIN_MODE_INPUT(GPIOG_DIS48) |          \
                                     PIN_MODE_INPUT(GPIOG_SPI1_MISO) |      \
                                     PIN_MODE_INPUT(GPIOG_SPI1_NSS) |       \
                                     PIN_MODE_INPUT(GPIOG_SPI1_SCK) |       \
                                     PIN_MODE_INPUT(GPIOG_VDD1) |           \
                                     PIN_MODE_INPUT(GPIOG_VDD2) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN15))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOG_DIS11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_DIS12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_DIS42) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_DIS43) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_DIS44) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_DIS45) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_DIS46) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_DIS47) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_DIS48) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_SPI1_MISO) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_SPI1_NSS) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOG_SPI1_SCK) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOG_VDD1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_VDD2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN15))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOG_DIS11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_DIS12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_DIS42) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_DIS43) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_DIS44) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_DIS45) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_DIS46) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_DIS47) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_DIS48) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_SPI1_MISO) |  \
                                     PIN_OSPEED_VERYLOW(GPIOG_SPI1_NSS) |   \
                                     PIN_OSPEED_VERYLOW(GPIOG_SPI1_SCK) |   \
                                     PIN_OSPEED_VERYLOW(GPIOG_VDD1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_VDD2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN15))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_PULLUP(GPIOG_DIS11) |        \
                                     PIN_PUPDR_PULLUP(GPIOG_DIS12) |        \
                                     PIN_PUPDR_PULLUP(GPIOG_DIS42) |        \
                                     PIN_PUPDR_PULLUP(GPIOG_DIS43) |        \
                                     PIN_PUPDR_PULLUP(GPIOG_DIS44) |        \
                                     PIN_PUPDR_PULLUP(GPIOG_DIS45) |        \
                                     PIN_PUPDR_PULLUP(GPIOG_DIS46) |        \
                                     PIN_PUPDR_PULLUP(GPIOG_DIS47) |        \
                                     PIN_PUPDR_PULLUP(GPIOG_DIS48) |        \
                                     PIN_PUPDR_PULLUP(GPIOG_SPI1_MISO) |    \
                                     PIN_PUPDR_PULLUP(GPIOG_SPI1_NSS) |     \
                                     PIN_PUPDR_PULLUP(GPIOG_SPI1_SCK) |     \
                                     PIN_PUPDR_FLOATING(GPIOG_VDD1) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_VDD2) |       \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN15))
#define VAL_GPIOG_ODR               (PIN_ODR_HIGH(GPIOG_DIS11) |            \
                                     PIN_ODR_HIGH(GPIOG_DIS12) |            \
                                     PIN_ODR_HIGH(GPIOG_DIS42) |            \
                                     PIN_ODR_HIGH(GPIOG_DIS43) |            \
                                     PIN_ODR_HIGH(GPIOG_DIS44) |            \
                                     PIN_ODR_HIGH(GPIOG_DIS45) |            \
                                     PIN_ODR_HIGH(GPIOG_DIS46) |            \
                                     PIN_ODR_HIGH(GPIOG_DIS47) |            \
                                     PIN_ODR_HIGH(GPIOG_DIS48) |            \
                                     PIN_ODR_HIGH(GPIOG_SPI1_MISO) |        \
                                     PIN_ODR_HIGH(GPIOG_SPI1_NSS) |         \
                                     PIN_ODR_HIGH(GPIOG_SPI1_SCK) |         \
                                     PIN_ODR_HIGH(GPIOG_VDD1) |             \
                                     PIN_ODR_HIGH(GPIOG_VDD2) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN15))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_DIS11, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_DIS12, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_DIS42, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_DIS43, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_DIS44, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_DIS45, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_DIS46, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_DIS47, 0U))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_DIS48, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_SPI1_MISO, 0U) |     \
                                     PIN_AFIO_AF(GPIOG_SPI1_NSS, 0U) |      \
                                     PIN_AFIO_AF(GPIOG_SPI1_SCK, 0U) |      \
                                     PIN_AFIO_AF(GPIOG_VDD1, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_VDD2, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN15, 0U))

/*
 * GPIOH setup:
 *
 * PH0  - PIN0                      (input pullup).
 * PH1  - PIN1                      (input pullup).
 * PH2  - PIN2                      (input pullup).
 * PH3  - PIN3                      (input pullup).
 * PH4  - PIN4                      (input pullup).
 * PH5  - PIN5                      (input pullup).
 * PH6  - DIS24                     (input pullup).
 * PH7  - DIS25                     (input pullup).
 * PH8  - DIS26                     (input pullup).
 * PH9  - DIS27                     (input pullup).
 * PH10 - DIS28                     (input pullup).
 * PH11 - DIS29                     (input pullup).
 * PH12 - DIS30                     (input pullup).
 * PH13 - MOTION_INT1               (input pulldown).
 * PH14 - PIN14                     (input pullup).
 * PH15 - PIN15                     (input pullup).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOH_DIS24) |          \
                                     PIN_MODE_INPUT(GPIOH_DIS25) |          \
                                     PIN_MODE_INPUT(GPIOH_DIS26) |          \
                                     PIN_MODE_INPUT(GPIOH_DIS27) |          \
                                     PIN_MODE_INPUT(GPIOH_DIS28) |          \
                                     PIN_MODE_INPUT(GPIOH_DIS29) |          \
                                     PIN_MODE_INPUT(GPIOH_DIS30) |          \
                                     PIN_MODE_INPUT(GPIOH_MOTION_INT1) |    \
                                     PIN_MODE_INPUT(GPIOH_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN15))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_DIS24) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_DIS25) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_DIS26) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_DIS27) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_DIS28) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_DIS29) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_DIS30) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_MOTION_INT1) |\
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN15))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOH_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_DIS24) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_DIS25) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_DIS26) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_DIS27) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_DIS28) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_DIS29) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_DIS30) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_MOTION_INT1) |\
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN15))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_PULLUP(GPIOH_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_DIS24) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_DIS25) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_DIS26) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_DIS27) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_DIS28) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_DIS29) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_DIS30) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOH_MOTION_INT1) |\
                                     PIN_PUPDR_PULLUP(GPIOH_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN15))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOH_DIS24) |            \
                                     PIN_ODR_HIGH(GPIOH_DIS25) |            \
                                     PIN_ODR_HIGH(GPIOH_DIS26) |            \
                                     PIN_ODR_HIGH(GPIOH_DIS27) |            \
                                     PIN_ODR_HIGH(GPIOH_DIS28) |            \
                                     PIN_ODR_HIGH(GPIOH_DIS29) |            \
                                     PIN_ODR_HIGH(GPIOH_DIS30) |            \
                                     PIN_ODR_HIGH(GPIOH_MOTION_INT1) |      \
                                     PIN_ODR_HIGH(GPIOH_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN15))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_DIS24, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_DIS25, 0U))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_DIS26, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_DIS27, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_DIS28, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_DIS29, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_DIS30, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_MOTION_INT1, 0U) |   \
                                     PIN_AFIO_AF(GPIOH_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN15, 0U))

/*
 * GPIOI setup:
 *
 * PI0  - BUTTON_ALT                (input pullup).
 * PI1  - BUTTON_DOWN               (input pullup).
 * PI2  - BUTTON_UP                 (input pulldown).
 * PI3  - BUTTON_PORT               (input pullup).
 * PI4  - PIN4                      (input pullup).
 * PI5  - PIN5                      (input pullup).
 * PI6  - HP_EN                     (output pushpull minimum).
 * PI7  - CODEC_EN                  (output pushpull minimum).
 * PI8  - PIN8                      (input pullup).
 * PI9  - LED1                      (output pushpull minimum).
 * PI10 - PIN10                     (input pullup).
 * PI11 - PIN11                     (input pullup).
 * PI12 - PIN12                     (input pullup).
 * PI13 - PIN13                     (input pullup).
 * PI14 - PIN14                     (input pullup).
 * PI15 - PIN15                     (input pullup).
 */
#define VAL_GPIOI_MODER             (PIN_MODE_INPUT(GPIOI_BUTTON_ALT) |     \
                                     PIN_MODE_INPUT(GPIOI_BUTTON_DOWN) |    \
                                     PIN_MODE_INPUT(GPIOI_BUTTON_UP) |      \
                                     PIN_MODE_INPUT(GPIOI_BUTTON_PORT) |    \
                                     PIN_MODE_INPUT(GPIOI_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN5) |           \
                                     PIN_MODE_OUTPUT(GPIOI_HP_EN) |         \
                                     PIN_MODE_OUTPUT(GPIOI_CODEC_EN) |      \
                                     PIN_MODE_INPUT(GPIOI_PIN8) |           \
                                     PIN_MODE_OUTPUT(GPIOI_LED1) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN15))
#define VAL_GPIOI_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOI_BUTTON_ALT) | \
                                     PIN_OTYPE_PUSHPULL(GPIOI_BUTTON_DOWN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOI_BUTTON_UP) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOI_BUTTON_PORT) |\
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_HP_EN) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_CODEC_EN) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_LED1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN15))
#define VAL_GPIOI_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOI_BUTTON_ALT) | \
                                     PIN_OSPEED_VERYLOW(GPIOI_BUTTON_DOWN) |\
                                     PIN_OSPEED_VERYLOW(GPIOI_BUTTON_UP) |  \
                                     PIN_OSPEED_VERYLOW(GPIOI_BUTTON_PORT) |\
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_HP_EN) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_CODEC_EN) |   \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_LED1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN15))
#define VAL_GPIOI_PUPDR             (PIN_PUPDR_PULLUP(GPIOI_BUTTON_ALT) |   \
                                     PIN_PUPDR_PULLUP(GPIOI_BUTTON_DOWN) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOI_BUTTON_UP) |  \
                                     PIN_PUPDR_PULLUP(GPIOI_BUTTON_PORT) |  \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN5) |         \
                                     PIN_PUPDR_FLOATING(GPIOI_HP_EN) |      \
                                     PIN_PUPDR_FLOATING(GPIOI_CODEC_EN) |   \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN8) |         \
                                     PIN_PUPDR_FLOATING(GPIOI_LED1) |       \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN15))
#define VAL_GPIOI_ODR               (PIN_ODR_HIGH(GPIOI_BUTTON_ALT) |       \
                                     PIN_ODR_HIGH(GPIOI_BUTTON_DOWN) |      \
                                     PIN_ODR_HIGH(GPIOI_BUTTON_UP) |        \
                                     PIN_ODR_HIGH(GPIOI_BUTTON_PORT) |      \
                                     PIN_ODR_HIGH(GPIOI_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN5) |             \
                                     PIN_ODR_LOW(GPIOI_HP_EN) |             \
                                     PIN_ODR_LOW(GPIOI_CODEC_EN) |          \
                                     PIN_ODR_HIGH(GPIOI_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOI_LED1) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN15))
#define VAL_GPIOI_AFRL              (PIN_AFIO_AF(GPIOI_BUTTON_ALT, 0U) |    \
                                     PIN_AFIO_AF(GPIOI_BUTTON_DOWN, 0U) |   \
                                     PIN_AFIO_AF(GPIOI_BUTTON_UP, 0U) |     \
                                     PIN_AFIO_AF(GPIOI_BUTTON_PORT, 0U) |   \
                                     PIN_AFIO_AF(GPIOI_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_HP_EN, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_CODEC_EN, 0U))
#define VAL_GPIOI_AFRH              (PIN_AFIO_AF(GPIOI_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_LED1, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN15, 0U))

/*
 * GPIOJ setup:
 *
 * PJ0  - PIN0                      (input pullup).
 * PJ1  - PIN1                      (input pullup).
 * PJ2  - PIN2                      (input pullup).
 * PJ3  - PIN3                      (input pullup).
 * PJ4  - PIN4                      (input pullup).
 * PJ5  - PIN5                      (input pullup).
 * PJ6  - PIN6                      (input pullup).
 * PJ7  - PIN7                      (input pullup).
 * PJ8  - PIN8                      (input pullup).
 * PJ9  - PIN9                      (input pullup).
 * PJ10 - PIN10                     (input pullup).
 * PJ11 - PIN11                     (input pullup).
 * PJ12 - PIN12                     (input pullup).
 * PJ13 - PIN13                     (input pullup).
 * PJ14 - PIN14                     (input pullup).
 * PJ15 - PIN15                     (input pullup).
 */
#define VAL_GPIOJ_MODER             (PIN_MODE_INPUT(GPIOJ_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOJ_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOJ_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOJ_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOJ_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOJ_PIN15))
#define VAL_GPIOJ_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOJ_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN15))
#define VAL_GPIOJ_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOJ_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN15))
#define VAL_GPIOJ_PUPDR             (PIN_PUPDR_PULLUP(GPIOJ_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN15))
#define VAL_GPIOJ_ODR               (PIN_ODR_HIGH(GPIOJ_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOJ_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOJ_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOJ_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOJ_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOJ_PIN15))
#define VAL_GPIOJ_AFRL              (PIN_AFIO_AF(GPIOJ_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN7, 0U))
#define VAL_GPIOJ_AFRH              (PIN_AFIO_AF(GPIOJ_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOJ_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOJ_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOJ_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOJ_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOJ_PIN15, 0U))

/*
 * GPIOK setup:
 *
 * PK0  - PIN0                      (input pullup).
 * PK1  - PIN1                      (input pullup).
 * PK2  - PIN2                      (input pullup).
 * PK3  - PIN3                      (input pullup).
 * PK4  - PIN4                      (input pullup).
 * PK5  - PIN5                      (input pullup).
 * PK6  - PIN6                      (input pullup).
 * PK7  - PIN7                      (input pullup).
 * PK8  - PIN8                      (input pullup).
 * PK9  - PIN9                      (input pullup).
 * PK10 - PIN10                     (input pullup).
 * PK11 - PIN11                     (input pullup).
 * PK12 - PIN12                     (input pullup).
 * PK13 - PIN13                     (input pullup).
 * PK14 - PIN14                     (input pullup).
 * PK15 - PIN15                     (input pullup).
 */
#define VAL_GPIOK_MODER             (PIN_MODE_INPUT(GPIOK_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOK_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOK_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOK_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOK_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOK_PIN15))
#define VAL_GPIOK_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOK_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN15))
#define VAL_GPIOK_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOK_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN15))
#define VAL_GPIOK_PUPDR             (PIN_PUPDR_PULLUP(GPIOK_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN15))
#define VAL_GPIOK_ODR               (PIN_ODR_HIGH(GPIOK_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOK_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOK_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOK_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOK_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOK_PIN15))
#define VAL_GPIOK_AFRL              (PIN_AFIO_AF(GPIOK_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN7, 0U))
#define VAL_GPIOK_AFRH              (PIN_AFIO_AF(GPIOK_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOK_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOK_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOK_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOK_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOK_PIN15, 0U))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_H */
