/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
 * Board frequencies.
 * NOTE: The LSE crystal is not fitted by default on the board.
 */
#define STM32_LSECLK            0
#define STM32_HSECLK            8000000

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD               300

/*
 * MCU type as defined in the ST header file stm32f4xx.h.
 */
#define STM32F40_41xxx

/*
 * USB VBUSSENS pin is not connected.
 */
#define BOARD_OTG_NOVBUSSENS

/*
 * IO pins assignments.
 */
#define GPIOH_LED1              15
#define GPIOA_OTG_FS_DM         11
#define GPIOA_OTG_FS_DP         12
#define GPIOA_UART1_TX          9
#define GPIOA_UART1_RX          10
#define GPIOA_I2C3_SCL          8
#define GPIOC_I2C3_SDA          9
#define GPIOG_BUTTON            15
#define GPIOI_SPI2_SCK          1
#define GPIOI_SPI2_MISO         2
#define GPIOI_SPI2_MOSI         3

#define GPIOH_OSC_IN            0
#define GPIOH_OSC_OUT           1

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_2M(n)            (0U << ((n) * 2))
#define PIN_OSPEED_25M(n)           (1U << ((n) * 2))
#define PIN_OSPEED_50M(n)           (2U << ((n) * 2))
#define PIN_OSPEED_100M(n)          (3U << ((n) * 2))
#define PIN_PUDR_FLOATING(n)        (0U << ((n) * 2))
#define PIN_PUDR_PULLUP(n)          (1U << ((n) * 2))
#define PIN_PUDR_PULLDOWN(n)        (2U << ((n) * 2))
#define PIN_AFIO_AF(n, v)           ((v##U) << ((n % 8) * 4))

/*
 * Port A setup.
 * All input with pull-up except:
 * PA11 - GPIOA_OTG_FS_DM       (alternate 10).
 * PA12 - GPIOA_OTG_FS_DP       (alternate 10).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(0) |         \
                                     PIN_MODE_INPUT(1) |                    \
                                     PIN_MODE_INPUT(2) |                    \
                                     PIN_MODE_INPUT(3) |                    \
                                     PIN_MODE_INPUT(4) |                    \
                                     PIN_MODE_INPUT(5) |                    \
                                     PIN_MODE_INPUT(6) |                    \
                                     PIN_MODE_INPUT(7) |                    \
                                     PIN_MODE_INPUT(8) |                    \
                                     PIN_MODE_INPUT(9) |                    \
                                     PIN_MODE_INPUT(10) |          \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DM) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DP) |  \
                                     PIN_MODE_INPUT(13) |                   \
                                     PIN_MODE_INPUT(14) |                   \
                                     PIN_MODE_INPUT(15))
#define VAL_GPIOA_OTYPER            0x00000000
#define VAL_GPIOA_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOA_PUPDR             (PIN_PUDR_PULLUP(0) |      \
                                     PIN_PUDR_PULLUP(1) |                   \
                                     PIN_PUDR_PULLUP(2) |                   \
                                     PIN_PUDR_PULLUP(3) |                   \
                                     PIN_PUDR_PULLUP(4) |                   \
                                     PIN_PUDR_PULLUP(5) |                   \
                                     PIN_PUDR_PULLUP(6) |                   \
                                     PIN_PUDR_PULLUP(7) |                   \
                                     PIN_PUDR_PULLUP(8) |                   \
                                     PIN_PUDR_PULLUP(9) |                   \
                                     PIN_PUDR_PULLUP(10) |                   \
                                     PIN_PUDR_FLOATING(GPIOA_OTG_FS_DM) |   \
                                     PIN_PUDR_FLOATING(GPIOA_OTG_FS_DP) |   \
                                     PIN_PUDR_PULLUP(13) |                   \
                                     PIN_PUDR_PULLUP(14) |                   \
                                     PIN_PUDR_PULLUP(15))
#define VAL_GPIOA_ODR               0xFFFFFFFF
#define VAL_GPIOA_AFRL              0x00000000
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_OTG_FS_DM, 10) |     \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DP, 10))

/*
 * Port B setup.
 * All input with pull-up except:
 * PB2 (BOOT1)  - Tied to GROUND        (input floating).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(0) |                    \
                                     PIN_MODE_INPUT(1) |                    \
                                     PIN_MODE_INPUT(2) |                    \
                                     PIN_MODE_INPUT(3) |                    \
                                     PIN_MODE_INPUT(4) |                    \
                                     PIN_MODE_INPUT(5) |                    \
                                     PIN_MODE_INPUT(6) |                    \
                                     PIN_MODE_INPUT(7) |                    \
                                     PIN_MODE_INPUT(8) |                    \
                                     PIN_MODE_INPUT(9) |                    \
                                     PIN_MODE_INPUT(10) |                   \
                                     PIN_MODE_INPUT(11) |                   \
                                     PIN_MODE_INPUT(12) |                   \
                                     PIN_MODE_INPUT(13) |                   \
                                     PIN_MODE_INPUT(14) |                   \
                                     PIN_MODE_INPUT(15))
#define VAL_GPIOB_OTYPER            0x00000000
#define VAL_GPIOB_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOB_PUPDR             (PIN_PUDR_PULLUP(0) |                   \
                                     PIN_PUDR_PULLUP(1) |                   \
                                     PIN_PUDR_FLOATING(2) |                 \
                                     PIN_PUDR_PULLUP(3) |                   \
                                     PIN_PUDR_PULLUP(4) |                   \
                                     PIN_PUDR_PULLUP(5) |                   \
                                     PIN_PUDR_PULLUP(6) |                   \
                                     PIN_PUDR_PULLUP(7) |                   \
                                     PIN_PUDR_PULLUP(8) |                   \
                                     PIN_PUDR_PULLUP(9) |                   \
                                     PIN_PUDR_PULLUP(10) |                  \
                                     PIN_PUDR_PULLUP(11) |                  \
                                     PIN_PUDR_PULLUP(12) |                  \
                                     PIN_PUDR_PULLUP(13) |                  \
                                     PIN_PUDR_PULLUP(14) |                  \
                                     PIN_PUDR_PULLUP(15))
#define VAL_GPIOB_ODR               0xFFFFFFFF
#define VAL_GPIOB_AFRL              0x00000000
#define VAL_GPIOB_AFRH              0x00000000

/*
 * Port C setup.
 * All input with pull-up except:
 */
#define VAL_GPIOC_MODER             0x00000000
#define VAL_GPIOC_OTYPER            0x00000000
#define VAL_GPIOC_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOC_PUPDR             0xFFFFFFFF
#define VAL_GPIOC_ODR               0xFFFFFFFF
#define VAL_GPIOC_AFRL              0x00000000
#define VAL_GPIOC_AFRH              0x00000000

/*
 * Port D setup.
 * All input with pull-up except:
 */
#define VAL_GPIOD_MODER             0x00000000
#define VAL_GPIOD_OTYPER            0x00000000
#define VAL_GPIOD_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOD_PUPDR             0xFFFFFFFF
#define VAL_GPIOD_ODR               0xFFFFFFFF
#define VAL_GPIOD_AFRL              0x00000000
#define VAL_GPIOD_AFRH              0x00000000

/*
 * Port E setup.
 * All input with pull-up except:
 */
#define VAL_GPIOE_MODER             0x00000000
#define VAL_GPIOE_OTYPER            0x00000000
#define VAL_GPIOE_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOE_PUPDR             0xFFFFFFFF
#define VAL_GPIOE_ODR               0xFFFFFFFF
#define VAL_GPIOE_AFRL              0x00000000
#define VAL_GPIOE_AFRH              0x00000000

/*
 * Port F setup.
 * All input with pull-up except:
 * PF0  - GPIOF_SCL             (alternate 4).
 * PF1  - GPIOF_SDA             (alternate 4).
 */
#define VAL_GPIOF_MODER             0x00000000
#define VAL_GPIOF_OTYPER            0x00000000
#define VAL_GPIOF_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOF_PUPDR             0xFFFFFFFF
#define VAL_GPIOF_ODR               0xFFFFFFFF
#define VAL_GPIOF_AFRL              0x00000000
#define VAL_GPIOF_AFRH              0x00000000

/*
 * Port G setup.
 * All input with pull-up.
 */
#define VAL_GPIOG_MODER             0x00000000
#define VAL_GPIOG_OTYPER            0x00000000
#define VAL_GPIOG_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOG_PUPDR             0xFFFFFFFF
#define VAL_GPIOG_ODR               0xFFFFFFFF
#define VAL_GPIOG_AFRL              0x00000000
#define VAL_GPIOG_AFRH              0x00000000

/*
 * Port H setup.
 * All input with pull-up except:
 * PH0  - GPIOH_OSC_IN          (input floating).
 * PH1  - GPIOH_OSC_OUT         (input floating).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) |         \
                                     PIN_MODE_INPUT(GPIOH_OSC_OUT) |        \
                                     PIN_MODE_INPUT(2) |                    \
                                     PIN_MODE_INPUT(3) |                    \
                                     PIN_MODE_INPUT(4) |                    \
                                     PIN_MODE_INPUT(5) |                    \
                                     PIN_MODE_INPUT(6) |                    \
                                     PIN_MODE_INPUT(7) |                    \
                                     PIN_MODE_INPUT(8) |                    \
                                     PIN_MODE_INPUT(9) |                    \
                                     PIN_MODE_INPUT(10) |                   \
                                     PIN_MODE_INPUT(11) |                   \
                                     PIN_MODE_INPUT(12) |                   \
                                     PIN_MODE_INPUT(13) |                   \
                                     PIN_MODE_INPUT(14) |                   \
                                     PIN_MODE_INPUT(15))
#define VAL_GPIOH_OTYPER            0x00000000
#define VAL_GPIOH_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOH_PUPDR             (PIN_PUDR_FLOATING(GPIOH_OSC_IN) |      \
                                     PIN_PUDR_FLOATING(GPIOH_OSC_OUT) |     \
                                     PIN_PUDR_PULLUP(2) |                   \
                                     PIN_PUDR_PULLUP(3) |                   \
                                     PIN_PUDR_PULLUP(4) |                   \
                                     PIN_PUDR_PULLUP(5) |                   \
                                     PIN_PUDR_PULLUP(6) |                   \
                                     PIN_PUDR_PULLUP(7) |                   \
                                     PIN_PUDR_PULLUP(8) |                   \
                                     PIN_PUDR_PULLUP(9) |                   \
                                     PIN_PUDR_PULLUP(10) |                  \
                                     PIN_PUDR_PULLUP(11) |                  \
                                     PIN_PUDR_PULLUP(12) |                  \
                                     PIN_PUDR_PULLUP(13) |                  \
                                     PIN_PUDR_PULLUP(14) |                  \
                                     PIN_PUDR_PULLUP(15))
#define VAL_GPIOH_ODR               0xFFFFFFFF
#define VAL_GPIOH_AFRL              0x00000000
#define VAL_GPIOH_AFRH              0x00000000

/*
 * Port I setup.
 * All input with pull-up.
 */
#define VAL_GPIOI_MODER             0x00000000
#define VAL_GPIOI_OTYPER            0x00000000
#define VAL_GPIOI_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOI_PUPDR             0xFFFFFFFF
#define VAL_GPIOI_ODR               0xFFFFFFFF
#define VAL_GPIOI_AFRL              0x00000000
#define VAL_GPIOI_AFRH              0x00000000

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
