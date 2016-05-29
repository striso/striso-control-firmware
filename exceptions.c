/**
 * Copyright (C) 2015 Johannes Taelman
 *
 * This file is part of Axoloti.
 *
 * Axoloti is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * Axoloti is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * Axoloti. If not, see <http://www.gnu.org/licenses/>.
 */

#include "ch.h"
#include "hal.h"
#include "exceptions.h"

#define ERROR_MAGIC_NUMBER 0xE1212012

typedef enum {
  fault = 0,
  watchdog_soft,
  watchdog_hard,
  brownout,
  goto_DFU
} faulttype;

typedef struct {
  volatile uint32_t magicnumber;
  volatile faulttype type;
  volatile uint32_t r0;
  volatile uint32_t r1;
  volatile uint32_t r2;
  volatile uint32_t r3;
  volatile uint32_t r12;
  volatile uint32_t lr;
  volatile uint32_t pc;
  volatile uint32_t psr;
  volatile uint32_t ipsr;
  volatile uint32_t cfsr;
  volatile uint32_t hfsr;
  volatile uint32_t mmfar;
  volatile uint32_t bfar;
  volatile uint32_t i;
} exceptiondump_t;

#define exceptiondump ((exceptiondump_t *)BKPSRAM_BASE)

/**
 * @brief   Jumps into the System ROM bootloader
 * @details This will only work before other initializations!
 */
void BootLoaderInit(void) {
  int reg, psp;
  reg = 0;
  asm volatile ("msr     CONTROL, %0" : : "r" (reg));
  asm volatile ("isb");
  psp = 0;
  asm volatile ("cpsie   i");
  asm volatile ("msr     PSP, %0" : : "r" (psp));
  SCB_FPCCR = 0;
  asm volatile ("LDR     R0, =0x40023844 ;");
  // RCC_APB2ENR (+0x18)
  asm volatile ("LDR     R1, =0x4000 ;");
  // ENABLE SYSCFG CLOCK (1)
  asm volatile ("STR     R1, [R0, #0]    ;");
  asm volatile ("NOP");
  asm volatile ("NOP");
  asm volatile ("NOP");
  asm volatile ("LDR     R0, =0x40013800 ;");
  // SYSCFG_CFGR1 (+0x00)
  asm volatile ("LDR     R1, =0x1 ;");
  // MAP ROM
  asm volatile ("STR     R1, [R0, #0]    ;");
  // MAP ROM AT ZERO (1)
  asm volatile ("NOP");
  asm volatile ("NOP");
  asm volatile ("NOP");
  asm volatile ("MOVS    R1, #0          ;");
  //  ADDRESS OF ZERO
  asm volatile ("LDR     R0, [R1, #0]    ;");
  // SP @ +0
  asm volatile ("MOV     SP, R0");
  asm volatile ("LDR     R0, [R1, #4]    ;");
  // PC @ +4
  asm volatile ("NOP");
  asm volatile ("BX      R0");
}

/**
 * @brief   Check exception magic bytes for DFU mode request.
 * @details Enables access to battery backup SRAM.
 */
void exception_check_DFU(void) {
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  PWR->CR |= PWR_CR_DBP;
  RCC->AHB1ENR |= RCC_AHB1ENR_BKPSRAMEN;
  asm volatile ("NOP");
  asm volatile ("NOP");
  asm volatile ("NOP");
  if (exception_check() && (exceptiondump->type == goto_DFU)) {
    exception_clear();
    BootLoaderInit();
  }
}

int exception_check(void) {
  if (exceptiondump->magicnumber == ERROR_MAGIC_NUMBER)
    return 1; // exception happened
  else
    return 0; // all fine
}

void exception_clear(void) {
  exceptiondump->magicnumber = 0;
}

/**
 * @brief   Initiate jumping into the system ROM bootloader.
 * @details By writing magic bytes and going through a soft reboot...
 */
void exception_initiate_dfu(void) {
  exceptiondump->r0 = 1;
  exceptiondump->r1 = 2;
  exceptiondump->r2 = 3;
  exceptiondump->r3 = 4;
  palSetPadMode(GPIOA, 11, PAL_MODE_INPUT);
  palSetPadMode(GPIOA, 12, PAL_MODE_INPUT);
  volatile int i = 20;
  while (i--) {
    volatile int j = 1 << 12;
    palTogglePad(GPIOA, GPIOA_LED1);
    while (j--) {
      volatile int k = 1 << 8;
      while (k--) {
      }
      //watchdog_feed();
    }
  }
  exceptiondump->magicnumber = ERROR_MAGIC_NUMBER;
  exceptiondump->type = goto_DFU;
  NVIC_SystemReset();
}
