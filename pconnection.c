/**
 * Copyright (C) 2013, 2014, 2015 Johannes Taelman
 * Copyright (C) 2019 Piers Titus van der Torren
 *
 * This file is part of Striso Control.
 *
 * Striso Control is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * Striso Control is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * Striso Control. If not, see <http://www.gnu.org/licenses/>.
 */
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "pconnection.h"
#include "usbcfg.h"
#include "midi.h"
//#include "string.h"
// #include "exceptions.h"
#include "bulk_usb.h"
#include "midi.h"
#include "midi_usb.h"
#include "config.h"
#include "version.h"
#include "ws2812.h"

//#define DEBUG_SERIAL 1

void BootLoaderInit(void);

#define BOOTLOADER_RTC_SIGNATURE   0x71a21877

/**
 * Set boot signature and reset
 */
void reset_to_uf2_bootloader(void) {
  // Enable writing to backup domain
  // STM32F4:
  // PWR->CR |= PWR_CR_DBP;
  // STM32H7:
  PWR->CR1 |= PWR_CR1_DBP;
  // Set boot signature in RTC backup register
  RTC->BKP0R = BOOTLOADER_RTC_SIGNATURE;

  NVIC_SystemReset();
}

static void cmd_threads(BaseSequentialStream *chp) {
  static const char *states[] = {CH_STATE_NAMES};
  thread_t *tp;

  chprintf(chp, "stklimit    stack     addr   unused refs prio     state         name\r\n");
  tp = chRegFirstThread();
  do {
#if (CH_DBG_ENABLE_STACK_CHECK == TRUE) || (CH_CFG_USE_DYNAMIC == TRUE)
    uint32_t stklimit = (uint32_t)tp->wabase;
#else
    uint32_t stklimit = 0U;
#endif
#if CH_DBG_FILL_THREADS
    char *stk = (char *)(tp->wabase);
    int nfree = 0;
    while(*stk == 0x55){
      nfree++;
      stk++;
    }
#else
    int nfree -1;
#endif
    chprintf(chp, "%08lx %08lx %08lx %8lu %4lu %4lu %9s %12s\r\n",
             stklimit, (uint32_t)tp->ctx.sp, (uint32_t)tp, nfree,
             (uint32_t)tp->refs - 1, (uint32_t)tp->prio, states[tp->state],
             tp->name == NULL ? "" : tp->name);
    tp = chRegNextThread(tp);
  } while (tp != NULL);
}

void InitPConnection(void) {

  /*
   * Initializes a serial-over-USB CDC driver.
   */
  mduObjectInit(&MDU1);
  mduStart(&MDU1, &midiusbcfg);
  bduObjectInit(&BDU1);
  bduStart(&BDU1, &bulkusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(midiusbcfg.usbp);
  chThdSleepMilliseconds(500);
  usbStart(midiusbcfg.usbp, &usbcfg);
  usbConnectBus(midiusbcfg.usbp);
}

void PExReceiveByte(unsigned char c) {
  // ws2812_write_led(0, 0,67,0);
  static char header = 0;
  static int state = 0;

  if (!header) {
    switch (state) {
    case 0:
      if (c == 'S')
        state++;
      break;
    case 1:
      if (c == 't')
        state++;
      else
        state = 0;
      break;
    case 2:
      if (c == 'c')
        state++;
      else
        state = 0;
      break;
    case 3:
      state = 0;
      if (c == 'D') { // go to DFU mode
        // ws2812_write_led(0, 64, 8, 0);
        // ws2812_write_led(1, 64, 8, 0);
        // ws2812_write_led(2, 64, 8, 0);
        chprintf((BaseSequentialStream * )&BDU1, "Resetting to DFU mode...\r\n");
        chThdSleepMilliseconds(2);
        // exception_initiate_dfu();
      }
      else if (c == 'B') { // go to bootloader
        // ws2812_write_led(0, 64, 8, 0);
        // ws2812_write_led(1, 64, 8, 0);
        // ws2812_write_led(2, 64, 8, 0);
        chprintf((BaseSequentialStream * )&BDU1, "Resetting to UF2 bootloader mode...\r\n");
        chThdSleepMilliseconds(2);
        reset_to_uf2_bootloader();
      }
      else if (c == 'S') { // enable binary protocol over USB Bulk
        config.send_usb_bulk = 1;
      }
      else if (c == 's') { // disable binary protocol over USB Bulk
        config.send_usb_bulk = 0;
        chprintf((BaseSequentialStream * )&BDU1, "Stcs\r\n");
      }
      else if (c == 'V') { // firmware version
        config.send_usb_bulk = 0;
        chSysLock();
        if (!obqIsEmptyI(&BDU1.obqueue)) {
          obqResetI(&BDU1.obqueue);
        }
        chSysUnlock();
        chprintf((BaseSequentialStream *)&BDU1, FWVERSION " %08X%08X%08X\r\n",
                 ((uint32_t*)UID_BASE)[2], ((uint32_t*)UID_BASE)[1], ((uint32_t*)UID_BASE)[0]);
      }
      else if (c == 'I') { // thread info
        cmd_threads((BaseSequentialStream *)&BDU1);
      }
      break;
    }
  }
}

void PExReceive(void) {
  // get USB bulk bytes from host
  unsigned char received;
  while (chnReadTimeout(&BDU1, &received, 1, TIME_IMMEDIATE)) {
    PExReceiveByte(received);
  }

  // get USB MIDI bytes from host
  // TODO: check if midi buffer is alligned with 4 bytes
  uint8_t r[4];
  while (chnReadTimeout(&MDU1, &r[0], 4, TIME_IMMEDIATE)) {
    MidiInMsgHandler(MIDI_DEVICE_USB_DEVICE, ((r[0] & 0xF0) >> 4) + 1, r[1],
                    r[2], r[3]);
  }
}

/*
 void USBDMidiPoll(void) {
 uint8_t r[4];
 while (chnReadTimeout(&MDU1, &r, 4, TIME_IMMEDIATE)) {
 MidiInMsgHandler(MIDI_DEVICE_USB_DEVICE, (( r[0] & 0xF0) >> 4)+ 1, r[1], r[2], r[3]);
 }
 }
 */
