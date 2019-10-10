/**
 * Copyright (C) 2013, 2014, 2015 Johannes Taelman
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
#include "chprintf.h"
#include "pconnection.h"
#include "usbcfg.h"
#include "midi.h"
//#include "string.h"
#include "exceptions.h"
#include "bulk_usb.h"
#include "midi.h"
#include "midi_usb.h"
#include "config.h"
#include "version.h"

//#define DEBUG_SERIAL 1

void BootLoaderInit(void);

static WORKING_AREA(waThreadUSBDMidi, 256);

__attribute__((noreturn))
    static msg_t ThreadUSBDMidi(void *arg) {
  (void)arg;
#if CH_USE_REGISTRY
  chRegSetThreadName("usbdmidi");
#endif
  uint8_t r[4];
  while (1) {
    chnReadTimeout(&MDU1, &r[0], 4, TIME_INFINITE);
    MidiInMsgHandler(MIDI_DEVICE_USB_DEVICE, ((r[0] & 0xF0) >> 4) + 1, r[1],
                    r[2], r[3]);
  }
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
  chThdSleepMilliseconds(1000);
  usbStart(midiusbcfg.usbp, &usbcfg);
  usbConnectBus(midiusbcfg.usbp);

  chThdCreateStatic(waThreadUSBDMidi, sizeof(waThreadUSBDMidi), NORMALPRIO,
                    ThreadUSBDMidi, NULL);
}

void PExReceiveByte(unsigned char c) {
  ws2812_write_led(0, 0,67,0);
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
      if (c == 'D') { // go to DFU mode
        state = 0;
        ws2812_write_led(0, 64, 8, 0);
        ws2812_write_led(1, 64, 8, 0);
        ws2812_write_led(2, 64, 8, 0);
        chprintf((BaseSequentialStream * )&BDU1, "Resetting to DFU mode...\r\n");
        chThdSleepMilliseconds(2);
        exception_initiate_dfu();
      }
      else if (c == 'S') { // enable binary protocol over USB Bulk
        state = 0;
        config.send_usb_bulk = 1;
      }
      else if (c == 's') { // disable binary protocol over USB Bulk
        state = 0;
        config.send_usb_bulk = 0;
      }
      else if (c == 'V') { // firmware version
        state = 0;
        config.send_usb_bulk = 0;
        // if (!chOQIsEmptyI(&BDU1.oqueue)) {
        //   chThdSleepMilliseconds(1);
        //   BDU1.oqueue.q_notify(&BDU1.oqueue);
        // } else {
          chprintf((BaseSequentialStream * )&BDU1, FWVERSION "\r\n");
          //chOQWriteTimeout(&BDU1.oqueue, cmsg, 14, TIME_INFINITE);
        // }
      }
      else
        state = 0;
      break;
    }
  }
}

void PExReceive(void) {
  unsigned char received;
  while (chnReadTimeout(&BDU1, &received, 1, TIME_IMMEDIATE)) {
    PExReceiveByte(received);
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
