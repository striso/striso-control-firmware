/**
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

#define CONFIG_HERE
#include "config.h"
#undef CONFIG_HERE
#include "striso.h"
#include "usbcfg.h"
// #include "exceptions.h"
#include "pconnection.h"
#include "synth.h"
#include "button_read.h"
#include "messaging.h"
#include "motionsensor.h"
#include "ws2812.h"
#include "version.h"

/**
 *  Firmware version description on fixed flash address for bootloader
 */
__attribute__ ((section(".fwversion"))) __attribute__((used))
const char fwversion[] = "Firmware version: striso_control_" FWVERSION "\r\n";

// force sqrtf to use FPU, the standard one apparently doesn't
float vsqrtf(float op1) {
  float result;
  __ASM volatile ("vsqrt.f32 %0, %1" : "=w" (result) : "w" (op1) );
  return (result);
}

static SerialConfig ser_cfg = {
    500000,
    0,
    0,
    0,
};

/*
 * LED flash thread.
 */
static THD_WORKING_AREA(waThread1, 128);
static void Thread1(void *arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  // int msg[8];
  // msg[0] = ID_SYS;
  // msg[1] = ID_SYS_MSGQUE_OVERFLOW_BB;
  while (TRUE) {
    chThdSleepMilliseconds(300);
    palSetLine(LINE_LED1);
    chThdSleepMilliseconds(300);
    // msg[2] = underruns;
    //if (!msgSend(3,msg))
      palClearLine(LINE_LED1);
  }
}

static void pack(int *in, uint8_t *out, int n) {
  int c;
  for (c=0; c<n; c++) {
    out[c*2] = 0x7f & (uint8_t)(in[c]>>7);
    out[c*2+1] = 0x7f & (uint8_t)(in[c]);
  }
}

static void unpack(uint8_t *in, int *out, int n) {
  int c;
  for (c=0; c<n; c++) {
    out[c] = ((int)in[c*2])<<7 | ((int)in[c*2+1]);
  }
}

/*
 * Message send thread
 */
static THD_WORKING_AREA(waThreadSend, 256);
static void ThreadSend(void *arg) {

  (void)arg;
  chRegSetThreadName("send messages");
  int msg[9];
  uint8_t cmsg[16];
  int size;
  cmsg[0] = 0;
  while (TRUE) {
    size = msgGet(9, msg);
    if (size >= 2 && size <= 9) {
      synth_message(size, msg);

      cmsg[0] = 0x80 | ((uint8_t)msg[0])<<3 | ((uint8_t)(size-2));
      cmsg[1] = 0x7f & (uint8_t)msg[1];
      pack(&msg[2], &cmsg[2], size - 2);
#ifdef USE_UART
      chSequentialStreamWrite((BaseSequentialStream *)&SD1, cmsg, 2+(size-2)*2);
#endif
#ifdef USE_USB
      if (config.send_usb_bulk) {
        //chSequentialStreamWrite((BaseSequentialStream *)&BDU1,cmsg, 2+(size-2)*2);
        obqWriteTimeout(&BDU1.obqueue, cmsg, 2+(size-2)*2, TIME_IMMEDIATE);
      }
#endif
    }
    // sleep to limit the output stream to 2 messages per millisecond (with CH_FREQUENCY = 2000)
    // for compatibility with MIDI usb on Axoloti
    chThdSleep(1);
  }
}

/*
 * Application entry point.
 */
int main(void) {
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Activates the serial driver using the driver default configuration.
   */
#ifdef USE_UART
  sdStart(&SD1, &ser_cfg);
#endif

#ifdef USE_WS2812
  ws2812_init();
  ws2812_write_led(0, 15, 31,  0);
  ws2812_write_led(1, 15, 31,  0);
  ws2812_write_led(2, 15, 31,  0);
  ws2812_write_led(3,  0, 15, 15);
#endif

#ifdef USE_USB
  InitPConnection();
#endif

  MessagingInit();

  /*
   * Creates the message send thread.
   */
  chThdCreateStatic(waThreadSend, sizeof(waThreadSend), NORMALPRIO, ThreadSend, NULL);

  /*
   * Creates the LED flash thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

#ifdef USE_MIDI_OUT
  // Send initial configuration MIDI
  chThdSleepMilliseconds(200);
  midi_config();
#endif

#ifdef USE_INTERNAL_SYNTH
  codec_init(SAMPLERATE);
#endif

  ButtonReadStart();
  
#if defined(USE_MPU6050) || defined(USE_LSM6DSL)
  MotionSensorStart();
#endif

#ifdef USE_WS2812
  ws2812_write_led(0,  0,  1,  0);
  ws2812_write_led(1,  0,  0,  0);
  ws2812_write_led(2,  0,  0,  0);
  ws2812_write_led(3,  0, 15, 15);
#endif

  while (1) {
    chThdSleepMilliseconds(2);
    synth_tick();

    // Jack detection
    if (palReadLine(LINE_AUX_JACK_DETECT)) {
      palSetLine(LINE_LED_ALT);
      palSetLine(LINE_AUX_UART2_TX);
    } else {
      palClearLine(LINE_LED_ALT);
    }
    if (palReadLine(LINE_JACK_DETECT)) {
      // palSetLine(LINE_LED_R);
    } else {
      // palClearLine(LINE_LED_R);
    }

#ifdef USE_USB
    PExReceive();
#endif
  }
}
