/*
    Striso Control is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "config.h"
#include "striso.h"
#include "usbcfg.h"
#include "exceptions.h"
#include "pconnection.h"
#include "synth.h"
#include "button_read.h"
#include "messaging.h"
#include "motionsensor.h"
#include "ws2812.h"

// next lines are necessary since synth_contol.cpp is included
// and are copied from ChibiOS/testhal/STM32F4xx/RTC/main.c
/* libc stub */
int _getpid(void) {return 1;}
/* libc stub */
void _exit(int i) {(void)i;while(1);}
/* libc stub */
#include <errno.h>
#undef errno
extern int errno;
int _kill(int pid, int sig) {
  (void)pid;
  (void)sig;
  errno = EINVAL;
  return -1;
}

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
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  int msg[8];
  msg[0] = ID_SYS;
  msg[1] = ID_SYS_MSGQUE_OVERFLOW_BB;
  while (TRUE) {
    chThdSleepMilliseconds(500);
    palSetPad(GPIOA, GPIOA_LED1);
    ws2812_write_led(0, 15, 0, 31);
    chThdSleepMilliseconds(500);
    msg[2] = underruns;
    //if (!msgSend(3,msg))
      palClearPad(GPIOA, GPIOA_LED1);
    ws2812_write_led(0, 15, 31, 0);
  }

  return 0;
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
static WORKING_AREA(waThreadSend, 128);
static msg_t ThreadSend(void *arg) {

  (void)arg;
  chRegSetThreadName("send messages");
  int msg[9];
  uint8_t cmsg[16];
  int size;
  cmsg[0] = 0;
  while (TRUE) {
    size = msgGet(9, msg);
    if (size >= 2 && size <= 9) {
      synth_message(size, &msg);

      cmsg[0] = 0x80 | ((uint8_t)msg[0])<<3 | ((uint8_t)(size-2));
      cmsg[1] = 0x7f & (uint8_t)msg[1];
      pack(&msg[2], &cmsg[2], size - 2);
#ifdef USE_UART
      chSequentialStreamWrite((BaseSequentialStream *)&SD1, cmsg, 2+(size-2)*2);
#endif

      //chSequentialStreamWrite((BaseSequentialStream *)&BDU1,cmsg, 2+(size-2)*2);
      chOQWriteTimeout(&BDU1.oqueue, cmsg, 2+(size-2)*2, TIME_IMMEDIATE);
    }
    else if (size == 0) {
      chThdSleep(1);
    }
  }

  return 0;
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
  ws2812_write_led(0, 15, 0, 0);
#endif

  InitPConnection();

  MessagingInit();

  /*
   * Creates the message send thread.
   */
  chThdCreateStatic(waThreadSend, sizeof(waThreadSend), NORMALPRIO, ThreadSend, NULL);

  /*
   * Creates the LED flash thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  ButtonReadStart();
  
#ifdef USE_MPU6050
  MotionSensorStart();
#endif

  while (1) {
    chThdSleepMilliseconds(2);
    synth_tick();
  }
}
