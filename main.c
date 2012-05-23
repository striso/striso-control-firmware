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

#include "ch.h"
#include "hal.h"
#include "test.h"
#include "chprintf.h"

static void pwmpcb(PWMDriver *pwmp);
static void adccb(ADCDriver *adcp, adcsample_t *buffer, size_t n);

/* Total number of channels to be sampled by a single ADC operation.*/
#define ADC_GRP1_NUM_CHANNELS   3

/* Depth of the conversion buffer, channels are sampled one time each.*/
#define ADC_GRP1_BUF_DEPTH      1

#define ADC_GRP1_BUF_COUNT      (17*ADC_GRP1_NUM_CHANNELS)

#define OUT_NUM_CHANNELS        51

static const ioportid_t out_channels_port[51] = {
  GPIOA, GPIOA, GPIOA, GPIOA, GPIOC, GPIOC, GPIOB, GPIOB, GPIOF, GPIOF,
  GPIOF, GPIOF, GPIOF, GPIOG, GPIOG, GPIOE, GPIOE, GPIOE,
  GPIOE, GPIOE, GPIOE, GPIOE, GPIOE, GPIOE, GPIOB, GPIOB,
  GPIOB, GPIOB, GPIOB, GPIOB, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD,
  GPIOD, GPIOD, GPIOG, GPIOG, GPIOG, GPIOG, GPIOG, GPIOG, GPIOG,
  GPIOC, GPIOC, GPIOC, GPIOC, GPIOA, GPIOA
};
static const int out_channels_pad[51] = {
   4,  5,  6,  7,  4,  5,  0,  1, 11, 12,
  13, 14, 15,  0,  1,  7,  8,  9,
  10, 11, 12, 13, 14, 15, 10, 11,
  12, 13, 14, 15,  8,  9, 10, 11, 12, 13,
  14, 15,  2,  3,  4,  5,  6,  7,  8, 
   6,  7,  8,  9,  8,  9
};

static int cur_channel = 0;

/*
 * ADC samples buffer.
 */
static adcsample_t samples[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH * ADC_GRP1_BUF_COUNT];

//#define ADC_SAMPLE_DEF ADC_SAMPLE_3
#define ADC_SAMPLE_DEF ADC_SAMPLE_15
/*
 * ADC conversion group.
 * Mode:        Linear buffer, 4 samples of 2 channels, SW triggered.
 * Channels:    IN11   (48 cycles sample time)
 *              Sensor (192 cycles sample time)
 */
static const ADCConversionGroup adcgrpcfg = {
  FALSE,
  ADC_GRP1_NUM_CHANNELS,
  adccb,
  NULL,
  /* HW dependent part.*/
  0,
  ADC_CR2_SWSTART,
  ADC_SMPR2_SMP_AN0(ADC_SAMPLE_DEF) | ADC_SMPR2_SMP_AN1(ADC_SAMPLE_DEF) | ADC_SMPR2_SMP_AN2(ADC_SAMPLE_DEF),
  0,
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),
  0,
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN1) | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN2)
  //ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN3) | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN1) | ADC_SQR3_SQ4_N(ADC_CHANNEL_IN3) | ADC_SQR3_SQ5_N(ADC_CHANNEL_IN2)

};

/*
 * PWM configuration structure.
 * Cyclic callback enabled, channels 1 and 4 enabled without callbacks,
 * the active state is a logic one.
 */
static PWMConfig pwmcfg = {
  10000,                                    /* 10kHz PWM clock frequency.   */
  2,                                    /* PWM period 1S (in ticks).    */
  pwmpcb,
  {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}
  },
  /* HW dependent part.*/
  0
};

static SerialConfig ser_cfg = {
    2000000,
    0,
    0,
    0,
};

/*
 * PWM cyclic callback.
 * A new ADC conversion is started.
 */
static void pwmpcb(PWMDriver *pwmp) {

  (void)pwmp;

  /* Starts an asynchronous ADC conversion operation, the conversion
     will be executed in parallel to the current PWM cycle and will
     terminate before the next PWM cycle.*/
  chSysLockFromIsr();
  adcStartConversionI(&ADCD1, &adcgrpcfg, &samples[cur_channel*ADC_GRP1_NUM_CHANNELS], ADC_GRP1_BUF_DEPTH);
  //adcStartConversionI(&ADCD1, &adcgrpcfg, samples, ADC_GRP1_BUF_DEPTH);
  chSysUnlockFromIsr();
}

/*
 * ADC end conversion callback.
 * The PWM channels are reprogrammed using the latest ADC samples.
 * The latest samples are transmitted into a single SPI transaction.
 */
void adccb(ADCDriver *adcp, adcsample_t *buffer, size_t n) {

  (void) buffer; (void) n;
  /* Note, only in the ADC_COMPLETE state because the ADC driver fires an
     intermediate callback when the buffer is half full.*/
  if (adcp->state == ADC_COMPLETE) {
    int old_channel = cur_channel;

    chSysLockFromIsr();

    palSetPad(out_channels_port[cur_channel], out_channels_pad[cur_channel]);       /* Open old channel */
    cur_channel = (cur_channel+1) % OUT_NUM_CHANNELS;
    palClearPad(out_channels_port[cur_channel], out_channels_pad[cur_channel]);         /* Drain new channel */

    chSysUnlockFromIsr();

    int s0,s1,s2;
    s0 = 4095-samples[old_channel*ADC_GRP1_NUM_CHANNELS];
    s1 = 4095-samples[old_channel*ADC_GRP1_NUM_CHANNELS+1] - (s0-30)/40;
    s2 = 4095-samples[old_channel*ADC_GRP1_NUM_CHANNELS+2] - (s1-30)/40;

    chprintf((BaseChannel *)&SD2, "%04d %04d %04d ", s0, s1, s2);
    //4095-samples[old_channel*ADC_GRP1_NUM_CHANNELS], 4095-samples[old_channel*ADC_GRP1_NUM_CHANNELS+1], 4095-samples[old_channel*ADC_GRP1_NUM_CHANNELS+2]);
    //chprintf((BaseChannel *)&SD2, "%02d %02d %02d ", (4095-samples[old_channel*3])/40, (4095-samples[old_channel*3+1])/40, (4095-samples[old_channel*3+2])/40);
    //chprintf((BaseChannel *)&SD2, "%04d %04d %04d ", samples[0], samples[1], samples[2]);
    if (cur_channel == 0)
      chprintf((BaseChannel *)&SD2, "\r");

    //chThdSleepMilliseconds(1);

    //chSysLockFromIsr();
    //adcStartConversionI(&ADCD1, &adcgrpcfg, &samples[cur_channel*3], ADC_GRP1_BUF_DEPTH);
    //chSysUnlockFromIsr();

    //adcsample_t avg_ch1, avg_ch2;

    //chSysLockFromIsr();
    //chSysUnlockFromIsr();
  }
}

/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED.
 */
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (TRUE) {
    palSetPad(GPIOA, GPIOA_LED1);       /* Orange.  */
    chThdSleepMilliseconds(500);
    palClearPad(GPIOA, GPIOA_LED1);     /* Orange.  */
    chThdSleepMilliseconds(500);
  }
}

/*
 * Application entry point.
 */
int main(void) {
  int n;

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
   * Activates the serial driver 2 using the driver default configuration.
   * PD5(TX) and PD6(RX) are routed to USART2.
   */
  sdStart(&SD2, &ser_cfg);
  palSetPadMode(GPIOD, 5, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOD, 6, PAL_MODE_ALTERNATE(7));

  /*
   * If the user button is pressed after the reset then the test suite is
   * executed immediately before activating the various device drivers in
   * order to not alter the benchmark scores.
   */
  if (palReadPad(GPIOG, GPIOG_BUTTON))
    TestThread(&SD2);

  /*
   * Initialize output channels for the buttons as opendrain
   */
  for (n=0; n<OUT_NUM_CHANNELS; n++) {
    palSetPadMode(out_channels_port[n], out_channels_pad[n], PAL_MODE_OUTPUT_OPENDRAIN);
  }

  /*
   * Initializes the ADC driver 1.
   * The pin PA0,PA1,PA2 on the port GPIOC are programmed as analog input.
   */
  adcStart(&ADCD1, NULL);
  palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
  palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
  palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);

  /*
   * Initializes the PWM driver 4.
   */
  pwmStart(&PWMD4, &pwmcfg);
  //adcStartConversionI(&ADCD1, &adcgrpcfg, &samples[cur_channel*3], ADC_GRP1_BUF_DEPTH);

  /*
   * Creates the example thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Initializes the SPI driver 1 in order to access the MEMS. The signals
   * are initialized in the board file.
   * Several LIS302DL registers are then initialized.
   */

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state, when the button is
   * pressed the test procedure is launched with output on the serial
   * driver 2.
   */
  while (TRUE) {
//    int8_t x, y, z;

    if (palReadPad(GPIOG, GPIOG_BUTTON))
      TestThread(&SD2);
/*
    x = (int8_t)lis302dlReadRegister(&SPID1, LIS302DL_OUTX);
    y = (int8_t)lis302dlReadRegister(&SPID1, LIS302DL_OUTY);
    z = (int8_t)lis302dlReadRegister(&SPID1, LIS302DL_OUTZ);*/
    //chprintf((BaseChannel *)&SD2, "%03d, %03d, %03d \r", x, y, z);
    chThdSleepMilliseconds(50);
  }
}
