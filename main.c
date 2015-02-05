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
#include "chprintf.h"

#include "usbcfg.h"

#include "calib.h"

#include "adc_multi.h"

/* Virtual serial port over USB.*/
SerialUSBDriver SDU1;

/*
 * end accelerometer
 */

#define ID_DIS 0
#define ID_BAS 1
#define ID_CONTROL 2
#define ID_ACCEL 3
#define ID_SYS 4

#define INTERNAL_ONE (1<<24)
#define ADCFACT (1<<12)
#define MSGFACT (1<<11)
#define MSGFACT_VELO (MSGFACT/32)
#define FILT 8

#define ADC_OFFSET 512

/* Number of ADCs used in multi ADC mode (2 or 3) */
#define ADC_N_ADCS 3

/* Total number of channels to be sampled by a single ADC operation.*/
#define ADC_GRP1_NUM_CHANNELS_PER_ADC   2

/* Depth of the conversion buffer, channels are sampled one time each.*/
#define ADC_GRP1_BUF_DEPTH      (2*ADC_N_ADCS) // must be 1 or even

#define OUT_NUM_CHANNELS        51

static const ioportid_t out_channels_port[51] = {
  GPIOC, GPIOC, GPIOC, GPIOG, GPIOG, GPIOG, GPIOG, GPIOG,
  GPIOG, GPIOG, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD,
  GPIOD, GPIOB, GPIOB, GPIOB, GPIOH, GPIOH, GPIOH, GPIOH,
  GPIOH, GPIOH, GPIOH, GPIOB, GPIOB, GPIOE, GPIOE, GPIOE, GPIOE,
  GPIOE, GPIOE, GPIOE, GPIOE, GPIOE, GPIOG, GPIOG, GPIOF,
  GPIOF, GPIOF, GPIOF, GPIOF, GPIOB, GPIOB, GPIOC, GPIOC, GPIOA,
};
static const int out_channels_pad[51] = {
   8, 7, 6, 8, 7, 6, 5, 4, 3, 2,15,14,13,12,11,10, 9,
   8,15,14,13,12,11,10, 9, 8, 7, 6,11,10,15,14,13,12,
  11,10, 9, 8, 7, 1, 0,15,14,13,12,11, 1, 0, 5, 4, 7,
};

static const ioportid_t out_channels_bas_port[51] = {
  GPIOA, GPIOC, GPIOC, GPIOC, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD,
  GPIOD, GPIOD, GPIOD, GPIOG, GPIOG, GPIOG, GPIOG, GPIOG,
  GPIOG, GPIOG, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB,
  GPIOE, GPIOE, GPIOI, GPIOI, GPIOI, GPIOI, GPIOE, GPIOE,
  GPIOE, GPIOE, GPIOE, GPIOI, GPIOI, GPIOI, GPIOF, GPIOF, GPIOF,
  GPIOF, GPIOF, GPIOF, GPIOF, GPIOF, GPIOF, GPIOF, GPIOF,
};
static const int out_channels_bas_pad[51] = {
  15,10,11,12, 0, 1, 2, 3, 4, 5, 6, 7, 9,10,11,12,13,
  14,15, 3, 4, 5, 6, 7, 8, 9, 0, 1, 4, 5, 6, 7, 2, 3,
   4, 5, 6, 9,10,11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,
};

static int cur_channel = 0;
static int next_conversion = 0;
static int proc_conversion = 0;

typedef struct struct_button {
  int32_t s0;
  int32_t s1;
  int32_t s2;
  int32_t v0;
  int32_t v1;
  int32_t v2;
  int pressed;
  int timer;
  int but_id;
  int src_id;
} button_t;

typedef struct struct_slider {
  int32_t s[27];
  int32_t v[27];
  int timer;
} slider_t;

static button_t buttons[51];
static button_t buttons_bas[51];
static int buttons_pressed = 0;

/*
 * ADC samples buffer.
 */
static adcsample_t adc_samples[ADC_GRP1_NUM_CHANNELS_PER_ADC * ADC_N_ADCS * ADC_GRP1_BUF_DEPTH];
static adcsample_t samples0[102] = {0};
static adcsample_t samples1[102] = {0};
static adcsample_t samples2[102] = {0};
static adcsample_t* samples[3] = {samples0, samples1, samples2};

static adcsample_t samples_bas0[102] = {0};
static adcsample_t samples_bas1[102] = {0};
static adcsample_t* samples_bas[2] = {samples_bas0, samples_bas1};

static void adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
  (void)adcp;
  (void)n;

  /* Open old channel */
  palSetPad(out_channels_port[cur_channel], out_channels_pad[cur_channel]);
  palSetPad(out_channels_bas_port[cur_channel], out_channels_bas_pad[cur_channel]);
  cur_channel = (next_conversion+1) % OUT_NUM_CHANNELS;
  /* Drain new channel */
  palClearPad(out_channels_port[cur_channel], out_channels_pad[cur_channel]);
  palClearPad(out_channels_bas_port[cur_channel], out_channels_bas_pad[cur_channel]);

  // start next ADC conversion
  adcp->adc->CR2 |= ADC_CR2_SWSTART;

  /* copy adc_samples */
  samples0[next_conversion] = buffer[0];
  samples1[next_conversion] = buffer[1];
  samples2[next_conversion] = buffer[2];
  //samples3[next_conversion] = buffer[3];

  samples_bas0[next_conversion] = buffer[4];
  samples_bas1[next_conversion] = buffer[5];

  next_conversion = (next_conversion+1) % 102;
}

#define SENDFACT 1
//#define ADC_SAMPLE_DEF ADC_SAMPLE_3
//#define ADC_SAMPLE_DEF ADC_SAMPLE_15
//#define ADC_SAMPLE_DEF ADC_SAMPLE_28
//#define ADC_SAMPLE_DEF ADC_SAMPLE_56
//#define ADC_SAMPLE_DEF ADC_SAMPLE_84
//#define ADC_SAMPLE_DEF ADC_SAMPLE_112
#define ADC_SAMPLE_DEF ADC_SAMPLE_144 // 0.83 ms per cycle
//#define ADC_SAMPLE_DEF ADC_SAMPLE_480 // 2.7 ms per cycle

/*
 * ADC conversion group for ADC0 as multi ADC mode master.
 * Mode:        Circular buffer, triple ADC mode master, SW triggered.
 * Channels:    PA0, PA3
 */
static const ADCConversionGroup adcgrpcfg1 = {
  TRUE, // Circular conversion
  ADC_GRP1_NUM_CHANNELS_PER_ADC,
  adccallback, /* end of conversion callback */
  NULL, /* error callback */
  /* HW dependent part.*/
  0, // CR1
  ADC_CR2_SWSTART, // CR2
  0, // SMPR1
  ADC_SMPR2_SMP_AN0(ADC_SAMPLE_DEF)
   | ADC_SMPR2_SMP_AN3(ADC_SAMPLE_DEF), // SMPR2
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS_PER_ADC), // SQR1
  0, // SQR2
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0)
   | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN3) // SQR3
};

/*
 * ADC conversion group for ADC2.
 * Mode:        triple ADC mode slave.
 * Channels:    PA1, PC0
 */
static const ADCConversionGroup adcgrpcfg2 = {
  TRUE,
  0,
  NULL, /* end of conversion callback */
  NULL, /* error callback */
  /* HW dependent part.*/
  0, // CR1
  0, // CR2
  ADC_SMPR1_SMP_AN12(ADC_SAMPLE_DEF), // SMPR1
  ADC_SMPR2_SMP_AN1(ADC_SAMPLE_DEF), // SMPR2
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS_PER_ADC), // SQR1
  0, // SQR2
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN1)
   | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN12) // SQR3
};

/*
 * ADC conversion group for ADC3.
 * Mode:        triple ADC mode slave.
 * Channels:    PA2, PC2
 */
static const ADCConversionGroup adcgrpcfg3 = {
  TRUE,
  0,
  NULL, /* end of conversion callback */
  NULL, /* error callback */
  /* HW dependent part.*/
  0, // CR1
  0, // CR2
  ADC_SMPR1_SMP_AN10(ADC_SAMPLE_DEF), // SMPR1
  ADC_SMPR2_SMP_AN2(ADC_SAMPLE_DEF), // SMPR2
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS_PER_ADC), // SQR1
  0, // SQR2
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN2)
   | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN10) // SQR3
};

#define BUFFERSIZE 240
static int msg_buffer[BUFFERSIZE];
static int msg_read = 0;
static int msg_write = 0;
static Mutex msg_lock;
static int underruns = 0;

static int msgSend(int size, int* msg) {
  chMtxLock(&msg_lock);

  int n;
  int old_write = msg_write;
  msg_buffer[msg_write] = size;
  msg_write = (msg_write + 1) % BUFFERSIZE;
  for (n = 0; n < size; n++) {
    if (msg_read == msg_write) {
      //msg_read = (msg_read + msg_buffer[msg_read] + 1) % BUFFERSIZE;
      msg_write = old_write;
      underruns++;
      chMtxUnlock();
      return 1;
    }
    msg_buffer[msg_write] = msg[n];
    msg_write = (msg_write + 1) % BUFFERSIZE;
  }

  chMtxUnlock();
  return 0;
}

static int msgGet(int maxsize, int* msg) {
  chMtxLock(&msg_lock);

  if (msg_read == msg_write) {
    chMtxUnlock();
    return 0;
  }

  int size = msg_buffer[msg_read];

  if (size > maxsize) {
    chMtxUnlock();
    return -10;
  }
  msg_read = (msg_read + 1) % BUFFERSIZE;

  int n;
  for (n = 0; n < size; n++) {
    if (msg_read == msg_write) {
      chMtxUnlock();
      return -1;
    }
    msg[n] = msg_buffer[msg_read];
    msg_read = (msg_read + 1) % BUFFERSIZE;
  }

  chMtxUnlock();
  return size;
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
  msg[1] = 1;
  while (TRUE) {
    chThdSleepMilliseconds(250);
    palSetPad(GPIOH, GPIOH_LED1);
    chThdSleepMilliseconds(250);
    msg[2] = underruns;
    //if (!msgSend(3,msg))
      palClearPad(GPIOH, GPIOH_LED1);
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
  chRegSetThreadName("send messages over USB");
  int msg[8];
  uint8_t cmsg[16];
  int size;
  cmsg[0] = 0;
  while (TRUE) {
    size = msgGet(8, msg);
    if (size > 0 && size <= 9) {
      cmsg[0] = 0x80 | ((uint8_t)msg[0])<<3 | ((uint8_t)(size-2));
      cmsg[1] = 0x7f & (uint8_t)msg[1];
      pack(&msg[2], &cmsg[2], size - 2);
      size = chSequentialStreamWrite((BaseSequentialStream *)&SD1, cmsg, 2+(size-2)*2);
    }
    else if (size == 0) {
      chThdSleep(1);
    }
  }

  return 0;
}

// Schlick power function, approximation of power function
float powf_schlick(const float a, const float b) {
  return (a / (b - a * b + a));
}

/*
 * Second order Kalman like filter with fast signal end conditions
 */
void update_and_filter(int32_t* s, int32_t* v, int32_t s_new) {
  int32_t old_s = *s;
  *s = ((FILT-1) * (old_s + *v) + s_new) / FILT;
  if (*s < 0) {
    *v = 0;
  } else if (*s > INTERNAL_ONE) {
    *s = INTERNAL_ONE;
    *v = 0;
  } else {
    *v = ((FILT-1) * (*v) + (*s - old_s)) / FILT;
  }
}

int32_t calibrate(int32_t s, int pad_idx) {
  s = ADCFACT * (int32_t)(4095-s) - MSGFACT * ADC_OFFSET;
  return s;
  if (s>0) {
    float sf = ((float)s) * (1.0 / INTERNAL_ONE);
    sf = powf_schlick(sf * calib_mul[pad_idx], calib_pow[pad_idx]);
    s = (uint32_t)(sf * INTERNAL_ONE);
  }
  return s;
}

void update_button(button_t* but, adcsample_t* inp) {
  int but_id = but->but_id;
  int32_t s_new;
  int msg[8];
  msg[0] = but->src_id;

  s_new = calibrate(inp[0], but_id * 3 + 0);
  update_and_filter(&but->s0, &but->v0, s_new);
  s_new = calibrate(inp[1], but_id * 3 + 1);
  update_and_filter(&but->s1, &but->v1, s_new);
  s_new = calibrate(inp[2], but_id * 3 + 2);
  update_and_filter(&but->s2, &but->v2, s_new);

  if (but->s0 > 0 || but->s1 > 0 || but->s2 > 0) {
    if (but->pressed == 0) {
      but->pressed = 1;
      buttons_pressed++;
    }
    if (but->timer <= 0) {
      msg[1] = but_id;
      msg[2] = but->s0 / MSGFACT;
      msg[3] = but->s1 / MSGFACT;
      msg[4] = but->s2 / MSGFACT;
      if (msg[2] < 0) msg[2] = 0;
      if (msg[3] < 0) msg[3] = 0;
      if (msg[4] < 0) msg[4] = 0;
      msg[5] = but->v0 / MSGFACT_VELO;
      msg[6] = but->v1 / MSGFACT_VELO;
      msg[7] = but->v2 / MSGFACT_VELO;
      msgSend(8, msg);
      but->timer = buttons_pressed * SENDFACT;
    } else {
      but->timer--;
    }
  }
  else if (but->pressed) {
    but->pressed = 0;
    buttons_pressed--;
    but->timer = 0;
    msg[1] = but_id;
    msg[2] = 0;
    msg[3] = 0;
    msg[4] = 0;
    msg[5] = but->v0 / MSGFACT_VELO;
    msg[6] = but->v1 / MSGFACT_VELO;
    msg[7] = but->v2 / MSGFACT_VELO;
    while (msgSend(8, msg)) {
      chThdSleep(1);
    }
  }
}

/*
 * Read out buttons and create messages.
 */
static WORKING_AREA(waThreadReadButtons, 128);
static msg_t ThreadReadButtons(void *arg) {
  (void)arg;

  int cur_conv, but_id, note_id;
  button_t* but;

  while (TRUE) {
    while (proc_conversion != next_conversion) {
      // process 3 buttons if all 3 values * 3 buttons are available
      if ((proc_conversion % 3) == 2) {
        note_id = (proc_conversion / 3) % 17;
        cur_conv = (proc_conversion - 2);

        /*
         * Check button in each octave/adc-channel
         */
        for (int n = 0; n < 3; n++) {
          but_id = note_id + n * 17;
          but = &buttons[but_id];
          update_button(but, &samples[n][cur_conv]);
        }
        but_id = note_id;
        but = &buttons_bas[but_id];
        update_button(but, &samples_bas[0][cur_conv]);
        if (note_id % 2) {
          but_id = note_id + 17;
          but = &buttons_bas[but_id];
          update_button(but, &samples_bas[1][cur_conv]);
        } else {
          // slider
          //but_id = note_id + 2*17;
          //but = &buttons_bas[but_id];
          //update_button(but, &samples_bas[1][cur_conv]);
        }

      }
      proc_conversion = (proc_conversion+1) % 102;
    }

    chThdSleepMicroseconds(100);
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

  palSetPadMode(GPIOH, GPIOH_LED1, PAL_MODE_OUTPUT_PUSHPULL);

  /*
   * Activates the serial driver using the driver default configuration.
   */
  sdStart(&SD1, &ser_cfg);
  palSetPadMode(GPIOA, GPIOA_UART1_TX, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, GPIOA_UART1_RX, PAL_MODE_ALTERNATE(7));

  /*
   * Initializes a serial-over-USB CDC driver.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  //usbDisconnectBus(serusbcfg.usbp);
  //chThdSleepMilliseconds(1000);
  usbStart(serusbcfg.usbp, &usbcfg);
  //usbConnectBus(serusbcfg.usbp);

  //chThdSleepMilliseconds(1000);

  // init msg mutex
  chMtxInit(&msg_lock);

  // Initialize buttons
  for (int n=0; n<51; n++) {
    buttons[n].but_id = n;
    buttons[n].src_id = ID_DIS;
  }
  for (int n=0; n<51; n++) {
    buttons_bas[n].but_id = n;
    buttons_bas[n].src_id = ID_BAS;
  }

  /*
   * Initialize output channels for the buttons as opendrain
   */
  for (int n=0; n<OUT_NUM_CHANNELS; n++) {
    palSetPadMode(out_channels_port[n], out_channels_pad[n], PAL_MODE_OUTPUT_OPENDRAIN);
    palSetPadMode(out_channels_bas_port[n], out_channels_bas_pad[n], PAL_MODE_OUTPUT_OPENDRAIN);
  }

  /*
   * Initializes the ADC driver 1.
   * The pin PA0,PA1,PA2 on the port GPIOA are programmed as analog input.
   */
  adcMultiStart();
  palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
  palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
  palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
  palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);
  palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);
  palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);

  /*
   * Creates the message send thread.
   */
  chThdCreateStatic(waThreadSend, sizeof(waThreadSend), NORMALPRIO, ThreadSend, NULL);

  /*
   * Creates the LED flash thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  adcMultiStartConversion(&adcgrpcfg1, &adcgrpcfg2, &adcgrpcfg3, adc_samples, ADC_GRP1_BUF_DEPTH);
  /*
   * Creates the thread to process the adc samples
   */
  chThdCreateStatic(waThreadReadButtons, sizeof(waThreadReadButtons), NORMALPRIO, ThreadReadButtons, NULL);

  while (1) {
    chThdSleepMilliseconds(500);
  }
}
