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

#include "adc_multi.h"

#include "calib.h"

// force sqrtf to use FPU, the standard one apparently doesn't
float vsqrtf(float op1) {
  float result;
  __ASM volatile ("vsqrt.f32 %0, %1" : "=w" (result) : "w" (op1) );
  return (result);
}

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

#define IDC_SLD_NPRESS 8
#define IDC_SLD_SLIDE 9
#define IDC_SLD_SLIDEZOOM 10

#define INTERNAL_ONE (1<<24)
#define ADCFACT (1<<12)
#define VELOFACT 32
#define MSGFACT (1<<11)
#define MSGFACT_VELO (MSGFACT/VELOFACT)
#define FILT 8

#define ADC_OFFSET (16>>1)
//#define CALIBRATION_MODE TRUE

/* Number of ADCs used in multi ADC mode (2 or 3) */
#define ADC_N_ADCS 3

/* Total number of channels to be sampled by a single ADC operation.*/
#define ADC_GRP1_NUM_CHANNELS_PER_ADC   2

/* Depth of the conversion buffer, channels are sampled one time each.*/
#define ADC_GRP1_BUF_DEPTH      (2*ADC_N_ADCS) // must be 1 or even

#define OUT_NUM_CHANNELS        51
#define N_BUTTONS               68
#define N_BUTTONS_BAS           51

static const ioportid_t out_channels_port[51] = {
  GPIOC, GPIOC, GPIOC, GPIOG, GPIOG, GPIOG, GPIOG, GPIOG,
  GPIOG, GPIOG, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD,
  GPIOD, GPIOB, GPIOB, GPIOB, GPIOH, GPIOH, GPIOH, GPIOH,
  GPIOH, GPIOH, GPIOH, GPIOB, GPIOB, GPIOE, GPIOE, GPIOE, GPIOE,
  GPIOE, GPIOE, GPIOE, GPIOE, GPIOE, GPIOG, GPIOG, GPIOF,
  GPIOF, GPIOF, GPIOF, GPIOF, GPIOB, GPIOB, GPIOC, GPIOC, GPIOA,
};
static const ioportmask_t out_channels_pad[51] = {
  1<< 8, 1<< 7, 1<< 6, 1<< 8, 1<< 7, 1<< 6, 1<< 5, 1<< 4,
  1<< 3, 1<< 2, 1<<15, 1<<14, 1<<13, 1<<12, 1<<11, 1<<10, 1<< 9,
  1<< 8, 1<<15, 1<<14, 1<<13, 1<<12, 1<<11, 1<<10, 1<< 9,
  1<< 8, 1<< 7, 1<< 6, 1<<11, 1<<10, 1<<15, 1<<14, 1<<13, 1<<12,
  1<<11, 1<<10, 1<< 9, 1<< 8, 1<< 7, 1<< 1, 1<< 0, 1<<15,
  1<<14, 1<<13, 1<<12, 1<<11, 1<< 1, 1<< 0, 1<< 5, 1<< 4, 1<< 7,
};

static const ioportid_t out_channels_bas_port[51] = {
  GPIOA, GPIOC, GPIOC, GPIOC, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD,
  GPIOD, GPIOD, GPIOD, GPIOG, GPIOG, GPIOG, GPIOG, GPIOG,
  GPIOG, GPIOG, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB,
  GPIOE, GPIOE, GPIOI, GPIOI, GPIOI, GPIOI, GPIOE, GPIOE,
  GPIOE, GPIOE, GPIOE, GPIOI, GPIOI, GPIOI, GPIOF, GPIOF, GPIOF,
  GPIOF, GPIOF, GPIOF, GPIOF, GPIOF, GPIOF, GPIOF, GPIOF,
};
static const ioportmask_t out_channels_bas_pad[51] = {
  1<<15, 1<<10, 1<<11, 1<<12, 1<< 0, 1<< 1, 1<< 2, 1<< 3, 1<< 4,
  1<< 5, 1<< 6, 1<< 7, 1<< 9, 1<<10, 1<<11, 1<<12, 1<<13,
  1<<14, 1<<15, 1<< 3, 1<< 4, 1<< 5, 1<< 6, 1<< 7, 1<< 8, 1<< 9,
  1<< 0, 1<< 1, 1<< 4, 1<< 5, 1<< 6, 1<< 7, 1<< 2, 1<< 3,
  1<< 4, 1<< 5, 1<< 6, 1<< 9, 1<<10, 1<<11, 1<< 0, 1<< 1, 1<< 2,
  1<< 3, 1<< 4, 1<< 5, 1<< 6, 1<< 7, 1<< 8, 1<< 9, 1<<10,
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
  int32_t c_force;
  int32_t c_offset;
  int pressed;
  int timer;
  int but_id;
  int src_id;
} button_t;

typedef struct struct_slider {
  int32_t s[27];
  int32_t v[27];
  int timer;
  int dbtimer;
  int pres[4];
  int pos[4];
  int velo[4];
  int sort[4];
  int n_press;
  int move;
  int zoom;
} slider_t;

static slider_t sld;

static button_t buttons[N_BUTTONS];
static button_t buttons_bas[N_BUTTONS_BAS];
static int buttons_pressed[2] = {0};

/*
 * ADC samples buffer.
 */
static adcsample_t adc_samples[ADC_GRP1_NUM_CHANNELS_PER_ADC * ADC_N_ADCS * ADC_GRP1_BUF_DEPTH];
static adcsample_t samples0[102] = {0};
static adcsample_t samples1[102] = {0};
static adcsample_t samples2[102] = {0};
static adcsample_t samples3[102] = {0};
static adcsample_t* samples[4] = {samples0, samples1, samples2, samples3};

static adcsample_t samples_bas0[102] = {0};
static adcsample_t samples_bas1[102] = {0};
static adcsample_t* samples_bas[2] = {samples_bas0, samples_bas1};

static void adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
  (void)adcp;
  (void)n;

  /* Open old channel */
  palSetPort(out_channels_port[cur_channel], out_channels_pad[cur_channel]);
  palSetPort(out_channels_bas_port[cur_channel], out_channels_bas_pad[cur_channel]);

  cur_channel = (next_conversion+1) % OUT_NUM_CHANNELS;
  /* Drain new channel */
  palClearPort(out_channels_port[cur_channel], out_channels_pad[cur_channel]);
  palClearPort(out_channels_bas_port[cur_channel], out_channels_bas_pad[cur_channel]);

  // start next ADC conversion
  adcp->adc->CR2 |= ADC_CR2_SWSTART;

  /* copy adc_samples */
  samples0[next_conversion] = buffer[0];
  samples1[next_conversion] = buffer[1];
  samples2[next_conversion] = buffer[2];
  samples3[next_conversion] = buffer[3];

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
  } else if (*s >= INTERNAL_ONE) {
    *s = INTERNAL_ONE - 1;
    *v = 0;
  } else {
    *v = ((FILT-1) * (*v) + (*s - old_s)) / FILT;
    if (*v >= (INTERNAL_ONE/VELOFACT)) {
      *v = (INTERNAL_ONE/VELOFACT) - 1;
    } else if (*v <= -(INTERNAL_ONE/VELOFACT)) {
      *v = -(INTERNAL_ONE/VELOFACT) + 1;
    }
  }
}

int32_t calibrate(int32_t s, int32_t c, int32_t offset) {
  s = s + offset;
  #ifdef CALIBRATION_MODE
  /* keep linear voltage for calibration */
  s = ADCFACT * (4095-s);
  #else
  /* convert adc value to force */
  // c is the normalisation value for the force
  //    2^18   * 2^12 / 2^12 * ADCFACT/2^6 / c
  s = ((1<<18) * (4095-s)/s) * c;
  #endif
  return s;
}

void update_button(button_t* but, adcsample_t* inp) {
  int but_id = but->but_id;
  int32_t s_new;
  int msg[8];
  msg[0] = but->src_id;

  s_new = calibrate(inp[0], but->c_force, but->c_offset);
  update_and_filter(&but->s0, &but->v0, s_new);
  s_new = calibrate(inp[1], but->c_force, but->c_offset);
  update_and_filter(&but->s1, &but->v1, s_new);
  s_new = calibrate(inp[2], but->c_force, but->c_offset);
  update_and_filter(&but->s2, &but->v2, s_new);

  if (but->s0 > (INTERNAL_ONE/64) || but->s1 > 0 || but->s2 > 0) {
    if (but->pressed == 0) {
      but->pressed = 1;
      buttons_pressed[but->src_id]++;
    }
    if (but->timer <= 0) {
      msg[1] = but_id;
      if (but->s0 <= 0)
        msg[2] = 0;
      else
        msg[2] = but->s0 / MSGFACT;
        //msg[2] = (int32_t)(vsqrtf((float)but->s0 / INTERNAL_ONE) * (INTERNAL_ONE / MSGFACT));
      if (but->s1 <= 0)
        msg[3] = 0;
      else
        msg[3] = but->s1 / MSGFACT;
        //msg[3] = (int32_t)(vsqrtf((float)but->s1 / INTERNAL_ONE) * (INTERNAL_ONE / MSGFACT));
      if (but->s2 <= 0)
        msg[4] = 0;
      else
        msg[4] = but->s2 / MSGFACT;
        //msg[4] = (int32_t)(vsqrtf((float)but->s2 / INTERNAL_ONE) * (INTERNAL_ONE / MSGFACT));
      msg[5] = but->v0 / MSGFACT_VELO;
      msg[6] = but->v1 / MSGFACT_VELO;
      msg[7] = but->v2 / MSGFACT_VELO;
      msgSend(8, msg);
      but->timer = (buttons_pressed[0] + buttons_pressed[1]) * SENDFACT;
    } else {
      but->timer--;
    }
  }
  else if (but->pressed) {
    but->pressed = 0;
    buttons_pressed[but->src_id]--;
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
typedef struct struct_slider {
  int32_t s[27];
  int32_t v[27];
  int timer;
  int pres[4];
  int pos[4];
  int velo[4];
  int sort[4];
  int n_press;
  int move;
  int zoom;
} slider_t;
*/
#define SLD_MAX_DIST 2
#define N_PEAKS 8
#define N_PRESS 4
#define N_SENS 27
#define SLD_STEP (1<<8)
int slider_interp(int n) {
  int a = sld.s[n-1];
  int b = sld.s[n];
  int c = sld.s[n+1];
  if (a > c) {
    return n * SLD_STEP + (c-a) / ((b-c)/SLD_STEP) / 2;
  } else {
    return n * SLD_STEP + (c-a) / ((b-a)/SLD_STEP) / 2;
  }
}

void update_slider(void) {
  int n;
  int np = 0;
  int peaks[N_PEAKS];
  int msg[8];
  msg[0] = ID_CONTROL;

  // make slider less sensitive when buttons are pressed, because of crosstalk
  int min_pres = 0;
  for (n=18; n<(18+17); n+=2) {
    if (min_pres < buttons_bas[n].s0)
      min_pres = buttons_bas[n].s0;
    if (min_pres < buttons_bas[n].s1)
      min_pres = buttons_bas[n].s1;
    if (min_pres < buttons_bas[n].s2)
      min_pres = buttons_bas[n].s2;
  }

  // find peaks
  for (n=1; n<27-1; n++) {
    if (sld.s[n] > min_pres + (INTERNAL_ONE/64) && sld.s[n-1] <= sld.s[n] && sld.s[n] > sld.s[n+1]) {
      peaks[np++] = n;
    }
  }

  // debug output
  #ifdef DEBUG_SDU1
  if (SDU1.state == SDU_READY) {
    if (sld.dbtimer <= 0) {
      for (n=0; n<27; n++) {
        chprintf((BaseSequentialStream *)&SDU1, " %4d", sld.s[n]>>10);
      }
      chprintf((BaseSequentialStream *)&SDU1, "\r\n    ");
      for (n=1; n<27-1; n++) {
        chprintf((BaseSequentialStream *)&SDU1, " %4d", sld.s[n] > 0 && sld.s[n-1] <= sld.s[n] && sld.s[n] > sld.s[n+1]);
      }
      chprintf((BaseSequentialStream *)&SDU1, "     npeaks: %d\r\n",np);
      sld.dbtimer = 100;
    } else {
      sld.dbtimer--;
    }
  }
  #endif

  /*
  signals:
  slide
  zoom
  2up
  2down

   */
  if (np != sld.n_press) {
    msg[1] = IDC_SLD_NPRESS;
    msg[2] = np;
    msgSend(3, msg);
  }

  // single slide (volume)
  if (np == 1) {
    int pos = slider_interp(peaks[0]);
    if (sld.n_press == 1) {
      if (sld.timer <= 0) {
        msg[1] = IDC_SLD_SLIDE;
        msg[2] = -(pos - sld.pos[0]);
        msgSend(3, msg);
        sld.timer = 16 * SENDFACT;
        sld.pos[0] = pos;
      } else {
        sld.timer--;
      }
    } else {
      // new press
      sld.pos[0] = pos;
      sld.timer = 16 * SENDFACT;
    }
  }
  // slide/zoom (tuning)
  else if (np == 2) {
    int pos0 = slider_interp(peaks[0]);
    int pos1 = slider_interp(peaks[1]);
    if (sld.n_press == 2) {
      if (sld.timer <= 0) {
        msg[1] = IDC_SLD_SLIDEZOOM;
        msg[2] = ((pos0 + pos1) - (sld.pos[0] + sld.pos[1]))/2;
        msg[3] = (pos1 - pos0) - (sld.pos[1] - sld.pos[0]);
        msgSend(4, msg);
        sld.timer = 16 * SENDFACT;
        sld.pos[0] = pos0;
        sld.pos[1] = pos1;
      } else {
        sld.timer--;
      }
    } else {
      // new press
      sld.pos[0] = pos0;
      sld.pos[1] = pos1;
      sld.timer = 16 * SENDFACT;
    }
  }
  sld.n_press = np;
  // 1 up/down
  // 2 up/down/middle
  // n press


/*
  // match to presses
  for (n = 0; n<np; n++) {
    int dp = INTERNAL_ONE;
    for (k = 0; k<N_PRESS; k++) {
      if (sld.pres[k] > 0) {
        d = abs(sld.pos[k] - peaks[n]);
        if (d < SLD_MAX_DIST && d < dp && dists[k] == INTERNAL_ONE) {
          dp = d;
          pp[n] = k;
          //if (dists[k] < INTERNAL_ONE) {
          //  dists[k] = INTERNAL_ONE;
          //  for (m = 0; m<n; m++)
          //    if pp[m]
          //}
        }
      }
    }
    if (dp != INTERNAL_ONE) {
      dists[pp[n]] = dp;
    }
  }
  // add new presses
  for (n = 0; n<np; n++) {
    if (pp[n] == -1) {
      for (k = 0; k<N_PRESS; k++) {
        if (sld.pres[k] == 0) {
          pp[n] = k;
          dists[k] = 0;
          break;
        }
      }
    }
  }
  // delete ended presses
  for (k = 0; k<N_PRESS; k++) {
    if (sld.pres[k] > 0 && dists[k]== INTERNAL_ONE) {
      sld.pres[k] = 0;
    }
  }

  // update
  for (n = 0; n<N_PRESS; n++) {
    if (sld.pres[n] > 0) {
      update_peak(n);
      // remove doubles
    }
  }*/
}

/*
 * Read out buttons and create messages.
 */
static WORKING_AREA(waThreadReadButtons, 128);
static msg_t ThreadReadButtons(void *arg) {
  (void)arg;

  int32_t s_new;
  int cur_conv, but_id, note_id;
  button_t* but;

  while (TRUE) {
    while (proc_conversion != next_conversion) {
      // process 3 buttons if all 3 values * 3 buttons are available
      if ((proc_conversion % 3) == 2) {
        note_id = (proc_conversion / 3) % 17;
        cur_conv = (proc_conversion - 2);

        /*
         * Update button in each octave/adc-channel
         */
        // dis side
        /* TODO: reduce crosstalk
if self is weak and note in other octave is on or just off:
  subtract a bit
if note in other octave is stronger:
  if self is weak:
    subtract a bit
  if other button in same octave played:
    subtract something based on (self, other octave, other note, 4th note)
    if other octave is almost the same:
      subtract almost nothing
    if other note and/or 4th note are stronger:
    if 4th button is stronger:
      subtract something

 1 6       6
 6 1     6

 2 6       6
 6 6     6 6

 2 6       6
 6 4     6 2

 2 4
 6 4

 2 2
 6 2

 3 2
 6 3

 0
 1       1

m, id = max(octaves)
if m > self:
  mn, idn = max(othernotes)
  if mn:
    self -= ...(self, m, mn, note(id, idn))
  else:
    self -= ...(self, m)
*/

/*
if note in multiple octaves:
  subtract f * (max - n) from n
  max = 8.5 * n (from test with v1.9, on sensor values)
  f = 1/7.5
  but for very light touches 
 */
        for (int m = 0; m < 3; m++) {
          int max = samples[0][cur_conv + m];
          for (int n = 1; n < 4; n++) {
            if (max > samples[n][cur_conv + m]) { // samples is invert so >
              max = samples[n][cur_conv + m];
            }
          }
          for (int n = 0; n < 4; n++) {
            samples[n][cur_conv + m] -= (max - samples[n][cur_conv + m]) / 6; // / 7; // TODO: - k if other notes in this octave
            if (samples[n][cur_conv + m] > 4095) samples[n][cur_conv + m] = 4095;
          }
        }

/*
if four corners are on remove lowest
*/


        for (int n = 0; n < 4; n++) {
          but_id = note_id + n * 17;
          but = &buttons[but_id];
          update_button(but, &samples[n][cur_conv]);
        }
        // bas side
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

          but_id = (note_id / 2) * 3;

          s_new = calibrate(samples_bas[1][cur_conv + 0], (ADCFACT>>6) / 6, ADC_OFFSET);
          update_and_filter(&sld.s[but_id + 0], &sld.v[but_id + 0], s_new);
          s_new = calibrate(samples_bas[1][cur_conv + 1], (ADCFACT>>6) / 6, ADC_OFFSET);
          update_and_filter(&sld.s[but_id + 1], &sld.v[but_id + 1], s_new);
          s_new = calibrate(samples_bas[1][cur_conv + 2], (ADCFACT>>6) / 6, ADC_OFFSET);
          update_and_filter(&sld.s[but_id + 2], &sld.v[but_id + 2], s_new);

          if (note_id == 16) {
            update_slider();
          }
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

  /*
   * Activates the serial driver using the driver default configuration.
   */
  sdStart(&SD1, &ser_cfg);

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
  for (int n=0; n<N_BUTTONS; n++) {
    buttons[n].but_id = n;
    buttons[n].src_id = ID_DIS;
    buttons[n].c_force = calib_dis[n];//(ADCFACT>>6) / 6;
    buttons[n].c_offset = ADC_OFFSET;
  }
  for (int n=0; n<N_BUTTONS_BAS; n++) {
    buttons_bas[n].but_id = n;
    buttons_bas[n].src_id = ID_BAS;
    buttons_bas[n].c_force = calib_bas[n];//(ADCFACT>>6) / 2;
    buttons_bas[n].c_offset = ADC_OFFSET * 16;
  }

  /*
   * Initializes the ADC driver.
   */
  adcMultiStart();

  /*
   * Creates the message send thread.
   */
  chThdCreateStatic(waThreadSend, sizeof(waThreadSend), NORMALPRIO, ThreadSend, NULL);

  /*
   * Creates the LED flash thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Start first ADC conversion. Next conversions are triggered from the adc callback.
   */
  adcMultiStartConversion(&adcgrpcfg1, &adcgrpcfg2, &adcgrpcfg3, adc_samples, ADC_GRP1_BUF_DEPTH);

  /*
   * Creates the thread to process the adc samples
   */
  chThdCreateStatic(waThreadReadButtons, sizeof(waThreadReadButtons), NORMALPRIO, ThreadReadButtons, NULL);

  while (1) {
    chThdSleepMilliseconds(500);
  }
}
