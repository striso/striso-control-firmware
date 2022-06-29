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
#include "button_read.h"
#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "ccportab.h"

#include "config.h"
#include "striso.h"
#include "synth.h"
#include "usbcfg.h"
#include "led.h"

#include "messaging.h"
#ifdef STM32F4XX
#include "adc_multi.h"
#endif

#define INTERNAL_ONE (1<<24)
#define ADC_BITS 12
#define ADC_MAX ((1<<ADC_BITS)-1)
#define ADCFACT (INTERNAL_ONE >> ADC_BITS)  // factor from adc sample to INTERNAL_ONE
#define VELOFACT 32      // extra factor for velocity
#define MSGFACT (1<<11)  // factor from 14 bit message to INTERNAL_ONE
#define MSGFACT_VELO (MSGFACT/VELOFACT)
#define FILT 8  // min: 1 (no filter), max: 64 (1<<32 / INTERNAL_ONE)
#define FILTV 8 // min: 1 (no filter), max: 64 (1<<32 / INTERNAL_ONE)
#define ZERO_LEVEL_FACT 300 / 256  // safety factor for zero level. Without brackets so multiplication goes before division
#define ZERO_LEVEL_TIME 500
#define ZERO_LEVEL_MAX_PRES (INTERNAL_ONE/32)
#define ZERO_LEVEL_MAX_VELO 500
#define COMMON_CHANNEL_FILT 0.5
#define KEY_DETECT 64   // key_detect threshold
#define KEY_DETECT2 (256-KEY_DETECT)  // additional threshold when another key in the column is pressed
#define KEY_DETECT3 (320-KEY_DETECT2-KEY_DETECT) // additional threshold when 3 or 4 corners are pressed
#define MIN_MEASURES 4 // minimum notes to measure, must be >= 2
#define MULTISAMPLE 4  // multisampling of pressure, also hardcoded in some places

#define INTEGRATED_PRES_TRESHOLD (INTERNAL_ONE/8)
#define SENDFACT    config.message_interval

#ifdef STM32F4XX
//#define ADC_SAMPLE_DEF ADC_SAMPLE_3   // 0.05 ms per cycle
//#define ADC_SAMPLE_DEF ADC_SAMPLE_15  // 0.11 ms per cycle
//#define ADC_SAMPLE_DEF ADC_SAMPLE_28  // 0.18 ms per cycle
//#define ADC_SAMPLE_DEF ADC_SAMPLE_56  // 0.33 ms per cycle
//#define ADC_SAMPLE_DEF ADC_SAMPLE_84  // 0.50 ms per cycle
//#define ADC_SAMPLE_DEF ADC_SAMPLE_112 // 0.65 ms per cycle
#define ADC_SAMPLE_DEF ADC_SAMPLE_144 // 0.83 ms per cycle
//#define ADC_SAMPLE_DEF ADC_SAMPLE_480 // 2.7 ms per cycle

/* Number of ADCs used in multi ADC mode (2 or 3) */
#define ADC_N_ADCS 3

/* Total number of channels to be sampled by a single ADC operation.*/
#define ADC_GRP1_NUM_CHANNELS_PER_ADC   2

/* Depth of the conversion buffer, channels are sampled one time each.*/
#define ADC_GRP1_BUF_DEPTH      1 // must be 1 or even

#define ADC_GRP1_NUM_CHANNELS (ADC_GRP1_NUM_CHANNELS_PER_ADC * ADC_N_ADCS)

#elif defined(STM32H7XX)
// timing calculation: ADCCLK/(SMP+6.5)/51/4 (*2 for dual ADC)
//#define ADC_SAMPLE_DEF ADC_SMPR_SMP_1P5   //
//#define ADC_SAMPLE_DEF ADC_SMPR_SMP_2P5   //
//#define ADC_SAMPLE_DEF ADC_SMPR_SMP_8P5   //
//#define ADC_SAMPLE_DEF ADC_SMPR_SMP_16P5  //
//#define ADC_SAMPLE_DEF ADC_SMPR_SMP_32P5 // 1174 Hz @10MHz single ADC measured (1257 Hz calculated)
#define ADC_SAMPLE_DEF ADC_SMPR_SMP_64P5  // 1289 Hz @10MHz dual ADC measured (1380 Hz calculated)
//#define ADC_SAMPLE_DEF ADC_SMPR_SMP_384P5 // 123.6 Hz @10MHz single ADC measured (125.4 Hz calculated) 245 Hz dual ADC
//#define ADC_SAMPLE_DEF ADC_SMPR_SMP_810P5 //

/* Total number of channels to be sampled by a single ADC operation.*/
#define ADC_GRP1_NUM_CHANNELS   4

/* Depth of the conversion buffer, channels are sampled one time each.*/
#define ADC_GRP1_BUF_DEPTH      1 // must be 1 or even. Strange behaviour when it is 2.

#endif

#define ADC_OFFSET (16>>1)

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
static const iopadid_t out_channels_pad[51] = {
   8,  7,  6,  8,  7,  6,  5,  4,
   3,  2, 15, 14, 13, 12, 11, 10,  9,
   8, 15, 14, 13, 12, 11, 10,  9,
   8,  7,  6, 11, 10, 15, 14, 13, 12,
  11, 10,  9,  8,  7,  1,  0, 15,
  14, 13, 12, 11,  1,  0,  5,  4,  7,
};
static const ioportmask_t out_channels_portmask[51] = {
  1<< 8, 1<< 7, 1<< 6, 1<< 8, 1<< 7, 1<< 6, 1<< 5, 1<< 4,
  1<< 3, 1<< 2, 1<<15, 1<<14, 1<<13, 1<<12, 1<<11, 1<<10, 1<< 9,
  1<< 8, 1<<15, 1<<14, 1<<13, 1<<12, 1<<11, 1<<10, 1<< 9,
  1<< 8, 1<< 7, 1<< 6, 1<<11, 1<<10, 1<<15, 1<<14, 1<<13, 1<<12,
  1<<11, 1<<10, 1<< 9, 1<< 8, 1<< 7, 1<< 1, 1<< 0, 1<<15,
  1<<14, 1<<13, 1<<12, 1<<11, 1<< 1, 1<< 0, 1<< 5, 1<< 4, 1<< 7,
};

#ifdef USE_BAS
static const ioportid_t out_channels_bas_port[51] = {
  GPIOA, GPIOC, GPIOC, GPIOC, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD,
  GPIOD, GPIOD, GPIOD, GPIOG, GPIOG, GPIOG, GPIOG, GPIOG,
  GPIOG, GPIOG, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB,
  GPIOE, GPIOE, GPIOI, GPIOI, GPIOI, GPIOI, GPIOE, GPIOE,
  GPIOE, GPIOE, GPIOE, GPIOI, GPIOI, GPIOI, GPIOF, GPIOF, GPIOF,
  GPIOF, GPIOF, GPIOF, GPIOF, GPIOF, GPIOF, GPIOF, GPIOF,
};
static const iopadid_t out_channels_bas_pad[51] = {
  15, 10, 11, 12,  0,  1,  2,  3,  4,
   5,  6,  7,  9, 10, 11, 12, 13,
  14, 15,  3,  4,  5,  6,  7,  8,  9,
   0,  1,  4,  5,  6,  7,  2,  3,
   4,  5,  6,  9, 10, 11,  0,  1,  2,
   3,  4,  5,  6,  7,  8,  9, 10,
};
static const ioportmask_t out_channels_bas_portmask[51] = {
  1<<15, 1<<10, 1<<11, 1<<12, 1<< 0, 1<< 1, 1<< 2, 1<< 3, 1<< 4,
  1<< 5, 1<< 6, 1<< 7, 1<< 9, 1<<10, 1<<11, 1<<12, 1<<13,
  1<<14, 1<<15, 1<< 3, 1<< 4, 1<< 5, 1<< 6, 1<< 7, 1<< 8, 1<< 9,
  1<< 0, 1<< 1, 1<< 4, 1<< 5, 1<< 6, 1<< 7, 1<< 2, 1<< 3,
  1<< 4, 1<< 5, 1<< 6, 1<< 9, 1<<10, 1<<11, 1<< 0, 1<< 1, 1<< 2,
  1<< 3, 1<< 4, 1<< 5, 1<< 6, 1<< 7, 1<< 8, 1<< 9, 1<<10,
};
#endif // USE_BAS

static int cur_channel = 0;
static int cur_conversion = 0;
static int cur_phase = 0;
static volatile int next_note_id = 0;

enum button_status {
  OFF = 0,
  STARTING = 1,
  ON = 2,
};

typedef struct struct_button button_t;
struct struct_button {
  int32_t on;
  int32_t p;
  int32_t s0;
  int32_t s1;
  int32_t s2;
  int32_t pres;
  int32_t velo;
  int32_t c_force;
  int32_t c_breakpoint;
  int32_t c_force2;
  int32_t zero_offset;
  int32_t zero_time;
  int32_t zero_max;
  int32_t key_detect3;
  float fact;
  enum button_status status;
  int timer;
  int but_id;
  int src_id;
};

#ifdef USE_BAS
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
#endif

static button_t buttons[N_BUTTONS];
#ifdef USE_BAS
static button_t buttons_bas[N_BUTTONS_BAS];
#endif
static int buttons_pressed[2] = {0};
static int col_pressed[2][17] = {0};

#ifdef USE_AUX_BUTTONS
// #define LINE_BUTTON_PORT   PAL_LINE(GPIOI,  2U)
// #define LINE_BUTTON_UP     PAL_LINE(GPIOI,  1U)
// #define LINE_BUTTON_DOWN   PAL_LINE(GPIOA,  9U) // GPIOA_UART1_TX
// #define LINE_BUTTON_ALT    PAL_LINE(GPIOA, 10U) // GPIOA_UART1_RX

static const ioline_t aux_buttons_line[4] = {LINE_BUTTON_PORT, LINE_BUTTON_UP, LINE_BUTTON_DOWN, LINE_BUTTON_ALT};
static const bool aux_buttons_on[4] = {false, true, false, false};
static const int aux_buttons_msg[4] = {IDC_PORTAMENTO, IDC_OCT_UP, IDC_OCT_DOWN, IDC_ALT};
static uint32_t aux_buttons_state[4] = {0};
#endif

/*
 * ADC samples buffer.
 */
/* Buffers are allocated with size and address aligned to the cache
   line size.*/
#if CACHE_LINE_SIZE > 0
#define CC_CACHE_ALIGN CC_ALIGN(CACHE_LINE_SIZE)
#else
#define CC_CACHE_ALIGN
#endif
//CC_SECTION(".ram3")
CC_CACHE_ALIGN static adcsample_t adc_samples[CACHE_SIZE_ALIGN(adcsample_t, ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH)];
// static adcsample_t adc_samples[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];

static int measure[20] = {0};
static int *measure_put = measure;
static int *measure_get = measure;

static thread_t *tpReadButtons = NULL;

static void adccallback(ADCDriver *adcp) {
  // invalidate buffer after DMA transfer
  cacheBufferInvalidate(adc_samples, sizeof (adc_samples) / sizeof (adcsample_t));

  if (cur_conversion < 17) { // key press detection phase
    int next_conversion = cur_conversion + 1;
    /* Open old channels */
    palSetPort(out_channels_port[cur_channel+0], out_channels_portmask[cur_channel+0]);
    palSetPort(out_channels_port[cur_channel+1], out_channels_portmask[cur_channel+1]);
    palSetPort(out_channels_port[cur_channel+2], out_channels_portmask[cur_channel+2]);
    if (next_conversion == 17) {
      // switch to measure phase
      for (int n=0; n<OUT_NUM_CHANNELS; n++) {
        palSetPadMode(out_channels_port[n], out_channels_pad[n], PAL_MODE_OUTPUT_OPENDRAIN | PAL_STM32_OSPEED_HIGHEST);
      }
      measure_get = measure;
      next_note_id = *measure_get;
      cur_channel = next_note_id * 3;
    } else {
      cur_channel = next_conversion * 3;
    }
    /* Drain new channels */
    palClearPort(out_channels_port[cur_channel+0], out_channels_portmask[cur_channel+0]);
    palClearPort(out_channels_port[cur_channel+1], out_channels_portmask[cur_channel+1]);
    palClearPort(out_channels_port[cur_channel+2], out_channels_portmask[cur_channel+2]);

    // store values
    buttons[cur_conversion   ].on = 4095 - adc_samples[0];
    buttons[cur_conversion+17].on = 4095 - adc_samples[1];
    buttons[cur_conversion+34].on = 4095 - adc_samples[2];
    buttons[cur_conversion+51].on = 4095 - adc_samples[3];
    if (adc_samples[0] < (4095-KEY_DETECT) ||
        adc_samples[1] < (4095-KEY_DETECT) ||
        adc_samples[2] < (4095-KEY_DETECT) ||
        adc_samples[3] < (4095-KEY_DETECT)) {
          *measure_put++ = cur_conversion;
    }

    cur_conversion = next_conversion;
  } else if (cur_conversion == 17) { // key measurement phase
    switch (cur_phase) {
    case 0: { // read whole button, 4x multisampled (MULTISAMPLE hardcoded)
      buttons[next_note_id   ].p = 4095 - adc_samples[0];
      buttons[next_note_id+17].p = 4095 - adc_samples[1];
      buttons[next_note_id+34].p = 4095 - adc_samples[2];
      buttons[next_note_id+51].p = 4095 - adc_samples[3];

      cur_phase++;
    } break;
    case 1:
    case 2: {
      buttons[next_note_id   ].p += 4095 - adc_samples[0];
      buttons[next_note_id+17].p += 4095 - adc_samples[1];
      buttons[next_note_id+34].p += 4095 - adc_samples[2];
      buttons[next_note_id+51].p += 4095 - adc_samples[3];

      cur_phase++;
    } break;
    case 3: {
      /* Open old channels */
      palSetPort(out_channels_port[cur_channel+1], out_channels_portmask[cur_channel+1]);
      palSetPort(out_channels_port[cur_channel+2], out_channels_portmask[cur_channel+2]);

      buttons[next_note_id   ].p = buttons[next_note_id   ].p + 4095 - adc_samples[0];
      buttons[next_note_id+17].p = buttons[next_note_id+17].p + 4095 - adc_samples[1];
      buttons[next_note_id+34].p = buttons[next_note_id+34].p + 4095 - adc_samples[2];
      buttons[next_note_id+51].p = buttons[next_note_id+51].p + 4095 - adc_samples[3];

      cur_phase = 100;
    } break;
    case 100: { // read s0
      /* Open old channels */
      palSetPort(out_channels_port[cur_channel+0], out_channels_portmask[cur_channel+0]);
      /* Drain new channels */
      palClearPort(out_channels_port[cur_channel+1], out_channels_portmask[cur_channel+1]);

      buttons[next_note_id   ].s0 = (4095 - adc_samples[0]) * MULTISAMPLE;
      buttons[next_note_id+17].s0 = (4095 - adc_samples[1]) * MULTISAMPLE;
      buttons[next_note_id+34].s0 = (4095 - adc_samples[2]) * MULTISAMPLE;
      buttons[next_note_id+51].s0 = (4095 - adc_samples[3]) * MULTISAMPLE;

      cur_phase = 101;
    } break;
    case 101: { // read s1
      /* Open old channels */
      palSetPort(out_channels_port[cur_channel+1], out_channels_portmask[cur_channel+1]);
      /* Drain new channels */
      palClearPort(out_channels_port[cur_channel+2], out_channels_portmask[cur_channel+2]);

      buttons[next_note_id   ].s1 = (4095 - adc_samples[0]) * MULTISAMPLE;
      buttons[next_note_id+17].s1 = (4095 - adc_samples[1]) * MULTISAMPLE;
      buttons[next_note_id+34].s1 = (4095 - adc_samples[2]) * MULTISAMPLE;
      buttons[next_note_id+51].s1 = (4095 - adc_samples[3]) * MULTISAMPLE;

      cur_phase = 102;
    } break;
    case 102: { // read s2
      /* Open old channels */
      palSetPort(out_channels_port[cur_channel+2], out_channels_portmask[cur_channel+2]);

      buttons[next_note_id   ].s2 = (4095 - adc_samples[0]) * MULTISAMPLE;
      buttons[next_note_id+17].s2 = (4095 - adc_samples[1]) * MULTISAMPLE;
      buttons[next_note_id+34].s2 = (4095 - adc_samples[2]) * MULTISAMPLE;
      buttons[next_note_id+51].s2 = (4095 - adc_samples[3]) * MULTISAMPLE;

      // Next channel
      measure_get++;
      if (measure_get < measure_put) {
        next_note_id = *measure_get;
        cur_channel = next_note_id * 3;
      } else {
        if (measure_put - measure < MIN_MEASURES) {
          // switch to delay phase
          cur_conversion = 20 + 4 * (measure_put - measure);
          next_note_id = 16; // process all but last notes to keep from hanging at note_id 0
        } else {
          // switch to detection phase
          cur_conversion = 0;
          next_note_id = 0;
        }
        measure_put = measure;
        cur_channel = 0;
        for (int n=0; n<OUT_NUM_CHANNELS; n++) {
          palSetPadMode(out_channels_port[n], out_channels_pad[n], PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
        }
      }

      /* Drain new channels */
      palClearPort(out_channels_port[cur_channel+0], out_channels_portmask[cur_channel+0]);
      palClearPort(out_channels_port[cur_channel+1], out_channels_portmask[cur_channel+1]);
      palClearPort(out_channels_port[cur_channel+2], out_channels_portmask[cur_channel+2]);

      cur_phase = 0;

      // Wake up processing thread
      chSysLockFromISR();
      if (tpReadButtons != NULL) {
        chSchReadyI(tpReadButtons);
        tpReadButtons = NULL;
      }
      chSysUnlockFromISR();
    } break;
    }
  } else { // delay phase
    /* When less than MIN_MEASURES buttons are measured delay a bit so the
       total conversion time and velocity sensitivity stay constant.
       When more than MIN_MEASURES buttons are measured it slows down, increasing
       the velocity sensitivity */
    cur_conversion++;
    if (cur_conversion >= 20 + (MULTISAMPLE + 3) * MIN_MEASURES) {
      // switch to detection phase
      cur_conversion = 0;
      next_note_id = 0;

      // Wake up processing thread
      chSysLockFromISR();
      if (tpReadButtons != NULL) {
        chSchReadyI(tpReadButtons);
        tpReadButtons = NULL;
      }
      chSysUnlockFromISR();
    }
  }

  // start next ADC conversion
#if defined(STM32F4XX)
  adcp->adc->CR2 |= ADC_CR2_SWSTART;
#elif defined(STM32H7XX)
  adcp->adcm->CR |= ADC_CR_ADSTART;
#endif

}

#ifdef STM32F4XX
/*
 * ADC conversion group for ADC0 as multi ADC mode master.
 * Mode:        Circular buffer, triple ADC mode master, SW triggered.
 * Channels:    PA0, PA3
 */
static const ADCConversionGroup adcgrpcfg1 = {
  TRUE, // Circular conversion
  ADC_GRP1_NUM_CHANNELS_PER_ADC * ADC_N_ADCS,
  adccallback, /* end of conversion callback */
  NULL, /* error callback */
  /* HW dependent part.*/
  0, // CR1
  ADC_CR2_SWSTART, // CR2
  0, // SMPR1
  ADC_SMPR2_SMP_AN0(ADC_SAMPLE_DEF)
   | ADC_SMPR2_SMP_AN3(ADC_SAMPLE_DEF), // SMPR2
  0, // HTR
  0, // LTR
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
  0, // HTR
  0, // LTR
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
  0, // HTR
  0, // LTR
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS_PER_ADC), // SQR1
  0, // SQR2
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN2)
   | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN10) // SQR3
};
#endif

#ifdef STM32H7XX
const ADCConfig adccfg1 = {
  .difsel       = 0U,
  .calibration  = 0U
};
#if STM32_ADC_DUAL_MODE == TRUE
/*
 * ADC conversion group 1.
 * Mode:        One shot, 2 channels, SW triggered.
 * Channels:    IN0, IN5.
 */
const ADCConversionGroup adcgrpcfg1 = {
  .circular     = TRUE,
  .num_channels = ADC_GRP1_NUM_CHANNELS,
  .end_cb       = adccallback,
  .error_cb     = NULL,
  .cfgr         = ADC_CFGR_RES_12BITS,
  .cfgr2        = 0U,
  .ccr          = ADC_CCR_DUAL_REG_SIMULT, // 6U
  .pcsel        = ADC_SELMASK_IN3 | ADC_SELMASK_IN19 | ADC_SELMASK_IN18 | ADC_SELMASK_IN15,
  .ltr1         = 0x00000000U,
  .htr1         = 0x03FFFFFFU,
  .ltr2         = 0x00000000U,
  .htr2         = 0x03FFFFFFU,
  .ltr3         = 0x00000000U,
  .htr3         = 0x03FFFFFFU,
  .smpr         = {
    0U,
    ADC_SMPR2_SMP_AN15(ADC_SAMPLE_DEF) | ADC_SMPR2_SMP_AN19(ADC_SAMPLE_DEF)
  },
  .sqr          = {
    ADC_SQR1_SQ1_N(ADC_CHANNEL_IN15) | ADC_SQR1_SQ2_N(ADC_CHANNEL_IN19),
    0U,
    0U,
    0U
  },
  .ssmpr        = {
    ADC_SMPR1_SMP_AN3(ADC_SAMPLE_DEF),
    ADC_SMPR2_SMP_AN18(ADC_SAMPLE_DEF)
  },
  .ssqr         = {
    ADC_SQR1_SQ1_N(ADC_CHANNEL_IN18) | ADC_SQR1_SQ2_N(ADC_CHANNEL_IN3),
    0U,
    0U,
    0U
  }
};
#else // STM32_ADC_DUAL_MODE == FALSE
const ADCConversionGroup adcgrpcfg1 = {
  .circular     = TRUE,
  .num_channels = ADC_GRP1_NUM_CHANNELS,
  .end_cb       = adccallback,
  .error_cb     = NULL,
  .cfgr         = ADC_CFGR_RES_12BITS,
  .cfgr2        = 0U,
  .ccr          = 0U,
  .pcsel        = ADC_SELMASK_IN3 | ADC_SELMASK_IN19 | ADC_SELMASK_IN18 | ADC_SELMASK_IN15,
  .ltr1         = 0x00000000U,
  .htr1         = 0x03FFFFFFU,
  .ltr2         = 0x00000000U,
  .htr2         = 0x03FFFFFFU,
  .ltr3         = 0x00000000U,
  .htr3         = 0x03FFFFFFU,
  .smpr         = {
    ADC_SMPR1_SMP_AN3(ADC_SAMPLE_DEF),
    ADC_SMPR2_SMP_AN19(ADC_SAMPLE_DEF) |
    ADC_SMPR2_SMP_AN18(ADC_SAMPLE_DEF) |
    ADC_SMPR2_SMP_AN15(ADC_SAMPLE_DEF)
  },
  .sqr          = {
    ADC_SQR1_SQ1_N(ADC_CHANNEL_IN15) | ADC_SQR1_SQ2_N(ADC_CHANNEL_IN18) |
    ADC_SQR1_SQ3_N(ADC_CHANNEL_IN19) | ADC_SQR1_SQ4_N(ADC_CHANNEL_IN3),
    0U,
    0U,
    0U
  }
};
#endif // STM32_ADC_DUAL_MODE
#endif // STM32H7XX

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
  if (*s >= INTERNAL_ONE) {
    *s = INTERNAL_ONE - 1;
    *v = 0;
  } else {
    *v = ((FILTV-1) * (*v) + (*s - old_s)) / FILTV;
    if (*v >= (INTERNAL_ONE/VELOFACT)) {
      *v = (INTERNAL_ONE/VELOFACT) - 1;
    } else if (*v <= -(INTERNAL_ONE/VELOFACT)) {
      *v = -(INTERNAL_ONE/VELOFACT) + 1;
    }
    if (*s < 0) {
      *s = 0;
    }
  }
}

int32_t linearize(int32_t s) {
#ifdef CALIBRATION_MODE
  /* keep linear voltage for calibration */
  return ADCFACT / MULTISAMPLE * s;
#else
  /* convert adc value to force */
  return (ADCFACT>>6) * s/((MULTISAMPLE*4095)-s+1);
#endif // CALIBRATION_MODE
}

int32_t calibrate(int32_t s, button_t* but) {
#ifdef CALIBRATION_MODE
  return s;
#endif
  // c is the normalisation value for the force
  //    2^18   * 2^12 / 2^12 * ADCFACT/2^6 / c
  // s = (but->c_force * (4095-s)/(s+1)) * (ADCFACT>>6);
  s = (but->c_force * s) * but->fact;
  #ifdef BREAKPOINT_CALIBRATION
  // breakpoint calibration
  if (s > but->c_breakpoint) {
    s += but->c_force2 * ((s - but->c_breakpoint)>>8);
  }
  #endif // BREAKPOINT_CALIBRATION
  return s;
}

#define max(x,y) ((x)>(y)?(x):(y))
#define min(x,y) ((x)<(y)?(x):(y))

void update_button(button_t* but) {
  int but_id = but->but_id;
  int32_t s_new;
  int msg[8];
  msg[0] = but->src_id;

  int key_detect2 = KEY_DETECT2 * (col_pressed[but->src_id][but_id % 17] - (but->status != OFF) >= 1);

  if (but->on > KEY_DETECT + key_detect2 + but->key_detect3) {

    s_new = calibrate(linearize(but->p), but);
    // four corner correction algoritm
    if (but->key_detect3) {
      for (button_t* but2 = &buttons[but_id % 17]; but2 < &buttons[N_BUTTONS]; but2 = &but2[17]) {
        if (but != but2 && but2->key_detect3 && but2->status) {
          int32_t s_new2 = calibrate(linearize(but2->p), but2);
          if (s_new2 > s_new) {
            s_new -= (s_new2 - s_new) / 2;
          } else {
            s_new -= (s_new2 - s_new);
          }
          break;
        }
      }
    }
    update_and_filter(&but->pres, &but->velo, s_new);

#ifdef DETECT_STUCK_NOTES
    // adjust zero pressure level dynamically
    if (but->pres < ZERO_LEVEL_MAX_PRES
        && but->velo < ZERO_LEVEL_MAX_VELO
        && but->velo > -ZERO_LEVEL_MAX_VELO) {
      if (but->pres > but->zero_max) but->zero_max = but->pres;
      but->zero_time++;
      if (but->zero_time > ZERO_LEVEL_TIME) {
        but->zero_time = 0;
#ifdef DETECT_STUCK_NOTES_DECREASE
        but->zero_offset = but->zero_max * ZERO_LEVEL_FACT;
#else
        but->zero_offset = max(but->zero_offset, but->zero_max * ZERO_LEVEL_FACT);
#endif
      }
    } else {
      but->zero_time = 0;
      but->zero_max = 0;
    }
#endif

    // if button is off start integration timer
    if (but->status == OFF) {
      but->status = STARTING;
      but->timer = INTEGRATED_PRES_TRESHOLD;
      col_pressed[but->src_id][but_id % 17]++;
    }
    // if button is in start integration reduce timer
    if (but->status == STARTING && but->pres > (config.zero_offset + but->zero_offset + MSGFACT)) {
      but->timer -= (but->pres - but->zero_offset - MSGFACT);
    }
    // note off if .pres is too low even though .on is high enough
    else if (but->status == ON && but->pres < (config.zero_offset / 2 + but->zero_offset + MSGFACT)) {
      but->status = STARTING;
      buttons_pressed[but->src_id]--;
      but->timer = INTEGRATED_PRES_TRESHOLD;

      msg[1] = but_id;
      msg[2] = 0;
      msg[3] = min(but->velo, 0) / MSGFACT_VELO;
      msg[4] = 0;
      msg[5] = 0;
      while (msgSend(6, msg)) { // note off messages are more important so keep trying
        chThdSleep(1);
      }
    }
    // if integration is succesful and interval is ready send note message
    if (--but->timer <= 0) {
      if (but->status != ON) {
        but->status = ON;
        buttons_pressed[but->src_id]++;
      }

      // calculate values from signals
      #define CENTERTEND 0.02f
      int32_t but_x, but_y;
      int32_t s0 = calibrate(linearize(but->s0), but);
      int32_t s1 = calibrate(linearize(but->s1), but);
      int32_t s2 = calibrate(linearize(but->s2), but);
      // m = max(s0, s1, s2)
      int32_t m = s0;
      if (s1 > m) m = s1;
      if (s2 > m) m = s2;
      if (m > 0) {
          float mf = ((float)m)/INTERNAL_ONE;
          float fact = 1.0f/(mf + CENTERTEND/mf - CENTERTEND);
          but_x = (s2 - s0) * fact;
          but_y = ((s0 + s2) / 2 - s1) * fact;
      } else {
          but_x = 0;
          but_y = 0;
      }

      msg[1] = but_id;
      msg[2] = but->pres / MSGFACT;
      msg[3] = but->velo / MSGFACT_VELO; // but->on;// s_new / MSGFACT; //
      msg[4] = but_x / MSGFACT;
      msg[5] = but_y / MSGFACT;
      msgSend(6, msg);
      but->timer = (buttons_pressed[0] + buttons_pressed[1]) * SENDFACT;
    }
  }
  else if (but->status) {
    if (but->status == ON) {
      msg[1] = but_id;
      msg[2] = 0;
      msg[3] = min(but->velo, 0) / MSGFACT_VELO;
      msg[4] = 0;
      msg[5] = 0;
      while (msgSend(6, msg)) { // note off messages are more important so keep trying
        chThdSleep(1);
      }
      buttons_pressed[but->src_id]--;
    }
    but->status = OFF;
    but->p = 0;
    col_pressed[but->src_id][but_id % 17]--;
    // reset filter
    but->pres = 0;
    but->velo = 0;
    but->zero_time = 0;
  } else {
    but->p = 0;
#ifdef DETECT_STUCK_NOTES_DECREASE
    but->zero_time++;
    if (but->zero_time == ZERO_LEVEL_TIME) {
      but->zero_offset = 0;
    }
#endif
  }
  but->fact = 1.0f;
  but->key_detect3 = 0;
}

#ifdef USE_BAS
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
#endif

/*
 * Read out buttons and create messages.
 */
static THD_WORKING_AREA(waThreadReadButtons, 128);
static void ThreadReadButtons(void *arg) {
  (void)arg;

  chRegSetThreadName("read_buttons");
  int note_id = 0;

#ifdef DETECT_STUCK_NOTES
  int count = 0;
  while (count < 100) {
    while (count < 100 && note_id != next_note_id) {
      for (int n = 0; n < 4; n++) {
        button_t* but = &buttons[note_id + n * 17];
        if (but->on > KEY_DETECT) {
          int32_t s_new = calibrate(linearize(but->p), but);
          update_and_filter(&but->pres, &but->velo, s_new);
          s_new = but->pres * ZERO_LEVEL_FACT;
          if (s_new > but->zero_offset) {
            but->zero_offset = s_new;
          }
        }
      }
      // Once per cycle, after the last buttons
      if (note_id == 16) {
        count++;
      }
      note_id = (note_id + 1) % 17;
    }

    chSysLock();
    tpReadButtons = chThdGetSelfX();
    chSchGoSleepS(CH_STATE_SUSPENDED);
    chSysUnlock();
  }
#endif // DETECT_STUCK_NOTES

  while (TRUE) {
    while (note_id != next_note_id) {
        // Update button in each octave/adc-channel
        for (int n = 0; n < 4; n++) {
          update_button(&buttons[note_id + n * 17]);
        }

        // calculate cross talk correction factors
        /* correction factor for 1k adc pull up resistors:
           1: 600/570 = 1.05
           2: 600/320 = 1.9
           3: 600/230 = 2.6
           4: 600/180 = 3.3
        */
        float oct_fact[4] = {1.0f};
        for (int n = 0; n < 4; n++) {
          float fact = 1.0f + buttons[note_id + n * 17].p * (0.05f / 0.9f / (MULTISAMPLE*4095.0f));
          oct_fact[n] = max(oct_fact[n], fact);
          for (int k = n+1; k < 4; k++) {
            fact = 1.0f + (min(buttons[note_id + n * 17].p, buttons[note_id + k * 17].p) - (KEY_DETECT+KEY_DETECT2))
                   * (0.9f / 0.95f / (MULTISAMPLE*4095.0f));
            oct_fact[n] = max(oct_fact[n], fact);
            oct_fact[k] = max(oct_fact[k], fact);
          }
        }
        bool set_oct[4];
        for (int b = 0; b < 17; b++) {
          if (b != note_id) {
            for (int n = 0; n < 4; n++) {
              set_oct[n] = true;
            }
            for (int n = 0; n < 4; n++) {
              for (int k = n+1; k < 4; k++) {
                if (buttons[note_id + n * 17].status != OFF && buttons[note_id + k * 17].status != OFF &&
                    (buttons[b + k * 17].status != OFF || buttons[b + n * 17].status != OFF)) {
                  if (buttons[b + n * 17].status != OFF && buttons[b + k * 17].status != OFF) {
                    // four corners pressed, do not set correction and reduce sensitivity
                    set_oct[n] = false;
                    set_oct[k] = false;
                    buttons[b + n * 17].key_detect3 = KEY_DETECT3 | 1;
                    buttons[b + k * 17].key_detect3 = KEY_DETECT3 | 1;
                  } else {
                    // three corners pressed, only reduce sensitivity for not pressed corner.
                    // | 1 as signal to the four corner correction algoritm
                    buttons[b + n * 17].key_detect3 = KEY_DETECT3 * (buttons[b + k * 17].status != OFF) | 1;
                    buttons[b + k * 17].key_detect3 = KEY_DETECT3 * (buttons[b + n * 17].status != OFF) | 1;
                  }
                }
              }
              // set correction if larger than current correction
              if (set_oct[n] && oct_fact[n] > buttons[b + n * 17].fact) {
                buttons[b + n * 17].fact = oct_fact[n];
              }
            }
          }
        }
        // Once per cycle, after the last buttons
        if (note_id == 16) {
#ifdef USE_AUX_BUTTONS
          int msg[8];
          for (int n = 0; n < 4; n++) {
            if (aux_buttons_state[n] & 0xff) {
              aux_buttons_state[n]--;
            } else if (palReadLine(aux_buttons_line[n]) == !aux_buttons_state[n]) {
              msg[0] = ID_CONTROL;
              msg[1] = aux_buttons_msg[n];
              if (aux_buttons_state[n]) {
                msg[2] = !aux_buttons_on[n];
                aux_buttons_state[n] = AUX_BUTTON_DEBOUNCE_TIME;
              } else {
                msg[2] = aux_buttons_on[n];
                aux_buttons_state[n] = AUX_BUTTON_DEBOUNCE_TIME | 0x100;
              }
              msgSend(3, msg);
            }
          }
#endif // USE_AUX_BUTTONS
        }

      note_id = (note_id + 1) % 17;
    }

    chSysLock();
    tpReadButtons = chThdGetSelfX();
    chSchGoSleepS(CH_STATE_SUSPENDED);
    chSysUnlock();
  }
}

void ButtonBoardTest(void) {
  // test for shorted pins
  for (int n=1; n<OUT_NUM_CHANNELS; n++) {
    /* Drain channel */
    palClearPad(out_channels_port[n-1], out_channels_pad[n-1]);
#ifdef USE_BAS
    palClearPad(out_channels_bas_port[n-1], out_channels_bas_pad[n-1]);
#endif

    palSetPadMode(out_channels_port[n], out_channels_pad[n], PAL_MODE_INPUT_PULLUP);
#ifdef USE_BAS
    palSetPadMode(out_channels_bas_port[n], out_channels_bas_pad[n], PAL_MODE_INPUT_PULLUP);
#endif

    chThdSleep(1);
    if (!palReadPad(out_channels_port[n], out_channels_pad[n])) {
      // pin n and n-1 shorted

    }

    palSetPadMode(out_channels_port[n], out_channels_pad[n], PAL_MODE_OUTPUT_OPENDRAIN);
#ifdef USE_BAS
    palSetPadMode(out_channels_bas_port[n], out_channels_bas_pad[n], PAL_MODE_OUTPUT_OPENDRAIN);
#endif
    /* Open channel */
    palSetPad(out_channels_port[n-1], out_channels_pad[n-1]);
#ifdef USE_BAS
    palSetPad(out_channels_bas_port[n-1], out_channels_bas_pad[n-1]);
#endif
  }
}

void buttonSetCalibration(void) {
  uint16_t base_calib_force = devspec_id->base_calib_force;
  // use default when not device flash not yet initialized
  if (base_calib_force == 0xffff) base_calib_force = CALIB_FORCE;
  // fall back to old default when info not available (old struct)
  if (base_calib_force == 0x0000) base_calib_force = ((1<<18)/64);

  for (int n=0; n<N_BUTTONS; n++) {
    chprintf((BaseSequentialStream *)&BDU1, "but[%d] c_force: %d zero_offset: %d\r\n", n, buttons[n].c_force, buttons[n].zero_offset);
    buttons[n].c_force = base_calib_force;
    buttons[n].c_breakpoint = INT32_MAX;
  }
  chprintf((BaseSequentialStream *)&BDU1, "c_force: %d\r\n", base_calib_force);
}

void ButtonReadStart(void) {

#if defined(USE_AUX_BUTTONS) && defined(STM32F4XX)
  palSetLineMode(LINE_BUTTON_PORT, PAL_MODE_INPUT_PULLDOWN);
  palSetLineMode(LINE_BUTTON_UP,   PAL_MODE_INPUT_PULLDOWN);
  palSetLineMode(LINE_BUTTON_DOWN, PAL_MODE_INPUT_PULLDOWN);
  palSetLineMode(LINE_BUTTON_ALT,  PAL_MODE_INPUT_PULLDOWN);
#endif

  for (int n=0; n<OUT_NUM_CHANNELS; n++) {
    // palSetPadMode(out_channels_port[n], out_channels_pad[n], PAL_MODE_OUTPUT_OPENDRAIN | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(out_channels_port[n], out_channels_pad[n], PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
  }

  /*
   * Initializes the ADC driver.
   */
#ifdef STM32F4XX
  adcMultiStart();
#elif defined(STM32H7XX)
  adcStart(&ADCD1, &adccfg1);
#endif

  // Initialize buttons
  for (int n=0; n<N_BUTTONS; n++) {
    buttons[n].but_id = n;
    buttons[n].src_id = ID_DIS;
    buttons[n].c_force = CALIB_FORCE;
    buttons[n].key_detect3 = 0;
    buttons[n].pres = 0;
    buttons[n].velo = 0;
    buttons[n].zero_offset = 0;
    buttons[n].zero_time = 0;
    buttons[n].zero_max = 0;
    buttons[n].fact = 1.0f;
  }
  // disable not existing buttons
  buttons[52].c_force = 0;
  buttons[54].c_force = 0;
  buttons[57].c_force = 0;
  buttons[59].c_force = 0;
  buttons[61].c_force = 0;
  buttons[64].c_force = 0;
  buttons[66].c_force = 0;
#ifdef USE_BAS
  for (int n=0; n<N_BUTTONS_BAS; n++) {
    buttons_bas[n].but_id = n;
    buttons_bas[n].src_id = ID_BAS;
    buttons_bas[n].c_force = (ADCFACT>>6) / 6;//calib_bas[n];//(ADCFACT>>6) / 2;
    buttons_bas[n].c_offset = ADC_OFFSET;
  }
#endif

  uint32_t* const UID = (uint32_t*)UID_BASE;

  if (calib_dis_force->UID[0] == UID[0] &&
      calib_dis_force->UID[1] == UID[1] &&
      calib_dis_force->UID[2] == UID[2]) {
    for (int n=0; n<N_BUTTONS; n++) {
#ifdef BREAKPOINT_CALIBRATION
      buttons[n].c_force = calib_dis_force->calib[n];
#else
      if (calib_dis_force->type == 0x02) { /* devspec_id->hardware_revision == 1 */
        buttons[n].c_force = 2 * calib_dis_force->calib[n]; // * 2 to make up for non-linearity
      } else if (calib_dis_force->type == 0x00) { /* devspec_id->hardware_revision == 0 */
        // calibrated per sensor, so should be / 3 to get the same sensitivity.
        // However with 10k pullup the higher force measurement is not stable, so make it 1.5 times more sensitive
        buttons[n].c_force = 2 * calib_dis_force->calib[n] / 2;
      }
#endif
    }

    CC_ALIGN(8) char key[8] = "fCoffC_";
    float f;
    for (int n = 0; n < 61; n++) {
      key[2] = 'f';
      key[3] = 'a';
      key[4] = 'c';
      put_button_name(n, &key[5]);
      int but = button_number_map[n];
      f = getConfigFloat(key);
      if (f != 1.0f && f >= 0.0f && f <= 10.0f) {
        buttons[but].c_force = (int32_t)(buttons[but].c_force * f + 0.5f);
      }
      key[2] = 'o';
      key[3] = 'f';
      key[4] = 'f';
      f = getConfigFloat(key);
      if (f > 0.0f && f <= 1.0f) {
        buttons[but].zero_offset = f * INTERNAL_ONE;
      }
    }
  } else {
    led_rgb(0xff0000);
    chThdSleepMilliseconds(200);
  }

  if (calib_dis_breakpoint->UID[0] == UID[0] &&
      calib_dis_breakpoint->UID[1] == UID[1] &&
      calib_dis_breakpoint->UID[2] == UID[2] &&
      calib_dis_breakpoint->type == 0xb1) {
    for (int n=0; n<N_BUTTONS; n++) {
      buttons[n].c_breakpoint = calib_dis_breakpoint->calib[n] << 16;
    }
  } else {
    for (int n=0; n<N_BUTTONS; n++) {
      buttons[n].c_breakpoint = INT32_MAX;
    }
  }
  if (calib_dis_force2->UID[0] == UID[0] &&
      calib_dis_force2->UID[1] == UID[1] &&
      calib_dis_force2->UID[2] == UID[2] &&
      calib_dis_force2->type == 0x01) {
    for (int n=0; n<N_BUTTONS; n++) {
      buttons[n].c_force2 = calib_dis_force2->calib[n];
    }
  }

  /*
   * Start first ADC conversion. Next conversions are triggered from the adc callback.
   */
#ifdef STM32F4XX
  adcMultiStartConversion(&adcgrpcfg1, &adcgrpcfg2, &adcgrpcfg3, adc_samples, ADC_GRP1_BUF_DEPTH);
#elif defined(STM32H7XX)
  adcStartConversion(&ADCD1, &adcgrpcfg1, adc_samples, ADC_GRP1_BUF_DEPTH);
#endif

  /*
   * Creates the thread to process the adc samples
   */
  chThdCreateStatic(waThreadReadButtons, sizeof(waThreadReadButtons), NORMALPRIO, ThreadReadButtons, NULL);
}
