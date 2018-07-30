
#include "button_read.h"
#include "ch.h"
#include "hal.h"

#include "config.h"
#include "striso.h"
#include "synth.h"

#include "messaging.h"
#include "adc_multi.h"


#define INTERNAL_ONE (1<<24)
#define ADCFACT (1<<12)
#define VELOFACT 32
#define MSGFACT (1<<11)
#define MSGFACT_VELO (MSGFACT/VELOFACT)
#define FILT 8  // max:  1<<32 / INTERNAL_ONE = 64

#define INTEGRATED_PRES_TRESHOLD (INTERNAL_ONE/2)
#define SENDFACT    config.message_interval

//#define ADC_SAMPLE_DEF ADC_SAMPLE_3   // 0.05 ms per cycle
//#define ADC_SAMPLE_DEF ADC_SAMPLE_15  // 0.11 ms per cycle
//#define ADC_SAMPLE_DEF ADC_SAMPLE_28  // 0.18 ms per cycle
//#define ADC_SAMPLE_DEF ADC_SAMPLE_56  // 0.33 ms per cycle
//#define ADC_SAMPLE_DEF ADC_SAMPLE_84  // 0.50 ms per cycle
//#define ADC_SAMPLE_DEF ADC_SAMPLE_112 // 0.65 ms per cycle
#define ADC_SAMPLE_DEF ADC_SAMPLE_144 // 0.83 ms per cycle
//#define ADC_SAMPLE_DEF ADC_SAMPLE_480 // 2.7 ms per cycle

#define ADC_OFFSET (16>>1)

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

enum button_status {
  OFF = 0,
  STARTING = 1,
  ON = 2,
  PHANTOM_FLAG = 4,
};

typedef struct struct_button button_t;
struct struct_button {
  int32_t s0;
  int32_t s1;
  int32_t s2;
  int32_t p;
  int32_t v0;
  int32_t v1;
  int32_t v2;
  int32_t c_force;
  int32_t c_offset;
  enum button_status status;
  int timer;
  int but_id;
  int src_id;
  button_t* prev_but;
};

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
static int col_pressed[2][17] = {0};
static int32_t max_pres, max_pres1;

#ifdef USE_AUX_BUTTONS
#define GPIOI_BUTTON_PORT    2
#define GPIOI_BUTTON_UP      1
#define GPIOA_BUTTON_DOWN    9 // GPIOA_UART1_TX
#define GPIOA_BUTTON_ALT    10 // GPIOA_UART1_RX

static const ioportid_t aux_buttons_port[4] = {GPIOI, GPIOI, GPIOA, GPIOA};
static const int aux_buttons_pad[4] = {GPIOI_BUTTON_PORT, GPIOI_BUTTON_UP, GPIOA_BUTTON_DOWN, GPIOA_BUTTON_ALT};
static const int aux_buttons_msg[4] = {IDC_PORTAMENTO, IDC_OCT_UP, IDC_OCT_DOWN, IDC_ALT};
static uint32_t aux_buttons_state[4] = {0};
#endif

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

  /* copy adc_samples */
  samples0[next_conversion] = buffer[0];
  samples1[next_conversion] = buffer[1];
  samples2[next_conversion] = buffer[2];
  samples3[next_conversion] = buffer[3];

  samples_bas0[next_conversion] = buffer[4];
  samples_bas1[next_conversion] = buffer[5];

  next_conversion = (next_conversion+1) % 102;

  // start next ADC conversion
  adcp->adc->CR2 |= ADC_CR2_SWSTART;
}

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
  but->p = but->s0 + but->s1 + but->s2;

  int min_pres = ((but->prev_but->s2 > MSGFACT) + (but->prev_but->s2 > INTERNAL_ONE/4)) * (INTERNAL_ONE/64) // stop ADC reaction time phantom presses
    + max_pres/32   // stop three nearby corner phantom presses TODO: fix for bas
    + buttons_pressed[0] * (INTERNAL_ONE/256); // reduce sensitivity a bit when many buttons are pressed TODO: fix for bas
  if (but->s0 > MSGFACT + min_pres || but->s1 > MSGFACT + min_pres || but->s2 > MSGFACT + min_pres) {
    // if button is off start integration timer
    if (but->status == OFF) {
      but->status = STARTING;
      but->timer = INTEGRATED_PRES_TRESHOLD;
      buttons_pressed[but->src_id]++;
      col_pressed[but->src_id][but_id % 17]++;
    }
    // if button is in start integration reduce timer
    if (but->status == STARTING) {
      but->timer -= but->p; //(but->s0 + but->s1 + but->s2);
    }
    // if button is phantom pressed release it
    if (but->status & PHANTOM_FLAG) {
      if (but->status == (ON | PHANTOM_FLAG)) {
          msg[1] = but_id;
          msg[2] = 0;
          msg[3] = 0;
          msg[4] = 0;
          msg[5] = 0;
          msg[6] = 0;
          msg[7] = 0;
          msgSend(8, msg);
      }
      // reset phantom flag since next round it can change
      but->status = STARTING;
    }
    // if integration is succesful and interval is ready send note message
    else if (--but->timer <= 0) {
      but->status = ON;
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
    }
  }
  else if (but->status) {
    if (but->status & ON) {
      msg[1] = but_id;
      msg[2] = 0;
      msg[3] = 0;
      msg[4] = 0;
      msg[5] = but->v0 / MSGFACT_VELO;
      msg[6] = but->v1 / MSGFACT_VELO;
      msg[7] = but->v2 / MSGFACT_VELO;
      while (msgSend(8, msg)) { // note off messages are more important so keep trying
        chThdSleep(1);
      }
    }
    but->status = OFF;
    buttons_pressed[but->src_id]--;
    col_pressed[but->src_id][but_id % 17]--;
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

  int cur_conv, but_id, note_id;
  button_t* but;

  while (TRUE) {
    while (proc_conversion != next_conversion) {
      // process 3 buttons if all 3 values * 3 buttons are available
      if ((proc_conversion % 3) == 2) {
        note_id = (proc_conversion / 3) % 17;
        cur_conv = (proc_conversion - 2);

        /* Octave crosstalk
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

        // Update button in each octave/adc-channel
        for (int n = 0; n < 4; n++) {
          but_id = note_id + n * 17;
          but = &buttons[but_id];
          update_button(but, &samples[n][cur_conv]);
        }
        
        // Once per cycle, after the last buttons
        if (note_id == 16) {
          // Find maximum pressure (and second to maximum)
          max_pres1 = 0;
          max_pres = 0;
          for (int n = 0; n<17; n++) {
            for (int nr = 0; nr<4; nr++) {
              if (buttons[n + 17*nr].p > max_pres1) {
                max_pres1 = buttons[n + 17*nr].p;
                if (max_pres1 > max_pres) {
                  max_pres1 = max_pres;
                  max_pres = buttons[n + 17*nr].p;
                }
              }
            }
          }
          // Find phantom presses (presses on the fourth corner that appear when 3 corners are pressed)
          // Specific for dis side, on bas side there's only one octave
          for (int n = 0; n<17; n++) {
            if (col_pressed[0][n] >= 2) {
              for (int k = n+1; k<17; k++) {
                if (col_pressed[0][k] >= 2) {
                  for (int nr = 0; nr<4; nr++) {
                    if (buttons[n + 17*nr].status && buttons[k + 17*nr].status) {
                      for (int kr = nr+1; kr<4; kr++) {
                        if (buttons[n + 17*kr].status && buttons[k + 17*kr].status) {
                          // 4 pressed corners are found, give lowest pressure a phantom flag
                          button_t* min = &buttons[n + 17*nr];
                          button_t* b1  = &buttons[k + 17*nr];
                          button_t* b2  = &buttons[n + 17*kr];
                          button_t* b3  = &buttons[k + 17*kr];
                          if (b1->p < min->p) {min = b1;}
                          if (b2->p < min->p) {min = b2;}
                          if (b3->p < min->p) {min = b3;}
                          min->status |= PHANTOM_FLAG;
                        }
                      }
                    }
                  }
                }
              }
            }
          }
          
#ifdef USE_AUX_BUTTONS
          int msg[8];
          int tmpfix = 1; // mistake in pcb, last 3 buttons pull to ground
          for (int n = 0; n < 4; n++) {
            if (aux_buttons_state[n] & 0xff) {
              aux_buttons_state[n]--;
            } else if ((tmpfix == palReadPad(aux_buttons_port[n], aux_buttons_pad[n])) == !aux_buttons_state[n]) {
              msg[0] = ID_CONTROL;
              msg[1] = aux_buttons_msg[n];
              if (aux_buttons_state[n]) {
                msg[2] = 0;
                aux_buttons_state[n] = AUX_BUTTON_DEBOUNCE_TIME;
              } else {
                msg[2] = 1;
                aux_buttons_state[n] = AUX_BUTTON_DEBOUNCE_TIME | 0x100;
                ws2812_write_led(0, 15,  0,  0);
              }
              msgSend(3, msg);
            }
            tmpfix = 0;
          }
#endif // USE_AUX_BUTTONS
        }
        
#ifdef USE_BAS
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

          int32_t s_new;
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
#endif // USE_BAS
      }
      proc_conversion = (proc_conversion+1) % 102;
    }

    chThdSleepMicroseconds(100);
  }
  return 0;
}

void ButtonBoardTest(void) {
  // test for shorted pins
  for (int n=1; n<OUT_NUM_CHANNELS; n++) {
    /* Drain channel */
    palClearPad(out_channels_port[n-1], out_channels_pad[n-1]);
    palClearPad(out_channels_bas_port[n-1], out_channels_bas_pad[n-1]);

    palSetPadMode(out_channels_port[n], out_channels_pad[n], PAL_MODE_INPUT_PULLUP);
    palSetPadMode(out_channels_bas_port[n], out_channels_bas_pad[n], PAL_MODE_INPUT_PULLUP);

    chThdSleep(1);
    if (!palReadPad(out_channels_port[n], out_channels_pad[n])) {
      // pin n and n-1 shorted

    }

    palSetPadMode(out_channels_port[n], out_channels_pad[n], PAL_MODE_OUTPUT_OPENDRAIN);
    palSetPadMode(out_channels_bas_port[n], out_channels_bas_pad[n], PAL_MODE_OUTPUT_OPENDRAIN);
    /* Open channel */
    palSetPad(out_channels_port[n-1], out_channels_pad[n-1]);
    palSetPad(out_channels_bas_port[n-1], out_channels_bas_pad[n-1]);
  }
}

void ButtonReadStart(void) {
  
#ifdef USE_AUX_BUTTONS
  palSetPadMode(GPIOI, GPIOI_BUTTON_PORT, PAL_MODE_INPUT_PULLDOWN);
  palSetPadMode(GPIOI, GPIOI_BUTTON_UP,   PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOA, GPIOA_BUTTON_DOWN, PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOA, GPIOA_BUTTON_ALT,  PAL_MODE_INPUT_PULLUP);
#endif
  
  /*
   * Initializes the ADC driver.
   */
  adcMultiStart();
  
  // Initialize buttons
  for (int n=0; n<N_BUTTONS; n++) {
    buttons[n].but_id = n;
    buttons[n].src_id = ID_DIS;
    buttons[n].c_force = (ADCFACT>>6) / 12;//calib_dis[n];//(ADCFACT>>6) / 6;
    buttons[n].c_offset = ADC_OFFSET;
    buttons[n].prev_but = &buttons[(n/17) * 17 + ((n+17-1) % 17)];
  }
  for (int n=0; n<N_BUTTONS_BAS; n++) {
    buttons_bas[n].but_id = n;
    buttons_bas[n].src_id = ID_BAS;
    buttons_bas[n].c_force = (ADCFACT>>6) / 6;//calib_bas[n];//(ADCFACT>>6) / 2;
    buttons_bas[n].c_offset = ADC_OFFSET;
    buttons_bas[n].prev_but = &buttons_bas[(n/17) * 17 + ((n+17-1) % 17)];
  }

  /*
   * Start first ADC conversion. Next conversions are triggered from the adc callback.
   */
  adcMultiStartConversion(&adcgrpcfg1, &adcgrpcfg2, &adcgrpcfg3, adc_samples, ADC_GRP1_BUF_DEPTH);

  /*
   * Creates the thread to process the adc samples
   */
  chThdCreateStatic(waThreadReadButtons, sizeof(waThreadReadButtons), NORMALPRIO, ThreadReadButtons, NULL);
}
