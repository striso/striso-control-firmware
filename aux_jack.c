#include "ch.h"
#include "codec.h"
#include "hal.h"

#include "aux_jack.h"
#include "ccportab.h"
#include "config_store.h"
#include "midi_serial.h"

/* Total number of channels to be sampled by a single ADC operation.*/
#define ADC_GRP3_NUM_CHANNELS   2

/* Depth of the conversion buffer, channels are sampled one time each.*/
#define ADC_GRP3_BUF_DEPTH      1 // must be 1 or even.

const ADCConfig adccfg3 = {
  .difsel       = 0U,
  .calibration  = 0U
};

// AUX jack Tip ADC: IN10
// AUX jack Ring ADC: IN11

const ADCConversionGroup adcgrpcfg3 = {
  .circular     = TRUE,
  .num_channels = ADC_GRP3_NUM_CHANNELS,
  .end_cb       = NULL,
  .error_cb     = NULL,
  .cfgr         = ADC_CFGR_RES_12BITS,
  .cfgr2        = 0U,
  .ccr          = 0U,
  .pcsel        = ADC_SELMASK_IN10 | ADC_SELMASK_IN11,
  .ltr1         = 0x00000000U,
  .htr1         = 0x03FFFFFFU,
  .ltr2         = 0x00000000U,
  .htr2         = 0x03FFFFFFU,
  .ltr3         = 0x00000000U,
  .htr3         = 0x03FFFFFFU,
  .smpr         = {
    0U,
    ADC_SMPR2_SMP_AN10(ADC_SMPR_SMP_810P5) |
    ADC_SMPR2_SMP_AN11(ADC_SMPR_SMP_810P5)
  },
  .sqr          = {
    ADC_SQR1_SQ1_N(ADC_CHANNEL_IN10) | ADC_SQR1_SQ2_N(ADC_CHANNEL_IN11),
    0U,
    0U,
    0U
  }
};

CC_ALIGN(CACHE_LINE_SIZE) static adcsample_t adc_samples[CACHE_SIZE_ALIGN(adcsample_t, ADC_GRP3_NUM_CHANNELS * ADC_GRP3_BUF_DEPTH)];

void aux_jack_init(void) {
  const char* s = getConfigSetting("sGjack2 ");
  if (cmp8(s, "pedal   ")) {
    aux_jack_mode_pedal();
    config.jack2_mode = JACK2_MODE_PEDAL;
  } else if (cmp8(s, "linein  ")) {
    codec_linein_enable();
    config.jack2_mode = JACK2_MODE_LINEIN;
  } else { // if (cmp8(s, "midi    ")) {
#ifdef USE_MIDI_SERIAL
    serial_midi_init();
    config.jack2_mode = JACK2_MODE_MIDI;
#endif
  }
}

void detect_jack(void) {

}

void aux_jack_mode_midi(void) {

}

void aux_jack_mode_pedal(void) {

}

void aux_jack_mode_audio_in(void) {

}
