#include "ch.h"
#include "codec.h"
#include "config.h"
#include "hal.h"

#include "aux_jack.h"
#include "ccportab.h"
#include "config_store.h"
#include "midi_serial.h"
#include "messaging.h"
#include "striso.h"

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
  .circular     = FALSE,
  .num_channels = ADC_GRP3_NUM_CHANNELS,
  .end_cb       = NULL,
  .error_cb     = NULL,
  .cfgr         = ADC_CFGR_RES_14BITS,
  .cfgr2        = ADC_CFGR2_ROVSE | ADC_CFGR2_OVSR_N(127) | ADC_CFGR2_OVSS_N(7), // 128x oversampling, gives around 80Hz sample rate
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
    ADC_SMPR2_SMP_AN11(ADC_SMPR_SMP_64P5)
  },
  .sqr          = {
    ADC_SQR1_SQ1_N(ADC_CHANNEL_IN10) | ADC_SQR1_SQ2_N(ADC_CHANNEL_IN11),
    0U,
    0U,
    0U
  }
};

// ADC3 samples need to be in ram4!
CC_SECTION(".ram4") CC_ALIGN(CACHE_LINE_SIZE) static adcsample_t aux_adc_samples[CACHE_SIZE_ALIGN(adcsample_t, ADC_GRP3_NUM_CHANNELS * ADC_GRP3_BUF_DEPTH)];

static THD_WORKING_AREA(waAuxJack, 128);
static THD_FUNCTION(AuxJack, arg) {
  (void)arg;
  chRegSetThreadName("auxjack");
  while (true) {
    /* Performing a one-shot conversion on two channels.*/
    adcConvert(&ADCD3, &adcgrpcfg3, aux_adc_samples, ADC_GRP3_BUF_DEPTH);
    cacheBufferInvalidate(aux_adc_samples, sizeof (aux_adc_samples) / sizeof (adcsample_t));
    if (config.jack2_mode == JACK2_MODE_PEDAL) {
      int msg[3];
      msg[0] = ID_CONTROL;
      msg[1] = IDC_PEDAL;
      msg[2] = aux_adc_samples[0] >> 1;
      msgSend(3, msg);
    }
  }
}

void aux_jack_init(void) {
  palSetLineMode(LINE_AUX_ADC_T, PAL_MODE_INPUT_ANALOG);
  palSetLineMode(LINE_AUX_ADC_R, PAL_MODE_INPUT_ANALOG);

  palSetLineMode(LINE_AUX_UART2_RX, PAL_MODE_INPUT);

  palSetLineMode(LINE_AUX_JACK_DETECT, PAL_MODE_INPUT_PULLUP);

  palSetLineMode(LINE_AUX_UART2_TX, PAL_MODE_INPUT_ANALOG);

  palSetLineMode(LINE_AUX_VDD, PAL_MODE_OUTPUT_OPENDRAIN);
  // Disable AUX power on ring
  palSetLine(LINE_AUX_VDD); // aux_power_disable();

  adcStart(&ADCD3, &adccfg3);

  const char* s = getConfigSetting("sGjack2 ");
  if (cmp8(s, "pedal   ")) {
    aux_jack_switch_mode(JACK2_MODE_PEDAL);
  } else if (cmp8(s, "linein  ")) {
    aux_jack_switch_mode(JACK2_MODE_LINEIN);
  } else if (cmp8(s, "midi    ")) {
#ifdef USE_MIDI_SERIAL
    aux_jack_switch_mode(JACK2_MODE_MIDI);
#endif
  } else {
    aux_jack_switch_mode(JACK2_MODE_DISABLED);
  }

  chThdCreateStatic(waAuxJack, sizeof(waAuxJack), NORMALPRIO, AuxJack, NULL);
}

void pedal_enable(void) {
  // Enable AUX power on ring
  palClearLine(LINE_AUX_VDD); // aux_power_enable();
}

void pedal_disable(void) {
  palSetLine(LINE_AUX_VDD); // aux_power_disable();
}

void aux_jack_switch_mode(jack2_mode_t mode) {
  if (mode == config.jack2_mode) {
    return;
  }
  palSetLine(LINE_AUX_VDD); // aux_power_disable();

  switch (config.jack2_mode) {
  case (JACK2_MODE_DISABLED): {
  } break;
  case (JACK2_MODE_MIDI): {
    serial_midi_disable();
  } break;
  case (JACK2_MODE_PEDAL): {
    pedal_disable();
  } break;
  case (JACK2_MODE_LINEIN): {
    codec_linein_disable();
  } break;
  }

  switch (mode) {
  case (JACK2_MODE_DISABLED): {
    config.jack2_mode = JACK2_MODE_DISABLED;
  } break;
  case (JACK2_MODE_MIDI): {
    serial_midi_enable();
    config.jack2_mode = JACK2_MODE_MIDI;
  } break;
  case (JACK2_MODE_PEDAL): {
    pedal_enable();
    config.jack2_mode = JACK2_MODE_PEDAL;
  } break;
  case (JACK2_MODE_LINEIN): {
    codec_linein_enable();
    config.jack2_mode = JACK2_MODE_LINEIN;
  } break;
  }
}

void detect_jack(void) {
  /*
  Jack possibilities:
  - no jack
  - loose stereo jack
  - midi
  - mono jack/switch pedal
  - stereo jack with switch pedal
  - stereo jack with double/triple switch pedal
  - stereo jack with expression pedal, wiper on tip
  - stereo jack with expression pedal, wiper on ring (not supported, could be done but power on tip has no short-circuit safety)
  - line-in

  for jack detect tip should be 0

  jack detect is important for line-in

  line-in, loose jack and midi are hard to differenciate
  */
  if (palReadLine(LINE_AUX_JACK_DETECT) == 0) {
    // jack disconnected
  } else {
    // jack might be connected
  }
}
