#include "ch.h"
#include "hal.h"

#include <stdlib.h>

#include "codec.h"
#include "config.h"
#include "aux_jack.h"
#include "ccportab.h"
#include "config_store.h"
#include "midi_serial.h"
#include "messaging.h"
#include "striso.h"
#include "led.h"

int aux_power = 0;
int jack_detect = 31;
int short_circuit_count = 0;

int last_pedal_exp = 0;
bool last_pedal1 = 0;
bool last_pedal2 = 1;
bool last_pedal3 = 0;

void aux_power_enable(void) {
  // Enable AUX power on ring
  palClearLine(LINE_AUX_VDD);
  aux_power = 1;
}

void aux_power_disable(void) {
  // Disable AUX power on ring
  palSetLine(LINE_AUX_VDD);
  aux_power = 0;
}

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

  int msg[6];
  msg[0] = ID_CONTROL;
  bool tip_was_closed = FALSE;

  while (true) {
    /* Performing a one-shot conversion on two channels.*/
    adcConvert(&ADCD3, &adcgrpcfg3, aux_adc_samples, ADC_GRP3_BUF_DEPTH);
    cacheBufferInvalidate(aux_adc_samples, sizeof (aux_adc_samples) / sizeof (adcsample_t));
    // check jack unplug
    switch (config.jack2_mode) {
      case JACK2_MODE_DISABLED: { // Jack detection
        // jack_detect high, tx input_pulldown, rx input_pulldown, aux_power off

        if (aux_adc_samples[0] < 500) { // if tip/tx goes low, jack is inserted
          tip_was_closed = FALSE;
          aux_jack_switch_mode(config.jack2_mode_setting);
        }
      } break;
      case JACK2_MODE_AUTODETECT: {
        // jack_detect high, tx input_pulldown, rx input_analog, aux_power on

        /*
        Jack possibilities:
        - no jack
        - loose stereo jack
        - midi out type A (power on ring) -> cyan blink
        - midi in type A (not supported), no conflict -> currently blinks heavily
        - midi out type B (power on tip) (not supported) -> red blinking error (if > 3.3V)
        - midi in type B (not supported), avoid MIDI output on tip! -> red blinking error (if > 3.3V)
        - mono jack/switch pedal -> pink blink
        - stereo jack with switch pedal -> pink blink
        - stereo jack with double/triple switch pedal -> pink blink
        - stereo jack with expression pedal, wiper on tip -> yellow blink, if pedal is not set at an extreme
        - stereo jack with expression pedal, wiper on ring (not supported, could be done but power on tip has no short-circuit safety) -> yellow blink or error
        - line-in -> yellow blink or error, depending on output impedance
        - CV in on tip (through 1k resistors like with the Continuum) -> yellow, or error if outside of 3.3V range
        - CV in on tip and ring (not supported (yet?)) -> not tested
        - serial tx/rx -> not planned
        */

        /* Note: Pins are not 5V tolerant if pull-up or pull-down are enabled,
           according to datasheet page 215:
         3. For operation with voltage higher than Min (VDD, VDDA, VDD33USB) +0.3V,
            the internal Pull-up and Pull-Down resistors must be disabled
         */

        // keep checking for switch pedal, expession pedal, MIDI connection, or jack unplug

        int pulldown = aux_adc_samples[0];
        palSetLineMode(LINE_AUX_UART2_TX, PAL_MODE_INPUT_PULLUP);
        chThdSleepMilliseconds(1);
        adcConvert(&ADCD3, &adcgrpcfg3, aux_adc_samples, ADC_GRP3_BUF_DEPTH);
        cacheBufferInvalidate(aux_adc_samples, sizeof (aux_adc_samples) / sizeof (adcsample_t));
        int pullup = aux_adc_samples[0];

        // check if signal is stable
        palSetLineMode(LINE_AUX_UART2_TX, PAL_MODE_INPUT_PULLDOWN);
        chThdSleepMilliseconds(1);
        adcConvert(&ADCD3, &adcgrpcfg3, aux_adc_samples, ADC_GRP3_BUF_DEPTH);
        cacheBufferInvalidate(aux_adc_samples, sizeof (aux_adc_samples) / sizeof (adcsample_t));
        int pulldown2 = aux_adc_samples[0];
        palSetLineMode(LINE_AUX_UART2_TX, PAL_MODE_INPUT_PULLUP);
        chThdSleepMilliseconds(1);
        adcConvert(&ADCD3, &adcgrpcfg3, aux_adc_samples, ADC_GRP3_BUF_DEPTH);
        cacheBufferInvalidate(aux_adc_samples, sizeof (aux_adc_samples) / sizeof (adcsample_t));
        if (abs(pulldown2 - pulldown) > 100 || abs(aux_adc_samples[0] - pullup) > 100) {
          // signal not stable, reset pin state and skip detection
          palSetLineMode(LINE_AUX_UART2_TX, PAL_MODE_INPUT_PULLDOWN);
          break;
        }

        palSetLineMode(LINE_AUX_UART2_TX, PAL_MODE_INPUT_ANALOG);
        chThdSleepMilliseconds(1);
        adcConvert(&ADCD3, &adcgrpcfg3, aux_adc_samples, ADC_GRP3_BUF_DEPTH);
        cacheBufferInvalidate(aux_adc_samples, sizeof (aux_adc_samples) / sizeof (adcsample_t));
        int floating = aux_adc_samples[0];
        palSetLineMode(LINE_AUX_UART2_TX, PAL_MODE_INPUT_PULLDOWN);
        bool downfixed = floating < 6 * pulldown;
        bool upfixed = ((1<<14)-floating) < 6 * ((1<<14)-pullup);

        // msg[0] = ID_SYS;
        // msg[1] = 99;
        // msg[2] = floating >> 1;
        // msg[3] = pulldown >> 1;
        // msg[4] = pullup >> 1;
        // msg[5] = aux_adc_samples[1] >> 1;
        // msgSend(6, msg);
        // msg[0] = ID_CONTROL;

        if (pulldown == (1<<14)-1 || aux_adc_samples[1] == (1<<14)-1) {
          // >3.3V input on one of the lines
          aux_jack_switch_mode(JACK2_MODE_ERROR);
        } else if (aux_adc_samples[1] < 6000) {
          // short circuit on ring, mono jack or double/triple switch with ring switch closed
          if (short_circuit_count++ > 5) {
            aux_jack_switch_mode(JACK2_MODE_PEDAL_SWITCH);
            break;
          }
        } else if (pullup < 500) {
          // if tip fixed to 0 -> switch pedal (closed) (or expression pedal at zero)
          tip_was_closed = TRUE;
        } else if (pulldown < 500 && pullup > 16000) {
          // if tip floating, loose jack cable or switch pedal
          if (tip_was_closed) {
            // if tip has been closed since insertion -> switch pedal
            aux_jack_switch_mode(JACK2_MODE_PEDAL_SWITCH);
          }
        } else if (pulldown > 15000) { // 15890
          // jack unplugged,
          // or expression pedal at max,
          // or short between tip and ring,
          // or power suplied on tip, e.g. MIDI in type B

          // check if really unplugged
          int unplugged = 1;
          for (int i = 0; unplugged && i < 10; i++) {
            palClearLine(LINE_AUX_JACK_DETECT);
            chThdSleepMicroseconds(100);
            unplugged &= !palReadLine(LINE_AUX_UART2_TX);
            palSetLine(LINE_AUX_JACK_DETECT);
            chThdSleepMicroseconds(100);
            unplugged &= palReadLine(LINE_AUX_UART2_TX);
          }
          if (unplugged) aux_jack_switch_mode(JACK2_MODE_DISABLED);
        } else if (upfixed && downfixed) {
          // if tip holds value -> expression pedal. Support from 10k till 500k Ohm. Half way would be 250k with 40k pullup
          aux_jack_switch_mode(JACK2_MODE_PEDAL_EXPRESSION);
        } else if (pullup > 16000 && pulldown > 1000) {
          // if midi is connected there's an optocoupler from ring to tip that prevents tip from going low with a 40kOhm pulldown
          // if tip goes high but doesn't go all the way low -> midi
          // Tests: 5200 with axoloti, 5100 with Casio keyboard (out of 8192)
          aux_jack_switch_mode(JACK2_MODE_MIDI);
        }
        short_circuit_count = 0;
      } break;
      case JACK2_MODE_MIDI: {
        // jack_detect high, tx alternate, rx input_analog, aux_power on

        // check jack unplug
        if (aux_adc_samples[0] > 15000) {
          aux_jack_switch_mode(JACK2_MODE_DISABLED);
        }
        if (aux_adc_samples[1] < 6000) {
          // short circuit
          if (short_circuit_count++ > 5) {
            aux_jack_switch_mode(JACK2_MODE_ERROR);
          }
        } else {
          short_circuit_count = 0;
        }
      } break;
      case JACK2_MODE_LINEIN: {
        // jack_detect high, tx input_analog, rx input_analog, aux_power off

        // check jack unplug
        if (aux_adc_samples[0] > 15000) {
          aux_jack_switch_mode(JACK2_MODE_DISABLED);
        }
      } break;
      case JACK2_MODE_PEDAL_SWITCH: {
        // jack_detect high, tx input_pullup, rx input_pullup, aux_power off

        // check jack unplug
        palSetLineMode(LINE_AUX_UART2_TX, PAL_MODE_INPUT_PULLDOWN);
        chThdSleepMicroseconds(100);
        int unplugged = palReadLine(LINE_AUX_UART2_TX);

        for (int i = 0; unplugged && i < 10; i++) {
          palClearLine(LINE_AUX_JACK_DETECT);
          chThdSleepMicroseconds(100);
          unplugged &= !palReadLine(LINE_AUX_UART2_TX);
          palSetLine(LINE_AUX_JACK_DETECT);
          chThdSleepMicroseconds(100);
          unplugged &= palReadLine(LINE_AUX_UART2_TX);
        }
        palSetLineMode(LINE_AUX_UART2_TX, PAL_MODE_INPUT_PULLUP);

        if (unplugged) {
          aux_jack_switch_mode(JACK2_MODE_DISABLED);
          // make sure to send pedal off messages after unplug
          aux_adc_samples[0] = 1<<14;
          aux_adc_samples[1] = 1<<14;
        }

        if (last_pedal1 != (aux_adc_samples[0] < 1<<13)) {
          last_pedal1 = !last_pedal1;
          msg[1] = IDC_PEDAL_1;
          msg[2] = last_pedal1;
          msgSend(3, msg);
        }
        if (last_pedal2 != (aux_adc_samples[1] < 1<<13)) {
          last_pedal2 = !last_pedal2;
          msg[1] = IDC_PEDAL_2;
          msg[2] = last_pedal2;
          msgSend(3, msg);
        }
        // TODO: detect third button which closes both switches through a diode
      } break;
      case JACK2_MODE_PEDAL_EXPRESSION: {
        // jack_detect high, tx input_analog, rx input_analog, aux_power on

        if (aux_adc_samples[0] >> 9 == jack_detect) {
          // maybe jack unplug, check with jack_detect low
          int unplugged = 1;
          palSetLineMode(LINE_AUX_UART2_TX, PAL_MODE_INPUT);
          for (int i = 0; unplugged && i < 10; i++) {
            palClearLine(LINE_AUX_JACK_DETECT);
            chThdSleepMicroseconds(100);
            unplugged &= !palReadLine(LINE_AUX_UART2_TX);
            palSetLine(LINE_AUX_JACK_DETECT);
            chThdSleepMicroseconds(100);
            unplugged &= palReadLine(LINE_AUX_UART2_TX);
          }
          palSetLineMode(LINE_AUX_UART2_TX, PAL_MODE_INPUT_ANALOG);

          if (unplugged) {
            jack_detect = 31;
            aux_jack_switch_mode(JACK2_MODE_DISABLED);
            break;
          }
          if (jack_detect) {
            jack_detect = 0;
            palClearLine(LINE_AUX_JACK_DETECT);
          } else {
            jack_detect = 31;
          }
        }
        if (aux_adc_samples[1] < 6000) {
          // short circuit
          if (short_circuit_count++ > 5) {
            aux_jack_switch_mode(JACK2_MODE_ERROR);
          }
        } else {
          short_circuit_count = 0;
        }

        msg[1] = IDC_PEDAL_EXP;
        msg[2] = aux_adc_samples[0] >> 1;
        msg[3] = aux_adc_samples[1] >> 1;
        msgSend(4, msg);
      } break;
      case JACK2_MODE_ERROR: {
        if (short_circuit_count++ > 80) {
          led_rgb3_blink(90, 0, 0, TIME_MS2I(200)); // red blink, error
          short_circuit_count = 0;
        }

        int unplugged = 1;
        for (int i = 0; unplugged && i < 10; i++) {
          palClearLine(LINE_AUX_JACK_DETECT);
          chThdSleepMicroseconds(100);
          unplugged &= !palReadLine(LINE_AUX_UART2_TX);
          palSetLine(LINE_AUX_JACK_DETECT);
          chThdSleepMicroseconds(100);
          unplugged &= palReadLine(LINE_AUX_UART2_TX);
        }
        if (unplugged) aux_jack_switch_mode(JACK2_MODE_DISABLED);
      } break;
    }
  }
}

void aux_jack_init(void) {
  palSetLineMode(LINE_AUX_ADC_T, PAL_MODE_INPUT_ANALOG);
  palSetLineMode(LINE_AUX_ADC_R, PAL_MODE_INPUT_ANALOG);

  palSetLineMode(LINE_AUX_UART2_RX, PAL_MODE_INPUT_PULLDOWN);
  palSetLineMode(LINE_AUX_UART2_TX, PAL_MODE_INPUT_PULLDOWN);

  palSetLineMode(LINE_AUX_JACK_DETECT, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLine(LINE_AUX_JACK_DETECT);

  palSetLineMode(LINE_AUX_VDD, PAL_MODE_OUTPUT_OPENDRAIN);
  // Disable AUX power on ring
  aux_power_disable();

  adcStart(&ADCD3, &adccfg3);

  const char* s = getConfigSetting("sGjack2 ");
  if (cmp8(s, "pedal_ex")) {
    aux_jack_switch_mode_setting(JACK2_MODE_PEDAL_EXPRESSION);
  } else if (cmp8(s, "pedal_sw")) {
    aux_jack_switch_mode_setting(JACK2_MODE_PEDAL_SWITCH);
  } else if (cmp8(s, "midi    ")) {
    aux_jack_switch_mode_setting(JACK2_MODE_MIDI);
  } else if (cmp8(s, "linein  ")) {
    aux_jack_switch_mode_setting(JACK2_MODE_LINEIN);
  } else { // assume "auto"
    aux_jack_switch_mode_setting(JACK2_MODE_AUTODETECT);
  }

  chThdCreateStatic(waAuxJack, sizeof(waAuxJack), NORMALPRIO, AuxJack, NULL);
}

void aux_jack_switch_mode(jack2_mode_t mode) {
  if (mode == config.jack2_mode) {
    return;
  }
  aux_power_disable();
  palSetLine(LINE_AUX_JACK_DETECT);

  switch (config.jack2_mode) {
  case (JACK2_MODE_MIDI): {
#ifdef USE_MIDI_SERIAL
    serial_midi_disable();
#endif
  } break;
  case (JACK2_MODE_LINEIN): {
    codec_linein_disable();
  } break;
  }

  switch (mode) {
  case (JACK2_MODE_DISABLED): {
    palSetLineMode(LINE_AUX_UART2_TX, PAL_MODE_INPUT_PULLDOWN);
    palSetLineMode(LINE_AUX_UART2_RX, PAL_MODE_INPUT_PULLDOWN);
    config.jack2_mode = JACK2_MODE_DISABLED;
    led_rgb3_blink(90, 20, 0, TIME_MS2I(500)); // red, unplug
  } break;
  case (JACK2_MODE_AUTODETECT): {
    palSetLineMode(LINE_AUX_UART2_TX, PAL_MODE_INPUT_PULLDOWN);
    palSetLineMode(LINE_AUX_UART2_RX, PAL_MODE_INPUT_ANALOG);
    aux_power_enable();
    config.jack2_mode = JACK2_MODE_AUTODETECT;
    led_rgb3_blink(0, 186, 0, TIME_MS2I(1000)); // green, autodetect
  } break;
  case (JACK2_MODE_MIDI): {
#ifdef USE_MIDI_SERIAL
    serial_midi_enable();
    config.jack2_mode = JACK2_MODE_MIDI;
    led_rgb3_blink(0, 90, 90, TIME_MS2I(1000)); // cyan, midi
#endif
  } break;
  case (JACK2_MODE_PEDAL_EXPRESSION): {
    aux_power_enable();
    palSetLineMode(LINE_AUX_UART2_RX, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(LINE_AUX_UART2_TX, PAL_MODE_INPUT_ANALOG);
    config.jack2_mode = JACK2_MODE_PEDAL_EXPRESSION;
    led_rgb3_blink(90, 90, 0, TIME_MS2I(1000)); // yellow, expression pedal
  } break;
  case (JACK2_MODE_PEDAL_SWITCH): {
    palSetLineMode(LINE_AUX_UART2_RX, PAL_MODE_INPUT_PULLUP);
    palSetLineMode(LINE_AUX_UART2_TX, PAL_MODE_INPUT_PULLUP);
    config.jack2_mode = JACK2_MODE_PEDAL_SWITCH;
    last_pedal2 = 1;
    last_pedal1 = 0;
    led_rgb3_blink(90, 0, 90, TIME_MS2I(1000)); // pink, switch pedal
  } break;
  case (JACK2_MODE_LINEIN): {
    codec_linein_enable();
    palSetLineMode(LINE_AUX_UART2_RX, PAL_MODE_INPUT_ANALOG); // or PULLDOWN if line out is through a capacitor?
    palSetLineMode(LINE_AUX_UART2_TX, PAL_MODE_INPUT_ANALOG);
    config.jack2_mode = JACK2_MODE_LINEIN;
    led_rgb3_blink(0, 0, 90, TIME_MS2I(1000)); // blue, line in
  } break;
  case (JACK2_MODE_ERROR): {
    palSetLineMode(LINE_AUX_UART2_RX, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(LINE_AUX_UART2_TX, PAL_MODE_INPUT);
    config.jack2_mode = JACK2_MODE_ERROR;
    led_rgb3_blink(90, 0, 0, TIME_MS2I(200)); // red, error
  } break;
  }
}

void aux_jack_switch_mode_setting(jack2_mode_t mode) {
  if (config.jack2_mode_setting == mode) return;

  config.jack2_mode_setting = mode;
  // disable jack to retrigger detection
  aux_jack_switch_mode(JACK2_MODE_DISABLED);
}
