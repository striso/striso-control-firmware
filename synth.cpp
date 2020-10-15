/* ------------------------------------------------------------
name: "synth.dsp.tmp"
Code generated with Faust 2.27.2 (https://faust.grame.fr)
Compilation options: -lang cpp -scal -ftz 0
------------------------------------------------------------ */

extern "C" {
    #include "ch.h"
    #include "hal.h"
    #include "synth.h"
}

// variables
synth_interface_t synth_interface;

void computebufI(int32_t *inp, int32_t *outp) {
  int i;
    for (i = 0; i < PLAYBACK_BUFFER_SIZE; i++) {
      outp[i] = (i - PLAYBACK_BUFFER_SIZE / 2) * 1<<26;
      // square wave 34952 * (1<<8) * 2 * ((i>PLAYBACK_BUFFER_SIZE/2)-0.5);
      // saw ware (i - PLAYBACK_BUFFER_SIZE / 2) * 100000; // testing noise 0;
    }
}
