
#ifndef _AUX_JACK_H_
#define _AUX_JACK_H_

#include "config.h"

void aux_jack_init(void);
void aux_jack_switch_mode(jack2_mode_t mode);

void detect_jack(void);

void aux_jack_mode_midi(void);
void aux_jack_mode_pedal(void);
void aux_jack_mode_audio_in(void);

#endif
