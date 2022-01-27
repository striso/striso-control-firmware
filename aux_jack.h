
#ifndef _AUX_JACK_H_
#define _AUX_JACK_H_

void aux_jack_init(void);

void detect_jack(void);

void aux_jack_mode_midi(void);
void aux_jack_mode_pedal(void);
void aux_jack_mode_audio_in(void);

#endif
