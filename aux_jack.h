
#ifndef _AUX_JACK_H_
#define _AUX_JACK_H_

#include "config.h"

void aux_jack_init(void);
void aux_jack_switch_mode(jack2_mode_t mode);
void aux_jack_switch_mode_setting(jack2_mode_t mode);

void aux_power_enable(void);
void aux_power_disable(void);

#endif
