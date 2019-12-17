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
#ifndef SYNTH_H_
#define SYNTH_H_

#include "ch.h"
#include "hal.h"
#include "config.h"
#include "midi.h"

#define SAMPLINGFREQ 44100
#define CHANNEL_BUFFER_SIZE		32
#define PLAYBACK_BUFFER_SIZE	(CHANNEL_BUFFER_SIZE*2)

typedef struct struct_synth_interface {
	float* acc_abs;
	float* acc_x;
	float* acc_y;
	float* acc_z;
	float* rot_x;
	float* rot_y;
	float* rot_z;
	float* note[MAX_VOICECOUNT];
	float* pres[MAX_VOICECOUNT];
	float* vpres[MAX_VOICECOUNT];
	float* but_x[MAX_VOICECOUNT];
	float* but_y[MAX_VOICECOUNT];
} synth_interface_t;

#ifdef USE_SYNTH_INTERFACE
extern synth_interface_t synth_interface;
extern synth_interface_t synth_interface_bas;

void start_synth_thread(void);
#endif

int synth_message(int size, int* msg);
void synth_tick(void);
void midi_config(void);

void clear_dead_notes(void);

void MidiInMsgHandler(midi_device_t dev, uint8_t port, uint8_t b0, uint8_t b1, uint8_t b2);

extern float volume;

#endif /* SYNTH_H_ */
