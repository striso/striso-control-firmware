/**
 * Copyright (C) 2013, 2014, 2015 Johannes Taelman
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
#ifndef __MIDI_SERIAL_H
#define __MIDI_SERIAL_H

#include <stdint.h>

#define SDMIDI SD2

#ifdef __cplusplus
extern "C" {
#endif
void serial_midi_init(void);
void serial_MidiSend1(uint8_t b0);
void serial_MidiSend2(uint8_t b0, uint8_t b1);
void serial_MidiSend3(uint8_t b0, uint8_t b1, uint8_t b2);

int  serial_MidiGetOutputBufferPending(void);

#ifdef __cplusplus
}
#endif

#endif
