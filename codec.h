/**
 * Copyright (C) 2013, 2014 Johannes Taelman
 *
 * This file is part of Axoloti.
 *
 * Axoloti is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * Axoloti is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * Axoloti. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __CODEC_H
#define __CODEC_H
#include <stdint.h>

#include "config.h"

// double buffers for DMA, interleaved stereo
extern int32_t buf[PLAYBACK_BUFFER_SIZE];
extern int32_t buf2[PLAYBACK_BUFFER_SIZE];
extern int32_t rbuf[PLAYBACK_BUFFER_SIZE];
extern int32_t rbuf2[PLAYBACK_BUFFER_SIZE];

extern void codec_init(uint16_t samplerate);
extern void codec_stop(void);

extern void computebufI(int32_t *inp, int32_t *outp);

void codec_clearbuffer(void);

void codec_linein_enable(void);
void codec_linein_disable(void);

#endif /* __CODEC_H */
