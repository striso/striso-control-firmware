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

#include "hal.h"

#define ADC_CCR_MULTI_INDEPENDENT   (0x00 << 0)
#define ADC_CCR_MULTI_REG_SIMUL_AND_INJECTED_SIMUL   (0x01 << 0)
#define ADC_CCR_MULTI_REG_SIMUL_AND_ALTERNATE_TRIG   (0x02 << 0)
#define ADC_CCR_MULTI_INJECTED_SIMUL   (0x05 << 0)
#define ADC_CCR_MULTI_REGULAR_SIMUL   (0x06 << 0)
#define ADC_CCR_MULTI_INTERLEAVED   (0x07 << 0)
#define ADC_CCR_MULTI_ALTERNATE_TRIG   (0x09 << 0)
#define ADC_CCR_MULTI_DUAL   (0x00 << 0)
#define ADC_CCR_MULTI_TRIPLE   (0x10 << 0)

void adcMultiStart(void);

void adcMultiStartConversion(
                        const ADCConversionGroup *grpp1,
                        const ADCConversionGroup *grpp2,
                        const ADCConversionGroup *grpp3,
                        adcsample_t *samples,
                        size_t depth);
