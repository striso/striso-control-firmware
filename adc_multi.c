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

#include "adc_multi.h"

/**
 * @brief   ADC DMA ISR service routine.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void adc_lld_serve_rx_interrupt(ADCDriver *adcp, uint32_t flags) {

  /* DMA errors handling.*/
  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0) {
    /* DMA, this could help only if the DMA tries to access an unmapped
       address space or violates alignment rules.*/
    _adc_isr_error_code(adcp, ADC_ERR_DMAFAILURE);
  }
  else {
    /* It is possible that the conversion group has already be reset by the
       ADC error handler, in this case this interrupt is spurious.*/
    if (adcp->grpp != NULL) {
      if ((flags & STM32_DMA_ISR_TCIF) != 0) {
        /* Transfer complete processing.*/
        _adc_isr_full_code(adcp);
      }
      else if ((flags & STM32_DMA_ISR_HTIF) != 0) {
        /* Half transfer processing.*/
        _adc_isr_half_code(adcp);
      }
    }
  }
}

/**
 * @brief   Configures and activates the ADC peripheral.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_multi_lld_start(void) {
  ADCDriver *adcp = &ADCD1;

  /* If in stopped state then enables the ADC and DMA clocks.*/
  if (adcp->state == ADC_STOP) {
#if STM32_ADC_USE_ADC1
    adcp->dmastp = dmaStreamAllocI(STM32_ADC_ADC1_DMA_STREAM,
                                    STM32_ADC_ADC1_DMA_IRQ_PRIORITY,
                                    (stm32_dmaisr_t)adc_lld_serve_rx_interrupt,
                                    (void *)adcp);
    osalDbgAssert(adcp->dmastp != NULL, "unable to allocate stream");
    dmaStreamSetPeripheral(adcp->dmastp, &ADC->CDR); // ADC->CDR instead of ADC1->DR
    rccEnableADC1(true);
#endif /* STM32_ADC_USE_ADC1 */

#if STM32_ADC_USE_ADC2
    rccEnableADC2(true);
#endif /* STM32_ADC_USE_ADC2 */

#if STM32_ADC_USE_ADC3
    rccEnableADC3(true);
#endif /* STM32_ADC_USE_ADC3 */

    /* This is a common register but apparently it requires that at least one
       of the ADCs is clocked in order to allow writing, see bug 3575297.*/
    ADC->CCR = (ADC->CCR & (ADC_CCR_TSVREFE | ADC_CCR_VBATE)) |
               (STM32_ADC_ADCPRE << 16);

    /* ADC initial setup, starting the analog part here in order to reduce
       the latency when starting a conversion.*/
#if STM32_ADC_USE_ADC1
    adcp = &ADCD1;
    adcp->adc->CR1 = 0;
    adcp->adc->CR2 = 0;
    adcp->adc->CR2 = ADC_CR2_ADON;
#endif /* STM32_ADC_USE_ADC1 */

#if STM32_ADC_USE_ADC2
    adcp = &ADCD2;
    adcp->adc->CR1 = 0;
    adcp->adc->CR2 = 0;
    adcp->adc->CR2 = ADC_CR2_ADON;
#endif /* STM32_ADC_USE_ADC2 */

#if STM32_ADC_USE_ADC3
    adcp = &ADCD3;
    adcp->adc->CR1 = 0;
    adcp->adc->CR2 = 0;
    adcp->adc->CR2 = ADC_CR2_ADON;
#endif /* STM32_ADC_USE_ADC3 */

  }
}

/**
 * @brief   Deactivates the ADC peripheral.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_multi_lld_stop(void) {
  ADCDriver *adcp = &ADCD1;

  /* If in ready state then disables the ADC clock.*/
  if (adcp->state == ADC_READY) {

    dmaStreamFreeI(adcp->dmastp);
    adcp->dmastp = NULL;

    adcp->adc->CR1 = 0;
    adcp->adc->CR2 = 0;

#if STM32_ADC_USE_ADC1
    if (&ADCD1 == adcp)
      rccDisableADC1();
#endif

#if STM32_ADC_USE_ADC2
    if (&ADCD2 == adcp)
      rccDisableADC2();
#endif

#if STM32_ADC_USE_ADC3
    if (&ADCD3 == adcp)
      rccDisableADC3();
#endif
  }
}

/**
 * @brief   Starts an ADC conversion.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_multi_lld_start_conversion(ADCDriver *adcp) {
  uint32_t mode;
  const ADCConversionGroup *grpp = adcp->grpp;

  /* DMA setup.*/
  mode = adcp->dmamode;
  if (grpp->circular) {
    mode |= STM32_DMA_CR_CIRC;
    if (adcp->depth > 1) {
      /* If circular buffer depth > 1, then the half transfer interrupt
         is enabled in order to allow streaming processing.*/
      mode |= STM32_DMA_CR_HTIE;
    }
  }
  dmaStreamSetMemory0(adcp->dmastp, adcp->samples);
  dmaStreamSetTransactionSize(adcp->dmastp, (uint32_t)grpp->num_channels *
                                            (uint32_t)adcp->depth);
  dmaStreamSetMode(adcp->dmastp, mode);
  dmaStreamEnable(adcp->dmastp);

  /* ADC setup.*/
  adcp->adc->SR    = 0;
  adcp->adc->SMPR1 = grpp->smpr1;
  adcp->adc->SMPR2 = grpp->smpr2;
  adcp->adc->HTR   = grpp->htr;
  adcp->adc->LTR   = grpp->ltr;
  adcp->adc->SQR1  = grpp->sqr1 | ADC_SQR1_NUM_CH(grpp->num_channels);
  adcp->adc->SQR2  = grpp->sqr2;
  adcp->adc->SQR3  = grpp->sqr3;

  /* ADC configuration and start, the start is performed using the method
     specified in the CR2 configuration, usually ADC_CR2_SWSTART.*/
  adcp->adc->CR1   = grpp->cr1 | ADC_CR1_OVRIE | ADC_CR1_SCAN;
  if ((grpp->cr2 & ADC_CR2_SWSTART) != 0) {
    adcp->adc->CR2 = (grpp->cr2 & ~ADC_CR2_SWSTART) | ADC_CR2_ADON | ADC_CR2_CONT;
    // Setting SWSTART in a separate write prevents spurious DMA request
    adcp->adc->CR2 |= ADC_CR2_SWSTART;
  }
  else {
    adcp->adc->CR2 = grpp->cr2 | ADC_CR2_ADON;
  }
}

/**
 * @brief   Configures and activates the ADC peripheral.
 *
 * @api
 */
void adcMultiStart(void) {
  ADCDriver *adcp = &ADCD1;

  osalSysLock();
  osalDbgAssert((adcp->state == ADC_STOP) || (adcp->state == ADC_READY),
                "invalid state");
  adc_multi_lld_start();
  adcp->state = ADC_READY;
  ADCD2.state = ADC_READY;
  ADCD3.state = ADC_READY;
  osalSysUnlock();
}

/**
 * @brief   Starts an ADC conversion.
 * @details Starts an asynchronous conversion operation.
 * @post    The callbacks associated to the conversion group will be invoked
 *          on buffer fill and error events.
 * @note    The buffer is organized as a matrix of M*N elements where M is the
 *          channels number configured into the conversion group and N is the
 *          buffer depth. The samples are sequentially written into the buffer
 *          with no gaps.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 * @param[in] grpp      pointer to a @p ADCConversionGroup object
 * @param[out] samples  pointer to the samples buffer
 * @param[in] depth     buffer depth (matrix rows number). The buffer depth
 *                      must be one or an even number.
 *
 * @iclass
 */
void adcMultiStartConversionI(ADCDriver *adcp,
                         const ADCConversionGroup *grpp,
                         adcsample_t *samples,
                         size_t depth) {

  osalDbgCheckClassI();
  osalDbgCheck((adcp != NULL) && (grpp != NULL) && (samples != NULL) &&
               (depth > 0U) && ((depth == 1U) || ((depth & 1U) == 0U)));
  osalDbgAssert((adcp->state == ADC_READY) ||
                (adcp->state == ADC_ERROR),
                "not ready");

  adcp->samples  = samples;
  adcp->depth    = depth;
  adcp->grpp     = grpp;
  adcp->state    = ADC_ACTIVE;
  adc_multi_lld_start_conversion(adcp);
}

/**
 * @brief   Starts an ADC conversion.
 * @details Starts an asynchronous conversion operation.
 * @note    The buffer is organized as a matrix of M*N elements where M is the
 *          channels number configured into the conversion group and N is the
 *          buffer depth. The samples are sequentially written into the buffer
 *          with no gaps.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 * @param[in] grpp      pointer to a @p ADCConversionGroup object
 * @param[out] samples  pointer to the samples buffer
 * @param[in] depth     buffer depth (matrix rows number). The buffer depth
 *                      must be one or an even number.
 *
 * @api
 */
void adcMultiStartConversion(
                        const ADCConversionGroup *grpp1,
                        const ADCConversionGroup *grpp2,
                        const ADCConversionGroup *grpp3,
                        adcsample_t *samples,
                        size_t depth) {

  osalSysLock();
  // Multi-DMA mode tuning
  ADC->CCR |= ADC_CCR_DMA_0 | ADC_CCR_DDS \
              | (ADC_CCR_MULTI_TRIPLE | ADC_CCR_MULTI_REGULAR_SIMUL);
  // DMA Mode 1
  // Triple mode: ADC1, 2 and 3 working together
  // 10110: Regular simultaneous mode only

  ADCD2.grpp = grpp2;
  ADCD2.state    = ADC_ACTIVE;
  /* ADC setup.*/
  ADCD2.adc->SR    = 0;
  ADCD2.adc->SMPR1 = grpp2->smpr1;
  ADCD2.adc->SMPR2 = grpp2->smpr2;
  ADCD2.adc->SQR1  = grpp2->sqr1;
  ADCD2.adc->SQR2  = grpp2->sqr2;
  ADCD2.adc->SQR3  = grpp2->sqr3;
  ADCD2.adc->CR1   = grpp2->cr1 | ADC_CR1_OVRIE | ADC_CR1_SCAN;
  ADCD2.adc->CR2   = grpp2->cr2 | ADC_CR2_CONT  | ADC_CR2_ADON;

  ADCD3.grpp = grpp3;
  ADCD3.state    = ADC_ACTIVE;
  /* ADC setup.*/
  ADCD3.adc->SR    = 0;
  ADCD3.adc->SMPR1 = grpp3->smpr1;
  ADCD3.adc->SMPR2 = grpp3->smpr2;
  ADCD3.adc->SQR1  = grpp3->sqr1;
  ADCD3.adc->SQR2  = grpp3->sqr2;
  ADCD3.adc->SQR3  = grpp3->sqr3;
  ADCD3.adc->CR1   = grpp3->cr1 | ADC_CR1_OVRIE | ADC_CR1_SCAN;
  ADCD3.adc->CR2   = grpp3->cr2 | ADC_CR2_CONT  | ADC_CR2_ADON;

  adcMultiStartConversionI(&ADCD1, grpp1, samples, depth);
  osalSysUnlock();
}
