/**
 * Copyright (C) 2013, 2014 Johannes Taelman, 2020 Piers Titus van der Torren
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
#include "ch.h"
#include "hal.h"
#include "ccportab.h"

#include "hal_sai_lld.h"
#include "codec_tlv320aic3x.h"
#include "codec.h"

// #define STM32_SAI_A_DMA_STREAM STM32_DMA_STREAM_ID(2, 1)
// #define STM32_SAI_B_DMA_STREAM STM32_DMA_STREAM_ID(2, 4)
#define STM32_SAI_A_DMA_STREAM STM32_DMA_STREAM_ID_ANY
#define STM32_SAI_B_DMA_STREAM STM32_DMA_STREAM_ID_ANY
#define SAI_A_DMA_CHANNEL 0
#define SAI_B_DMA_CHANNEL 1
#define STM32_SAI_A_DMA_PRIORITY 1
#define STM32_SAI_B_DMA_PRIORITY 1
#define STM32_SAI_A_IRQ_PRIORITY 2
#define STM32_SAI_B_IRQ_PRIORITY 2

const stm32_dma_stream_t* sai_a_dma;
const stm32_dma_stream_t* sai_b_dma;

//int codec_interrupt_timestamp;

int32_t buf[PLAYBACK_BUFFER_SIZE] __attribute__ ((section (".nocache")));
int32_t buf2[PLAYBACK_BUFFER_SIZE] __attribute__ ((section (".nocache")));
int32_t rbuf[PLAYBACK_BUFFER_SIZE] __attribute__ ((section (".nocache")));
int32_t rbuf2[PLAYBACK_BUFFER_SIZE] __attribute__ ((section (".nocache")));

void codec_clearbuffer(void) {
  int i;
  for (i = 0; i < PLAYBACK_BUFFER_SIZE; i++) {
    buf[i] = 0;
    buf2[i] = 0;
  }
}

void codec_ADAU1961_hw_reset(void) {
}

/* I2C interface  */
static const I2CConfig i2ccfg = {
  // 100kHz, 300,300 rise,fall:
  STM32_TIMINGR_PRESC(8U) |
  STM32_TIMINGR_SCLDEL(6U) | STM32_TIMINGR_SDADEL(3U) |
  STM32_TIMINGR_SCLH(43U)  | STM32_TIMINGR_SCLL(58U),
  0,
  0
};

CC_ALIGN(CACHE_LINE_SIZE) static uint8_t i2crxbuf[32];
CC_ALIGN(CACHE_LINE_SIZE) static uint8_t i2ctxbuf[32];

#define I2CD_CODEC      I2CD2


uint8_t aic3xReadRegister(uint8_t RegisterAddr) {
  msg_t status;
  i2ctxbuf[0] = RegisterAddr;
  cacheBufferFlush(&i2ctxbuf[0], sizeof i2ctxbuf);
  status = i2cMasterTransmit(&I2CD_CODEC, AIC3X_I2C_ADDR, i2ctxbuf, 1,
                             i2crxbuf, 1);
  cacheBufferInvalidate(&i2crxbuf[0], sizeof i2crxbuf);
  return (i2crxbuf[0]);
}

void aic3xWriteRegister(uint8_t RegisterAddr, uint8_t RegisterValue) {
  msg_t status;
  i2ctxbuf[0] = RegisterAddr;
  i2ctxbuf[1] = RegisterValue;

  cacheBufferFlush(&i2ctxbuf[0], sizeof i2ctxbuf);
  status = i2cMasterTransmit(&I2CD_CODEC, AIC3X_I2C_ADDR, i2ctxbuf, 2,
                             i2crxbuf, 0);

  static uint8_t rd;
  rd = aic3xReadRegister(RegisterAddr);
  if (rd != RegisterValue) {
//    setErrorFlag(ERROR_CODEC_I2C);
  } else {
  }
}

void codec_aic3x_init(uint16_t samplerate) {

  palSetLineMode(LINE_I2C2_SDA, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
  palSetLineMode(LINE_I2C2_SCL, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
  i2cStart(&I2CD_CODEC, &i2ccfg);

  palSetLine(LINE_CODEC_EN);
  chThdSleepMilliseconds(1);

  // Reset
  aic3xWriteRegister(AIC3X_RESET, 0x01);
  // Set Audio data word length = 32 bits
  aic3xWriteRegister(AIC3X_ASD_INTF_CTRLB, 0b00110000);
  // Route Line1LP to the Left ADC, Power up Left ADC
  //aic3xWriteRegister(LINE1L_2_LADC_CTRL, 0b00000100);
  // Route Line2L to the Left ADC
  // aic3xWriteRegister(17, 0b00001111);
  // Power up Left ADC
  // aic3xWriteRegister(LINE1L_2_LADC_CTRL, 0b01111100);
  // Route Line1RP to the Right ADC, Power up Right ADC
  // aic3xWriteRegister(LINE1R_2_RADC_CTRL, 0b00000100);
  // Route Line2R to the Right ADC
  // aic3xWriteRegister(18, 0b11110000);
  // Power up Right ADC
  // aic3xWriteRegister(LINE1R_2_RADC_CTRL, 0b01111100);
  // Unmute Left PGA, set gain to 0 dB
  // aic3xWriteRegister(LADC_VOL, 0x00);
  // Unmute Right PGA, set gain to 0dB
  // aic3xWriteRegister(RADC_VOL, 0x00);
  // Route Left data to Left DAC, Route Right data to Right DAC
  aic3xWriteRegister(AIC3X_CODEC_DATAPATH_REG, 0x0A); // 0b00001010
  // Power up Left and Right DAC’s
  aic3xWriteRegister(DAC_PWR, 0xC0); // 0b11000000
  // Unmute Left digital volume control, set gain to 0 dB
  aic3xWriteRegister(LDAC_VOL, 0x00);
  // Unmute Right digital volume control, set gain to 0 dB
  aic3xWriteRegister(RDAC_VOL, 0x00);
  // Route Left DAC output to Left line outs
  aic3xWriteRegister(DACL1_2_LLOPM_VOL, 0x80);
  // Route Left input to Left line outs
  // aic3xWriteRegister(PGAL_2_LLOPM_VOL, 0x80);
  // Route Right input to Right line outs
  // aic3xWriteRegister(PGAR_2_RLOPM_VOL, 0x80);
  // Route Right DAC output to Right Line outs
  aic3xWriteRegister(DACR1_2_RLOPM_VOL, 0x80);
  // Power up Left line out ± (differential), set gain to 0dB
  aic3xWriteRegister(LLOPM_CTRL, 0x09);
  // Power up Right line out ± (differential), set gain to 0 dB
  aic3xWriteRegister(RLOPM_CTRL, 0x09);

  // aic3xWriteRegister(46, 0b10000000);
  // aic3xWriteRegister(47, 0b10000000);
  // aic3xWriteRegister(51, 0b00001111);
  // aic3xWriteRegister(51, 0b00001111);

  palSetLine(LINE_HP_EN);
}

static void dma_sai_a_interrupt(void* dat, uint32_t flags) {
  (void)dat;
  (void)flags;

  //codec_interrupt_timestamp = hal_lld_get_counter_value();
  if ((sai_a_dma)->stream->CR & STM32_DMA_CR_CT) {
    computebufI(rbuf2, buf);
  }
  else {
    computebufI(rbuf, buf2);
  }
  dmaStreamClearInterrupt(sai_a_dma);
}

volatile SAI_Block_TypeDef *sai_a;
volatile SAI_Block_TypeDef *sai_b;
const stm32_dma_stream_t* sai_a_dma;
const stm32_dma_stream_t* sai_b_dma;

void codec_i2s_init(uint16_t sampleRate) {
  sai_a = SAI1_Block_A;
  sai_b = SAI1_Block_B;
//configure MCO // Axoloti uses MCO instead of MCLK, to let the codec do the PLL
  //palSetLineMode(LINE_SAI_MCLK, PAL_MODE_OUTPUT_PUSHPULL);
  //palSetLineMode(LINE_SAI_MCLK, PAL_MODE_ALTERNATE(0));
  //chThdSleepMilliseconds(10);
// release SAI
  palSetLineMode(LINE_SAI1_MCLK_A, PAL_MODE_INPUT);
  palSetLineMode(LINE_SAI1_FS_A,   PAL_MODE_INPUT);
  palSetLineMode(LINE_SAI1_SCK_A,  PAL_MODE_INPUT);
  palSetLineMode(LINE_SAI1_SD_B, PAL_MODE_INPUT);
  palSetLineMode(LINE_SAI1_SD_A, PAL_MODE_INPUT);
// configure SAI
  RCC->APB2ENR |= RCC_APB2ENR_SAI1EN;
  chThdSleepMilliseconds(1);
  SAI1_Block_A->CR2 = 0; //SAI_xCR2_FTH_1;
  SAI1_Block_B->CR2 = 0; //SAI_xCR2_FTH_0;
  SAI1_Block_A->FRCR = /*SAI_xFRCR_FSDEF |*/ SAI_xFRCR_FRL_(64)
      | SAI_xFRCR_FSALL_(32) | SAI_xFRCR_FSOFF;
  SAI1_Block_B->FRCR = /*SAI_xFRCR_FSDEF |*/ SAI_xFRCR_FRL_(64)
      | SAI_xFRCR_FSALL_(32) | SAI_xFRCR_FSOFF;
  SAI1_Block_A->SLOTR = (3 << 16) | SAI_xSLOTR_NBSLOT_0;
  SAI1_Block_B->SLOTR = (3 << 16) | SAI_xSLOTR_NBSLOT_0;
// SAI1_A is master transmitter
// SAI1_B is synchronous slave receiver
  SAI1_Block_A->CR1 = SAI_xCR1_DS_32
      | SAI_xCR1_MCKDIV_(14) // should be 4 for 48kHz, 2 for 44.1kHz // TODO: find out why it's 3.5 times faster than it should be, and the clock dividers doesn't seem to have influence.
      | SAI_xCR1_DMAEN | SAI_xCR1_CKSTR;
  SAI1_Block_B->CR1 = SAI_xCR1_DS_32
      | SAI_xCR1_SYNCEN_SYNCINT | SAI_xCR1_MODE_SLAVE_RX // synchronous slave receiver
      | SAI_xCR1_DMAEN | SAI_xCR1_CKSTR;
  chThdSleepMilliseconds(1);

  palSetLineMode(LINE_SAI1_MCLK_A, PAL_MODE_ALTERNATE(6));
  palSetLineMode(LINE_SAI1_FS_A,   PAL_MODE_ALTERNATE(6));
  palSetLineMode(LINE_SAI1_SCK_A,  PAL_MODE_ALTERNATE(6));
  palSetLineMode(LINE_SAI1_SD_A,   PAL_MODE_ALTERNATE(6));
  palSetLineMode(LINE_SAI1_SD_B,   PAL_MODE_ALTERNATE(6));

  uint32_t sai_a_dma_mode = STM32_DMA_CR_CHSEL(SAI_A_DMA_CHANNEL)
      | STM32_DMA_CR_PL(STM32_SAI_A_DMA_PRIORITY) | STM32_DMA_CR_DIR_M2P
      | STM32_DMA_CR_TEIE | STM32_DMA_CR_TCIE | STM32_DMA_CR_DBM | // double buffer mode
      STM32_DMA_CR_PSIZE_WORD | STM32_DMA_CR_MSIZE_WORD;
  uint32_t sai_b_dma_mode = STM32_DMA_CR_CHSEL(SAI_B_DMA_CHANNEL)
      | STM32_DMA_CR_PL(STM32_SAI_B_DMA_PRIORITY) | STM32_DMA_CR_DIR_P2M
      | STM32_DMA_CR_TEIE | STM32_DMA_CR_TCIE | STM32_DMA_CR_DBM | // double buffer mode
      STM32_DMA_CR_PSIZE_WORD | STM32_DMA_CR_MSIZE_WORD;

  sai_a_dma = dmaStreamAlloc(STM32_SAI_A_DMA_STREAM,
                             STM32_SAI_A_IRQ_PRIORITY,
                             (stm32_dmaisr_t)dma_sai_a_interrupt,
                             (void *)0);
  dmaSetRequestSource(sai_a_dma, STM32_DMAMUX1_SAI1_A);
  dmaStreamSetPeripheral(sai_a_dma, &(sai_a->DR));
  dmaStreamSetMemory0(sai_a_dma, buf);
  dmaStreamSetMemory1(sai_a_dma, buf2);
  dmaStreamSetTransactionSize(sai_a_dma, PLAYBACK_BUFFER_SIZE);
  dmaStreamSetMode(sai_a_dma, sai_a_dma_mode | STM32_DMA_CR_MINC);

  sai_b_dma = dmaStreamAlloc(STM32_SAI_B_DMA_STREAM,
                             STM32_SAI_B_IRQ_PRIORITY,
                             (stm32_dmaisr_t)0,
                             (void *)0);
  dmaSetRequestSource(sai_b_dma, STM32_DMAMUX1_SAI1_B);

  dmaStreamSetPeripheral(sai_b_dma, &(sai_b->DR));
  dmaStreamSetMemory0(sai_b_dma, rbuf);
  dmaStreamSetMemory1(sai_b_dma, rbuf2);
  dmaStreamSetTransactionSize(sai_b_dma, PLAYBACK_BUFFER_SIZE);
  dmaStreamSetMode(sai_b_dma, sai_b_dma_mode | STM32_DMA_CR_MINC);

  #if CODEC_ENABLE_INPUT
  dmaStreamClearInterrupt(sai_b_dma);
  #endif
  dmaStreamClearInterrupt(sai_a_dma);
  chSysLock();
  SAI1_Block_A->CR2 |= SAI_xCR2_FFLUSH;
  SAI1_Block_B->CR2 |= SAI_xCR2_FFLUSH;
  SAI1_Block_A->DR=0;
  SAI1_Block_A->DR=0;
  #if CODEC_ENABLE_INPUT
  dmaStreamEnable(sai_b_dma);
  #endif
  dmaStreamEnable(sai_a_dma);

  #if CODEC_ENABLE_INPUT
  SAI1_Block_B->CR1 |= SAI_xCR1_SAIEN;
  #endif
  SAI1_Block_A->CR1 |= SAI_xCR1_SAIEN;
  chSysUnlock();
}

void codec_init(uint16_t samplerate) {
  codec_i2s_init(samplerate);
  codec_aic3x_init(samplerate);
}

void codec_start(void) {
}

void codec_stop(void) {
}
