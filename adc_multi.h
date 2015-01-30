#define 	ADC_CCR_MULTI_INDEPENDENT   (0x00 << 0)
#define 	ADC_CCR_MULTI_REG_SIMUL_AND_INJECTED_SIMUL   (0x01 << 0)
#define 	ADC_CCR_MULTI_REG_SIMUL_AND_ALTERNATE_TRIG   (0x02 << 0)
#define 	ADC_CCR_MULTI_INJECTED_SIMUL   (0x05 << 0)
#define 	ADC_CCR_MULTI_REGULAR_SIMUL   (0x06 << 0)
#define 	ADC_CCR_MULTI_INTERLEAVED   (0x07 << 0)
#define 	ADC_CCR_MULTI_ALTERNATE_TRIG   (0x09 << 0)
#define 	ADC_CCR_MULTI_DUAL   (0x00 << 0)
#define 	ADC_CCR_MULTI_TRIPLE   (0x10 << 0)

extern void adcMultiStart(void);

extern void adcMultiStartConversion(
                        const ADCConversionGroup *grpp1,
                        const ADCConversionGroup *grpp2,
                        const ADCConversionGroup *grpp3,
                        adcsample_t *samples,
                        size_t depth);
