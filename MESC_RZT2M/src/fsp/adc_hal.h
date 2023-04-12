#ifndef ADC_HAL_H_
#define ADC_HAL_H_

fsp_err_t adc_hal_init(void);
void adc_hal_deinit(void);

void g_adc0_callback(adc_callback_args_t * p_args);
void g_adc1_callback(adc_callback_args_t * p_args);

/*Defines*/
#define ADC_CHANNEL0	(1U << 0U)
#define ADC_CHANNEL1	(1U << 1U)
#define ADC_CHANNEL2	(1U << 2U)
#define ADC_CHANNEL3	(1U << 3U)
#define ADC_CHANNEL4	(1U << 4U)
#define ADC_CHANNEL5	(1U << 5U)
#define ADC_CHANNEL6	(1U << 6U)
#define ADC_CHANNEL7	(1U << 7U)
#define ADC_CHANNEL8	(1U << 8U)
#define ADC_CHANNEL9	(1U << 9U)
#define ADC_CHANNEL10	(1U << 10U)
#define ADC_CHANNEL11	(1U << 11U)

#define ADC_RESOLUTION_12_BIT  0        ///< 12 bit resolution
#define ADC_RESOLUTION_10_BIT  1        ///< 10 bit resolution
#define ADC_RESOLUTION_8_BIT   2        ///< 8 bit resolution
#define ADC_RESOLUTION_14_BIT  3        ///< 14 bit resolution
#define ADC_RESOLUTION_16_BIT  4        ///< 16 bit resolution
#define ADC_RESOLUTION_24_BIT  5        ///< 24 bit resolution

#define ADC_UINT0		0
#define ADC_UINT1		1
/*Defines*/

/*Configuration*/

/*Configuration*/
#endif /* ADC_HAL_H_ */
