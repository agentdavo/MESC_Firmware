#include "MESC_HAL_common.h"
#include "stm32f4xx_hal.h"

void adc_init(ADC_HandleTypeDef *hadc, uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig;

    // Configure ADC channel
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    HAL_ADC_ConfigChannel(hadc, &sConfig);
}

uint16_t adc_read(ADC_HandleTypeDef *hadc)
{
    uint16_t adc_value;

    // Start ADC conversion
    HAL_ADC_Start(hadc);

    // Poll for ADC conversion to complete
    HAL_ADC_PollForConversion(hadc, 100);

    // Read ADC value from data register
    adc_value = HAL_ADC_GetValue(hadc);

    // Stop ADC conversion
    HAL_ADC_Stop(hadc);

    return adc_value;
}