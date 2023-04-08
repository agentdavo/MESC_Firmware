#ifndef MESC_HAL_ADC_H
#define MESC_HAL_ADC_H

#include "MESC_HAL_common.h"

// ADC channel configuration structure
typedef struct {
    MESC_HAL_ADC_TypeDef *adc;              // ADC instance
    uint32_t channel;                       // ADC channel
    MESC_HAL_GPIO_PortTypeDef *gpio_port;   // GPIO port
    uint32_t gpio_pin;                      // GPIO pin
} MESC_HAL_ADC_ChannelConfTypeDef;

// ADC configuration structure
typedef struct {
    MESC_HAL_ADC_TypeDef *adc;              // ADC instance
    uint32_t resolution;                    // ADC resolution
    uint32_t sampling_time;                 // ADC sampling time
} MESC_HAL_ADC_InitTypeDef;

// Initialize ADC
MESC_HAL_StatusTypeDef adc_init(MESC_HAL_ADC_InitTypeDef *init);

// Start ADC conversion
MESC_HAL_StatusTypeDef adc_start(MESC_HAL_ADC_HandleTypeDef *hadc);

// Poll for ADC conversion to complete
MESC_HAL_StatusTypeDef adc_poll_for_conversion(MESC_HAL_ADC_HandleTypeDef *hadc, uint32_t timeout);

// Stop ADC conversion
MESC_HAL_StatusTypeDef adc_stop(MESC_HAL_ADC_HandleTypeDef *hadc);

// Get ADC conversion result
uint32_t adc_get_value(MESC_HAL_ADC_HandleTypeDef *hadc);

#endif /* MESC_HAL_ADC_H */
