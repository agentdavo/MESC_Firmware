#ifndef MESC_HAL_TIMER_H
#define MESC_HAL_TIMER_H

#include "MESC_HAL_common.h"

// Timer channel configuration structure
typedef struct {
    MESC_HAL_GPIO_PortTypeDef *gpio_port;   // GPIO port
    uint32_t gpio_pin;                      // GPIO pin
    uint32_t gpio_af;                       // GPIO alternate function
} MESC_HAL_TIM_ChannelConfTypeDef;

// Timer configuration structure
typedef struct {
    MESC_HAL_TIM_TypeDef *tim;             // Timer instance
    uint32_t frequency;                    // Timer frequency
    MESC_HAL_TIM_ChannelConfTypeDef ch_conf[MESC_HAL_TIM_MAX_CHANNELS];    // Timer channel configurations
} MESC_HAL_TIM_InitTypeDef;

// Initialize timer
MESC_HAL_StatusTypeDef tim_init(MESC_HAL_TIM_InitTypeDef *init);

// Start timer
MESC_HAL_StatusTypeDef tim_start(MESC_HAL_TIM_HandleTypeDef *htim);

// Stop timer
MESC_HAL_StatusTypeDef tim_stop(MESC_HAL_TIM_HandleTypeDef *htim);

// Set timer channel duty cycle
MESC_HAL_StatusTypeDef tim_set_duty_cycle(MESC_HAL_TIM_HandleTypeDef *htim, uint32_t channel, uint32_t duty_cycle);

#endif /* MESC_HAL_TIMER_H */