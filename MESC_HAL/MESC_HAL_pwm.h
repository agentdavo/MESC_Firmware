#ifndef MESC_HAL_PWM_H
#define MESC_HAL_PWM_H

#include "MESC_HAL_common.h"

// PWM timer configuration structure
typedef struct {
    MESC_HAL_TIM_TypeDef *timer;            // PWM timer
    uint32_t channel;                       // PWM channel
    MESC_HAL_GPIO_PortTypeDef *gpio_port;   // GPIO port
    uint32_t gpio_pin;                      // GPIO pin
    uint32_t gpio_af;                       // GPIO alternate function
    uint32_t prescaler;                     // Timer prescaler
    uint32_t period;                        // Timer period
    uint32_t pulse;                         // Pulse width
    uint32_t polarity;                      // PWM polarity
    uint32_t mode;                          // PWM mode
} MESC_HAL_PWM_InitTypeDef;

// Initialize PWM timer
void pwm_init(MESC_HAL_PWM_InitTypeDef *init);

// Start PWM timer
void pwm_start(MESC_HAL_TIM_HandleTypeDef *htim);

// Stop PWM timer
void pwm_stop(MESC_HAL_TIM_HandleTypeDef *htim);

// Set PWM pulse width
void pwm_set_pulse_width(MESC_HAL_TIM_HandleTypeDef *htim, uint32_t channel, uint32_t pulse_width);

#endif /* MESC_HAL_PWM_H */