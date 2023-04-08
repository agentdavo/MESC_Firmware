#include "MESC_HAL_common.h"
#include "stm32f4xx_hal.h"

// Initialize PWM output on the STM32 MCU

void pwm_init(TIM_HandleTypeDef *htim, uint32_t channel) {
    // Configure PWM output
    TIM_OC_InitTypeDef sConfigOC; // Declare timer output compare initialization structure

    htim->Instance = TIMx;
    htim->Init.Prescaler = 0;
    htim->Init.Period = 0xFFFF;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    HAL_TIM_PWM_Init(htim); // Initialize timer for PWM output

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, channel); // Configure timer output compare for PWM output

    HAL_TIM_PWM_Start(htim, channel); // Start PWM output
}

// Set the duty cycle of the PWM output on the STM32 MCU

void pwm_set_duty(TIM_HandleTypeDef *htim, uint32_t channel, float duty_cycle) {
    // Set the duty cycle of the specified PWM output
    TIM_OC_InitTypeDef sConfigOC; // Declare timer output compare initialization structure
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (uint32_t)(duty_cycle * 0xFFFF);
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, channel); // Configure timer output compare for PWM output
    HAL_TIM_PWM_Start(htim, channel); // Start PWM output


// Start PWM output on specified channel on the STM32 MCU

void pwm_start(uint32_t channel) {
    // Start TIM PWM output on specified channel
    HAL_TIM_PWM_Start(TIMx, channel); // Start TIM PWM output on the specified channel
}

// Stop PWM output on specified channel on the STM32 MCU

void pwm_stop(uint32_t channel) {
    // Stop TIM PWM output on specified channel
    HAL_TIM_PWM_Stop(TIMx, channel); // Stop TIM PWM output on the specified channel
}