#include "MESC_HAL_common.h"
#include "stm32l4xx_hal.h"

// Initialize GPIO pins on the STM32 MCU

void gpio_init(void) {
    // Configure GPIO pins
    GPIO_InitTypeDef GPIO_InitStruct; // Declare GPIO initialization structure

    // Initialize motor enable pin
    __HAL_RCC_GPIOx_CLK_ENABLE(); // Enable GPIO port clock
    GPIO_InitStruct.Pin = MOTOR_ENABLE_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR_ENABLE_GPIO_PORT, &GPIO_InitStruct); // Initialize motor enable pin

    // Initialize motor direction pin
    GPIO_InitStruct.Pin = MOTOR_DIRECTION_GPIO_PIN;
    HAL_GPIO_Init(MOTOR_DIRECTION_GPIO_PORT, &GPIO_InitStruct); // Initialize motor direction pin

    // Initialize brake pin
    GPIO_InitStruct.Pin = BRAKE_GPIO_PIN;
    HAL_GPIO_Init(BRAKE_GPIO_PORT, &GPIO_InitStruct); // Initialize brake pin
}

// Write a state (HIGH or LOW) to a GPIO pin on the STM32 MCU

void gpio_write(uint32_t pin, uint8_t state) {
    // Write a state to the specified GPIO pin
    HAL_GPIO_WritePin(GPIOx, pin, (GPIO_PinState)state); // Write a state to the specified GPIO pin
}

// Read the state (HIGH or LOW) of a GPIO pin on the STM32 MCU

uint8_t gpio_read(uint32_t pin) {
    // Read the state of the specified GPIO pin
    return (uint8_t)HAL_GPIO_ReadPin(GPIOx, pin); // Read the state of the specified GPIO pin
}
