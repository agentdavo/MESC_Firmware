#ifndef MESC_HAL_COMMON_H
#define MESC_HAL_COMMON_H

#include <stdint.h>
#include <stddef.h>

// Define GPIO pin state type
typedef enum {
    MESC_HAL_GPIO_PIN_RESET = 0,
    MESC_HAL_GPIO_PIN_SET
} MESC_HAL_GPIO_PinState;

// Define GPIO port type
typedef struct {
    uint32_t id;
} MESC_HAL_GPIO_PortTypeDef;

// GPIO port structure
typedef struct {
    uint32_t port_number;
} MESC_HAL_GPIO_PortTypeDef;

// ADC structure
typedef struct {
    uint32_t instance;
} MESC_HAL_ADC_TypeDef;

// UART structure
typedef struct {
    uint32_t instance;
} MESC_HAL_UART_TypeDef;

// TIM structure
typedef struct {
    uint32_t instance;
} MESC_HAL_TIM_TypeDef;

// Error codes
typedef enum {
    MESC_HAL_OK = 0,
    MESC_HAL_ERROR,
    MESC_HAL_BUSY,
    MESC_HAL_TIMEOUT
} MESC_HAL_StatusTypeDef;

#endif /* MESC_HAL_COMMON_H */