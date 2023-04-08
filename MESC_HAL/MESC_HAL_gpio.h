#ifndef MESC_HAL_GPIO_H
#define MESC_HAL_GPIO_H

#include "MESC_HAL_common.h"

// GPIO pin mode enumeration
typedef enum {
    MESC_HAL_GPIO_MODE_INPUT,
    MESC_HAL_GPIO_MODE_OUTPUT
} MESC_HAL_GPIO_ModeTypeDef;

// GPIO pin state enumeration
typedef enum {
    MESC_HAL_GPIO_PIN_RESET = 0,
    MESC_HAL_GPIO_PIN_SET
} MESC_HAL_GPIO_PinState;

// GPIO pin configuration structure
typedef struct {
    MESC_HAL_GPIO_PortTypeDef *port;    // GPIO port
    uint16_t pin;                       // GPIO pin number
    MESC_HAL_GPIO_ModeTypeDef mode;     // GPIO pin mode
} MESC_HAL_GPIO_InitTypeDef;

// Initialize GPIO pin
void gpio_init(MESC_HAL_GPIO_InitTypeDef *init);

// Set GPIO pin state
void gpio_write_pin(MESC_HAL_GPIO_PortTypeDef *port, uint16_t pin, MESC_HAL_GPIO_PinState state);

// Read GPIO pin state
MESC_HAL_GPIO_PinState gpio_read_pin(MESC_HAL_GPIO_PortTypeDef *port, uint16_t pin);

// GPIO port definitions
extern MESC_HAL_GPIO_PortTypeDef MESC_HAL_GPIO_PortA;
extern MESC_HAL_GPIO_PortTypeDef MESC_HAL_GPIO_PortB;
extern MESC_HAL_GPIO_PortTypeDef MESC_HAL_GPIO_PortC;
extern MESC_HAL_GPIO_PortTypeDef MESC_HAL_GPIO_PortD;
extern MESC_HAL_GPIO_PortTypeDef MESC_HAL_GPIO_PortE;
extern MESC_HAL_GPIO_PortTypeDef MESC_HAL_GPIO_PortF;
extern MESC_HAL_GPIO_PortTypeDef MESC_HAL_GPIO_PortG;
extern MESC_HAL_GPIO_PortTypeDef MESC_HAL_GPIO_PortH;
extern MESC_HAL_GPIO_PortTypeDef MESC_HAL_GPIO_PortI;
extern MESC_HAL_GPIO_PortTypeDef MESC_HAL_GPIO_PortJ;
extern MESC_HAL_GPIO_PortTypeDef MESC_HAL_GPIO_PortK;

// GPIO pin macros
#define MESC_HAL_GPIO_PIN_0      ((uint16_t) 0x0001)
#define MESC_HAL_GPIO_PIN_1      ((uint16_t) 0x0002)
#define MESC_HAL_GPIO_PIN_2      ((uint16_t) 0x0004)
#define MESC_HAL_GPIO_PIN_3      ((uint16_t) 0x0008)
#define MESC_HAL_GPIO_PIN_4      ((uint16_t) 0x0010)
#define MESC_HAL_GPIO_PIN_5      ((uint16_t) 0x0020)
#define MESC_HAL_GPIO_PIN_6      ((uint16_t) 0x0040)
#define MESC_HAL_GPIO_PIN_7      ((uint16_t) 0x0080)
#define MESC_HAL_GPIO_PIN_8      ((uint16_t) 0x0100)
#define MESC_HAL_GPIO_PIN_9      ((uint16_t) 0x0200)
#define MESC_HAL_GPIO_PIN_10     ((uint16_t) 0x0400)
#define MESC_HAL_GPIO_PIN_11     ((uint16_t) 0x0800)
#define MESC_HAL_GPIO_PIN_12     ((uint16_t) 0x1000)
#define MESC_HAL_GPIO_PIN_13     ((uint16_t) 0x2000)
#define MESC_HAL_GPIO_PIN_14     ((uint16_t) 0x4000)
#define MESC_HAL_GPIO_PIN_15     ((uint16_t) 0x8000)

#endif /* MESC_HAL_GPIO_H */