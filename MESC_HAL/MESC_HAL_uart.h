#ifndef MESC_HAL_UART_H
#define MESC_HAL_UART_H

#include "MESC_HAL_common.h"

// UART configuration structure
typedef struct {
    MESC_HAL_UART_TypeDef *uart;        // UART instance
    uint32_t baud_rate;                 // UART baud rate
    uint32_t word_length;               // UART word length
    uint32_t stop_bits;                 // UART stop bits
    uint32_t parity;                    // UART parity
    uint32_t mode;                      // UART mode
} MESC_HAL_UART_InitTypeDef;

// UART handle structure
typedef struct {
    MESC_HAL_UART_TypeDef *uart;        // UART instance
    uint8_t *tx_buffer;                 // Transmit buffer
    uint16_t tx_size;                   // Transmit buffer size
    uint16_t tx_index;                  // Transmit buffer index
    uint8_t *rx_buffer;                 // Receive buffer
    uint16_t rx_size;                   // Receive buffer size
    uint16_t rx_index;                  // Receive buffer index
} MESC_HAL_UART_HandleTypeDef;

// Initialize UART
MESC_HAL_StatusTypeDef uart_init(MESC_HAL_UART_InitTypeDef *init);

// Transmit data over UART
MESC_HAL_StatusTypeDef uart_transmit(MESC_HAL_UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);

// Receive data over UART
MESC_HAL_StatusTypeDef uart_receive(MESC_HAL_UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);

// Check if UART transmit is complete
bool uart_is_transmit_complete(MESC_HAL_UART_HandleTypeDef *huart);

// Check if UART receive is complete
bool uart_is_receive_complete(MESC_HAL_UART_HandleTypeDef *huart);

#endif /* MESC_HAL_UART_H */
