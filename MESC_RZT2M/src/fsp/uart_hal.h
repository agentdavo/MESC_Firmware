#ifndef UART_HAL_H_
#define UART_HAL_H_

#include "comms_thread.h"

fsp_err_t uart_hal_init(void);
fsp_err_t uart_hal_deinit(void);

void g_uart0_callback (uart_callback_args_t * p_args);

#endif /* UART_HAL_H_ */
