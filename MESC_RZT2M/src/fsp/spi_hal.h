#ifndef SPI_HAL_H_
#define SPI_HAL_H_

#include "comms_thread.h"

fsp_err_t spi_hal_init(void);
void spi_hal_deinit(void);

void g_spi0_callback(spi_callback_args_t * p_args);
void g_spi2_callback(spi_callback_args_t * p_args);
void g_spi3_callback(spi_callback_args_t * p_args);

#endif /* SPI_HAL_H_ */
