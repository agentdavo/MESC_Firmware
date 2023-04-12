#ifndef RZT2M_FSP_INIT_HAL_H_
#define RZT2M_FSP_INIT_HAL_H_

#include "hal_data.h"
#include "elc_hal.h"
#include "gpt_timers_hal.h"
#include "mtu3_hal.h"
#include "poe3_hal.h"
#include "adc_hal.h"
#include "dsmif_hal.h"
#include "uart_hal.h"
#include "canfd_hal.h"
#include "spi_hal.h"
#include "max17841.h"

fsp_err_t rzt2m_fsp_init(void);

#endif /* RZT2M_FSP_INIT_HAL_H_ */
