#ifndef CANFD_HAL_H_
#define CANFD_HAL_H_

#include "comms_thread.h"

fsp_err_t canfd_hal_init(void);
fsp_err_t canfd_hal_deinit(void);

void canfd0_callback(can_callback_args_t *p_args);
void canfd1_callback(can_callback_args_t *p_args);

#endif /* CANFD_HAL_H_ */
