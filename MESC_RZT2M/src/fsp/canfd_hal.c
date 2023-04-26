#include "hal_data.h"

#include "canfd_hal.h"

#include "stdio.h"
#define APP_ERR_PRINT printf

bool g_canfd0_write_complete = false;
bool g_canfd1_write_complete = false;

bool g_canfd0_error = false;
bool g_canfd1_error = false;

fsp_err_t canfd_hal_init(void)
{
    fsp_err_t err = FSP_SUCCESS;
	
	/* Initializes CANFD 0 */
    err = R_CANFD_Open(&g_canfd0_ctrl, &g_canfd0_cfg);
    if (FSP_SUCCESS != err )
    {
        APP_ERR_PRINT("** R_CANFD_Open API for CANFD0 failed ** \r\n");
    }
	
	/* Initializes CANFD 1 */
    err = R_CANFD_Open(&g_canfd1_ctrl, &g_canfd1_cfg);
    if (FSP_SUCCESS != err )
    {
        APP_ERR_PRINT("** R_CANFD_Open API for CANFD1 failed ** \r\n");
    }
    return err;
}


fsp_err_t canfd_hal_deinit(void)
{
    fsp_err_t err = FSP_SUCCESS;
	
    err = R_CANFD_Close(&g_canfd0_ctrl);
    assert(FSP_SUCCESS == err);
	
    err = R_CANFD_Close(&g_canfd1_ctrl);
    assert(FSP_SUCCESS == err);

    return err;
}

void g_canfd0_callback(can_callback_args_t *p_args)
{
    switch (p_args->event)
    {
        case CAN_EVENT_TX_COMPLETE:
        case CAN_EVENT_TX_FIFO_EMPTY:
        {
            g_canfd0_write_complete = true;
            break;
        }
        case CAN_EVENT_RX_COMPLETE:
        case CAN_EVENT_ERR_WARNING:             // error warning event
        case CAN_EVENT_ERR_PASSIVE:             // error passive event
        case CAN_EVENT_ERR_BUS_OFF:             // error Bus Off event
        case CAN_EVENT_BUS_RECOVERY:            // bus recovery error event
        case CAN_EVENT_MAILBOX_MESSAGE_LOST:    // overwrite/overrun error event
        case CAN_EVENT_ERR_BUS_LOCK:            // bus lock detected (32 consecutive dominant bits).
        case CAN_EVENT_ERR_CHANNEL:             // channel error has occurred.
        case CAN_EVENT_TX_ABORTED:              // transmit abort event.
        case CAN_EVENT_ERR_GLOBAL:              // Global error has occurred.
        default:
        {
            g_canfd0_error = true;
            break;
        }
    }
}

void g_canfd1_callback(can_callback_args_t *p_args)
{
    switch (p_args->event)
    {
        case CAN_EVENT_TX_COMPLETE:
        case CAN_EVENT_TX_FIFO_EMPTY:
        {
            g_canfd0_write_complete = true;
            break;
        }
        case CAN_EVENT_RX_COMPLETE:
        case CAN_EVENT_ERR_WARNING:             // error warning event
        case CAN_EVENT_ERR_PASSIVE:             // error passive event
        case CAN_EVENT_ERR_BUS_OFF:             // error Bus Off event
        case CAN_EVENT_BUS_RECOVERY:            // bus recovery error event
        case CAN_EVENT_MAILBOX_MESSAGE_LOST:    // overwrite/overrun error event
        case CAN_EVENT_ERR_BUS_LOCK:            // bus lock detected (32 consecutive dominant bits).
        case CAN_EVENT_ERR_CHANNEL:             // channel error has occurred.
        case CAN_EVENT_TX_ABORTED:              // transmit abort event.
        case CAN_EVENT_ERR_GLOBAL:              // global error has occurred.
        default:
        {
            g_canfd1_error = true;
            break;
        }
    }
}
