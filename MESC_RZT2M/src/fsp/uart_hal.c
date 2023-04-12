#include "hal_data.h"

#include "uart_hal.h"

fsp_err_t uart_hal_init(void)
{
    fsp_err_t err = FSP_SUCCESS;
	
	/* Initializes UART0 */
    err = R_SCI_UART_Open(&g_uart0_ctrl, &g_uart0_cfg);
    if (FSP_SUCCESS != err )
    {
        APP_ERR_PRINT("** R_SCI_UART_Open API for UART0 failed ** \r\n");
    }

    return err;
}


fsp_err_t uart_hal_deinit(void)
{
    fsp_err_t err = FSP_SUCCESS;
	
    err = R_SCI_UART_Close(&g_uart0_ctrl);
    assert(FSP_SUCCESS == err);

    return err;
}

void g_uart0_callback (uart_callback_args_t * p_args)
{
	if (p_args->channel == 0)
    {
		/* Handle the UART event */
		switch (p_args->event)
		{
			/* Receive complete */
			case UART_EVENT_RX_COMPLETE:
			{
				break;
			}
			
			/* Transmit complete */
			case UART_EVENT_TX_COMPLETE:
			{
				break;
			}
			
			/* Received a character */
			case UART_EVENT_RX_CHAR:
			{
				break;
			}
			
			/* Last byte trasmitting */			
			case UART_EVENT_TX_DATA_EMPTY:
			{
				break;
			}
			
			default:
			{
			}
		}
    }
}
