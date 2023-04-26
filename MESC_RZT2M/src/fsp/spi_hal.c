#include "hal_data.h"
#include "comms_thread.h"

#include "spi_hal.h"

#include "stdio.h"
#define APP_ERR_PRINT printf

static volatile spi_event_t g_spi0_event_flag;
static volatile spi_event_t g_spi2_event_flag;
static volatile spi_event_t g_spi3_event_flag;

fsp_err_t spi_hal_init(void)
{
    fsp_err_t err = FSP_SUCCESS;
	
	/* Reset SPI event flags */
    g_spi0_event_flag = 0;
	g_spi2_event_flag = 0;
    g_spi3_event_flag = 0;
	
	err = R_SPI_Open(&g_spi0_ctrl, &g_spi0_cfg);
    if (FSP_SUCCESS != err )
    {
        APP_ERR_PRINT("** R_SPI_Open API for SPI0 failed ** \r\n");
    }
	
//    err = R_SPI_Open(&g_spi2_ctrl, &g_spi2_cfg);
//    if (FSP_SUCCESS != err )
//    {
//        APP_ERR_PRINT("** R_SPI_Open API for SPI2 failed ** \r\n");
//    }
	
    err = R_SPI_Open(&g_spi3_ctrl, &g_spi3_cfg);
    if (FSP_SUCCESS != err )
    {
        APP_ERR_PRINT("** R_SPI_Open API for SPI3 failed ** \r\n");
    }
	
    return err;
}


void spi_hal_deinit(void)
{
	fsp_err_t err = FSP_SUCCESS;
	
    err = R_SPI_Close(&g_spi0_ctrl);
	if (FSP_SUCCESS != err )
    {
        APP_ERR_PRINT("** R_SPI_Close API for SPI0  failed **  \r\n");
    }

//	err = R_SPI_Close(&g_spi2_ctrl);
//	if (FSP_SUCCESS != err )
//    {
//        APP_ERR_PRINT("** R_SPI_Close API for SPI2 failed **  \r\n");
//    }
	
	err = R_SPI_Close(&g_spi3_ctrl);
	if (FSP_SUCCESS != err )
    {
        APP_ERR_PRINT("** R_SPI_Close API for SPI3 failed **  \r\n");
    }
	
}


void g_spi0_callback(spi_callback_args_t * p_args)
{
	if (SPI_EVENT_TRANSFER_COMPLETE == p_args->event)
	{
		g_spi0_event_flag = SPI_EVENT_TRANSFER_COMPLETE;
	}
	else
	{
		g_spi0_event_flag = SPI_EVENT_TRANSFER_ABORTED;
	}
}

void g_spi2_callback(spi_callback_args_t * p_args)
{
	if (SPI_EVENT_TRANSFER_COMPLETE == p_args->event)
	{
		g_spi2_event_flag = SPI_EVENT_TRANSFER_COMPLETE;
	}
	else
	{
		g_spi2_event_flag = SPI_EVENT_TRANSFER_ABORTED;
	}
}

void g_spi3_callback(spi_callback_args_t * p_args)
{
	if (SPI_EVENT_TRANSFER_COMPLETE == p_args->event)
	{
		g_spi3_event_flag = SPI_EVENT_TRANSFER_COMPLETE;
	}
	else
	{
		g_spi3_event_flag = SPI_EVENT_TRANSFER_ABORTED;
	}
}
