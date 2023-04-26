#include "hal_data.h"

#include "elc_hal.h"

#include "stdio.h"
#define APP_ERR_PRINT printf

fsp_err_t elc_hal_init(void)
{
    fsp_err_t err = FSP_SUCCESS;

    err = R_ELC_Open(&g_elc_ctrl, &g_elc_cfg);
    if (FSP_SUCCESS != err )
    {
        APP_ERR_PRINT("** R_ELC_Open API for ELC failed ** \r\n");
    }

    err = R_ELC_Enable(&g_elc_ctrl);
    if (FSP_SUCCESS != err )
    {
        APP_ERR_PRINT("** R_ELC_Enable API for ELC failed ** \r\n");
    }

    return err;
}

fsp_err_t elc_hal_deinit(void)
{
    fsp_err_t err = FSP_SUCCESS;

    err = R_ELC_Close(&g_elc_ctrl);
    assert(FSP_SUCCESS == err);
	
    return err;
}
