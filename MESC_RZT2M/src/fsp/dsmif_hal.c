#include "hal_data.h"

#include "dsmif_hal.h"

#include "stdio.h"
#define APP_ERR_PRINT printf

fsp_err_t dsmif_hal_init(void)
{
    fsp_err_t err = FSP_SUCCESS;

    /* DSMIF unit 0 & 1 initialize */

    err = R_DSMIF_Open(&g_dsmif0_ctrl, &g_dsmif0_cfg);
    if (FSP_SUCCESS != err )
    {
        APP_ERR_PRINT("** R_DSMIF_Open API for DSMIF0 failed ** \r\n");
    }

    err = R_DSMIF_Open(&g_dsmif1_ctrl, &g_dsmif1_cfg);
    if (FSP_SUCCESS != err )
    {
        APP_ERR_PRINT("** R_DSMIF_Open API for DSMIF1 failed ** \r\n");
    }

    return err;
}

fsp_err_t dsmif_hal_deinit(void)
{
    fsp_err_t err = FSP_SUCCESS;

    /* DSMIF unit 0 & 1 deinitialize */

    err = R_DSMIF_Close(&g_dsmif0_ctrl);
    assert(FSP_SUCCESS == err);

    err = R_DSMIF_Close(&g_dsmif1_ctrl);
    assert(FSP_SUCCESS == err);

    return err;
}
