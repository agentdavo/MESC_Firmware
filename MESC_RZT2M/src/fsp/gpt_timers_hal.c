#include "gpt_timers_hal.h"

#include "hal_data.h"

#include "stdio.h"
#define APP_ERR_PRINT printf

fsp_err_t gpt_timers_hal_init(void)
{
    fsp_err_t err = FSP_SUCCESS;

    /* MESC hyperLoop timer */
	err = R_GPT_Open (&g_gpt0_hyperLoop_ctrl, &g_gpt0_hyperLoop_cfg);
    if (FSP_SUCCESS != err )
    {
        APP_ERR_PRINT("** R_GPT_Open API for GPT0_HYPERLOOP failed ** \r\n");
    }
    err = R_GPT_Start (&g_gpt0_hyperLoop_ctrl);
    if (FSP_SUCCESS != err )
    {
        APP_ERR_PRINT("** R_GPT_Start API for GPT0_HYPERLOOP failed ** \r\n");
    }
	
    /* MESC fastLoop timer */
    err = R_GPT_Open (&g_gpt1_fastLoop_ctrl, &g_gpt1_fastLoop_cfg);
    if (FSP_SUCCESS != err )
    {
        APP_ERR_PRINT("** R_GPT_Open API for GPT1_FASTLOOP failed ** \r\n");
    }
    err = R_GPT_Start (&g_gpt1_fastLoop_ctrl);
    if (FSP_SUCCESS != err )
    {
        APP_ERR_PRINT("** R_GPT_Start API for GPT1_FASTLOOP failed ** \r\n");
    }
	
    /* MESC slowLoop timer */
    err = R_GPT_Open (&g_gpt2_slowLoop_ctrl, &g_gpt2_slowLoop_cfg);
    if (FSP_SUCCESS != err )
    {
        APP_ERR_PRINT("** R_GPT_Open API for GPT2_SLOWLOOP failed ** \r\n");
    }
    err = R_GPT_Start (&g_gpt2_slowLoop_ctrl);
    if (FSP_SUCCESS != err )
    {
        APP_ERR_PRINT("** R_GPT_Start API for GPT2_SLOWLOOP failed ** \r\n");
    }

    return err;
}

fsp_err_t gpt_timers_hal_deinit(void)
{
    fsp_err_t err = FSP_SUCCESS;
	
	err = R_GPT_Close(&g_gpt0_hyperLoop_ctrl);
	assert(FSP_SUCCESS == err);

    err = R_GPT_Close(&g_gpt1_fastLoop_ctrl);
	assert(FSP_SUCCESS == err);
	
	err = R_GPT_Close(&g_gpt2_slowLoop_ctrl);
	assert(FSP_SUCCESS == err);
	
	return err;
}
