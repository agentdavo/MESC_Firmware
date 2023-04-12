#include "hal_data.h"

#include "mtu3_hal.h"

fsp_err_t mtu3_hal_init(void)
{
    fsp_err_t err = FSP_SUCCESS;
	
	/* Initializes MTU3_34 PWM */
    err = R_MTU3_THREE_PHASE_Open(&g_mtu3_m0_3ph_drv_ctrl, &g_mtu3_m0_3ph_drv_cfg);
    if (FSP_SUCCESS != err )
    {
        APP_ERR_PRINT("** R_MTU3_THREE_PHASE_Open API for mtu3_m0 failed ** \r\n");
    }
	
	/* Initializes MTU3_67 PWM */
    err = R_MTU3_THREE_PHASE_Open(&g_mtu3_m1_3ph_drv_ctrl, &g_mtu3_m1_3ph_drv_cfg);
    if (FSP_SUCCESS != err )
    {
        APP_ERR_PRINT("** R_MTU3_THREE_PHASE_Open API for mtu3_m1 failed ** \r\n");
    }

    /* Start the MTU3_34 PWM timer */
	err = R_MTU3_THREE_PHASE_Start(&g_mtu3_m0_3ph_drv_ctrl);
    if (FSP_SUCCESS != err )
    {
        APP_ERR_PRINT("** R_MTU3_THREE_PHASE_Start API for mtu3_m0 failed ** \r\n");
    }
	
    /* Start the MTU3_67 PWM timer */
	err = R_MTU3_THREE_PHASE_Start(&g_mtu3_m1_3ph_drv_ctrl);
    if (FSP_SUCCESS != err )
    {
        APP_ERR_PRINT("** R_MTU3_THREE_PHASE_Start API for mtu3_m1 failed ** \r\n");
    }
	
    return err;
}

fsp_err_t mtu3_hal_deinit(void)
{
    fsp_err_t err = FSP_SUCCESS;

	err = R_MTU3_THREE_PHASE_Stop(&g_mtu3_m0_3ph_drv_ctrl);
    assert(FSP_SUCCESS == err);
	
	err = R_MTU3_THREE_PHASE_Stop(&g_mtu3_m0_3ph_drv_ctrl);
    assert(FSP_SUCCESS == err);
	
	err = R_MTU3_THREE_PHASE_Close(&g_mtu3_m1_3ph_drv_ctrl);
    assert(FSP_SUCCESS == err);
	
	err = R_MTU3_THREE_PHASE_Close(&g_mtu3_m1_3ph_drv_ctrl);
    assert(FSP_SUCCESS == err);
	
	return err;
}
