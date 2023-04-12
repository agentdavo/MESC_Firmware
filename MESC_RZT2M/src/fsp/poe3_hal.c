#include "hal_data.h"

#include "poe3_hal.h"

fsp_err_t poe3_hal_init(void)
{
    fsp_err_t err = FSP_SUCCESS;
	
	/* Start the MTU3 Port Out Enable POE3 */
	err = R_POE3_Open(&g_mtu3_three_phase_poe_ctrl, &g_mtu3_three_phase_poe_cfg);
    if (FSP_SUCCESS != err )
    {
        APP_ERR_PRINT("** R_POE3_Open API for POE3 failed ** \r\n");
    }

    return err;
}

fsp_err_t poe3_hal_deinit(void)
{
    fsp_err_t err = FSP_SUCCESS;
	
    /* Close the MTU3 Port Out Enable POE3 */
	err = R_POE3_Close(&g_mtu3_three_phase_poe_ctrl);
    assert(FSP_SUCCESS == err);
	
	return err;
}
