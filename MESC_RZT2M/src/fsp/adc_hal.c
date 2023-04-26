#include "hal_data.h"

#include "adc_hal.h"
#include "MESCerror.h"
#include "MESCfoc.h"

#include "stdio.h"
#define APP_ERR_PRINT printf

volatile bool g_adc0_err_flag = false;
volatile bool g_adc0_scan_complete_flag = false;

volatile bool g_adc1_err_flag = false;
volatile bool g_adc1_scan_complete_flag = false;

void           adc_window_compare_isr(void);



fsp_err_t adc_hal_init(void)
{
    fsp_err_t err = FSP_SUCCESS;
    /*Intialize the configurations for ADC units*/

    /*A/D Compare Function Control Register Settings*/
    g_adc1_ctrl.p_reg->ADCMPCR=0x0000; /*Reset the full register*/
    g_adc1_ctrl.p_reg->ADCMPCR_b.CMPAE = 1; /*Permit operation in Window A */
    g_adc1_ctrl.p_reg->ADCMPCR_b.WCMPE = 1; /*Enable window function*/
    g_adc1_ctrl.p_reg->ADCMPCR_b.CMPAIE = 1; /*Enable window A compare match interrupt */
    /*A/D Compare Function Control Register Settings*/

    /*Select channels for Compare match*/
    g_adc1_ctrl.p_reg->ADCMPANSR0=0x0E00; /*Channel 4,5,6 for phases currents */
    /*Select channels for Compare match*/

    /*Interrupt will be generated when converted value is greater than lower threshold and less than higher threshold*/
    g_adc1_ctrl.p_reg->ADCMPLR0=0x00;
    /*Interrupt will be generated when converted value is greater than lower threshold and less than higher threshold*/

    /*Set Lower threshold*/
    g_adc1_ctrl.p_reg->ADCMPDR0=48;
    /*Set higher threshold*/
    g_adc1_ctrl.p_reg->ADCMPDR1=4048;


    /* Initialize the ADC0 and ADC1 peripherals */
//    err = R_ADC_Open (&g_adc0_ctrl, &g_adc0_cfg);
//    if (FSP_SUCCESS != err )
//    {
//        APP_ERR_PRINT("** R_ADC_Open API for ADC0 failed ** \r\n");
//        return err;
//    }
	
    err = R_ADC_Open (&g_adc1_ctrl, &g_adc1_cfg);
    if (FSP_SUCCESS != err )
    {
        APP_ERR_PRINT("** R_ADC_Open API for ADC1 failed ** \r\n");
        return err;
    }

    /* Configure ADC scanning groups */
//    err = R_ADC_ScanCfg (&g_adc0_ctrl, &g_adc0_channel_cfg);
//    if (FSP_SUCCESS != err )
//    {
//        APP_ERR_PRINT("** R_ADC_ScanCfg API for ADC0 failed ** \r\n");
//        adc_hal_deinit();
//        return err;
//    }
	
    err = R_ADC_ScanCfg (&g_adc1_ctrl, &g_adc1_channel_cfg);
    if (FSP_SUCCESS != err )
    {
        APP_ERR_PRINT("** R_ADC_ScanCfg API for ADC1 failed ** \r\n");
        adc_hal_deinit();
        return err;
    }

    /* Enable scan triggering from ELC events. */
//    err = R_ADC_ScanStart (&g_adc0_ctrl);
//    if (FSP_SUCCESS != err )
//    {
//        APP_ERR_PRINT("** R_ADC_ScanStart API for ADC0 failed ** \r\n");
//        adc_hal_deinit();
//        return err;
//    }
	
    err = R_ADC_ScanStart (&g_adc1_ctrl);
    if (FSP_SUCCESS != err )
    {
        APP_ERR_PRINT("** R_ADC_ScanStart API for ADC1 failed ** \r\n");
        adc_hal_deinit();
        return err;
    }

    APP_ERR_PRINT("** ADC0 & ADC1 module init successful ** \r\n");
    return err;
}


void adc_hal_deinit(void)
{
    fsp_err_t err = FSP_SUCCESS;

    /* Close ADC0 */
//    err = R_ADC_Close (&g_adc0_ctrl);
//    if (FSP_SUCCESS != err )
//    {
//        APP_ERR_PRINT("** R_ADC_Close API for ADC0 failed ** \r\n");
//    }
	
    /* Close ADC1 */	
    err = R_ADC_Close (&g_adc1_ctrl);
    if (FSP_SUCCESS != err )
    {
        APP_ERR_PRINT("** R_ADC_Close API for ADC1 failed ** \r\n");
    }

}


void g_adc0_callback(adc_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    g_adc0_scan_complete_flag = true;
}


void g_adc1_callback(adc_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    g_adc1_scan_complete_flag = true;
}

static void adc_hal_window_compare_callback(uint8_t adc_unit,uint16_t adc_channels0,uint16_t adc_channels1,uint8_t window0,uint8_t window1)
{
	if (1 == adc_unit)
	{
		if(adc_channels1&ADC_CHANNEL4)
		{
			handleError(&mtr[0], ERROR_ADC_OUT_OF_RANGE_IA);
		}
		else if(adc_channels1&ADC_CHANNEL5)
		{
			handleError(&mtr[0], ERROR_ADC_OUT_OF_RANGE_IB);
		}
		else if(adc_channels1&ADC_CHANNEL6)
		{
			handleError(&mtr[0], ERROR_ADC_OUT_OF_RANGE_IC);
		}
		else
		{
			/* Do nothing*/
		}
	}
	else
	{
		/* Do nothing*/
	}
}
/*******************************************************************************************************************//**
 * This function implements the interrupt handler for window compare events.
 **********************************************************************************************************************/
void adc_window_compare_isr (void)
{
    /* Save context if RTOS is used */
    FSP_CONTEXT_SAVE;

    IRQn_Type irq = R_FSP_CurrentIrqGet();

    //adc_instance_ctrl_t * p_instance_ctrl = (adc_instance_ctrl_t *) R_FSP_IsrContextGet(irq);
    //adc_extended_cfg_t  * p_extend        = (adc_extended_cfg_t *) p_instance_ctrl->p_cfg->p_extend;
    uint8_t adc_unit=(uint8_t) 0x00;
    uint16_t adc_channels0=(uint16_t) 0x00;
    uint16_t adc_channels1=(uint16_t) 0x00;
    uint8_t window0=(uint8_t) 0x00;
    uint8_t window1=(uint8_t) 0x00;
    /*Check whether compare match interrupt was from Window A or Window B, for which unit and channels*/
    if(0==g_adc1_ctrl.p_reg->ADWINMON_b.MONCMPA)
    {
        /* Get all Window A status registers */
    	adc_channels1 = g_adc1_ctrl.p_reg->ADCMPSR0;
        /* Clear the status flag */
        g_adc1_ctrl.p_reg->ADCMPSR0 = (uint16_t) 0x00;
        adc_unit|=0x10;
        window1|=0x01;
    }
    else
    {
        /*If Window B is needed */
    }

    if(0==g_adc0_ctrl.p_reg->ADWINMON_b.MONCMPA)
	{
		/* Get all Window A status registers */
    	adc_channels0 = g_adc0_ctrl.p_reg->ADCMPSR0;
		/* Clear the status flag */
		g_adc0_ctrl.p_reg->ADCMPSR0 = (uint16_t) 0x00;
		adc_unit|=0x01;
		window0|=0x01;
	}
	else
	{
		/*If Window B is needed */
	}

    adc_hal_window_compare_callback(adc_unit,adc_channels0,adc_channels1,window0,window1);

    //R_BSP_IrqStatusClear(irq);

    /* Restore context if RTOS is used */
    FSP_CONTEXT_RESTORE;
}
