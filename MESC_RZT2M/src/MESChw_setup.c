#include <MESChw_rzt2m.h>
#include <MESCfoc.h>

#include "hal_data.h"

hw_setup_s g_hw_setup;
motor_s motor;

static volatile adc_event_t g_adc0_scan_complete_flag;
static volatile adc_event_t g_adc1_scan_complete_flag;

void hw_init(MESC_motor* _motor)
{
  g_hw_setup.Imax       = ABS_MAX_PHASE_CURRENT;
  g_hw_setup.Vmax       = ABS_MAX_BUS_VOLTAGE;
  g_hw_setup.Vmin       = ABS_MIN_BUS_VOLTAGE;
  g_hw_setup.Rshunt     = R_SHUNT;
  g_hw_setup.RVBT       = R_VBUS_TOP;
  g_hw_setup.RVBB       = R_VBUS_BOTTOM;
  g_hw_setup.VBGain     = 0;
  g_hw_setup.RIphPU     = 0;
  g_hw_setup.RIphSR     = 0;
  g_hw_setup.Igain      = 0;
  g_hw_setup.RawCurrLim = 0;
  g_hw_setup.RawVoltLim = 0;

  motor.Lphase          = 0;
}

void setAWDVals()
{
}

void getRawADC(MESC_motor* _motor)
{
	fsp_err_t err = FSP_SUCCESS;
	
  uint16_t adc_data0  = 0 ;
	uint16_t adc_data1  = 0 ;
	uint16_t adc_data2  = 0 ;
	uint16_t adc_data3  = 0 ;
	uint16_t adc_data4  = 0 ;
	uint16_t adc_data5  = 0 ;
	uint16_t adc_data6  = 0 ;
  uint16_t adc_data7  = 0 ;
  uint16_t adc_data8  = 0 ;
  uint16_t adc_data9  = 0 ;
	uint16_t adc_data10 = 0 ;
  uint16_t adc_data11 = 0 ;
	
	// if _motor->id = 0
	// ppoulate raw adc values for Motor 1
	
		g_adc0_scan_complete_flag = false;
		while (!g_adc0_scan_complete_flag)
		{
			;
		}
		err = R_ADC_Read(&g_adc0_ctrl, M1_VOL_B,  &adc_data0);
		err = R_ADC_Read(&g_adc0_ctrl, M1_VOL_U,  &adc_data1);
		err = R_ADC_Read(&g_adc0_ctrl, M1_VOL_V,  &adc_data2);
		err = R_ADC_Read(&g_adc0_ctrl, M1_VOL_W,  &adc_data3);
		err = R_ADC_Read(&g_adc0_ctrl, M1_CUR_U,  &adc_data4);
		err = R_ADC_Read(&g_adc0_ctrl, M1_CUR_V,  &adc_data5);
		err = R_ADC_Read(&g_adc0_ctrl, M1_CUR_W,  &adc_data6);
		err = R_ADC_Read(&g_adc0_ctrl, M1_MOST_U, &adc_data7);
		err = R_ADC_Read(&g_adc0_ctrl, M1_MOST_V, &adc_data8);
		err = R_ADC_Read(&g_adc0_ctrl, M1_MOST_W, &adc_data9);
		err = R_ADC_Read(&g_adc0_ctrl, M1_THROT,  &adc_data10);
		err = R_ADC_Read(&g_adc0_ctrl, M1_TEMP,   &adc_data11);
		
	// if _motor->id = 1
	// ppoulate raw adc values for Motor 2
	
		g_adc1_scan_complete_flag = false;
		while (!g_adc1_scan_complete_flag)
		{
			;
		}
		err = R_ADC_Read(&g_adc1_ctrl, M2_VOL_B,  &adc_data0);
		err = R_ADC_Read(&g_adc1_ctrl, M2_VOL_U,  &adc_data1);
		err = R_ADC_Read(&g_adc1_ctrl, M2_VOL_V,  &adc_data2);
		err = R_ADC_Read(&g_adc1_ctrl, M2_VOL_W,  &adc_data3);
		err = R_ADC_Read(&g_adc1_ctrl, M2_CUR_U,  &adc_data4);
		err = R_ADC_Read(&g_adc1_ctrl, M2_CUR_V,  &adc_data5);
		err = R_ADC_Read(&g_adc1_ctrl, M2_CUR_W,  &adc_data6);
		err = R_ADC_Read(&g_adc1_ctrl, M2_MOST_U, &adc_data7);
		err = R_ADC_Read(&g_adc1_ctrl, M2_MOST_V, &adc_data8);
		err = R_ADC_Read(&g_adc1_ctrl, M2_MOST_W, &adc_data9);
		err = R_ADC_Read(&g_adc1_ctrl, M2_THROT,  &adc_data10);
		err = R_ADC_Read(&g_adc1_ctrl, M2_TEMP,   &adc_data11);
		
	// motor phase voltages
    _motor->Raw.Vu          = adc_data1;
    _motor->Raw.Vv          = adc_data2;
    _motor->Raw.Vw          = adc_data3;		
	
	// motor bus voltage
    _motor->Raw.Vbus        = adc_data0;
	
	// motor phase currents
    _motor->Raw.Iu          = adc_data4;
    _motor->Raw.Iv          = adc_data5;
    _motor->Raw.Iw          = adc_data6;
	
	 // inverter phase temperatures
    _motor->Raw.MOSu_T      = adc_data7;
    _motor->Raw.MOSv_T      = adc_data8;
    _motor->Raw.MOSw_T      = adc_data9;
	
	_motor->Raw.ADC_in_ext1 = adc_data10;
	_motor->Raw.Motor_T     = adc_data11;
}

void getRawADCVph(MESC_motor* _motor)
{
	fsp_err_t err = FSP_SUCCESS;
	
  uint16_t adc_data0  = 0 ;
	uint16_t adc_data1  = 0 ;
	uint16_t adc_data2  = 0 ;
	
	// if _motor->id = 0
	// ppoulate raw adc values for Motor 1
	
		g_adc0_scan_complete_flag = false;
		while (!g_adc0_scan_complete_flag)
		{
			;
		}
		err = R_ADC_Read(&g_adc0_ctrl, M1_VOL_U,  &adc_data0);
		err = R_ADC_Read(&g_adc0_ctrl, M1_VOL_V,  &adc_data1);
		err = R_ADC_Read(&g_adc0_ctrl, M1_VOL_W,  &adc_data2);
		
	// if _motor->id = 1
	// ppoulate raw adc values for Motor 2
	
		g_adc0_scan_complete_flag = false;
		while (!g_adc0_scan_complete_flag)
		{
			;
		}
		err = R_ADC_Read(&g_adc1_ctrl, M2_VOL_V,  &adc_data0);
		err = R_ADC_Read(&g_adc1_ctrl, M2_VOL_V,  &adc_data1);
		err = R_ADC_Read(&g_adc1_ctrl, M2_VOL_W,  &adc_data2);
	
	// motor phase voltages
    _motor->Raw.Vu = adc_data0;
    _motor->Raw.Vv = adc_data1;
    _motor->Raw.Vw = adc_data2;
}

void mesc_init_1(MESC_motor* _motor)
{
}

void mesc_init_2(MESC_motor* _motor)
{
}

void mesc_init_3(MESC_motor* _motor)
{
}
