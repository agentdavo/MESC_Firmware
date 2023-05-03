/*
 * Simple_coms.c
 *
 *  Created on: Dec 14, 2022
 *      Author: HPEnvy
 */

#include <Simple_coms.h>
#include <MESCfoc.h>

#include <HAL/MESC_HAL.h>

#include <stdio.h>

void SimpleComsInit(COMS_data_t *coms_instance)
{
	coms_instance->time = 0;
	coms_instance->period = 100;
}

void SimpleComsProcess(COMS_data_t *coms_instance)
{
	if(!print_samples_now)
  {
    if((SystemGetTick() - coms_instance->time) > coms_instance->period)
    {
      coms_instance->time = SystemGetTick();
      coms_instance->len = sprintf(coms_instance->data,"Vbus: %.2f, eHz: %.2f, Id: %.2f, Iq: %.2f, P: %.2f, \r\n",
      //coms_instance->len = sprintf(coms_instance->data,"%.2f,%.2f,%.2f,%.2f, %.2f \r\n",
        mtr[0].Conv.Vbus,
        mtr[0].FOC.eHz,
        mtr[0].FOC.Idq_smoothed.d,
        mtr[0].FOC.Idq_smoothed.q,
        (mtr[0].FOC.currentPower.q+mtr[0].FOC.currentPower.d));
      uart_transmit(coms_instance->data, coms_instance->len);
    }
	}
  else
  {
		printSamples();
	}
}
