/***********************************************************************************************************************
 * Copyright [2020-2023] Renesas Electronics Corporation and/or its affiliates.  All Rights Reserved.
 *
 * This software and documentation are supplied by Renesas Electronics Corporation and/or its affiliates and may only
 * be used with products of Renesas Electronics Corp. and its affiliates ("Renesas").  No other uses are authorized.
 * Renesas products are sold pursuant to Renesas terms and conditions of sale.  Purchasers are solely responsible for
 * the selection and use of Renesas products and Renesas assumes no liability.  No license, express or implied, to any
 * intellectual property right is granted by Renesas.  This software is protected under all applicable laws, including
 * copyright laws. Renesas reserves the right to change or discontinue this software and/or this documentation.
 * THE SOFTWARE AND DOCUMENTATION IS DELIVERED TO YOU "AS IS," AND RENESAS MAKES NO REPRESENTATIONS OR WARRANTIES, AND
 * TO THE FULLEST EXTENT PERMISSIBLE UNDER APPLICABLE LAW, DISCLAIMS ALL WARRANTIES, WHETHER EXPLICITLY OR IMPLICITLY,
 * INCLUDING WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT, WITH RESPECT TO THE
 * SOFTWARE OR DOCUMENTATION.  RENESAS SHALL HAVE NO LIABILITY ARISING OUT OF ANY SECURITY VULNERABILITY OR BREACH.
 * TO THE MAXIMUM EXTENT PERMITTED BY LAW, IN NO EVENT WILL RENESAS BE LIABLE TO YOU IN CONNECTION WITH THE SOFTWARE OR
 * DOCUMENTATION (OR ANY PERSON OR ENTITY CLAIMING RIGHTS DERIVED FROM YOU) FOR ANY LOSS, DAMAGES, OR CLAIMS WHATSOEVER,
 * INCLUDING, WITHOUT LIMITATION, ANY DIRECT, CONSEQUENTIAL, SPECIAL, INDIRECT, PUNITIVE, OR INCIDENTAL DAMAGES; ANY
 * LOST PROFITS, OTHER ECONOMIC DAMAGE, PROPERTY DAMAGE, OR PERSONAL INJURY; AND EVEN IF RENESAS HAS BEEN ADVISED OF THE
 * POSSIBILITY OF SUCH LOSS, DAMAGES, CLAIMS OR COSTS.
 **********************************************************************************************************************/

#include "blinky_thread.h"

extern bsp_leds_t g_bsp_leds;

/* Blinky Thread entry function */
void blinky_thread_entry (void * pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);

    /* LED type structure */
    bsp_leds_t leds = g_bsp_leds;

    /* If this board has no LEDs then trap here */
    if (0 == leds.led_count)
    {
        while (1)
        {
            ;                          // There are no LEDs on this board
        }
    }

    /* This code uses BSP IO functions to show how it is used.*/
    /* Turn off LEDs */
    for (uint32_t i = 0; i < leds.led_count; i++)
    {
        R_BSP_PinClear(BSP_IO_REGION_SAFE, (bsp_io_port_pin_t) leds.p_leds[i]);
    }

    while (1)
    {
        /* Toggle board LEDs */
        for (uint32_t i = 0; i < leds.led_count; i++)
        {
            R_BSP_PinToggle(BSP_IO_REGION_SAFE, (bsp_io_port_pin_t) leds.p_leds[i]);
        }

        vTaskDelay(configTICK_RATE_HZ);
    }
}
