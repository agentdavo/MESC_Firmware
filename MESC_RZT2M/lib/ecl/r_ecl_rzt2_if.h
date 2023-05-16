/*******************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only
* intended for use with Renesas products. No other uses are authorized. This
* software is owned by Renesas Electronics Corporation and is protected under
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
* AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software
* and to discontinue the availability of this software. By using this software,
* you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2022 Renesas Electronics Corporation. All rights reserved.
*******************************************************************************/
/*******************************************************************************
* System Name  : Encoder I/F Configuration Library
* File Name    : r_ecl_rzt2_if.h
* Version      : 
* Device       : RZ/T2
* Abstract     : Header file of Encoder I/F Configuration Library.
* Tool-Chain   : IAR Embedded Workbench for ARM
*                Renesas e2 studio
* OS           : Not use
* H/W Platform : Renesas Starter Kit+ for RZ/T2
* Description  : Header file for using Encoder I/F Configuration Library.
* Limitation   : None
*******************************************************************************/
/*******************************************************************************
* History      : History is managed by Revision Control System.
*******************************************************************************/

#ifndef R_ECL_RZT2_IF_H
#define R_ECL_RZT2_IF_H

/*******************************************************************************
Includes <System Includes> , "Project Includes"
*******************************************************************************/
#include <stdint.h>

/*******************************************************************************
Macro definitions
*******************************************************************************/
#define R_ECL_SUCCESS       (0)
#define R_ECL_ERR_ARG       (-1)
#define R_ECL_ERR_FORMAT    (-2)
#define R_ECL_ERR_CRC       (-3)
#define R_ECL_ERR_DEVICE    (-4)
#define R_ECL_ERR_BUSY      (-5)
#define R_ECL_ERR_INTERNAL  (-6)
#define R_ECL_ERR_OVERWRITE (-7)

#define R_ECL_ERR_OS                    (-8)
#define R_ECL_ERR_STATUS                (-9)
#define R_ECL_ERR_TILE_PATTERN          (-10)
#define R_ECL_ERR_STOPPED               (-11)
#define R_ECL_ERR_FREQ                  (-12) /* for future reservations */

#define R_ECL_CH_0   (0x01)
#define R_ECL_CH_1   (0x02)

#define R_ECL_FREQ_10000KHZ             (10000UL) // 10.000MHz
#define R_ECL_FREQ_28571KHZ             (28571UL) // 28.571MHz
#define R_ECL_FREQ_33333KHZ             (33333UL) // 33.333MHz

/*******************************************************************************
Typedef definitions
*******************************************************************************/

/*******************************************************************************
Public Functions
*******************************************************************************/
int32_t R_ECL_Initialize(void);
int32_t R_ECL_Terminate(void);
int32_t R_ECL_ConfigurePin(const void *const pconfig1);
int32_t R_ECL_Configure(const uint8_t id, const void *const pconfig2);
int32_t R_ECL_UnConfigure(const uint8_t id);
int32_t R_ECL_Start(const uint8_t id, const uint32_t freq);
int32_t R_ECL_Stop(const uint8_t id);
uint32_t R_ECL_GetVersion(void);

#endif /* R_ECL_RZT2_IF_H */
