/*
 * stm32fxxx_hal.h
 *
 *  Created on: Oct 16, 2021
 *      Author: Lenovo
 */

#pragma once

#include "stm32f4xx_hal.h"
#include "MESC_F405.h"
// TODO move all the 405 specific defines (STM32F405xx) into this from:
// MESCfoc.c
// MESC_Comms.c

/*
Hardware identifiers
*/

#define MESC_GPIO_HALL GPIOC

extern TIM_HandleTypeDef htim7;
extern SPI_HandleTypeDef hspi3;
#define debugtim htim7

/*
Function prototypes
*/

#define getHallState(...) ((MESC_GPIO_HALL->IDR >> 6) & 0x7)

/*
Profile defaults
*/

/* Temperature parameters */
#define MESC_PROFILE_TEMP_R_F     10000.0f
#define MESC_PROFILE_TEMP_SCHEMA  TEMP_SCHEMA_R_F_ON_R_T
#define MESC_PROFILE_TEMP_SH_BETA 3437.864258f
#define MESC_PROFILE_TEMP_SH_R    0.098243f
#define MESC_PROFILE_TEMP_SH_R0   10000.0f

