#pragma once

// TODO: Should be moved to HAL
#define MESC_GPIO_HALL //GPIOC

//extern TIM_HandleTypeDef htim7;
//extern SPI_HandleTypeDef hspi3;
//#define debugtim htim7

// TODO: Should be moved to HAL
#define getHallState(...) 0//((MESC_GPIO_HALL->IDR >> 6) & 0x7)

/* Temperature parameters */
#define MESC_PROFILE_TEMP_R_F     10000.0f
#define MESC_PROFILE_TEMP_SCHEMA  TEMP_SCHEMA_R_F_ON_R_T
#define MESC_PROFILE_TEMP_SH_BETA 3437.864258f
#define MESC_PROFILE_TEMP_SH_R    0.098243f
#define MESC_PROFILE_TEMP_SH_R0   10000.0f

