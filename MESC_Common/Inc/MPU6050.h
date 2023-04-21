/*
 * MPU6050.h
 *
 *  Created on: Dec 13, 2022
 *      Author: David Molony
 */
#pragma once

#include <stdint.h>

typedef struct
{
  uint16_t MPU6050_address;
  int16_t Xacc;
  int16_t Yacc;
  int16_t Zacc;
  int16_t temp;
  int16_t XGYR;
  int16_t YGYR;
  int16_t ZGYR;
} MPU6050_data_t;

#ifdef __cplus_plus
extern "C"
{
#endif

  int MPU6050Init(uint16_t address, MPU6050_data_t* MPU_instance);
  int MPU6050GetData(MPU6050_data_t* MPU_instance);

#ifdef __cplus_plus
}
#endif
