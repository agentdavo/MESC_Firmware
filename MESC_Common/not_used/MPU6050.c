/*
 * MPU6050.c
 *
 *  Created on: Dec 13, 2022
 *      Author: David Molony
 */

#include <MPU6050.h>

#include <HAL/MESC_HAL.h>

#include <stdint.h>

int MPU6050Init(uint16_t address, MPU6050_data_t* MPU_instance)
{
  //MPU_instance->MPU6050_I2C = i2c_handle;
  MPU_instance->MPU6050_address = address;
  uint8_t check = 0;
  mpu6050_Read(MPU_instance->MPU6050_address, 0x75, 1, &check, 1, 10);
  //HAL_I2C_Mem_Read(MPU_instance->MPU6050_I2C, MPU_instance->MPU6050_address, 0x75, 1, &check, 1,10);
  SystemNOP();
  uint8_t Data = 0;
  mpu6050_Write(MPU_instance->MPU6050_address, 0x6B, 1, &Data, 1, 1000);
  //HAL_I2C_Mem_Write(MPU_instance->MPU6050_I2C, MPU_instance->MPU6050_address, 0x6B, 1,&Data, 1, 1000);//PWR_MGMT_1_REG
  Data = 0x07;
  mpu6050_Write(MPU_instance->MPU6050_address, 0x19, 1, &Data, 1, 1000);
  //HAL_I2C_Mem_Write(MPU_instance->MPU6050_I2C, MPU_instance->MPU6050_address, 0x19, 1, &Data, 1, 1000);//SMPLRT_DIV_REG
  Data = 0x00;
  mpu6050_Write(MPU_instance->MPU6050_address, 0x1B, 1, &Data, 1, 1000);
  //HAL_I2C_Mem_Write(MPU_instance->MPU6050_I2C, MPU_instance->MPU6050_address, 0x1B, 1, &Data, 1, 1000); //GYRO_CONFIG_REG
  Data = 0x00;
  mpu6050_Write(MPU_instance->MPU6050_address, 0x1C, 1, &Data, 1, 1000);
  //HAL_I2C_Mem_Write(MPU_instance->MPU6050_I2C, MPU_instance->MPU6050_address, 0x1C, 1, &Data, 1, 1000); //ACCEL_CONFIG_REG
  if (check == 0x68)
    {
      return 1;
    }
}

uint8_t get_data[16];

int MPU6050GetData(MPU6050_data_t* MPU_instance)
{
  mpu6050_Read(MPU_instance->MPU6050_address, 0x3B, 1, get_data, 14, 100);
  //HAL_I2C_Mem_Read(MPU_instance->MPU6050_I2C, MPU_instance->MPU6050_address,0x3B, 1, get_data,14,100);

  MPU_instance->Xacc = (int16_t)(get_data[0] << 8 | get_data[1]);
  MPU_instance->Yacc = (int16_t)(get_data[2] << 8 | get_data[3]);
  MPU_instance->Zacc = (int16_t)(get_data[4] << 8 | get_data[5]);
  MPU_instance->temp = ((int16_t)(get_data[6] << 8 | get_data[7])) / 340 + 36;
  MPU_instance->XGYR = (int16_t)(get_data[8] << 8 | get_data[9]);
  MPU_instance->YGYR = (int16_t)(get_data[10] << 8 | get_data[11]);
  MPU_instance->ZGYR = (int16_t)(get_data[12] << 8 | get_data[13]);
  //HAL_I2C_Mem_Read(MPU_instance->MPU6050_I2C, MPU_instance->MPU6050_address,0x1C, 1, get_data,1,100);
  mpu6050_Read(MPU_instance->MPU6050_address, 0x1C, 1, get_data, 1, 100);
  SystemNOP();
}
