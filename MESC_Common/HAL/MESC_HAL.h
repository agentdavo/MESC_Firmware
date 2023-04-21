#pragma once

#include <MESC_HAL_Config.h>

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

bool flash_Unlock();
bool flash_Lock();
bool flash_Program(uint32_t addr, const char* src);

void adc1_start();

void mpu6050_Write(uint16_t devAddress, uint16_t memAddress, uint16_t memAddSize, uint8_t* data, uint16_t size, uint32_t timeout);
//HAL_I2C_Mem_Write(MPU_instance->MPU6050_I2Ca

void mpu6050_Read(uint16_t devAddress, uint16_t memAddress, uint16_t memAddSize, uint8_t* data, uint16_t size, uint32_t timeout);
//HAL_I2C_Mem_Read(MPU_instance->MPU6050_I2C

void ssd1306_WriteData(uint8_t* buffer, size_t buff_size);
//{
//  HAL_I2C_Mem_Write(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 0x40, 1, buffer, buff_size, HAL_MAX_DELAY);
//}

uint8_t ssd1306_WriteCommand(uint8_t command);
//{
//  return HAL_I2C_Mem_Write(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 0x00, 1, &command, 1, 10);
//}

void Delay(uint32_t ms);

// TODO: May be should be moved somewhere
//int getHallState();

void SystemNOP();
uint32_t SystemGetTick();

void SystemMCInit();
#if 0           // OI
// TODO: In addition to next comment, IMHO this should be done in port layer side not here
#ifdef STM32L4  // For some reason, ST have decided to have a different name for the L4 timer DBG freeze...
	DBGMCU->APB2FZ |= DBGMCU_APB2FZ_DBG_TIM1_STOP;
#else
	DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM1_STOP;
#endif
#ifdef FASTLED
	FASTLED->MODER |= 0x1<<(FASTLEDIONO*2);
	FASTLED->MODER &= ~(0x2<<(FASTLEDIONO*2));
#endif
#ifdef SLOWLED
	SLOWLED->MODER |= 0x1<<(SLOWLEDIONO*2);
	SLOWLED->MODER &= ~(0x2<<(SLOWLEDIONO*2));
#endif

#ifdef KILLSWITCH_GPIO
	KILLSWITCH_GPIO->MODER &= ~(0b11<<(2*KILLSWITCH_IONO));
#endif

#ifdef INV_ENABLE_M1
	INV_ENABLE_M1->MODER |= 0x1<<(INV_ENABLE_M1_IONO*2);
	INV_ENABLE_M1->MODER &= ~(0x2<<(INV_ENABLE_M1_IONO*2));
#endif
#ifdef INV_ENABLE_M2
	INV_ENABLE_M2->MODER |= 0x1<<(INV_ENABLE_M2_IONO*2);
	INV_ENABLE_M2->MODER &= ~(0x2<<(INV_ENABLE_M2_IONO*2));
#endif

//enable cycle counter
DEMCR |= DEMCR_TRCENA;
DWT_CTRL |= CYCCNTENA;
#endif

void SystemConfigureDeadTimes();
////Reconfigure dead times
////This is only useful up to 1500ns for 168MHz clock, 3us for an 84MHz clock
//#ifdef CUSTOM_DEADTIME
//tim1_setup_deadtime();
//#if 0  // OI
//  uint32_t tempDT;
//  uint32_t tmpbdtr = 0U;
//  tmpbdtr = mtr->mtimer->Instance->BDTR;
//  tempDT = (uint32_t)(((float)CUSTOM_DEADTIME * (float)HAL_RCC_GetHCLKFreq())/(float)1000000000.0f);
//  if(tempDT < 128)
//  {
//    MODIFY_REG(tmpbdtr, TIM_BDTR_DTG, tempDT);
//  }
//  else
//  {
//	  uint32_t deadtime = CUSTOM_DEADTIME;
//	  deadtime = deadtime-(uint32_t)(127.0f*1000000000.0f/(float)HAL_RCC_GetHCLKFreq());
//	  tempDT = 0b10000000 + (uint32_t)(((float)deadtime * (float)HAL_RCC_GetHCLKFreq())/(float)2000000000.0f);
//	  MODIFY_REG(tmpbdtr, TIM_BDTR_DTG, tempDT);
//  }
//  mtr->mtimer->Instance->BDTR = tmpbdtr;
//#endif
//#endif

void SystemStartSlowdownTimer();
////Start the slowloop timer
//HAL_TIM_Base_Start(_motor->stimer);
//// Here we can auto set the prescaler to get the us input regardless of the main clock
//__HAL_TIM_SET_PRESCALER(_motor->stimer, ((HAL_RCC_GetHCLKFreq()) / 1000000 - 1));
//__HAL_TIM_SET_AUTORELOAD(_motor->stimer, (1000000 / SLOWTIM_SCALER) / SLOW_LOOP_FREQUENCY);  //Run slowloop at 100Hz
//__HAL_TIM_ENABLE_IT(_motor->stimer, TIM_IT_UPDATE);

uint32_t SystemHCLKFreq();

void tim1_enable();
void tim1_disable();
void tim1_duty_1(int duty);
void tim1_duty_2(int duty);
void tim1_duty_3(int duty);
void tim1_duty_4(int duty);
uint32_t tim1_get_prescaller();
uint32_t tim1_get_autoreload();
void tim1_set_autoreload(uint32_t);
void tim1_enable_main_output();
void tim1_setup_deadtime();

// Turn all phase U FETs off, Tristate the HBridge output - For BLDC mode
// mainly, but also used for measuring, software fault detection and recovery
void phU_Break();
//{
//  uint32_t tmpccmrx = htim1.Instance->CCMR1;
//  tmpccmrx &= ~TIM_CCMR1_OC1M;
//  tmpccmrx &= ~TIM_CCMR1_CC1S;
//  tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE;
//  htim1.Instance->CCMR1 = tmpccmrx;
//  htim1.Instance->CCER &= ~TIM_CCER_CC1E;   // disable
//  htim1.Instance->CCER &= ~TIM_CCER_CC1NE;  // disable
//}

// Basically un-break phase U, opposite of above...
void phU_Enable();
//{
//  uint32_t tmpccmrx = htim1.Instance->CCMR1;
//  tmpccmrx &= ~TIM_CCMR1_OC1M;
//  tmpccmrx &= ~TIM_CCMR1_CC1S;
//  tmpccmrx |= TIM_OCMODE_PWM1;
//  htim1.Instance->CCMR1 = tmpccmrx;
//  htim1.Instance->CCER |= TIM_CCER_CC1E;   // enable
//  htim1.Instance->CCER |= TIM_CCER_CC1NE;  // enable
//}

void phV_Break();
//{
//  uint32_t tmpccmrx = htim1.Instance->CCMR1;
//  tmpccmrx &= ~TIM_CCMR1_OC2M;
//  tmpccmrx &= ~TIM_CCMR1_CC2S;
//  tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE << 8;
//  htim1.Instance->CCMR1 = tmpccmrx;
//  htim1.Instance->CCER &= ~TIM_CCER_CC2E;   // disable
//  htim1.Instance->CCER &= ~TIM_CCER_CC2NE;  // disable
//}

void phV_Enable();
//{
//  uint32_t tmpccmrx = htim1.Instance->CCMR1;
//  tmpccmrx &= ~TIM_CCMR1_OC2M;
//  tmpccmrx &= ~TIM_CCMR1_CC2S;
//  tmpccmrx |= TIM_OCMODE_PWM1 << 8;
//  htim1.Instance->CCMR1 = tmpccmrx;
//  htim1.Instance->CCER |= TIM_CCER_CC2E;   // enable
//  htim1.Instance->CCER |= TIM_CCER_CC2NE;  // enable
//}

void phW_Break();
//{
//  uint32_t tmpccmrx = htim1.Instance->CCMR2;
//  tmpccmrx &= ~TIM_CCMR2_OC3M;
//  tmpccmrx &= ~TIM_CCMR2_CC3S;
//  tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE;
//  htim1.Instance->CCMR2 = tmpccmrx;
//  htim1.Instance->CCER &= ~TIM_CCER_CC3E;   // disable
//  htim1.Instance->CCER &= ~TIM_CCER_CC3NE;  // disable
//}

void phW_Enable();
//{
//  uint32_t tmpccmrx = htim1.Instance->CCMR2;
//  tmpccmrx &= ~TIM_CCMR2_OC3M;
//  tmpccmrx &= ~TIM_CCMR2_CC3S;
//  tmpccmrx |= TIM_OCMODE_PWM1;
//  htim1.Instance->CCMR2 = tmpccmrx;
//  htim1.Instance->CCER |= TIM_CCER_CC3E;   // enable
//  htim1.Instance->CCER |= TIM_CCER_CC3NE;  // enable
//}

void invEnableM1();
//#ifdef INV_ENABLE_M1
//INV_ENABLE_M1->BSRR = INV_ENABLE_M1_IO;  //Write the inverter enable pin high
//#endif
void invEnableM2();
//#ifdef INV_ENABLE_M2
//INV_ENABLE_M2->BSRR = INV_ENABLE_M2_IO;  //Write the inverter enable pin high
//#endif

void invDisableM1();
//#ifdef INV_ENABLE_M1
//INV_ENABLE_M1->BSRR = INV_ENABLE_M1_IO << 16U;  //Write the inverter enable pin low
//#endif
void invDisableM2();
//#ifdef INV_ENABLE_M2
//INV_ENABLE_M2->BSRR = INV_ENABLE_M2_IO << 16U;  //Write the inverter enable pin low
//#endif

void uart_transmit(const char*, uint32_t);

//extern DMA_HandleTypeDef hdma_usart3_tx;
//#ifdef MESC_UART_USB
//CDC_Transmit_FS(coms_instance->data, coms_instance->len);
//#else
//HAL_UART_Transmit_DMA(coms_instance->UART_handle, coms_instance->data, coms_instance->len);
//#endif

//#ifdef MESC_UART_USB
//CDC_Transmit_FS(send_buffer, length);
//#else
//HAL_UART_Transmit_DMA(uart, send_buffer, length);
//while(hdma_usart3_tx.State != HAL_DMA_STATE_READY)
//{//Pause here
////while(hdma_usart3_tx.Lock != HAL_UNLOCKED){//Pause here
//__NOP();
//}
//#endif

void fastled(bool enable);
bool tim1_is_downcounting();