#include <HAL/MESC_HAL.h>

#include <MESCfoc.h>
#include <stm32f4xx_hal_i2c.h>
#include <usbd_cdc_if.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

bool flash_Unlock()
{
  HAL_FLASH_Unlock();
}

bool flash_Lock()
{
  HAL_FLASH_Lock();
}

bool flash_Program(uint32_t addr, const char* src)
{
#ifndef STM32L4xx_HAL_H  //ToDo FIX THIS HACK... L4 series cannot use FLASH_TYPEPROGRAM_WORD... only DOUBLEWORD
		  return HAL_FLASH_Program( FLASH_TYPEPROGRAM_WORD, addr, src) == HAL_OK;
#else
		  return false;
#endif
}

void adc1_start()
{
#ifdef SOFTWARE_ADC_REGULAR
       HAL_ADC_Start(&hadc1); //Try to eliminate the HAL call, slow and inefficient. Leaving this here for now.
        //hadc1.Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
#endif
}

void mpu6050_Write(uint16_t devAddress, uint16_t memAddress, uint16_t memAddSize, uint8_t* data, uint16_t size, uint32_t timeout)
{
  //HAL_I2C_Mem_Write(, devAddress, memAddress, memAddSize, data, size, timeout);
}

void mpu6050_Read(uint16_t devAddress, uint16_t memAddress, uint16_t memAddSize, uint8_t* data, uint16_t size, uint32_t timeout)
{
  //HAL_I2C_Mem_Read(, devAddress, memAddress, memAddSize, data, size, timeout);
}

void ssd1306_WriteData(uint8_t* buffer, size_t buff_size)
{
  //HAL_I2C_Mem_Write(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 0x40, 1, buffer, buff_size, HAL_MAX_DELAY);
}

uint8_t ssd1306_WriteCommand(uint8_t command)
{
  return 0;//HAL_I2C_Mem_Write(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 0x00, 1, &command, 1, 10);
}

void Delay(uint32_t ms)
{
    HAL_Delay(ms);
}

//int getHallState()
//{
//
//}

void SystemNOP()
{
  __NOP();
}

uint32_t SystemGetTick()
{
  return HAL_GetTick();
}

void SystemMCInit()
{
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
//  DEMCR |= DEMCR_TRCENA;
//  DWT_CTRL |= CYCCNTENA;
}

void SystemConfigureDeadTimes()
{
//Reconfigure dead times
//This is only useful up to 1500ns for 168MHz clock, 3us for an 84MHz clock
#ifdef CUSTOM_DEADTIME
  uint32_t tempDT;
  uint32_t tmpbdtr = 0U;
  tmpbdtr = htim1.Instance->BDTR;
  tempDT = (uint32_t)(((float)CUSTOM_DEADTIME * (float)HAL_RCC_GetHCLKFreq())/(float)1000000000.0f);
  if(tempDT < 128)
  {
    MODIFY_REG(tmpbdtr, TIM_BDTR_DTG, tempDT);
  }
  else
  {
    uint32_t deadtime = CUSTOM_DEADTIME;
    deadtime = deadtime-(uint32_t)(127.0f*1000000000.0f/(float)HAL_RCC_GetHCLKFreq());
    tempDT = 0b10000000 + (uint32_t)(((float)deadtime * (float)HAL_RCC_GetHCLKFreq())/(float)2000000000.0f);
    MODIFY_REG(tmpbdtr, TIM_BDTR_DTG, tempDT);
  }
  htim1.Instance->BDTR = tmpbdtr;
#endif
}

void SystemStartSlowdownTimer()
{
  //Start the slowloop timer
  HAL_TIM_Base_Start(&htim2);
  // Here we can auto set the prescaler to get the us input regardless of the main clock
  __HAL_TIM_SET_PRESCALER(&htim2, ((HAL_RCC_GetHCLKFreq()) / 1000000 - 1));
  __HAL_TIM_SET_AUTORELOAD(&htim2, (1000000 / SLOWTIM_SCALER) / SLOW_LOOP_FREQUENCY);  //Run slowloop at 100Hz
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
}

uint32_t SystemHCLKFreq()
{
  return HAL_RCC_GetHCLKFreq();
}

void tim1_enable()
{
  htim1.Instance->BDTR |= (0b01);
}

void tim1_disable()
{

}

void tim1_duty_1(int duty)
{
  htim1.Instance->CCR1 = duty;
}

void tim1_duty1_add(int diff)
{
  htim1.Instance->CCR1 += diff;
}

void tim1_duty_2(int duty)
{
  htim1.Instance->CCR2 = duty;
}

void tim1_duty2_add(int diff)
{
  htim1.Instance->CCR2 += diff;
}

void tim1_duty_3(int duty)
{
  htim1.Instance->CCR3 = duty;
}

void tim1_duty3_add(int diff)
{
  htim1.Instance->CCR3 += diff;
}

void tim1_duty_4(int duty)
{
  htim1.Instance->CCR4 = duty;
}

void tim1_duty4_add(int diff)
{
  htim1.Instance->CCR4 += diff;
}

uint32_t tim1_get_prescaller()
{
  return htim1.Instance->PSC;
}

uint32_t tim1_get_autoreload()
{
  return htim1.Instance->ARR;
}

void tim1_set_autoreload(uint32_t value)
{
  htim1.Instance->ARR = value;
}

void tim1_enable_main_output()
{
  htim1.Instance->BDTR |= TIM_BDTR_MOE;
}

//void tim1_setup_deadtime()
//{
//
//}

// Turn all phase U FETs off, Tristate the HBridge output - For BLDC mode
// mainly, but also used for measuring, software fault detection and recovery
void phU_Break()
{
  uint32_t tmpccmrx = htim1.Instance->CCMR1;
  tmpccmrx &= ~TIM_CCMR1_OC1M;
  tmpccmrx &= ~TIM_CCMR1_CC1S;
  tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE;
  htim1.Instance->CCMR1 = tmpccmrx;
  htim1.Instance->CCER &= ~TIM_CCER_CC1E;   // disable
  htim1.Instance->CCER &= ~TIM_CCER_CC1NE;  // disable
}

// Basically un-break phase U, opposite of above...
void phU_Enable()
{
  uint32_t tmpccmrx = htim1.Instance->CCMR1;
  tmpccmrx &= ~TIM_CCMR1_OC1M;
  tmpccmrx &= ~TIM_CCMR1_CC1S;
  tmpccmrx |= TIM_OCMODE_PWM1;
  htim1.Instance->CCMR1 = tmpccmrx;
  htim1.Instance->CCER |= TIM_CCER_CC1E;   // enable
  htim1.Instance->CCER |= TIM_CCER_CC1NE;  // enable
}

void phV_Break()
{
  uint32_t tmpccmrx = htim1.Instance->CCMR1;
  tmpccmrx &= ~TIM_CCMR1_OC2M;
  tmpccmrx &= ~TIM_CCMR1_CC2S;
  tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE << 8;
  htim1.Instance->CCMR1 = tmpccmrx;
  htim1.Instance->CCER &= ~TIM_CCER_CC2E;   // disable
  htim1.Instance->CCER &= ~TIM_CCER_CC2NE;  // disable
}

void phV_Enable()
{
  uint32_t tmpccmrx = htim1.Instance->CCMR1;
  tmpccmrx &= ~TIM_CCMR1_OC2M;
  tmpccmrx &= ~TIM_CCMR1_CC2S;
  tmpccmrx |= TIM_OCMODE_PWM1 << 8;
  htim1.Instance->CCMR1 = tmpccmrx;
  htim1.Instance->CCER |= TIM_CCER_CC2E;   // enable
  htim1.Instance->CCER |= TIM_CCER_CC2NE;  // enable
}

void phW_Break()
{
  uint32_t tmpccmrx = htim1.Instance->CCMR2;
  tmpccmrx &= ~TIM_CCMR2_OC3M;
  tmpccmrx &= ~TIM_CCMR2_CC3S;
  tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE;
  htim1.Instance->CCMR2 = tmpccmrx;
  htim1.Instance->CCER &= ~TIM_CCER_CC3E;   // disable
  htim1.Instance->CCER &= ~TIM_CCER_CC3NE;  // disable
}

void phW_Enable()
{
  uint32_t tmpccmrx = htim1.Instance->CCMR2;
  tmpccmrx &= ~TIM_CCMR2_OC3M;
  tmpccmrx &= ~TIM_CCMR2_CC3S;
  tmpccmrx |= TIM_OCMODE_PWM1;
  htim1.Instance->CCMR2 = tmpccmrx;
  htim1.Instance->CCER |= TIM_CCER_CC3E;   // enable
  htim1.Instance->CCER |= TIM_CCER_CC3NE;  // enable
}

void invEnableM1()
{
#ifdef INV_ENABLE_M1
  INV_ENABLE_M1->BSRR = INV_ENABLE_M1_IO;  //Write the inverter enable pin high
#endif
}

void invEnableM2()
{
#ifdef INV_ENABLE_M2
  INV_ENABLE_M2->BSRR = INV_ENABLE_M2_IO;  //Write the inverter enable pin high
#endif
}

void invDisableM1()
{
#ifdef INV_ENABLE_M1
  INV_ENABLE_M1->BSRR = INV_ENABLE_M1_IO << 16U;  //Write the inverter enable pin low
#endif
}

void invDisableM2()
{
#ifdef INV_ENABLE_M2
  INV_ENABLE_M2->BSRR = INV_ENABLE_M2_IO << 16U;  //Write the inverter enable pin low
#endif
}

void uart_transmit(const char* send_buffer, uint32_t length)
{
//#ifdef MESC_UART_USB
//CDC_Transmit_FS(coms_instance->data, coms_instance->len);
//#else
//HAL_UART_Transmit_DMA(coms_instance->UART_handle, coms_instance->data, coms_instance->len);
//#endif

#ifdef MESC_UART_USB
CDC_Transmit_FS(send_buffer, length);
#else
extern DMA_HandleTypeDef hdma_usart3_tx;
HAL_UART_Transmit_DMA(uart, send_buffer, length);
while(hdma_usart3_tx.State != HAL_DMA_STATE_READY)
{//Pause here
//while(hdma_usart3_tx.Lock != HAL_UNLOCKED){//Pause here
__NOP();
}
#endif
}

void fastled(bool enable)
{
#ifdef FASTLED
  FASTLED->BSRR = enable ? FASTLEDIO : FASTLEDIO<<16U;
#endif
}

bool tim1_is_downcounting()
{
  return htim1.Instance->CR1 & TIM_CR1_DIR;
}
