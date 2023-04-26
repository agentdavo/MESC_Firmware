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

// TODO: Moved from MESChw_setup.c
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

extern TIM_HandleTypeDef htim1;
extern SPI_HandleTypeDef hspi3;

hw_setup_s g_hw_setup;
motor_s motor;
uint32_t ADC1_buffer[5];
uint32_t ADC2_buffer[4];

void hw_init(MESC_motor* _motor)
{
  g_hw_setup.Imax = ABS_MAX_PHASE_CURRENT;  	// Imax is the current at which we are either no longer able to
                                            // read it, or hardware "don't ever exceed to avoid breakage"
  g_hw_setup.Vmax = ABS_MAX_BUS_VOLTAGE;  // Headroom beyond which likely to get avalanche of
                                            // MOSFETs or DCDC converter
  g_hw_setup.Vmin = ABS_MIN_BUS_VOLTAGE;  // This implies that the PSU has crapped out or a wire
                                            // has fallen out, and suddenly there will be no power.
  g_hw_setup.Rshunt = R_SHUNT;
  g_hw_setup.RVBB = R_VBUS_BOTTOM;   //
  g_hw_setup.RVBT = R_VBUS_TOP;  //
  g_hw_setup.OpGain = OPGAIN;   //
  g_hw_setup.VBGain = (3.3f / 4096.0f) * (g_hw_setup.RVBB + g_hw_setup.RVBT) / g_hw_setup.RVBB;
  g_hw_setup.Igain = 3.3f / (g_hw_setup.Rshunt * 4096.0f * g_hw_setup.OpGain * SHUNT_POLARITY);  // TODO
  g_hw_setup.RawCurrLim = g_hw_setup.Imax * g_hw_setup.Rshunt * g_hw_setup.OpGain * (4096.0f / 3.3f) + 2048.0f;
  if (g_hw_setup.RawCurrLim > 4000)
    {
      g_hw_setup.RawCurrLim = 4000;
    }  // 4000 is 96 counts away from ADC saturation, allow headroom for opamp not
       // pulling rail:rail.
  g_hw_setup.RawVoltLim = (uint16_t)(4096.0f * (g_hw_setup.Vmax / 3.3f) * g_hw_setup.RVBB /
                                     (g_hw_setup.RVBB + g_hw_setup.RVBT));
}

void getRawADC(MESC_motor* _motor)
{
  //Get the injected critical conversions

  _motor->Raw.Iu = hadc1.Instance->JDR1;  // U Current
  _motor->Raw.Iv = hadc2.Instance->JDR1;  // V Current
  _motor->Raw.Iw = hadc3.Instance->JDR1;  // W Current
  _motor->Raw.Vbus = hadc3.Instance->JDR3;  // DC Link Voltage

  GET_THROTTLE_INPUT; //Define a similar macro in the header file for your board that maps the throttle
#ifdef GET_THROTTLE_INPUT2
  GET_THROTTLE_INPUT2; //Define a similar macro in the header file for your board that maps the throttle
#endif
#ifdef GET_FETU_T
  GET_FETU_T;
#endif
#ifdef GET_FETV_T
  GET_FETV_T;
#endif
#ifdef GET_FETW_T
  GET_FETW_T;
#endif
#ifdef GET_MOTOR_T
  GET_MOTOR_T;
#endif
}

void getRawADCVph(MESC_motor* _motor)
{
  //Voltage sense
  _motor->Raw.Vu = hadc1.Instance->JDR2; //PhaseU Voltage
  _motor->Raw.Vv = hadc2.Instance->JDR2; //PhaseV Voltage
  _motor->Raw.Vw = hadc3.Instance->JDR2; //PhaseW Voltage
}

static uint32_t const flash_sector_map[] = {
// 4 x  16k
FLASH_BASE + (0 * (16 << 10)),
FLASH_BASE + (1 * (16 << 10)),
FLASH_BASE + (2 * (16 << 10)),
FLASH_BASE + (3 * (16 << 10)),
// 1 x  64k
FLASH_BASE + (4 * (16 << 10)),
// 7 x 128k
FLASH_BASE + (4 * (16 << 10)) + (64 << 10) + (0 * (128 << 10)),
FLASH_BASE + (4 * (16 << 10)) + (64 << 10) + (1 * (128 << 10)),
FLASH_BASE + (4 * (16 << 10)) + (64 << 10) + (2 * (128 << 10)),
FLASH_BASE + (4 * (16 << 10)) + (64 << 10) + (3 * (128 << 10)),
FLASH_BASE + (4 * (16 << 10)) + (64 << 10) + (4 * (128 << 10)),
FLASH_BASE + (4 * (16 << 10)) + (64 << 10) + (5 * (128 << 10)),
FLASH_BASE + (4 * (16 << 10)) + (64 << 10) + (6 * (128 << 10)),
// END
FLASH_BASE + (4 * (16 << 10)) + (64 << 10) + (7 * (128 << 10)),
};

static uint32_t getFlashSectorAddress(uint32_t const index)
{
  return flash_sector_map[index];
}

static uint32_t getFlashSectorIndex(uint32_t const address)
{
  for (uint32_t i = 0; i < ((sizeof(flash_sector_map) / sizeof(*flash_sector_map)) - 1); i++)
    {
      if ((flash_sector_map[i] <= address) && (address < flash_sector_map[i + 1]))
        {
          return i;
        }
    }
  return UINT32_MAX;
}

uint32_t getFlashBaseAddress( void )
{
  /*
The base address is FLASH_BASE = 0x08000000 but this is shared with program
memory and so a suitable offset should be used
*/
  return getFlashSectorAddress( 11 );
}

uint32_t getFlashBaseSize( void )
{
  return getFlashSectorAddress( FLASH_STORAGE_PAGE + 1) - getFlashSectorAddress( FLASH_STORAGE_PAGE );
}

ProfileStatus eraseFlash( uint32_t const address, uint32_t const length )
{
  // Disallow zero length (could ignore)
  if (length == 0)
    {
      return PROFILE_STATUS_ERROR_DATA_LENGTH;
    }

  uint32_t const saddr = address;
  uint32_t const eaddr = saddr + length - 1;

  uint32_t const ssector = getFlashSectorIndex( saddr );
  uint32_t const esector = getFlashSectorIndex( eaddr );

  // Limit erasure to a single sector
  if (ssector != esector)
    {
      return PROFILE_STATUS_ERROR_DATA_LENGTH;
    }

  FLASH_EraseInitTypeDef sector_erase;

  sector_erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  sector_erase.Banks     = FLASH_BANK_1; // (ignored)
  sector_erase.Sector    = ssector;
  sector_erase.NbSectors = (esector - ssector + 1);

  uint32_t bad_sector = 0;

  HAL_StatusTypeDef const sts = HAL_FLASHEx_Erase( &sector_erase, &bad_sector );

  switch (sts)
    {
      case HAL_OK:
        return PROFILE_STATUS_SUCCESS;
      case HAL_ERROR:
        return PROFILE_STATUS_ERROR_STORAGE_WRITE;
      case HAL_BUSY:
      case HAL_TIMEOUT:
      default:
        return PROFILE_STATUS_UNKNOWN;
    }
}

void mesc_init_1(MESC_motor* _motor)
{
  // Do nothing
}

void mesc_init_2(MESC_motor* _motor)
{
  // Do nothing
}

void mesc_init_3(MESC_motor* _motor)
{
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADC1_buffer, 5);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&ADC2_buffer, 4);

  HAL_ADCEx_InjectedStart( &hadc1 );
  HAL_ADCEx_InjectedStart( &hadc2 );
  HAL_ADCEx_InjectedStart( &hadc3 );
  //For inverting the ADC trigger polarity, intended for BLDC, not sure it works...
  //    hadc1.Instance->CR2|=ADC_CR2_JEXTEN;
  //    hadc2.Instance->CR2|=ADC_CR2_JEXTEN;
  //    hadc3.Instance->CR2|=ADC_CR2_JEXTEN;


  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);    //OI HAL_TIM_PWM_Start(_motor->mtimer, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);    //OI HAL_TIM_PWM_Start(_motor->mtimer, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); //OI HAL_TIMEx_PWMN_Start(_motor->mtimer, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);    //OI HAL_TIM_PWM_Start(_motor->mtimer, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2); //OI HAL_TIMEx_PWMN_Start(_motor->mtimer, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);    //OI HAL_TIM_PWM_Start(_motor->mtimer, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3); //OI HAL_TIMEx_PWMN_Start(_motor->mtimer, TIM_CHANNEL_3);

  generateBreak(0); //We have started the timers, but we really do not want them PWMing yet

  HAL_Delay(50); //Need to let the ADC start before we enable the fastloop interrupt, otherwise it returns 0 and errors.

  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE); //OI __HAL_TIM_ENABLE_IT(_motor->mtimer, TIM_IT_UPDATE);
}