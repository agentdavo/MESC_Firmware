#include <HAL/MESC_HAL.h>

#include <MESCfoc.h>

#include <hal_data.h>

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

bool flash_Unlock()
{
  //HAL_FLASH_Unlock();
}

bool flash_Lock()
{
  //HAL_FLASH_Lock();
}

bool flash_Program(uint32_t addr, const char* src)
{
  //#ifndef STM32L4xx_HAL_H  //ToDo FIX THIS HACK... L4 series cannot use FLASH_TYPEPROGRAM_WORD... only DOUBLEWORD
  //		  return HAL_FLASH_Program( FLASH_TYPEPROGRAM_WORD, addr, src) == HAL_OK;
  //#else
  //		  return false;
  //#endif
}

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
  // The base address is FLASH_BASE = 0x08000000 but this is shared with program memory and so a suitable offset should be used
  return getFlashSectorAddress( 11 );
}

uint32_t getFlashBaseSize( void )
{
  return getFlashSectorAddress( FLASH_STORAGE_PAGE + 1) - getFlashSectorAddress( FLASH_STORAGE_PAGE );
}

#if 0
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
#endif

void uart_transmit(const char* send_buffer, uint32_t length)
{
#if 0
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
#endif
}

void adc1_start()
{
//#ifdef SOFTWARE_ADC_REGULAR
//       HAL_ADC_Start(&hadc1); //Try to eliminate the HAL call, slow and inefficient. Leaving this here for now.
//        //hadc1.Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
//#endif
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
    R_BSP_SoftwareDelay(ms, BSP_DELAY_UNITS_MILLISECONDS);
}

void SystemNOP()
{
  //__NOP();
}

uint32_t SystemGetTick()
{
  return 0;//HAL_GetTick();
}

uint32_t SystemHCLKFreq()
{
  return 0;//HAL_RCC_GetHCLKFreq();
}

//int getHallState()
//{
//
//}

hw_setup_s g_hw_setup;
motor_s motor;
uint32_t ADC1_buffer[5];
uint32_t ADC2_buffer[4];

void hw_init(MESC_motor* _motor)
{
  g_hw_setup.Imax = ABS_MAX_PHASE_CURRENT; // Imax is the current at which we are either no longer able to
                                           // read it, or hardware "don't ever exceed to avoid breakage"
  g_hw_setup.Vmax = ABS_MAX_BUS_VOLTAGE;   // Headroom beyond which likely to get avalanche of
                                           // MOSFETs or DCDC converter
  g_hw_setup.Vmin = ABS_MIN_BUS_VOLTAGE;   // This implies that the PSU has crapped out or a wire
                                           // has fallen out, and suddenly there will be no power.
  g_hw_setup.Rshunt = R_SHUNT;
  g_hw_setup.RVBT = R_VBUS_TOP;            //
  g_hw_setup.RVBB = R_VBUS_BOTTOM;         //
  g_hw_setup.OpGain = OPGAIN;              //
  g_hw_setup.VBGain = (3.3f / 4096.0f) * (g_hw_setup.RVBB + g_hw_setup.RVBT) / g_hw_setup.RVBB;
  g_hw_setup.Igain = 3.3f / (g_hw_setup.Rshunt * 4096.0f * g_hw_setup.OpGain * SHUNT_POLARITY);  // TODO
  g_hw_setup.RIphPU = 0;
  g_hw_setup.RIphSR = 0;
  g_hw_setup.RawCurrLim = g_hw_setup.Imax * g_hw_setup.Rshunt * g_hw_setup.OpGain * (4096.0f / 3.3f) + 2048.0f;
  if (g_hw_setup.RawCurrLim > 4000)
    {
      g_hw_setup.RawCurrLim = 4000;
    }  // 4000 is 96 counts away from ADC saturation, allow headroom for opamp not
       // pulling rail:rail.
  g_hw_setup.RawVoltLim = (uint16_t)(4096.0f * (g_hw_setup.Vmax / 3.3f) * g_hw_setup.RVBB /
                                     (g_hw_setup.RVBB + g_hw_setup.RVBT));
  motor.Lphase = 0;
}

void MotorInitStage0_0(MESC_motor* _motor)
{
#if 0
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
#endif
}

void MotorInitStage0_1(MESC_motor* _motor)
{

}

void MotorInitStage1_0(MESC_motor* _motor)
{
  // Do nothing
}

void MotorInitStage1_1(MESC_motor* _motor)
{
  // Do nothing
}

void MotorInitStage2_0(MESC_motor* _motor)
{
  // Do nothing
}

void MotorInitStage2_1(MESC_motor* _motor)
{
  // Do nothing
}

// TODO: Should be moved to mesc_init_3
void SystemConfigureDeadTimes()
{
#if 0
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
#endif
}

void MotorInitStage3_0(MESC_motor* _motor)
{
  SystemConfigureDeadTimes();
}

void MotorInitStage3_1(MESC_motor* _motor)
{
#if 0
  SystemConfigureDeadTimes();

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
#endif
}

void FastLed0(bool enabled)
{
#ifdef FASTLED
  //FASTLED->BSRR = enable ? FASTLEDIO : FASTLEDIO<<16U;
  //R_BSP_PinClear(BSP_IO_REGION_SAFE, (bsp_io_port_pin_t)leds.p_leds[i]);
  //R_BSP_PinToggle(BSP_IO_REGION_SAFE, (bsp_io_port_pin_t)leds.p_leds[i]);
#endif
}

void SlowLed0(bool enabled)
{
#ifdef SLOWLED
  //SLOWLED->BSRR = enabled ? SLOWLEDIO : SLOWLEDIO << 16U;
  //R_BSP_PinClear(BSP_IO_REGION_SAFE, (bsp_io_port_pin_t)leds.p_leds[i]);
  //R_BSP_PinToggle(BSP_IO_REGION_SAFE, (bsp_io_port_pin_t)leds.p_leds[i]);
#endif
}

void FastLed1(bool enabled)
{

}

void SlowLed1(bool enabled)
{

}

void SlowTimerStart0()
{
#if 0
  //Start the slowloop timer
  HAL_TIM_Base_Start(&htim2);
  // Here we can auto set the prescaler to get the us input regardless of the main clock
  __HAL_TIM_SET_PRESCALER(&htim2, ((HAL_RCC_GetHCLKFreq()) / 1000000 - 1));
  __HAL_TIM_SET_AUTORELOAD(&htim2, (1000000 / SLOWTIM_SCALER) / SLOW_LOOP_FREQUENCY);  //Run slowloop at 100Hz
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
#endif
}

void SlowTimerStart1()
{
#if 0
  //Start the slowloop timer
  HAL_TIM_Base_Start(&htim2);
  // Here we can auto set the prescaler to get the us input regardless of the main clock
  __HAL_TIM_SET_PRESCALER(&htim2, ((HAL_RCC_GetHCLKFreq()) / 1000000 - 1));
  __HAL_TIM_SET_AUTORELOAD(&htim2, (1000000 / SLOWTIM_SCALER) / SLOW_LOOP_FREQUENCY);  //Run slowloop at 100Hz
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
#endif
}

void EncoderInit0()
{
  //OI HAL_SPI_Init(&hspi3);
}

void EncoderInit1()
{
  //OI HAL_SPI_Init(&hspi3);
}

void EncoderChipSelectEnable(bool isEnabled);
//OI HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // enable
//OI HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET); // disable

void EncoderSpiSend(uint8_t const* data, uint32_t len, uint32_t timeout);
//OI HAL_SPI_Transmit(&hspi3, (uint8_t*)&reg, 1, 1000);
void EncoderSpiReceive(uint8_t* data, uint32_t len, uint32_t timeout);
//OI HAL_SPI_Receive(&hspi3, (uint8_t*)&pkt, len, 1000);

uint16_t EncoderAngle0()
{
//#ifdef USE_ENCODER
//  struct __attribute__((__packed__)) SamplePacket
//  {
//    struct
//    {
//      uint8_t crc;
//      uint8_t STAT_RESP;  // Should be 0xF_?
//    } safetyword;
//    uint16_t angle;
//    int16_t speed;
//    uint16_t revolutions;
//  };
//  typedef struct SamplePacket SamplePacket;
//  static SamplePacket pkt;
//  uint16_t const len = sizeof(pkt) / sizeof(uint16_t);
//  uint16_t reg = (UINT16_C(1) << 15)     /* RW=Read */
//                 | (UINT16_C(0x0) << 11) /* Lock */
//                 | (UINT16_C(0x0) << 10) /* UPD=Buffer */
//                 | (UINT16_C(0x02) << 4) /* ADDR */
//                 | (len - 1);            /* ND */
//  EncoderChipSelectEnable(true); //OI HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
//  EncoderSpiSend((uint8_t*)&reg, 1, 1000);//OI HAL_SPI_Transmit(&hspi3, (uint8_t*)&reg, 1, 1000);
//  EncoderSpiReceive((uint8_t*)&pkt, len, 1000);//OI HAL_SPI_Receive(&hspi3, (uint8_t*)&pkt, len, 1000);
//  //      volatile uint8_t crc = 0;
//  //#if 1
//  //      reg ^= 0xFF00;
//  //      crc = pkt_crc8( crc, &((uint8_t *)&reg)[1], 1 );
//  //      crc = pkt_crc8( crc, &((uint8_t *)&reg)[0], 1 );
//  //      crc = pkt_crc8( crc, &((uint8_t *)&pkt.angle)[1], 1 );
//  //      crc = pkt_crc8( crc, &((uint8_t *)&pkt.angle)[0], 1 );
//  //      crc = pkt_crc8( crc, &((uint8_t *)&pkt.speed)[1], 1 );
//  //      crc = pkt_crc8( crc, &((uint8_t *)&pkt.speed)[0], 1 );
//  //      crc = pkt_crc8( crc, &((uint8_t *)&pkt.revolutions)[1], 1 );
//  //      crc = pkt_crc8( crc, &((uint8_t *)&pkt.revolutions)[0], 1 );
//  //#else
//  //      crc = pkt_crc8( crc, &reg, 2 );
//  //      crc = pkt_crc8( crc, &pkt.angle, 6 );
//  //#endif
//  //      crc = pkt_crc8( crc, &pkt.safetyword.STAT_RESP, 1 );
//  //      crc = ~crc;
//  //      if (crc != pkt.safetyword.crc)
//  //      {
//  //    	  __NOP();
//  //    	  __NOP();
//  //    	  __NOP();
//  //      }
//  //      else
//  //      {
//  //    	  __NOP();
//  //      }
//  EncoderChipSelectEnable(false); //OI HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
//  pkt.revolutions = pkt.revolutions & 0b0000000111111111;
//  return pkt.angle & 0x7fff;
//#else
//  return 0;
//#endif
}

uint16_t EncoderAngle1()
{

}

void MotorTimerEnable0()
{
  //OI htim1.Instance->BDTR |= (0b01);
}

void MotorTimerEnable1()
{
  //OI htim1.Instance->BDTR |= (0b01);
}

void MotorTimerDuty1_0(int32_t duty)
{
  //OI htim1.Instance->CCR1 = duty;
}

void MotorTimerDuty1_1(int32_t duty)
{
  //OI htim1.Instance->CCR1 = duty;
}

void MotorTimerDutyAdd1_0(int32_t diff)
{
  //OI htim1.Instance->CCR1 += diff;
}

void MotorTimerDutyAdd1_1(int32_t diff)
{
  //OI htim1.Instance->CCR1 += diff;
}

void MotorTimerDuty2_0(int32_t duty)
{
  //OI htim1.Instance->CCR2 = duty;
}

void MotorTimerDutyAdd2_0(int32_t diff)
{
  //OI htim1.Instance->CCR2 += diff;
}

void MotorTimerDuty2_1(int32_t duty)
{
  //OI htim1.Instance->CCR2 = duty;
}

void MotorTimerDutyAdd2_1(int32_t diff)
{
  //OI htim1.Instance->CCR2 += diff;
}

void MotorTimerDuty3_0(int32_t duty)
{
  //OI htim1.Instance->CCR3 = duty;
}

void MotorTimerDutyAdd3_0(int32_t diff)
{
  //OI htim1.Instance->CCR3 += diff;
}

void MotorTimerDuty3_1(int32_t duty)
{
  //OI htim1.Instance->CCR3 = duty;
}

void MotorTimerDutyAdd3_1(int32_t diff)
{
  //OI htim1.Instance->CCR3 += diff;
}

void MotorTimerDuty4_0(int32_t duty)
{
  //OI htim1.Instance->CCR4 = duty;
}

void MotorTimerDutyAdd4_0(int32_t diff)
{
  //OI htim1.Instance->CCR4 += diff;
}

void MotorTimerDuty4_1(int32_t duty)
{
  //OI htim1.Instance->CCR4 = duty;
}

void MotorTimerDutyAdd4_1(int32_t diff)
{
  //OI htim1.Instance->CCR4 += diff;
}

uint32_t MotorTimerGetPrescaller0()
{
  return 0;//OI htim1.Instance->PSC;
}

uint32_t MotorTimerGetAutoreload0()
{
  return 0; //OI htim1.Instance->ARR;
}

void MotorTimerSetAutoreload0(uint32_t value)
{
  //OI htim1.Instance->ARR = value;
}

uint32_t MotorTimerGetPrescaller1()
{
  return 0;//OI htim1.Instance->PSC;
}

uint32_t MotorTimerGetAutoreload1()
{
  return 0; //OI htim1.Instance->ARR;
}

void MotorTimerSetAutoreload1(uint32_t value)
{
  //OI htim1.Instance->ARR = value;
}

void MotorTimerEnableMainOutput0()
{
  //OI htim1.Instance->BDTR |= TIM_BDTR_MOE;
}

void MotorTimerEnableMainOutput1()
{
  //OI htim1.Instance->BDTR |= TIM_BDTR_MOE;
}

//void tim1_setup_deadtime()
//{
//
//}

// Turn all phase U FETs off, Tristate the HBridge output - For BLDC mode
// mainly, but also used for measuring, software fault detection and recovery
void phU_Break0()
{

}

void phU_Break1()
{
#if 0
  uint32_t tmpccmrx = htim1.Instance->CCMR1;
  tmpccmrx &= ~TIM_CCMR1_OC1M;
  tmpccmrx &= ~TIM_CCMR1_CC1S;
  tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE;
  htim1.Instance->CCMR1 = tmpccmrx;
  htim1.Instance->CCER &= ~TIM_CCER_CC1E;   // disable
  htim1.Instance->CCER &= ~TIM_CCER_CC1NE;  // disable
#endif
}

// Basically un-break phase U, opposite of above...
void phU_Enable0()
{

}

void phU_Enable1()
{
#if 0
  uint32_t tmpccmrx = htim1.Instance->CCMR1;
  tmpccmrx &= ~TIM_CCMR1_OC1M;
  tmpccmrx &= ~TIM_CCMR1_CC1S;
  tmpccmrx |= TIM_OCMODE_PWM1;
  htim1.Instance->CCMR1 = tmpccmrx;
  htim1.Instance->CCER |= TIM_CCER_CC1E;   // enable
  htim1.Instance->CCER |= TIM_CCER_CC1NE;  // enable
#endif
}

void phV_Break0()
{

}

void phV_Break1()
{
#if 0
  uint32_t tmpccmrx = htim1.Instance->CCMR1;
  tmpccmrx &= ~TIM_CCMR1_OC2M;
  tmpccmrx &= ~TIM_CCMR1_CC2S;
  tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE << 8;
  htim1.Instance->CCMR1 = tmpccmrx;
  htim1.Instance->CCER &= ~TIM_CCER_CC2E;   // disable
  htim1.Instance->CCER &= ~TIM_CCER_CC2NE;  // disable
#endif
}

void phV_Enable0()
{

}

void phV_Enable1()
{
#if 0
  uint32_t tmpccmrx = htim1.Instance->CCMR1;
  tmpccmrx &= ~TIM_CCMR1_OC2M;
  tmpccmrx &= ~TIM_CCMR1_CC2S;
  tmpccmrx |= TIM_OCMODE_PWM1 << 8;
  htim1.Instance->CCMR1 = tmpccmrx;
  htim1.Instance->CCER |= TIM_CCER_CC2E;   // enable
  htim1.Instance->CCER |= TIM_CCER_CC2NE;  // enable
#endif
}

void phW_Break0()
{

}

void phW_Break1()
{
#if 0
  uint32_t tmpccmrx = htim1.Instance->CCMR2;
  tmpccmrx &= ~TIM_CCMR2_OC3M;
  tmpccmrx &= ~TIM_CCMR2_CC3S;
  tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE;
  htim1.Instance->CCMR2 = tmpccmrx;
  htim1.Instance->CCER &= ~TIM_CCER_CC3E;   // disable
  htim1.Instance->CCER &= ~TIM_CCER_CC3NE;  // disable
#endif
}

void phW_Enable0()
{

}

void phW_Enable1()
{
#if 0
  uint32_t tmpccmrx = htim1.Instance->CCMR2;
  tmpccmrx &= ~TIM_CCMR2_OC3M;
  tmpccmrx &= ~TIM_CCMR2_CC3S;
  tmpccmrx |= TIM_OCMODE_PWM1;
  htim1.Instance->CCMR2 = tmpccmrx;
  htim1.Instance->CCER |= TIM_CCER_CC3E;   // enable
  htim1.Instance->CCER |= TIM_CCER_CC3NE;  // enable
#endif
}

void invEnableM0(bool enabled)
{
#ifdef INV_ENABLE_M1
  INV_ENABLE_M1->BSRR = enabled ? INV_ENABLE_M1_IO :  INV_ENABLE_M1_IO << 16U;  //Write the inverter enable pin high/low
#endif
}

void invEnableM1(bool enabled)
{
#ifdef INV_ENABLE_M2
  INV_ENABLE_M2->BSRR = enabled ? INV_ENABLE_M2_IO : INV_ENABLE_M2_IO << 16U;  //Write the inverter enable pin high/low
#endif
}

bool tim1_is_downcounting()
{
  return 0;//OI htim1.Instance->CR1 & TIM_CR1_DIR;
}

#define CHECK(x) while(x != FSP_SUCCESS);

uint16_t R_ADC_Read_(adc_ctrl_t* p_ctrl, adc_channel_t const reg_id)
{
  uint16_t adc_data = 0;
  CHECK(R_ADC_Read(p_ctrl, reg_id, &adc_data));
  return adc_data;
}

static volatile adc_event_t g_adc0_scan_complete_flag;
static volatile adc_event_t g_adc1_scan_complete_flag;

#if 0
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
#endif

void getRawADC0(MESC_motor* _motor)
{
  g_adc0_scan_complete_flag = false;
  while (!g_adc0_scan_complete_flag) {}

  // motor bus voltage
  _motor->Raw.Vbus = R_ADC_Read_(&g_adc0_ctrl, M1_VOL_B);

  // motor phase voltages
  _motor->Raw.Vu = R_ADC_Read_(&g_adc0_ctrl, M1_VOL_U);
  _motor->Raw.Vv = R_ADC_Read_(&g_adc0_ctrl, M1_VOL_V);
  _motor->Raw.Vw = R_ADC_Read_(&g_adc0_ctrl, M1_VOL_W);

  // motor phase currents
  _motor->Raw.Iu = R_ADC_Read_(&g_adc0_ctrl, M1_CUR_U);
  _motor->Raw.Iv = R_ADC_Read_(&g_adc0_ctrl, M1_CUR_V);
  _motor->Raw.Iw = R_ADC_Read_(&g_adc0_ctrl, M1_CUR_W);

  // inverter phase temperatures
  _motor->Raw.MOSu_T = R_ADC_Read_(&g_adc0_ctrl, M1_MOST_U);
  _motor->Raw.MOSv_T = R_ADC_Read_(&g_adc0_ctrl, M1_MOST_V);
  _motor->Raw.MOSw_T = R_ADC_Read_(&g_adc0_ctrl, M1_MOST_W);

  _motor->Raw.ADC_in_ext1 = R_ADC_Read_(&g_adc0_ctrl, M1_THROT);
  _motor->Raw.Motor_T = R_ADC_Read_(&g_adc0_ctrl, M1_TEMP);
}

void getRawADC1(MESC_motor* _motor)
{
  g_adc1_scan_complete_flag = false;
  while (!g_adc1_scan_complete_flag) {}

  // motor bus voltage
  _motor->Raw.Vbus = R_ADC_Read_(&g_adc1_ctrl, M2_VOL_B);

  // motor phase voltages
  _motor->Raw.Vu = R_ADC_Read_(&g_adc1_ctrl, M2_VOL_U);
  _motor->Raw.Vv = R_ADC_Read_(&g_adc1_ctrl, M2_VOL_V);
  _motor->Raw.Vw = R_ADC_Read_(&g_adc1_ctrl, M2_VOL_W);

  // motor phase currents
  _motor->Raw.Iu = R_ADC_Read_(&g_adc1_ctrl, M2_CUR_U);
  _motor->Raw.Iv = R_ADC_Read_(&g_adc1_ctrl, M2_CUR_V);
  _motor->Raw.Iw = R_ADC_Read_(&g_adc1_ctrl, M2_CUR_W);

  // inverter phase temperatures
  _motor->Raw.MOSu_T = R_ADC_Read_(&g_adc1_ctrl, M2_MOST_U);
  _motor->Raw.MOSv_T = R_ADC_Read_(&g_adc1_ctrl, M2_MOST_V);
  _motor->Raw.MOSw_T = R_ADC_Read_(&g_adc1_ctrl, M2_MOST_W);

  _motor->Raw.ADC_in_ext1 = R_ADC_Read_(&g_adc1_ctrl, M2_THROT);
  _motor->Raw.Motor_T = R_ADC_Read_(&g_adc1_ctrl, M2_TEMP);
}

void getRawADC0Vph(MESC_motor* _motor)
{
  // if _motor->id = 0
  // ppoulate raw adc values for Motor 1

  g_adc0_scan_complete_flag = false;
  while (!g_adc0_scan_complete_flag) {}
  _motor->Raw.Vu = R_ADC_Read_(&g_adc0_ctrl, M1_VOL_U);
  _motor->Raw.Vv = R_ADC_Read_(&g_adc0_ctrl, M1_VOL_V);
  _motor->Raw.Vw = R_ADC_Read_(&g_adc0_ctrl, M1_VOL_W);
}

void getRawADC1Vph(MESC_motor* _motor)
{
  // if _motor->id = 1
  // ppoulate raw adc values for Motor 2

  g_adc0_scan_complete_flag = false;
  while (!g_adc0_scan_complete_flag) {}

  // motor phase voltages
  _motor->Raw.Vu = R_ADC_Read_(&g_adc1_ctrl, M2_VOL_U);
  _motor->Raw.Vv = R_ADC_Read_(&g_adc1_ctrl, M2_VOL_V);
  _motor->Raw.Vw = R_ADC_Read_(&g_adc1_ctrl, M2_VOL_W);
}

void setAWDVals()
{
}

MESC_hal* getHalForMotor0()
{
  static MESC_hal mescHal;
  mescHal.MotorInitStage0 = &MotorInitStage0_0;
  mescHal.MotorInitStage1 = &MotorInitStage1_0;
  mescHal.MotorInitStage2 = &MotorInitStage2_0;
  mescHal.MotorInitStage3 = &MotorInitStage3_0;
  mescHal.FastLed = &FastLed0;
  mescHal.SlowLed = &SlowLed0;
  mescHal.SlowTimerStart = &SlowTimerStart0;
  mescHal.EncoderInit = &EncoderInit0;
  mescHal.EncoderAngle = &EncoderAngle0;
  //mescHal.MotorTimerInit = ;
  mescHal.MotorTimerEnable = &MotorTimerEnable0;
  //mescHal.MotorTimerDisable = ;
  mescHal.MotorTimerDuty1 = &MotorTimerDuty1_0;
  mescHal.MotorTimerDuty2 = &MotorTimerDuty2_0;
  mescHal.MotorTimerDuty3 = &MotorTimerDuty3_0;
  mescHal.MotorTimerDuty4 = &MotorTimerDuty4_0;
  mescHal.MotorTimerDutyAdd1 = &MotorTimerDutyAdd1_0;
  mescHal.MotorTimerDutyAdd2 = &MotorTimerDutyAdd2_0;
  mescHal.MotorTimerDutyAdd3 = &MotorTimerDutyAdd3_0;
  mescHal.MotorTimerDutyAdd4 = &MotorTimerDutyAdd4_0;
  mescHal.MotorTimerGetPrescaller = &MotorTimerGetPrescaller0;
  mescHal.MotorTimerGetAutoreload = &MotorTimerGetAutoreload0;
  mescHal.MotorTimerSetAutoreload = &MotorTimerSetAutoreload0;
  mescHal.MotorTimerEnableMainOutput = &MotorTimerEnableMainOutput0;
  //mescHal.MotorTimerSetupDeadtime = ;
  mescHal.phU_Break = &phU_Break0;
  mescHal.phV_Break = &phV_Break0;
  mescHal.phW_Break = &phW_Break0;
  mescHal.phU_Enable = &phU_Enable0;
  mescHal.phV_Enable = &phV_Enable0;
  mescHal.phW_Enable = &phW_Enable0;
  //void (*invDisable)(void);
  mescHal.invEnable = &invEnableM0;
  mescHal.getRawADC = &getRawADC0;
  mescHal.getRawADCVph = &getRawADC0Vph;
  return &mescHal;
}

MESC_hal* getHalForMotor1()
{
  static MESC_hal mescHal;
  mescHal.MotorInitStage0 = &MotorInitStage0_1;
  mescHal.MotorInitStage1 = &MotorInitStage1_1;
  mescHal.MotorInitStage2 = &MotorInitStage2_1;
  mescHal.MotorInitStage3 = &MotorInitStage3_1;
  mescHal.FastLed = &FastLed0;
  mescHal.SlowLed = &SlowLed0;
  mescHal.SlowTimerStart = &SlowTimerStart1;
  mescHal.EncoderInit = &EncoderInit1;
  mescHal.EncoderAngle = &EncoderAngle1;
  //mescHal.MotorTimerInit = ;
  mescHal.MotorTimerEnable = &MotorTimerEnable1;
  //mescHal.MotorTimerDisable = ;
  mescHal.MotorTimerDuty1 = &MotorTimerDuty1_1;
  mescHal.MotorTimerDuty2 = &MotorTimerDuty2_1;
  mescHal.MotorTimerDuty3 = &MotorTimerDuty3_1;
  mescHal.MotorTimerDuty4 = &MotorTimerDuty4_1;
  mescHal.MotorTimerDutyAdd1 = &MotorTimerDutyAdd1_1;
  mescHal.MotorTimerDutyAdd2 = &MotorTimerDutyAdd2_1;
  mescHal.MotorTimerDutyAdd3 = &MotorTimerDutyAdd3_1;
  mescHal.MotorTimerDutyAdd4 = &MotorTimerDutyAdd4_1;
  mescHal.MotorTimerGetPrescaller = &MotorTimerGetPrescaller1;
  mescHal.MotorTimerGetAutoreload = &MotorTimerGetAutoreload1;
  mescHal.MotorTimerSetAutoreload = &MotorTimerSetAutoreload1;
  mescHal.MotorTimerEnableMainOutput = &MotorTimerEnableMainOutput1;
  //mescHal.MotorTimerSetupDeadtime = ;
  mescHal.phU_Break = &phU_Break1;
  mescHal.phV_Break = &phV_Break1;
  mescHal.phW_Break = &phW_Break1;
  mescHal.phU_Enable = &phU_Enable1;
  mescHal.phV_Enable = &phV_Enable1;
  mescHal.phW_Enable = &phW_Enable1;
  //void (*invDisable)(void);
  mescHal.invEnable = &invEnableM1;
  mescHal.getRawADC = &getRawADC1;
  mescHal.getRawADCVph = &getRawADC0Vph;
  return &mescHal;
}




