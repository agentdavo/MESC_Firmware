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
void slowled(bool enable);
bool tim1_is_downcounting();

bool isKillSwitch();
//OI KILLSWITCH_GPIO->IDR & (0x01 << KILLSWITCH_IONO)

uint32_t RTOS_flash_clear(void * address, uint32_t len);
uint32_t RTOS_flash_start_write(void * address, void * data, uint32_t len);
uint32_t RTOS_flash_write(void * address, void * data, uint32_t len);
uint32_t RTOS_flash_end_write(void * address, void * data, uint32_t len);
uint32_t RTOS_flash_sector_address( uint32_t const index );
uint32_t RTOS_flash_sector_index(uint32_t const address);
uint32_t RTOS_flash_base_address(void);
uint32_t RTOS_flash_base_size(void);

struct port_str;

void putbuffer_uart(unsigned char *buf, unsigned int len, struct port_str * port);
//{
//  UART_HandleTypeDef *uart_handle = port->hw;
//
//
//  if(port->half_duplex){
//      uart_handle->Instance->CR1 &= ~USART_CR1_RE;
//      vTaskDelay(1);
//    }
//  HAL_UART_Transmit_DMA(uart_handle, buf, len);
//  //vTaskDelay(1);
//  while(uart_handle->gState == HAL_UART_STATE_BUSY_TX){
//      vTaskDelay(0);
//    }
//
//  if(port->half_duplex) uart_handle->Instance->CR1 |= USART_CR1_RE;
//  xSemaphoreGive(port->tx_semaphore);
//}

void uart_init(struct port_str * port);
//{
//  UART_HandleTypeDef *uart_handle = port->hw;
//
//  HAL_UART_MspInit(uart_handle);
//  if(port->half_duplex){
//      HAL_HalfDuplex_Init(uart_handle);
//    }
//  HAL_UART_Receive_DMA(uart_handle, port->rx_buffer, port->rx_buffer_size);
//  CLEAR_BIT(uart_handle->Instance->CR3, USART_CR3_EIE);
//}

uint32_t uart_get_write_pos(struct port_str * port);
//{
//  UART_HandleTypeDef *uart_handle = port->hw;
//  return ( ((uint32_t)port->rx_buffer_size - __HAL_DMA_GET_COUNTER(uart_handle->hdmarx)) & ((uint32_t)port->rx_buffer_size -1));
//}

void uart_msp_deinit(struct port_str * port);
//OI HAL_UART_MspDeInit(port->hw);

void putbuffer_usb(unsigned char *buf, unsigned int len, struct port_str * port);
//{
//#ifdef MESC_UART_USB
//  xSemaphoreTake(port->tx_semaphore, portMAX_DELAY);
//  while(CDC_Transmit_FS((uint8_t*)buf, len)== USBD_BUSY){
//      vTaskDelay(1);
//    }
//  xSemaphoreGive(port->tx_semaphore);
//#endif
//}

#define FLASH_STORAGE_PAGE 7

typedef float hardware_vars_t;  ///< Let's have all the hardware and everything in float for
                                ///> now, until we start running out of clock cycles?

typedef struct
{
  hardware_vars_t Imax;    ///< Max board voltage allowable
  hardware_vars_t Vmax;    ///< Max board voltage allowable
  hardware_vars_t Vmin;    ///< Min voltage at which we turn off the PWM to avoid
                           ///< brownouts, nastiness.
  hardware_vars_t Rshunt;  ///< Shunt resistance, ohms
  hardware_vars_t RVBT;    ///< Vbus top divider - Also for switch divider
  hardware_vars_t RVBB;    ///< Vbus bottom divider - Also for switch divider
  hardware_vars_t VBGain;  ///< =RVBB/(RVBB+RVBT);         //Resistor divider
                           ///< network gain (fractional)
  hardware_vars_t RIphPU;  ///< phase current pullup
  hardware_vars_t RIphSR;  ///< phase current series resistance
  hardware_vars_t OpGain;  ///< OpAmp gain, if external, or internal PGA
  hardware_vars_t Igain;   ///< e.g. Rshunt*OpGain*RIphPU/(RIphSR+RIphPU);    //network gain
                           ///< network*opamp gain - total gain before the current hits the
                           ///< ADC, might want this inverted to avoid using division?
  uint16_t RawCurrLim;     ///< Current limit that will trigger a software
                           ///< generated break from ADC. Actual current equal to
                           ///< (RawCurrLim-IMid)*3.3/4096/Gain/Rshunt //example
                           ///< (4096-2048)*3.3/(4096*16*0.001)= 103A
  uint16_t RawVoltLim;     ///< Voltage limit that will trigger a software
                           ///<  generated break from ADC. Actual voltage equal to
                           ///< RawVoltLim*3.3*Divider/4096            //
                           ///< example 2303*3.3/4096*(R1k5+R47k/R1K5)=60V
} hw_setup_s;

extern hw_setup_s g_hw_setup;  // TODO PROFILE
// _OR_
// void hw_setup_init( hw_setp_s * hw_setup );

typedef struct
{
  hardware_vars_t Rphase;    ///< float containing phase resistance in mOhms,
                             ///< populated by MEASURING if not already known;
  hardware_vars_t Lphase;    ///< float containing phase inductance in uH,
  hardware_vars_t Lqphase;   ///< range from very very low inductance high kV strong
                             ///< magnet BLDC motors to low kV weak magnet ones;
  hardware_vars_t Lqd_diff;  ///< Lq-Ld for using MTPA
  uint8_t uncertainty;       ///<
  float motor_flux;          ///<
  float measure_current;     ///<
  float measure_voltage;     ///<
} motor_s;

extern motor_s motor;  // TODO PROFILE

struct MESC_motor_;
typedef struct MESC_motor_ MESC_motor;

/**
 * Fills the parameters of the hardware struct, simplifies
 * some into useful overall gain values.
 * @param _motor
 */
void hw_init(MESC_motor* _motor);

/**
 *
 */
void setAWDVals();

/**
 *
 * @param _motor
 */
void getRawADC(MESC_motor* _motor);

/**
 *
 * @param _motor
 */
void getRawADCVph(MESC_motor* _motor);

/**
 *
 * @return
 */
uint32_t getFlashBaseAddress(void);

/**
 *
 * @return
 */
uint32_t getFlashBaseSize(void);

/**
 * Perform HW specific initialisation for MESCInit() before delay.
 * @param _motor
 */
void mesc_init_1(MESC_motor* _motor);

/**
 * Perform HW specific initialisation for MESCInit() after delay.
 * @param _motor
 */
void mesc_init_2(MESC_motor* _motor);

/**
 * Perform HW specific initialisation for MESCInit() after hw_init().
 * @param _motor
 */
void mesc_init_3(MESC_motor* _motor);

/*
Profile defaults

Temperature parameters
#define MESC_PROFILE_TEMP_R_F
#define MESC_PROFILE_TEMP_SCHEMA
#define MESC_PROFILE_TEMP_SH_BETA
#define MESC_PROFILE_TEMP_SH_R
#define MESC_PROFILE_TEMP_SH_R0
*/