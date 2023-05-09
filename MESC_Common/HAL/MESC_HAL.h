#pragma once

#include <MESC_HAL_Config.h>

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

void ssd1306_WriteData(uint8_t* buffer, size_t buff_size);
//{
//  HAL_I2C_Mem_Write(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 0x40, 1, buffer, buff_size, HAL_MAX_DELAY);
//}

uint8_t ssd1306_WriteCommand(uint8_t command);
//{
//  return HAL_I2C_Mem_Write(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 0x00, 1, &command, 1, 10);
//}

void UsbDeviceInit();
//OI MX_USB_DEVICE_Init(); // TODO: Should be implemented for RZT

// TODO: May be should be moved somewhere
//int getHallState();

void Delay(uint32_t ms);
void SystemNOP();
uint32_t SystemGetTick();
uint32_t SystemHCLKFreq();

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

//void fastled(bool enable);
//void slowled(bool enable);
//void ToggleLed();
//OI HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin); // TODO: Should be implemented for RZT

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

uint32_t getFlashBaseAddress(void);
uint32_t getFlashBaseSize(void);
bool flash_Unlock();
bool flash_Lock();
bool flash_Program(uint32_t addr, const char* src);