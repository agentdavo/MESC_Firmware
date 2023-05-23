#include <HAL/MESC_HAL.h>

#include <MESCfoc.h>

#include <comms_thread.h>

#include <stdio.h>
#define APP_ERR_PRINT printf

extern MESC_motor mtr[NUM_MOTORS];

void R_BSP_WarmStart(bsp_warm_start_event_t event)
{
  switch(event)
  {
    case BSP_WARM_START_RESET:
      break;
    case BSP_WARM_START_POST_CLOCK:
      break;
    case BSP_WARM_START_POST_C:
      R_IOPORT_Open(&g_ioport_ctrl, &g_bsp_pin_cfg);
      break;
    default:
      break;
  }
}

void uart0_callback(uart_callback_args_t* p_args)
{
  if (p_args->channel == 0)
  {
    // Handle the UART event
    switch (p_args->event)
    {
      case UART_EVENT_RX_COMPLETE:
      { // Receive complete
        break;
      }

      case UART_EVENT_TX_COMPLETE:
      { // Transmit complete
        break;
      }

      case UART_EVENT_RX_CHAR:
      { // Received a character
        break;
      }

      case UART_EVENT_TX_DATA_EMPTY:
      { // Last byte trasmitting
        break;
      }

      default:
        break;
    }
  }
}

static volatile spi_event_t g_spi2_event_flag;
void spi2_callback(spi_callback_args_t * p_args)
{
    g_spi2_event_flag = (SPI_EVENT_TRANSFER_COMPLETE == p_args->event) ? SPI_EVENT_TRANSFER_COMPLETE : SPI_EVENT_TRANSFER_ABORTED;
}

bool g_canfd0_write_complete = false;
bool g_canfd0_error = false;
void canfd0_callback(can_callback_args_t *p_args)
{
  switch (p_args->event)
    {
      case CAN_EVENT_TX_COMPLETE:
      case CAN_EVENT_TX_FIFO_EMPTY:
        {
          g_canfd0_write_complete = true;
          break;
        }
      case CAN_EVENT_RX_COMPLETE:
      case CAN_EVENT_ERR_WARNING:             // error warning event
      case CAN_EVENT_ERR_PASSIVE:             // error passive event
      case CAN_EVENT_ERR_BUS_OFF:             // error Bus Off event
      case CAN_EVENT_BUS_RECOVERY:            // bus recovery error event
      case CAN_EVENT_MAILBOX_MESSAGE_LOST:    // overwrite/overrun error event
      case CAN_EVENT_ERR_BUS_LOCK:            // bus lock detected (32 consecutive dominant bits).
      case CAN_EVENT_ERR_CHANNEL:             // channel error has occurred.
      case CAN_EVENT_TX_ABORTED:              // transmit abort event.
      case CAN_EVENT_ERR_GLOBAL:              // Global error has occurred.
      default:
        {
          g_canfd0_error = true;
          break;
        }
    }
}

bool g_canfd1_write_complete = false;
bool g_canfd1_error = false;
void canfd1_callback(can_callback_args_t *p_args)
{
  switch (p_args->event)
    {
      case CAN_EVENT_TX_COMPLETE:
      case CAN_EVENT_TX_FIFO_EMPTY:
        {
          g_canfd1_write_complete = true;
          break;
        }
      case CAN_EVENT_RX_COMPLETE:
      case CAN_EVENT_ERR_WARNING:             // error warning event
      case CAN_EVENT_ERR_PASSIVE:             // error passive event
      case CAN_EVENT_ERR_BUS_OFF:             // error Bus Off event
      case CAN_EVENT_BUS_RECOVERY:            // bus recovery error event
      case CAN_EVENT_MAILBOX_MESSAGE_LOST:    // overwrite/overrun error event
      case CAN_EVENT_ERR_BUS_LOCK:            // bus lock detected (32 consecutive dominant bits).
      case CAN_EVENT_ERR_CHANNEL:             // channel error has occurred.
      case CAN_EVENT_TX_ABORTED:              // transmit abort event.
      case CAN_EVENT_ERR_GLOBAL:              // global error has occurred.
      default:
        {
          g_canfd1_error = true;
          break;
        }
    }
}

// MAX17841B Automotive SPI Communication Interface

/* SPI Transactions Table p18 */
#define BMS_CLR_TX_BUF                  0x20
#define BMS_RX_RD_POINTER               0x91
#define BMS_RD_NXT_MSG                  0x93
#define BMS_WR_NXT_LD_Q0                0xB0
#define BMS_WR_NXT_LD_Q1                0xB2
#define BMS_WR_NXT_LD_Q2                0xB4
#define BMS_WR_NXT_LD_Q3                0xB6
#define BMS_WR_NXT_LD_Q4                0xB8
#define BMS_WR_NXT_LD_Q5                0xBA
#define BMS_WR_NXT_LD_Q6                0xBC
#define BMS_WR_LD_Q0                    0xC0
#define BMS_WR_LD_Q1                    0xC2
#define BMS_WR_LD_Q2                    0xC4
#define BMS_WR_LD_Q3                    0xC6
#define BMS_WR_LD_Q4                    0xC8
#define BMS_WR_LD_Q5                    0xCA
#define BMS_WR_LD_Q6                    0xCC
#define BMS_RD_LD_Q0                    0xC1
#define BMS_RD_LD_Q1                    0xC3
#define BMS_RD_LD_Q2                    0xC5
#define BMS_RD_LD_Q3                    0xC7
#define BMS_RD_LD_Q4                    0xC9
#define BMS_RD_LD_Q5                    0xCB
#define BMS_RD_LD_Q6                    0xCD
#define BMS_CLR_RX_BUF                  0xE0

/* Register Table p26 */
#define BMS_RD__RX_STATUS               0x01
#define BMS_RD__TX_STATUS               0x03
#define BMS_WR_RX_INTERRUPT_ENABLE      0x04
#define BMS_RD_RX_INTERRUPT_ENABLE      0x05
#define BMS_WR_TX_INTERRUPT_ENABLE      0x06
#define BMS_RD_TX_INTERRUPT_ENABLE      0x07
#define BMS_WR_RX_INTERRUPT_FLAGS       0x08
#define BMS_RD__RX_INTERRUPT_FLAGS      0x09
#define BMS_WR_TX_INTERRUPT_FLAGS       0x0A
#define BMS_RD_TX_INTERRUPT_FLAGS       0x0B
#define BMS_WR_CONFIGURATION_1          0xOC
#define BMS_RD_CONFIGURATION_1          0x0D
#define BMS_WR_CONFIGURATION_2          0x0E
#define BMS_RD_CONFIGURATION_2          0x0F
#define BMS_WR_CONFIGURATION_3          0x10
#define BMS_RD_CONFIGURATION_3          0x11
#define BMS_RD_FMEA                     0x13
#define BMS_RD_MODEL                    0x15
#define BMS_RD_VERSION                  0x17
#define BMS_RD_RX_BYTE                  0x19
#define BMS_RD_RX_SPACE                 0x1B
#define BMS_RD_TX_QUEUE_SELECTS         0x95
#define BMS_RD_RX_READ_POINTER          0x97
#define BMS_RD_RX_WRITE_POINTER         0x99
#define BMS_RD_RX_NEXT_MESSAGE          0x9B

fsp_err_t max17841_init(void)
{
  fsp_err_t status = FSP_SUCCESS;
  return status;
}

fsp_err_t max17841_deinit(void)
{
  fsp_err_t status = FSP_SUCCESS;
  return status;
}

void bms_wr_cmd(uint8_t cmd)
{
  //R_SPI_Write(&g_spi2_ctrl, &cmd, sizeof( cmd ), SPI_BIT_WIDTH_8_BITS);
  R_SPI_Write(&g_spi3_ctrl, &cmd, sizeof( cmd ), SPI_BIT_WIDTH_8_BITS);
}

void bms_wakeup_hello_all()
{
  // 1 Enable Keep-Alive mode (prior to the UART slave wake-up to prevent shutdown)
  bms_wr_cmd(0x10); // xxh. Write Configuration 3 register
  bms_wr_cmd(0x05); // xxh. Set keep - alive period to 160µs

  // 2 Enable Rx Interrupt flags for RX_Error and RX_Overflow
  bms_wr_cmd(0x04); // xxh. Write RX_Interrupt_Enable register
  bms_wr_cmd(0x88); // xxh. Set the RX_Error_INT_Enable and RX_Overflow_INT_Enable bits

  // 3 Clear receive buffer
  bms_wr_cmd(0xE0); // xxh. Clear receive buffer

  // 4 Wake-up UART slave devices (transmit preambles)
  bms_wr_cmd(0x0E); // xxh. Write Configuration 2 register
  bms_wr_cmd(0x30); // xxh. Enable Transmit Preambles mode

  // 5 Wait for all UART slave devices to wake up (poll RX_Busy_Status bit)
  bms_wr_cmd(0x01); // xxh. Read RX_Status register (RX_Busy_Status and RX_Empty_Status should be true)
  bms_wr_cmd(0xFF); // 21h. If RX_Status = 21h, continue. Otherwise, repeat transaction until true or timeout.

  // 6 End of UART slave device wake-up period
  bms_wr_cmd(0x0E); // xxh. Write Configuration 2 register
  bms_wr_cmd(0x10); // xxh. Disable Transmit Preambles mode

  // 7 Wait for null message to be received (poll RX_Empty_Status bit)
  bms_wr_cmd(0x01); // xxh. Read RX_Status register

  // 8 Clear transmit buffer
  bms_wr_cmd(0x20); // xxh. Clear transmit buffer

  // 9 Clear receive buffer
  bms_wr_cmd(0xE0); // xxh. Clear receive buffer

  // 10 Load the HELLOALL command sequence into the load queue
  bms_wr_cmd(0xC0); // xxh. WR_LD_Q SPI command byte (write the load queue)
  bms_wr_cmd(0x03); // xxh. Message length
  bms_wr_cmd(0x57); // xxh. HELLOALL command byte
  bms_wr_cmd(0x00); // xxh. Register address (0x00)
  bms_wr_cmd(0x00); // xxh. Initialization address of HELLOALL

  // 11 Verify contents of the load queue
  bms_wr_cmd(0xC1); // xxh. RD_LD_Q SPI command byte
  bms_wr_cmd(0xFF); // 03h. OK
  bms_wr_cmd(0xFF); // 57h. OK
  bms_wr_cmd(0xFF); // 00h. OK
  bms_wr_cmd(0xFF); // 00h. OK

  // 12 Transmit HELLOALL sequence
  bms_wr_cmd(0xB0); // xxh. WR_NXT_LD_Q SPI command byte (write the next load queue)

  // 13 Poll RX_Stop_Status bit
  bms_wr_cmd(0x01); // xxh. Read RX_Status register
  bms_wr_cmd(0xFF); // 12h. If RX_Status[1] is true, continue. If false, then repeat transaction until true.

  // 14 Service receive buffer. Read the HELLOALL message that propagated through the daisy-chain and was
  bms_wr_cmd(0x93); // xxh. RD_NXT_MSG SPI transaction
  bms_wr_cmd(0xFF); // 57h. Sent command byte (HELLOALL)
  bms_wr_cmd(0xFF); // 00h. Sent address = 00h
  bms_wr_cmd(0xFF); // 02h. Returned address = 02h

  // 15 Check for receive buffer errors
  bms_wr_cmd(0x09); // xxh. Read RX_Interrupt_Flags register
  bms_wr_cmd(0xFF); // 00h. If no errors, continue. Otherwise, clear and go to error routine.

}

void bms_wr_all_slaves(uint8_t reg, uint16_t data) {

  uint8_t cmd = 0x02; // WRITEALL command byte
  uint8_t data_lsb = (uint8_t)(data & 0xFF);
  uint8_t data_msb = (uint8_t)(data >> 8) & 0xFF;
  //uint8_t message[4] = {cmd, reg, data_lsb, data_msb};
  uint8_t message_pec = 0x00;

  // fsp_err_t err = FSP_SUCCESS;
  // err = R_CRC_Open(&g_crc_ctrl, &g_crc_cfg);
  // err = R_CRC_Calculate(&g_crc_ctrl, &message, &message_pec);

  // 1 Load the WRITEALL command sequence into the load queue
  bms_wr_cmd(0xC0); // xxh. WR_LD_Q SPI command byte
  bms_wr_cmd(0x06); // xxh. Message length = 6
  bms_wr_cmd(cmd);  // xxh. Command byte
  bms_wr_cmd(reg);  // xxh. Register address of the device
  bms_wr_cmd(data_lsb); // xxh. LS byte of register data to be written
  bms_wr_cmd(data_msb); // xxh. MS byte of register data to be written
  bms_wr_cmd(message_pec);  // xxh. PEC byte for 02h, 12h, B1h, B2h
  bms_wr_cmd(0x00); // xxh. Alive-counter byte (seed value = 0)

  // 2 Start transmitting the WRITEALL sequence from the transmit queue
  bms_wr_cmd(0xB0); // xxh. WR_NXT_LD_Q SPI command byte

  // 3 Check if a message has been received into the receive buffer
  bms_wr_cmd(0x01); // xxh. Read RX_Status register
  bms_wr_cmd(0xFF); // 12h. If RX_Status[1] is true, continue. Otherwise, repeat until true or timeout.

  // 4 Read receive buffer to verify the sent WRITEALL message
  bms_wr_cmd(0x93); // xxh. RD_NXT_MSG SPI
  bms_wr_cmd(0xFF); // 02h. Sent command byte (WRITEALL)
  bms_wr_cmd(0xFF); // 12h. Sent address
  bms_wr_cmd(0xFF); // B1h. Sent LS byte
  bms_wr_cmd(0xFF); // B2h. Sent MS byte
  bms_wr_cmd(0xFF); // C4h. Sent PEC
  bms_wr_cmd(0xFF); // 02h. Alive-counter byte (= sent seed + 2, if alive counter enabled)

  // 5 Check for receive buffer errors
  bms_wr_cmd(0x09); // xxh. Read RX_Interrupt_Flags register
  bms_wr_cmd(0xFF); // 00h. If no errors, continue. Otherwise, clear and go to error routine.
}

void bms_rd_all_slaves(uint8_t reg) {

  uint8_t cmd = 0x03; // READALL command byte
  uint8_t message[3] = {cmd, reg, 0x00};
  uint8_t message_pec = 0x00;

  // fsp_err_t err = FSP_SUCCESS;
  // err = R_CRC_Open(&g_crc_ctrl, &g_crc_cfg);
  // err = R_CRC_Calculate(&g_crc_ctrl, &message, &message_pec);

  // 1 Load the READALL command sequence into the load queue
  bms_wr_cmd(0xC0); // xxh. WR_NXT_LD_Q SPI command byte
  bms_wr_cmd(0x1D); // xxh. Message length (5 + 2 x n = 29)
  bms_wr_cmd(cmd);  // xxh. Command byte
  bms_wr_cmd(reg);  // xxh. Register address
  bms_wr_cmd(0x00); // xxh. Data-check byte (seed value = 00h)
  bms_wr_cmd(message_pec);  // xxh. PEC byte for bytes 03h, 12h, 00h
  bms_wr_cmd(0x00); // xxh. Alive-counter byte (seed value = 00h)

  // 2 Start transmitting the READALL sequence
  bms_wr_cmd(0xB0); // xxh. WR_NXT_LD_Q SPI command byte

  // 3 Check if a message has been received into the receive buffer
  bms_wr_cmd(0x01); // xxh. Read the RX_Status register
  bms_wr_cmd(0xFF); // 12h. If RX_Status[1] is true, continue. Otherwise, repeat until true or timeout.

  // 4 Read the receive buffer and verify that the device register data is what was written during the WRITEALL sequence
  bms_wr_cmd(0x93); // xxh. RD_NXT_MSG SPI command byte
  bms_wr_cmd(0xFF); // 03h. Sent command byte (READALL)
  bms_wr_cmd(0xFF); // 12h. Sent register address
  bms_wr_cmd(0xFF); // B1h. LS byte of device 1
  bms_wr_cmd(0xFF); // B2h. MS byte of device 1
  bms_wr_cmd(0xFF); // B1h. LS byte of device 0
  bms_wr_cmd(0xFF); // B2h. MS byte of device 0
  bms_wr_cmd(0x00); // 00h. Data-check byte (= 00h if all status bits have been cleared)
  bms_wr_cmd(0xFF); // 67h. PEC (for the previous 7 bytes)
  bms_wr_cmd(0xFF); // 02h. Alive-counter byte (= sent seed + 2, if alive counter is enabled)

  // 5 Check for receive buffer errors
  bms_wr_cmd(0x09); // xxh. Read RX_Interrupt_Flags register
  bms_wr_cmd(0xFF); // 00h. If no errors, continue. Otherwise, clear and go to error routine
}

void hal_entry(void)
{
  // TODO: add your own code here

}

#if 0
fsp_err_t rzt2m_fsp_init(void)
{
  fsp_err_t status = FSP_SUCCESS;

  //  status = usb_hal_init();
  //  if (FSP_SUCCESS != status)
  //  {
  //    usb_hal_deinit();
  //  }

  status = mtu3_hal_init();

  fsp_err_t err = FSP_SUCCESS;

  // DSMIF unit 0 & 1 initialize
  if (R_DSMIF_Open(&g_dsmif0_ctrl, &g_dsmif0_cfg) != FSP_SUCCESS)
  {
    APP_ERR_PRINT("** R_DSMIF_Open API for DSMIF0 failed ** \r\n");
  }

  if (R_DSMIF_Open(&g_dsmif1_ctrl, &g_dsmif1_cfg) != FSP_SUCCESS)
  {
    APP_ERR_PRINT("** R_DSMIF_Open API for DSMIF1 failed ** \r\n");
  }

  if (R_SCI_UART_Open(&g_uart0_ctrl, &g_uart0_cfg) != FSP_SUCCESS)
  {
    APP_ERR_PRINT("** R_SCI_UART_Open API for UART0 failed ** \r\n");
  }

  // Initializes CANFD 0
  if (R_CANFD_Open(&g_canfd0_ctrl, &g_canfd0_cfg) != FSP_SUCCESS)
  {
    APP_ERR_PRINT("** R_CANFD_Open API for CANFD0 failed ** \r\n");
  }

  // Initializes CANFD 1
  if (R_CANFD_Open(&g_canfd1_ctrl, &g_canfd1_cfg) != FSP_SUCCESS)
  {
    APP_ERR_PRINT("** R_CANFD_Open API for CANFD1 failed ** \r\n");
  }

  // Reset SPI event flags
  g_spi2_event_flag = 0;

  if (max17841_init() != FSP_SUCCESS)
  {
    max17841_deinit();
  }

  return status;
}
#endif

void R_IRQ1_isr(void)
{

}

void r_mtu_tgiv3_interrupt(void)
{
  MESC_PWM_IRQ_handler(&mtr[0]);
}

void r_mtu_tgiv6_interrupt(void)
{
  MESC_PWM_IRQ_handler(&mtr[1]);
}

//void gmac_isr_pmt(void);
//void gmac_isr_sbd(void);
//void ethsw_isr_intr(void);

void m_sci0_eri_interrupt(void)
{

}

void m_sci0_rxi_interrupt(void)
{

}

void m_sci0_txi_interrupt(void)
{

}

void m_sci0_tei_interrupt(void)
{

}

//void canfd_rx_fifo_isr(void);
//void canfd_error_isr(void);
//void canfd_channel_tx_isr(void);
//void spi_rxi_isr(void);
//void spi_txi_isr(void);
//void spi_eri_isr(void);
//void spi_tei_isr(void);
//void adc_scan_end_isr(void);

//void r_enc_ch0_int_isr(void)
//{
//
//}

//void r_enc_ch0_err_int_isr(void)
//{
//
//}

//void r_enc_ch1_int_isr(void)
//{
//
//}

//void r_enc_ch1_err_int_isr(void)
//{
//
//}

//TODO: Should be checked for RZT because sectors organization will be different
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
  __NOP();
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

void MotorInitStage0_0(MESC_motor* _motor)
{
#ifdef IC_TIMER
  MESC_IC_Init(IC_TIMER);
#endif

#ifdef USE_ENCODER
  _motor->hal->EncoderInit();
#endif

#if 0
// TODO: In addition to next comment, IMHO this should be done in port layer side not here
#ifdef STM32L4  // For some reason, ST have decided to have a different name for the L4 timer DBG freeze...
	DBGMCU->APB2FZ |= DBGMCU_APB2FZ_DBG_TIM1_STOP;
#else
	DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM1_STOP;
#endif

#ifdef KILLSWITCH_GPIO
	KILLSWITCH_GPIO->MODER &= ~(0b11<<(2*KILLSWITCH_IONO));
#endif

#ifdef INV_ENABLE_M1
	INV_ENABLE_M1->MODER |= 0x1<<(INV_ENABLE_M1_IONO*2);
	INV_ENABLE_M1->MODER &= ~(0x2<<(INV_ENABLE_M1_IONO*2));
#endif

  //enable cycle counter
//  DEMCR |= DEMCR_TRCENA;
//  DWT_CTRL |= CYCCNTENA;
#endif

  // TODO: According to my understanding it is instead killswitch???
  // Start the MTU3 Port Out Enable POE3
  if (R_POE3_Open(&g_mtu3_three_phase_poe_ctrl, &g_mtu3_three_phase_poe_cfg) != FSP_SUCCESS)
  {
    APP_ERR_PRINT("** R_POE3_Open API for POE3 failed ** \r\n");
  }

  // Close the MTU3 Port Out Enable POE3
  //err = R_POE3_Close(&g_mtu3_three_phase_poe_ctrl);
  //assert(FSP_SUCCESS == err);

  // TODO: Due to not clear architectured how it will be used
//  if (R_ELC_Open(&g_elc_ctrl, &g_elc_cfg) != FSP_SUCCESS)
//  {
//    APP_ERR_PRINT("** R_ELC_Open API for ELC failed ** \r\n");
//  }
//
//  if (R_ELC_Enable(&g_elc_ctrl) != FSP_SUCCESS)
//  {
//    APP_ERR_PRINT("** R_ELC_Enable API for ELC failed ** \r\n");
//  }
}

void MotorInitStage0_1(MESC_motor* _motor)
{
#if 0
#ifdef INV_ENABLE_M2
  INV_ENABLE_M2->MODER |= 0x1<<(INV_ENABLE_M2_IONO*2);
  INV_ENABLE_M2->MODER &= ~(0x2<<(INV_ENABLE_M2_IONO*2));
#endif
#endif
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

void MotorInitHardware0(MESC_motor* _motor)
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
  }  // 4000 is 96 counts away from ADC saturation, allow headroom for opamp not pulling rail:rail.
  g_hw_setup.RawVoltLim = (uint16_t)(4096.0f * (g_hw_setup.Vmax / 3.3f) * g_hw_setup.RVBB /
                                     (g_hw_setup.RVBB + g_hw_setup.RVBT));
  motor.Lphase = 0;
}

void MotorInitHardware1(MESC_motor* _motor)
{

}

void MotorInitStage3_0(MESC_motor* _motor)
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
  // Initializes MTU3_34 PWM
  fsp_err_t err = R_MTU3_THREE_PHASE_Open(&g_mtu3_m0_3ph_drv_ctrl, &g_mtu3_m0_3ph_drv_cfg);
  if (FSP_SUCCESS != err )
  {
    // TODO: IMHO it had better to get safe state of bridge and go to reset with some diagnostic of the next start
    // TODO: Do not forget about eeror code FSP_ERR_ALREADY_OPEN which shows that it is already open
    err = R_MTU3_THREE_PHASE_Close(&g_mtu3_m1_3ph_drv_ctrl);
    assert(FSP_SUCCESS == err);
    APP_ERR_PRINT("** R_MTU3_THREE_PHASE_Open API for mtu3_m0 failed ** \r\n");
    return;
  }

  // Start the MTU3_34 PWM timer
  err = R_MTU3_THREE_PHASE_Start(&g_mtu3_m0_3ph_drv_ctrl);
  if (FSP_SUCCESS != err)
  {
    err = R_MTU3_THREE_PHASE_Stop(&g_mtu3_m0_3ph_drv_ctrl);
    assert(FSP_SUCCESS == err);
    APP_ERR_PRINT("** R_MTU3_THREE_PHASE_Start API for mtu3_m0 failed ** \r\n");
    return;
  }
}

void MotorInitStage3_1(MESC_motor* _motor)
{
  // Initializes MTU3_67 PWM
  fsp_err_t err = R_MTU3_THREE_PHASE_Open(&g_mtu3_m1_3ph_drv_ctrl, &g_mtu3_m1_3ph_drv_cfg);
  if (FSP_SUCCESS != err)
  {
    // TODO: IMHO it had better to get safe state of bridge and go to reset with some diagnostic of the next start
    // TODO: Do not forget about eeror code FSP_ERR_ALREADY_OPEN which shows that it is already open
    err = R_MTU3_THREE_PHASE_Close(&g_mtu3_m1_3ph_drv_ctrl);
    assert(FSP_SUCCESS == err);
    APP_ERR_PRINT("** R_MTU3_THREE_PHASE_Open API for mtu3_m1 failed ** \r\n");
    return;
  }

  // Start the MTU3_67 PWM timer
  err = R_MTU3_THREE_PHASE_Start(&g_mtu3_m1_3ph_drv_ctrl);
  if (FSP_SUCCESS != err )
  {
    err = R_MTU3_THREE_PHASE_Stop(&g_mtu3_m1_3ph_drv_ctrl);
    assert(FSP_SUCCESS == err);
    err = R_MTU3_THREE_PHASE_Close(&g_mtu3_m1_3ph_drv_ctrl);
    assert(FSP_SUCCESS == err);
    APP_ERR_PRINT("** R_MTU3_THREE_PHASE_Start API for mtu3_m1 failed ** \r\n");
    return;
  }
}

void FastLed0(bool enabled)
{
#ifdef FASTLED
  if (enabled)
  {
    R_BSP_PinSet(BSP_IO_REGION_NOT_SAFE, M1_OT_LED);
  }
  else
  {
    R_BSP_PinClear(BSP_IO_REGION_NOT_SAFE, M1_OT_LED);
  }
#endif
}

void SlowLed0(bool enabled)
{
#ifdef SLOWLED
  if (enabled)
  {
    R_BSP_PinSet(BSP_IO_REGION_NOT_SAFE, M1_OC_LED);
  }
  else
  {
    R_BSP_PinClear(BSP_IO_REGION_NOT_SAFE, M1_OC_LED);
  }
#endif
}

void FastLed1(bool enabled)
{
#ifdef FASTLED
  if (enabled)
  {
    R_BSP_PinSet(BSP_IO_REGION_NOT_SAFE, M2_OT_LED);
  }
  else
  {
    R_BSP_PinClear(BSP_IO_REGION_NOT_SAFE, M2_OT_LED);
  }
#endif
}

void SlowLed1(bool enabled)
{
#ifdef SLOWLED
  if (enabled)
  {
    R_BSP_PinSet(BSP_IO_REGION_NOT_SAFE, M2_OC_LED);
  }
  else
  {
    R_BSP_PinClear(BSP_IO_REGION_NOT_SAFE, M2_OC_LED);
  }
#endif
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
  // MESC hyperLoop timer
  // TODO: hyperLoop loop timer should be init somewhere else
//  if (R_GPT_Open (&g_gpt0_hyperLoop_ctrl, &g_gpt0_hyperLoop_cfg) != FSP_SUCCESS)
//  {
//    APP_ERR_PRINT("** R_GPT_Open API for GPT0_HYPERLOOP failed ** \r\n");
//  }
//  if (R_GPT_Start (&g_gpt0_hyperLoop_ctrl) != FSP_SUCCESS)
//  {
//    APP_ERR_PRINT("** R_GPT_Start API for GPT0_HYPERLOOP failed ** \r\n");
//  }
  // TODO: Fast loop timer should be init somewhere else
  // Start the fastloop timer
//  if (R_GPT_Open (&g_gpt1_fastLoop_ctrl, &g_gpt1_fastLoop_cfg) != FSP_SUCCESS)
//  {
//    APP_ERR_PRINT("** R_GPT_Open API for GPT1_FASTLOOP failed ** \r\n");
//  }
//  if (R_GPT_Start (&g_gpt1_fastLoop_ctrl) != FSP_SUCCESS)
//  {
//    APP_ERR_PRINT("** R_GPT_Start API for GPT1_FASTLOOP failed ** \r\n");
//  }

  // Start the slowloop timer
  if (R_GPT_Open(&g_gpt2_slowLoop_ctrl, &g_gpt2_slowLoop_cfg) != FSP_SUCCESS)
  {
    APP_ERR_PRINT("** R_GPT_Open API for GPT2_SLOWLOOP failed ** \r\n");
  }
  if (R_GPT_Start(&g_gpt2_slowLoop_ctrl) != FSP_SUCCESS )
  {
    APP_ERR_PRINT("** R_GPT_Start API for GPT2_SLOWLOOP failed ** \r\n");
  }
}

void SlowTimerStart1()
{
  // TODO: Do nothing slow loop common for both motors
}

static volatile spi_event_t g_spi0_event_flag;
void g_spi0_callback(spi_callback_args_t* p_args)
{
  g_spi0_event_flag = (SPI_EVENT_TRANSFER_COMPLETE == p_args->event)
                          ? SPI_EVENT_TRANSFER_COMPLETE
                          : SPI_EVENT_TRANSFER_ABORTED;
}

void EncoderInit0()
{
  g_spi0_event_flag = 0;
  if (R_SPI_Open(&g_spi0_ctrl, &g_spi0_cfg) != FSP_SUCCESS)
  {
    while(1) {};
  }
  // TODO: Setup chip select gpio
}

static volatile spi_event_t g_spi3_event_flag;
void spi3_callback(spi_callback_args_t* p_args)
{
  g_spi3_event_flag = (SPI_EVENT_TRANSFER_COMPLETE == p_args->event)
                          ? SPI_EVENT_TRANSFER_COMPLETE
                          : SPI_EVENT_TRANSFER_ABORTED;
}

void EncoderInit1()
{
  g_spi3_event_flag = 0;
  if (R_SPI_Open(&g_spi3_ctrl, &g_spi3_cfg) != FSP_SUCCESS)
  {
    while(1) {};
  }
  // TODO: Setup chip select gpio
}

uint16_t EncoderAngle0()
{
#ifdef USE_ENCODER
  struct __attribute__((__packed__)) SamplePacket
  {
    struct
    {
      uint8_t crc;
      uint8_t STAT_RESP;  // Should be 0xF_?
    } safetyword;
    uint16_t angle;
    int16_t speed;
    uint16_t revolutions;
  };
  typedef struct SamplePacket SamplePacket;
  static SamplePacket pkt;
  uint16_t const len = sizeof(pkt) / sizeof(uint16_t);
  uint16_t reg = (UINT16_C(1) << 15)     /* RW=Read */
                 | (UINT16_C(0x0) << 11) /* Lock */
                 | (UINT16_C(0x0) << 10) /* UPD=Buffer */
                 | (UINT16_C(0x02) << 4) /* ADDR */
                 | (len - 1);            /* ND */
  //OI HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
  R_SPI_Write(&g_spi0_ctrl, (uint8_t*)&reg, 1, SPI_BIT_WIDTH_8_BITS); //OI HAL_SPI_Transmit(&hspi3, (uint8_t*)&reg, 1, 1000);
  R_SPI_Read(&g_spi0_ctrl, (uint8_t*)&pkt, len, SPI_BIT_WIDTH_8_BITS); //OI HAL_SPI_Receive(&hspi3, (uint8_t*)&pkt, len, 1000);
  pkt.revolutions = pkt.revolutions & 0b0000000111111111;
  return pkt.angle & 0x7fff;
#else
  return 0;
#endif
}

uint16_t EncoderAngle1()
{
#ifdef USE_ENCODER
  struct __attribute__((__packed__)) SamplePacket
  {
    struct
    {
      uint8_t crc;
      uint8_t STAT_RESP;  // Should be 0xF_?
    } safetyword;
    uint16_t angle;
    int16_t speed;
    uint16_t revolutions;
  };
  typedef struct SamplePacket SamplePacket;
  static SamplePacket pkt;
  uint16_t const len = sizeof(pkt) / sizeof(uint16_t);
  uint16_t reg = (UINT16_C(1) << 15)     /* RW=Read */
                 | (UINT16_C(0x0) << 11) /* Lock */
                 | (UINT16_C(0x0) << 10) /* UPD=Buffer */
                 | (UINT16_C(0x02) << 4) /* ADDR */
                 | (len - 1);            /* ND */
  //OI HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
  R_SPI_Write(&g_spi3_ctrl, (uint8_t*)&reg, 1, SPI_BIT_WIDTH_8_BITS); //OI HAL_SPI_Transmit(&hspi3, (uint8_t*)&reg, 1, 1000);
  R_SPI_Read(&g_spi3_ctrl, (uint8_t*)&pkt, len, SPI_BIT_WIDTH_8_BITS); //OI HAL_SPI_Receive(&hspi3, (uint8_t*)&pkt, len, 1000);
  pkt.revolutions = pkt.revolutions & 0b0000000111111111;
  return pkt.angle & 0x7fff;
#else
  return 0;
#endif
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
  if (!enabled)
  {
    if (R_POE3_OutputDisable(&g_mtu3_three_phase_poe_ctrl) != FSP_SUCCESS)
    {

    }
  }
  else
  {
    if (R_POE3_Reset(&g_mtu3_three_phase_poe_ctrl) != FSP_SUCCESS)
    {

    }
  }
}

void invEnableM1(bool enabled)
{
#ifdef INV_ENABLE_M2
  INV_ENABLE_M2->BSRR = enabled ? INV_ENABLE_M2_IO : INV_ENABLE_M2_IO << 16U;  //Write the inverter enable pin high/low
#endif
  if (!enabled)
  {
    if (R_POE3_OutputDisable(&g_mtu3_three_phase_poe_ctrl) != FSP_SUCCESS)
    {

    }
  }
  else
  {
    if (R_POE3_Reset(&g_mtu3_three_phase_poe_ctrl) != FSP_SUCCESS)
    {

    }
  }
}

bool tim1_is_downcounting()
{
  return 0;//OI htim1.Instance->CR1 & TIM_CR1_DIR;
}

void GenerateBreakAll(struct MESC_motor_* _motor)
{
  //  for (int i = 0; i < NUM_MOTORS; i++)
  //  {
  //    mtr[i].hal->invEnable(false);
  //  }
  for (int i = 0; i < NUM_MOTORS; i++)
    {
      generateBreak(&mtr[i]);
    }
}

//Defines
#define ADC_CHANNEL0	(1U << 0U)
#define ADC_CHANNEL1	(1U << 1U)
#define ADC_CHANNEL2	(1U << 2U)
#define ADC_CHANNEL3	(1U << 3U)
#define ADC_CHANNEL4	(1U << 4U)
#define ADC_CHANNEL5	(1U << 5U)
#define ADC_CHANNEL6	(1U << 6U)
#define ADC_CHANNEL7	(1U << 7U)
#define ADC_CHANNEL8	(1U << 8U)
#define ADC_CHANNEL9	(1U << 9U)
#define ADC_CHANNEL10	(1U << 10U)
#define ADC_CHANNEL11	(1U << 11U)

// m1 adc macros
#define M1_VOL_B    ADC_CHANNEL_0
#define M1_VOL_U    ADC_CHANNEL_1
#define M1_VOL_V    ADC_CHANNEL_2
#define M1_VOL_W    ADC_CHANNEL_3
#define M1_CUR_U    ADC_CHANNEL_4
#define M1_CUR_V    ADC_CHANNEL_5
#define M1_CUR_W    ADC_CHANNEL_6
#define M1_MOST_U   ADC_CHANNEL_7
#define M1_MOST_V   ADC_CHANNEL_8
#define M1_MOST_W   ADC_CHANNEL_9
#define M1_THROT    ADC_CHANNEL_10
#define M1_TEMP     ADC_CHANNEL_11

// m1 adc macros
#define M2_VOL_B    ADC_CHANNEL_0
#define M2_VOL_U    ADC_CHANNEL_1
#define M2_VOL_V    ADC_CHANNEL_2
#define M2_VOL_W    ADC_CHANNEL_3
#define M2_CUR_U    ADC_CHANNEL_4
#define M2_CUR_V    ADC_CHANNEL_5
#define M2_CUR_W    ADC_CHANNEL_6
#define M2_MOST_U   ADC_CHANNEL_7
#define M2_MOST_V   ADC_CHANNEL_8
#define M2_MOST_W   ADC_CHANNEL_9
#define M2_THROT    ADC_CHANNEL_10
#define M2_TEMP     ADC_CHANNEL_11

#define ADC_RESOLUTION_12_BIT  0 ///< 12 bit resolution
#define ADC_RESOLUTION_10_BIT  1 ///< 10 bit resolution
#define ADC_RESOLUTION_8_BIT   2 ///< 8 bit resolution
#define ADC_RESOLUTION_14_BIT  3 ///< 14 bit resolution
#define ADC_RESOLUTION_16_BIT  4 ///< 16 bit resolution
#define ADC_RESOLUTION_24_BIT  5 ///< 24 bit resolution

#define ADC_UINT0	0
#define ADC_UINT1	1

volatile bool g_adc0_err_flag = false;

volatile bool g_adc1_err_flag = false;

void adc_window_compare_isr(void);

#if 0
static void adc_hal_window_compare_callback(uint8_t adc_unit,
                                            uint16_t adc_channels0,
                                            uint16_t adc_channels1,
                                            uint8_t window0,
                                            uint8_t window1)
{
  if (1 == adc_unit)
  {
    if(adc_channels1&ADC_CHANNEL4)
    {
      handleError(&mtr[0], ERROR_ADC_OUT_OF_RANGE_IA);
    }
    else if(adc_channels1&ADC_CHANNEL5)
    {
      handleError(&mtr[0], ERROR_ADC_OUT_OF_RANGE_IB);
    }
    else if(adc_channels1&ADC_CHANNEL6)
    {
      handleError(&mtr[0], ERROR_ADC_OUT_OF_RANGE_IC);
    }
    else
    {
      // Do nothing
    }
  }
  else
  {
    // Do nothing
  }
}

// TODO: MAy be will not be used for now
void adc_window_compare_isr(void)
{
  // Save context if RTOS is used
  FSP_CONTEXT_SAVE;

  IRQn_Type irq = R_FSP_CurrentIrqGet();
  (void)irq;
  //adc_instance_ctrl_t * p_instance_ctrl = (adc_instance_ctrl_t *) R_FSP_IsrContextGet(irq);
  //adc_extended_cfg_t  * p_extend        = (adc_extended_cfg_t *) p_instance_ctrl->p_cfg->p_extend;
  uint8_t adc_unit=(uint8_t) 0x00;
  uint16_t adc_channels0=(uint16_t) 0x00;
  uint16_t adc_channels1=(uint16_t) 0x00;
  uint8_t window0=(uint8_t) 0x00;
  uint8_t window1=(uint8_t) 0x00;
  /*Check whether compare match interrupt was from Window A or Window B, for which unit and channels*/
  if(0==g_adc1_ctrl.p_reg->ADWINMON_b.MONCMPA)
  {
    /* Get all Window A status registers */
    adc_channels1 = g_adc1_ctrl.p_reg->ADCMPSR0;
    /* Clear the status flag */
    g_adc1_ctrl.p_reg->ADCMPSR0 = (uint16_t) 0x00;
    adc_unit|=0x10;
    window1|=0x01;
  }
  else
    {
      /*If Window B is needed */
    }

  if(0==g_adc0_ctrl.p_reg->ADWINMON_b.MONCMPA)
    {
      /* Get all Window A status registers */
      adc_channels0 = g_adc0_ctrl.p_reg->ADCMPSR0;
      /* Clear the status flag */
      g_adc0_ctrl.p_reg->ADCMPSR0 = (uint16_t) 0x00;
      adc_unit|=0x01;
      window0|=0x01;
    }
  else
    {
      /*If Window B is needed */
    }

  adc_hal_window_compare_callback(adc_unit,adc_channels0,adc_channels1,window0,window1);

  //R_BSP_IrqStatusClear(irq);

  // Restore context if RTOS is used
  FSP_CONTEXT_RESTORE;
}
#endif

#define CHECK(x) while(x != FSP_SUCCESS);

uint16_t R_ADC_Read_(adc_ctrl_t* p_ctrl, adc_channel_t const reg_id)
{
  uint16_t adc_data = 0;
  CHECK(R_ADC_Read(p_ctrl, reg_id, &adc_data));
  return adc_data;
}

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

volatile bool g_adc0_scan_complete_flag = false;
void g_adc0_callback(adc_callback_args_t *p_args)
{
  FSP_PARAMETER_NOT_USED(p_args);
  g_adc0_scan_complete_flag = true;
}

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

volatile bool g_adc1_scan_complete_flag = false;
void g_adc1_callback(adc_callback_args_t *p_args)
{
  FSP_PARAMETER_NOT_USED(p_args);
  g_adc1_scan_complete_flag = true;
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

void AdcStart0()
{
  //#ifdef SOFTWARE_ADC_REGULAR
  //       HAL_ADC_Start(&hadc1); //Try to eliminate the HAL call, slow and inefficient. Leaving this here for now.
  //        //hadc1.Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
  //#endif

  fsp_err_t err = FSP_SUCCESS;
  // Intialize the configurations for ADC units

  // A/D Compare Function Control Register Settings
  g_adc1_ctrl.p_reg->ADCMPCR=0x0000; // Reset the full register
  g_adc1_ctrl.p_reg->ADCMPCR_b.CMPAE = 1; // Permit operation in Window A
  g_adc1_ctrl.p_reg->ADCMPCR_b.WCMPE = 1; // Enable window function
  g_adc1_ctrl.p_reg->ADCMPCR_b.CMPAIE = 1; // Enable window A compare match interrupt
  // A/D Compare Function Control Register Settings

  // Select channels for Compare match
  g_adc1_ctrl.p_reg->ADCMPANSR0=0x0E00; // Channel 4,5,6 for phases currents
  // Select channels for Compare match

  // Interrupt will be generated when converted value is greater than lower threshold and less than higher threshold
  g_adc1_ctrl.p_reg->ADCMPLR0=0x00;
  // Interrupt will be generated when converted value is greater than lower threshold and less than higher threshold

  // Set Lower threshold
  g_adc1_ctrl.p_reg->ADCMPDR0=48;
  // Set higher threshold
  g_adc1_ctrl.p_reg->ADCMPDR1=4048;


  // Initialize the ADC0 and ADC1 peripherals
  //    if (R_ADC_Open (&g_adc0_ctrl, &g_adc0_cfg) != FSP_SUCCESS)
  //    {
  //        APP_ERR_PRINT("** R_ADC_Open API for ADC0 failed ** \r\n");
  //        return;
  //    }

  if (R_ADC_Open (&g_adc1_ctrl, &g_adc1_cfg) != FSP_SUCCESS)
  {
    APP_ERR_PRINT("** R_ADC_Open API for ADC1 failed ** \r\n");
    return;
  }

  // Configure ADC scanning groups
  //    if (R_ADC_ScanCfg (&g_adc0_ctrl, &g_adc0_channel_cfg) != FSP_SUCCESS)
  //    {
  //        APP_ERR_PRINT("** R_ADC_ScanCfg API for ADC0 failed ** \r\n");
  //        return;
  //    }

  if (R_ADC_ScanCfg (&g_adc1_ctrl, &g_adc1_channel_cfg) != FSP_SUCCESS)
  {
    APP_ERR_PRINT("** R_ADC_ScanCfg API for ADC1 failed ** \r\n");
    return;
  }

  // Enable scan triggering from ELC events.
  //    err = R_ADC_ScanStart (&g_adc0_ctrl);
  //    if (FSP_SUCCESS != err )
  //    {
  //        APP_ERR_PRINT("** R_ADC_ScanStart API for ADC0 failed ** \r\n");
  //        return err;
  //    }

  if (R_ADC_ScanStart(&g_adc1_ctrl) != FSP_SUCCESS)
  {
    APP_ERR_PRINT("** R_ADC_ScanStart API for ADC1 failed ** \r\n");
    return;
  }

  APP_ERR_PRINT("** ADC0 & ADC1 module init successful ** \r\n");
}

void AdcStart1()
{
  //#ifdef SOFTWARE_ADC_REGULAR
  //       HAL_ADC_Start(&hadc1); //Try to eliminate the HAL call, slow and inefficient. Leaving this here for now.
  //        //hadc1.Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
  //#endif

  fsp_err_t err = FSP_SUCCESS;
  // Intialize the configurations for ADC units

  // A/D Compare Function Control Register Settings
  g_adc1_ctrl.p_reg->ADCMPCR=0x0000; // Reset the full register
  g_adc1_ctrl.p_reg->ADCMPCR_b.CMPAE = 1; // Permit operation in Window A
  g_adc1_ctrl.p_reg->ADCMPCR_b.WCMPE = 1; // Enable window function
  g_adc1_ctrl.p_reg->ADCMPCR_b.CMPAIE = 1; // Enable window A compare match interrupt
  // A/D Compare Function Control Register Settings

  // Select channels for Compare match
  g_adc1_ctrl.p_reg->ADCMPANSR0=0x0E00; // Channel 4,5,6 for phases currents
  // Select channels for Compare match

  // Interrupt will be generated when converted value is greater than lower threshold and less than higher threshold
  g_adc1_ctrl.p_reg->ADCMPLR0=0x00;
  // Interrupt will be generated when converted value is greater than lower threshold and less than higher threshold

  // Set Lower threshold
  g_adc1_ctrl.p_reg->ADCMPDR0=48;
  // Set higher threshold
  g_adc1_ctrl.p_reg->ADCMPDR1=4048;


  // Initialize the ADC0 and ADC1 peripherals
  //    if (R_ADC_Open (&g_adc0_ctrl, &g_adc0_cfg) != FSP_SUCCESS)
  //    {
  //        APP_ERR_PRINT("** R_ADC_Open API for ADC0 failed ** \r\n");
  //        return;
  //    }

  if (R_ADC_Open (&g_adc1_ctrl, &g_adc1_cfg) != FSP_SUCCESS)
  {
    APP_ERR_PRINT("** R_ADC_Open API for ADC1 failed ** \r\n");
    return;
  }

  /* Configure ADC scanning groups */
  //    err = R_ADC_ScanCfg (&g_adc0_ctrl, &g_adc0_channel_cfg);
  //    if (FSP_SUCCESS != err )
  //    {
  //        APP_ERR_PRINT("** R_ADC_ScanCfg API for ADC0 failed ** \r\n");
  //        adc_hal_deinit();
  //        return err;
  //    }

  if (R_ADC_ScanCfg (&g_adc1_ctrl, &g_adc1_channel_cfg) != FSP_SUCCESS)
  {
    APP_ERR_PRINT("** R_ADC_ScanCfg API for ADC1 failed ** \r\n");
    return;
  }

  // Enable scan triggering from ELC events.
  //    if (R_ADC_ScanStart (&g_adc0_ctrl) != FSP_SUCCESS)
  //    {
  //        APP_ERR_PRINT("** R_ADC_ScanStart API for ADC0 failed ** \r\n");
  //        return;
  //    }

  if (R_ADC_ScanStart (&g_adc1_ctrl) != FSP_SUCCESS)
  {
    APP_ERR_PRINT("** R_ADC_ScanStart API for ADC1 failed ** \r\n");
    return;
  }
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
  mescHal.MotorInitHardware = &MotorInitHardware0;
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
  mescHal.GenerateBreakAll = &GenerateBreakAll;
  mescHal.phU_Enable = &phU_Enable0;
  mescHal.phV_Enable = &phV_Enable0;
  mescHal.phW_Enable = &phW_Enable0;
  //void (*invDisable)(void);
  mescHal.invEnable = &invEnableM0;
  mescHal.getRawADC = &getRawADC0;
  mescHal.getRawADCVph = &getRawADC0Vph;
  mescHal.AdcStart = &AdcStart0;
  return &mescHal;
}

MESC_hal* getHalForMotor1()
{
  static MESC_hal mescHal;
  mescHal.MotorInitStage0 = &MotorInitStage0_1;
  mescHal.MotorInitStage1 = &MotorInitStage1_1;
  mescHal.MotorInitStage2 = &MotorInitStage2_1;
  mescHal.MotorInitHardware = &MotorInitHardware1;
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
  mescHal.GenerateBreakAll = &GenerateBreakAll;
  mescHal.phU_Enable = &phU_Enable1;
  mescHal.phV_Enable = &phV_Enable1;
  mescHal.phW_Enable = &phW_Enable1;
  //void (*invDisable)(void);
  mescHal.invEnable = &invEnableM1;
  mescHal.getRawADC = &getRawADC1;
  mescHal.getRawADCVph = &getRawADC0Vph;
  mescHal.AdcStart = &AdcStart1;
  return &mescHal;
}

// TODO: Temporary hack for resolving issues for RZT
float sqrtf(float x)
{
  while(1) {};
  return 0;
}


