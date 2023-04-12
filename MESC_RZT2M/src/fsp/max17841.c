#include "hal_data.h"

#include "max17841.h"

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

void bms_wakeup_hello_all() {

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
	uint8_t message[4] = {cmd, reg, data_lsb, data_msb};
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

void bms_wr_cmd(uint8_t cmd)
{
    R_SPI_Write(&g_spi2_ctrl, &cmd, sizeof( cmd ), SPI_BIT_WIDTH_8_BITS);
	R_SPI_Write(&g_spi3_ctrl, &cmd, sizeof( cmd ), SPI_BIT_WIDTH_8_BITS);
}
