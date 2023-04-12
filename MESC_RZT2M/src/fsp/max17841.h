#ifndef MAX17841_H_
#define MAX17841_H_

#include "comms_thread.h"

fsp_err_t max17841_init(void);
fsp_err_t max17841_deinit(void);

/* MAX17841B Automotive SPI Communication Interface */

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

// see table 10 p9 for spi sequencing
void bms_wakeup_hello_all();
void bms_wr_all_slaves(uint8_t reg, uint16_t data);
void bms_rd_all_slaves(uint8_t reg);
void bms_wr_cmd(uint8_t cmd);
// void bms_clear_status_fmea();
// void bms_enable_measurement();
// void bms_get_cell_voltages();
// void bms_get_block_voltages();

#endif /* MAX17841_H_ */
