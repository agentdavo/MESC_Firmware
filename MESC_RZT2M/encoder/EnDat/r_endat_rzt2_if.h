#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Definition of the mode command */
#define R_ENDAT_POS                 (0x07u)
#define R_ENDAT_MEM                 (0x0Eu)
#define R_ENDAT_RX_PARAM            (0x1Cu)
#define R_ENDAT_PARAM               (0x23u)
#define R_ENDAT_RESET               (0x2Au)
#define R_ENDAT_POS_ADD_DATA        (0x38u)
#define R_ENDAT_POS_MEM             (0x09u)
#define R_ENDAT_POS_RX_PARAM        (0x1Bu)
#define R_ENDAT_POS_PARAM           (0x24u)
#define R_ENDAT_POS_RX_ERR_RESET    (0x2Du)

/* Definition of transmission frequency */
#define R_ENDAT_FTCLK_16670         (0x3u)
#define R_ENDAT_FTCLK_8330          (0x6u)
#define R_ENDAT_FTCLK_4160          (0xBu)
#define R_ENDAT_FTCLK_4000          (0x8u)
#define R_ENDAT_FTCLK_2000          (0xCu)
#define R_ENDAT_FTCLK_1000          (0xDu)
#define R_ENDAT_FTCLK_200           (0xEu)
#define R_ENDAT_FTCLK_100           (0xFu)

/* Definition of watchdog timer */
#define R_ENDAT_WD_RANGE_US         (0x00u)
#define R_ENDAT_WD_RANGE_MS         (0x80u)

/* Definition of data Low period at the start of data transmission */
#define R_ENDAT_TST_HALF_TCLK       (0x00u)
#define R_ENDAT_TST_500NS           (0x01u)
#define R_ENDAT_TST_1US             (0x02u)
#define R_ENDAT_TST_1500NS          (0x03u)
#define R_ENDAT_TST_2US             (0x04u)
#define R_ENDAT_TST_4US             (0x05u)
#define R_ENDAT_TST_8US             (0x06u)
#define R_ENDAT_TST_10US            (0x07u)

/* Definition of the MRS code */
#define R_ENDAT_MRS_INFO1_NOP       (0x40u)
#define R_ENDAT_MRS_DIA             (0x41u)
#define R_ENDAT_MRS_POS2_LSB        (0x42u)
#define R_ENDAT_MRS_POS2_CENTER     (0x43u)
#define R_ENDAT_MRS_POS2_MSB        (0x44u)
#define R_ENDAT_MRS_MEM_LSB         (0x45u)
#define R_ENDAT_MRS_MEM_MSB         (0x46u)
#define R_ENDAT_MRS_MRS_CODE        (0x47u)
#define R_ENDAT_MRS_TEST_SMD        (0x48u)
#define R_ENDAT_MRS_TEST_LSB        (0x49u)
#define R_ENDAT_MRS_TEST_CENTER     (0x4Au)
#define R_ENDAT_MRS_TEST_MSB        (0x4Bu)
#define R_ENDAT_MRS_TEMP1           (0x4Cu)
#define R_ENDAT_MRS_TEMP2           (0x4Du)
#define R_ENDAT_MRS_ADD_SEN         (0x4Eu)
#define R_ENDAT_MRS_NOT_INFO1       (0x4Fu)
#define R_ENDAT_MRS_INFO2_NOP       (0x50u)
#define R_ENDAT_MRS_COM             (0x51u)
#define R_ENDAT_MRS_ACC             (0x52u)
#define R_ENDAT_MRS_COM_ACC         (0x53u)
#define R_ENDAT_MRS_LIM_POS         (0x54u)
#define R_ENDAT_MRS_LIM_POS_ACC     (0x55u)
#define R_ENDAT_MRS_ASY_POS_LSB     (0x56u)
#define R_ENDAT_MRS_ASY_POS_CENTER  (0x57u)
#define R_ENDAT_MRS_ASY_POS_MSB     (0x58u)
#define R_ENDAT_MRS_OPE_STA_ERR     (0x59u)
#define R_ENDAT_MRS_TIM_STA         (0x5Bu)
#define R_ENDAT_MRS_NOT_INFO2       (0x5Fu)
#define R_ENDAT_MRS_OPE_STAT        (0xB9u)
#define R_ENDAT_MRS_ENC_MANU1       (0xA1u)
#define R_ENDAT_MRS_ENC_MANU2       (0xA3u)
#define R_ENDAT_MRS_ENC_MANU3       (0xA5u)
#define R_ENDAT_MRS_OPE_PARAM       (0xA7u)
#define R_ENDAT_MRS_OEM1            (0xA9u)
#define R_ENDAT_MRS_OEM2            (0xABu)
#define R_ENDAT_MRS_OEM3            (0xADu)
#define R_ENDAT_MRS_OEM4            (0xAFu)
#define R_ENDAT_MRS_COMP_VAL1       (0xB1u)
#define R_ENDAT_MRS_COMP_VAL2       (0xB3u)
#define R_ENDAT_MRS_COMP_VAL3       (0xB5u)
#define R_ENDAT_MRS_COMP_VAL4       (0xB7u)
#define R_ENDAT_MRS_PARAM_ENDAT22   (0xBDu)
#define R_ENDAT_MRS_PARAM_SEC2      (0xBFu)
#define R_ENDAT_MRS_OPE_PARAM2      (0xBBu)

/* API error codes */
typedef enum endat_err_e
{
    ENDAT_SUCCESS = 0,        /* Success          */
    ENDAT_ERR_INVALID_ARG,    /* Invalid argument */
    ENDAT_ERR_BUSY,           /* Busy             */
    ENDAT_ERR_ACCESS,         /* Access error     */
    ENDAT_ERR_DRV             /* Driver error     */
} r_endat_err_t;

/* Command of R_ENDAT_Control function.*/
typedef enum endat_cmd_e
{
    ENDAT_CMD_REQ,            /* Request */
    ENDAT_CMD_POS_STOP
} r_endat_cmd_t;

/* Request send and receive results */
typedef enum r_endat_req_err_e
{
    ENDAT_REQ_SUCCESS   = 0,
    ENDAT_REQ_ERR,
} r_endat_req_err_t;

/* receive data */
typedef struct r_endat_data_s
{
    uint64_t pos;
    uint32_t add_datum1;
    uint32_t add_datum2;
} r_endat_data_t;

/* Encoder of status */
typedef struct r_endat_status_s
{
    bool busy;
    bool rm;
    bool wrn;
} r_endat_status_t;

/* Error of EnDat */
typedef struct r_endat_protocol_err_s
{
   bool err1;
   bool crc1;
   bool ftype1;
   bool ftype2;
   bool mrsadr;
   bool err2;
   bool crc3;
   bool crc2;
   bool wdg;
   bool ftype3;
   bool modeerr;
} r_endat_protocol_err_t;

/* Send and receive results */
typedef struct r_endat_result_s
{
    r_endat_req_err_t result;
    r_endat_data_t    data;
    r_endat_status_t  status;
} r_endat_result_t;

/* Declaration of the callback function */
typedef void (*r_endat_wait_cb_t)(void);
typedef void (*r_endat_isr_result_cb_t)(r_endat_result_t * presult, r_endat_protocol_err_t *perr);
typedef void (*r_endat_isr_rdst_cb_t)(void);

/* Watchdog set time */
typedef struct r_endat_watchdog_s
{
    uint8_t range;
    uint8_t time;
} r_endat_watchdog_t;

/* Initialization information of EnDat control unit */
typedef struct r_endat_info_s
{ 
    uint8_t             ftclk;
    bool                filter;
    bool                delay_comp;
    uint8_t             tst;
    r_endat_wait_cb_t   penc_init_reset_wait;
    r_endat_wait_cb_t   penc_init_mem_wait;
    r_endat_wait_cb_t   penc_init_pram_wait;
    r_endat_wait_cb_t   penc_init_cable_wait;
} r_endat_info_t;

/* Request information to be sent to the encoder */
typedef struct r_endat_req_s
{
    uint8_t                 mode_cmd;
    bool                    dt;
    uint8_t                 mrs;
    uint8_t                 addr;
    uint16_t                param_instruction;
    r_endat_watchdog_t      watchdog;
    bool                    elc;
    r_endat_isr_result_cb_t pisr_result;
    r_endat_isr_rdst_cb_t   pisr_rdst;
} r_endat_req_t;

uint32_t R_ENDAT_GetVersion(void);
r_endat_err_t R_ENDAT_Open(const int32_t id, r_endat_info_t* pinfo);
r_endat_err_t R_ENDAT_Control(const int32_t id, const r_endat_cmd_t cmd, void *const pbuf);
r_endat_err_t R_ENDAT_Close(const int32_t id);

