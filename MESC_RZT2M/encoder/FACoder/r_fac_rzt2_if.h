#pragma once

#include <stdint.h>
#include <stdbool.h>

/* API error codes */
typedef enum r_fac_err_e
{
    R_FAC_SUCCESS = 0,        /* Success          */
    R_FAC_ERR_INVALID_ARG,    /* Invalid argument */
    R_FAC_ERR_BUSY,           /* Busy             */
    R_FAC_ERR_ACCESS          /* Access error     */
} r_fac_err_t;

/* Command of R_FAC_Control function.*/
typedef enum r_fac_cmd_e
{
    R_FAC_CMD_REQ,            /* Request */
    R_FAC_CMD_E2PROM,         /* E2PROM access */
    R_FAC_CMD_ELCTIMER,       /* ELC Timer operation */
    R_FAC_CMD_ELCSTOP         /* Stop ELC Timer */
} r_fac_cmd_t;

/* FACoder control error */
typedef enum r_fac_rx_err_e
{
    R_FAC_RX_SUCCESS = 0,    /* FA-CODER control success */
    R_FAC_RX_ERR          /* FA-CODER control error (RX) */
} r_fac_rx_err_t;

typedef enum r_fac_e2prom_dir_e
{
    R_FAC_E2PROM_READ,
    R_FAC_E2PROM_WRITE
} r_fac_e2prom_dir_t;


typedef struct r_fac_result_s
{
    r_fac_rx_err_t result;
    bool rse;
    bool ide;
    bool ebusy;
    uint8_t rxid;
    uint8_t rxidp;
    uint8_t rxsfic;
    uint8_t rxsfea;
    uint8_t rxsfca;
    uint8_t crc;
    bool conte;
    bool crce;
    bool fome;
    bool sfome;
    bool timote;
    bool rxedfe;
    bool rxadfe;
    bool dfovfe;
} r_fac_result_t;

typedef void (*r_fac_req_result_cb_t)(r_fac_result_t * presult, uint8_t * prxdf);
typedef void (*r_fac_e2prom_result_cb_t)(r_fac_result_t * presult, uint8_t adf, uint8_t edf);

typedef struct r_fac_req_s
{
    uint8_t txid;
    uint8_t dfnum;
    uint16_t timotn;
    r_fac_req_result_cb_t presult_cb;
} r_fac_req_t;

typedef struct r_fac_e2prom_s
{
    uint16_t timotn;
    uint8_t adr;
    uint8_t data;
    r_fac_e2prom_dir_t dir;
    r_fac_e2prom_result_cb_t presult_cb;
} r_fac_e2prom_data_t;

r_fac_err_t R_FAC_Open(const int32_t id);
r_fac_err_t R_FAC_Close(const int32_t id);
r_fac_err_t R_FAC_Control(const int32_t id, const r_fac_cmd_t cmd, void *const pbuf);
uint32_t R_FAC_GetVersion(void);

