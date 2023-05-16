#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Driver type */
#define R_A_AS_A_FORMAT                 (0u)

/* Connection type */
#define R_A_AS_ONE_FOR_ONE              (0u)
#define R_A_AS_BUS                      (1u)

/* Bit rate */
#define R_A_AS_2500KBPS                 (0u)
#define R_A_AS_4MBPS                    (1u)
#define R_A_AS_6670KBPS                 (2u)
#define R_A_AS_8MBPS                    (3u)

/* Delay of the cable length */
#define R_A_AS_IFMG_2500KBPS            (0x001E)
#define R_A_AS_IFMG_4MBPS               (0x0014)
#define R_A_AS_IFMG_6670KBPS            (0x0014)
#define R_A_AS_IFMG_8MBPS               (0x000E)

/* R_A_AS_Control function control commands */
#define R_A_AS_CMD_SET_PARAM            (0xAF000000u)
#define R_A_AS_CMD_ELC_DISABLE          (0xAF000002u)
#define R_A_AS_CMD_TX_TRG               (0xAF000003u)
#define R_A_AS_CMD_TX_ELC               (0xAF000005u)

/* Encoder address */
#define R_A_AS_ECN1                     (0u)
#define R_A_AS_ECN2                     (1u)
#define R_A_AS_ECN3                     (2u)
#define R_A_AS_ECN4                     (3u)
#define R_A_AS_ECN5                     (4u)
#define R_A_AS_ECN6                     (5u)
#define R_A_AS_ECN7                     (6u)
#define R_A_AS_ECN8                     (7u)

/* Command */
#define R_A_AS_CDF0                     (0u)
#define R_A_AS_CDF1                     (1u)
#define R_A_AS_CDF2                     (2u)
#define R_A_AS_CDF3                     (3u)
#define R_A_AS_CDF4                     (4u)
#define R_A_AS_CDF5                     (5u)
#define R_A_AS_CDF6                     (6u)
#define R_A_AS_CDF7                     (7u)
#define R_A_AS_CDF8                     (8u)
#define R_A_AS_CDF9                     (9u)
#define R_A_AS_CDF10                    (10u)
#define R_A_AS_CDF11                    (11u)
#define R_A_AS_CDF12                    (12u)
#define R_A_AS_CDF13                    (13u)
#define R_A_AS_CDF14                    (14u)
#define R_A_AS_CDF15                    (15u)
#define R_A_AS_CDF16                    (16u)
#define R_A_AS_CDF17                    (17u)
#define R_A_AS_CDF18                    (18u)
#define R_A_AS_CDF19                    (19u)
#define R_A_AS_CDF20                    (20u)
#define R_A_AS_CDF21                    (21u)
#define R_A_AS_CDF22                    (22u)
#define R_A_AS_CDF27                    (27u)
#define R_A_AS_CDF28                    (28u)
#define R_A_AS_CDF29                    (29u)
#define R_A_AS_CDF30                    (30u)

/* Error code */
#define R_A_AS_SUCCESS                  (0)
#define R_A_AS_ERR_INVALID_ARG          (-1)
#define R_A_AS_ERR_BUSY                 (-2)
#define R_A_AS_ERR_ACCESS               (-3)

/* The maximum number of field */
#define R_A_AS_TX_FLD_MAX_NUM           (4u)
#define R_A_AS_RX_FLD_MAX_NUM           (4u)

/* Reception result from the encoder */
typedef enum r_a_as_req_err_e
{
    R_A_AS_REQ_SUCCESS = 0,
    R_A_AS_REQ_ERR,
    R_A_AS_REQ_BP_ERR
} r_a_as_req_err_t;

/* Initialization information of A_AS control unit */
typedef struct r_a_as_info_s
{
    uint8_t                     connect;
    uint8_t                     bitrate;
    uint16_t                    ifmg;
} r_a_as_info_t;

/* Received data */
typedef struct r_a_as_data_s
{
    uint32_t                    rxi;
    uint32_t                    rxd0;
    uint32_t                    rxd1;
} r_a_as_data_t;

/* A-format communication controller status */
typedef struct r_a_as_status_s
{
    bool                        iwdgerr;
    bool                        dwdgerr;
    bool                        starterr;
    bool                        stoperr;
    bool                        syncerr;
    bool                        rxeaerr;
    bool                        crcerr;
    bool                        rxccerr;
    bool                        mdaterr;
    bool                        madrerr;
    bool                        rxdzerr;
    bool                        fd1err;
    bool                        fd2err;
    bool                        fd3err;
    bool                        fd5err;
    bool                        elcin;
    uint8_t                     txcc;
    bool                        rxset;
    bool                        timer;
    bool                        txerr;
    bool                        rxend;
} r_a_as_status_t;

/* Send and receive results */
typedef struct r_a_as_result_s
{
    r_a_as_req_err_t            result;
    r_a_as_data_t               data;
    r_a_as_status_t             status;
} r_a_as_result_t;
typedef void (*r_a_as_result_cb_t)(r_a_as_result_t *presult);

/* Request information to be sent to the encoder */
typedef struct r_a_as_req_s
{
    uint8_t                     encadr;
    uint8_t                     cmd;
    uint8_t                     memadr;
    uint16_t                    memdat;
    uint32_t                    encid;
    r_a_as_result_cb_t          cbadr_txerr;
    r_a_as_result_cb_t          cbadr_rxset;
    r_a_as_result_cb_t          cbadr_rxend;
    bool                        pre;
} r_a_as_req_t;

typedef uint8_t r_a_as_type_t;

/* Command of R_A_AS_Control function.*/
typedef uint32_t r_a_as_cmd_t;

int32_t R_A_AS_Open(const int32_t id, r_a_as_info_t* pinfo);
int32_t R_A_AS_Close(const int32_t id);
uint32_t R_A_AS_GetVersion(const r_a_as_type_t type );
int32_t R_A_AS_Control(const int32_t id, const r_a_as_cmd_t cmd, void *const pbuf);

