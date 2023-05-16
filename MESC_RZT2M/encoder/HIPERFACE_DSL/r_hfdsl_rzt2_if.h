#pragma once

//#include "bsp_api.h"
#include <stdint.h>
#include <stdbool.h>

/* R_HFDSL_Control function control commands */
#define R_HFDSL_CMD_INIT1               (1u)
#define R_HFDSL_CMD_INIT2               (2u)
#define R_HFDSL_CMD_INIT3               (3u)
#define R_HFDSL_CMD_INIT4               (4u)
#define R_HFDSL_CMD_INIT5               (5u)
#define R_HFDSL_CMD_INIT6               (6u)
#define R_HFDSL_CMD_POS                 (11u)
#define R_HFDSL_CMD_VPOS                (12u)
#define R_HFDSL_CMD_VEL                 (13u)
#define R_HFDSL_CMD_MSG                 (14u)
#define R_HFDSL_CMD_RST                 (15u)

/* Error code */
#define R_HFDSL_SUCCESS                 (0)
#define R_HFDSL_ERR_INVALID_ARG         (-1)
#define R_HFDSL_ERR_ACCESS              (-2)
#define R_HFDSL_ERR_INIT                (-3)

typedef void (*r_hfdsl_int_nml_cb_t)(uint8_t event);
typedef void (*r_hfdsl_int_err_cb_t)(uint32_t event_err);
typedef void (*r_hfdsl_int_fifo_raw_cb_t)(uint16_t* raw_data);
typedef void (*r_hfdsl_int_init_cb_t)(void);

/* Initialization information of HFDSL driver control unit */
typedef struct r_hfdsl_info_s
{
    r_hfdsl_int_nml_cb_t        pcb_nml;
    r_hfdsl_int_err_cb_t        pcb_err;
    r_hfdsl_int_fifo_raw_cb_t   pcb_raw;
    r_hfdsl_int_init_cb_t       pcb_init;
} r_hfdsl_info_t;

/* Store of Fast Position */
typedef struct r_hfdsl_pos_s
{
    bool                        all;
    uint8_t                     posh;
    uint32_t                    pos;
} r_hfdsl_pos_t;

/* Store of Safe Position */
typedef struct r_hfdsl_vpos_s
{
    uint8_t                     vposh;
    uint32_t                    vpos;
    uint16_t                    crc;
} r_hfdsl_vpos_t;

typedef void (*r_hfdsl_msg_cb_t)(uint16_t* msg_data);

/* Store of message transmission data */
typedef struct r_hfdsl_send_msg_s
{
    uint16_t                    *pdata;
    r_hfdsl_msg_cb_t            pcb_msg;
} r_hfdsl_send_msg_t;

/* Command of R_HFDSL_Control function.*/
typedef uint32_t r_hfdsl_cmd_t;

int32_t R_HFDSL_Open(const int32_t id, r_hfdsl_info_t* pinfo);
int32_t R_HFDSL_Close(const int32_t id);
uint32_t R_HFDSL_GetVersion(void);
int32_t R_HFDSL_Control(const int32_t id, const r_hfdsl_cmd_t cmd, void *const pbuf);
int32_t R_HFDSL_CheckInitSeq(const int32_t id);

