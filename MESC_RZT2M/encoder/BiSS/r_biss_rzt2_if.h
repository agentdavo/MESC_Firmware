#pragma once

#include <stdint.h>
#include <stdbool.h>

#define BISS_ID_NUM   (2)

// API error codes
#define R_BISS_SUCCESS          (0)     /* Success          */
#define R_BISS_ERR_INVALID_ARG  (-1)    /* Invalid argument */
#define R_BISS_ERR_BUSY         (-2)    /* Busy             */
#define R_BISS_ERR_ACCESS       (-3)    /* Access error     */

// Encoder Type
#define R_BISS_TYPE_CMN         (0)     /* BiSS Common */
#define R_BISS_TYPE_C           (2u)    /* BiSS-C */

typedef void (*r_biss_isr_cb_t)(void *presult);

// API error codes
typedef int8_t r_biss_err_t;

// Encoder Type
typedef uint8_t r_biss_type_t;

// Initial Information
typedef struct biss_pos_crc_s
{
    uint8_t                 crc_size;           /* CRC size (0-16bit) */
    uint16_t                crc_polynomial;     /* CRC polynomial */
    uint16_t                crc_start_value;    /* CRC start value */
} r_biss_pos_crc_t;

typedef struct biss_info_s
{
    uint16_t                clk_freq;           /* Clock frequency to encoder(80kHz-10MHz) */
    uint16_t                clk_reg_freq;       /* Not use */
    r_biss_pos_crc_t        crc_info;           /* CRC Information */
    uint8_t                 mtdata_size;        /* Multi-turn data size */
    uint8_t                 stdata_size;        /* Single-turn data size */
    uint8_t                 aldata_size;        /* Alignment data size */
} r_biss_info_t;

// Command of R_BISS_Control function.
typedef uint32_t r_biss_cmd_t;

r_biss_err_t R_BISS_Open(const int32_t id, const r_biss_type_t type, r_biss_info_t* pinfo);
r_biss_err_t R_BISS_Close(const int32_t id);
uint32_t     R_BISS_GetVersion(const r_biss_type_t type);
r_biss_err_t R_BISS_Control(const int32_t id, const r_biss_cmd_t cmd, void *const pbuf);

