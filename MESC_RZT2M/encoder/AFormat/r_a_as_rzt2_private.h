#pragma once

#include <ecl/r_ecl_rzt2_if.h>

#define A_AS_VER_ERR                    (0xFFFFFFFFu)

/* A_AS base address */
#define A_AS_BASE_ADDR0                 (0xA011C100u)
#define A_AS_BASE_ADDR1                 (0xA031C100u)

#define A_AS_FDAT_DR(addr)              (*(volatile A_AS_FDAT *)(addr))
#define BASE_FDAT_ADDR_CH0              (0xA0FD3000u)
#define BASE_FDAT_ADDR_CH1              (0xA0FD3200u)

#define A_AS_NF_DR(addr)                (*(volatile A_AS_NF *)(addr))
#define BASE_NF_ADDR_CH0                (0xA0FD5003u)
#define BASE_NF_ADDR_CH1                (0xA0FD5017u)


/* A_AS ID of Encoder I/F */
#define A_AS_ID0                        (R_ECL_CH_0)
#define A_AS_ID1                        (R_ECL_CH_1)

/* A_AS Channel */
#define A_AS_CH0                        (0u)
#define A_AS_CH1                        (1u)
#define A_AS_CH_NUM                     (2u)

/* Parameter check */
#define A_AS_CONNECT_NUM                (2u)
#define A_AS_BITRATE_NUM                (4u)
#define A_AS_ENCADR_NUM                 (8u)
#define A_AS_IFMG_MAX                   (0xFFFFu)
#define A_AS_CMD_MAX                    (0x1Fu)
#define A_AS_ENC_ID_MAX                 (0xFFFFFFu)
#define A_AS_INTERVAL_MAX               (0xFFFFFEu)

/* Send trigger */
#define A_AS_SEND_TRG                   (0u)

/* Transmission mode */
#define A_AS_TRANS_MODE_SINGLE          (0u)
#define A_AS_TRANS_MODE_MULTI           (1u)

/* SS register */
#define A_AS_SS_IWDGERR_BIT             (0x00000001u)
#define A_AS_SS_DWDGERR_BIT             (0x00000002u)
#define A_AS_SS_STARTERR_BIT            (0x00000004u)
#define A_AS_SS_STOPERR_BIT             (0x00000008u)
#define A_AS_SS_SYNCERR_BIT             (0x00000010u)
#define A_AS_SS_RXEAERR_BIT             (0x00000020u)
#define A_AS_SS_CRCERR_BIT              (0x00000040u)
#define A_AS_SS_RXCCERR_BIT             (0x00000080u)
#define A_AS_SS_MDATERR_BIT             (0x00000100u)
#define A_AS_SS_MADRERR_BIT             (0x00000200u)
#define A_AS_SS_RXDZERR_BIT             (0x00000400u)
#define A_AS_SS_FD1ERR_BIT              (0x00000800u)
#define A_AS_SS_FD2ERR_BIT              (0x00001000u)
#define A_AS_SS_FD3ERR_BIT              (0x00002000u)
#define A_AS_FSS_RXFFUL_BIT             (0x00004000u)
#define A_AS_SS_FD5ERR_BIT              (0x00008000u)
#define A_AS_SS_ELCIN_BIT               (0x01000000u)
#define A_AS_SS_TXCC_BIT                (0x0C000000u)
#define A_AS_SS_RXSET_BIT               (0x10000000u)
#define A_AS_SS_TIMER_BIT               (0x20000000u)
#define A_AS_SS_TXERR_BIT               (0x40000000u)
#define A_AS_SS_RXEND_BIT               (0x80000000u)
#define A_AS_SS_ERR_BIT                 (0x40FFBFFFu)
#define A_AS_FSS_ERR_BIT                (0x40FFFFFFu)
#define A_AS_SS_TXCC_CDF21              (0x04000000u)

/* Bit manipulation */
#define BIT_SHIFT_16                    (16u)
#define BIT_SHIFT_CC                    (3u)
#define BIT_SHIFT_TXFLD_CC              (8u)
#define BIT_SHIFT_TXCC                  (26u)

/* Interrupt settig */
#define A_AS_INTE_RXEND                 (0x80u)

/* ELC trigger */
#define A_AS_ELC_ENABLE                 (1u)
#define A_AS_ELC_DISABLE                (0u)

/* Parameters for each trigger */
#define A_AS_PARAM                      (0u)
#define A_AS_PARAM_PRE                  (1u)
#define A_AS_PARAM_NUM                  (2u)

/* FDAT trigger */
#define A_AS_FDAT0_BIT                  (0x0000FFFFu)

/* Even number */
#define A_AS_EVE_NUM                    (2u)

/* Number of send field */
#define A_AS_TX_FLD0                    (0u)
#define A_AS_TX_FLD1                    (1u)
#define A_AS_TX_FLD2                    (2u)
#define A_AS_TX_FLD3                    (3u)

/* Read the number of FDAT register */
#define A_AS_FDAT_READ_CLEAR            (9u)

/*******************************************************************************
Typedef definitions
*******************************************************************************/
/* A_AS Status */
typedef enum a_as_state_s
{
    A_AS_STATE_CLOSE,
    A_AS_STATE_IDLE,
    A_AS_STATE_TRANS,
    A_AS_STATE_ELC,
    A_AS_STATE_ELC_TRANS
} a_as_state_t;

typedef void (*a_as_isr_func_t)(uint8_t a_as_ch);

/* Parameter to use with the R_A_AS_Control */
typedef struct a_as_control_s
{
    uint8_t                     a_as_ch;
    a_as_state_t                state;
} a_as_control_t;

typedef int32_t (*a_as_cmd_func_t)(a_as_control_t *const pa_as_control, void *const pbuf);

