#include "r_a_format_rzt2_private.h"

#include <AFormat/r_a_as_rzt2_if.h>
#include <AFormat/iodefine_a_as.h>
//#include <AFormat/r_a_as_rzt2_config.h>
#include <AFormat/r_a_as_rzt2_private.h>

//#include "bsp_api.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define A_FORMAT_VERSION                (0x00010001u)
#define A_FROMAT_CMD_NUM                (4u)
#define PARAMETER_NOT_USED(p)           (void)((p))

typedef struct a_format_control_cmd_s
{
    r_a_as_cmd_t        control_cmd;
    a_as_cmd_func_t     control_cmd_func;
} a_format_control_cmd_t;

static int32_t a_format_cmd_set_param(a_as_control_t *const pa_as_control, void *const pbuf);
static int32_t a_format_set_tx(const uint8_t a_as_ch, r_a_as_req_t *pa_as_req);
static int32_t a_format_cmd_elc_disable(a_as_control_t *const pa_as_control, void *const pbuf);
static int32_t a_format_cmd_tx_trg(a_as_control_t *const pa_as_control, void *const pbuf);
static int32_t a_format_cmd_tx_elc(a_as_control_t *const pa_as_control, void *const pbuf);
static void a_format_isr_trans(const uint8_t a_as_ch, const uint32_t ss_reg, const uint8_t trans_mode);
static void a_format_isr_common(const uint8_t encadr, const uint8_t a_as_ch, const uint32_t ss_reg);
static void a_format_isr_status_clear(const uint8_t encadr, const uint8_t a_as_ch);
static void a_format_isr_status_set(const uint8_t encadr, const uint32_t ss_reg, const uint8_t a_as_ch);

// A_AS base address
static void* pa_format_base_addr_tbl[A_AS_CH_NUM] =
{
    (void *)A_AS_BASE_ADDR0,
    (void *)A_AS_BASE_ADDR1
};

static a_format_control_cmd_t a_format_control_cmd_tbl[A_FROMAT_CMD_NUM] =
{
    {R_A_AS_CMD_SET_PARAM , &a_format_cmd_set_param},
    {R_A_AS_CMD_ELC_DISABLE, &a_format_cmd_elc_disable},
    {R_A_AS_CMD_TX_TRG, &a_format_cmd_tx_trg},
    {R_A_AS_CMD_TX_ELC, &a_format_cmd_tx_elc},
};

// Request information and send and receive results
static r_a_as_req_t a_format_req[A_AS_CH_NUM][A_AS_PARAM_NUM];
static r_a_as_result_t  a_format_result[A_AS_CH_NUM][A_AS_ENCADR_NUM];

// Parameters enabled flag
static bool a_format_param_flg[A_AS_CH_NUM] =
{
    false,
    false
};

// Preset valid flag
static bool a_format_pre_flg[A_AS_CH_NUM] =
{
    false,
    false
};

void R_AFORMAT_Open(const uint8_t a_as_ch)
{
    for (uint8_t loop = 0; loop < A_AS_PARAM_NUM; loop++)
    {
        (void)memset(&a_format_req[a_as_ch][loop], 0, sizeof(r_a_as_req_t));
    }
    for (uint8_t loop = 0; loop < A_AS_ENCADR_NUM; loop++)
    {
        (void)memset(&a_format_result[a_as_ch][loop], 0, sizeof(r_a_as_result_t));
    }
    a_format_param_flg[a_as_ch] = false;
    a_format_pre_flg[a_as_ch] = false;
}

uint32_t R_AFORMAT_Getversion(const r_a_as_type_t type)
{
    return (R_A_AS_A_FORMAT == type) ? A_FORMAT_VERSION : A_AS_VER_ERR;
}

int32_t R_AFORMAT_Control(const r_a_as_cmd_t cmd,
                          a_as_control_t *const pa_as_control, void *const pbuf)
{
    if (NULL == pa_as_control)
    {
        return R_A_AS_ERR_INVALID_ARG;
    }

    for (uint32_t index = 0; index < A_FROMAT_CMD_NUM; index++)
    {
        if (cmd == a_format_control_cmd_tbl[index].control_cmd)
        {
            return a_format_control_cmd_tbl[index].control_cmd_func(pa_as_control, pbuf);
        }
    }
    return R_A_AS_ERR_INVALID_ARG;
}

static int32_t a_format_cmd_set_param(a_as_control_t* const pa_as_control, void* const pbuf)
{
    r_a_as_req_t* pa_as_req = pbuf;

    if ((NULL == pa_as_req) ||
        (NULL == pa_as_control))
    { // Check of parameters
        return R_A_AS_ERR_INVALID_ARG;
    }

    // Setting of the send parameters
    int32_t result = a_format_set_tx(pa_as_control->a_as_ch, pa_as_req);
    if (R_A_AS_SUCCESS != result)
    {
        return result;
    }

    // Copy of the request information
    if (false == pa_as_req->pre)
    { // Save of the now request information
        (void)memcpy(&a_format_req[pa_as_control->a_as_ch][A_AS_PARAM], pa_as_req, sizeof(r_a_as_req_t));
        a_format_param_flg[pa_as_control->a_as_ch] = true;
        a_format_pre_flg[pa_as_control->a_as_ch] = false;
    }
    else
    { // Save of the next request information
        (void)memcpy(&a_format_req[pa_as_control->a_as_ch][A_AS_PARAM_PRE], pa_as_req, sizeof(r_a_as_req_t));
        a_format_pre_flg[pa_as_control->a_as_ch] = true;
    }
    return R_A_AS_SUCCESS;
}

static int32_t a_format_set_tx(const uint8_t a_as_ch, r_a_as_req_t* pa_as_req)
{
    // Check of Encoder address and Command data frame
    if ((A_AS_ENCADR_NUM <= pa_as_req->encadr) ||
        (A_AS_CMD_MAX <= pa_as_req->cmd))
    {
        return R_A_AS_ERR_INVALID_ARG;
    }

    if ((false == a_format_param_flg[a_as_ch]) ||
        (a_format_req[a_as_ch][A_AS_PARAM].cmd != pa_as_req->cmd) ||
        (a_format_req[a_as_ch][A_AS_PARAM].encadr != pa_as_req->encadr))
    { // Setting of the send parameters
        A_AS_DR(pa_format_base_addr_tbl[a_as_ch]).TXC.LONG =
            (uint32_t)((pa_as_req->cmd << BIT_SHIFT_CC) | pa_as_req->encadr);
    }

    switch(pa_as_req->cmd)
    {
        case R_A_AS_CDF13: // Setting the address of the encoder memory
            if ((false == a_format_param_flg[a_as_ch]) ||
                (a_format_req[a_as_ch][A_AS_PARAM].memadr != pa_as_req->memadr))
            {
                A_AS_DR(pa_format_base_addr_tbl[a_as_ch]).TXD.LONG =
                    (uint32_t)(pa_as_req->memadr << BIT_SHIFT_16);
            }
            break;

        case R_A_AS_CDF14: // Setting of address and data encoder memory
            if ((false == a_format_param_flg[a_as_ch]) ||
                (a_format_req[a_as_ch][A_AS_PARAM].memadr != pa_as_req->memadr) ||
                (a_format_req[a_as_ch][A_AS_PARAM].memdat != pa_as_req->memdat))
            {
                A_AS_DR(pa_format_base_addr_tbl[a_as_ch]).TXD.LONG =
                    (uint32_t)((pa_as_req->memadr << BIT_SHIFT_16) | pa_as_req->memdat);
            }
            break;

        case R_A_AS_CDF18: // Setting of the encoder ID
        case R_A_AS_CDF19:
        case R_A_AS_CDF20:
            if (A_AS_ENC_ID_MAX < pa_as_req->encid)
            {
                return R_A_AS_ERR_INVALID_ARG;
            }
            if ((false == a_format_param_flg[a_as_ch]) ||
                (a_format_req[a_as_ch][A_AS_PARAM].encid != pa_as_req->encid))
            {
                A_AS_DR(pa_format_base_addr_tbl[a_as_ch]).TXD.LONG = pa_as_req->encid;
            }
            break;

        default:
            break;
    }
    return R_A_AS_SUCCESS;
}

static int32_t a_format_cmd_elc_disable(a_as_control_t *const pa_as_control, void *const pbuf)
{
    PARAMETER_NOT_USED(pa_as_control);
    PARAMETER_NOT_USED(pbuf);
    return R_A_AS_SUCCESS;
}

static int32_t a_format_cmd_tx_trg(a_as_control_t *const pa_as_control, void *const pbuf)
{
    PARAMETER_NOT_USED(pa_as_control);
    PARAMETER_NOT_USED(pbuf);
    return R_A_AS_SUCCESS;
}

static int32_t a_format_cmd_tx_elc(a_as_control_t *const pa_as_control, void *const pbuf)
{
    PARAMETER_NOT_USED(pa_as_control);
    PARAMETER_NOT_USED(pbuf);
    return R_A_AS_SUCCESS;
}

void R_AFORMAT_Clear_Param(const uint8_t a_as_ch)
{
    a_format_param_flg[a_as_ch] = false;
}

void R_AFORMAT_INT_END_Isr(const uint8_t a_as_ch, const uint32_t ss_reg)
{
    uint8_t trans_mode;
    uint8_t cmd = a_format_req[a_as_ch][A_AS_PARAM].cmd;
    switch(cmd)
    {
        // Multiple transmission mode
        case R_A_AS_CDF4:
        case R_A_AS_CDF5:
        case R_A_AS_CDF6:
        case R_A_AS_CDF7:
        case R_A_AS_CDF22:
        case R_A_AS_CDF28:
        case R_A_AS_CDF30:
            trans_mode = A_AS_TRANS_MODE_MULTI;
            break;

        // Single transmission mode
        default:
            trans_mode = A_AS_TRANS_MODE_SINGLE;
            break;
    }
    a_format_isr_trans(a_as_ch, ss_reg, trans_mode);
}

static void a_format_isr_trans(const uint8_t a_as_ch, const uint32_t ss_reg, const uint8_t trans_mode)
{
    uint32_t ss_reg_multi;
    uint8_t loop_enc;

    // Process of TXERR
    if (A_AS_SS_TXERR_BIT == (ss_reg & A_AS_SS_TXERR_BIT))
    {
        a_format_isr_common(a_format_req[a_as_ch][A_AS_PARAM].encadr, a_as_ch, ss_reg);
        if (NULL != a_format_req[a_as_ch][A_AS_PARAM].cbadr_txerr)
        {
            a_format_req[a_as_ch][A_AS_PARAM].cbadr_txerr(&a_format_result[a_as_ch][0]);
        }
    }

    // Process of RXSET
    if (A_AS_SS_RXSET_BIT == (ss_reg & A_AS_SS_RXSET_BIT))
    {
        // Read of the transmission result of the request
        if (A_AS_TRANS_MODE_SINGLE == trans_mode)
        {
            a_format_isr_common(a_format_req[a_as_ch][A_AS_PARAM].encadr, a_as_ch, ss_reg);
            a_format_result[a_as_ch][a_format_req[a_as_ch][A_AS_PARAM].encadr].data.rxi = A_AS_DR(pa_format_base_addr_tbl[a_as_ch]).RXI.LONG;
            a_format_result[a_as_ch][a_format_req[a_as_ch][A_AS_PARAM].encadr].data.rxd0 = A_AS_DR(pa_format_base_addr_tbl[a_as_ch]).RXD0.LONG;
            a_format_result[a_as_ch][a_format_req[a_as_ch][A_AS_PARAM].encadr].data.rxd1 = A_AS_DR(pa_format_base_addr_tbl[a_as_ch]).RXD1.LONG;
        }
        else
        {
            loop_enc = 0u;
            a_format_isr_common(loop_enc, a_as_ch, ss_reg);
            a_format_result[a_as_ch][loop_enc].data.rxi   = A_AS_DR(pa_format_base_addr_tbl[a_as_ch]).RXI.LONG;
            a_format_result[a_as_ch][loop_enc].data.rxd0  = A_AS_DR(pa_format_base_addr_tbl[a_as_ch]).RXD0.LONG;
            a_format_result[a_as_ch][loop_enc].data.rxd1  = A_AS_DR(pa_format_base_addr_tbl[a_as_ch]).RXD1.LONG;
            for (loop_enc = 1u; loop_enc <=  a_format_req[a_as_ch][A_AS_PARAM].encadr; loop_enc++)
            {
                A_AS_DR(pa_format_base_addr_tbl[a_as_ch]).ESEL.BYTE = loop_enc;
                ss_reg_multi = A_AS_DR(pa_format_base_addr_tbl[a_as_ch]).SS.LONG;
                if ((ss_reg_multi & A_AS_SS_RXSET_BIT) == 0 &&
                    (ss_reg & A_AS_SS_RXEND_BIT) == 0)
                {
                    A_AS_DR(pa_format_base_addr_tbl[a_as_ch]).ESEL.BYTE = 0u;
                    return;  // following encoder data is not ready. return and wait for ready.
                }
                a_format_isr_common(loop_enc, a_as_ch, ss_reg_multi);
                a_format_result[a_as_ch][loop_enc].data.rxi   = A_AS_DR(pa_format_base_addr_tbl[a_as_ch]).RXI.LONG;
                a_format_result[a_as_ch][loop_enc].data.rxd0  = A_AS_DR(pa_format_base_addr_tbl[a_as_ch]).RXD0.LONG;
                a_format_result[a_as_ch][loop_enc].data.rxd1  = A_AS_DR(pa_format_base_addr_tbl[a_as_ch]).RXD1.LONG;
            }
            A_AS_DR(pa_format_base_addr_tbl[a_as_ch]).ESEL.BYTE = 0u;
        }

        if (NULL != a_format_req[a_as_ch][A_AS_PARAM].cbadr_rxset)
        {
            a_format_req[a_as_ch][A_AS_PARAM].cbadr_rxset(&a_format_result[a_as_ch][0]);
        }
    }

    // Process of RXEND
    if (A_AS_SS_RXEND_BIT == (ss_reg & A_AS_SS_RXEND_BIT))
    { // Read of the transmission result of the request
        if (A_AS_TRANS_MODE_SINGLE == trans_mode)
        {
            a_format_isr_common(a_format_req[a_as_ch][A_AS_PARAM].encadr, a_as_ch, ss_reg);
        }
        else
        {
            loop_enc = 0u;
            a_format_isr_common(loop_enc, a_as_ch, ss_reg);
            for (loop_enc = 1u; loop_enc <= a_format_req[a_as_ch][A_AS_PARAM].encadr; loop_enc++)
            {
                A_AS_DR(pa_format_base_addr_tbl[a_as_ch]).ESEL.BYTE = loop_enc;
                ss_reg_multi = A_AS_DR(pa_format_base_addr_tbl[a_as_ch]).SS.LONG;
                a_format_isr_common(loop_enc, a_as_ch, ss_reg_multi);
            }
            A_AS_DR(pa_format_base_addr_tbl[a_as_ch]).ESEL.BYTE = 0u;
        }

        if (NULL !=  a_format_req[a_as_ch][A_AS_PARAM].cbadr_rxend)
        {
            a_format_req[a_as_ch][A_AS_PARAM].cbadr_rxend(&a_format_result[a_as_ch][0]);
        }
    }
}

static void a_format_isr_common(const uint8_t encadr, const uint8_t a_as_ch, const uint32_t ss_reg)
{
    a_format_result[a_as_ch][encadr].result = (0u == (ss_reg & A_AS_SS_ERR_BIT)) ? R_A_AS_REQ_SUCCESS : R_A_AS_REQ_ERR;
    a_format_isr_status_clear(encadr, a_as_ch);
    a_format_isr_status_set(encadr, ss_reg, a_as_ch);
}

static void a_format_isr_status_clear(const uint8_t encadr, const uint8_t a_as_ch)
{
    a_format_result[a_as_ch][encadr].status.iwdgerr     = false;
    a_format_result[a_as_ch][encadr].status.dwdgerr     = false;
    a_format_result[a_as_ch][encadr].status.starterr    = false;
    a_format_result[a_as_ch][encadr].status.stoperr     = false;
    a_format_result[a_as_ch][encadr].status.syncerr     = false;
    a_format_result[a_as_ch][encadr].status.rxeaerr     = false;
    a_format_result[a_as_ch][encadr].status.crcerr      = false;
    a_format_result[a_as_ch][encadr].status.rxccerr     = false;
    a_format_result[a_as_ch][encadr].status.mdaterr     = false;
    a_format_result[a_as_ch][encadr].status.madrerr     = false;
    a_format_result[a_as_ch][encadr].status.rxdzerr     = false;
    a_format_result[a_as_ch][encadr].status.fd1err      = false;
    a_format_result[a_as_ch][encadr].status.fd2err      = false;
    a_format_result[a_as_ch][encadr].status.fd3err      = false;
    a_format_result[a_as_ch][encadr].status.fd5err      = false;
    a_format_result[a_as_ch][encadr].status.elcin       = false;
    a_format_result[a_as_ch][encadr].status.rxset       = false;
    a_format_result[a_as_ch][encadr].status.timer       = false;
    a_format_result[a_as_ch][encadr].status.txerr       = false;
    a_format_result[a_as_ch][encadr].status.rxend       = false;
}

static void a_format_isr_status_set(const uint8_t encadr, const uint32_t ss_reg, const uint8_t a_as_ch)
{
    if (A_AS_SS_IWDGERR_BIT == (ss_reg & A_AS_SS_IWDGERR_BIT))
    {
        a_format_result[a_as_ch][encadr].status.iwdgerr = true;
    }
    if (A_AS_SS_DWDGERR_BIT == (ss_reg & A_AS_SS_DWDGERR_BIT))
    {
        a_format_result[a_as_ch][encadr].status.dwdgerr = true;
    }
    if (A_AS_SS_STARTERR_BIT == (ss_reg & A_AS_SS_STARTERR_BIT))
    {
        a_format_result[a_as_ch][encadr].status.starterr = true;
    }
    if (A_AS_SS_STOPERR_BIT == (ss_reg & A_AS_SS_STOPERR_BIT))
    {
        a_format_result[a_as_ch][encadr].status.stoperr = true;
    }
    if (A_AS_SS_SYNCERR_BIT == (ss_reg & A_AS_SS_SYNCERR_BIT))
    {
        a_format_result[a_as_ch][encadr].status.syncerr = true;
    }
    if (A_AS_SS_RXEAERR_BIT == (ss_reg & A_AS_SS_RXEAERR_BIT))
    {
        a_format_result[a_as_ch][encadr].status.rxeaerr = true;
    }
    if (A_AS_SS_CRCERR_BIT == (ss_reg & A_AS_SS_CRCERR_BIT))
    {
        a_format_result[a_as_ch][encadr].status.crcerr = true;
    }
    if (A_AS_SS_RXCCERR_BIT == (ss_reg & A_AS_SS_RXCCERR_BIT))
    {
        a_format_result[a_as_ch][encadr].status.rxccerr = true;
    }
    if (A_AS_SS_MDATERR_BIT == (ss_reg & A_AS_SS_MDATERR_BIT))
    {
        a_format_result[a_as_ch][encadr].status.mdaterr = true;
    }
    if (A_AS_SS_MADRERR_BIT == (ss_reg & A_AS_SS_MADRERR_BIT))
    {
        a_format_result[a_as_ch][encadr].status.madrerr = true;
    }
    if (A_AS_SS_RXDZERR_BIT == (ss_reg & A_AS_SS_RXDZERR_BIT))
    {
        a_format_result[a_as_ch][encadr].status.rxdzerr = true;
    }
    if (A_AS_SS_FD1ERR_BIT == (ss_reg & A_AS_SS_FD1ERR_BIT))
    {
        a_format_result[a_as_ch][encadr].status.fd1err = true;
    }
    if (A_AS_SS_FD2ERR_BIT == (ss_reg & A_AS_SS_FD2ERR_BIT))
    {
        a_format_result[a_as_ch][encadr].status.fd2err = true;
    }
    if (A_AS_SS_FD3ERR_BIT == (ss_reg & A_AS_SS_FD3ERR_BIT))
    {
        a_format_result[a_as_ch][encadr].status.fd3err = true;
    }
    if (A_AS_SS_FD5ERR_BIT == (ss_reg & A_AS_SS_FD5ERR_BIT))
    {
        a_format_result[a_as_ch][encadr].status.fd5err = true;
    }
    if (A_AS_SS_ELCIN_BIT == (ss_reg & A_AS_SS_ELCIN_BIT))
    {
        a_format_result[a_as_ch][encadr].status.elcin = true;
    }

    a_format_result[a_as_ch][encadr].status.txcc = (uint8_t)((ss_reg & A_AS_SS_TXCC_BIT) >> BIT_SHIFT_TXCC);

    if (A_AS_SS_RXSET_BIT == (ss_reg & A_AS_SS_RXSET_BIT))
    {
        a_format_result[a_as_ch][encadr].status.rxset = true;
    }
    if (A_AS_SS_TIMER_BIT == (ss_reg & A_AS_SS_TIMER_BIT))
    {
        a_format_result[a_as_ch][encadr].status.timer = true;
    }
    if (A_AS_SS_TXERR_BIT == (ss_reg & A_AS_SS_TXERR_BIT))
    {
        a_format_result[a_as_ch][encadr].status.txerr = true;
    }
    if (A_AS_SS_RXEND_BIT == (ss_reg & A_AS_SS_RXEND_BIT))
    {
        a_format_result[a_as_ch][encadr].status.rxend = true;
    }
}

void R_AFORMAT_Copy_Param(const uint8_t a_as_ch)
{
    // Copy of the request information
    if (false != a_format_pre_flg[a_as_ch])
    {
        a_format_pre_flg[a_as_ch] = false;
        (void)memcpy(&a_format_req[a_as_ch][A_AS_PARAM],
                     &a_format_req[a_as_ch][A_AS_PARAM_PRE],
                     sizeof(r_a_as_req_t));
        a_format_param_flg[a_as_ch] = true;
    }
}

