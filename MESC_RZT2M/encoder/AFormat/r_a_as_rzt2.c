#include "r_a_as_rzt2_if.h"

#include "bsp_api.h"

#include "iodefine_a_as.h"
#include "r_a_as_rzt2_private.h"
#include "r_a_as_rzt2_config.h"

#include "a_format/r_a_format_rzt2_private.h"

#include <stdio.h>

/* NF register */
#define A_AS_INF_ON                     (1u)
#define A_AS_INF_OFF                    (0u)

/* Mask other than SET_PARAM */
#define A_AS_CMD_SET_PARAM_MASK         (0x000000FFu)

void enc_a_as_ch0_int_isr(void);
void enc_a_as_ch1_int_isr(void);

static int32_t a_as_reg_init(const uint8_t a_as_ch, const r_a_as_info_t* const pinfo);
static void a_as_int_end_set(const uint8_t a_as_ch);
static int32_t a_as_control_state_check(const uint8_t a_as_ch, r_a_as_cmd_t cmd);
static int32_t a_as_control_operation(const uint8_t a_as_ch, r_a_as_cmd_t cmd );
static void a_as_int_isr_common(const uint8_t a_as_ch);

/* A_AS base address */
static void*  pa_as_base_addr_tbl[A_AS_CH_NUM] =
{
    (void *)A_AS_BASE_ADDR0,
    (void *)A_AS_BASE_ADDR1
};

static void*  pa_as_fdat_addr_tbl[A_AS_CH_NUM] =
{
    (void *)BASE_FDAT_ADDR_CH0,
    (void *)BASE_FDAT_ADDR_CH1,
};

static void*  pa_as_nf_addr_tbl[A_AS_CH_NUM] =
{
    (void *)BASE_NF_ADDR_CH0,
    (void *)BASE_NF_ADDR_CH1,
};

/* Status of A_AS driver */
static a_as_state_t a_as_state_tbl[A_AS_CH_NUM] =
{
    A_AS_STATE_CLOSE,
    A_AS_STATE_CLOSE
};

#if 0
/* Address of the interrupt function */
static const void* pa_as_int_end_isr_addr_tbl[A_AS_CH_NUM] =
{
    (void *)&a_as0_int_isr,
    (void *)&a_as1_int_isr
};
#endif

/* Parameters of the noise filter */
static  uint8_t a_as_nfintv_tbl[A_AS_BITRATE_NUM] =
{
    R_A_AS_NFINTV_2500KBPS,
    R_A_AS_NFINTV_4MBPS,
    R_A_AS_NFINTV_6670KBPS,
    R_A_AS_NFINTV_8MBPS
};

static  uint8_t a_as_nfscnt_tbl[A_AS_BITRATE_NUM] =
{
    R_A_AS_NFSCNT_2500KBPS,
    R_A_AS_NFSCNT_4MBPS,
    R_A_AS_NFSCNT_6670KBPS,
    R_A_AS_NFSCNT_8MBPS
};

/* Parameters of the timing setting register */
static  uint16_t a_as_t2reg_tbl[A_AS_CONNECT_NUM][A_AS_BITRATE_NUM] =
{
    {
        R_A_AS_T2_ONE_2500KBPS,
        R_A_AS_T2_ONE_4MBPS,
        R_A_AS_T2_ONE_6670KBPS,
        R_A_AS_T2_ONE_8MBPS,
    },
    {
        R_A_AS_T2_BUS_2500KBPS,
        R_A_AS_T2_BUS_4MBPS,
        R_A_AS_T2_BUS_6670KBPS,
        R_A_AS_T2_BUS_8MBPS,
    }
};

static  uint16_t a_as_t3reg_tbl[A_AS_BITRATE_NUM] =
{
    R_A_AS_T3_2500KBPS,
    R_A_AS_T3_4MBPS,
    R_A_AS_T3_6670KBPS,
    R_A_AS_T3_8MBPS
};

static  uint16_t a_as_t5reg_tbl[A_AS_BITRATE_NUM] =
{
    R_A_AS_T5_2500KBPS,
    R_A_AS_T5_4MBPS,
    R_A_AS_T5_6670KBPS,
    R_A_AS_T5_8MBPS
};

static  uint16_t a_as_t9reg_tbl[A_AS_CONNECT_NUM] =
{
    R_A_AS_T9_ONE,
    R_A_AS_T9_BUS
};

int32_t R_A_AS_Open(const int32_t id, r_a_as_info_t* pinfo)
{
    uint8_t a_as_ch;

    if (A_AS_ID0 == id)
    {
        a_as_ch = A_AS_CH0;
    }
    else if (A_AS_ID1 == id)
    {
        a_as_ch = A_AS_CH1;
    }
    else
    {
        return R_A_AS_ERR_INVALID_ARG;
    }

    if (NULL == pinfo)
    {
        return R_A_AS_ERR_INVALID_ARG;
    }

    if (A_AS_STATE_CLOSE != a_as_state_tbl[a_as_ch])
    {
        return R_A_AS_ERR_ACCESS;
    }

    R_AFORMAT_Open(a_as_ch);

    int32_t result = a_as_reg_init(a_as_ch, pinfo);
    if (R_A_AS_SUCCESS != result)
    {
        return result;
    }

    a_as_int_end_set(a_as_ch); // Interrupt settings
    a_as_state_tbl[a_as_ch] = A_AS_STATE_IDLE; // set A-format Status

    return result;
}

int32_t R_A_AS_Close(const int32_t id)
{
    uint8_t a_as_ch;

    if (A_AS_ID0 == id)
    {
        a_as_ch = A_AS_CH0;
    }
    else if (A_AS_ID1 == id)
    {
        a_as_ch = A_AS_CH1;
    }
    else
    {
        return R_A_AS_ERR_INVALID_ARG;
    }

    if (A_AS_STATE_IDLE == a_as_state_tbl[a_as_ch])
    {
        if (a_as_ch == 0)
        {
            R_BSP_IrqDisable(VECTOR_NUMBER_ENCIF_INT0);
        }
        else if (a_as_ch == 1)
        {
            R_BSP_IrqDisable(VECTOR_NUMBER_ENCIF_INT4);
        }
        else
        {
            return R_A_AS_ERR_INVALID_ARG;
        }
        a_as_state_tbl[a_as_ch] = A_AS_STATE_CLOSE;
    }
    else
    {
        return R_A_AS_ERR_ACCESS;
    }
    return R_A_AS_SUCCESS;
}

uint32_t R_A_AS_GetVersion(const r_a_as_type_t type)
{
    return R_AFORMAT_Getversion(type);
}

int32_t R_A_AS_Control(const int32_t id, const r_a_as_cmd_t cmd, void *const pbuf)
{
    a_as_control_t a_as_control;
    uint8_t a_as_ch;

    if (A_AS_ID0 == id)
    {
        a_as_ch = A_AS_CH0;
        a_as_control.a_as_ch = A_AS_CH0;
    }
    else if (A_AS_ID1 == id)
    {
        a_as_ch = A_AS_CH1;
        a_as_control.a_as_ch = A_AS_CH1;
    }
    else
    {
        return R_A_AS_ERR_INVALID_ARG;
    }

    int32_t result = a_as_control_state_check(a_as_ch, cmd);
    if (R_A_AS_SUCCESS != result)
    {
        return result;
    }

    a_as_control.state = a_as_state_tbl[a_as_ch]; // Initialization of automatic variables

    result = R_AFORMAT_Control(cmd, &a_as_control, pbuf); // Control function of A-format
    if (R_A_AS_SUCCESS == result)
    { // Send processing
        result = a_as_control_operation(a_as_ch, cmd);
    }
    return result;
}

static int32_t a_as_reg_init(const uint8_t a_as_ch, const r_a_as_info_t* const pinfo)
{
    if ((A_AS_CH_NUM <= a_as_ch) ||
        (NULL == pinfo) ||
        (A_AS_CONNECT_NUM <= pinfo->connect) ||
        (A_AS_BITRATE_NUM <= pinfo->bitrate))
    {
        return R_A_AS_ERR_INVALID_ARG;
    }

    // Initial setting (Noise Filter)
    A_AS_NF_DR(pa_as_nf_addr_tbl[a_as_ch]).BYTE = (a_as_nfintv_tbl[pinfo->bitrate] |
                                                   a_as_nfscnt_tbl[pinfo->bitrate]);
    A_AS_DR(pa_as_base_addr_tbl[a_as_ch]).T2.WORD = a_as_t2reg_tbl[pinfo->connect][pinfo->bitrate];
    A_AS_DR(pa_as_base_addr_tbl[a_as_ch]).T3.WORD = a_as_t3reg_tbl[pinfo->bitrate];
    A_AS_DR(pa_as_base_addr_tbl[a_as_ch]).T4.WORD = R_A_AS_T4;
    A_AS_DR(pa_as_base_addr_tbl[a_as_ch]).T5.WORD = a_as_t5reg_tbl[pinfo->bitrate];
    A_AS_DR(pa_as_base_addr_tbl[a_as_ch]).T9.WORD = a_as_t9reg_tbl[pinfo->connect];
    A_AS_DR(pa_as_base_addr_tbl[a_as_ch]).BR.BYTE = pinfo->bitrate;
    A_AS_DR(pa_as_base_addr_tbl[a_as_ch]).IFMG.WORD = pinfo->ifmg;

    A_AS_DR(pa_as_base_addr_tbl[a_as_ch]).SS.LONG   = 0u;

    for (uint8_t loop = 0; loop < A_AS_FDAT_READ_CLEAR; loop++)
    {
        A_AS_FDAT_DR(pa_as_fdat_addr_tbl[a_as_ch]).LONG;  // dummy read
        A_AS_DR(pa_as_base_addr_tbl[a_as_ch]).SS.LONG;  // dummy read
    }
    return R_A_AS_SUCCESS;
}

static void a_as_int_end_set(const uint8_t a_as_ch)
{
    if (a_as_ch == 0)
    {
      R_BSP_IrqDisable(VECTOR_NUMBER_ENCIF_INT0);
      R_BSP_IrqCfg(VECTOR_NUMBER_ENCIF_INT0, 11, NULL);
      R_BSP_IrqEnable(VECTOR_NUMBER_ENCIF_INT0);
      A_AS_DR(pa_as_base_addr_tbl[a_as_ch]).INTE.BYTE = R_A_AS_INTE;
    }
    else
    {
      R_BSP_IrqDisable(VECTOR_NUMBER_ENCIF_INT4);
      R_BSP_IrqCfg(VECTOR_NUMBER_ENCIF_INT4, 11, NULL);
      R_BSP_IrqEnable(VECTOR_NUMBER_ENCIF_INT4);
      A_AS_DR(pa_as_base_addr_tbl[a_as_ch]).INTE.BYTE = R_A_AS_INTE;
    }
}

static int32_t a_as_control_state_check(const uint8_t a_as_ch, r_a_as_cmd_t cmd)
{
    if (A_AS_STATE_CLOSE == a_as_state_tbl[a_as_ch])
    {
        return R_A_AS_ERR_ACCESS;
    }
    switch(cmd)
    {
        case R_A_AS_CMD_TX_TRG:
            if ((A_AS_STATE_IDLE != a_as_state_tbl[a_as_ch]) &&
                (A_AS_STATE_ELC != a_as_state_tbl[a_as_ch]))
            {
                return R_A_AS_ERR_BUSY;
            }
            break;

        case R_A_AS_CMD_TX_ELC:
            if (A_AS_STATE_IDLE != a_as_state_tbl[a_as_ch])
            {
                return R_A_AS_ERR_BUSY;
            }
            break;

        case R_A_AS_CMD_ELC_DISABLE:
            if ((A_AS_STATE_ELC != a_as_state_tbl[a_as_ch]) &&
                (A_AS_STATE_ELC_TRANS != a_as_state_tbl[a_as_ch]))
            {
                return R_A_AS_ERR_ACCESS;
            }
            break;

        default:
            break;
    }

    return R_A_AS_SUCCESS;
}

static int32_t a_as_control_operation(const uint8_t a_as_ch, r_a_as_cmd_t cmd)
{
    switch(cmd)
    {
        case R_A_AS_CMD_TX_TRG: // Single trigger
            if (A_AS_INTE_RXEND == (R_A_AS_INTE & A_AS_INTE_RXEND))
            {
                if (A_AS_STATE_IDLE == a_as_state_tbl[a_as_ch])
                {
                    a_as_state_tbl[a_as_ch] = A_AS_STATE_TRANS;
                }
                if (A_AS_STATE_ELC == a_as_state_tbl[a_as_ch])
                {
                    a_as_state_tbl[a_as_ch] = A_AS_STATE_ELC_TRANS;
                }
            }
            A_AS_DR(pa_as_base_addr_tbl[a_as_ch]).TRG.BYTE = A_AS_SEND_TRG;
            break;

        case R_A_AS_CMD_TX_ELC: // ELC trigger
            a_as_state_tbl[a_as_ch] = A_AS_STATE_ELC;
            A_AS_DR(pa_as_base_addr_tbl[a_as_ch]).CTL.BIT.ELCIN0E = A_AS_ELC_ENABLE;
            break;

        case R_A_AS_CMD_ELC_DISABLE: // Disable of ELC
            a_as_state_tbl[a_as_ch] = A_AS_STATE_IDLE;
            A_AS_DR(pa_as_base_addr_tbl[a_as_ch]).CTL.BIT.ELCIN0E = A_AS_ELC_DISABLE;
            break;
        default: // Do nothing
            break;
    }
    return R_A_AS_SUCCESS;
}

static void a_as_int_isr_common(const uint8_t a_as_ch)
{
    uint32_t ss_reg = A_AS_DR(pa_as_base_addr_tbl[a_as_ch]).SS.LONG; // Read of the SS register
    A_AS_DR(pa_as_base_addr_tbl[a_as_ch]).SS.LONG = ss_reg; // Clear of SS register
    A_AS_DR(pa_as_base_addr_tbl[a_as_ch]).SS.LONG; // Dummy read

    if (a_as_state_tbl[a_as_ch] != A_AS_STATE_IDLE)
    {
        if (A_AS_SS_TXCC_BIT != (ss_reg & A_AS_SS_TXCC_BIT))
        {
            R_AFORMAT_INT_END_Isr(a_as_ch, ss_reg); // Interrupt processing of A-format
        }
    }

    R_AFORMAT_Copy_Param(a_as_ch); // Copy of driver internal variable

    // Change of state
    if (A_AS_SS_RXEND_BIT == (ss_reg & A_AS_SS_RXEND_BIT))
    {
        if (A_AS_STATE_TRANS == a_as_state_tbl[a_as_ch])
        {
            a_as_state_tbl[a_as_ch] = A_AS_STATE_IDLE;
        }
        if (A_AS_STATE_ELC_TRANS == a_as_state_tbl[a_as_ch])
        {
            a_as_state_tbl[a_as_ch] = A_AS_STATE_ELC;
        }
    }
}

void enc_a_as_ch0_int_isr(void)
{
    a_as_int_isr_common(A_AS_CH0);
    __DMB();
}

void enc_a_as_ch1_int_isr(void)
{
    a_as_int_isr_common(A_AS_CH1);
    __DMB();
}

