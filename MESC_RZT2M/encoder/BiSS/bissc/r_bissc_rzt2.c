#include "r_bissc_rzt2_if.h"

#include <BiSS/iodefine_biss.h>

#include "bsp_api.h"

#include <stdio.h>

#define BISSC_ISR_PRI0      (15)
#define BISSC_ISR_PRI1      (15)

/* BiSS-C driver Version */
#define BISSC_VERSION           (0x00010000u)

/* Register setting of BiSS-C */
#define BISS_VER_ERR            (0xFFFFFFFFu)

/* BiSS base address */
#define BISS(addr)              (*(volatile struct st_biss *)(addr))
#define BISS_BASE_ADDR0         (0xA0034000)
#define BISS_BASE_ADDR1         (0xA0234000)
#define BISS_FDAT_DR(ch)        (*(volatile BISS_FDAT *)((ch)==0 ? BASE_FDAT_ADDR_CH0 :\
                                                                   BASE_FDAT_ADDR_CH1))
#define BASE_FDAT_ADDR_CH0      (0xA0FD3000u)
#define BASE_FDAT_ADDR_CH1      (0xA0FD3200u)
#define BISS_NF_DR(ch)          (*(volatile BISS_NF *)((ch)==0 ? BASE_NF_ADDR_CH0 :\
                                                                 BASE_NF_ADDR_CH1))
#define BASE_NF_ADDR_CH0        (0xA0FD5003u)
#define BASE_NF_ADDR_CH1        (0xA0FD5017u)

/* set value of NF register */
#define BISS_INF_ON                 (1u)
#define BISS_INF_OFF                (0u)

#define BISSC_NFINTV_0              (0x00)
#define BISSC_NFINTV_1              (0x01)
#define BISSC_NFINTV_2              (0x02)
#define BISSC_NFINTV_3              (0x03)
#define BISSC_NFINTV_4              (0x04)

#define BISSC_NFSCNT_1              (0x01)
#define BISSC_NFSCNT_2              (0x02)
#define BISSC_NFSCNT_4              (0x04)
#define BISSC_NFSCNT_7              (0x07)
#define BISSC_NFSCNT_9              (0x09)
#define BISSC_NFSCNT_11             (0x0B)
#define BISSC_NFSCNT_15             (0x0F)

#define BISSC_CMD_NUM               (7)     /* Number of commands */

/* BiSS-C Status(bissc_state_t) */
#define BISSC_STATE_STOP            (0u)    /* stop(not open) */
#define BISSC_STATE_IDLE            (1u)    /* idle */
#define BISSC_STATE_POS             (2u)    /* get position */
#define BISSC_STATE_REGACS          (3u)    /* register read/write */
#define BISSC_STATE_REGACS_WT       (4u)    /* register read/write(wait position command) */
#define BISSC_STATE_SREGACS         (5u)    /* sequential register read/write */
#define BISSC_STATE_SREGACS_WT      (6u)    /* sequential register read/write(wait position command) */
#define BISSC_STATE_NUM             (7u)    /* Status max */

/* register access status for bissc_stop_req */
#define BISSC_SPREQ_NON             (0u)    /* clear stop request */
#define BISSC_SPREQ_SET             (1u)    /* set stop request */
#define BISSC_SPREQ_WAIT            (2u)    /* wait the stop processing */

/* max number of read/write for register access */
#define BISSC_REGNUM_MAX            (64u)

/* max address of register for register access */
#define BISSC_REGADR_MAX            (0x7F)

/* Data Type */
#define BISSC_MOD_SENS              (3u)    /* BiSS-C Sensor mode */
#define BISSC_MOD_REG               (4u)    /* BiSS-C Register access mode */
#define BISSC_MOD_SEQREG            (5u)    /* BiSS-C Sequential register access mode */

/* direction of register access(read or write) */
#define BISSC_REG_READ              (0u)    /* read register */
#define BISSC_REG_WRITE             (1u)    /* write register */

/* RXRST register mask pattern */
#define RXRST_WDG_MASK              (0x00000001u)
#define RXRST_CRC1_MASK             (0x00000002u)
#define RXRST_CRC2_MASK             (0x00000004u)
#define RXRST_ARM_MASK              (0x00000008u)
#define RXRST_WRN_MASK              (0x00000010u)
#define RXRST_REGWERR_MASK          (0x00000020u)
#define RXRST_RDBERR_MASK           (0x00000040u)
#define RXRST_WTBERR_MASK           (0x00000080u)
#define RXRST_STBERR_MASK           (0x00000100u)
#define RXRST_AIDLERR_MASK          (0x00000200u)
#define RXRST_REGNRD_MASK           (0x00001000u)
#define RXRST_REGEND_MASK           (0x00002000u)
#define RXRST_REGCST_MASK           (0x00004000u)
#define RXRST_ENDBT_MASK            (0x00008000u)

#define RXRST_SNSERR_MASK           (0x00000003u)
#define RXRST_REGERR_MASK           (0x000002E7u)

/* RXD2 register mask pattern */
#define RXD2_REG_MASK               (0x000000FFu)
#define RXD2_TEMP_MASK              (0x0000FF00u)

#define REG_SSCL_CLR                (0x0000B3FFu)           /* (0b1011001111111111) */

/* Index of arrays, number of arrays */
#define BISS0_INDEX                 (0)
#define BISS1_INDEX                 (1)
#define RXD0_INDEX                  (0)
#define RXD1_INDEX                  (1)
#define RXD2_INDEX                  (2)
#define RXD3_INDEX                  (3)
#define RXD_NUM                     (4)

/* Bit manipulation */
#define BISS_SET_D0SIZ(send,v)      (((send)&~BISS_D0SIZ_MASK) | ((uint32_t)(v)&BISS_D0SIZ_MASK))
#define BISS_SET_D1SIZ(send,v)      (((send)&~BISS_D1SIZ_MASK) | (((uint32_t)(v)<<BISS_D1SIZ_SHIFT)&BISS_D1SIZ_MASK))
#define BISS_SET_DTSIZ(send,v)      (((send)&~BISS_DTSIZ_MASK) | (((uint32_t)(v)<<BISS_DTSIZ_SHIFT)&BISS_DTSIZ_MASK))
#define BISS_SET_CRC1SIZ(send,v)    (((send)&~BISS_CRC1SIZ_MASK) \
                                        | (((uint32_t)(v)<<BISS_CRC1SIZ_SHIFT)&BISS_CRC1SIZ_MASK))
#define BISS_D0SIZ_MASK             (0x0000003FuL)
#define BISS_D1SIZ_MASK             (0x00003F00uL)
#define BISS_DTSIZ_MASK             (0x001F0000uL)
#define BISS_CRC1SIZ_MASK           (0x1F000000uL)
#define BISS_D1SIZ_SHIFT            (8u)
#define BISS_DTSIZ_SHIFT            (16u)
#define BISS_CRC1SIZ_SHIFT          (24u)
#define BISS_SET_WNR(send,v)        (((send)&~BISS_WNR_MASK) | ((uint32_t)(v)&BISS_WNR_MASK))
#define BISS_SET_ADR(send,v)        (((send)&~BISS_ADR_MASK) | (((uint32_t)(v)<<BISS_ADR_SHIFT)&BISS_ADR_MASK))
#define BISS_SET_ID(send,v)         (((send)&~BISS_ID_MASK) | (((uint32_t)(v)<<BISS_ID_SHIFT)&BISS_ID_MASK))
#define BISS_SET_TXD(send,v)        (((send)&~BISS_TXD_MASK) | (((uint32_t)(v)<<BISS_TXD_SHIFT)&BISS_TXD_MASK))
#define BISS_WNR_MASK               (0x00000001uL)
#define BISS_ADR_MASK               (0x000000FEuL)
#define BISS_ID_MASK                (0x00000700uL)
#define BISS_TXD_MASK               (0x00FF0000uL)
#define BISS_ADR_SHIFT              (1u)
#define BISS_ID_SHIFT               (8u)
#define BISS_TXD_SHIFT              (16u)
#define BISS_SET_ELCINE(reg,v)      ((uint8_t)(((reg)&~BISS_ELCINE_MASK) | ((v)&BISS_ELCINE_MASK)))
#define BISS_ELCINE_MASK            (0x01u)

#define PARAMETER_NOT_USED(p)       (void) ((p))

/* BiSS-C Status */
typedef uint8_t bissc_state_t;

typedef struct {
    r_biss_cmd_t    cmd;
    r_biss_err_t    (*func)(const int32_t biss_no, void *const pbuf);
} bissc_cmd_tbl_t;

typedef struct {
    uint16_t        br;
    uint8_t         nfintv;
    uint8_t         nfscnt;
} bissc_nf_tbl_t;

#ifdef __GNUC__
void bissc0_rx_int_isr(void);
void bissc1_rx_int_isr(void);
#endif /* __GNUC__ */

static bool bissc_set_reg_result(uint32_t biss_index, uint32_t rxrst, uint32_t rxd);
static void bissc_set_comresult(uint32_t biss_index);
static r_biss_err_t bissc_get_position(const int32_t biss_no, void *const pbuf);
static r_biss_err_t bissc_reg_read(const int32_t biss_no, void *const pbuf);
static r_biss_err_t bissc_reg_write(const int32_t biss_no, void *const pbuf);
static r_biss_err_t bissc_reg_seq_read(const int32_t biss_no, void *const pbuf);
static r_biss_err_t bissc_reg_seq_write(const int32_t biss_no, void *const pbuf);
static r_biss_err_t bissc_reg_access(const int32_t biss_no, const uint8_t reg_mode, const bool seq_acs,
                                    r_bissc_regreq_t *const pbiss_req);
static r_biss_err_t bissc_reg_seq_stop(const int32_t biss_no, void *const pbuf);
static r_biss_err_t bissc_reg_forcedstop(const int32_t biss_no, void *const pbuf);
static r_biss_err_t bissc_reg_init(const int32_t biss_no, r_biss_info_t* pinfo);
static uint32_t bissc_cmd_to_index(r_biss_cmd_t cmd);
static bool bissc_chk_reg_access_end(uint32_t biss_index);

/*-----------------------------------------------------------------------------*/
/* Status of BiSS-C */
static bissc_state_t bissc_state[BISS_ID_NUM] =
{
    BISSC_STATE_STOP,
    BISSC_STATE_STOP
};
static uint8_t bissc_reg_state[BISS_ID_NUM];        /* register read or write */

/* Stop request for sequential register access */
static uint8_t bissc_stop_req[BISS_ID_NUM] =
{
    BISSC_SPREQ_NON,
    BISSC_SPREQ_NON
};

/* number of read/write for register access */
static uint8_t req_data_num[BISS_ID_NUM];

/* call-back function of Sensor data */
static r_biss_isr_cb_t pbissc_sensdata_cb[BISS_ID_NUM] =
{
    NULL,
    NULL
};

/* call-back function of register access */
static r_biss_isr_cb_t pbissc_regdata_cb[BISS_ID_NUM] =
{
    NULL,
    NULL
};

/* Sensor data from BiSS-C Encoder */
static r_bissc_sensordt_t sens_result[BISS_ID_NUM];

/* Register access data from BiSS-C Encoder */
static r_bissc_regdata_t reg_result[BISS_ID_NUM];

/* pointer of Register data buff */
static uint8_t *preg_data[BISS_ID_NUM];
static uint8_t regdt_cnt[BISS_ID_NUM];

/*-----------------------------------------------------------------------------*/
/* Check Commands of R_BISSC_Control */
static r_biss_err_t bissc_cmd_chk_tbl[BISSC_STATE_NUM][BISSC_CMD_NUM] =
{
                                 /* R_BISSC_CMD_POS          , R_BISSC_CMD_REG_READ      , R_BISSC_CMD_REG_WRITE
                                  , R_BISSC_CMD_REG_SREAD    , R_BISSC_CMD_REG_SWRITE    , R_BISSC_CMD_REG_SSTOP
                                  , R_BISSC_CMD_REG_FSTOP */
/* BISSC_STATE_STOP         */   { R_BISS_ERR_ACCESS        , R_BISS_ERR_ACCESS         , R_BISS_ERR_ACCESS
                                 , R_BISS_ERR_ACCESS        , R_BISS_ERR_ACCESS         , R_BISS_ERR_ACCESS
                                 , R_BISS_ERR_ACCESS },
/* BISSC_STATE_IDLE         */   { R_BISS_SUCCESS           , R_BISS_SUCCESS            , R_BISS_SUCCESS
                                 , R_BISS_SUCCESS           , R_BISS_SUCCESS            , R_BISS_ERR_ACCESS
                                 , R_BISS_SUCCESS },
/* BISSC_STATE_POS          */   { R_BISS_ERR_BUSY          , R_BISS_ERR_BUSY           , R_BISS_ERR_BUSY
                                 , R_BISS_ERR_BUSY          , R_BISS_ERR_BUSY           , R_BISS_ERR_ACCESS
                                 , R_BISS_SUCCESS },
/* BISSC_STATE_REGACS       */   { R_BISS_ERR_BUSY          , R_BISS_ERR_BUSY           , R_BISS_ERR_BUSY
                                 , R_BISS_ERR_BUSY          , R_BISS_ERR_BUSY           , R_BISS_ERR_ACCESS
                                 , R_BISS_SUCCESS },
/* BISSC_STATE_REGACS_WT    */   { R_BISS_SUCCESS           , R_BISS_ERR_BUSY           , R_BISS_ERR_BUSY
                                 , R_BISS_ERR_BUSY          , R_BISS_ERR_BUSY           , R_BISS_ERR_ACCESS
                                 , R_BISS_SUCCESS },
/* BISSC_STATE_SREGACS      */   { R_BISS_ERR_BUSY          , R_BISS_ERR_BUSY           , R_BISS_ERR_BUSY
                                 , R_BISS_ERR_BUSY          , R_BISS_ERR_BUSY           , R_BISS_SUCCESS
                                 , R_BISS_SUCCESS },
/* BISSC_STATE_SREGACS_WT   */   { R_BISS_SUCCESS           , R_BISS_ERR_BUSY           , R_BISS_ERR_BUSY
                                 , R_BISS_ERR_BUSY          , R_BISS_ERR_BUSY           , R_BISS_SUCCESS
                                 , R_BISS_SUCCESS },
};

/* Commands and Functions of R_BISSC_Control (NULL: There is no command for the encoder) */
static  bissc_cmd_tbl_t bissc_cmd_tbl[BISSC_CMD_NUM] =
{
    /* Command Name             , Function  */
    { R_BISSC_CMD_POS           , &bissc_get_position   },
    { R_BISSC_CMD_REG_READ      , &bissc_reg_read       },
    { R_BISSC_CMD_REG_WRITE     , &bissc_reg_write      },
    { R_BISSC_CMD_REG_SREAD     , &bissc_reg_seq_read   },
    { R_BISSC_CMD_REG_SWRITE    , &bissc_reg_seq_write  },
    { R_BISSC_CMD_REG_SSTOP     , &bissc_reg_seq_stop   },
    { R_BISSC_CMD_REG_FSTOP     , &bissc_reg_forcedstop }
};

/* Base address of register for BiSS encoder */
static void * pbiss_base_adr[BISS_ID_NUM] =
{
    (void *)BISS_BASE_ADDR0,
    (void *)BISS_BASE_ADDR1
};

/* noise filter */
static  bissc_nf_tbl_t bissc_nf_tbl[R_BISSC_BR_NUM] =
{
    /* value of BR register , NFINTV(NF register)   , NFSCNT(NF register) */
    { R_BISSC_BR_10MHZ      , BISSC_NFINTV_0        , BISSC_NFSCNT_1    },
    { R_BISSC_BR_8_33MHZ    , BISSC_NFINTV_0        , BISSC_NFSCNT_1    },
    { R_BISSC_BR_4MHZ       , BISSC_NFINTV_0        , BISSC_NFSCNT_4    },
    { R_BISSC_BR_2_5MHZ     , BISSC_NFINTV_0        , BISSC_NFSCNT_7    },
    { R_BISSC_BR_1MHZ       , BISSC_NFINTV_1        , BISSC_NFSCNT_9    },
    { R_BISSC_BR_400KHZ     , BISSC_NFINTV_2        , BISSC_NFSCNT_11   },
    { R_BISSC_BR_299_40KHZ  , BISSC_NFINTV_2        , BISSC_NFSCNT_15   },
    { R_BISSC_BR_200KHZ     , BISSC_NFINTV_3        , BISSC_NFSCNT_11   },
    { R_BISSC_BR_100KHZ     , BISSC_NFINTV_4        , BISSC_NFSCNT_11   },
    { R_BISSC_BR_80_12KHZ   , BISSC_NFINTV_4        , BISSC_NFSCNT_15   }
};

r_biss_err_t R_BISSC_Open(const int32_t biss_no, r_biss_info_t* pinfo)
{
    r_biss_err_t result = R_BISS_SUCCESS;

    /* check biss_no */
    if (BISS_ID_NUM <= biss_no)
    {
        goto open_invalid_arg;
    }

    /* check pinfo */
    if (NULL == pinfo)
    {
        goto open_invalid_arg;
    }

    /* check BiSS-C status */
    if (BISSC_STATE_STOP != bissc_state[biss_no])
    {
        goto open_err_access;
    }

    /* set register for BiSS-C */
    result = bissc_reg_init(biss_no, pinfo);
    if (R_BISS_SUCCESS == result)
    {
        /* Interrupt settings */

        /* set BiSS-C Status */

       if(biss_no == 0){
          R_BSP_IrqDisable(VECTOR_NUMBER_ENCIF_INT0);
          R_BSP_IrqCfg(VECTOR_NUMBER_ENCIF_INT0, 11, NULL);
          R_BSP_IrqEnable(VECTOR_NUMBER_ENCIF_INT0);
       }else{
          R_BSP_IrqDisable(VECTOR_NUMBER_ENCIF_INT4);
          R_BSP_IrqCfg(VECTOR_NUMBER_ENCIF_INT4, 11, NULL);
          R_BSP_IrqEnable(VECTOR_NUMBER_ENCIF_INT4);
       }

        bissc_state[biss_no] = BISSC_STATE_IDLE;
    }

    goto func_open_end;

open_invalid_arg:
    result = R_BISS_ERR_INVALID_ARG;
    goto func_open_end;

open_err_access:
    result = R_BISS_ERR_ACCESS;
    goto func_open_end;

func_open_end:
    return result;
}

r_biss_err_t R_BISSC_Close(const int32_t biss_no)
{
    r_biss_err_t result = R_BISS_SUCCESS;

    /* check biss_no */
    if (BISS_ID_NUM <= biss_no)
    {
        goto bissccls_invalid_arg;
    }

    /* check BiSS-C status */
    if (BISSC_STATE_IDLE == bissc_state[biss_no])
    {
        /* Stop encoder. */
        /* set BiSS-C Status */
        bissc_state[biss_no] = BISSC_STATE_STOP;
    }
    else if (BISSC_STATE_STOP == bissc_state[biss_no])
    {
        /* Do Nothing */
    }
    else
    {
        goto bissccls_busy;
    }
    goto bissccls_end;

bissccls_busy:
    result = R_BISS_ERR_BUSY;
    goto bissccls_end;

bissccls_invalid_arg:
    result = R_BISS_ERR_INVALID_ARG;
    goto bissccls_end;

bissccls_end:
    return result;
}

uint32_t R_BISSC_GetVersion(void)
{
    return BISSC_VERSION;
}

r_biss_err_t R_BISSC_Control(const int32_t biss_no, const r_biss_cmd_t cmd, void *const pbuf)
{
    r_biss_err_t result = R_BISS_SUCCESS;
    uint32_t cmd_index;

    /* check biss_no */
    if (BISS_ID_NUM <= biss_no)
    {
        goto bisscctl_err_arg;
    }

    /* Search for function executing the command. */
    cmd_index = bissc_cmd_to_index(cmd);
    if (BISSC_CMD_NUM <= cmd_index)
    {
        goto bisscctl_err_arg;
    }

    /* check command */
    result = bissc_cmd_chk_tbl[bissc_state[biss_no]][cmd_index];
    if (R_BISS_SUCCESS != result)
    {
        goto bisscctl_end;
    }
    if (NULL == bissc_cmd_tbl[cmd_index].func)
    {
        goto bisscctl_err_arg;
    }

    /* Execute command. */
    result = bissc_cmd_tbl[cmd_index].func(biss_no, pbuf);
    goto bisscctl_end;

bisscctl_err_arg:
    result = R_BISS_ERR_INVALID_ARG;
    goto bisscctl_end;

bisscctl_end:
    return result;
}

static r_biss_err_t bissc_get_position(const int32_t biss_no, void *const pbuf)
{
    r_biss_err_t result = R_BISS_SUCCESS;
    r_bissc_sensreq_t *pbiss_req = pbuf;

    /* check biss_no */
    if (BISS_ID_NUM <= biss_no)
    {
        goto getpos_err_arg;
    }

    /* check Request information */
    if (NULL == pbiss_req)
    {
        goto getpos_err_arg;
    }

    /* check Slave ID */
    if (R_BISSC_SLAVE_7 < pbiss_req->slaveid)
    {
        goto getpos_err_arg;
    }

    /* check ELC */
    BISS(pbiss_base_adr[biss_no]).CTL.BYTE =
        BISS_SET_ELCINE(BISS(pbiss_base_adr[biss_no]).CTL.BYTE, pbiss_req->elc_enable);

    switch (bissc_state[biss_no])
    {
        case BISSC_STATE_IDLE:

            /* set BiSS-C status */
            bissc_state[biss_no] = BISSC_STATE_POS;

            /* set data type */
            BISS(pbiss_base_adr[biss_no]).MODE.BIT.MODE = BISSC_MOD_SENS;
        break;

        case BISSC_STATE_REGACS_WT:

            /* set BiSS-C status */
            bissc_state[biss_no] = BISSC_STATE_REGACS;
        break;

        case BISSC_STATE_SREGACS_WT:

            /* set BiSS-C status */
            bissc_state[biss_no] = BISSC_STATE_SREGACS;
        break;

        default:
            result = R_BISS_ERR_BUSY;
            goto getpos_end;
        break;
    }

    /* call-back Function */
    pbissc_sensdata_cb[biss_no] = pbiss_req->sdresult_cb;

    /* set slave ID */
    BISS(pbiss_base_adr[biss_no]).REGACS.LONG
        = BISS_SET_ID(BISS(pbiss_base_adr[biss_no]).REGACS.LONG, pbiss_req->slaveid);

    /* set Time-out clock(Tx/Rx Start) */
    BISS(pbiss_base_adr[biss_no]).WDG.LONG = pbiss_req->timeout_clk;
    goto getpos_end;

getpos_err_arg:
    result = R_BISS_ERR_INVALID_ARG;
    goto getpos_end;

getpos_end:
    return result;
}

static r_biss_err_t bissc_reg_read(const int32_t biss_no, void *const pbuf)
{
    r_biss_err_t result = R_BISS_SUCCESS;
    r_bissc_regreq_t *pbiss_req = pbuf;

    /* Execute register access */
    result = bissc_reg_access(biss_no, BISSC_REG_READ, false, pbiss_req);

    if (R_BISS_SUCCESS == result)
    {
        /* set BiSS-C status */
        bissc_state[biss_no] = BISSC_STATE_REGACS;
        bissc_reg_state[biss_no] = BISSC_REG_READ;

        /* set Time-out clock(Tx/Rx Start) */
        BISS(pbiss_base_adr[biss_no]).WDG.LONG = pbiss_req->timeout_clk;
    }
    else
    {
        /* Do Nothing */
    }
    return result;
}

static r_biss_err_t bissc_reg_write(const int32_t biss_no, void *const pbuf)
{
    r_biss_err_t result = R_BISS_SUCCESS;
    r_bissc_regreq_t *pbiss_req = pbuf;

    /* Execute register access */
    result = bissc_reg_access(biss_no, BISSC_REG_WRITE, false, pbiss_req);

    if (R_BISS_SUCCESS == result)
    {
        /* set BiSS-C status */
        bissc_state[biss_no] = BISSC_STATE_REGACS;
        bissc_reg_state[biss_no] = BISSC_REG_WRITE;

        /* set Time-out clock(Tx/Rx Start) */
        BISS(pbiss_base_adr[biss_no]).WDG.LONG = pbiss_req->timeout_clk;
    }
    else
    {
        /* Do Nothing */
    }
    return result;
}

static r_biss_err_t bissc_reg_seq_read(const int32_t biss_no, void *const pbuf)
{
    r_biss_err_t result = R_BISS_SUCCESS;
    r_bissc_regreq_t *pbiss_req = pbuf;

    /* Execute register access */
    result = bissc_reg_access(biss_no, BISSC_REG_READ, true, pbiss_req);

    if (R_BISS_SUCCESS == result)
    {
        /* set BiSS-C status */
        bissc_state[biss_no] = BISSC_STATE_SREGACS;
        bissc_reg_state[biss_no] = BISSC_REG_READ;

        /* set Time-out clock(Tx/Rx Start) */
        BISS(pbiss_base_adr[biss_no]).WDG.LONG = pbiss_req->timeout_clk;
    }
    else
    {
        /* Do Nothing */
    }
    return result;
}

static r_biss_err_t bissc_reg_seq_write(const int32_t biss_no, void *const pbuf)
{
    r_biss_err_t result = R_BISS_SUCCESS;
    r_bissc_regreq_t *pbiss_req = pbuf;

    /* Execute register access */
    result = bissc_reg_access(biss_no, BISSC_REG_WRITE, true, pbiss_req);

    if (R_BISS_SUCCESS == result)
    {
        /* set BiSS-C status */
        bissc_state[biss_no] = BISSC_STATE_SREGACS;
        bissc_reg_state[biss_no] = BISSC_REG_WRITE;

        /* set Time-out clock(Tx/Rx Start) */
        BISS(pbiss_base_adr[biss_no]).WDG.LONG = pbiss_req->timeout_clk;
    }
    else
    {
        /* Do Nothing */
    }
    return result;
}

static r_biss_err_t bissc_reg_seq_stop(const int32_t biss_no, void *const pbuf)
{
    PARAMETER_NOT_USED(pbuf);

    r_biss_err_t result = R_BISS_SUCCESS;

    /* check biss_no */
    if (BISS_ID_NUM <= biss_no)
    {
        goto seqstp_err_arg;
    }

    /* set stop request */
    bissc_stop_req[biss_no] = BISSC_SPREQ_SET;
    goto seqstp_end;

seqstp_err_arg:
    result = R_BISS_ERR_INVALID_ARG;
    goto seqstp_end;

seqstp_end:
    return result;
}

static r_biss_err_t bissc_reg_forcedstop(const int32_t biss_no, void *const pbuf)
{
    PARAMETER_NOT_USED(pbuf);

    r_biss_err_t result = R_BISS_SUCCESS;

    /* check biss_no */
    if (BISS_ID_NUM <= biss_no)
    {
        goto fstp_err_arg;
    }

    /* Forced stop of Register Access */
    bissc_state[biss_no] = BISSC_STATE_IDLE;
    goto fstp_end;

fstp_err_arg:
    result = R_BISS_ERR_INVALID_ARG;
    goto fstp_end;

fstp_end:
    return result;
}

static r_biss_err_t bissc_reg_access(const int32_t biss_no, const uint8_t reg_mode, const bool seq_acs,
                                    r_bissc_regreq_t *const pbiss_req)
{
    uint32_t regacs_val;
    r_biss_err_t result = R_BISS_SUCCESS;

    /* check biss_no */
    if (BISS_ID_NUM <= biss_no)
    {
        goto regacs_err_arg;
    }

    /* NULL check */
    if ((NULL == pbiss_req)||(NULL == pbiss_req->regdata))
    {
        goto regacs_err_arg;
    }

    /* check Slave ID */
    if (R_BISSC_SLAVE_7 < pbiss_req->slaveid)
    {
        goto regacs_err_arg;
    }

    /* check number of read/write register */
    if ((BISSC_REGNUM_MAX < pbiss_req->regdtnum)||(0 == pbiss_req->regdtnum))
    {
        goto regacs_err_arg;
    }

    /* check address of read/write register */
    if (BISSC_REGADR_MAX < pbiss_req->regaddress)
    {
        goto regacs_err_arg;
    }

    /* clear stop request */
    bissc_stop_req[biss_no] = BISSC_SPREQ_NON;

    /* set call-back Function */
    pbissc_sensdata_cb[biss_no]   = pbiss_req->sdresult_cb;
    pbissc_regdata_cb[biss_no]    = pbiss_req->rdresult_cb;

    /* set register address */
    regacs_val = BISS(pbiss_base_adr[biss_no]).REGACS.LONG;
    regacs_val = BISS_SET_ADR(regacs_val, pbiss_req->regaddress);

    /* save number of read/write register */
    req_data_num[biss_no] = pbiss_req->regdtnum;

    /* set register data for read/write */
    if (BISSC_REG_READ == reg_mode)
    {

        regacs_val = BISS_SET_WNR(regacs_val, BISSC_REG_READ);

    }
    else
    {

        regacs_val = BISS_SET_WNR(regacs_val, BISSC_REG_WRITE);
        regacs_val = BISS_SET_TXD(regacs_val, *pbiss_req->regdata);

    }
    preg_data[biss_no]  = pbiss_req->regdata;
    regdt_cnt[biss_no] = 0;

    /* set data type */
    if (true == seq_acs)
    {
        BISS(pbiss_base_adr[biss_no]).MODE.BIT.MODE = BISSC_MOD_SEQREG;
    }
    else
    {
        BISS(pbiss_base_adr[biss_no]).MODE.BIT.MODE = BISSC_MOD_REG;
    }

    /* set slave ID */
    regacs_val = BISS_SET_ID(regacs_val, pbiss_req->slaveid);
    BISS(pbiss_base_adr[biss_no]).REGACS.LONG = regacs_val;

    goto regacs_end;

regacs_err_arg:
    result = R_BISS_ERR_INVALID_ARG;
    goto regacs_end;

regacs_end:
    return result;
}

static r_biss_err_t bissc_reg_init(const int32_t biss_no, r_biss_info_t* pinfo)
{
    r_biss_err_t result = R_BISS_SUCCESS;
    uint32_t index;
    uint32_t size_val;

    /* check biss_no */
    if (BISS_ID_NUM <= biss_no)
    {
        goto error_arg;
    }

    /* check pinfo */
    if (NULL == pinfo)
    {
        goto error_arg;
    }

    /* set clock frequency */
    BISS(pbiss_base_adr[biss_no]).BR1.WORD = pinfo->clk_freq;
    
    /* set crc size */
    size_val = BISS(pbiss_base_adr[biss_no]).SIZE.LONG;
    size_val = BISS_SET_CRC1SIZ(size_val, pinfo->crc_info.crc_size);

    /* set crc polynomial */
    BISS(pbiss_base_adr[biss_no]).CPOLY.WORD            = pinfo->crc_info.crc_polynomial;

    /* set crc start value */
    BISS(pbiss_base_adr[biss_no]).CINIT.WORD            = pinfo->crc_info.crc_start_value;

    /* set multi-turn data size */
    size_val = BISS_SET_D1SIZ(size_val, pinfo->mtdata_size);

    /* set single-turn data size */
    size_val = BISS_SET_D0SIZ(size_val, pinfo->stdata_size);

    /* set Alignment data size */
    size_val = BISS_SET_DTSIZ(size_val, pinfo->aldata_size);
    BISS(pbiss_base_adr[biss_no]).SIZE.LONG = size_val;

    /* NoiseFilter Setting */
    for (index = 0; index < R_BISSC_BR_NUM; index++)
    {
        if (pinfo->clk_freq == bissc_nf_tbl[index].br)
        {
            break;
        }
    }
    if (R_BISSC_BR_NUM > index)
    {

        BISS_NF_DR(biss_no).BYTE = (uint8_t)( bissc_nf_tbl[index].nfintv | (bissc_nf_tbl[index].nfscnt<<4) );

    }

    goto func_end;

error_arg:
    result = R_BISS_ERR_INVALID_ARG;
    goto func_end;

func_end:
    return result;
}

static uint32_t bissc_cmd_to_index(r_biss_cmd_t cmd)
{
    uint32_t index;

    for (index = 0; index < BISSC_CMD_NUM; index++)
    {
        if (cmd == bissc_cmd_tbl[index].cmd)
        {
            break;
        }
    }

    return index;
}

static bool bissc_chk_reg_access_end(uint32_t biss_index)
{
    bool regendf = false;

    regdt_cnt[biss_index] = (uint8_t)(regdt_cnt[biss_index] + 1);
    if (regdt_cnt[biss_index] >= req_data_num[biss_index])
    {
        /* complete the register access of the user's setting byte */
        reg_result[biss_index].result  = R_BISSC_SUCCESS;

        /* end register access */
        regendf = true;
    }
    else if ((req_data_num[biss_index] - 1) == regdt_cnt[biss_index])
    {
        /* set 1byte register access */
        BISS(pbiss_base_adr[biss_index]).MODE.BIT.MODE = BISSC_MOD_REG;

        /* continues register access */
    }
    else if (BISSC_SPREQ_SET == bissc_stop_req[biss_index])
    {
        /* stop request of sequential register access */
        bissc_stop_req[biss_index] = BISSC_SPREQ_WAIT;

        /* set 1byte register access */
        BISS(pbiss_base_adr[biss_index]).MODE.BIT.MODE = BISSC_MOD_REG;

        /* continues register access */
    }
    else if (BISSC_SPREQ_WAIT == bissc_stop_req[biss_index])
    {
        /* complete the stop processing of sequential register access */
        reg_result[biss_index].result  = R_BISSC_SUCCESS;
        bissc_stop_req[biss_index]     = BISSC_SPREQ_NON;

        /* end register access */
        regendf = true;
    }
    else
    {
        /* continues register access */
    }
    return regendf;
}

static bool bissc_set_reg_result(uint32_t biss_index, uint32_t rxrst, uint32_t rxd)
{
    bool regendf = false;

    if ((rxrst & RXRST_REGERR_MASK) == 0u)
    {
        /* register access end(1byte) */
        if (BISSC_REG_READ == bissc_reg_state[biss_index])
        {
            *(preg_data[biss_index] + regdt_cnt[biss_index]) = (rxd & RXD2_REG_MASK);
        }
        else
        {
            BISS(pbiss_base_adr[biss_index]).REGACS.BIT.TXD = *((preg_data[biss_index] + regdt_cnt[biss_index]) + 1);
        }

        regendf = bissc_chk_reg_access_end(biss_index);
        
        if ((rxrst & RXRST_STBERR_MASK) != 0u)
        {
            reg_result[biss_index].result      = R_BISSC_SUCCESS;
            regendf = true;
        }
    }
    else
    {
        if ((rxrst & RXRST_REGCST_MASK) == 0u)
        {
            reg_result[biss_index].result      = R_BISSC_REQ_ERR;
        }
        else
        {
            reg_result[biss_index].result      = R_BISSC_REQ_ERR_WR;
        }

        /* end register access */
        regendf = true;
    }
    return regendf;
}

static void bissc_set_comresult(uint32_t biss_index)
{
    uint32_t rxrst;
    uint32_t rxd[RXD_NUM];
    bool regend_flg = false;

    /* save sensor data and register data */
    rxrst = (uint32_t)BISS(pbiss_base_adr[biss_index]).SS.WORD;
    rxd[RXD0_INDEX] = BISS(pbiss_base_adr[biss_index]).RXD0.LONG;
    rxd[RXD1_INDEX] = BISS(pbiss_base_adr[biss_index]).RXD1.LONG;
    rxd[RXD2_INDEX] = BISS(pbiss_base_adr[biss_index]).RXD2.WORD;
    rxd[RXD3_INDEX] = BISS(pbiss_base_adr[biss_index]).IDL.BYTE;

    /* get sensor data */
    if ((rxrst & RXRST_SNSERR_MASK) == 0u)
    {
        sens_result[biss_index].result = R_BISSC_SUCCESS;
    }
    else
    {
        sens_result[biss_index].result = R_BISSC_REQ_ERR;
    }
    sens_result[biss_index].stdata     = rxd[RXD0_INDEX];
    sens_result[biss_index].mtdata     = rxd[RXD1_INDEX];
    sens_result[biss_index].timeout    = ((rxrst & RXRST_WDG_MASK) != 0u);
    sens_result[biss_index].crc1_err   = ((rxrst & RXRST_CRC1_MASK) != 0u);
    sens_result[biss_index].alarm      = ((rxrst & RXRST_ARM_MASK) != 0u);
    sens_result[biss_index].warning    = ((rxrst & RXRST_WRN_MASK) != 0u);
    sens_result[biss_index].notready   = ((rxrst & RXRST_REGNRD_MASK) != 0u);

    /* Call the user's callback function of sensor data */
    if (NULL != pbissc_sensdata_cb[biss_index])
    {
        pbissc_sensdata_cb[biss_index](&sens_result[biss_index]);
    }

    /* get register access data */
    if ((BISSC_STATE_REGACS  == bissc_state[biss_index]) ||
        (BISSC_STATE_SREGACS == bissc_state[biss_index]))
    {
        /* check register data */
        if (((rxrst & RXRST_REGERR_MASK) != 0u) || ((rxrst & RXRST_REGEND_MASK) != 0u)
             || ((rxrst & RXRST_STBERR_MASK) != 0u))
        {
            regend_flg = bissc_set_reg_result(biss_index, rxrst, rxd[RXD2_INDEX]);
        }

        /* set the register data */
        if (true == regend_flg)
        {
            reg_result[biss_index].idl          = (uint8_t)rxd[RXD3_INDEX];
            reg_result[biss_index].combyte      = regdt_cnt[biss_index];
            reg_result[biss_index].timeout      = ((rxrst & RXRST_WDG_MASK) != 0u);
            reg_result[biss_index].crc1_err     = ((rxrst & RXRST_CRC1_MASK) != 0u);
            reg_result[biss_index].crc2_err     = ((rxrst & RXRST_CRC2_MASK) != 0u);
            reg_result[biss_index].alarm        = ((rxrst & RXRST_ARM_MASK) != 0u);
            reg_result[biss_index].warning      = ((rxrst & RXRST_WRN_MASK) != 0u);
            reg_result[biss_index].regw_err     = ((rxrst & RXRST_REGWERR_MASK) != 0u);
            reg_result[biss_index].readbit_err  = ((rxrst & RXRST_RDBERR_MASK) != 0u);
            reg_result[biss_index].writebit_err = ((rxrst & RXRST_WTBERR_MASK) != 0u);
            reg_result[biss_index].stopbit_err  = ((rxrst & RXRST_STBERR_MASK) != 0u);
            reg_result[biss_index].cds_err      = ((rxrst & RXRST_AIDLERR_MASK) != 0u);

            /* Call the user's callback function of register data */
            if (NULL != pbissc_regdata_cb[biss_index])
            {
                pbissc_regdata_cb[biss_index](&reg_result[biss_index]);
            }

            /* set BiSS-C status */
            bissc_state[biss_index] = BISSC_STATE_IDLE;
        }
        else
        {
            /* set BiSS-C status */
            if (BISSC_STATE_REGACS  == bissc_state[biss_index])
            {
                bissc_state[biss_index] = BISSC_STATE_REGACS_WT;
            }
            else
            {
                bissc_state[biss_index] = BISSC_STATE_SREGACS_WT;
            }
        }
    }
    else if (BISSC_STATE_POS  == bissc_state[biss_index])
    {
        /* case BISSC_STATE_POS */
        bissc_state[biss_index] = BISSC_STATE_IDLE;
    }
    else
    {
        /* case BISSC_STATE_STOP,BISSC_STATE_IDLE,BISSC_STATE_REGACS_WT,BISSC_STATE_SREGACS_WT */
        /* Do Nothing */
    }

    /* clear the Interrupt factor */
    BISS(pbiss_base_adr[biss_index]).SSCL.WORD = (uint16_t)REG_SSCL_CLR;

    /* SS register dummy read */
    rxrst = (uint32_t)BISS(pbiss_base_adr[biss_index]).SS.WORD;

    return;
}

void bissc0_rx_int_isr(void)
{
    bissc_set_comresult(BISS0_INDEX);
    __DMB();
}

void bissc1_rx_int_isr(void)
{
    bissc_set_comresult(BISS1_INDEX);
    __DMB();
}

