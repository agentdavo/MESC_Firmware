#include "r_fac_rzt2_if.h"

#include "iodefine_fac.h"

#include <bsp_api.h>

#include <ecl/r_ecl_rzt2_if.h>

#include <stdio.h>

#define R_FAC_TX_ID_E2PROM_W    (6)
#define R_FAC_TX_ID_E2PROM_R    (0x0D)

/* FA-CODER driver Version */
#define FAC_VERSION             (0x00000002u)   /* Version of Encoder I/F */

#define FAC_ID_NUM              (2)             /* Number of configuration */

#define FAC_CMD_NUM             (4)             /* Number of commands */

/* ID */
#define FAC_ID_0                (R_ECL_CH_0)
#define FAC_ID_1                (R_ECL_CH_1)
#define FAC0_INDEX              (0)
#define FAC1_INDEX              (1)

/* Base address of register */
#define FAC0_BASE_ADR       (0xA011C100)    /* Base address of FA-CODER for channel 0 */
#define FAC1_BASE_ADR       (0xA031C100)    /* Base address of FA-CODER for channel 1 */
#define FAC0_NF_ADR         (0xA0FD5000)
#define FAC1_NF_ADR         (0xA0FD5014)

#define FAC(adr)                (*(volatile struct st_fac *)(adr))  /* Register of FA-CODER */
#define FAC_NF(adr)             (*(volatile st_fac_nf *)(adr))

/* TXADF register bit */
#define TXADF_ADR_BIT           (0)
#define TXADF_BUSY_BIT          (7)

/* TXEDF register bit */
#define TXEDF_TX_DATA_BIT       (0)

/* TX register bit */
#define TX_ID_BIT               (0)
#define TX_DFNUM_BIT            (4)
#define TX_EEPEN_BIT            (8) /* preliminary */
#define TX_EEPDIR_BIT           (9) /* preliminary */

/* RXRESULT register mask pattern */
#define RXRESULT_RSE_MASK       (0x00000001u)
#define RXRESULT_IDE_MASK       (0x00000002u)
#define RXRESULT_EBUSY_MASK     (0x00000004u)
#define RXRESULT_RXID_MASK      (0x00000078u)
#define RXRESULT_RXIDP_MASK     (0x00000080u)
#define RXRESULT_RXSFIC_MASK    (0x00000F00u)
#define RXRESULT_RXSFEA_MASK    (0x00003000u)
#define RXRESULT_RXSFCA_MASK    (0x0000C000u)
#define RXRESULT_CRC_MASK       (0x00FF0000u)
#define RXRESULT_CONTE_MASK     (0x01000000u)
#define RXRESULT_CRCE_MASK      (0x02000000u)
#define RXRESULT_FOME_MASK      (0x04000000u)
#define RXRESULT_SFOME_MASK     (0x08000000u)
#define RXRESULT_TIMOTE_MASK    (0x10000000u)
#define RXRESULT_RXEDFE_MASK    (0x20000000u)
#define RXRESULT_RXADFE_MASK    (0x40000000u)
#define RXRESULT_DFOVFE_MASK    (0x80000000u)

/* RXRESULT register bit */
#define RXRESULT_RSE_BIT        (0)
#define RXRESULT_IDE_BIT        (1)
#define RXRESULT_RXID_BIT       (3)
#define RXRESULT_RXIDP_BIT      (7)
#define RXRESULT_RXSFIC_BIT     (8)
#define RXRESULT_RXSFEA_BIT     (12)
#define RXRESULT_RXSFCA_BIT     (14)
#define RXRESULT_CRC_BIT        (16)
#define RXRESULT_CONTE_BIT      (24)
#define RXRESULT_CRCE_BIT       (25)
#define RXRESULT_FOME_BIT       (26)
#define RXRESULT_SFOME_BIT      (27)
#define RXRESULT_TIMOTE_BIT     (28)

/* RXDn register mask pattern */
#define RXD0_RXDF0_MASK         (0x000000FFu)
#define RXD0_RXDF1_MASK         (0x0000FF00u)
#define RXD0_RXDF2_MASK         (0x00FF0000u)
#define RXD0_RXDF3_MASK         (0xFF000000u)
#define RXD1_RXDF4_MASK         (0x000000FFu)
#define RXD1_RXDF5_MASK         (0x0000FF00u)
#define RXD1_RXDF6_MASK         (0x00FF0000u)
#define RXD1_RXDF7_MASK         (0xFF000000u)

/* RXDn register bit */
#define RXD0_RXDF0_BIT          (0)
#define RXD0_RXDF1_BIT          (8)
#define RXD0_RXDF2_BIT          (16)
#define RXD0_RXDF3_BIT          (24)
#define RXD1_RXDF4_BIT          (0)
#define RXD1_RXDF5_BIT          (8)
#define RXD1_RXDF6_BIT          (16)
#define RXD1_RXDF7_BIT          (24)

/* Index of arrays, number of arrays */
#define RXD0_INDEX              (0)
#define RXD1_INDEX              (1)
#define RXD_NUM                 (2)
#define RXDF0_INDEX             (0)
#define RXDF1_INDEX             (1)
#define RXDF2_INDEX             (2)
#define RXDF3_INDEX             (3)
#define RXDF4_INDEX             (4)
#define RXDF5_INDEX             (5)
#define RXDF6_INDEX             (6)
#define RXDF7_INDEX             (7)
#define RXDF0_RXD_INDEX         (0)
#define RXDF1_RXD_INDEX         (0)
#define RXDF2_RXD_INDEX         (0)
#define RXDF3_RXD_INDEX         (0)
#define RXDF4_RXD_INDEX         (1)
#define RXDF5_RXD_INDEX         (1)
#define RXDF6_RXD_INDEX         (1)
#define RXDF7_RXD_INDEX         (1)

/* Boundary of ID and DFNUM and ADF */
#define REQ_ID_MAX              (15)
#define REQ_DFNUM_MIN           (1)
#define REQ_DFNUM_MAX           (FAC_RXDF_MAX)
#define FAC_RXDF_MAX            (8)

#define FAC_NF_VAL0             (0x1500000C)
#define FAC_NF_VAL1             (0x1510000C)

/* set value of NF register */
#define FAC_NFINTV_25MHZ_40NS   (0)

#define FAC_INF_ON              (1u)
#define FAC_INF_OFF             (0u)

/* 2.5MHz 40ns 0 7 40ns */
#define FAC_NFSCNT_25MHZ_40NS   (0x07)

#define FAC_ELC_DISABLE         (0)
#define FAC_ELC_ENABLE          (1)

#define PARAMETER_NOT_USED(p)       (void)((p))

typedef r_fac_err_t (*r_fac_control_func_t)(const int32_t fac_no, void *const pbuf);

/* State of FA-CODER */
typedef enum fac_state_e
{
    FAC_STATE_STOP = 0,
    FAC_STATE_IDLE,
    FAC_STATE_REQ,
    FAC_STATE_E2PROM,
    FAC_STATE_ELC
} fac_state_t;

static uint32_t fac_id_to_index(const int32_t id);
static uint32_t fac_cmd_to_index(r_fac_cmd_t cmd);
static r_fac_err_t fac_req(const int32_t fac_no, void *const pbuf);
static r_fac_err_t fac_e2prom(const int32_t fac_no, void *const pbuf);
static r_fac_err_t fac_elctimer(const int32_t fac_no, void *const pbuf);
static r_fac_err_t fac_elcstop(const int32_t fac_no, void *const pbuf);
static void fac_set_result(const uint32_t fac_index);

void enc_fac_ch0_int_isr(void);
void enc_fac_ch1_int_isr(void);

/* Configurations of encoder */
static int32_t fac_id_tbl[FAC_ID_NUM] =
{
    FAC_ID_0,
    FAC_ID_1
};

/* Table of base address(NULL: There is no channel) */
static void * pfac_base_adr_tbl[FAC_ID_NUM] =
{
    (void *)FAC0_BASE_ADR,
    (void *)FAC1_BASE_ADR
};

static void * pfac_nf_adr_tbl[FAC_ID_NUM] =
{
    (void *)FAC0_NF_ADR,
    (void *)FAC1_NF_ADR
};

/* Commands of R_FAC_Control */
static  r_fac_cmd_t fac_cmd_tbl[FAC_CMD_NUM] =
{
    R_FAC_CMD_REQ, R_FAC_CMD_E2PROM, R_FAC_CMD_ELCTIMER, R_FAC_CMD_ELCSTOP
};

/* Functions fo R_FAC_Control commands (NULL: There is no command for the encoder) */
static  r_fac_control_func_t fac_control_func_tbl[FAC_CMD_NUM] =
{
    &fac_req, &fac_e2prom, &fac_elctimer, &fac_elcstop
};

static fac_state_t fac_state[FAC_ID_NUM] =
{
    FAC_STATE_STOP,
    FAC_STATE_STOP
};

//static uint32_t fac_isr_id[FAC_ID_NUM];
static r_fac_req_result_cb_t pfac_req_result_cb[FAC_ID_NUM] =
{
    NULL,
    NULL
};
static r_fac_e2prom_result_cb_t pfac_e2prom_result_cb[FAC_ID_NUM] =
{
    NULL,
    NULL
};

r_fac_err_t R_FAC_Open(const int32_t id)
{
    r_fac_err_t result = R_FAC_SUCCESS;
    uint32_t id_index;

    /* check index */
    id_index = fac_id_to_index(id);
    if (FAC_ID_NUM <= id_index)
    {
        goto error_invalid_arg;
    }

    if (FAC_STATE_STOP != fac_state[id_index])
    {
        goto error_already_open;   
    }

    if(id_index == 0){
      R_BSP_IrqDisable(VECTOR_NUMBER_ENCIF_INT0);
      R_BSP_IrqCfg(VECTOR_NUMBER_ENCIF_INT0, 11, NULL);
      R_BSP_IrqEnable(VECTOR_NUMBER_ENCIF_INT0);    

      FAC_NF(pfac_nf_adr_tbl[id_index]).LONG = FAC_NF_VAL0;
    }else{
      R_BSP_IrqDisable(VECTOR_NUMBER_ENCIF_INT4);
      R_BSP_IrqCfg(VECTOR_NUMBER_ENCIF_INT4, 11, NULL);
      R_BSP_IrqEnable(VECTOR_NUMBER_ENCIF_INT4);    

      FAC_NF(pfac_nf_adr_tbl[id_index]).LONG = FAC_NF_VAL1;
    }

    fac_state[id_index] = FAC_STATE_IDLE;

    goto func_end;
    
error_already_open:
    result = R_FAC_ERR_ACCESS;
    goto func_end;
    
error_invalid_arg:
    result = R_FAC_ERR_INVALID_ARG;
    goto func_end;

func_end:
    return result;
}

r_fac_err_t R_FAC_Close(const int32_t id)
{
    r_fac_err_t result = R_FAC_SUCCESS;
    uint32_t id_index;

    /* check index */
    id_index = fac_id_to_index(id);
    if (FAC_ID_NUM <= id_index)
    {
        goto error_invalid_arg;
    }

    switch(fac_state[id_index])
    {
        case FAC_STATE_STOP:

            /* Do Nothing */
        break;
        case FAC_STATE_IDLE:

            /* Stop encoder. */
            if(id_index == 0){
              R_BSP_IrqDisable(VECTOR_NUMBER_ENCIF_INT0);   
            }else{
              R_BSP_IrqDisable(VECTOR_NUMBER_ENCIF_INT4);   
            }            
            
            /* set FA-CODER Status */
            fac_state[id_index] = FAC_STATE_STOP;
        break;
        case FAC_STATE_E2PROM:
        case FAC_STATE_REQ:
            result = R_FAC_ERR_BUSY;
        break;
        default:
            result = R_FAC_ERR_ACCESS;
        break;
    }
    goto func_end;

error_invalid_arg:
    result = R_FAC_ERR_INVALID_ARG;
    goto func_end;

func_end:
    return result;
}

r_fac_err_t R_FAC_Control(const int32_t id, const r_fac_cmd_t cmd, void *const pbuf)
{
    r_fac_err_t result = R_FAC_SUCCESS;
    uint32_t cmd_index;
    uint32_t id_index;

    /* check index */
    id_index = fac_id_to_index(id);
    if (FAC_ID_NUM <= id_index)
    {
        goto error_invalid_arg;
    }
    
    if (FAC_STATE_STOP == fac_state[id_index])
    {
        goto error_not_open;
    }
    else
    {
        /* Do Noting */
    }

    /* Search for function executing the command. */
    cmd_index = fac_cmd_to_index(cmd);
    if (FAC_CMD_NUM <= cmd_index)
    {
        goto error_invalid_arg;
    }

    /* check command */
    if (NULL == fac_control_func_tbl[cmd_index])
    {
        goto error_invalid_arg;
    }

    /* Execute command. */
    result = fac_control_func_tbl[cmd_index]((int32_t)id_index, pbuf);
    
    goto func_end;
    
error_not_open:
    result = R_FAC_ERR_ACCESS;
    goto func_end;

error_invalid_arg:
    result = R_FAC_ERR_INVALID_ARG;
    goto func_end;
    
func_end:
    return result;
}

uint32_t R_FAC_GetVersion(void)
{
    return FAC_VERSION;
}

static uint32_t fac_id_to_index(const int32_t id)
{
    uint32_t fac_no;

    for (fac_no = 0; fac_no < FAC_ID_NUM; fac_no++)
    {
        if (id == fac_id_tbl[fac_no])
        {
            break;
        }
    }

    return fac_no;
}

static uint32_t fac_cmd_to_index(r_fac_cmd_t cmd)
{
    uint32_t index;

    for (index = 0; index < FAC_CMD_NUM; index++)
    {
        if (cmd == fac_cmd_tbl[index])
        {
            break;
        }
    }

    return index;
}

static r_fac_err_t fac_req(const int32_t fac_no, void *const pbuf)
{
    r_fac_err_t result = R_FAC_SUCCESS;
    r_fac_req_t * pfac_req = pbuf;
    uint8_t fac_id;
    
    if (NULL == pfac_req)
    {
        goto error_invalid_arg;
    }
    
    fac_id = pfac_req->txid;
    
    /* check upper limit of ID. */
    if (REQ_ID_MAX < fac_id)
    {
        goto error_invalid_arg;
    }
    
    /* check range of data field number. */
    if ((REQ_DFNUM_MIN > pfac_req->dfnum)
        || (REQ_DFNUM_MAX < pfac_req->dfnum))
    {
        goto error_invalid_arg;
    }
    
    switch ((int32_t)fac_state[fac_no])
    {
        case (int32_t)FAC_STATE_IDLE:

            /* set address for callback function */
            pfac_req_result_cb[fac_no] = pfac_req->presult_cb;

            /* set state of control to request */
            fac_state[fac_no] = FAC_STATE_REQ;

            /* set timeout */
            FAC(pfac_base_adr_tbl[fac_no]).TIMOT.BIT.TIMOT = pfac_req->timotn;

            /* Communication start when TX register was written. */
            FAC(pfac_base_adr_tbl[fac_no]).TX.WORD
                = (uint16_t)((0 << TX_EEPEN_BIT) | (pfac_req->dfnum << TX_DFNUM_BIT) | (pfac_req->txid << TX_ID_BIT));
        break;
        case (int32_t)FAC_STATE_REQ:
        case (int32_t)FAC_STATE_E2PROM:
        case (int32_t)FAC_STATE_ELC:
            result = R_FAC_ERR_BUSY;
        break;
        default:
            result = R_FAC_ERR_ACCESS;
        break;
    }
    
    goto func_end;
    
error_invalid_arg:
    result = R_FAC_ERR_INVALID_ARG;
    goto func_end;

func_end:
    return result;
}

static r_fac_err_t fac_e2prom(const int32_t fac_no, void *const pbuf)
{
    r_fac_err_t result = R_FAC_SUCCESS;
    r_fac_e2prom_data_t * pfac_e2prom = pbuf;
    
    if (NULL == pfac_e2prom)
    {
        goto error_invalid_arg;
    }
    
    if ((R_FAC_E2PROM_READ != pfac_e2prom->dir) && (R_FAC_E2PROM_WRITE != pfac_e2prom->dir))
    {
        goto error_invalid_arg;
    }
    
    switch ((int32_t)fac_state[fac_no])
    {
        case (int32_t)FAC_STATE_IDLE:

            /* set address for callback function */
            pfac_e2prom_result_cb[fac_no] = pfac_e2prom->presult_cb;

            /* set state of control to e2prom */
            fac_state[fac_no] = FAC_STATE_E2PROM;

            /* set timeout */
            FAC(pfac_base_adr_tbl[fac_no]).TIMOT.BIT.TIMOT = pfac_e2prom->timotn;

            /* set TXADF register */
            FAC(pfac_base_adr_tbl[fac_no]).TXADF.BYTE
                = (0 << TXADF_BUSY_BIT) | ((uint8_t)pfac_e2prom->adr << TXADF_ADR_BIT);

            /* set TXEDF register */
            FAC(pfac_base_adr_tbl[fac_no]).TXEDF.BYTE
                = (pfac_e2prom->data << TXEDF_TX_DATA_BIT);

            /* Communication start when TX register was written. */
            if (R_FAC_E2PROM_READ == pfac_e2prom->dir)
            {
                FAC(pfac_base_adr_tbl[fac_no]).TX.WORD
                    = ((0 << TX_EEPDIR_BIT) | (1 << TX_EEPEN_BIT)) | (R_FAC_TX_ID_E2PROM_R << TX_ID_BIT);
            }
            else
            {
                FAC(pfac_base_adr_tbl[fac_no]).TX.WORD
                    = ((1 << TX_EEPDIR_BIT) | (1 << TX_EEPEN_BIT)) | (R_FAC_TX_ID_E2PROM_W << TX_ID_BIT);
            }
        break;
        case (int32_t)FAC_STATE_REQ:
        case (int32_t)FAC_STATE_E2PROM:
        case (int32_t)FAC_STATE_ELC:
            result = R_FAC_ERR_BUSY;
        break;
        default:
            result = R_FAC_ERR_ACCESS;
        break;
    }
    
    goto func_end;
    
error_invalid_arg:
    result = R_FAC_ERR_INVALID_ARG;
    goto func_end;

func_end:
    return result;
}

static r_fac_err_t fac_elctimer(const int32_t fac_no, void *const pbuf)
{
    r_fac_err_t result = R_FAC_SUCCESS;
    r_fac_req_t * pfac_req = pbuf;
    uint8_t fac_id;
    
    if (NULL == pfac_req)
    {
        goto error_invalid_arg;
    }
    
    fac_id = pfac_req->txid;
    
    /* check upper limit of ID. */
    if (REQ_ID_MAX < fac_id)
    {
        goto error_invalid_arg;
    }
    
    /* check range of data field number. */
    if ((REQ_DFNUM_MIN > pfac_req->dfnum)
        || (REQ_DFNUM_MAX < pfac_req->dfnum))
    {
        goto error_invalid_arg;
    }
    
    switch ((int32_t)fac_state[fac_no])
    {
        case (int32_t)FAC_STATE_IDLE:

            /* set address for callback function */
            pfac_req_result_cb[fac_no] = pfac_req->presult_cb;

            /* set state of control to request */
            fac_state[fac_no] = FAC_STATE_ELC;

            /* set timeout */
            FAC(pfac_base_adr_tbl[fac_no]).TIMOT.BIT.TIMOT = pfac_req->timotn;

            /* set option register */
            FAC(pfac_base_adr_tbl[fac_no]).OPT.WORD = FAC_ELC_ENABLE;

            /* Communication start when TX register was written. */
            FAC(pfac_base_adr_tbl[fac_no]).TX.WORD
                = (uint16_t)((0 << TX_EEPEN_BIT) | (pfac_req->dfnum << TX_DFNUM_BIT) | (pfac_req->txid << TX_ID_BIT));
        break;
        case (int32_t)FAC_STATE_REQ:
        case (int32_t)FAC_STATE_E2PROM:
        case (int32_t)FAC_STATE_ELC:
            result = R_FAC_ERR_BUSY;
        break;
        default:
            result = R_FAC_ERR_ACCESS;
        break;
    }
    
    goto func_end;
    
error_invalid_arg:
    result = R_FAC_ERR_INVALID_ARG;
    goto func_end;

func_end:
    return result;
}

static r_fac_err_t fac_elcstop(const int32_t fac_no, void *const pbuf)
{
    PARAMETER_NOT_USED(pbuf);
    r_fac_err_t result = R_FAC_SUCCESS;
    
    switch ((int32_t)fac_state[fac_no])
    {
        case (int32_t)FAC_STATE_ELC:

            /* clear address for callback function */
            pfac_req_result_cb[fac_no] = NULL;

            /* set state of control to request */
            fac_state[fac_no] = FAC_STATE_IDLE;

            /* set option register */
            FAC(pfac_base_adr_tbl[fac_no]).OPT.WORD = FAC_ELC_DISABLE;

        break;
        case (int32_t)FAC_STATE_IDLE:
        case (int32_t)FAC_STATE_REQ:
        case (int32_t)FAC_STATE_E2PROM:
        default:
            result = R_FAC_ERR_ACCESS;
        break;
    }
    
    goto func_end;
    
func_end:
    return result;
}

static void fac_set_result(const uint32_t fac_index)
{
    uint32_t rxresult;
    uint32_t rxd[RXD_NUM];
    static r_fac_result_t fac_rx_result[FAC_ID_NUM];
    static uint8_t rxdf[FAC_ID_NUM][FAC_RXDF_MAX];

    /* Clear interrupt when RXRESULT was read. */
    rxresult = FAC(pfac_base_adr_tbl[fac_index]).RXRESULT.LONG;

    rxd[RXD0_INDEX] = FAC(pfac_base_adr_tbl[fac_index]).RXD0.LONG;
    rxd[RXD1_INDEX] = FAC(pfac_base_adr_tbl[fac_index]).RXD1.LONG;

    if ((((rxresult & RXRESULT_RSE_MASK) == 0u) && ((rxresult & RXRESULT_IDE_MASK) == 0u))
      && ((rxresult & RXRESULT_EBUSY_MASK) == 0u))
    {
        fac_rx_result[fac_index].result = R_FAC_RX_SUCCESS;
    }
    else
    {
        fac_rx_result[fac_index].result = R_FAC_RX_ERR;
    }
    fac_rx_result[fac_index].rse    = ((rxresult & RXRESULT_RSE_MASK) != 0u);
    fac_rx_result[fac_index].ide    = ((rxresult & RXRESULT_IDE_MASK) != 0u);
    fac_rx_result[fac_index].ebusy  = ((rxresult & RXRESULT_EBUSY_MASK) != 0u);
    fac_rx_result[fac_index].rxid   = (uint8_t)((rxresult & RXRESULT_RXID_MASK) >> RXRESULT_RXID_BIT);
    fac_rx_result[fac_index].rxidp  = (uint8_t)((rxresult & RXRESULT_RXIDP_MASK) >> RXRESULT_RXIDP_BIT);
    fac_rx_result[fac_index].rxsfic = (uint8_t)((rxresult & RXRESULT_RXSFIC_MASK) >> RXRESULT_RXSFIC_BIT);
    fac_rx_result[fac_index].rxsfea = (uint8_t)((rxresult & RXRESULT_RXSFEA_MASK) >> RXRESULT_RXSFEA_BIT);
    fac_rx_result[fac_index].rxsfca = (uint8_t)((rxresult & RXRESULT_RXSFCA_MASK) >> RXRESULT_RXSFCA_BIT);
    fac_rx_result[fac_index].crc    = (uint8_t)((rxresult & RXRESULT_CRC_MASK) >> RXRESULT_CRC_BIT);
    fac_rx_result[fac_index].conte  = ((rxresult & RXRESULT_CONTE_MASK) != 0u);
    fac_rx_result[fac_index].crce   = ((rxresult & RXRESULT_CRCE_MASK) != 0u);
    fac_rx_result[fac_index].fome   = ((rxresult & RXRESULT_FOME_MASK) != 0u);
    fac_rx_result[fac_index].sfome  = ((rxresult & RXRESULT_SFOME_MASK) != 0u);
    fac_rx_result[fac_index].timote = ((rxresult & RXRESULT_TIMOTE_MASK) != 0u);
    fac_rx_result[fac_index].rxedfe = ((rxresult & RXRESULT_RXEDFE_MASK) != 0u);;
    fac_rx_result[fac_index].rxadfe = ((rxresult & RXRESULT_RXADFE_MASK) != 0u);;
    fac_rx_result[fac_index].dfovfe = ((rxresult & RXRESULT_DFOVFE_MASK) != 0u);;

    rxdf[fac_index][RXDF0_INDEX] = (uint8_t)((rxd[RXDF0_RXD_INDEX] & RXD0_RXDF0_MASK) >> RXD0_RXDF0_BIT);
    rxdf[fac_index][RXDF1_INDEX] = (uint8_t)((rxd[RXDF1_RXD_INDEX] & RXD0_RXDF1_MASK) >> RXD0_RXDF1_BIT);
    rxdf[fac_index][RXDF2_INDEX] = (uint8_t)((rxd[RXDF2_RXD_INDEX] & RXD0_RXDF2_MASK) >> RXD0_RXDF2_BIT);
    rxdf[fac_index][RXDF3_INDEX] = (uint8_t)((rxd[RXDF3_RXD_INDEX] & RXD0_RXDF3_MASK) >> RXD0_RXDF3_BIT);
    rxdf[fac_index][RXDF4_INDEX] = (uint8_t)((rxd[RXDF4_RXD_INDEX] & RXD1_RXDF4_MASK) >> RXD1_RXDF4_BIT);
    rxdf[fac_index][RXDF5_INDEX] = (uint8_t)((rxd[RXDF5_RXD_INDEX] & RXD1_RXDF5_MASK) >> RXD1_RXDF5_BIT);
    rxdf[fac_index][RXDF6_INDEX] = (uint8_t)((rxd[RXDF6_RXD_INDEX] & RXD1_RXDF6_MASK) >> RXD1_RXDF6_BIT);
    rxdf[fac_index][RXDF7_INDEX] = (uint8_t)((rxd[RXDF7_RXD_INDEX] & RXD1_RXDF7_MASK) >> RXD1_RXDF7_BIT);

    if (fac_state[fac_index] == FAC_STATE_REQ || fac_state[fac_index] == FAC_STATE_ELC)
    {
        if (NULL != pfac_req_result_cb[fac_index])
        {
            pfac_req_result_cb[fac_index](&fac_rx_result[fac_index], &rxdf[fac_index][0]);
        }
    }
    else
    {
        if (NULL != pfac_e2prom_result_cb[fac_index])
        {
            pfac_e2prom_result_cb[fac_index](&fac_rx_result[fac_index], rxdf[fac_index][RXDF0_INDEX],
                                             rxdf[fac_index][RXDF1_INDEX]);
        }
    }

    /* RXRESULT register dummy read */
    rxresult = FAC(pfac_base_adr_tbl[fac_index]).RXRESULT.LONG;

    if (fac_state[fac_index] != FAC_STATE_ELC)
    {
        fac_state[fac_index] = FAC_STATE_IDLE;
    }
}

void enc_fac_ch0_int_isr(void)
{
    fac_set_result(FAC0_INDEX);
    __DMB();
}

void enc_fac_ch1_int_isr(void)
{
    fac_set_result(FAC1_INDEX);
    __DMB();
}

