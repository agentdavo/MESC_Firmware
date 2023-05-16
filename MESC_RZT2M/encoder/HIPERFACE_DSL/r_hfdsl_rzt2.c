#include "r_hfdsl_rzt2_if.h"

#include "r_hfdsl_rzt2_config.h"

#include <bsp_api.h>

#include <stdio.h>

#define HFDSL_DRV_VER                   (0x00010000u)

/* HFDSL base address */
#define BASE_ADDR_CH0                   (0xA011c100u)
#define BASE_ADDR_CH1                   (0xA031c100u)

#define HFDSL_FIFO_STS_DR(addr)         (*(volatile HFDSL_FIFO_STS *)(addr))
#define BASE_FIFO_STS_ADDR_CH0          (0xA0FD8100)
#define BASE_FIFO_STS_ADDR_CH1          (0xA0FD8104)

#define HFDSL_FIFO_DATA_DR(addr)        (*(volatile HFDSL_FIFO_DATA *)(addr))
#define BASE_FIFO_DATA_ADDR_CH0         (0xA0FD3000)
#define BASE_FIFO_DATA_ADDR_CH1         (0xA0FD3200)

/* HFDSL Channel */
#define CH0                             (0u)
#define CH1                             (1u)
#define CH_NUM                          (2u)

#define HFDSL_CMD_NUM                   (11)

/* Protocol initialization macros */
#define FIFO_DEP_MAX                    (16u)
#define FIFO_CLEAR                      (FIFO_DEP_MAX + 1)
#define EDGES_BIT                       (10u)
#define EDGES_10BIT_MASK                (0x3FFu)
#define EDGES_NG_7FFH                   (0x7FFu)
#define EDGES_NG_00H                    (0x00u)
#define EDGES_SHIFT_10                  (10u)
#define EDGES_PAT_10B                   (2u)
#define EDGES_PAT_01B                   (1u)
#define EDGES_3H_MASK                   (0x3u)
#define PAT_COUNT_OK                    (2u)
#define SUB1_FFH_MASK                   (0xFFu)
#define SUB1_10                         (10u)
#define SUB0_10                         (10u)
#define END_P_10                        (10u)
#define EDGES_SHIFT_20                  (10u)
#define EDGES_BIT_1                     (1u)
#define EDGES_SHIFT_1                   (1u)
#define SYS_INIT_0                      (0u)
#define SYS_INIT_1                      (1u)
#define SYS_INIT_2                      (2u)
#define SYS_INIT_3                      (3u)

/* Message Macro */
#define MSG_SIZE_MASK                   (0x000Fu)
#define SEND_SIZE_SHIFT                 (4u)

/* Interrupt macro */
#define INIT_END_BIT                    (0x04)
#define MIN_BIT                         (0x10u)
#define POS_RDY_BIT                     (0x20u)
#define MRCV_BIT                        (0x40u)
#define POS_UPD_BIT                     (0x80u)
#define EVENT_ERR_PRST_BIT              (0x00000001u)
#define EVENT_ERR_MRCV_ERR_BIT          (0x00000080u)
#define EVENT_ERR_RAW_FULL_BIT          (0x00000010u)

/* HFDSL Status */
typedef enum state_tbl_s
{
    HFDSL_STATE_CLOSE,
    HFDSL_STATE_IDLE,
    HFDSL_STATE_INIT1,
    HFDSL_STATE_INIT2,
    HFDSL_STATE_INIT3,
    HFDSL_STATE_INIT4,
    HFDSL_STATE_INIT5,
    HFDSL_STATE_TRANS,
} state_tbl_t;

typedef int32_t (*cmd_func_t)(const uint8_t hfdsl_ch, void *const pbuf);

typedef struct control_cmd_s
{
    r_hfdsl_cmd_t       control_cmd;
    cmd_func_t      control_cmd_func;
} control_cmd_t;

void hfdsl_int_nml_isr_ch0(void);
void hfdsl_int_err_isr_ch0(void);
void hfdsl_int_nml_isr_ch1(void);
void hfdsl_int_err_isr_ch1(void);

void hfdsl_int_nml_isr(uint8_t ch);
void hfdsl_int_err_isr(uint8_t ch);

static int32_t hfdsl_control(const uint8_t hfdsl_ch, const r_hfdsl_cmd_t cmd, void *const pbuf);
static int32_t hfdsl_cmd_init1(const uint8_t hfdsl_ch, void *const pbuf);
static int32_t hfdsl_cmd_init2(const uint8_t hfdsl_ch, void *const pbuf);
static int32_t hfdsl_cmd_init3(const uint8_t hfdsl_ch, void *const pbuf);
static int32_t hfdsl_cmd_init4(const uint8_t hfdsl_ch, void *const pbuf);
static int32_t hfdsl_cmd_init5(const uint8_t hfdsl_ch, void *const pbuf);
static int32_t hfdsl_cmd_init6(const uint8_t hfdsl_ch, void *const pbuf);
static int32_t hfdsl_cmd_pos(const uint8_t hfdsl_ch, void *const pbuf);
static int32_t hfdsl_cmd_vpos(const uint8_t hfdsl_ch, void *const pbuf);
static int32_t hfdsl_cmd_vel(const uint8_t hfdsl_ch, void *const pbuf);
static int32_t hfdsl_cmd_msg(const uint8_t hfdsl_ch, void *const pbuf);
static int32_t hfdsl_cmd_rst(const uint8_t hfdsl_ch, void *const pbuf);
static int32_t hfdsl_init_rssi(const uint8_t hfdsl_ch, uint16_t edges);
static void hfdsl_init_smp_msk(const uint8_t hfdsl_ch, uint16_t edges);
static void hfdsl_int_nml_com(const uint8_t hfdsl_ch);
static void hfdsl_int_err_com(const uint8_t hfdsl_ch);
static void hfdsl_fifo_data(const uint8_t hfdsl_ch); // for message/raw data

/*----------------------------------------------------------------------------*/
/* HFDSL base address */
void* pbaddr_tbl[CH_NUM] =
{
    (void *)BASE_ADDR_CH0,
    (void *)BASE_ADDR_CH1,
};

static void* pbaddr_fifo_data_tbl[CH_NUM] =
{
    (void *)BASE_FIFO_DATA_ADDR_CH0,
    (void *)BASE_FIFO_DATA_ADDR_CH1,
};

static void* pbaddr_fifo_sts_tbl[CH_NUM] =
{
    (void *)BASE_FIFO_STS_ADDR_CH0,
    (void *)BASE_FIFO_STS_ADDR_CH1,
};

static control_cmd_t control_cmd_tbl[HFDSL_CMD_NUM] =
{
    {R_HFDSL_CMD_POS,   &hfdsl_cmd_pos},
    {R_HFDSL_CMD_VEL,   &hfdsl_cmd_vel},
    {R_HFDSL_CMD_INIT1, &hfdsl_cmd_init1},
    {R_HFDSL_CMD_INIT2, &hfdsl_cmd_init2},
    {R_HFDSL_CMD_INIT3, &hfdsl_cmd_init3},
    {R_HFDSL_CMD_INIT4, &hfdsl_cmd_init4},
    {R_HFDSL_CMD_INIT5, &hfdsl_cmd_init5},
    {R_HFDSL_CMD_INIT6, &hfdsl_cmd_init6},
    {R_HFDSL_CMD_VPOS,  &hfdsl_cmd_vpos},
    {R_HFDSL_CMD_MSG,   &hfdsl_cmd_msg},
    {R_HFDSL_CMD_RST,   &hfdsl_cmd_rst},
};

/*----------------------------------------------------------------------------*/
/* Status of HFDSL driver */
static state_tbl_t hfdsl_state[CH_NUM] =
{
    HFDSL_STATE_CLOSE
};

/* Store callback function */
static r_hfdsl_int_nml_cb_t hfdsl_int_nml_cb[CH_NUM];
static r_hfdsl_int_err_cb_t hfdsl_int_err_cb[CH_NUM];
static r_hfdsl_int_fifo_raw_cb_t hfdsl_int_raw_cb[CH_NUM];
static r_hfdsl_msg_cb_t hfdsl_int_msg_cb[CH_NUM];
static r_hfdsl_int_init_cb_t hfdsl_int_init_cb[CH_NUM];

/* FIFO_RAW setting value */
static uint16_t hfdsl_raw_data[CH_NUM][FIFO_DEP_MAX];

/* FIFO_MSGR, FIFO_MSGR setting value */
static uint8_t hfdsl_msg_rec_num[CH_NUM];
static uint16_t hfdsl_msg_data[CH_NUM][FIFO_DEP_MAX];


/* FIFO raw count */
static uint8_t hfdsl_raw_bit_init[CH_NUM];
static uint8_t hfdsl_raw_bit[CH_NUM];
static uint8_t hfdsl_raw_msg_cnt[CH_NUM];

int32_t R_HFDSL_Open(const int32_t id, r_hfdsl_info_t* pinfo)
{
    int32_t result = R_HFDSL_SUCCESS;
    uint8_t hfdsl_ch;
    uint8_t loop;

    /* Check id */
    if (R_HFDSL0_ID == id){
      hfdsl_ch = CH0;
    }else if (R_HFDSL1_ID == id){
      hfdsl_ch = CH1;
    }else{
      goto err_invalid;
    }

    /* Check Status */
    if (HFDSL_STATE_CLOSE != hfdsl_state[hfdsl_ch])
    {
        goto err_access;
    }

    /* Check pinfo */
    if (NULL == pinfo)
    {
        goto err_invalid;
    }

    /* Interrupt settings */
    if(hfdsl_ch == 0){
      R_BSP_IrqDisable(VECTOR_NUMBER_ENCIF_INT0);
      R_BSP_IrqCfg(    VECTOR_NUMBER_ENCIF_INT0, 11, NULL);
      R_BSP_IrqEnable( VECTOR_NUMBER_ENCIF_INT0);    
      R_BSP_IrqDisable(VECTOR_NUMBER_ENCIF_INT1);
      R_BSP_IrqCfg(    VECTOR_NUMBER_ENCIF_INT1, 11, NULL);
      R_BSP_IrqEnable( VECTOR_NUMBER_ENCIF_INT1);
    }else{
      R_BSP_IrqDisable(VECTOR_NUMBER_ENCIF_INT4);
      R_BSP_IrqCfg(    VECTOR_NUMBER_ENCIF_INT4, 11, NULL);
      R_BSP_IrqEnable( VECTOR_NUMBER_ENCIF_INT4);    
      R_BSP_IrqDisable(VECTOR_NUMBER_ENCIF_INT5);
      R_BSP_IrqCfg(    VECTOR_NUMBER_ENCIF_INT5, 11, NULL);
      R_BSP_IrqEnable( VECTOR_NUMBER_ENCIF_INT5);
    }
    
    /* Setting the callback function */
    hfdsl_int_nml_cb[hfdsl_ch] = pinfo->pcb_nml;
    hfdsl_int_err_cb[hfdsl_ch] = pinfo->pcb_err;
    hfdsl_int_raw_cb[hfdsl_ch] = pinfo->pcb_raw;
    hfdsl_int_init_cb[hfdsl_ch] = pinfo->pcb_init;

    /* Status transition */
    hfdsl_state[hfdsl_ch] = HFDSL_STATE_IDLE;

    /* Var init */
    hfdsl_raw_bit[hfdsl_ch]=0x01;
    for(loop=0;loop<8;loop++)
    {
      if(hfdsl_raw_bit[hfdsl_ch] & R_HFDSL_RAW_EN)
      {
        break;
      }else{
        hfdsl_raw_bit[hfdsl_ch] = (uint8_t)(hfdsl_raw_bit[hfdsl_ch]<<1u);
      }
    }
    hfdsl_raw_bit_init[hfdsl_ch] = hfdsl_raw_bit[hfdsl_ch];
    hfdsl_raw_msg_cnt[hfdsl_ch]=0u;
    
    goto end;

err_invalid:
    result = R_HFDSL_ERR_INVALID_ARG;
    goto end;

err_access:
    result = R_HFDSL_ERR_ACCESS;
    goto end;

end:
    return result;
}

int32_t R_HFDSL_Close(const int32_t id)
{
    int32_t result = R_HFDSL_SUCCESS;
    uint8_t hfdsl_ch;

    /* Check id */
    if (R_HFDSL0_ID == id){
      hfdsl_ch = CH0;
    }else if (R_HFDSL1_ID == id){
      hfdsl_ch = CH1;
    }else{
      goto err_invalid;
    }

    /* Disable interrupt */
    if(hfdsl_ch == 0){
       R_BSP_IrqDisable(VECTOR_NUMBER_ENCIF_INT0);
       R_BSP_IrqDisable(VECTOR_NUMBER_ENCIF_INT1);
    }else{
       R_BSP_IrqDisable(VECTOR_NUMBER_ENCIF_INT4);
       R_BSP_IrqDisable(VECTOR_NUMBER_ENCIF_INT5);
    }

    /* Status transition */
    hfdsl_state[hfdsl_ch] = HFDSL_STATE_CLOSE;
    goto end;

err_invalid:
    result = R_HFDSL_ERR_INVALID_ARG;

end:
    return result;
}

uint32_t R_HFDSL_GetVersion(void)
{
    return HFDSL_DRV_VER;
}

int32_t R_HFDSL_Control(const int32_t id, const r_hfdsl_cmd_t cmd, void *const pbuf)
{
    int32_t result = R_HFDSL_SUCCESS;
    uint8_t hfdsl_ch;

    /* Check id */
    if (R_HFDSL0_ID == id){
      hfdsl_ch = CH0;
    }else if (R_HFDSL1_ID == id){
      hfdsl_ch = CH1;
    }else{
      goto err_invalid;
    }

    /* Control function of HFDSL driver */
    result = hfdsl_control(hfdsl_ch, cmd, pbuf);

    goto end;

err_invalid:
    result = R_HFDSL_ERR_INVALID_ARG;
    goto end;

end:
    return result;
}

int32_t R_HFDSL_CheckInitSeq(const int32_t id)
{
    int32_t result;
    uint8_t hfdsl_ch;

    /* Check id */
    if (R_HFDSL0_ID == id){
      hfdsl_ch = CH0;
    }else if (R_HFDSL1_ID == id){
      hfdsl_ch = CH1;
    }else{
      goto err_invalid;
    }

    result = (int32_t)(HFDSL_DR(pbaddr_tbl[hfdsl_ch]).EVENT.BIT.INIT);
    goto end;

err_invalid:
    result = R_HFDSL_ERR_INVALID_ARG;
    goto end;

end:
    return result;
}

static int32_t hfdsl_control(const uint8_t hfdsl_ch, const r_hfdsl_cmd_t cmd, void *const pbuf)
{
    int32_t result = R_HFDSL_SUCCESS;
    uint32_t index;

    /* Check cmd */
    for (index = 0; index < HFDSL_CMD_NUM; index++)
    {
        if (cmd == control_cmd_tbl[index].control_cmd)
        {
            break;
        }
    }

    if (HFDSL_CMD_NUM <= index)
    {
        goto err_invalid;
    }

    /* Call control command function */
    result = control_cmd_tbl[index].control_cmd_func(hfdsl_ch, pbuf);

    goto end;

err_invalid:
    result = R_HFDSL_ERR_INVALID_ARG;

end:
    return result;
}

static int32_t hfdsl_cmd_init1(const uint8_t hfdsl_ch, void *const pbuf)
{
    int32_t result = R_HFDSL_SUCCESS;

    /* Check Status */
    if (HFDSL_STATE_IDLE != hfdsl_state[hfdsl_ch])
    {
        goto err_access;
    }

    /* Check pbuf */
    if (NULL != pbuf)
    {
        goto err_invalid;
    }

    /* Run communication reset. */
    HFDSL_DR(pbaddr_tbl[hfdsl_ch]).FIFORST.BIT.RST = 1u;

    /* Release communication reset */
    HFDSL_DR(pbaddr_tbl[hfdsl_ch]).FIFORST.BIT.RST = 0u;
    
    /* Start output enable of HFDSL */
    HFDSL_DR(pbaddr_tbl[hfdsl_ch]).SYS_INIT.BYTE = SYS_INIT_0;
    HFDSL_DR(pbaddr_tbl[hfdsl_ch]).SYS_CTRL.BIT.PRST = 0;

    /* Status transition */
    hfdsl_state[hfdsl_ch] = HFDSL_STATE_INIT1;

    R_BSP_SoftwareDelay(2, BSP_DELAY_UNITS_MICROSECONDS);

    goto end;

err_invalid:
    result = R_HFDSL_ERR_INVALID_ARG;
    goto end;

err_access:
    result = R_HFDSL_ERR_ACCESS;

end:
    return result;
}

static int32_t hfdsl_cmd_init2(const uint8_t hfdsl_ch, void *const pbuf)
{
    int32_t result = R_HFDSL_SUCCESS;

    /* Check Status */
    if (HFDSL_STATE_INIT1 != hfdsl_state[hfdsl_ch])
    {
        goto err_access;
    }

    /* Check pbuf */
    if (NULL != pbuf)
    {
        goto err_invalid;
    }

    /* Start bit sampling  */
    HFDSL_DR(pbaddr_tbl[hfdsl_ch]).SYS_INIT.BYTE = SYS_INIT_1;

    /* Status transition */
    hfdsl_state[hfdsl_ch] = HFDSL_STATE_INIT2;

    R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MICROSECONDS);

    goto end;

err_invalid:
    result = R_HFDSL_ERR_INVALID_ARG;
    goto end;

err_access:
    result = R_HFDSL_ERR_ACCESS;

end:
    return result;
}

static int32_t hfdsl_cmd_init3(const uint8_t hfdsl_ch, void *const pbuf)
{       
    int32_t result = R_HFDSL_SUCCESS;
    
    /* Check Status */
    if (HFDSL_STATE_INIT2 != hfdsl_state[hfdsl_ch])
    {
        goto err_access;
    }
    
    /* Check pbuf */
    if (NULL != pbuf)
    {
        goto err_invalid;
    }
    
    /* Setting of HFDSL */
    HFDSL_DR(pbaddr_tbl[hfdsl_ch]).SYS_CTRL.WORD    = R_HFDSL_ES_PRDY;
    HFDSL_DR(pbaddr_tbl[hfdsl_ch]).MAXACC.WORD      = R_HFDSL_MAXACC;
    HFDSL_DR(pbaddr_tbl[hfdsl_ch]).ACC_ERR.BYTE     = R_HFDSL_ACC_ERR;
    HFDSL_DR(pbaddr_tbl[hfdsl_ch]).MAXDEV.WORD      = R_HFDSL_MAXDEV;
    HFDSL_DR(pbaddr_tbl[hfdsl_ch]).MASK.BYTE        = R_HFDSL_MASK;
    HFDSL_DR(pbaddr_tbl[hfdsl_ch]).MASK_ERR.LONG    = R_HFDSL_MASK_ERR;
    HFDSL_DR(pbaddr_tbl[hfdsl_ch]).TH_RAW.LONG      = R_HFDSL_TH_RAW;
    HFDSL_DR(pbaddr_tbl[hfdsl_ch]).RAW_EN.BYTE      = R_HFDSL_RAW_EN;
    HFDSL_DR(pbaddr_tbl[hfdsl_ch]).STUFF.BYTE       = R_HFDSL_STUFF;
    HFDSL_DR(pbaddr_tbl[hfdsl_ch]).EXLEN.BYTE       = R_HFDSL_EXLEN;
    HFDSL_DR(pbaddr_tbl[hfdsl_ch]).EXTRA.WORD       = R_HFDSL_EXTRA;
    
    /* Status transition */
    hfdsl_state[hfdsl_ch] = HFDSL_STATE_INIT3;

    R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MICROSECONDS);
    
    goto end;
    
err_access:
    result = R_HFDSL_ERR_ACCESS;
    goto end;
    
err_invalid:
    result = R_HFDSL_ERR_INVALID_ARG;
    goto end;
    
end:
    return result;
}   

static int32_t hfdsl_cmd_init4(const uint8_t hfdsl_ch, void *const pbuf)
{       
    int32_t result = R_HFDSL_SUCCESS;
    uint16_t edges;
    
    /* Check Status */
    if (HFDSL_STATE_INIT3 != hfdsl_state[hfdsl_ch])
    {
        goto err_access;
    }
    
    /* Check pbuf */
    if (NULL != pbuf)
    {
        goto err_invalid;
    }
    
    /* Read EDGES register value */
    edges = HFDSL_DR(pbaddr_tbl[hfdsl_ch]).EDGES.BIT.EDGES & EDGES_10BIT_MASK;
    
    /* Initialization of RSSI function */
    result = hfdsl_init_rssi(hfdsl_ch, edges);
    if (R_HFDSL_ERR_INIT == result)
    {
        goto end;
    }
    
    /* Setting SMP_MSK register */
    hfdsl_init_smp_msk(hfdsl_ch, edges);
    
    /* Status transition */
    hfdsl_state[hfdsl_ch] = HFDSL_STATE_INIT4;

    R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MICROSECONDS);
    
    goto end;
    
err_invalid:
    result = R_HFDSL_ERR_INVALID_ARG;
    goto end;
    
err_access:
    result = R_HFDSL_ERR_ACCESS;
    
end:
    return result;
}       

static int32_t hfdsl_cmd_init5(const uint8_t hfdsl_ch, void *const pbuf)
{
    int32_t result = R_HFDSL_SUCCESS;

    /* Check Status */
    if (HFDSL_STATE_INIT4 != hfdsl_state[hfdsl_ch])
    {
        goto err_access;
    }

    /* Check pbuf */
    if (NULL != pbuf)
    {
        goto err_invalid;
    }

    /* Check Transmission delay */
    if (HFDSL_DR(pbaddr_tbl[hfdsl_ch]).DELAY.BIT.DELAY > R_HFDSL_DELAY_UPP_LIMIT)
    {
        goto err_init;
    }

    /* Get Encoder ID */
    HFDSL_DR(pbaddr_tbl[hfdsl_ch]).SYS_INIT.BYTE = SYS_INIT_2;

    /* Status transition */
    hfdsl_state[hfdsl_ch] = HFDSL_STATE_INIT5;

    R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MICROSECONDS);
    
    goto end;
    
err_invalid:
    result = R_HFDSL_ERR_INVALID_ARG;
    goto end;
    
err_access:
    result = R_HFDSL_ERR_ACCESS;
    goto end;
    
err_init:
    result = R_HFDSL_ERR_INIT;
    
end:
    return result;
}       

static int32_t hfdsl_cmd_init6(const uint8_t hfdsl_ch, void *const pbuf)
{
    int32_t result = R_HFDSL_SUCCESS;
    uint32_t* penc_id = pbuf;

    /* Check Status */
    if (HFDSL_STATE_INIT5 != hfdsl_state[hfdsl_ch])
    {
        goto err_access;
    }

    /* Check enc_id */
    if (NULL == penc_id)
    {
        goto err_invalid;
    }

    /* Check Encoder ID */
    if (HFDSL_DR(pbaddr_tbl[hfdsl_ch]).ENC_ID.BIT.ENC_ID != (*penc_id))
    {
        goto err_init;
    }

    /* Normal communication hfdsl_state start */
    HFDSL_DR(pbaddr_tbl[hfdsl_ch]).SYS_INIT.BYTE = SYS_INIT_3;

    /* Status transition */
    hfdsl_state[hfdsl_ch] = HFDSL_STATE_TRANS;

    R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MICROSECONDS);

    goto end;

err_invalid:
    result = R_HFDSL_ERR_INVALID_ARG;
    goto end;

err_access:
    result = R_HFDSL_ERR_ACCESS;
    goto end;

err_init:
    result = R_HFDSL_ERR_INIT;

end:
    return result;
}

static int32_t hfdsl_cmd_pos(const uint8_t hfdsl_ch, void *const pbuf)
{
    int32_t result = R_HFDSL_SUCCESS;
    r_hfdsl_pos_t* ppos = pbuf;

    /* Check pos */
    if (NULL == ppos)
    {
        goto err_invalid;
    }

    ppos->pos = HFDSL_DR(pbaddr_tbl[hfdsl_ch]).POS.LONG;
    
    if (false !=ppos->all)
    {
        ppos->posh = HFDSL_DR(pbaddr_tbl[hfdsl_ch]).POS_H.BYTE;
    }
    
    goto end;
    
err_invalid:
    result = R_HFDSL_ERR_INVALID_ARG;

end:
    return result;
}

static int32_t hfdsl_cmd_vpos(const uint8_t hfdsl_ch, void *const pbuf)
{
    int32_t result = R_HFDSL_SUCCESS;
    r_hfdsl_vpos_t* pvpos = pbuf;

    /* Check vpos */
    if (NULL == pvpos)
    {
        goto err_invalid;
    }
    
    pvpos->vpos  = HFDSL_DR(pbaddr_tbl[hfdsl_ch]).VPOS.LONG;
    pvpos->vposh = HFDSL_DR(pbaddr_tbl[hfdsl_ch]).VPOS_H.BYTE;
    pvpos->crc   = HFDSL_DR(pbaddr_tbl[hfdsl_ch]).VPOSCRC.WORD;

    goto end;

err_invalid:
    result = R_HFDSL_ERR_INVALID_ARG;

end:
    return result;
}

static int32_t hfdsl_cmd_vel(const uint8_t hfdsl_ch, void *const pbuf)
{
    int32_t result = R_HFDSL_SUCCESS;
    uint32_t* pvel = pbuf;

    /* Check of vel */
    if (NULL == pvel)
    {
        goto err_invalid;
    }

    *pvel = HFDSL_DR(pbaddr_tbl[hfdsl_ch]).VEL.LONG;

    goto end;

err_invalid:
    result = R_HFDSL_ERR_INVALID_ARG;

end:
    return result;
}

static int32_t hfdsl_cmd_msg(const uint8_t hfdsl_ch, void *const pbuf)
{
    int32_t result = R_HFDSL_SUCCESS;
    r_hfdsl_send_msg_t* pmsg = pbuf;
    uint8_t send_size;
    uint8_t loop;

    /* Check Status */
    if (HFDSL_STATE_TRANS != hfdsl_state[hfdsl_ch])
    {
        goto err_access;
    }

    /* Check pmsg */
    if (NULL == pmsg)
    {
        goto err_invalid;
    }

    /* Check pmsg->pdata */
    if (NULL == pmsg->pdata)
    {
        goto err_invalid;
    }

    /* Save receive size and callback function */
    hfdsl_msg_rec_num[hfdsl_ch] = pmsg->pdata[0] & MSG_SIZE_MASK;
    send_size = (pmsg->pdata[0] >> SEND_SIZE_SHIFT) & MSG_SIZE_MASK;
    hfdsl_int_msg_cb[hfdsl_ch] = pmsg->pcb_msg;

    /* Write send data to FIFO_MSGW */
    for (loop = 0; loop < (send_size + 1); loop++)
    {
        HFDSL_FIFO_DATA_DR(pbaddr_fifo_data_tbl[hfdsl_ch]).WORD = pmsg->pdata[loop];
    }

    goto end;

err_access:
    result = R_HFDSL_ERR_ACCESS;
    goto end;

err_invalid:
    result = R_HFDSL_ERR_INVALID_ARG;

end:
    return result;
}

static int32_t hfdsl_cmd_rst(const uint8_t hfdsl_ch, void *const pbuf)
{
    int32_t result = R_HFDSL_SUCCESS;

    /* Check pbuf */
    if (NULL != pbuf)
    {
        goto err_invalid;
    }

    /* Protocol reset */
    HFDSL_DR(pbaddr_tbl[hfdsl_ch]).SYS_CTRL.BIT.PRST = 1;

    goto end;

err_invalid:
    result = R_HFDSL_ERR_INVALID_ARG;

end:
    return result;
}

static int32_t hfdsl_init_rssi(const uint8_t hfdsl_ch, uint16_t edges)
{
    int32_t result = R_HFDSL_SUCCESS;
    uint16_t edges11b;
    uint8_t shift;
    uint8_t pat_count;
    uint16_t sta_p;
    uint16_t end_p;
    uint8_t sub0;
    uint8_t sub1;
    uint8_t sub2;
    uint16_t tmp;

    /* Initialization of RSSI function */
    edges11b = edges | (((uint16_t)(edges << EDGES_BIT)) & (~EDGES_10BIT_MASK));

    if ((EDGES_NG_7FFH == edges11b) || (EDGES_NG_00H == edges11b))
    {
        goto err_init;
    }

    pat_count = 0;

    for (shift = 0; shift < EDGES_SHIFT_10;  shift++)
    {
        if (EDGES_PAT_10B == (edges11b & EDGES_3H_MASK))
        {
            sta_p = shift;
            pat_count++;
        }
        else if (EDGES_PAT_01B == (edges11b & EDGES_3H_MASK))
        {
            end_p = (uint16_t)(shift + 1u);
            pat_count++;
        }
        else
        {
            ;
        }
        edges11b = edges11b >> 1;
    }

    if (PAT_COUNT_OK != pat_count)
    {
        goto err_init;
    }

    if (END_P_10 == end_p)
    {
        end_p = 0;
    }

    if (sta_p == end_p)
    {
        sub0 = (uint8_t)(end_p + 1u);
        sub1 = 0;
        sub2 = 0;
    }
    else
    {
        sub0 = (uint8_t)(end_p + 1u);
        tmp = (1u <<sta_p) & SUB1_FFH_MASK;
        sub1 = ((uint8_t)(tmp >> sub0)) | ((uint8_t)(tmp << (SUB1_10 - sub0)));
        sub2 = (uint8_t)(sub1 - 1u);
    }

    if (SUB0_10 == sub0)
    {
        sub0 = 0;
    }

    HFDSL_DR(pbaddr_tbl[hfdsl_ch]).RSSI_SUB0.BYTE = sub0;
    HFDSL_DR(pbaddr_tbl[hfdsl_ch]).RSSI_SUB1.BIT.RSSI_SUB1 = sub1;
    HFDSL_DR(pbaddr_tbl[hfdsl_ch]).RSSI_SUB2.BIT.RSSI_SUB2 = sub2;

    goto end;

err_init:
    result = R_HFDSL_ERR_INIT;

end:
    return result;
}

static void hfdsl_init_smp_msk(const uint8_t hfdsl_ch, uint16_t edges)
{
    uint32_t edge_data;
    uint8_t tmp_z;
    uint8_t tmp_p;
    uint8_t sav_z;
    uint8_t sav_p;
    uint16_t shift;
    uint8_t shift_right;
    uint16_t smp_msk;

    edge_data = edges;
    edge_data |= (edge_data << EDGES_SHIFT_10);

    tmp_z = 0;
    tmp_p = 0;
    sav_z = 0;
    sav_p = 0;

    for (shift_right = 0; shift_right < EDGES_SHIFT_20; shift_right++)
    {
        if (EDGES_BIT_1 == (edge_data & EDGES_BIT_1))
        {
            tmp_p = (uint8_t)(shift_right + 1u);
            tmp_z = 0;
        }
        else
        {
            tmp_z++;
        }

        if (tmp_z > sav_z)
        {
            sav_p = tmp_p;
            sav_z = tmp_z;
        }

        edge_data >>= EDGES_SHIFT_1;
    }

    shift = (uint16_t)((sav_z >> EDGES_SHIFT_1) + sav_p);
    smp_msk = (uint16_t)(EDGES_SHIFT_1 << (shift % EDGES_BIT));
    HFDSL_DR(pbaddr_tbl[hfdsl_ch]).SMP_MSK.WORD = smp_msk;

    return;
}

static void hfdsl_int_nml_com(const uint8_t hfdsl_ch)
{
    uint8_t event;
    volatile uint8_t dummy8;

    /* Read EVENT register */
    event = HFDSL_DR(pbaddr_tbl[hfdsl_ch]).EVENT.BYTE;

    /* Creation of judgment value */
    event &= R_HFDSL_MASK;

    if (INIT_END_BIT == (event & INIT_END_BIT))
    {
        /* Calling R_HFDSL_IntInitCb functions */
        if (NULL != hfdsl_int_init_cb[hfdsl_ch])
        {
            hfdsl_int_init_cb[hfdsl_ch]();
        }
    }
    if ((MIN_BIT == (event & MIN_BIT)) ||
        (POS_RDY_BIT == (event & POS_RDY_BIT)) ||
        (POS_UPD_BIT == (event & POS_UPD_BIT)))
    {
        /* Calling R_HFDSL_IntNmlCb functions */
        if (NULL != hfdsl_int_nml_cb[hfdsl_ch])
        {
            hfdsl_int_nml_cb[hfdsl_ch](event);
        }
    }

    hfdsl_fifo_data(hfdsl_ch); // get message/raw_data from FIFO
    
    /* Clear interrupt factor */
    HFDSL_DR(pbaddr_tbl[hfdsl_ch]).EVENTCLR.BYTE = event;

    /* Dummy read */
    dummy8 = HFDSL_DR(pbaddr_tbl[hfdsl_ch]).EVENTCLR.BYTE;
    (void)dummy8;
    
    return;
}

/*
| RAW_EN | raw_data_dummy|
|--------+---------------|
|   0x01 | 0: vertical   |
|   0x02 | 1: s_par      |
|   0x04 | 2: pipeline   |
|   0x08 | 3: acc        |
|   0x10 | 4: acc_crc    |
|   0x20 | 5: second     |
|   0x40 |    (message)  |
|   0x80 | 6: pos_h      |
|   0x80 | 7: pos[31:16] |
|   0x80 | 8: pos[15:00] |
|   0x80 | 9: event_err  |
|      - |10: unknown    |
*/

static void hfdsl_fifo_data(const uint8_t hfdsl_ch)
{
  uint8_t  loop,i;
  uint16_t data;


  for(loop=0;loop<8;loop++)
  {
    if(HFDSL_FIFO_STS_DR(pbaddr_fifo_sts_tbl[hfdsl_ch]).WORD&0x001f0000)
    {
        data = HFDSL_FIFO_DATA_DR(pbaddr_fifo_data_tbl[hfdsl_ch]).WORD;

      if(hfdsl_raw_bit[hfdsl_ch] & 0x40){ // message
        if(data != 0xFFFF){
          // get message data
          if(hfdsl_raw_msg_cnt[hfdsl_ch] < hfdsl_msg_rec_num[hfdsl_ch]){
            hfdsl_msg_data[hfdsl_ch][hfdsl_raw_msg_cnt[hfdsl_ch]++] = data;

            if(hfdsl_raw_msg_cnt[hfdsl_ch] == hfdsl_msg_rec_num[hfdsl_ch]){
              /* Calling R_HFDSL_IntMsgCB functions */
              if (NULL != hfdsl_int_msg_cb[hfdsl_ch]){
                hfdsl_int_msg_cb[hfdsl_ch](&hfdsl_msg_data[hfdsl_ch][0]);
              }
              hfdsl_raw_msg_cnt[hfdsl_ch]=0u;
            }
          }
        }
      }else{
        if(      hfdsl_raw_bit[hfdsl_ch] & 0x01){ hfdsl_raw_data[hfdsl_ch][0] = data; // vertical
        }else if(hfdsl_raw_bit[hfdsl_ch] & 0x02){ hfdsl_raw_data[hfdsl_ch][1] = data; // s_par
        }else if(hfdsl_raw_bit[hfdsl_ch] & 0x04){ hfdsl_raw_data[hfdsl_ch][2] = data; // pipeline
        }else if(hfdsl_raw_bit[hfdsl_ch] & 0x08){ hfdsl_raw_data[hfdsl_ch][3] = data; // acc
        }else if(hfdsl_raw_bit[hfdsl_ch] & 0x10){ hfdsl_raw_data[hfdsl_ch][4] = data; // acc_crc
        }else if(hfdsl_raw_bit[hfdsl_ch] & 0x20){ hfdsl_raw_data[hfdsl_ch][5] = data; // second
        }else if(hfdsl_raw_bit[hfdsl_ch] & 0x80){ // pos
          hfdsl_raw_data[hfdsl_ch][6] = data; // pos_h
          hfdsl_raw_data[hfdsl_ch][7] = HFDSL_FIFO_DATA_DR(pbaddr_fifo_data_tbl[hfdsl_ch]).WORD; // pos[31:16]
          hfdsl_raw_data[hfdsl_ch][8] = HFDSL_FIFO_DATA_DR(pbaddr_fifo_data_tbl[hfdsl_ch]).WORD; // pos[15:00]
          hfdsl_raw_data[hfdsl_ch][9] = HFDSL_FIFO_DATA_DR(pbaddr_fifo_data_tbl[hfdsl_ch]).WORD; // event_err
        }else{ hfdsl_raw_data[hfdsl_ch][10] = data; // unknown
        }
        /* Calling R_HFDSL_IntRawCb functions */
        if (NULL != hfdsl_int_raw_cb[hfdsl_ch]){
          hfdsl_int_raw_cb[hfdsl_ch](&hfdsl_raw_data[hfdsl_ch][0]);
        }
      }

    // set next bit
    for(i=0;i<8;i++)
    {
      hfdsl_raw_bit[hfdsl_ch] = (uint8_t)(hfdsl_raw_bit[hfdsl_ch] << 1u);
      if(hfdsl_raw_bit[hfdsl_ch] & R_HFDSL_RAW_EN){ break; }
      if(hfdsl_raw_bit[hfdsl_ch] == 0){
        hfdsl_raw_bit[hfdsl_ch] = hfdsl_raw_bit_init[hfdsl_ch];
        break;
      }
    }

    }
  }

  return;
}

static void hfdsl_int_err_com(const uint8_t hfdsl_ch)
{
    uint32_t event_err;
    uint8_t loop;
    volatile uint16_t dummy16;
    volatile uint32_t dummy32;

    /* Read EVENT_ERR register */
    event_err = HFDSL_DR(pbaddr_tbl[hfdsl_ch]).EVENT_ERR.LONG;

    /* Creation of judgment value */
    event_err &= R_HFDSL_MASK_ERR;

    /* Clear FIFO_MSGR register */
    if (EVENT_ERR_MRCV_ERR_BIT == (event_err & EVENT_ERR_MRCV_ERR_BIT))
    {
        for (loop = 0; loop < FIFO_CLEAR; loop++)
        {
            dummy16 = HFDSL_FIFO_DATA_DR(pbaddr_fifo_data_tbl[hfdsl_ch]).WORD;
        }
    }

    /* Clear FIFO_RAW register */
    if (EVENT_ERR_RAW_FULL_BIT == (event_err & EVENT_ERR_RAW_FULL_BIT))
    {
        HFDSL_DR(pbaddr_tbl[hfdsl_ch]).RAW_EN.BYTE = 0;

        for (loop = 0; loop < FIFO_CLEAR; loop++)
        {
            dummy16 = HFDSL_FIFO_DATA_DR(pbaddr_fifo_data_tbl[hfdsl_ch]).WORD;
        }
    }

    /* Calling R_HFDSL_IntErrCb functions */
    if (NULL != hfdsl_int_err_cb[hfdsl_ch])
    {
        hfdsl_int_err_cb[hfdsl_ch](event_err);
    }

    /* Clear interrupt factor */
    HFDSL_DR(pbaddr_tbl[hfdsl_ch]).EVENTCLR_ERR.LONG = event_err;


    /* Resetting RAW_EN register */
    if (EVENT_ERR_RAW_FULL_BIT == (event_err & EVENT_ERR_RAW_FULL_BIT))
    {
        HFDSL_DR(pbaddr_tbl[hfdsl_ch]).RAW_EN.BYTE = R_HFDSL_RAW_EN;
    }
    /* Dummy read */
    dummy32 = HFDSL_DR(pbaddr_tbl[hfdsl_ch]).EVENTCLR_ERR.LONG;

    /* Status transition */
    if (EVENT_ERR_PRST_BIT == (event_err & EVENT_ERR_PRST_BIT))
    {
        hfdsl_state[hfdsl_ch] = HFDSL_STATE_IDLE;
    }

    (void)dummy16;
    (void)dummy32;
    
    return;
}

void hfdsl_int_nml_isr(uint8_t ch)
{
    hfdsl_int_nml_com(ch);
    __DMB();
}

void hfdsl_int_nml_isr_ch0(void)
{
    hfdsl_int_nml_isr(CH0);
}

void hfdsl_int_nml_isr_ch1(void)
{
    hfdsl_int_nml_isr(CH1);
}

void hfdsl_int_err_isr(uint8_t ch)
{
    hfdsl_int_err_com(ch);
    __DMB();
}

void hfdsl_int_err_isr_ch0(void)
{
    hfdsl_int_err_isr(CH0);
}

void hfdsl_int_err_isr_ch1(void)
{
    hfdsl_int_err_isr(CH1);
}

