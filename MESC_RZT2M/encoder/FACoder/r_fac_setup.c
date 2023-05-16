#include "r_fac_rzt2_if.h"

#include "r_enc_int_rzt2.h"

#include <bsp_api.h>
#include <hal_data.h>

#include <ecl/r_ecl_rzt2_if.h>

#include <eEncoderStatusFlags.h>
#include <eType.h>
#include <stdio.h>
#include <stdlib.h>
#include <tMotor.h>

#define R_FAC0_ID  (R_ECL_CH_0)
#define R_FAC1_ID  (R_ECL_CH_1)

#define FA_CODER_FMT23      1
#define FA_CODER_FMT17      0
#define FA_CODER_FMT14      0

#define MY_PRINTF_END       "\n"      /* Line feed code */
//#define MY_PRINTF(...)      (printf(__VA_ARGS__))
#define MY_PRINTF(...)

#define ARG_MAX             (4)         /* maximum number of comannd arguments */
#define CMD_BUF_SIZE        (256)       /* command buffer size */
#define CMD_NUM             (10)        /* number of commands */
#define EXIT_CMD_NUM        (9)         /* exit command number */
#define CMD_DELIMITER       (" \t\r\n") /* command line delimiter */
#define TIMOTN_VALUE        (4000)      /* timeout */

/* EC-lib macros */
#define R_FAC_FREQ (R_ECL_FREQ_10000KHZ)  //r_ecl_rzt2_if.h

/* "single" command parameter */
#define CMD_SINGLE_ARG_NUM  (1)
#define CMD_SINGLE_ID       (0)
#if FA_CODER_FMT14                      /* The 14-bit FA-CODER format includes only two bytes absolute single-turn position */
#define CMD_SINGLE_DFNUM    (2)
#else
#define CMD_SINGLE_DFNUM    (3)         /* The 17-bit and 23-bit FA-CODER includes three bytes absolute single-turn position */
#endif
#define RXDF_SINGLE0_INDEX  (0)
#define RXDF_SINGLE0_MASK   (0xFFu)
#define RXDF_SINGLE0_BIT    (0)
#define RXDF_SINGLE1_INDEX  (1)
#define RXDF_SINGLE1_MASK   (0xFFu)
#define RXDF_SINGLE1_BIT    (8)
#define RXDF_SINGLE2_INDEX  (2)
#define RXDF_SINGLE2_MASK   (0xFFu)
#define RXDF_SINGLE2_BIT    (16)

/* "multi" command parameter */
#define CMD_MULTI_ARG_NUM   (1)
#define CMD_MULTI_ID        (1)
#define CMD_MULTI_DFNUM     (3)
#define RXDF_MULTI0_INDEX   (0)
#define RXDF_MULTI0_MASK    (0xFFu)
#define RXDF_MULTI0_BIT     (0)
#define RXDF_MULTI1_INDEX   (1)
#define RXDF_MULTI1_MASK    (0xFFu)
#define RXDF_MULTI1_BIT     (8)

/* "encid" command parameter */
#define CMD_ENCID_ARG_NUM   (1)
#define CMD_ENCID_ID        (2)
#define CMD_ENCID_DFNUM     (1)
#define RXDF_ENCID_INDEX    (0)
#define RXDF_ENCID_MASK     (0xFFu)
#define RXDF_ENCID_BIT      (0)

/* "req" command parameter */
#define CMD_REQ_ARG_NUM     (3)
#define CMD_REQ_ARG_ID      (1)
#define CMD_REQ_ARG_DFNUM   (2)
#define CMD_REQ_ARG_BASE    (10)
#define REQ_ID_MAX          (15)
#define REQ_DFNUM_MIN       (1)
#define REQ_DFNUM_MAX       (8)

/* "E2PROM" command parameter */
#define CMD_E2PROM_W_ARG_NUM    (3)
#define CMD_E2PROM_R_ARG_NUM    (2)
#define CMD_E2PROM_ARG_ADR      (1)
#define CMD_E2PROM_ARG_DATA     (2)
#define CMD_E2PROM_ARG_BASE     (10)
#define E2PROM_ID_MAX           (15)
#define E2PROM_ADR_MIN          (0)
#define E2PROM_ADR_MAX          (79)
#define E2PROM_WRITE_DATA_MAX   (255)

/* "reset" command parameter */
#define CMD_RESET_ARG_NUM   (1)
#define CMD_RESET_SINGLE_ID (8)
#define CMD_RESET_MULTI_ID  (0x0c)
#define CMD_RESET_ALL_ID    (7)
#define CMD_RESET_DFNUM     (3)
#define RESET_CMD_CONT      (10)

/* "exit" command parameter */
#define CMD_EXIT_ARG_NUM    (1)

/* port set parameter */
#define PSEL_VALUE          (0x2B)          /* Value of MPC.PxxPFS.PSEL */

/* module stop */
#define PRCR_PRKEY_BIT      (0xa500)
#define PRCR_PRC1_BIT       (2)
#define MSTPCRA2_BIT        (4)

/* timer setting */
#define TIMER_VALUE         (0xFFFF)
#define CMSTR2_CONT_START   (1)
#define WAIT_CONT           (378)

typedef char char_t;
typedef void (*cmd_func_t)(char_t *parg[], const uint32_t arg_num);

extern uint8_t g_fac_config_dat[];
extern uint32_t g_pinmux_config[];

static r_fac_err_t fac_trans_req(uint8_t id, r_fac_req_t *const preq, uint16_t tout);
static r_fac_err_t fac_trans_async(uint8_t id, r_fac_req_t *const preq, uint16_t tout);
static r_fac_err_t fac_trans_e2prom(uint8_t id, r_fac_e2prom_data_t *const pe2prom_data, uint16_t tout);
static void callback_req_result(r_fac_result_t *presult, uint8_t *prxdf);
static void callback_e2prom_result(r_fac_result_t *presult, uint8_t adf, uint8_t edf);

static void enable_int(void);


volatile static bool fac_flg_done;
static r_fac_result_t *pfac_result;
static uint8_t *pfac_rxdf;
volatile static uint8_t fac_adf;
static uint8_t fac_edf;
extern uint8_t g_fac_config[];

extern void enc_fac_ch0_int_isr(void);
extern void enc_fac_ch1_int_isr(void);

#ifndef RAM_EXECUTION
#define A_ENCIF0_WBLOCK_ADDR	(uint8_t*)(0x00028000u)
#define A_ENCIF1_WBLOCK_ADDR	(uint8_t*)(0x00044000u)
#endif

int32_t main_fac(const int32_t idx, int32_t br)
{
    const r_enc_isr_t int_isr_func_ch0=
    {
        enc_fac_ch0_int_isr, NULL
    };

    const r_enc_isr_t int_isr_func_ch1=
    {
        enc_fac_ch1_int_isr, NULL
    };

    uint8_t id;
    unsigned char *p;
    if (idx == 0)
    {
        id = R_FAC0_ID;
        r_enc_int_isr_open(idx, &int_isr_func_ch0);
        r_pinmux_encif04_create(ETYPE_APE_FACODER);
        r_enc_ch0_elc_start();
		    p = A_ENCIF0_WBLOCK_ADDR;
    }
    else if (idx == 1)
    {
        id = R_FAC1_ID;
        r_enc_int_isr_open(idx, &int_isr_func_ch1);
        r_pinmux_encif59_create(ETYPE_APE_FACODER);
		    p = A_ENCIF1_WBLOCK_ADDR;
    }
    else
    {
        fac_flg_done = true;
        return 0;
    }
    copy_to_rodata_encif(p, g_fac_config_dat, 0x3000u);

#if 0
    cur_id = id;
#endif
    
    int32_t ret_code = R_ECL_Initialize();
    if (R_ECL_SUCCESS != ret_code)
    {
        MY_PRINTF("R_ECL_Init: error(%ld)" MY_PRINTF_END, ret_code);
        fac_flg_done = true;
        return ret_code;
    }

    ret_code = R_ECL_ConfigurePin(g_pinmux_config);
    if (R_ECL_SUCCESS != ret_code)
    {
        MY_PRINTF("R_ECL_ConfigurePin: error(%ld)" MY_PRINTF_END, ret_code);
        fac_flg_done = true;
        return ret_code;
    }

#ifndef RAM_EXECUTION
    // Configure Encoder I/F FA-Coder
    ret_code = R_ECL_Configure(id, p);
#else
    // Configure Encoder I/F FA-Coder
    ret_code = R_ECL_Configure(id, g_fac_config_dat);
#endif
    if (R_ECL_SUCCESS != ret_code)
    {
        MY_PRINTF("R_ECL_Configure: error(%ld)" MY_PRINTF_END, ret_code);
        fac_flg_done = true;
        return ret_code;
    }

    ret_code = R_ECL_Start(id, R_FAC_FREQ);
    if (R_ECL_SUCCESS != ret_code)
    {
        MY_PRINTF("R_ECL_Start: error(%ld)" MY_PRINTF_END, ret_code);
        fac_flg_done = true;
        return ret_code;
    }
    
    // Enable IRQ interrupt
    enable_int();

    r_fac_err_t err_code = R_FAC_Open(id);
    if (R_FAC_SUCCESS != err_code)
    {
        MY_PRINTF("R_FAC_Open: error(%ld)" MY_PRINTF_END, (int32_t)err_code);
        ret_code = (int32_t)err_code;
    }
    
func_end:    
    fac_flg_done = true;  
    return ret_code;
}

int32_t fac_close(const int32_t idx)
{
    uint8_t id;
    int32_t ret_code = 0;
    
    if(idx==0)
    {
        id = R_FAC0_ID;
        r_pinmux_encif04_release();
    }
    else
    {
        id = R_FAC1_ID;
        r_pinmux_encif59_release();
    }
    
    ret_code = R_FAC_Close(id);
    if (R_FAC_SUCCESS != ret_code)
    {
        MY_PRINTF("R_FAC_Close: error(%d)" MY_PRINTF_END, ret_code);
    }

    ret_code = R_ECL_Stop(id);
    if (R_ECL_SUCCESS != ret_code)
    {
        MY_PRINTF("R_ECL_Stop: error(%d)" MY_PRINTF_END, ret_code);
    }

    return ret_code;
}

static r_fac_err_t fac_trans_req(uint8_t id, r_fac_req_t *const preq, uint16_t tout)
{
    r_fac_err_t err_code;
    uint32_t tm_count=0u;

    preq->presult_cb = &callback_req_result;
    preq->timotn = tout;
    // Wait for an ongoing transaction to complete
    while (!fac_flg_done)
    {
        R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);
        tm_count++;
        if(10 < tm_count)
        {
            return R_FAC_ERR_BUSY;
        }
    }
    
    fac_flg_done = false;
    err_code = R_FAC_Control(id, R_FAC_CMD_REQ, preq);
    if (R_FAC_SUCCESS == err_code)
    {
        tm_count=0u;
        while (false == fac_flg_done)
        {
            R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);
            tm_count++;
            if(10 < tm_count)
            {
                err_code= R_FAC_ERR_BUSY;
                break;
            }
        }
    }
    else
    {
        MY_PRINTF("  R_FAC_Control(R_FAC_CMD_REQ) error: %ld" MY_PRINTF_END, (int32_t)err_code);
    }
    return err_code;
}

static r_fac_err_t fac_trans_async(uint8_t id, r_fac_req_t *const preq, uint16_t tout)
{
    preq->presult_cb = &callback_req_result;
    preq->timotn = tout;
    if (!fac_flg_done)
    {
        return R_FAC_ERR_BUSY;
    }
    fac_flg_done = false;
    return R_FAC_Control(id, R_FAC_CMD_REQ, preq);
}

static r_fac_err_t fac_trans_e2prom(uint8_t id, r_fac_e2prom_data_t *const pe2prom_data, uint16_t tout)
{
    r_fac_err_t err_code;
    uint32_t tm_count=0u;

    pe2prom_data->presult_cb = &callback_e2prom_result;
    pe2prom_data->timotn = tout;
    // Wait for an ongoing transaction to complete
    while (!fac_flg_done)
    {
        R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);
        tm_count++;
        if(10 < tm_count)
        {
            return R_FAC_ERR_BUSY;
        }
    }
    
    fac_flg_done = false;
    err_code = R_FAC_Control(id, R_FAC_CMD_E2PROM, pe2prom_data);
    if (R_FAC_SUCCESS == err_code)
    {
        tm_count=0u;
        while (false == fac_flg_done)
        {
            R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);
            tm_count++;
            if(10 < tm_count)
            {
                err_code= R_FAC_ERR_BUSY;
                break;
            }
        }
    }
    else
    {
        MY_PRINTF("  R_FAC_Control(R_FAC_CMD_E2PROM) error: %ld" MY_PRINTF_END, (int32_t)err_code);
    }
    return err_code;
}

static void callback_req_result(r_fac_result_t *presult, uint8_t *prxdf)
{
    pfac_result = presult;
    pfac_rxdf = prxdf;
    fac_flg_done = true;
}

static void callback_e2prom_result(r_fac_result_t *presult, uint8_t adf, uint8_t edf)
{
    pfac_result = presult;
    fac_adf = adf;
    fac_edf = edf;
    fac_flg_done = true;
}

static void enable_int(void)
{
    __asm("cpsie i");
    __asm("isb");
}

long fac_pos(struct t_motor* pm)
{
    uint8_t origin_flg = false;
    // Process the result of the last transaction if it was successful
    if (R_FAC_RX_SUCCESS == pfac_result->result)
    {
        int32_t res_old = (pm->real_res & 0xc0000000);

        uint32_t single_turn = ((((uint32_t)pfac_rxdf[RXDF_SINGLE0_INDEX] & RXDF_SINGLE0_MASK) << RXDF_SINGLE0_BIT) |
                                (((uint32_t)pfac_rxdf[RXDF_SINGLE1_INDEX] & RXDF_SINGLE1_MASK) << RXDF_SINGLE1_BIT)) |
                               (((uint32_t)pfac_rxdf[RXDF_SINGLE2_INDEX] & RXDF_SINGLE2_MASK) << RXDF_SINGLE2_BIT);
        
        // Convert result to 32-bit resolution per single turn
#if FA_CODER_FMT23
        int32_t res_new = (int32_t)(single_turn << 9);
#endif        
#if FA_CODER_FMT17
        int32_t res_new = (int32_t)(single_turn << 15);
#endif        
#if FA_CODER_FMT14
        int32_t res_new = (int32_t)(single_turn << 18);
#endif        
        pm->real_res = res_new;
        
        if (res_old == 0 && res_new < 0) 
        {
            if (pm->real_rot == (int16_t)0x8000) 
            {
                pm->act_state |= ACT_WrapAround;
            }
            pm->real_rot--; // decreement rotation
            
            origin_flg = true; // origin detection
            pm->captured_pos.Reg16.Low  = 0;
            pm->captured_pos.Reg16.High = (short)((pm->real_rot + 1) & 0xffff);
        } 
        else if (res_old == (int32_t)(0xc0000000) && res_new >= 0) 
        { 
            if (pm->real_rot == (int16_t)0x7fff) 
            {
                pm->act_state |= ACT_WrapAround;
            }
            pm->real_rot++; // increment rotation
            
            origin_flg = true; // origin detection
            pm->captured_pos.Reg16.Low  = 0;
            pm->captured_pos.Reg16.High = (short)(pm->real_rot & 0xffff);
        }
    }
    if (pm->enc_open)
    {
        // Initiate new transaction
        r_fac_req_t r_fac_req;
        r_fac_req.txid = CMD_SINGLE_ID;
        r_fac_req.dfnum = CMD_SINGLE_DFNUM;
        extern struct t_motor m1;
        extern struct t_motor m2;
        int ch = pm == &m1 ? R_FAC0_ID : R_FAC1_ID;
        fac_trans_async(ch, &r_fac_req, (uint16_t)pm->enc_timeout);
    }
    if(origin_flg == true)
    {
        // Apply position Offset parameter
        pm->captured_pos.Reg32 += pm->pos_offset;
        // Update Activity Status
        pm->act_state |= ACT_PosCaptured;
        pm->index_pos = pm->captured_pos.Reg32;
    }
    int32_t pos_new = (int32_t)((( (uint32_t)pm->real_rot << 16 ) & 0xffff0000) |
                                (( (uint32_t)pm->real_res >> 16 ) & 0x0000ffff));
    return pos_new;
}

long fac_enc_id(struct t_motor* pm)
{
    uint32_t encid=99u;
            
    r_fac_req_t r_fac_req;
    r_fac_err_t err_code;

    r_fac_req.txid = CMD_ENCID_ID;
    r_fac_req.dfnum = CMD_ENCID_DFNUM;
    extern struct t_motor m1;
    extern struct t_motor m2;
    int ch = pm == &m1 ? R_FAC0_ID : R_FAC1_ID;
    err_code = fac_trans_req(ch, &r_fac_req, (uint16_t)pm->enc_timeout);

    if (R_FAC_SUCCESS == err_code)
    {
        if (R_FAC_RX_SUCCESS == pfac_result->result)
        {
            uint16_t st = 0;
            encid = ((uint32_t)pfac_rxdf[RXDF_ENCID_INDEX] & RXDF_ENCID_MASK) << RXDF_ENCID_BIT;

            if (pfac_result->rxsfea & 0x01)
                st |= ESTATE_COUNT_ERR;
            if (pfac_result->rxsfea & 0x02)
                st |= ESTATE_MULTI_ERR;
            if (pfac_result->rxsfca & 0x01)
                st |= ESTATE_PARITY_ERR;
            if (pfac_result->rxsfca & 0x02)
                st |= ESTATE_DELIM_ERR;
            
            pm->enc_status = st;
        }
        else
        {
            encid = 0;
        }
    }
    return (long) encid;
}

long fac_eeprom_write(struct t_motor *pm, long write_data)
{
    r_fac_err_t err_code;
    
    r_fac_e2prom_data_t r_fac_e2prom_data;
    r_fac_e2prom_data.adr = (uint8_t)pm->eeprom_addr;
    r_fac_e2prom_data.data = (uint8_t)write_data;
    r_fac_e2prom_data.dir = R_FAC_E2PROM_WRITE;

    extern struct t_motor m1;
    extern struct t_motor m2;
    int ch = pm == &m1 ? R_FAC0_ID : R_FAC1_ID;
    err_code = fac_trans_e2prom(ch, &r_fac_e2prom_data, (uint16_t)pm->enc_timeout);
    if (R_FAC_SUCCESS == err_code)
    {
        if (R_FAC_RX_SUCCESS == pfac_result->result)
        {
          /*
            MY_PRINTF("  result:success" MY_PRINTF_END);
            MY_PRINTF("    adf data:     %d" MY_PRINTF_END,  fac_adf);
            MY_PRINTF("    edf data:     %d" MY_PRINTF_END,  fac_edf);
            MY_PRINTF("    request id:   %2XH" MY_PRINTF_END, pfac_result->rxid);
            MY_PRINTF("    parity bit:   %2XH" MY_PRINTF_END, pfac_result->rxidp);
            MY_PRINTF("    crc data:     %2XH" MY_PRINTF_END, pfac_result->crc);
          */
            return 0;
        }
    }
    return -1;
}

long fac_eeprom_read(struct t_motor *pm, long *read_data)
{
    r_fac_e2prom_data_t r_fac_e2prom_data;
    r_fac_err_t err_code;

    r_fac_e2prom_data.adr = (char)pm->eeprom_addr;
    r_fac_e2prom_data.dir = R_FAC_E2PROM_READ;

    extern struct t_motor m1;
    extern struct t_motor m2;
    int ch = pm == &m1 ? R_FAC0_ID : R_FAC1_ID;
    err_code = fac_trans_e2prom(ch, &r_fac_e2prom_data, (uint16_t)pm->enc_timeout);

    if (R_FAC_SUCCESS == err_code)
    {
        if (R_FAC_RX_SUCCESS == pfac_result->result)
        {
          /*
            MY_PRINTF("  result:success" MY_PRINTF_END);
            MY_PRINTF("    adf data:     %d" MY_PRINTF_END,  fac_adf);
            MY_PRINTF("    edf data:     %d" MY_PRINTF_END,  fac_edf);
            MY_PRINTF("    request id:   %2XH" MY_PRINTF_END, pfac_result->rxid);
            MY_PRINTF("    parity bit:   %2XH" MY_PRINTF_END, pfac_result->rxidp);
            MY_PRINTF("    crc data:     %2XH" MY_PRINTF_END, pfac_result->crc);
          */
            *read_data = fac_edf;
            return 0;
        }
    }
    return -1;
}

                             
