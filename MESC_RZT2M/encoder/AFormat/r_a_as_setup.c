#include "r_a_as_rzt2_if.h"

#include "iodefine_a_as.h"

#include "r_enc_int_rzt2.h"

#include <ecl/r_ecl_rzt2_if.h>

#include <eType.h>
#include <tMotor.h>

#include <bsp_api.h>
#include <hal_data.h>

#include <stdio.h>
#include <stdlib.h>

#define R_A_AS0_ID      (R_ECL_CH_0)
#define R_A_AS1_ID      (R_ECL_CH_1)
//#define R_A_AS_FREQ     (19)

#define A_AS_FMT24      0
#define A_AS_FMT20      1
#define A_AS_FMT17      0

//#define DEBUG_PRINT(...)            (printf(__VA_ARGS__))
#define R_A_AS_FREQ (R_ECL_FREQ_10000KHZ)  //r_ecl_rzt2_if.h
#define DEBUG_PRINT(...)
#define DEBUG_PRINT_END             "\n"        /* Line feed code */

#define ARG_MAX                     (7)         /* maximum number of command arguments */
#define CMD_BUF_SIZE                (256)       /* command buffer size */
#define CMD_NUM                     (4)         /* number of commands */
#define EXIT_CMD_NUM                (3)         /* exit command number */
#define CMD_DELIMITER               (" \t\r\n") /* command line delimiter */
#define CMD_EXIT_ARG_NUM            (1)         /* "exit" command parameter */
#define PSEL_VALUE                  (0x2B)      /* Value of MPC.PxxPFS.PSEL */

#define A_AS_CDF_NUM                (27u)       /* CDF number of commands */
#define A_AS_CDF_CHAR_NUM           (5u)        /* CDF number of characters  */
#define A_AS_ENC_NUM                (8u)        /* The number of encoder */
#define A_AS_IFMG_4MBPS             (0x0014)    /* Delay of the cable length */
#define A_AS_ENC_INIT_WAIT          (400u)      /* 400us wait */
#define A_AS_ENC_INIT_LOOP          (8)         /* Command transmission number of times */
#define A_AS_OPEN_ERR               (-1)        /* Failure to run the R_A_AS_Open function */
#define A_AS_TI_TBL_NUM             (10)        /* The number of received data by the timer operation */
#define A_AS_ARG1                   (0)         /* First argument */
#define A_AS_ARG2                   (1)         /* Second argument */
#define A_AS_ARG3                   (2)         /* Third argument */
#define A_AS_ARG4                   (3)         /* Fourth argument */
#define A_AS_ARG5                   (4)         /* Fifth argument */
#define A_AS_ARG6                   (5)         /* Sixth argument */
#define A_AS_ARG7                   (6)         /* Seventh argument */
#define A_AS_ARG_SUCCESS            (0)         /* The argument is normal */
#define A_AS_ARG_ERR                (-1)        /* Argument error */
#define A_AS_DEC                    (10)        /* Decimal number */
#define A_AS_HEX                    (16)        /* Hexadecimal */
#define A_AS_ADR_CONV               (1)         /* Convert the encoder number in address */
#define A_AS_MEM_ADR_MAX            (0xFF)      /* The upper limit of the address of the encoder of memory */
#define A_AS_MEM_DATA_MAX           (0xFFFF)    /* The upper limit of the data of the encoder of memory */
#define A_AS_ID_MAX                 (0xFFFFFF)  /* The upper limit of the encoder ID */
#define A_AS_TXFLD_MAX              (0xFFFF)    /* The upper limit of hte txfld */

#define CMT1_CMCOR_MAX_TO_US        (0xFFFF * 8 / 75) /* maximum time (us) to set CMCOR (PCLKD/8) */

#define BIT_SHIFT_RXI_ES            (8)
#define BIT_SHIFT_RXI_CC            (3)
#define BIT_SHIFT_MEMADR            (16)
#define BIT_MASK_EA                 (0x00000007)
#define BIT_MASK_ES                 (0x0000000F)
#define BIT_MASK_CC                 (0x0000001F)
#define BIT_MASK_L17                (0x0001FFFF)
#define BIT_MASK_MEMADR             (0x000000FF)
#define BIT_MASK_MEMDAT             (0x0000FFFF)
#define BIT_MASK_DB                 (0x000003FF)
#define BIT_SHIFT_EA_CDF21_22       (24)
#define BIT_SHIFT_RXI_ES_CDF21_22       (27)
#define BIT_MASK_ES_CDF21_22        (0x00000001)

#define PARAMETER_NOT_USED(p)       (void)((p))

typedef char char_t;
typedef void (*cmd_func_t)(uint32_t arg_num, char_t *parg[]);

extern uint8_t g_a_as_config_dat[];
extern uint32_t g_pinmux_config[];

// A-format sample program
static int32_t a_as_enc_init(const int32_t idx);
static void a_as_txerr_callback(r_a_as_result_t *presult);
static void a_as_rxset_callback(r_a_as_result_t *presult);
static void a_as_rxend_callback(r_a_as_result_t *presult);

volatile static bool a_as_ti_flg = false;
volatile static bool a_as_elc_flg = false;
volatile static bool elc_trans_flg = false;
volatile static bool a_as_flg_done;
volatile static bool a_as_init_end_flg;
static r_a_as_result_t *pa_as_result;
volatile static bool a_as_ti_flg;

#define R_A_AS_2500KBPS                 (0u)
#define R_A_AS_4MBPS                    (1u)
#define R_A_AS_6670KBPS                 (2u)
#define R_A_AS_8MBPS                    (3u)

extern void enc_a_as_ch0_int_isr(void);
extern void enc_a_as_ch1_int_isr(void);


uint8_t a_as_encode_br(int32_t br)
{
uint8_t result;

    switch (br)
    {
    case 8000:
        result = R_A_AS_8MBPS;
        break;
    case 6670:
        result = R_A_AS_6670KBPS;
        break;
    case 4000:
        result = R_A_AS_4MBPS;
        break;
    case 2500:
        result = R_A_AS_2500KBPS;
        break;
    default:
        result = R_A_AS_2500KBPS;
        break;
    }
        result = R_A_AS_4MBPS;
    return result;
}

#ifndef RAM_EXECUTION
#define A_ENCIF0_WBLOCK_ADDR	(uint8_t*)(0x00028000u)
#define A_ENCIF1_WBLOCK_ADDR	(uint8_t*)(0x00044000u)
#endif

int32_t main_a_as(const int32_t idx, uint32_t br)
{
    uint8_t id;
    unsigned char *p;
    
    const r_enc_isr_t int_isr_func_ch0=
    {
        enc_a_as_ch0_int_isr, NULL
    };

    const r_enc_isr_t int_isr_func_ch1=
    {
        enc_a_as_ch1_int_isr, NULL
    };

    if (idx == 0)
    {
        id = R_A_AS0_ID;
        r_enc_int_isr_open(idx, &int_isr_func_ch0);
        r_pinmux_encif04_create(ETYPE_APE_AFORMAT);
		    p = A_ENCIF0_WBLOCK_ADDR;
    }
    else if (idx == 1)
    {
        id = R_A_AS1_ID;
        r_enc_int_isr_open(idx, &int_isr_func_ch1);
        r_pinmux_encif59_create(ETYPE_APE_AFORMAT);
		    p = A_ENCIF1_WBLOCK_ADDR;
    }
    else
    {
      goto end;
    }
    copy_to_rodata_encif(p, g_a_as_config_dat, 0xf580u);

#if 0
    cur_id = id;
#endif
    
    int32_t ret_err_code = R_ECL_Initialize();
    if (R_ECL_SUCCESS != ret_err_code)
    {
      DEBUG_PRINT("R_ECL_Init: error(%ld)" DEBUG_PRINT_END, ret_err_code);
      goto end;
    }
    // Configuration of Encoder I/F A_AS
    ret_err_code = R_ECL_ConfigurePin(g_pinmux_config);
    if (R_ECL_SUCCESS != ret_err_code)
    {
        DEBUG_PRINT("R_ECL_ConfigurePincfg: error(%ld)" DEBUG_PRINT_END, ret_err_code);
        goto end;
    }
#ifndef RAM_EXECUTION
    // Configuration of Encoder I/F A_AS
    ret_err_code = R_ECL_Configure(id, p);
#else
    // Configuration of Encoder I/F A_AS
    ret_err_code = R_ECL_Configure(id, g_a_as_config_dat);
#endif
    if (R_ECL_SUCCESS != ret_err_code)
    {
        DEBUG_PRINT("R_ECL_Configure: error(%ld)" DEBUG_PRINT_END, ret_err_code);
        goto end;
    }

    // Start of Encoder I/F A_AS
    ret_err_code = R_ECL_Start(id, R_A_AS_FREQ);
    if (R_ECL_SUCCESS != ret_err_code)
    {
        DEBUG_PRINT("R_ECL_Start: error(%ld)" DEBUG_PRINT_END, ret_err_code);
        goto end;
    }

    // Control of A_AS
    r_a_as_info_t a_as_info;

    a_as_info.connect = R_A_AS_ONE_FOR_ONE;         // R_A_AS_BUS alternatively
    a_as_info.bitrate = a_as_encode_br(br);
    a_as_info.ifmg    = A_AS_IFMG_4MBPS;            // need to be confugurable for higher bitrates
    ret_err_code = R_A_AS_Open(id, &a_as_info);
    if (R_A_AS_SUCCESS != ret_err_code)
    {
        DEBUG_PRINT("R_A_AS_Open: error(%d)" DEBUG_PRINT_END, ret_err_code);
        goto end;
    }

    a_as_init_end_flg = false;
    // Status clear of encoder
    ret_err_code = a_as_enc_init(id);
    if (R_A_AS_SUCCESS != ret_err_code)
    {
        DEBUG_PRINT("Failed to initialize the encoder" DEBUG_PRINT_END);
        R_A_AS_Close(id);
    }

end:
    a_as_init_end_flg = true;
    return ret_err_code;
}

int32_t a_as_close(const int32_t idx)
{ 
    uint8_t id = (idx == 0) ? R_A_AS0_ID : R_A_AS1_ID;
    
    if (idx == 0)
    {
        r_pinmux_encif04_release();
    }
    else
    {
        r_pinmux_encif59_release();
    }
    
    int32_t ret_err_code = R_A_AS_Close(id);
    if (R_A_AS_SUCCESS != ret_err_code)
    {
        DEBUG_PRINT("R_A_AS_Close: error(%d)" DEBUG_PRINT_END, (int)ret_err_code);
    }
    
    // Stop of Encoder I/F A_AS
    ret_err_code = R_ECL_Stop(id);
    if (R_ECL_SUCCESS != ret_err_code)
    {
        DEBUG_PRINT("R_ECL_Stop: error(%d)" DEBUG_PRINT_END, (int)ret_err_code);
    }
    return ret_err_code;
}

static int32_t a_as_enc_init(const int32_t idx)
{
    int32_t ret_err_code = 0;
    r_a_as_req_t a_as_req_param;
    uint8_t loop_req;

    for (loop_req = 0; loop_req < A_AS_ENC_INIT_LOOP; loop_req++)
    {
        /* Set of send parameters  */
        a_as_req_param.encadr         = R_A_AS_ECN1;
        a_as_req_param.cmd            = R_A_AS_CDF8;
        a_as_req_param.cbadr_txerr    = &a_as_txerr_callback;
        a_as_req_param.cbadr_rxset    = &a_as_rxset_callback;
        a_as_req_param.cbadr_rxend    = &a_as_rxend_callback;
//        a_as_req_param.txbpe          = false;
//        a_as_req_param.rxbpe          = false;
        a_as_req_param.pre            = false;
        ret_err_code = R_A_AS_Control(idx, R_A_AS_CMD_SET_PARAM, &a_as_req_param);
        if (R_A_AS_SUCCESS != ret_err_code)
        {
            DEBUG_PRINT("  R_A_AS_Control(R_A_AS_CMD_SET_PARAM) error: %ld" DEBUG_PRINT_END, ret_err_code);
            return ret_err_code;
        }

        /* Send by a single trigger */
        a_as_flg_done = false;
        ret_err_code = R_A_AS_Control(idx, R_A_AS_CMD_TX_TRG, NULL);
        if (R_A_AS_SUCCESS == ret_err_code)
        {
            while (false == a_as_flg_done) {};
        }
        else
        {
            DEBUG_PRINT("  R_A_AS_Control(R_A_AS_CMD_TX_TRG) error: %d" DEBUG_PRINT_END, ret_err_code);
            return ret_err_code;
        }
    }

end:
    return ret_err_code;
}

static void a_as_txerr_callback(r_a_as_result_t* presult)
{
    pa_as_result = presult;
}

static void a_as_rxset_callback(r_a_as_result_t* presult)
{
    pa_as_result = presult;
}

static void a_as_rxend_callback(r_a_as_result_t* presult)
{
    PARAMETER_NOT_USED(presult);
    a_as_flg_done = true;
}

long a_as_pos(struct t_motor* pm)
{
    int32_t ret_err_code = 0;
    r_a_as_req_t a_as_req_param;
    uint8_t origin_flg = false;

    // Make sure the last transaction is completed
    if ((a_as_init_end_flg == true)&&
        (a_as_flg_done == true))
    {
        // Make sure the last transaction is successful before consuming the result
        if (pa_as_result->result == R_A_AS_REQ_SUCCESS)
        {
            int32_t res_old = (pm->real_res & 0xc0000000);

            // Convert result to 32-bit resolution per single turn
            // The content of rxd0 depends on the command request
#if A_AS_FMT24
            // CDF1 (24-bit) abs position
            uint32_t single_turn = pa_as_result->data.rxd0;
            int32_t res_new = (int32_t)(single_turn << 8);
#endif
#if A_AS_FMT20
            // CDF1 (20-bit) abs position
            uint32_t single_turn = pa_as_result->data.rxd0;
            int32_t res_new = (int32_t)(single_turn << 12);
#endif
#if A_AS_FMT17
            // CDF21 (17-bit) abs position (single turn)
            uint32_t single_turn = pa_as_result->data.rxi;
            int32_t res_new = (int32_t)(single_turn << 15);
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
#if A_AS_FMT24
            // CDF1 Get lower 24bit data - result is stored in the rxd0 register
            a_as_req_param.cmd = R_A_AS_CDF1;
#endif
#if A_AS_FMT20
            // CDF1 Get lower 20bit data - result is stored in the rxd0 register
            a_as_req_param.cmd = R_A_AS_CDF1;
#endif
#if A_AS_FMT17
            // CDF21 Get lower 17bit data - result is stored in the rxi register
            // (change of the command code requires change in the parsing of the result)
            a_as_req_param.cmd = R_A_AS_CDF21;
#endif
            // Always use ECN1 address
            a_as_req_param.encadr = R_A_AS_ECN1;

            // Send request - receive reply asynchronously via callback
            a_as_req_param.cbadr_txerr = &a_as_txerr_callback;
            a_as_req_param.cbadr_rxset = &a_as_rxset_callback;
            a_as_req_param.cbadr_rxend = &a_as_rxend_callback;
            // a_as_req_param.txbpe    = false;
            // a_as_req_param.rxbpe    = false;
            a_as_req_param.pre         = false;
            ret_err_code = R_A_AS_Control(R_A_AS0_ID, R_A_AS_CMD_SET_PARAM, &a_as_req_param);
            if (R_A_AS_SUCCESS != ret_err_code)
            {
                DEBUG_PRINT("  R_A_AS_Control(R_A_AS_CMD_SET_PARAM) error: %d" DEBUG_PRINT_END, ret_err_code);
                return 0;
            }

            a_as_flg_done = false;

            extern struct t_motor m1;
            extern struct t_motor m2;
            int ch = pm == &m1 ? 0 : 1; // Encoder channel index
            int chid = (ch == 0) ? R_A_AS0_ID : R_A_AS1_ID;
            // Send by a single trigger
            ret_err_code = R_A_AS_Control(chid, R_A_AS_CMD_TX_TRG, NULL);
            if (R_A_AS_SUCCESS != ret_err_code)
            {
                DEBUG_PRINT("  R_A_AS_Control(R_A_AS_CMD_TX_TRG) error: %d" DEBUG_PRINT_END, ret_err_code);
                return 0;
            }
        }
    }
    if (origin_flg == true)
    {
        pm->captured_pos.Reg32 += pm->pos_offset; // Apply position Offset parameter
        pm->act_state |= ACT_PosCaptured; // Update Activity Status
        pm->index_pos = pm->captured_pos.Reg32;
    }
    int32_t pos_new = (int32_t)((( (uint32_t)pm->real_rot << 16 ) & 0xffff0000) |
                                (( (uint32_t)pm->real_res >> 16 ) & 0x0000ffff));
    return pos_new;
end:
    return 0;
}

long a_as_enc_id(struct t_motor *pm)
{
    return 0;
}

long a_as_eeprom_read(struct t_motor *pm, long *read_data)
{
    return 0;
}

long a_as_eeprom_write(struct t_motor *pm, long write_data)
{
    return 0;
}

