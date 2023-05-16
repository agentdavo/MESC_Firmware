#include "r_biss_rzt2_if.h"

#include "bissc/r_bissc_rzt2_if.h"

#include "r_enc_int_rzt2.h"

#include <ecl/r_ecl_rzt2_if.h>

#include <bsp_api.h>
#include <hal_data.h>

#include <eType.h>
#include <tMotor.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define R_BISS0_ID  (R_ECL_CH_0)
#define R_BISS1_ID  (R_ECL_CH_1)
#define R_BISS_FREQ (R_ECL_FREQ_10000KHZ)

/* CRC Setting */
#define BISSC_CRC_SIZE          (6)             /* BiSS Encoder crc size */
#define BISSC_CRC_POLY          (0x0021)
#define BISSC_CRC_INIT          (0x0000)

#define BISSC_MT_DATA_BIT       (24)            /* multi-turn data size(bit) for AD36/1219 */
#define BISSC_ST_DATA_BIT       (17)            /* single-turn data size(bit) for AD36/1219 */
#define BISSC_AL_DATA_BIT       (0)             /* Alignment data size(bit) for AD36/1219 or 5bit */

#define ARG_MAX                 (4)             /* maximum number of command arguments */
#define CMD_BUF_SIZE            (256)           /* command buffer size */
#define CMD_NUM                 (6)             /* number of commands */
#define EXIT_CMD_NUM            (5)             /* exit command number */
#define CMD_DELIMITER           (" \t\r\n")     /* command line delimiter */

/* "get position" command parameter */
#define CMD_POS_ARG_NUM         (1)

/* "read register" command parameter */
#define CMD_REG_READ_ARG_NUM    (3)
#define CMD_REQ_READ_ADR        (1)
#define CMD_REQ_READ_BYTE       (2)
#define CMD_REQ_ARG_BASE        (16)

/* "write register" command paramter */
#define CMD_REG_WRT_ARG_NUM     (4)
#define CMD_REG_WRT_ADR         (1)
#define CMD_REG_WRT_BYTE        (2)
#define CMD_REG_WRT_DATA        (3)

#define CODE_NULL               (0x00)
#define CODE_ZERO               (0x30)
#define CODE_NINE               (0x39)
#define CODE_BIG_A              (0x41)
#define CODE_BIG_F              (0x46)
#define CODE_SMA_A              (0x61)
#define CODE_SMA_F              (0x66)
#define MASK_HEX10              (0x10)
#define NUM_STR_TO_HEX          (0x30)
#define BIG_STR_TO_HEX          (0x37)
#define SMA_STR_TO_HEX          (0x57)

/* "exit" command paramter */
#define CMD_EXIT_ARG_NUM        (1)

/* number of timeout clock */
#define BISSC_TIMEOUT_CLK       (0x00FFFFFFu)

/* Port setting */
#define PORT_PDR_HIZ            (0x0u)          /* Port direction register(Non-use).                      */
#define PORT_PMR_IO             (0x0u)          /* Port mode register(I/O port for peripheral functions). */
#define PORT_PMR_BISS           (0x1u)          /* Port mode register(peripheral function is BiSS).       */
#define MPC_P7NPFS_PSEL         (0x2Bu)         /* P7n pin function control register(BiSS).               */
#define MPC_PRNPFS_PSEL         (0x2Bu)         /* PRn pin function control register(BiSS).               */

#define REG_MAX                 (64)

#define DEBUG_PRINT_END         "\n"            /* Line feed code */
//#define r_printf(...)           printf(__VA_ARGS__)
#define r_printf(...)

#undef BISS_VERSION                             /* This macro outputs the BiSS driver version information. */
#ifdef BISS_VERSION
#define R_SHIFT_16              (16uL)
#define MASK_MAJOR              (0x0000FFFFuL)
#endif /* BISS_VERSION */

#define PCLKMHZ                 (400u)
#define PARAMETER_NOT_USED(p)   (void)((p))

typedef char char_t;
typedef void (*cmd_func_t)(char_t *parg[], const uint32_t arg_num);

extern uint8_t g_biss_config_dat[];
extern uint32_t g_pinmux_config[];

volatile static bool bissc_flg_done;
volatile static bool bissc_register_flg;
static r_bissc_sensreq_t bissc_sensreq;
static r_bissc_sensordt_t *pbissc_result_sns = NULL;

extern void bissc0_rx_int_isr(void);
extern void bissc1_rx_int_isr(void);

uint16_t biss_encode_br(int32_t br)
{
    switch (br)
    {
    case 10000:
        return R_BISSC_BR_10MHZ;
    case 8333:
        return R_BISSC_BR_8_33MHZ;
    case 4000:
        return R_BISSC_BR_4MHZ;
    case 2500:
        return R_BISSC_BR_2_5MHZ;
    case 1000:
        return R_BISSC_BR_1MHZ;
    case 400:
        return R_BISSC_BR_400KHZ;
    case 299:
        return R_BISSC_BR_299_40KHZ;
    case 200:
        return R_BISSC_BR_200KHZ;
    case 100:
        return R_BISSC_BR_100KHZ;
    case 80:
        return R_BISSC_BR_80_12KHZ;
    default:
        break;
    }
    return R_BISSC_BR_2_5MHZ;
}

#ifndef RAM_EXECUTION
#define A_ENCIF0_WBLOCK_ADDR	(uint8_t*)(0x00028000u)
#define A_ENCIF1_WBLOCK_ADDR	(uint8_t*)(0x00044000u)
#endif

#ifdef BISS_VERSION
static void biss_ver_disp(const int32_t idx)
{
  uint32_t drv_ver = R_ECL_GetVersion();
  uint32_t drv_ver_major = drv_ver >> R_SHIFT_16;
  uint32_t drv_ver_miner = drv_ver & MASK_MAJOR;
  r_printf("Encoder I/F Configuration Library version is %ld.%ld"DEBUG_PRINT_END, drv_ver_major, drv_ver_miner);

  drv_ver = R_BISS_GetVersion(R_BISS_TYPE_CMN);
  drv_ver_major = drv_ver >> R_SHIFT_16;
  drv_ver_miner = drv_ver & MASK_MAJOR;
  r_printf("BiSS sample driver version is %ld.%ld"DEBUG_PRINT_END, drv_ver_major, drv_ver_miner);

  drv_ver = R_BISS_GetVersion(R_BISS_TYPE_C);
  drv_ver_major = drv_ver >> R_SHIFT_16;
  drv_ver_miner = drv_ver & MASK_MAJOR;
  r_printf("BiSS-C sample driver version is %ld.%ld"DEBUG_PRINT_END, drv_ver_major, drv_ver_miner);
}
#endif // BISS_VERSION

int32_t main_biss(const int32_t idx, int32_t br)
{
    const r_enc_isr_t int_isr_func_ch0=
    {
        bissc0_rx_int_isr, NULL
    };

    const r_enc_isr_t int_isr_func_ch1=
    {
        bissc1_rx_int_isr, NULL
    };
    
    // get version
#ifdef BISS_VERSION
    biss_ver_disp(idx);
#endif // BISS_VERSION
    r_printf("BiSS sample program start" DEBUG_PRINT_END);

    uint8_t id;
    unsigned char* p = NULL;
    if (idx == 0)
    {
        id = R_BISS0_ID;
        r_enc_int_isr_open(idx, &int_isr_func_ch0);
        r_pinmux_encif04_create(ETYPE_APE_BISS);
        r_enc_ch0_elc_start();
		    p = A_ENCIF0_WBLOCK_ADDR;
    }
    else if (idx == 1)
    {
        id = R_BISS1_ID;
        r_enc_int_isr_open(idx, &int_isr_func_ch1);
        r_pinmux_encif59_create(ETYPE_APE_BISS);
		    p= A_ENCIF1_WBLOCK_ADDR;
    }
    else
    {
        bissc_flg_done = true;
        return 0;
    }
    copy_to_rodata_encif(p, g_biss_config_dat, 0xf580u);

#if 0
    cur_id = id;
#endif

    int32_t ret_code = R_ECL_Initialize();
    if (R_ECL_SUCCESS != ret_code)
    {
        r_printf("R_ECL_Init: error(%ld)" DEBUG_PRINT_END, ret_code);
        bissc_flg_done = true;
        return ret_code;
    }
    // Configure
    ret_code = R_ECL_ConfigurePin(g_pinmux_config);
    if (R_ECL_SUCCESS != ret_code)
    {
        r_printf("R_ECL_ConfigurePincfg: error(%ld)" DEBUG_PRINT_END, ret_code);
        bissc_flg_done = true;
        return ret_code;
    }

#ifndef RAM_EXECUTION
    // Configuration of Encoder I/F
    ret_code = R_ECL_Configure(id, p);
#else
    ret_code = R_ECL_Configure(id,g_biss_config_dat);
#endif
    if (R_ECL_SUCCESS != ret_code)
    {
        r_printf("R_ECL_ConfigurePincfg: error(%ld)" DEBUG_PRINT_END, ret_code);
        bissc_flg_done = true;
        return ret_code;
    }

    ret_code = R_ECL_Start(id, R_BISS_FREQ);
    if (R_ECL_SUCCESS != ret_code)
    {
        r_printf("R_ECL_Start: error(%ld)"DEBUG_PRINT_END, ret_code);
        bissc_flg_done = true;
        return ret_code;
    }

    __asm("cpsie i");
    __asm("isb");

    r_biss_info_t biss_info;
    biss_info.clk_freq                  = biss_encode_br(br); // set clock frequency
    biss_info.crc_info.crc_size         = BISSC_CRC_SIZE; // set crc
    biss_info.crc_info.crc_polynomial   = BISSC_CRC_POLY;
    biss_info.crc_info.crc_start_value  = BISSC_CRC_INIT;
    biss_info.mtdata_size               = BISSC_MT_DATA_BIT; // set multi-turn data size(bit)
    biss_info.stdata_size               = BISSC_ST_DATA_BIT; // set single-turn data size(bit)
    biss_info.aldata_size               = BISSC_AL_DATA_BIT; // set Alignment data size(bit)
//    bissc_cmd_flg                       = false;
    ret_code = R_BISS_Open(id, R_BISS_TYPE_C, &biss_info);
    if (R_BISS_SUCCESS != ret_code)
    {
        r_printf("R_BISS_Open: error(%d)"DEBUG_PRINT_END, ret_code);
        r_biss_err_t err_code = R_ECL_Stop(id);
        if (R_ECL_SUCCESS != err_code)
        {
            r_printf("R_ECL_Stop: error(%d)"DEBUG_PRINT_END, err_code);
        }
    }
    bissc_flg_done = true;
    return ret_code;
}

int32_t biss_close(const int32_t idx)
{
    uint8_t id;
    if (idx == 0)
    {
        id = R_BISS0_ID;
        r_pinmux_encif04_release();
        r_enc_ch0_elc_stop();
    }
    else
    {
        id = R_BISS1_ID;
        r_pinmux_encif59_release();
        r_enc_ch1_elc_stop();
    }

    int32_t ret_code = 0;
    r_biss_err_t err_code = R_BISS_Close(id);
    if (R_BISS_SUCCESS != err_code)
    {
        r_printf("R_BISS_Close: error(%d)"DEBUG_PRINT_END, err_code);
        ret_code = (int32_t)err_code;
    }

    ret_code = R_ECL_Stop(id);
    if (R_ECL_SUCCESS != ret_code)
    {
        r_printf("R_ECL_Start: error(%d)"DEBUG_PRINT_END, ret_code);
    }
    
    return ret_code;
}

static void callback_get_pos(void *presult)
{
    pbissc_result_sns = (r_bissc_sensordt_t *)presult;
    bissc_flg_done = true;
}

long bissc_get_pos(struct t_motor *pm)
{
    uint8_t origin_flg = false;
    if (bissc_flg_done == true)
    {
        bissc_flg_done = false;
        // Handle the reply from the last position read request
        if (NULL != pbissc_result_sns && R_BISSC_SUCCESS == pbissc_result_sns->result)
        {
            int32_t res_old = (pm->real_res & 0xc0000000);

            uint32_t single_turn = pbissc_result_sns->stdata;

            // Convert result to 32-bit resolution per single turn
 #if BISSC_ST_DATA_BIT == 19
            int32_t res_new = (int32_t)(single_turn << 13);
 #elif BISSC_ST_DATA_BIT == 17
            // 2's complement because CW -> CCW
            int32_t res_new = (int32_t)(((~single_turn)&0x1FFFFu) << 15);
 #elif BISSC_ST_DATA_BIT == 14
            int32_t res_new = (int32_t)(single_turn << 18);
 #else
            int32_t res_new = (int32_t)(single_turn << 16);
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
            // Setup new position read request
            bissc_sensreq.slaveid       = R_BISSC_SLAVE_0;
            bissc_sensreq.timeout_clk   = BISSC_TIMEOUT_CLK;
            bissc_sensreq.sdresult_cb   = &callback_get_pos;
            R_BISS_Control(R_BISS0_ID, R_BISSC_CMD_POS, &bissc_sensreq);
        }
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

long bissc_enc_id(struct t_motor* pm)
{
    return 0;
}

long bissc_eeprom_read(struct t_motor* pm, long* read_data)
{
    return 0;
}

long bissc_eeprom_write(struct t_motor* pm, long write_data)
{
    return 0;
}


