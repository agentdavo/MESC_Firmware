#include "r_endat_rzt2_if.h"

#include "iodefine_endat.h"

#include "r_enc_int_rzt2.h"
#include "bsp_api.h"
#include "hal_data.h"

#include <ecl/r_ecl_rzt2_if.h>

#include <eType.h>
#include <stdio.h>
#include <stdlib.h>
#include <tMotor.h>

#define R_ENDAT0_ID     (R_ECL_CH_0)
#define R_ENDAT1_ID     (R_ECL_CH_1)
#define R_ENDAT_FREQ    (16670UL) // 16.67MHz

#define ENDAT_FMT25      0
#define ENDAT_FMT23      1
#define ENDAT_FMT13      0

//#define DEBUG_PRINT(...)            printf(__VA_ARGS__)
#define DEBUG_PRINT(...)
#define DEBUG_PRINT_END             "\n"        /* Line feed code */
#define PARAMETER_NOT_USED(p)       (void) ((p))

#define ARG_MAX                     (4)         /* maximum number of command arguments */
#define CMD_BUF_SIZE                (256)       /* command buffer size */
#define CMD_NUM                     (6)         /* number of commands */
#define EXIT_CMD_NUM                (5)         /* exit command number */
#define CMD_DELIMITER               (" \t\r\n") /* command line delimiter */
#define CMD_EXIT_ARG_NUM            (1)         /* "exit" command parameter */
#define PSEL_VALUE                  (0x2B)      /* Value of MPC.PxxPFS.PSEL */

#define ENDAT_ENC_TSAT_WAIT         (1300u)     /* Waiting time after the power on reset */
#define ENDAT_ENC_100US_WAIT        (100u)      /* Wait 100us */
#define ENDAT_ENC_INIT_RESET_WAIT   (60u)       /* Wait 51ms */
#define ENDAT_ENC_INIT_MEM_WAIT     (743u)      /* Wait 743us */
#define ENDAT_ENC_INIT_PRAM_WAIT    (13u)       /* Wait 13ms */
#define ENDAT_ENC_INIT_CABLE_WAIT   (588u)      /* Wait 588us */
#define ENDAT_WDG_MAX               (127u)      /* Watchdog timer settings 25.4ms */
#define ENDAT_POS_NUM               (255u)      /* Store the number of position value */
#define ENDAT_ARG1                  (0)         /* First argument */
#define ENDAT_ARG2                  (1)         /* Second argument */
#define ENDAT_DEC                   (10)        /* Decimal number */
#define ENDAT_HEX                   (16)        /* Hexadecimal */
#define ENDAT_TEMP_SCA_FAC          (0.1)       /* Scaling function for temperature */
#define ENDAT_TEMP_ABS_ZERO         (273.2)     /* Absolute zero degrees Celsius */
#define ENDAT_SHIFT_32              (32)
#define ENDAT_MASK_LOW_32           (0xFFFFFFFFu)

#define CMT1_CMCOR_MAX_TO_US        (0xFFFF * 8 / 75) /* maximum time (us) to set CMCOR (PCLKD/8) */

typedef char char_t;
typedef void (*cmd_func_t)(uint32_t arg_num, char_t *parg[]);

extern uint8_t g_endat_config_dat[];
extern uint32_t g_pinmux_config[];

static void endat_power_on_wait(void);
static void enc_init_reset_wait_callback(void);
static void enc_init_mem_wait_callback(void);
static void enc_init_pram_wait_callback(void);
static void enc_init_cable_wait_callback(void);
static void endat_callback(r_endat_result_t * presult, r_endat_protocol_err_t *perr); 
static void endat_rdst_callback(void); 

long endat_pos(struct t_motor *pm);
float endat_temp(struct t_motor *pm);

int32_t endat_close(const int32_t idx);
void timer_wait(uint32_t time);

volatile static bool endat_flg_done;
static r_endat_result_t *pendat_result;
static r_endat_protocol_err_t *pendat_err;

//extern t_motor m1,m2;

extern void enc_endat_ch0_int_isr(void);
extern void enc_endat_ch1_int_isr(void);

uint8_t endat_encode_br(int32_t br)
{
uint8_t result;

    switch (br)
    {
    case 16670:
        result = R_ENDAT_FTCLK_16670;
        break;
    case 8330:
        result = R_ENDAT_FTCLK_8330;
        break;
    case 4160:
        result = R_ENDAT_FTCLK_4160;
        break;
    case 4000:
        result = R_ENDAT_FTCLK_4000;
        break;
    case 2000:
        result = R_ENDAT_FTCLK_2000;
        break;
    case 1000:
        result = R_ENDAT_FTCLK_1000;
        break;
    case 200:
        result = R_ENDAT_FTCLK_200;
        break;
    case 100:
        result = R_ENDAT_FTCLK_100;
        break;
    default:
        result = R_ENDAT_FTCLK_8330;
        break;
    }
    return result;
}

#ifndef RAM_EXECUTION
#define A_ENCIF0_WBLOCK_ADDR	(uint8_t*)(0x00028000u)
#define A_ENCIF1_WBLOCK_ADDR	(uint8_t*)(0x00044000u)
#endif

int32_t main_endat(const int32_t idx, int32_t br)
{
    const r_enc_isr_t int_isr_func_ch0 = { enc_endat_ch0_int_isr, NULL };
    const r_enc_isr_t int_isr_func_ch1 = { enc_endat_ch1_int_isr, NULL };
    
    DEBUG_PRINT("EnDat sample program start" DEBUG_PRINT_END);

    uint8_t id;
    unsigned char *p;
    if (idx == 0)
    {
        id = R_ENDAT0_ID;
        r_enc_int_isr_open(idx, &int_isr_func_ch0);
        r_pinmux_encif04_create(ETYPE_APE_ENDAT);
        r_enc_ch0_elc_start();
		    p = A_ENCIF0_WBLOCK_ADDR;
    }
    else
    {
        id = R_ENDAT1_ID;
        r_enc_int_isr_open(idx, &int_isr_func_ch1);
        r_pinmux_encif59_create(ETYPE_APE_ENDAT);
        r_enc_ch1_elc_start();
		    p = A_ENCIF1_WBLOCK_ADDR;
    }
    copy_to_rodata_encif(p, g_endat_config_dat, 0x1C000u);

#if 0
    cur_id = id;
#endif
    
    // Waiting time after power-on of the encoder
    endat_power_on_wait();

    int32_t err_ecl_code = R_ECL_Initialize();
    if (R_ECL_SUCCESS != err_ecl_code)
    {
        DEBUG_PRINT("R_ECL_Init: error(%ld)" DEBUG_PRINT_END, err_ecl_code);
        endat_flg_done = true;
        return err_ecl_code;
    }

    err_ecl_code = R_ECL_ConfigurePin(g_pinmux_config);
    if (R_ECL_SUCCESS != err_ecl_code)
    {
        DEBUG_PRINT("R_ECL_ConfigurePincfg: error(%ld)" DEBUG_PRINT_END, err_ecl_code);
        endat_flg_done = true;
        return err_ecl_code;
    }

#ifndef RAM_EXECUTION
    err_ecl_code = R_ECL_Configure(id, p);
//    err_ecl_code = R_ECL_Configure(id, __section_begin("A_AS_BIN_WBLOCK"));
#else
    // Configuration of Encoder I/F EnDat
    err_ecl_code = R_ECL_Configure(id, g_endat_config_dat);
#endif    
    if (R_ECL_SUCCESS != err_ecl_code)
    {
        DEBUG_PRINT("R_ECL_Configure: error(%ld)" DEBUG_PRINT_END, err_ecl_code);
        endat_flg_done = true;
        return err_ecl_code;
    }

    // Start of Encoder I/F EnDat
    err_ecl_code = R_ECL_Start(id, R_ENDAT_FREQ);
    if (R_ECL_SUCCESS != err_ecl_code)
    {
        DEBUG_PRINT("R_ECL_Start: error(%ld)" DEBUG_PRINT_END, err_ecl_code);
        endat_flg_done = true;
        return err_ecl_code;
    }
    R_BSP_SoftwareDelay(ENDAT_ENC_100US_WAIT,BSP_DELAY_UNITS_MICROSECONDS);

#if 1
    // Enable IRQ interrupt
    __asm("cpsie i");
    __asm("isb");
#endif
    r_endat_info_t endat_info;
    
    endat_info.ftclk                = endat_encode_br(br);
    endat_info.filter               = true;
    endat_info.delay_comp           = true;
    endat_info.tst                  = R_ENDAT_TST_10US;
    endat_info.penc_init_reset_wait = &enc_init_reset_wait_callback;
    endat_info.penc_init_mem_wait   = &enc_init_mem_wait_callback;
    endat_info.penc_init_pram_wait  = &enc_init_pram_wait_callback;
    endat_info.penc_init_cable_wait = &enc_init_cable_wait_callback;
    r_endat_err_t ret_code = R_ENDAT_Open(id, &endat_info);
    if (ENDAT_SUCCESS != ret_code)
    {
        DEBUG_PRINT("R_ENDAT_Open: error(%d)" DEBUG_PRINT_END, (int)err_endat_code); 
    }
    endat_flg_done = true;
    return ret_code;
}

int32_t endat_close(const int32_t idx)
{ 
    uint8_t id;
    if (idx == 0)
    {
        id = R_ENDAT0_ID;
        r_pinmux_encif04_release();
        r_enc_ch0_elc_stop();
    }
    else
    {
        id = R_ENDAT1_ID;
        r_pinmux_encif59_release();
        r_enc_ch1_elc_stop();
    }
    
    r_endat_err_t err_endat_code = R_ENDAT_Close(id);
    if (R_ECL_SUCCESS != err_endat_code)
    {
        DEBUG_PRINT("R_ENDAT_Close: error(%d)" DEBUG_PRINT_END, (int)err_endat_code);
    }
    
    int32_t err_ecl_code = R_ECL_Stop(id);
    if (R_ECL_SUCCESS != err_ecl_code)
    {
        DEBUG_PRINT("R_ECL_Start: error(%d)" DEBUG_PRINT_END, (int)err_ecl_code);
    }
    
    return err_endat_code;
}

static void endat_power_on_wait(void)
{
    R_BSP_SoftwareDelay(ENDAT_ENC_TSAT_WAIT,BSP_DELAY_UNITS_MILLISECONDS); // 1.3s = 1ms * 1300
}

static void enc_init_reset_wait_callback(void)
{
    R_BSP_SoftwareDelay(ENDAT_ENC_INIT_RESET_WAIT,BSP_DELAY_UNITS_MILLISECONDS); // 60ms = 1ms * 60
}

static void enc_init_mem_wait_callback(void)
{
    R_BSP_SoftwareDelay(ENDAT_ENC_INIT_MEM_WAIT,BSP_DELAY_UNITS_MICROSECONDS); // 743us
}

static void enc_init_pram_wait_callback(void)
{
    R_BSP_SoftwareDelay(ENDAT_ENC_INIT_PRAM_WAIT, BSP_DELAY_UNITS_MILLISECONDS); // 13ms
}

static void enc_init_cable_wait_callback(void)
{
    R_BSP_SoftwareDelay(ENDAT_ENC_INIT_CABLE_WAIT,BSP_DELAY_UNITS_MICROSECONDS); // 588us
}

float endat_temp(struct t_motor *pm)
{
    DEBUG_PRINT("temp command" DEBUG_PRINT_END);

    r_endat_req_t endat_req;
    endat_req.mode_cmd          = R_ENDAT_POS_MEM;
    endat_req.dt                = false;
    endat_req.mrs               = R_ENDAT_MRS_TEMP2;
    endat_req.watchdog.range    = R_ENDAT_WD_RANGE_US;
    endat_req.watchdog.time     = ENDAT_WDG_MAX;
    endat_req.elc               = false;
    endat_req.pisr_result       = &endat_callback;
    endat_req.pisr_rdst         = &endat_rdst_callback;

    endat_flg_done = false;
    extern struct t_motor m1;
    extern struct t_motor m2;
    int id = pm == &m1 ? R_ENDAT0_ID : R_ENDAT1_ID;
    r_endat_err_t err_code = R_ENDAT_Control(id, ENDAT_CMD_REQ, &endat_req);
    if (ENDAT_SUCCESS == err_code)
    {
        while (false == endat_flg_done) {}

        endat_req.mode_cmd       = R_ENDAT_POS_ADD_DATA;
        endat_req.dt             = false;
        endat_req.watchdog.range = R_ENDAT_WD_RANGE_US;
        endat_req.watchdog.time  = ENDAT_WDG_MAX;
        endat_req.elc            = false;
        endat_req.pisr_result    = &endat_callback;
        endat_req.pisr_rdst      = &endat_rdst_callback;

        endat_flg_done = false;
        err_code = R_ENDAT_Control(id, ENDAT_CMD_REQ, &endat_req);
        if (ENDAT_SUCCESS == err_code)
        {
            while (false == endat_flg_done) {}
            // result_display(pendat_result, pendat_err);
            float temp_celsius = (pendat_result->data.add_datum1 * ENDAT_TEMP_SCA_FAC) - ENDAT_TEMP_ABS_ZERO;
            // DEBUG_PRINT(" temperature : %d [Celsius]" DEBUG_PRINT_END, (int)temp_celsius);
            return temp_celsius;
        }
    }
    DEBUG_PRINT("  R_ENDAT_Control(ENDAT_CMD_REQ) error: %d" DEBUG_PRINT_END, (int16_t)err_code);
    return 0.0f;
}

static void endat_callback(r_endat_result_t * presult,
                           r_endat_protocol_err_t *perr)
{
    pendat_result = presult;
    pendat_err = perr;

    if (false != pendat_err->modeerr)
    {
        endat_flg_done = true;
    }
}

static void endat_rdst_callback(void)
{
    endat_flg_done = true;
}

long endat_pos(struct t_motor* pm)
{
    uint8_t origin_flg = false;
    if (endat_flg_done == true)
    {
        r_endat_req_t endat_req;
        
        endat_flg_done = false;
        if (false == pendat_err->modeerr)
        {
            int32_t res_old = (pm->real_res & 0xc0000000);

            uint32_t single_turn = pendat_result->data.pos & ENDAT_MASK_LOW_32;

            // Convert result to 32-bit resolution per single turn
#if ENDAT_FMT25
            int32_t res_new = (int32_t)(single_turn << 7);
#endif
#if ENDAT_FMT23
            // 2's complement because CW -> CCW
            int32_t res_new = (int32_t)( ((~single_turn)&0x7FFFFFu) << 9);
#endif
#if ENDAT_FMT13
            int32_t res_new = (int32_t)(single_turn << 19);
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
                pm->captured_pos.Reg16.Low = 0;
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
            endat_req.mode_cmd          = R_ENDAT_POS;
            endat_req.dt                = false;
            endat_req.watchdog.range    = R_ENDAT_WD_RANGE_US;
            endat_req.watchdog.time     = ENDAT_WDG_MAX;
            endat_req.elc               = false;
            endat_req.pisr_result       = &endat_callback;
            endat_req.pisr_rdst         = &endat_rdst_callback;
            extern struct t_motor m1;
            extern struct t_motor m2;
            int id = pm == &m1 ? R_ENDAT0_ID : R_ENDAT1_ID;
            R_ENDAT_Control(id, ENDAT_CMD_REQ, &endat_req);
        }
    }
    if (origin_flg == true)
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

long endat_enc_id(struct t_motor* pm)
{
    return 0;
}

long endat_eeprom_read(struct t_motor* pm, long* read_data)
{
    return 0;
}

long endat_eeprom_write(struct t_motor* pm, long write_data)
{
    return 0;
}

