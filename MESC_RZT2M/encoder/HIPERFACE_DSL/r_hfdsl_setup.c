#include "r_hfdsl_rzt2_if.h"

#include "r_hfdsl_rzt2_config.h"

#include "r_enc_int_rzt2.h"

#include <ecl/r_ecl_rzt2_if.h>

#include <bsp_api.h>
#include <hal_data.h>

#include <eType.h>
#include <tMotor.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define EFM50 0
#define EKM36 1
#define EEM37 2

#define ENC_MODEL EKM36

/* EC-lib macros */
#define R_HFDSL_FREQ (R_ECL_FREQ_33333KHZ)  //r_ecl_rzt2_if.h

/* Major macros */
#if (ENC_MODEL == EFM50)
#define ENC_ID                  (0x00000183)        /* Encoder ID of EFM50-0KF0A023A */
#define RES_BIT                 (0)                 /* LSB position of position information of POS register */
#define RES_MASK                (0x007FFFFF)        /* Mask of position information of POS register */
#define RES_MASK_H              (0x00000000)        /* Position information mask of POS_H register */
#define ROT_BIT                 (23)                /* LSB position of rotation number information of POS register */
#define ROT_MASK                (0x000001FF)        /* POS register rotational speed mask */
#define ROT_MASK_H              (0x00000E00)        /* POS_H register rotational speed mask */
#define LMSG_RECV_SIZE          (16)                /* Maximum receive data size of long message */
#elif (ENC_MODEL == EKM36)
 #define ENC_ID                  (0x00000153)        /* Encoder ID of EKM36-0KF0A020A */
 #define RES_BIT                 (0)                 /* LSB position of position information of POS register */
 #define RES_MASK                (0x000FFFFF)        /* Mask of position information of POS register */
 #define RES_MASK_H              (0x00000000)        /* Position information mask of POS_H register */
 #define ROT_BIT                 (20)                /* LSB position of rotation number information of POS register */
 #define ROT_MASK                (0x00000FFF)        /* POS register rotational speed mask */
 #define ROT_MASK_H              (0x000FF000)        /* POS_H register rotational speed mask */
 #define LMSG_RECV_SIZE          (16)                /* Maximum receive data size of long message */
#elif (ENC_MODEL == EEM37)
 #define ENC_ID                  (0x00000123)        /* Encoder ID of EEM37-0KF0A017A */
 #define RES_BIT                 (0)                 /* LSB position of position information of POS register */
 #define RES_MASK                (0x0001FFFF)        /* Mask of position information of POS register */
 #define RES_MASK_H              (0x00000000)        /* Position information mask of POS_H register */
 #define ROT_BIT                 (17)                /* LSB position of rotation number information of POS register */
 #define ROT_MASK                (0x00000FFF)        /* POS register rotational speed mask */
 #define ROT_MASK_H              (0x00000000)        /* POS_H register rotational speed mask */
 #define LMSG_RECV_SIZE          (16)                /* Maximum receive data size of long message */
#endif

/* Reset occurrence macros */
#define PRSET                   (0x00000001)

/* Get position macros */
#define POS_RDY_BIT             (0x20)

/* Long message macro */
#define SEND_SIZE               (6u)               /* Send size */
#define RECV_SIZE               (6u)               /* Receive size */
#define HEADER                  (0x5566)           /* Header data */
#define ADDR_H                  (0xD7F5)           /* Value before conversion 0xF4 */
#define ADDR_L                  (0xF9E7)           /* Value before conversion 0x80 */
#define OFFS_H                  (0xF9E7)           /* Value before conversion 0x80 */
#define OFFS_L                  (0xE7E7)           /* Value before conversion 0x00 */
#define CRC_H                   (0xE7E7)           /* Value before conversion 0x00 */
#define CRC_L                   (0x34D7)           /* Value before conversion 0xBF */

/* Display macro */
#define DEBUG_PRINT(...)
#define DEBUG_PRINT_END         "\n"                /* Line feed code */

/* Console command macro */
#define ARG_MAX                 (7)                 /* maximum number of comannd arguments */
#define CMD_BUF_SIZE            (256)               /* command buffer size */
#define CMD_NUM                 (4)                 /* number of commands */
#define CMD_DELIMITER           (" \t\r\n")         /* command line delimiter */
#define CMD_EXIT_ARG_NUM        (1)                 /* "exit" command paramter */
#define PSEL_VALUE              (0x2B)              /* Value of MPC.PxxPFS.PSEL */

/* Timer macro */
#define CMT1_CKS                (0u)                /* Set the clock input to CMT1 to PCLKD / 8 */
#define CMT1_CMCOR              (0xFFFFu)           /* Value of CMT1.CMCOR */
#define CMT1_TIME_PLACE         (10u)               /* Adjust the unit of time */
#define CMT1_TIME_RESOLUTION    (1u)                /* The smallest unit of count time by CMT1 */
#define CMT1_TIME_START         (1u)                /* Starts the timer */
#define CMT1_US_TO_CMCOR(us)    ((us) * 50 / 8)     /* us to CMT1.CMCOR (PCLKD/8) */
#define CMT1_CMCOR_MAX_TO_US    (0xFFFF * 8 / 75)   /* maximum time (us) to set CMCOR (PCLKD/8) */
#define CURRENT_LOOP            (50u)               /* Current loop period */

#define FIFO_DEP_MAX                    (16u)


extern void hfdsl_int_nml_isr_ch0(void);
extern void hfdsl_int_err_isr_ch0(void);
extern void hfdsl_int_nml_isr_ch1(void);
extern void hfdsl_int_err_isr_ch1(void);

typedef char char_t;
typedef void (*cmd_func_t)(void);

extern uint8_t g_hfdsl_config_dat[];
extern uint32_t g_pinmux_config[];

/* HFDSL sample program */
static void hfdsl_cmd_control(int32_t id);
static int32_t hfdsl_init(int32_t id);
long hfdsl_pos(struct t_motor *pm);
void hfdsl_vel(void);
void hfdsl_lmsg(struct t_motor *pm);
void hfdsl_shub(void);
static void hfdsl_int_nml_ch0_callback(uint8_t event);
static void hfdsl_int_nml_ch1_callback(uint8_t event);
static void hfdsl_int_err_callback(uint32_t event_err);
static void hfdsl_int_raw_callback(uint16_t* raw_data);
static void hfdsl_int_mrcv_callback(uint16_t* msg_data);
static void hfdsl_int_ch0_init_callback(void);
static void hfdsl_int_ch1_init_callback(void);

int32_t main_hfdsl(const int32_t idx, int32_t br);
int32_t hfdsl_close(const int32_t idx);
long hfdsl_enc_id(struct t_motor *pm);
long hfdsl_eeprom_read(struct t_motor *pm, long *read_data);
long hfdsl_eeprom_write(struct t_motor *pm, long write_data);

/* Timer function */
#if 0
static void timer_init(void);
static void timer_wait(uint32_t time);
static void timer_start(uint32_t us);
static void timer_stop(void);
#endif
/* Terminal program */

/* Send data of long message */
static uint16_t lmsg_send_data_tbl[SEND_SIZE + 1] =
{
    HEADER, ADDR_H, ADDR_L, OFFS_H, OFFS_L, CRC_H, CRC_L
};

/* Static variable */
static volatile bool mrcv_flg = false;
static uint32_t err_info;
static uint32_t pos_rot;
static uint32_t pos_res;
static uint32_t vpos_rot;
static uint32_t vpos_res;
static uint32_t vel;
static uint16_t lmsg_recv[LMSG_RECV_SIZE];
static uint16_t shub[FIFO_DEP_MAX];
static volatile bool init_end_flg = false;

#ifndef RAM_EXECUTION
#define A_ENCIF0_WBLOCK_ADDR	(uint8_t*)(0x00028000u)
#define A_ENCIF1_WBLOCK_ADDR	(uint8_t*)(0x00044000u)
#endif

int32_t main_hfdsl(const int32_t idx, int32_t br)
{
    const r_enc_isr_t int_isr_func_ch0=
    {
        hfdsl_int_nml_isr_ch0, hfdsl_int_err_isr_ch0
    };

    const r_enc_isr_t int_isr_func_ch1=
    {
        hfdsl_int_nml_isr_ch1, hfdsl_int_err_isr_ch1
    };

    DEBUG_PRINT("HFDSL sample program start" DEBUG_PRINT_END);

    /* Initialization of RZ/T2M peripheral */
//    timer_init();

    uint8_t id;
    unsigned char *p;
    if (idx == 0)
    {
        id = R_HFDSL0_ID;
        r_enc_int_isr_open(idx, &int_isr_func_ch0);
        r_pinmux_encif04_create(ETYPE_APE_HIPERFACE_DSL);
		    p = A_ENCIF0_WBLOCK_ADDR;
    }
    else if (idx == 1)
    {
        id = R_HFDSL1_ID;
        r_enc_int_isr_open(idx, &int_isr_func_ch1);
        r_pinmux_encif59_create(ETYPE_APE_HIPERFACE_DSL);
		    p= A_ENCIF1_WBLOCK_ADDR;
    }
    else
    {
        goto end;
    }
    copy_to_rodata_encif(p, g_hfdsl_config_dat, 0x18000u);

#if 0
    cur_id = id;
#endif
    
    int32_t ret_err_code = R_ECL_Initialize();
    if (R_ECL_SUCCESS != ret_err_code)
    {
      DEBUG_PRINT("R_ECL_Init: error(%ld)" DEBUG_PRINT_END, ret_err_code);
      goto end;
    }

    ret_err_code = R_ECL_ConfigurePin(g_pinmux_config);
    if (R_ECL_SUCCESS != ret_err_code)
    {
        DEBUG_PRINT("R_ECL_ConfigurePincfg: error(%ld)" DEBUG_PRINT_END, ret_err_code);
        hfdsl_close(idx);
        goto end;
    }

#ifndef RAM_EXECUTION
    // Configuration of Encoder I/F HFDSL
    ret_err_code = R_ECL_Configure(id, p);
#else
    /* Configuration of Encoder I/F HFDSL */
    ret_err_code = R_ECL_Configure(id, g_hfdsl_config_dat);
#endif
    if (R_ECL_SUCCESS != ret_err_code)
    {
        DEBUG_PRINT("R_ECL_Configure: error(%ld)" DEBUG_PRINT_END, ret_err_code);
        hfdsl_close(idx);
        goto end;
    }

    // Start of Encoder I/F HFDSL
    ret_err_code = R_ECL_Start(id, R_HFDSL_FREQ);
    if (R_ECL_SUCCESS != ret_err_code)
    {
        DEBUG_PRINT("R_ECL_Start: error(%ld)" DEBUG_PRINT_END, ret_err_code);
        hfdsl_close(idx);
        goto end;
    }

    // Enable IRQ interrupt
    __asm("cpsie i");
    __asm("isb");

    // Control of HFDSL
    hfdsl_cmd_control(id);
end:
    DEBUG_PRINT("HFDSL sample program end" DEBUG_PRINT_END);
    return ret_err_code;
}

int32_t hfdsl_close(const int32_t idx)
{ 
    uint8_t id;
    if (idx == 0)
    {
        id = R_HFDSL0_ID;
        r_pinmux_encif04_release();
        r_enc_ch0_elc_stop();
    }
    else
    {
        id = R_HFDSL1_ID;
        r_pinmux_encif59_release();
        r_enc_ch1_elc_stop();
    }

    int32_t ret_err_code = R_HFDSL_Close(id);
    if (R_HFDSL_SUCCESS != ret_err_code)
    {
        DEBUG_PRINT("R_HFDSL_Close: error(%d)" DEBUG_PRINT_END, ret_err_code);
    }
    
    // Stop of Encoder I/F HFDSL
    ret_err_code = R_ECL_Stop(id);
    if (R_ECL_SUCCESS != ret_err_code)
    {
        DEBUG_PRINT("R_ECL_Stop: error(%d)" DEBUG_PRINT_END, ret_err_code);
    }
    
//    timer_stop();
    return ret_err_code;
}

static void hfdsl_cmd_control(int32_t id)
{
    r_hfdsl_info_t hfdsl_info;
    if (R_HFDSL0_ID == id)
    {
        hfdsl_info.pcb_nml = &hfdsl_int_nml_ch0_callback;
        hfdsl_info.pcb_init = &hfdsl_int_ch0_init_callback;
    }
    else
    {
        hfdsl_info.pcb_nml = &hfdsl_int_nml_ch1_callback;
        hfdsl_info.pcb_init = &hfdsl_int_ch1_init_callback;
    }
    hfdsl_info.pcb_err  = &hfdsl_int_err_callback;
    hfdsl_info.pcb_raw  = &hfdsl_int_raw_callback;
//    hfdsl_info.pcb_init = &hfdsl_int_init_callback;
    int32_t ret_err_code = R_HFDSL_Open(id, &hfdsl_info);
    if (R_HFDSL_SUCCESS != ret_err_code)
    {
        DEBUG_PRINT("R_HFDSL_Open: error(%ld)" DEBUG_PRINT_END, ret_err_code);
        return;
    }

    // Protocol initialization
    ret_err_code = hfdsl_init(id);
    if (R_HFDSL_SUCCESS != ret_err_code)
    {
        DEBUG_PRINT("Protocol initialization failed" DEBUG_PRINT_END);
        ret_err_code = R_HFDSL_Close(id);
    }
}

static int32_t hfdsl_init(int32_t id)
{
    // Protocol initialization 1
    int32_t ret_err_code = R_HFDSL_Control(id, R_HFDSL_CMD_INIT1, NULL);
    if (R_HFDSL_SUCCESS != ret_err_code)
    {
        DEBUG_PRINT("R_HFDSL_Control(R_HFDSL_CMD_INIT1): error(%ld)" DEBUG_PRINT_END, ret_err_code);
        return ret_err_code;
    }

    // INIT bit = 1 ?
    while(1 != R_HFDSL_CheckInitSeq(id)) {}

    // Protocol initialization 2
    ret_err_code = R_HFDSL_Control(id, R_HFDSL_CMD_INIT2, NULL);
    if (R_HFDSL_SUCCESS != ret_err_code)
    {
        DEBUG_PRINT("R_HFDSL_Control(R_HFDSL_CMD_INIT2): error(%ld)" DEBUG_PRINT_END, ret_err_code);
        return ret_err_code;
    }

    // Protocol initialization 3
    ret_err_code = R_HFDSL_Control(id, R_HFDSL_CMD_INIT3, NULL);
    if (R_HFDSL_SUCCESS != ret_err_code)
    {
        DEBUG_PRINT("R_HFDSL_Control(R_HFDSL_CMD_INIT3): error(%ld)" DEBUG_PRINT_END, ret_err_code);
        return ret_err_code;
    }

    // INIT bit = 2 ?
    while(2 != R_HFDSL_CheckInitSeq(id)) {}

    // Wait 1 ms
    R_BSP_SoftwareDelay(1000, BSP_DELAY_UNITS_MICROSECONDS);
//    timer_wait(CMT1_US_TO_CMCOR(1000u));

    // Protocol initialization 4
    ret_err_code = R_HFDSL_Control(id, R_HFDSL_CMD_INIT4, NULL);
    if (R_HFDSL_SUCCESS != ret_err_code)
    {
        DEBUG_PRINT("R_HFDSL_Control(R_HFDSL_CMD_INIT4): error(%ld)" DEBUG_PRINT_END, ret_err_code);
        if (R_HFDSL_ERR_INIT == ret_err_code)
        {
            R_HFDSL_Control(id, R_HFDSL_CMD_RST, NULL);
        }
        return ret_err_code;
    }

    // Wait 24 us
    R_BSP_SoftwareDelay(24, BSP_DELAY_UNITS_MICROSECONDS);
//    timer_wait(CMT1_US_TO_CMCOR(24u));

    // Protocol initialization 5
    ret_err_code = R_HFDSL_Control(id, R_HFDSL_CMD_INIT5, NULL);
    if (R_HFDSL_SUCCESS != ret_err_code)
    {
        DEBUG_PRINT("R_HFDSL_Control(R_HFDSL_CMD_INIT5): error(%ld)" DEBUG_PRINT_END, ret_err_code);
        if (R_HFDSL_ERR_INIT == ret_err_code)
        {
            R_HFDSL_Control(id, R_HFDSL_CMD_RST, NULL);
        }
        return ret_err_code;
    }

    // INIT bit = 3 ?
    while(3 != R_HFDSL_CheckInitSeq(id)) {}

    // Protocol initialization 6
    uint32_t enc_id = ENC_ID;
    ret_err_code = R_HFDSL_Control(id, R_HFDSL_CMD_INIT6, &enc_id);
    if (R_HFDSL_SUCCESS != ret_err_code)
    {
        DEBUG_PRINT("R_HFDSL_Control(R_HFDSL_CMD_INIT6): error(%ld)" DEBUG_PRINT_END, ret_err_code);
        if (R_HFDSL_ERR_INIT == ret_err_code)
        {
            R_HFDSL_Control(id, R_HFDSL_CMD_RST, NULL);
        }
        return ret_err_code;
    }

    //  Waiting for completion of protocol initialization
    while(false == init_end_flg) {}
    return ret_err_code;
}       

long hfdsl_pos(struct t_motor *pm)
{
    uint8_t origin_flg = false;

    int32_t res_old = (pm->real_res & 0xc0000000);

    uint32_t single_turn = pos_res;

    // Convert result to 32-bit resolution per single turn
//    res_new = (int32_t)(single_turn << 12);
//    res_new = (int32_t)(single_turn << (32 - ROT_BIT));
    uint32_t single_turn_mask=((uint32_t)1u<<ROT_BIT)-1u;
    int32_t res_new = (int32_t)(((~single_turn)&single_turn_mask) << (32 - ROT_BIT));

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

void hfdsl_vel(void)
{
    DEBUG_PRINT("Motor rotation speed"DEBUG_PRINT_END);
    DEBUG_PRINT("   Speed       : 0x%08X"DEBUG_PRINT_END, vel);
    DEBUG_PRINT("Error information"DEBUG_PRINT_END);
    DEBUG_PRINT("   EVENT_ERR   : 0x%08X"DEBUG_PRINT_END, err_info);
}

void hfdsl_lmsg(struct t_motor *pm)
{
    // Send long message
    r_hfdsl_send_msg_t msg;
    msg.pdata = &lmsg_send_data_tbl[0];
    msg.pcb_msg = &hfdsl_int_mrcv_callback;

    extern struct t_motor m1;
    extern struct t_motor m2;
    int ch = pm == &m1 ? R_HFDSL0_ID : R_HFDSL1_ID;
    int32_t ret_err_code = R_HFDSL_Control(ch, R_HFDSL_CMD_MSG, &msg);
    if (R_HFDSL_SUCCESS != ret_err_code)
    {
        DEBUG_PRINT("R_HFDSL_Control(R_HFDSL_CMD_MSG): error(%ld)" DEBUG_PRINT_END, ret_err_code);
        return;
    }

    // MRCV bit = 1 ?
    while(false == mrcv_flg) {}

    mrcv_flg = false;

    // Display of received data
    DEBUG_PRINT("Received data"DEBUG_PRINT_END);
    for (uint8_t loop = 0; loop < RECV_SIZE; loop++)
    {
        DEBUG_PRINT("   FIFO_MSGR[%02d] : 0x%04X"DEBUG_PRINT_END,loop, lmsg_recv[loop]);
    }

    DEBUG_PRINT("Error information"DEBUG_PRINT_END);
    DEBUG_PRINT("   EVENT_ERR   : 0x%08lX"DEBUG_PRINT_END, err_info);
}

void hfdsl_shub(void)
{
    DEBUG_PRINT("RAW data"DEBUG_PRINT_END);
    DEBUG_PRINT("   vertical    : 0x%04X"DEBUG_PRINT_END, shub[0]);
    DEBUG_PRINT("   s_par       : 0x%04X"DEBUG_PRINT_END, shub[1]);
    DEBUG_PRINT("   pipeline    : 0x%04X"DEBUG_PRINT_END, shub[2]);
    DEBUG_PRINT("   acc         : 0x%04X"DEBUG_PRINT_END, shub[3]);
    DEBUG_PRINT("   acc_crc     : 0x%04X"DEBUG_PRINT_END, shub[4]);
    DEBUG_PRINT("   secondary   : 0x%04X"DEBUG_PRINT_END, shub[5]);
    DEBUG_PRINT("   pos_h       : 0x%04X"DEBUG_PRINT_END, shub[6]);
    DEBUG_PRINT("   pos[31:16]  : 0x%04X"DEBUG_PRINT_END, shub[7]);
    DEBUG_PRINT("   pos[15:00]  : 0x%04X"DEBUG_PRINT_END, shub[8]);
    DEBUG_PRINT("   event_err   : 0x%04X"DEBUG_PRINT_END, shub[9]);
    DEBUG_PRINT("Error information"DEBUG_PRINT_END);
    DEBUG_PRINT("   EVENT_ERR   : 0x%08lX"DEBUG_PRINT_END, err_info);
}

static void hfdsl_int_nml_ch0_callback(uint8_t event)
{
    r_hfdsl_pos_t fast;
    r_hfdsl_vpos_t safe;

    if (POS_RDY_BIT == (event & POS_RDY_BIT))
    {
        // Get fast position
        fast.all = true;
        (void)R_HFDSL_Control(R_HFDSL0_ID, R_HFDSL_CMD_POS, &fast);
        pos_res = (uint32_t)((((uint32_t)((uint64_t)fast.posh << (32 - RES_BIT))) & RES_MASK_H) |
                             ((fast.pos >> RES_BIT) & RES_MASK));
        pos_rot = ((((uint32_t)fast.posh << (32 - ROT_BIT)) & ROT_MASK_H) | ((fast.pos >> ROT_BIT) & ROT_MASK));

        // Get safe position
        (void)R_HFDSL_Control(R_HFDSL0_ID, R_HFDSL_CMD_VPOS, &safe);

        vpos_res = (uint32_t)((((uint64_t)safe.vposh << (32 - RES_BIT)) & RES_MASK_H) |
                               ((safe.vpos >> RES_BIT) & RES_MASK));
        vpos_rot = ((((uint32_t)safe.vposh << (32 - ROT_BIT)) & ROT_MASK_H) | ((safe.vpos >> ROT_BIT) & ROT_MASK));

        // Get motor rotation speed
        (void)R_HFDSL_Control(R_HFDSL0_ID, R_HFDSL_CMD_VEL, &vel);
    }
}

static void hfdsl_int_nml_ch1_callback(uint8_t event)
{
    r_hfdsl_pos_t fast;
    r_hfdsl_vpos_t safe;

    if (POS_RDY_BIT == (event & POS_RDY_BIT))
    {
        // Get fast position
        fast.all = true;
        (void)R_HFDSL_Control(R_HFDSL1_ID, R_HFDSL_CMD_POS, &fast);
        pos_res = (uint32_t)((((uint32_t)((uint64_t)fast.posh << (32 - RES_BIT))) & RES_MASK_H) |
                             ((fast.pos >> RES_BIT) & RES_MASK));
        pos_rot = ((((uint32_t)fast.posh << (32 - ROT_BIT)) & ROT_MASK_H) | ((fast.pos >> ROT_BIT) & ROT_MASK));

        // Get safe position
        (void)R_HFDSL_Control(R_HFDSL1_ID, R_HFDSL_CMD_VPOS, &safe);

        vpos_res = (uint32_t)((((uint64_t)safe.vposh << (32 - RES_BIT)) & RES_MASK_H) |
                               ((safe.vpos >> RES_BIT) & RES_MASK));
        vpos_rot = ((((uint32_t)safe.vposh << (32 - ROT_BIT)) & ROT_MASK_H) | ((safe.vpos >> ROT_BIT) & ROT_MASK));

        // Get motor rotation speed
        (void)R_HFDSL_Control(R_HFDSL1_ID, R_HFDSL_CMD_VEL, &vel);
    }
}

static void hfdsl_int_err_callback(uint32_t event_err)
{
    err_info = event_err;
}

static void hfdsl_int_raw_callback(uint16_t* raw_data)
{
    memcpy(shub, raw_data, FIFO_DEP_MAX*2u);
}

static void hfdsl_int_mrcv_callback(uint16_t* msg_data)
{
    uint8_t loop;
    for (loop = 0; loop < RECV_SIZE; loop++)
    {
        lmsg_recv[loop] = msg_data[loop];
    }
    mrcv_flg = true;
}

static void hfdsl_int_ch0_init_callback(void)
{
    /* Timer start */
//    timer_start(CURRENT_LOOP);
    r_enc_ch0_elc_start();
    init_end_flg = true;
}

static void hfdsl_int_ch1_init_callback(void)
{
    /* Timer start */
//    timer_start(CURRENT_LOOP);
//    R_ELC->ELC_SSEL_b[13].ELC_SEL2 = 0x1D0;   /* ENCIF0 CAP_TRG0:MTU4.TCNT underflow */
    r_enc_ch1_elc_start();
    init_end_flg = true;
}

#if 0
/*******************************************************************************
* Function Name: timer_init
* Description  : Initialization of the CMT1 of CMT unit 0.
* Arguments    : None.
* Return Value : None.
*******************************************************************************/
static void timer_init(void)
{
    R_BSP_RegisterProtectDisable(BSP_REG_PROTECT_LPC_RESET);
//    R_BSP_MODULE_START(FSP_IP_CMTW, 1);
    volatile unsigned long dummy; // Declared volatile to prevent optimization being applied
    
    R_SYSC_NS->MSTPCRD_b.MSTPCRD06 = 0; // Setting to release CMTW unit 1 from the module-stop state
    dummy = R_SYSC_NS->MSTPCRD; // Step 1: Dummy-read the MSTPCRm register.
    dummy = R_CMTW1->CMWCOR; // Step 2: Dummy-read any register of CMTW unit 1 at least 5 times.
    dummy = R_CMTW1->CMWCOR;
    dummy = R_CMTW1->CMWCOR;
    dummy = R_CMTW1->CMWCOR;
    dummy = R_CMTW1->CMWCOR;

    R_BSP_PinAccessEnable();
    
    R_CMTW1->CMWSTR_b.STR=0;        //counter STOP
    
    R_CMTW1->CMWCR_b.CKS=0;         //PCLKL/8(50MHz/8=160ns)
//    R_CMTW1->CMWCR_b.CKS=3;         //PCLKL/512(50MHz/512=10240ns)
    R_CMTW1->CMWCR_b.CMS=0;         //32bit
    R_CMTW1->CMWCR_b.CMWIE = 0;       /* interrupt disable */
    R_CMTW1->CMWCR_b.CCLR=1;         //Counter Clear:The CMWCNT counter is not cleared.
    R_CMTW1->CMWIOR=0u;             //For Freerun
    R_CMTW1->CMWCOR = CMT1_CMCOR;
    
    R_CMTW1->CMWCNT=0u;

    return;
}
/*******************************************************************************
End of function timer_init
*******************************************************************************/
/*******************************************************************************
* Function Name: timer_wait
* Description  : Generation of wait time.
* Arguments    : time - wait time.
* Return Value : None.
*******************************************************************************/
static void timer_wait(uint32_t time)
{
    uint16_t count;

    count = (uint16_t)time;

    R_CMTW1->CMWCNT = 0u;
    R_CMTW1->CMWCR_b.CMWIE = 0;       /* interrupt disable */
    R_CMTW1->CMWSTR_b.STR=1;        //counter START

    while (R_CMTW1->CMWCNT <= count)
    {
        ; /* DO NOTHING */
    }

    R_CMTW1->CMWSTR_b.STR=0;        //counter STOP

    return;
}
/*******************************************************************************
End of function timer_wait
*******************************************************************************/

/*******************************************************************************
* Function Name: timer_start
* Description  : Start timer.
* Arguments    : us - wait time (us).
* Return Value : None.
*******************************************************************************/
static void timer_start(uint32_t us)
{
    R_CMTW1->CMWCR_b.CKS=0;         //PCLKL/8(50MHz/8=160ns)
    R_CMTW1->CMWCR_b.CMWIE = 1;       /* interrupt enable */
    R_CMTW1->CMWCOR = CMT1_US_TO_CMCOR(us);
    R_CMTW1->CMWCNT = CMT1_US_TO_CMCOR(us) - 1;
    R_CMTW1->CMWSTR_b.STR = 1;

    return;
}
/*******************************************************************************
End of function timer_start
*******************************************************************************/

/*******************************************************************************
* Function Name: timer_stop
* Description  : Stop timer.
* Arguments    : None.
* Return Value : None.
*******************************************************************************/
static void timer_stop(void)
{
    R_CMTW1->CMWSTR_b.STR = 0;
    R_CMTW1->CMWCR_b.CKS=0;         //PCLKL/8(50MHz/8=160ns)
    R_CMTW1->CMWCR_b.CMWIE = 0;       /* interrupt disable */
    R_CMTW1->CMWCOR = 0;
    R_CMTW1->CMWCNT=0u;

    return;
}
/*******************************************************************************
End of function timer_start
*******************************************************************************/
#endif

long hfdsl_enc_id(struct t_motor *pm)
{
    return ENC_ID;
}

long hfdsl_eeprom_read(struct t_motor *pm, long *read_data)
{
    return 0;
}

long hfdsl_eeprom_write(struct t_motor *pm, long write_data)
{
    return 0;
}

