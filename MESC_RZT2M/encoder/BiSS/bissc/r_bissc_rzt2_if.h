#pragma once

#include <BiSS/r_biss_rzt2_if.h>

#include <stdint.h>
#include <stdbool.h>

/* Slave ID(0-7) */
#define R_BISSC_SLAVE_0         (0)
#define R_BISSC_SLAVE_1         (1)
#define R_BISSC_SLAVE_2         (2)
#define R_BISSC_SLAVE_3         (3)
#define R_BISSC_SLAVE_4         (4)
#define R_BISSC_SLAVE_5         (5)
#define R_BISSC_SLAVE_6         (6)
#define R_BISSC_SLAVE_7         (7)

/* Command of R_BISS_Control function.*/
#define R_BISSC_CMD_POS         (0xC0000001)      /* Get Position */
#define R_BISSC_CMD_REG_READ    (0xC0000002)      /* Read Register(1byte) */
#define R_BISSC_CMD_REG_WRITE   (0xC0000003)      /* Write Register(1byte) */
#define R_BISSC_CMD_REG_SREAD   (0xC0000004)      /* Sequential Register Access(read) */
#define R_BISSC_CMD_REG_SWRITE  (0xC0000005)      /* Sequential Register Access(write) */
#define R_BISSC_CMD_REG_SSTOP   (0xC0000006)      /* Stop sequential Register Access */
#define R_BISSC_CMD_REG_FSTOP   (0xC0000007)      /* Forced stop of Register Access */
#define R_BISSC_CMD_MAX         (0xC0000007)      /* Command num max */

/* receive results */
#define R_BISSC_SUCCESS         (0)             /* success */
#define R_BISSC_REQ_ERR         (-1)            /* error */
#define R_BISSC_REQ_ERR_WR      (-2)            /* error(register write error) */

/* set value of BR register */
#define R_BISSC_BR_10MHZ        (0x13)
#define R_BISSC_BR_8_33MHZ      (0x17)
#define R_BISSC_BR_4MHZ         (0x31)
#define R_BISSC_BR_2_5MHZ       (0x4F)
#define R_BISSC_BR_1MHZ         (0xC7)
#define R_BISSC_BR_400KHZ       (0x17C)
#define R_BISSC_BR_299_40KHZ    (0x1A6)
#define R_BISSC_BR_200KHZ       (0x1F9)
#define R_BISSC_BR_100KHZ       (0x27C)
#define R_BISSC_BR_80_12KHZ     (0x29B)
#define R_BISSC_BR_NUM          (10)

#define R_BISSC_ELC_DISABLE     (0u)
#define R_BISSC_ELC_ENABLE      (1u)

/* request information for sensor mode */
typedef int8_t r_bissc_req_err_t;

typedef struct bissc_sensreq_s
{
    uint8_t             slaveid;             /* SlaveID(0-7) */
    uint32_t            timeout_clk;         /* number of time-out clock(1clk=0.1us) */
    uint8_t             elc_enable;          /* elctimer enable flag */
    r_biss_isr_cb_t     sdresult_cb;         /* callback pointer for sensor data */
} r_bissc_sensreq_t;

/* request information for register access */
typedef struct bissc_regreq_s
{
    uint8_t             slaveid;            /* SlaveID(0-7) */
    uint32_t            timeout_clk;        /* number of time-out clock(1clk=0.1us) */
    uint8_t             regdtnum;           /* number of read/write register(1-64byte) */
    uint8_t             regaddress;         /* register address(0-127) */
    uint8_t             *regdata;           /* pointer of read/write data */
    r_biss_isr_cb_t     sdresult_cb;        /* callback pointer for sensor data */
    r_biss_isr_cb_t     rdresult_cb;        /* callback pointer for register access */
} r_bissc_regreq_t;

/* receive results of sensor mode */
typedef struct bissc_sensordt_s
{
    r_bissc_req_err_t   result;             /* receive results */
    uint32_t            stdata;             /* single-turn data */
    uint32_t            mtdata;             /* multi-turn data */
    bool                timeout;            /* time-out error */
    bool                crc1_err;           /* CRC1 error */
    bool                alarm;              /* Alarm error */
    bool                warning;            /* Warning error */
    bool                notready;           /* register access status(CDS NotReady) */
} r_bissc_sensordt_t;

/* Send and receive results of register access */
typedef struct bissc_regdata_s
{
    r_bissc_req_err_t   result;             /* receive results */
    uint8_t             idl;                /* Encoder connection information */
    uint8_t             combyte;            /* number of completed register access */
    bool                timeout;            /* time-out error */
    bool                crc1_err;           /* CRC1 error */
    bool                crc2_err;           /* CRC2 error */
    bool                alarm;              /* Alarm error */
    bool                warning;            /* Warning error */
    bool                regw_err;           /* register write error */
    bool                readbit_err;        /* register access error(read bit error) */
    bool                writebit_err;       /* register access error(write bit error) */
    bool                stopbit_err;        /* register access error(stop bit error) */
    bool                cds_err;            /* register access error(CDS error after IDL) */
} r_bissc_regdata_t;

r_biss_err_t R_BISSC_Open(const int32_t biss_no, r_biss_info_t* pinfo);
r_biss_err_t R_BISSC_Close(const int32_t biss_no);
uint32_t R_BISSC_GetVersion(void);
r_biss_err_t R_BISSC_Control(const int32_t biss_no, const r_biss_cmd_t cmd, void *const pbuf);