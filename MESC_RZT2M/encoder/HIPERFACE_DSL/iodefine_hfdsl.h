#pragma once

struct st_hfdsl{
    union {
        unsigned short WORD;
        struct {
            unsigned short :3;
            unsigned short PRDY:1;
            unsigned short :3;
            unsigned short PRST:1;
            unsigned short ES:8;
        } BIT;
    } SYS_CTRL;
    union {
        unsigned short WORD;
        struct {
            unsigned short SMP_MSK:10;
            unsigned short :6;
        } BIT;
    } SMP_MSK;
    union {
        unsigned char BYTE;
        struct {
            unsigned char CHEN0:1;
            unsigned char CHEN1:1;
            unsigned char CHEN2:1;
            unsigned char CHEN3:1;
            unsigned char CHEN4:1;
            unsigned char CHEN5:1;
            unsigned char MSGEN:1;
            unsigned char POSEN:1;
        } BIT;
    } RAW_EN;
    union {
        unsigned char BYTE;
        struct {
            unsigned char RSSI_SUB0:4;
            unsigned char :4;
        } BIT;
    } RSSI_SUB0;
    union {
        unsigned char BYTE;
        struct {
            unsigned char RSSI_SUB1:8;
        } BIT;
    } RSSI_SUB1;
    union {
        unsigned char BYTE;
        struct {
            unsigned char RSSI_SUB2:8;
        } BIT;
    } RSSI_SUB2;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SYS_INIT:8;
        } BIT;
    } SYS_INIT;
    union {
        unsigned char BYTE;
        struct {
            unsigned char :2;
            unsigned char MINIT_END:1;
            unsigned char :1;
            unsigned char MMIN:1;
            unsigned char MPOS_RDY:1;
            unsigned char MMRCV:1;
            unsigned char MPOS_UPD:1;
        } BIT;
    } MASK;
    union {
        unsigned char BYTE;
        struct {
            unsigned char :2;
            unsigned char CINIT_END:1;
            unsigned char :1;
            unsigned char CMIN:1;
            unsigned char CPOS_RDY:1;
            unsigned char CMRCV:1;
            unsigned char CPOS_UPD:1;
        } BIT;
    } EVENTCLR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ACC_ERR:5;
            unsigned char :3;
        } BIT;
    } ACC_ERR;
    union {
        unsigned long LONG;
        struct {
            unsigned long MPRST:1;
            unsigned long MDTE:1;
            unsigned long MQMLW:1;
            unsigned long MPOS:1;
            unsigned long MRAW_FULL:1;
            unsigned long :1;
            unsigned long MSUM_STS:1;
            unsigned long MMRCV_ERR:1;
            unsigned long MVPOS:1;
            unsigned long MSCE:1;
            unsigned long :2;
            unsigned long MACCERR:1;
            unsigned long MACCTHRE:1;
            unsigned long :2;
            unsigned long MSUM0:1;
            unsigned long MSUM1:1;
            unsigned long MSUM2:1;
            unsigned long MSUM3:1;
            unsigned long MSUM4:1;
            unsigned long MSUM5:1;
            unsigned long MSUM6:1;
            unsigned long MSUM7:1;
            unsigned long :8;
        } BIT;
    } MASK_ERR;
    union {
        unsigned short WORD;
        struct {
            unsigned short MAXACC:16;
        } BIT;
    } MAXACC;
    union {
        unsigned short WORD;
        struct {
            unsigned short MAXDEV:16;
        } BIT;
    } MAXDEV;
    union {
        unsigned long LONG;
        struct {
            unsigned long CPRST:1;
            unsigned long CDTE:1;
            unsigned long CQMLW:1;
            unsigned long CPOS:1;
            unsigned long CRAW_FULL:1;
            unsigned long :1;
            unsigned long CSUM_STS:1;
            unsigned long CMRCV_ERR:1;
            unsigned long CVPOS:1;
            unsigned long CSCE:1;
            unsigned long :2;
            unsigned long CACCERR:1;
            unsigned long CACCTHRE:1;
            unsigned long :2;
            unsigned long CSUM0:1;
            unsigned long CSUM1:1;
            unsigned long CSUM2:1;
            unsigned long CSUM3:1;
            unsigned long CSUM4:1;
            unsigned long CSUM5:1;
            unsigned long CSUM6:1;
            unsigned long CSUM7:1;
            unsigned long :8;
        } BIT;
    } EVENTCLR_ERR;
    char  dummy_stuff[0x1E8];
    union {
        unsigned char BYTE;
        struct {
            unsigned char STUFF:8;
        } BIT;
    } STUFF;
    union {
        unsigned char BYTE;
        struct {
            unsigned char EXLEN:8;
        } BIT;
    } EXLEN;
    union {
        unsigned short WORD;
        struct {
            unsigned short EXTRA:16;
        } BIT;
    } EXTRA;    
    char  dummy_VPOSCRC[0x1FE];
    union {
        unsigned short WORD;
        struct {
            unsigned short VPOSCRC:16;
        } BIT;
    } VPOSCRC;
    uint8_t  dummy_vpos_h[18];
    union {
        unsigned char BYTE;
        struct {
            unsigned char VPOS_H:8;
        } BIT;
    } VPOS_H;
    union {
        unsigned char BYTE;
        struct {
            unsigned char POS_H:8;
        } BIT;
    } POS_H;
    char  dummy_ENCID[0x1E8];
    union {
        unsigned long LONG;
        struct {
            unsigned long ENC_ID:20;
            unsigned long :12;
        } BIT;    
    } ENC_ID;         
    union {
        unsigned char BYTE;
        struct {
            unsigned char ENC_ID2:8;
        } BIT;
    } ENC_ID2;    
    char  dummy_ACC_CNT[0x1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char ACC_CNT:5;
            unsigned char :3;
        } BIT;
    } ACC_CNT;   
    union {
        unsigned char BYTE;
        struct {
            unsigned char INIT:2;
            unsigned char INIT_END:1;
            unsigned char :1;
            unsigned char MIN:1;
            unsigned char POS_RDY:1;
            unsigned char MRCV:1;
            unsigned char POS_UPD:1;
        } BIT;
    } EVENT;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRST:1;
            unsigned long DTE:1;
            unsigned long QMLW:1;
            unsigned long POS:1;
            unsigned long RAW_FULL:1;
            unsigned long :1;
            unsigned long SUM_STS:1;
            unsigned long MRCV_ERR:1;
            unsigned long VPOS:1;
            unsigned long SCE:1;
            unsigned long :2;
            unsigned long ACCERR:1;
            unsigned long ACCTHRE:1;
            unsigned long :2;
            unsigned long SUM0:1;
            unsigned long SUM1:1;
            unsigned long SUM2:1;
            unsigned long SUM3:1;
            unsigned long SUM4:1;
            unsigned long SUM5:1;
            unsigned long SUM6:1;
            unsigned long SUM7:1;
            unsigned long :8;
        } BIT;
    } EVENT_ERR;
    union {
        unsigned short WORD;
        struct {
            unsigned short EDGES:10;
            unsigned short :6;
        } BIT;
    } EDGES;
    union {
        unsigned char BYTE;
        struct {
            unsigned char DELAY:4;
            unsigned char BiSS:4;
        } BIT;
    } DELAY;
    union {
        unsigned char BYTE;
        struct {
            unsigned char QM:4;
            unsigned char :3;
            unsigned char LINK:1;
        } BIT;
    } MASTER_QM;
    union {
        unsigned short WORD;
        struct {
            unsigned short MAXDEVR:16;
        } BIT;
    } MAXDEVR;
    char  dummy_VEL[0x2];
    union {
        unsigned long LONG;
        struct {
            unsigned long VEL:24;
            unsigned long :8;
        } BIT;
    } VEL;    
    /* union {
        unsigned short WORD;
        struct {
            unsigned short MINR:8;
            unsigned short MAJR:8;
        } BIT;
    } VERSION; */
    char dummy_pos[0x600];
    union {
        unsigned long LONG;
        struct {
          unsigned long POS:32;
        } BIT;
    } POS;
    union {
        unsigned long LONG;
        struct {
          unsigned long VPOS:32;
        } BIT;
    } VPOS;
    char dummy_fiforst[0xB54E0];
    union {
        unsigned long LONG;
        struct {
            unsigned long RST:1;
            unsigned long :31;
        } BIT;
    } FIFORST;
    char dummy_th_raw[0x10];
    union {
        unsigned long LONG;
        struct {
            unsigned long RECV_TH:4;
            unsigned long :28;
        } BIT;
    } TH_RAW;
    char dummy_version[0xE8];
    union {
        unsigned long LONG;
        struct {
            unsigned long VERSION:32;
        } BIT;
    } VERSION0;    
    union {
        unsigned long LONG;
        struct {
            unsigned long VERSION:32;
        } BIT;
    } VERSION1;
};

typedef struct {
    unsigned short WORD;
    struct {
        unsigned short FIFO_DATA:16;
        // unsigned short :16;
    } BIT;
} HFDSL_FIFO_DATA;

typedef struct  {
    unsigned long WORD;
    struct {
      unsigned long RAW_PAE:1;
      unsigned long RAW_EMPTY:1;
      unsigned long :6;
      unsigned long MSG_PAF:1;
      unsigned long MSG_FULL:1;
      unsigned long :6;
      unsigned long RAW_EXIST:5;
      unsigned long :3;
      unsigned long MSG_SPACE:5;
      unsigned long :3;
    } BIT;
} HFDSL_FIFO_STS;

#define HFDSL0           (*(volatile struct st_hfdsl *)(0xA011c100))
#define HFDSL1           (*(volatile struct st_hfdsl *)(0xA031c100))
#define HFDSL_DR(addr)   (*(volatile struct st_hfdsl *)(addr))

