#pragma once

struct st_biss{                          /* struct BiSS IF   */
    char           wk00[0x12];
    union {                                 /* BR1(0xA0034012)                      */
        unsigned short WORD;                /* Word Access      */
        struct {                            /* Bit Access       */
            unsigned short BR1:10;
            unsigned short :6;
        } BIT;
    } BR1;
    char           wk0[0x84E];
    union {                                 /* CPOLY(0xA0034862)                    */
        unsigned short WORD;                /* Word Access      */
        struct {                            /* Bit Access       */
            unsigned short CPOLY:16;
        } BIT;
    } CPOLY;
    char           wk1[0xE];
    union {                                 /* CINIT(0xA0034872)                    */
        unsigned short WORD;                /* Word Access      */
        struct {                            /* Bit Access       */
            unsigned short CINIT:16;
        } BIT;
    } CINIT;
    char           wk2[0x0E788C];
    union {                                 /* SIZE(0xA011C100)                     */
        unsigned long LONG;                 /* Long Access      */
        struct {                            /* Bit Access       */
            unsigned long D0SIZ:6;
            unsigned long :2;
            unsigned long D1SIZ:6;
            unsigned long :2;
            unsigned long DTSIZ:5;
            unsigned long :3;
            unsigned long CRC1SIZ:5;
            unsigned long :3;
        } BIT;
    } SIZE;
    union {                                 /* REGACS(0xA011C104)                   */
        unsigned long LONG;                 /* Long Access      */
        struct {                            /* Bit Access       */
            unsigned long WNR:1;
            unsigned long ADR:7;
            unsigned long ID:3;
            unsigned long :5;
            unsigned long TXD:8;
            unsigned long :8;
        } BIT;
    } REGACS;
    union {                                 /* WDG(0xA011C108)                      */
        unsigned long LONG;                 /* Long Access      */
        struct {                            /* Bit Access       */
            unsigned long WDG:32;
        } BIT;
    } WDG;
    char           wk3[0x04];
    union {                                 /* MODE(0xA011C110)                     */
        unsigned char BYTE;                 /* Byte Access      */
        struct {                            /* Bit Access       */
            unsigned char MODE:3;
            unsigned char :5;
        } BIT;
    } MODE;
    char           wk4[0x01];
    union {                                 /* MODE(0xA011C112)                     */
        unsigned char BYTE;                 /* Byte Access      */
        struct {                            /* Bit Access       */
            unsigned char ELCINE:1;
            unsigned char FIFOE:1;
            unsigned char FIFOE2:1;
            unsigned char :5;
        } BIT;
    } CTL;
    char           wk5[0x01];
    union {                                 /* SSCL(0xA011C114)                     */
        unsigned short WORD;                /* Word Access      */
        struct {                            /* Bit Access       */
            unsigned short WDG:1;
            unsigned short CRC1:1;
            unsigned short CRC2:1;
            unsigned short ALM:1;
            unsigned short WRN:1;
            unsigned short REGWERR:1;
            unsigned short RBERR:1;
            unsigned short WBERR:1;
            unsigned short SBERR:1;
            unsigned short AIDLERR:1;
            unsigned short FIFOERR:1;
            unsigned short :1;
            unsigned short REGNRD:1;
            unsigned short REGCEND:1;
            unsigned short :1;
            unsigned short END:1;
        } BIT;
    } SSCL;
    char           wk6[0x3EA];
    union {                                 /* RXD0(0xA011C500)                     */
        unsigned long LONG;                 /* Long Access      */
        struct {                            /* Bit Access       */
            unsigned long RXD0:24;
            unsigned long :8;
        } BIT;
    } RAW0;
        union {                             /* RXD0(0xA011C504)                     */
        unsigned long LONG;                 /* Long Access      */
        struct {                            /* Bit Access       */
            unsigned long RXD1:32;
        } BIT;
    } RAW1;
        union {                             /* RXD0(0xA011C508)                     */
        unsigned long LONG;                 /* Long Access      */
        struct {                            /* Bit Access       */
            unsigned long RXD2:32;
        } BIT;
    } RAW2;
    char           wk7[0x1F4];
    union {                                 /* RXD0(0xA011C700)                     */
        unsigned long LONG;                 /* Long Access      */
        struct {                            /* Bit Access       */
            unsigned long RXD0:32;
        } BIT;
    } RXD0;
    union {                                 /* RXD1(0xA011C704)                     */
        unsigned long LONG;                 /* Long Access      */
        struct {                            /* Bit Access       */
            unsigned long RXD1:32;
        } BIT;
    } RXD1;
    union {                                 /* RXD2(0xA011C708)                     */
        unsigned short WORD;                /* Word Access      */
        struct {                            /* Bit Access       */
            unsigned short REG:8;
            unsigned short :8;
        } BIT;
    } RXD2;
    char           wk8[0x02];
    union {                                 /* SS(0xA011C70C)                       */
        unsigned short WORD;                /* Word Access      */
        struct {                            /* Bit Access       */
            unsigned short WDG:1;
            unsigned short CRC1:1;
            unsigned short CRC2:1;
            unsigned short ALM:1;
            unsigned short WRN:1;
            unsigned short REGWERR:1;
            unsigned short RBERR:1;
            unsigned short WBERR:1;
            unsigned short SBERR:1;
            unsigned short AIDLERR:1;
            unsigned short FIFOERR:1;
            unsigned short :1;
            unsigned short REGNRD:1;
            unsigned short REGCEND:1;
            unsigned short REGCST:1;
            unsigned short END:1;
        } BIT;
    } SS;
    char           wk9[0x02];
    union {                                 /* IDL(0xA011C710)                      */
        unsigned char BYTE;                 /* Byte Access      */
        struct {                            /* Bit Access       */
            unsigned char IDL:8;
        } BIT;
    } IDL;
    char           wk10[0xB5AEF];
    union {                                 /* VER(0xA01D2200)                      */
        unsigned long LONG;                 /* Long Access      */
        struct {                            /* Bit Access       */
            unsigned long :31;
            unsigned long SRST:1;
        } BIT;
    } FIFORST;
    char           wk11[0x10];
    union {                                 /* VER(0xA01D2214)                      */
        unsigned long LONG;                 /* Long Access      */
        struct {                            /* Bit Access       */
            unsigned long PAETH:4;
            unsigned long :28;
        } BIT;
    } FIFOTH;
    char           wk12[0xE8];
    union {                                 /* VER(0xA01D2300)                      */
        unsigned long LONG;                 /* Long Access      */
        struct {                            /* Bit Access       */
            unsigned long MINR:16;
            unsigned long MAJR:16;
        } BIT;
    } VER;
};

typedef struct {
    unsigned long LONG;
    struct {
        unsigned long FDAT:32;
    } BIT;
} BISS_FDAT;

typedef struct {
    unsigned char BYTE;
    struct {
        unsigned short NFINTV:4;
        unsigned short NFSCNT:4;
    } BIT;
} BISS_NF;


