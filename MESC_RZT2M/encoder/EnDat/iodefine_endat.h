#pragma once

struct st_endat{
    union {
        unsigned long LONG;
        struct {
            unsigned long PI:16;
            unsigned long MAP:8;
            unsigned long MB:6;
            unsigned long :2;
        } BIT;
    } SEND;
    union {
        unsigned long LONG;
        struct {
            unsigned long :1;
            unsigned long DU:1;
            unsigned long DT:1;
            unsigned long :1;
            unsigned long FTCLK:4;
            unsigned long DWL:6;
            unsigned long RWIN:1;
            unsigned long ARST:1;
            unsigned long CPTU:8;
            unsigned long DLY:1;
            unsigned long :4;
            unsigned long RST:1;
            unsigned long :2;
        } BIT;
    } CFG1;
    union {
        unsigned long LONG;
        struct {
            unsigned long :8;
            unsigned long WDG:8;
            unsigned long TST:3;
            unsigned long :13;
        } BIT;
    } CFG2;
    union {
        unsigned long LONG;
        struct {
            unsigned long RX1:1;
            unsigned long ERR1:1;
            unsigned long CRC1:1;
            unsigned long FTYPE1:1;
            unsigned long FTYPE2:1;
            unsigned long MRSADR:1;
            unsigned long FIFOERR:1;
            unsigned long :1;
            unsigned long RX2:1;
            unsigned long RX3:1;
            unsigned long ERR2:1;
            unsigned long CRC3:1;
            unsigned long CRC2:1;
            unsigned long BUSY:1;
            unsigned long RM:1;
            unsigned long WRN:1;
            unsigned long :1;
            unsigned long WDG:1;
            unsigned long FTYPE3:1;
            unsigned long RXEND:1;
            unsigned long MBERR:1;
            unsigned long RDSTC:1;
            unsigned long :10;
        } BIT;
    } INTE;
    union {
        unsigned char BYTE;
        struct {
            unsigned char STRB:8;
        } BIT;
    } SWST;
    unsigned char dummy000[0x07];
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELC0:1;
            unsigned char FIFOE:1;
            unsigned char FIFO2:1;
            unsigned char :5;
        } BIT;
    } MODE;
    unsigned char dummy001[0x5E7];
    union {
        unsigned long LONG;
        struct {
            unsigned long RXD1:32;
        } BIT;
    } RECV1L;
    union {
        unsigned long LONG;
        struct {
            unsigned long RXD1:16;
            unsigned long RXCRC1:5;
            unsigned long RXERR1:1;
            unsigned long RXERR2:1;
            unsigned long :9;
        } BIT;
    } RECV1U;    
    union {
        unsigned long LONG;
        struct {
            unsigned long RXD2:24;
            unsigned long RXCRC2:5;
            unsigned long :3;
        } BIT;
    } RECV2;
    union {
        unsigned long LONG;
        struct {
            unsigned long RXD3:24;
            unsigned long RXCRC3:5;
            unsigned long :3;
        } BIT;
    } RECV3;
    unsigned char dummy002[0x602];
    union {
        unsigned short WORD;
        struct {
            unsigned short CPTL:16;
        } BIT;
    } CPTL;
    union {
        unsigned long LONG;
        struct {
            unsigned long RX1:1;
            unsigned long ERR1:1;
            unsigned long CRC1:1;
            unsigned long FTYPE1:1;
            unsigned long FTYPE2:1;
            unsigned long MRSADR:1;
            unsigned long FIFOERR:1;
            unsigned long :1;
            unsigned long RX2:1;
            unsigned long RX3:1;
            unsigned long ERR2:1;
            unsigned long CRC3:1;
            unsigned long CRC2:1;
            unsigned long BUSY:1;
            unsigned long RM:1;
            unsigned long WRN:1;
            unsigned long :1;
            unsigned long WDG:1;
            unsigned long FTYPE3:1;
            unsigned long RXEND:1;
            unsigned long MBERR:1;
            unsigned long RDSTC:1;
            unsigned long LZK:1;
            unsigned long LZM:1;
            unsigned long :6;
            unsigned long RDST:1;
            unsigned long RXTGL:1;
        } BIT;
    } STAT;
    unsigned char  dummy003[0xB54E8];
    union {
        unsigned long LONG;
        struct {
            unsigned long SRST:1;
            unsigned long :31;
        } BIT;
    } FIFOSRST;
    unsigned char  dummy004[0x10];
    union {
        unsigned long LONG;
        struct {
            unsigned long PAETH:4;
            unsigned long :28;
        } BIT;
    } FIFOTH;
    unsigned char  dummy005[0xE8];
    union {
        unsigned long LONG;
        struct {
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
} ENDAT_FDAT;

typedef struct {
    unsigned char BYTE;
    struct {
        unsigned short NFINTV:4;
        unsigned short NFSCNT:4;
    } BIT;
} ENDAT_NF;


