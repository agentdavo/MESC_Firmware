#pragma once

struct st_a_as{
    union {
        unsigned long LONG;
        struct {
            unsigned long EA:3;
            unsigned long CC:5;
            unsigned long :8;
            unsigned long :5;
            unsigned long :3;
        } BIT;
    } TXC;
    union {
        unsigned long LONG;
        struct {
            unsigned long TXD:32;
        } BIT;
    } TXD;
    union {
        unsigned short WORD;
        struct {
            unsigned short T2:16;
        } BIT;
    } T2;
    union {
        unsigned short WORD;
        struct {
            unsigned short T3:16;
        } BIT;
    } T3;
    union {
        unsigned short WORD;
        struct {
            unsigned short T4:16;
        } BIT;
    } T4;
    union {
        unsigned short WORD;
        struct {
            unsigned short T5:16;
        } BIT;
    } T5;
    union {
        unsigned short WORD;
        struct {
            unsigned short T9:16;
        } BIT;
    } T9;
    union {
        unsigned short WORD;
        struct {
            unsigned short IFMG:16;
        } BIT;
    } IFMG;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMENA:1;
            unsigned char :1;
            unsigned char ELCIN0E:1;
            unsigned char ELCIN1E:1;
            unsigned char TXBPE:1;
            unsigned char RXBPE:1;
            unsigned char RXENDE:1;
            unsigned char :1;
        } BIT;
    } CTL;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ESEL:3;
            unsigned char :5;
        } BIT;
    } ESEL;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TRG:8;
        } BIT;
    } TRG;
    union {
        unsigned char BYTE;
        struct {
            unsigned char BR:2;
            unsigned char :6;
        } BIT;
    } BR;
    union {   // 11C118h
        unsigned long LONG;
        struct {
            unsigned long TI:24;
            unsigned long :8;
        } BIT;
    } TI;
    unsigned char  dummy11C11C[   0x1E4];
    union {  // 11C300h
        unsigned char BYTE;
        struct {
            unsigned char :4;
            unsigned char RXSET:1;
            unsigned char :1;
            unsigned char TXERR:1;
            unsigned char RXEND:1;
        } BIT;
    } INTE;
    unsigned char  dummy11C301[   0x407];
    union {  // 11C708h
        unsigned long LONG;
        struct {
            unsigned long RXI:32;
        } BIT;
    } RXI;
    union {  // 11C70Ch
        unsigned long LONG;
        struct {
            unsigned long RXD0:32;
        } BIT;
    } RXD0;
    union {  // 11C710h
        unsigned long LONG;
        struct {
            unsigned long RXD1:32;
        } BIT;
    } RXD1;
    unsigned char  dummy11C714[   0x608];
    union {  //  11CD1Ch
        unsigned long LONG;
        struct {
            unsigned long IWDGERR:1;
            unsigned long DWDGERR:1;
            unsigned long STARTERR:1;
            unsigned long STOPERR:1;
            unsigned long SYNCERR:1;
            unsigned long RXEAERR:1;
            unsigned long CRCERR:1;
            unsigned long RXCCERR:1;
            unsigned long MDATERR:1;
            unsigned long MADRERR:1;
            unsigned long RXDZERR:1;
            unsigned long FD1ERR:1;
            unsigned long FD2ERR:1;
            unsigned long FD3ERR:1;
            unsigned long :1;
            unsigned long FD5ERR:1;
            unsigned long :1;
            unsigned long :1;
            unsigned long :1;
            unsigned long :1;
            unsigned long :1;
            unsigned long :1;
            unsigned long :1;
            unsigned long :1;
            unsigned long ELCIN0:1;
            unsigned long ELCIN1:1;
            unsigned long TXCC:2;
            unsigned long RXSET:1;
            unsigned long TIMER:1;
            unsigned long TXERR:1;
            unsigned long RXEND:1;
        } BIT;
    } SS;
    unsigned char  dummy11CD20[ 0xB55E0];
    union {  // 1D2300h
        unsigned long LONG;
        struct {
            unsigned long MINR:16;
            unsigned long MAJR:16;
        } BIT;
    } VER;
#if 0
    unsigned char  dummy1D2304[ 0xE00CFC];
    union {  // FD3000h
        unsigned long LONG;
        struct {
            unsigned long DAT0:16;
            unsigned long DAT1:16;
        } BIT;
    } FDAT;
    unsigned char  dummyFD3004[ 0x1FFF];
    union {  // FD5003h
        unsigned char BYTE;
        struct {
            unsigned char NFINTV:4;
            unsigned char NFSCNT:4;
        } BIT;
    } NF;
#endif // 0
};

typedef struct {
    unsigned long LONG;
    struct {
        unsigned long DAT0:16;
        unsigned long DAT1:16;
    } BIT;
} A_AS_FDAT;

typedef struct {
    unsigned char BYTE;
    struct {
        unsigned char NFINTV:4;
        unsigned char NFSCNT:4;
    } BIT;
} A_AS_NF;

#define A_AS_DR(addr) (*(volatile struct st_a_as *)(addr))


