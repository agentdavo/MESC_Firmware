#pragma once

typedef struct st_fac {
    char    wk0[2];
    union {
        unsigned short WORD;
        struct {
            unsigned short TIMOT:16;
        } BIT;
    } TIMOT;
    char    wk2[4];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TXAD:7;
            unsigned char BUSY:1;
        } BIT;
    } TXADF;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TXED:8;
        } BIT;
    } TXEDF;
    union {
        unsigned short WORD;
        struct {
            unsigned short ELCEN:1;
            unsigned short :15;
        } BIT;
    } OPT;
    char    wk4[      22];
    char    wk5[   0x3e0];
    char    wk6[2];
    char    wk7[2];
    char    wk8[2];
    union {  
        unsigned long LONG;
        struct {
            unsigned long RXDF0:8;
            unsigned long RXDF1:8;
            unsigned long RXDF2:8;
            unsigned long RXDF3:8;
        } BIT;
    } RXD0;
    union {
        unsigned long LONG;
        struct {
            unsigned long RXDF4:8;
            unsigned long RXDF5:8;
            unsigned long RXDF6:8;
            unsigned long RXDF7:8;
        } BIT;
    } RXD1;
    char    wk9[1];
    char    wk10[      15];
    char    wk11[   0x7e0];
    union {
        unsigned short WORD;
        struct {
            unsigned short ID:4;
            unsigned short DFNUM:4;
            unsigned short EA:1;    /* EEPROM access enable(preliminary) */
            unsigned short EW:1;    /* EEPROM access direction(preliminary) */ 
        } BIT;
    } TX;
    char    wk12[       2];
    union {
        unsigned long LONG;
        struct {
            unsigned long RSE:1;
            unsigned long IDE:1;
            unsigned long EBUSY:1;
            unsigned long RXID:4;
            unsigned long RXIDP:1;
            unsigned long RXSFIC:4;
            unsigned long RXSFEA:2;
            unsigned long RXSFCA:2;
            unsigned long RXCRC:8;
            unsigned long CONTE:1;
            unsigned long CRCE:1;
            unsigned long FOME:1;
            unsigned long SFOME:1;
            unsigned long TIMOTE:1;
            unsigned long RXEDFE:1;
            unsigned long RXADFE:1;
            unsigned long DFOVFE:1;
        } BIT;
    } RXRESULT;
    char    wk13[       0xB55F8];
    union {
        unsigned long LONG;
        struct {
            unsigned long VER:32;
        } BIT;
    } VER;
#if 0
    char    wk14[       0xE02CFC];
    union {
        unsigned long LONG;
        struct {
            unsigned long :3;
            unsigned long INF:1;
            unsigned long :20;
            unsigned long NFINTV:4;
            unsigned long NFSCNT:4;
        } BIT;
    } NF;
#endif
} st_fac_t;

typedef struct {
    unsigned long LONG;
    struct {
        unsigned long :3;
        unsigned long INF:1;
        unsigned long :20;
        unsigned long NFINTV:4;
        unsigned long NFSCNT:4;
    } BIT;
} st_fac_nf;


