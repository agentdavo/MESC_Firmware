#pragma once

#include "iodefine_hfdsl.h"

#define R_HFDSL0_ID      (0x01)
#define R_HFDSL1_ID      (0x02)

/* HFDSL register settings */
#define R_HFDSL_ES_PRDY             (0x0100u)
#define R_HFDSL_MAXACC              (1023u)
#define R_HFDSL_ACC_ERR             (31u)
#define R_HFDSL_MAXDEV              (65535u)
#define R_HFDSL_MASK                (0x64u) // remove RAW_TOVR
#define R_HFDSL_MASK_ERR            (0x00FF33DFu)
#define R_HFDSL_TH_RAW              (0x0001u)
#define R_HFDSL_RAW_EN              (0xFFu) // message recv / raw_all enable
#define R_HFDSL_DELAY_UPP_LIMIT     (0x9u)
#define R_HFDSL_STUFF               (0x08u)
#define R_HFDSL_EXLEN               (0x00u)
#define R_HFDSL_EXTRA               (0x0003u)

