#pragma once

// Interrupt Enable
#define R_A_AS_INTE                     (0xD0)

// Value of the NF register
#define R_A_AS_NFINTV_2500KBPS          (0x00)
#define R_A_AS_NFINTV_4MBPS             (0x00)
#define R_A_AS_NFINTV_6670KBPS          (0x00)
#define R_A_AS_NFINTV_8MBPS             (0x00)
#define R_A_AS_NFSCNT_2500KBPS          (0x70)
#define R_A_AS_NFSCNT_4MBPS             (0x40)
#define R_A_AS_NFSCNT_6670KBPS          (0x20)
#define R_A_AS_NFSCNT_8MBPS             (0x10)

// Value of the T2 register
#define R_A_AS_T2_ONE_2500KBPS          (0x0014u)
#define R_A_AS_T2_ONE_4MBPS             (0x0014u)
#define R_A_AS_T2_ONE_6670KBPS          (0x0014u)
#define R_A_AS_T2_ONE_8MBPS             (0x0014u)
#define R_A_AS_T2_BUS_2500KBPS          (0x0096u)
#define R_A_AS_T2_BUS_4MBPS             (0x0064u)
#define R_A_AS_T2_BUS_6670KBPS          (0x0040u)
#define R_A_AS_T2_BUS_8MBPS             (0x003Cu)

// Value of the T3 register
#define R_A_AS_T3_2500KBPS              (0x001Eu)
#define R_A_AS_T3_4MBPS                 (0x0014u)
#define R_A_AS_T3_6670KBPS              (0x0014u)
#define R_A_AS_T3_8MBPS                 (0x000Eu)

// Value of the T4 register
#define R_A_AS_T4                       (0x00C8u)

// Value of the T5 register
#define R_A_AS_T5_2500KBPS              (0x003Cu)
#define R_A_AS_T5_4MBPS                 (0x0028u)
#define R_A_AS_T5_6670KBPS              (0x0028u)
#define R_A_AS_T5_8MBPS                 (0x001Eu)

// Value of the T9 register
#define R_A_AS_T9_ONE                   (0x005Eu)
#define R_A_AS_T9_BUS                   (0x00C2u)

