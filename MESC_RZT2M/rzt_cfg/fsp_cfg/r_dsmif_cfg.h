/* generated configuration header file - do not edit */
#ifndef R_DSMIF_CFG_H_
#define R_DSMIF_CFG_H_
#define DSMIF_CFG_PARAM_CHECKING_ENABLE (BSP_CFG_PARAM_CHECKING_ENABLE)
            #define DSMIF_CFG_MULTIPLEX_INTERRUPT_SUPPORTED ((0))
            #if DSMIF_CFG_MULTIPLEX_INTERRUPT_SUPPORTED
             #define DSMIF_CFG_MULTIPLEX_INTERRUPT_ENABLE         BSP_INTERRUPT_ENABLE
             #define DSMIF_CFG_MULTIPLEX_INTERRUPT_DISABLE        BSP_INTERRUPT_DISABLE
            #else
             #define DSMIF_CFG_MULTIPLEX_INTERRUPT_ENABLE
             #define DSMIF_CFG_MULTIPLEX_INTERRUPT_DISABLE
            #endif
#endif /* R_DSMIF_CFG_H_ */
