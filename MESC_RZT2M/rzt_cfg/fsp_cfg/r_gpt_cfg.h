/* generated configuration header file - do not edit */
#ifndef R_GPT_CFG_H_
#define R_GPT_CFG_H_
#define GPT_CFG_PARAM_CHECKING_ENABLE (BSP_CFG_PARAM_CHECKING_ENABLE)
#define GPT_CFG_OUTPUT_SUPPORT_ENABLE (0)
#define GPT_CFG_WRITE_PROTECT_ENABLE (0)

#define GPT_CFG_MULTIPLEX_INTERRUPT_SUPPORTED (0)
#if GPT_CFG_MULTIPLEX_INTERRUPT_SUPPORTED
 #define GPT_CFG_MULTIPLEX_INTERRUPT_ENABLE         BSP_INTERRUPT_ENABLE
 #define GPT_CFG_MULTIPLEX_INTERRUPT_DISABLE        BSP_INTERRUPT_DISABLE
#else
 #define GPT_CFG_MULTIPLEX_INTERRUPT_ENABLE
 #define GPT_CFG_MULTIPLEX_INTERRUPT_DISABLE
#endif
#endif /* R_GPT_CFG_H_ */
