#include "r_enc_int_rzt2.h"

#include "eType.h"

#include <bsp_api.h>

#include <stdio.h>

#define RZT2M_MOTOR_AXIS_NUM        (2u)

static r_enc_isr_t r_enc_isr_func[RZT2M_MOTOR_AXIS_NUM]=
{
  {NULL, NULL},
  {NULL, NULL}
};

void r_enc_int_isr_open(uint8_t ch, const r_enc_isr_t* p_func)
{
    r_enc_isr_func[ch].r_enc_int_isr_func= p_func->r_enc_int_isr_func;
    r_enc_isr_func[ch].r_enc_err_isr_func= p_func->r_enc_err_isr_func;
}

void r_enc_ch0_int_isr(void)
{
    if(NULL != r_enc_isr_func[0u].r_enc_int_isr_func)
    {
        r_enc_isr_func[0u].r_enc_int_isr_func();
    }
    __DMB();
}

void r_enc_ch1_int_isr(void)
{
    if(NULL != r_enc_isr_func[1u].r_enc_int_isr_func)
    {
        r_enc_isr_func[1u].r_enc_int_isr_func();
    }
    __DMB();
}

void r_enc_ch0_err_int_isr(void)
{
    if(NULL != r_enc_isr_func[0u].r_enc_err_isr_func)
    {
        r_enc_isr_func[0u].r_enc_err_isr_func();
    }
    __DMB();
}

void r_enc_ch1_err_int_isr(void)
{
    if(NULL != r_enc_isr_func[1u].r_enc_err_isr_func)
    {
        r_enc_isr_func[1u].r_enc_err_isr_func();
    }
    __DMB();
}

void r_enc_ch0_elc_start(void)
{
    R_ELC->ELC_SSEL_b[13].ELC_SEL2 = 0x1D0;   // ENCIF0 CAP_TRG0:MTU4.TCNT underflow
}

void r_enc_ch0_elc_stop(void)
{
    R_ELC->ELC_SSEL_b[13].ELC_SEL2 = 0x3FF;   // ENCIF0 CAP_TRG0:MTU4.TCNT underflow
}

void r_enc_ch1_elc_start(void)
{
    R_ELC->ELC_SSEL_b[14].ELC_SEL0 = 0x1D2;   // ENCIF1 CAP_TRG1:MTU7.TCNT underflow
}

void r_enc_ch1_elc_stop(void)
{
    R_ELC->ELC_SSEL_b[14].ELC_SEL0 = 0x3FF;   // ENCIF0 CAP_TRG0:MTU4.TCNT underflow
}

void r_pinmux_encif04_create(short encoder_type)
{
    uint32_t port_num = 0;               /* Initialize ioport registers to the reset value. */
    
    R_BSP_RegisterProtectDisable(BSP_REG_PROTECT_GPIO);
    
    for(port_num=0; port_num < 25; port_num++)
    {
        R_PTADR->RSELP[port_num] = 0xFF;
    }
    
    /* Set MTIOC1A pin */
    R_PORT_NSR->PFC_b[0x0B].PFC1 = 0U;
    R_PORT_NSR->PMC_b[0x0B].PMC1 = 0U;
    R_PORT_NSR->PM_b[0x0B].PM1 = 0U;
    R_PORT_NSR->DRCTL[0x0B].L_b.DRV1 = 0U;
    R_PORT_NSR->DRCTL[0x0B].L_b.PUD1 = 0U;
    R_PORT_NSR->DRCTL[0x0B].L_b.SMT1 = 0U;
    R_PORT_NSR->DRCTL[0x0B].L_b.SR1 = 0U;

    /* Set MTCLKA pin */
    R_PORT_NSR->PFC_b[0x0D].PFC5 = 0U;
    R_PORT_NSR->PMC_b[0x0D].PMC5 = 0U;
    R_PORT_NSR->PM_b[0x0D].PM5 = 0U;
    R_PORT_NSR->DRCTL[0x0D].H_b.DRV5 = 0U;
    R_PORT_NSR->DRCTL[0x0D].H_b.PUD5 = 0U;
    R_PORT_NSR->DRCTL[0x0D].H_b.SMT5 = 0U;
    R_PORT_NSR->DRCTL[0x0D].H_b.SR5 = 0U;

    /* Set MTCLKB pin */
    R_PORT_NSR->PFC_b[0x0D].PFC6 = 0U;
    R_PORT_NSR->PMC_b[0x0D].PMC6 = 0U;
    R_PORT_NSR->PM_b[0x0D].PM6 = 0U;
    R_PORT_NSR->DRCTL[0x0D].H_b.DRV6 = 0U;
    R_PORT_NSR->DRCTL[0x0D].H_b.PUD6 = 0U;
    R_PORT_NSR->DRCTL[0x0D].H_b.SMT6 = 0U;
    R_PORT_NSR->DRCTL[0x0D].H_b.SR6 = 0U;
    
    /* Set ENCIF0 pin */
    R_PORT_NSR->PFC_b[0x10].PFC0 = 5U;
    R_PORT_NSR->PMC_b[0x10].PMC0 = 1U;
    R_PORT_NSR->PM_b[0x10].PM0 = 1U;
    R_PORT_NSR->DRCTL[0x10].L_b.DRV0 = 0U;
    R_PORT_NSR->DRCTL[0x10].L_b.PUD0 = 0U;
    R_PORT_NSR->DRCTL[0x10].L_b.SMT0 = 0U;
    R_PORT_NSR->DRCTL[0x10].L_b.SR0 = 1U;
    
    /* Set ENCIF1 pin */
    R_PORT_NSR->PFC_b[0x0B].PFC6 = 3U;
    R_PORT_NSR->PMC_b[0x0B].PMC6 = 1U;
    R_PORT_NSR->PM_b[0x0B].PM6 = 3U;
    R_PORT_NSR->DRCTL[0x0B].H_b.DRV6 = 0U;
    R_PORT_NSR->DRCTL[0x0B].H_b.PUD6 = 0U;
    R_PORT_NSR->DRCTL[0x0B].H_b.SMT6 = 0U;
    R_PORT_NSR->DRCTL[0x0B].H_b.SR6 = 1U;

    /* Set ENCIF2 pin */
    R_PORT_NSR->PFC_b[0x0B].PFC7 = 3U;
    R_PORT_NSR->PMC_b[0x0B].PMC7 = 1U;
    R_PORT_NSR->PM_b[0x0B].PM7 = 3U;
    R_PORT_NSR->DRCTL[0x0B].H_b.DRV7 = 0U;
    R_PORT_NSR->DRCTL[0x0B].H_b.PUD7 = 0U;
    R_PORT_NSR->DRCTL[0x0B].H_b.SMT7 = 0U;
    R_PORT_NSR->DRCTL[0x0B].H_b.SR7 = 1U;

    if ((ETYPE_INCREMENTAL != encoder_type) &&
        (ETYPE_APE_BISS != encoder_type))
    {
        /* Set ENCIF3 pin */
        R_PORT_NSR->PFC_b[0x0C].PFC0 = 2U;
        R_PORT_NSR->PMC_b[0x0C].PMC0 = 1U;
        R_PORT_NSR->PM_b[0x0C].PM0 = 2U;
        R_PORT_NSR->DRCTL[0x0C].L_b.DRV0 = 0U;
        R_PORT_NSR->DRCTL[0x0C].L_b.PUD0 = 0U;
        R_PORT_NSR->DRCTL[0x0C].L_b.SMT0 = 0U;
        R_PORT_NSR->DRCTL[0x0C].L_b.SR0 = 1U;
    }
    else
    {
        //ETYPE_APE_BISS
        /* Set ENCIF3 pin */
        R_PORT_NSR->PFC_b[0x0C].PFC0 = 0U;
        R_PORT_NSR->PMC_b[0x0C].PMC0 = 0U;
        R_PORT_NSR->PM_b[0x0C].PM0 = 2U;
        R_PORT_NSR->DRCTL[0x0C].L_b.DRV0 = 0U;
        R_PORT_NSR->DRCTL[0x0C].L_b.PUD0 = 0U;
        R_PORT_NSR->DRCTL[0x0C].L_b.SMT0 = 0U;
        R_PORT_NSR->DRCTL[0x0C].L_b.SR0 = 0U;
        R_PORT_NSR->P_b[0x0C].POUT_0 = 0U;
    }

    /* Set ENCIF4 pin */
    R_PORT_NSR->PFC_b[0x0C].PFC1 = 2U;
    R_PORT_NSR->PMC_b[0x0C].PMC1 = 1U;
    R_PORT_NSR->PM_b[0x0C].PM1 = 2U;
    R_PORT_NSR->DRCTL[0x0C].L_b.DRV1 = 0U;
    R_PORT_NSR->DRCTL[0x0C].L_b.PUD1 = 0U;
    R_PORT_NSR->DRCTL[0x0C].L_b.SMT1 = 0U;
    R_PORT_NSR->DRCTL[0x0C].L_b.SR1 = 1U;
}

void r_pinmux_encif04_release(void)
{
    uint32_t port_num = 0; /* Initialize ioport registers to the reset value. */
    
    R_BSP_RegisterProtectDisable(BSP_REG_PROTECT_GPIO);
    
    for(port_num=0; port_num < 25; port_num++)
    {
        R_PTADR->RSELP[port_num] = 0xFF;
    }
    
    /* Set MTIOC1A pin */
    R_PORT_NSR->PFC_b[0x0B].PFC1 = 2U;
    R_PORT_NSR->PMC_b[0x0B].PMC1 = 1U;
    R_PORT_NSR->PM_b[0x0B].PM1 = 1U;
    R_PORT_NSR->DRCTL[0x0B].L_b.DRV1 = 0U;
    R_PORT_NSR->DRCTL[0x0B].L_b.PUD1 = 0U;
    R_PORT_NSR->DRCTL[0x0B].L_b.SMT1 = 0U;
    R_PORT_NSR->DRCTL[0x0B].L_b.SR1 = 1U;

    /* Set MTCLKA pin */
    R_PORT_NSR->PFC_b[0x0D].PFC5 = 4U;
    R_PORT_NSR->PMC_b[0x0D].PMC5 = 1U;
    R_PORT_NSR->PM_b[0x0D].PM5 = 1U;
    R_PORT_NSR->DRCTL[0x0D].H_b.DRV5 = 0U;
    R_PORT_NSR->DRCTL[0x0D].H_b.PUD5 = 0U;
    R_PORT_NSR->DRCTL[0x0D].H_b.SMT5 = 0U;
    R_PORT_NSR->DRCTL[0x0D].H_b.SR5 = 1U;

    /* Set MTCLKB pin */
    R_PORT_NSR->PFC_b[0x0D].PFC6 = 4U;
    R_PORT_NSR->PMC_b[0x0D].PMC6 = 1U;
    R_PORT_NSR->PM_b[0x0D].PM6 = 1U;
    R_PORT_NSR->DRCTL[0x0D].H_b.DRV6 = 0U;
    R_PORT_NSR->DRCTL[0x0D].H_b.PUD6 = 0U;
    R_PORT_NSR->DRCTL[0x0D].H_b.SMT6 = 0U;
    R_PORT_NSR->DRCTL[0x0D].H_b.SR6 = 1U;
    
    /* Set ENCIF0 pin */
    R_PORT_NSR->PFC_b[0x10].PFC0 = 0U;
    R_PORT_NSR->PMC_b[0x10].PMC0 = 0U;
    R_PORT_NSR->PM_b[0x10].PM0 = 0U;
    R_PORT_NSR->DRCTL[0x10].L_b.DRV0 = 0U;
    R_PORT_NSR->DRCTL[0x10].L_b.PUD0 = 0U;
    R_PORT_NSR->DRCTL[0x10].L_b.SMT0 = 0U;
    R_PORT_NSR->DRCTL[0x10].L_b.SR0 = 0U;
    
    /* Set ENCIF1 pin */
    R_PORT_NSR->PFC_b[0x0B].PFC6 = 0U;
    R_PORT_NSR->PMC_b[0x0B].PMC6 = 0U;
    R_PORT_NSR->PM_b[0x0B].PM6 = 2U;
    R_PORT_NSR->DRCTL[0x0B].H_b.DRV6 = 0U;
    R_PORT_NSR->DRCTL[0x0B].H_b.PUD6 = 0U;
    R_PORT_NSR->DRCTL[0x0B].H_b.SMT6 = 0U;
    R_PORT_NSR->DRCTL[0x0B].H_b.SR6 = 0U;
    R_PORT_NSR->P_b[0x0B].POUT_6 = 0U;

    /* Set ENCIF2 pin */
    R_PORT_NSR->PFC_b[0x0B].PFC7 = 0U;
    R_PORT_NSR->PMC_b[0x0B].PMC7 = 0U;
    R_PORT_NSR->PM_b[0x0B].PM7 = 0U;
    R_PORT_NSR->DRCTL[0x0B].H_b.DRV7 = 0U;
    R_PORT_NSR->DRCTL[0x0B].H_b.PUD7 = 0U;
    R_PORT_NSR->DRCTL[0x0B].H_b.SMT7 = 0U;
    R_PORT_NSR->DRCTL[0x0B].H_b.SR7 = 0U;

    /* Set ENCIF3 pin */
    R_PORT_NSR->PFC_b[0x0C].PFC0 = 0U;
    R_PORT_NSR->PMC_b[0x0C].PMC0 = 0U;
    R_PORT_NSR->PM_b[0x0C].PM0 = 2U;
    R_PORT_NSR->DRCTL[0x0C].L_b.DRV0 = 0U;
    R_PORT_NSR->DRCTL[0x0C].L_b.PUD0 = 0U;
    R_PORT_NSR->DRCTL[0x0C].L_b.SMT0 = 0U;
    R_PORT_NSR->DRCTL[0x0C].L_b.SR0 = 0U;
    R_PORT_NSR->P_b[0x0C].POUT_0 = 0U;

    /* Set ENCIF4 pin */
    R_PORT_NSR->PFC_b[0x0C].PFC1 = 0U;
    R_PORT_NSR->PMC_b[0x0C].PMC1 = 0U;
    R_PORT_NSR->PM_b[0x0C].PM1 = 0U;
    R_PORT_NSR->DRCTL[0x0C].L_b.DRV1 = 0U;
    R_PORT_NSR->DRCTL[0x0C].L_b.PUD1 = 0U;
    R_PORT_NSR->DRCTL[0x0C].L_b.SMT1 = 0U;
    R_PORT_NSR->DRCTL[0x0C].L_b.SR1 = 0U;
}

void r_pinmux_encif59_create(short encoder_type)
{
    R_BSP_RegisterProtectDisable(BSP_REG_PROTECT_GPIO);

    for(uint32_t port_num=0; port_num < 25; port_num++)
    {
        R_PTADR->RSELP[port_num] = 0xFF;
    }
    
    // Set MTIOC2A pin
    R_PORT_NSR->PFC_b[0x0B].PFC3 = 0U;
    R_PORT_NSR->PMC_b[0x0B].PMC3 = 0U;
    R_PORT_NSR->PM_b[0x0B].PM3 = 0U;
    R_PORT_NSR->DRCTL[0x0B].L_b.DRV3 = 0U;
    R_PORT_NSR->DRCTL[0x0B].L_b.PUD3 = 0U;
    R_PORT_NSR->DRCTL[0x0B].L_b.SMT3 = 0U;
    R_PORT_NSR->DRCTL[0x0B].L_b.SR3 = 0U;

    // Set MTCLKC pin
    R_PORT_NSR->PFC_b[0x0D].PFC7 = 0U;
    R_PORT_NSR->PMC_b[0x0D].PMC7 = 0U;
    R_PORT_NSR->PM_b[0x0D].PM7 = 0U;
    R_PORT_NSR->DRCTL[0x0D].H_b.DRV7 = 0U;
    R_PORT_NSR->DRCTL[0x0D].H_b.PUD7 = 0U;
    R_PORT_NSR->DRCTL[0x0D].H_b.SMT7 = 0U;
    R_PORT_NSR->DRCTL[0x0D].H_b.SR7 = 0U;

    // Set MTCLKD pin
    R_PORT_NSR->PFC_b[0x0D].PFC1 = 0U;
    R_PORT_NSR->PMC_b[0x0D].PMC1 = 0U;
    R_PORT_NSR->PM_b[0x0D].PM1 = 0U;
    R_PORT_NSR->DRCTL[0x0D].L_b.DRV1 = 0U;
    R_PORT_NSR->DRCTL[0x0D].L_b.PUD1 = 0U;
    R_PORT_NSR->DRCTL[0x0D].L_b.SMT1 = 0U;
    R_PORT_NSR->DRCTL[0x0D].L_b.SR1 = 0U;
    
    // Set ENCIF5 pin
    R_PORT_NSR->PFC_b[0x0C].PFC2 = 2U;
    R_PORT_NSR->PMC_b[0x0C].PMC2 = 1U;
    R_PORT_NSR->PM_b[0x0C].PM2 = 1U;
    R_PORT_NSR->DRCTL[0x0C].L_b.DRV2 = 0U;
    R_PORT_NSR->DRCTL[0x0C].L_b.PUD2 = 0U;
    R_PORT_NSR->DRCTL[0x0C].L_b.SMT2 = 0U;
    R_PORT_NSR->DRCTL[0x0C].L_b.SR2 = 1U;

    // Set ENCIF6 pin
    R_PORT_NSR->PFC_b[0x0C].PFC3 = 2U;
    R_PORT_NSR->PMC_b[0x0C].PMC3 = 1U;
    R_PORT_NSR->PM_b[0x0C].PM3 = 3U;
    R_PORT_NSR->DRCTL[0x0C].L_b.DRV3 = 0U;
    R_PORT_NSR->DRCTL[0x0C].L_b.PUD3 = 0U;
    R_PORT_NSR->DRCTL[0x0C].L_b.SMT3 = 0U;
    R_PORT_NSR->DRCTL[0x0C].L_b.SR3 = 1U;

    // Set ENCIF7 pin
    R_PORT_NSR->PFC_b[0x11].PFC5 = 6U;
    R_PORT_NSR->PMC_b[0x11].PMC5 = 1U;
    R_PORT_NSR->PM_b[0x11].PM5 = 3U;
    R_PORT_NSR->DRCTL[0x11].H_b.DRV5 = 0U;
    R_PORT_NSR->DRCTL[0x11].H_b.PUD5 = 0U;
    R_PORT_NSR->DRCTL[0x11].H_b.SMT5 = 0U;
    R_PORT_NSR->DRCTL[0x11].H_b.SR5 = 1U;

    if((ETYPE_INCREMENTAL != encoder_type)&&
        (ETYPE_APE_BISS != encoder_type))
    {
        // Set ENCIF8 pin
        R_PORT_NSR->PFC_b[0x11].PFC6 = 3U;
        R_PORT_NSR->PMC_b[0x11].PMC6 = 1U;
        R_PORT_NSR->PM_b[0x11].PM6 = 2U;
        R_PORT_NSR->DRCTL[0x11].H_b.DRV6 = 0U;
        R_PORT_NSR->DRCTL[0x11].H_b.PUD6 = 0U;
        R_PORT_NSR->DRCTL[0x11].H_b.SMT6 = 0U;
        R_PORT_NSR->DRCTL[0x11].H_b.SR6 = 1U;
    }
    else
    {
        //ETYPE_APE_BISS
        // Set ENCIF8 pin
        R_PORT_NSR->PFC_b[0x11].PFC6 = 0U;
        R_PORT_NSR->PMC_b[0x11].PMC6 = 0U;
        R_PORT_NSR->PM_b[0x11].PM6 = 2U;
        R_PORT_NSR->DRCTL[0x11].H_b.DRV6 = 0U;
        R_PORT_NSR->DRCTL[0x11].H_b.PUD6 = 0U;
        R_PORT_NSR->DRCTL[0x11].H_b.SMT6 = 0U;
        R_PORT_NSR->DRCTL[0x11].H_b.SR6 = 0U;
        R_PORT_NSR->P_b[0x11].POUT_6 = 0U;
    }

    // Set ENCIF9 pin
    R_PORT_NSR->PFC_b[0x11].PFC7 = 6U;
    R_PORT_NSR->PMC_b[0x11].PMC7 = 1U;
    R_PORT_NSR->PM_b[0x11].PM7 = 2U;
    R_PORT_NSR->DRCTL[0x11].H_b.DRV7 = 0U;
    R_PORT_NSR->DRCTL[0x11].H_b.PUD7 = 0U;
    R_PORT_NSR->DRCTL[0x11].H_b.SMT7 = 0U;
    R_PORT_NSR->DRCTL[0x11].H_b.SR7 = 1U;
}

void r_pinmux_encif59_release(void)
{
    R_BSP_RegisterProtectDisable(BSP_REG_PROTECT_GPIO);
    
    for(uint32_t port_num=0; port_num < 25; port_num++)
    {
        R_PTADR->RSELP[port_num] = 0xFF;
    }
    
    // Set MTIOC2A pin
    R_PORT_NSR->PFC_b[0x0B].PFC3 = 2U;
    R_PORT_NSR->PMC_b[0x0B].PMC3 = 1U;
    R_PORT_NSR->PM_b[0x0B].PM3 = 1U;
    R_PORT_NSR->DRCTL[0x0B].L_b.DRV3 = 0U;
    R_PORT_NSR->DRCTL[0x0B].L_b.PUD3 = 0U;
    R_PORT_NSR->DRCTL[0x0B].L_b.SMT3 = 0U;
    R_PORT_NSR->DRCTL[0x0B].L_b.SR3 = 1U;

    // Set MTCLKC pin
    R_PORT_NSR->PFC_b[0x0D].PFC7 = 4U;
    R_PORT_NSR->PMC_b[0x0D].PMC7 = 1U;
    R_PORT_NSR->PM_b[0x0D].PM7 = 1U;
    R_PORT_NSR->DRCTL[0x0D].H_b.DRV7 = 0U;
    R_PORT_NSR->DRCTL[0x0D].H_b.PUD7 = 0U;
    R_PORT_NSR->DRCTL[0x0D].H_b.SMT7 = 0U;
    R_PORT_NSR->DRCTL[0x0D].H_b.SR7 = 1U;

    // Set MTCLKD pin
    R_PORT_NSR->PFC_b[0x0D].PFC1 = 3U;
    R_PORT_NSR->PMC_b[0x0D].PMC1 = 1U;
    R_PORT_NSR->PM_b[0x0D].PM1 = 1U;
    R_PORT_NSR->DRCTL[0x0D].L_b.DRV1 = 0U;
    R_PORT_NSR->DRCTL[0x0D].L_b.PUD1 = 0U;
    R_PORT_NSR->DRCTL[0x0D].L_b.SMT1 = 0U;
    R_PORT_NSR->DRCTL[0x0D].L_b.SR1 = 1U;
    
    // Set ENCIF5 pin
    R_PORT_NSR->PFC_b[0x0C].PFC2 = 0U;
    R_PORT_NSR->PMC_b[0x0C].PMC2 = 0U;
    R_PORT_NSR->PM_b[0x0C].PM2 = 0U;
    R_PORT_NSR->DRCTL[0x0C].L_b.DRV2 = 0U;
    R_PORT_NSR->DRCTL[0x0C].L_b.PUD2 = 0U;
    R_PORT_NSR->DRCTL[0x0C].L_b.SMT2 = 0U;
    R_PORT_NSR->DRCTL[0x0C].L_b.SR2 = 0U;

    // Set ENCIF6 pin
    R_PORT_NSR->PFC_b[0x0C].PFC3 = 0U;
    R_PORT_NSR->PMC_b[0x0C].PMC3 = 0U;
    R_PORT_NSR->PM_b[0x0C].PM3 = 2U;
    R_PORT_NSR->DRCTL[0x0C].L_b.DRV3 = 0U;
    R_PORT_NSR->DRCTL[0x0C].L_b.PUD3 = 0U;
    R_PORT_NSR->DRCTL[0x0C].L_b.SMT3 = 0U;
    R_PORT_NSR->DRCTL[0x0C].L_b.SR3 = 0U;
    R_PORT_NSR->P_b[0x0C].POUT_3 = 0U;

    // Set ENCIF7 pin
    R_PORT_NSR->PFC_b[0x11].PFC5 = 0U;
    R_PORT_NSR->PMC_b[0x11].PMC5 = 0U;
    R_PORT_NSR->PM_b[0x11].PM5 = 0U;
    R_PORT_NSR->DRCTL[0x11].H_b.DRV5 = 0U;
    R_PORT_NSR->DRCTL[0x11].H_b.PUD5 = 0U;
    R_PORT_NSR->DRCTL[0x11].H_b.SMT5 = 0U;
    R_PORT_NSR->DRCTL[0x11].H_b.SR5 = 0U;

    // Set ENCIF8 pin
    R_PORT_NSR->PFC_b[0x11].PFC6 = 0U;
    R_PORT_NSR->PMC_b[0x11].PMC6 = 0U;
    R_PORT_NSR->PM_b[0x11].PM6 = 2U;
    R_PORT_NSR->DRCTL[0x11].H_b.DRV6 = 0U;
    R_PORT_NSR->DRCTL[0x11].H_b.PUD6 = 0U;
    R_PORT_NSR->DRCTL[0x11].H_b.SMT6 = 0U;
    R_PORT_NSR->DRCTL[0x11].H_b.SR6 = 0U;
    R_PORT_NSR->P_b[0x11].POUT_6 = 0U;

    // Set ENCIF9 pin
    R_PORT_NSR->PFC_b[0x11].PFC7 = 0U;
    R_PORT_NSR->PMC_b[0x11].PMC7 = 0U;
    R_PORT_NSR->PM_b[0x11].PM7 = 0U;
    R_PORT_NSR->DRCTL[0x11].H_b.DRV7 = 0U;
    R_PORT_NSR->DRCTL[0x11].H_b.PUD7 = 0U;
    R_PORT_NSR->DRCTL[0x11].H_b.SMT7 = 0U;
    R_PORT_NSR->DRCTL[0x11].H_b.SR7 = 0U;
}

void copy_to_rodata_encif(unsigned char *p, const void *p_org, unsigned int sz)
{
    volatile unsigned int   psize=sz;
    
    memcpy(p, p_org, psize);

    __asm volatile("dsb"); // Ensuring data-changing
}


