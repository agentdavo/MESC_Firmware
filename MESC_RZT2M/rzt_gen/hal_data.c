/* generated HAL source file - do not edit */
#include "hal_data.h"



/* Macros to tie dynamic ELC links to ADC_TRIGGER_SYNC_ELC option in adc_trigger_t. */
#define ADC_TRIGGER_ADC0_A      ADC_TRIGGER_SYNC_ELC
#define ADC_TRIGGER_ADC0_B      ADC_TRIGGER_SYNC_ELC
#define ADC_TRIGGER_ADC1_A      ADC_TRIGGER_SYNC_ELC
#define ADC_TRIGGER_ADC1_B      ADC_TRIGGER_SYNC_ELC



tsu_instance_ctrl_t g_tsu0_ctrl;
const adc_cfg_t g_tsu0_cfg =
{
    .unit           = 0,
    .mode           = ADC_MODE_SINGLE_SCAN,
    .resolution     = ADC_RESOLUTION_12_BIT,
    .alignment      = ADC_ALIGNMENT_RIGHT,
    .trigger        = ADC_TRIGGER_SOFTWARE,
    .scan_end_irq   = FSP_INVALID_VECTOR,
    .scan_end_ipl   = (BSP_IRQ_DISABLED),
    .scan_end_b_irq = FSP_INVALID_VECTOR,
    .scan_end_b_ipl = (BSP_IRQ_DISABLED),
    .scan_end_c_irq = FSP_INVALID_VECTOR,
    .scan_end_c_ipl = (BSP_IRQ_DISABLED),
    .p_callback     = NULL,
    .p_context      = NULL,
    .p_extend       = NULL,
};
/* Instance structure to use this module. */
const adc_instance_t g_tsu0 =
{
    .p_ctrl        = &g_tsu0_ctrl,
    .p_cfg         = &g_tsu0_cfg,
    .p_channel_cfg = NULL,
    .p_api         = &g_adc_on_tsu,
};
dsmif_channel_cfg_t g_dsmif1_channel_cfg2 =
{
    .ioel                = 0,
    .ioeh                = 0,
    .ise                 = 0,
    .iue                 = 0,
    .ckdir               = DSMIF_CLOCK_CTRL_SLAVE,
    .sedge               = DSMIF_CLOCK_EDGE_NEGATIVE,
    .ckdiv               = DSMIF_MASTER_CLOCK_25MHZ_PCLKH200,
    .cmsinc              = DSMIF_FILTER_ORDER_3RD,
    .cmdec               = 3,
    .sde                 = 0,
    .ocsinc              = DSMIF_FILTER_ORDER_3RD,
    .ocdec               = 3,
    .ocmptbl             = 0x0,
    .ocmptbh             = 0x0,
    .scntl               = 0x0,
    .scnth               = 0x0,
    .odel                = 0,
    .odeh                = 0,
    .cmsh                = (dsimf_data_shift_t)18,
    .ocsh                = (dsimf_data_shift_t)18
};
#define DSMIF_CHANNEL_12
dsmif_channel_cfg_t g_dsmif1_channel_cfg1 =
{
    .ioel                = 0,
    .ioeh                = 0,
    .ise                 = 0,
    .iue                 = 0,
    .ckdir               = DSMIF_CLOCK_CTRL_SLAVE,
    .sedge               = DSMIF_CLOCK_EDGE_NEGATIVE,
    .ckdiv               = DSMIF_MASTER_CLOCK_25MHZ_PCLKH200,
    .cmsinc              = DSMIF_FILTER_ORDER_3RD,
    .cmdec               = 3,
    .sde                 = 0,
    .ocsinc              = DSMIF_FILTER_ORDER_3RD,
    .ocdec               = 3,
    .ocmptbl             = 0x0,
    .ocmptbh             = 0x0,
    .scntl               = 0x0,
    .scnth               = 0x0,
    .odel                = 0,
    .odeh                = 0,
    .cmsh                = (dsimf_data_shift_t)18,
    .ocsh                = (dsimf_data_shift_t)18
};
#define DSMIF_CHANNEL_11
dsmif_channel_cfg_t g_dsmif1_channel_cfg0 =
{
    .ioel                = 0,
    .ioeh                = 0,
    .ise                 = 0,
    .iue                 = 0,
    .ckdir               = DSMIF_CLOCK_CTRL_SLAVE,
    .sedge               = DSMIF_CLOCK_EDGE_NEGATIVE,
    .ckdiv               = DSMIF_MASTER_CLOCK_25MHZ_PCLKH200,
    .cmsinc              = DSMIF_FILTER_ORDER_3RD,
    .cmdec               = 3,
    .sde                 = 0,
    .ocsinc              = DSMIF_FILTER_ORDER_3RD,
    .ocdec               = 3,
    .ocmptbl             = 0x0,
    .ocmptbh             = 0x0,
    .scntl               = 0x0,
    .scnth               = 0x0,
    .odel                = 0,
    .odeh                = 0,
    .cmsh                = (dsimf_data_shift_t)18,
    .ocsh                = (dsimf_data_shift_t)18
};
#define DSMIF_CHANNEL_10
dsmif_instance_ctrl_t g_dsmif1_ctrl;
#define DSMIF_MASK_UNIT_10  (DSMIF_UNIT0_MASK_10 | DSMIF_UNIT0_MASK_11 | DSMIF_UNIT0_MASK_12)
#define DSMIF_MASK_UNIT_11  (DSMIF_UNIT1_MASK_10 | DSMIF_UNIT1_MASK_11 | DSMIF_UNIT1_MASK_12)
/** DSMIF configuration extension. This extension is required and must be provided in adc_cfg_t::p_extend. */
const dsmif_extended_cfg_t g_dsmif1_cfg_extend =
{
    .isel                = 0,
    .iseh                = 0,
    .sedm                = DSMIF_SUM_ERR_DETECT_CHANNEL_0_2,
    .scmptbl             = 0x00000,
    .scmptbh             = 0x00000,
    .seel                = 0,
    .seeh                = 0,
    .cap_trig_a          = DSMIF_CAPTURE_TRIGGER_NOT,
    .cap_trig_b          = DSMIF_CAPTURE_TRIGGER_NOT,
    .cnt_init_trig       = DSMIF_COUNTER_INIT_TRIGGER_NOT,
    .edge                = DSMIF_CLOCK_EDGE_NEGATIVE,
#ifndef DSMIF_CHANNEL_10
#define DSMIF_UNIT0_MASK_10   (0)
#define DSMIF_UNIT1_MASK_10   (0)
    .p_channel_cfgs[0]   = NULL,
#else
#define DSMIF_UNIT0_MASK_10   (DSMIF_CHANNEL_MASK_0)
#define DSMIF_UNIT1_MASK_10   (DSMIF_CHANNEL_MASK_3)
    .p_channel_cfgs[0]   = &g_dsmif1_channel_cfg0,
#endif
#ifndef DSMIF_CHANNEL_11
#define DSMIF_UNIT0_MASK_11   (0)
#define DSMIF_UNIT1_MASK_11   (0)
    .p_channel_cfgs[1]   = NULL,
#else
#define DSMIF_UNIT0_MASK_11   (DSMIF_CHANNEL_MASK_1)
#define DSMIF_UNIT1_MASK_11   (DSMIF_CHANNEL_MASK_4)
    .p_channel_cfgs[1]   = &g_dsmif1_channel_cfg1,
#endif
#ifndef DSMIF_CHANNEL_12
#define DSMIF_UNIT0_MASK_12   (0)
#define DSMIF_UNIT1_MASK_12   (0)
    .p_channel_cfgs[2]   = NULL,
#else
#define DSMIF_UNIT0_MASK_12   (DSMIF_CHANNEL_MASK_2)
#define DSMIF_UNIT1_MASK_12   (DSMIF_CHANNEL_MASK_5)
    .p_channel_cfgs[2]   = &g_dsmif1_channel_cfg2,
#endif
    .channel_mask        = (dsmif_channel_mask_t)DSMIF_MASK_UNIT_11,
};
const adc_cfg_t g_dsmif1_cfg =
{
    .unit                = 1,
    .mode                = ADC_MODE_SYNCHRONIZE_SCAN,
#if defined(VECTOR_NUMBER_DSMIF1_CDRUI)
    .scan_end_irq        = VECTOR_NUMBER_DSMIF1_CDRUI,
#else
    .scan_end_irq        = FSP_INVALID_VECTOR,
#endif
    .scan_end_ipl           = (BSP_IRQ_DISABLED),
    .p_callback          = NULL,
    .p_context           = NULL,
    .p_extend            = &g_dsmif1_cfg_extend,
};
/* Instance structure to use this module. */
const adc_instance_t g_dsmif1 =
{
    .p_ctrl        = &g_dsmif1_ctrl,
    .p_cfg         = &g_dsmif1_cfg,
    .p_api         = &g_adc_on_dsmif
};
dsmif_channel_cfg_t g_dsmif0_channel_cfg2 =
{
    .ioel                = 0,
    .ioeh                = 0,
    .ise                 = 0,
    .iue                 = 0,
    .ckdir               = DSMIF_CLOCK_CTRL_SLAVE,
    .sedge               = DSMIF_CLOCK_EDGE_NEGATIVE,
    .ckdiv               = DSMIF_MASTER_CLOCK_25MHZ_PCLKH200,
    .cmsinc              = DSMIF_FILTER_ORDER_3RD,
    .cmdec               = 3,
    .sde                 = 0,
    .ocsinc              = DSMIF_FILTER_ORDER_3RD,
    .ocdec               = 3,
    .ocmptbl             = 0x0,
    .ocmptbh             = 0x0,
    .scntl               = 0x0,
    .scnth               = 0x0,
    .odel                = 0,
    .odeh                = 0,
    .cmsh                = (dsimf_data_shift_t)18,
    .ocsh                = (dsimf_data_shift_t)18
};
#define DSMIF_CHANNEL_02
dsmif_channel_cfg_t g_dsmif0_channel_cfg1 =
{
    .ioel                = 0,
    .ioeh                = 0,
    .ise                 = 0,
    .iue                 = 0,
    .ckdir               = DSMIF_CLOCK_CTRL_SLAVE,
    .sedge               = DSMIF_CLOCK_EDGE_NEGATIVE,
    .ckdiv               = DSMIF_MASTER_CLOCK_25MHZ_PCLKH200,
    .cmsinc              = DSMIF_FILTER_ORDER_3RD,
    .cmdec               = 3,
    .sde                 = 0,
    .ocsinc              = DSMIF_FILTER_ORDER_3RD,
    .ocdec               = 3,
    .ocmptbl             = 0x0,
    .ocmptbh             = 0x0,
    .scntl               = 0x0,
    .scnth               = 0x0,
    .odel                = 0,
    .odeh                = 0,
    .cmsh                = (dsimf_data_shift_t)18,
    .ocsh                = (dsimf_data_shift_t)18
};
#define DSMIF_CHANNEL_01
dsmif_channel_cfg_t g_dsmif0_channel_cfg0 =
{
    .ioel                = 0,
    .ioeh                = 0,
    .ise                 = 0,
    .iue                 = 0,
    .ckdir               = DSMIF_CLOCK_CTRL_SLAVE,
    .sedge               = DSMIF_CLOCK_EDGE_NEGATIVE,
    .ckdiv               = DSMIF_MASTER_CLOCK_25MHZ_PCLKH200,
    .cmsinc              = DSMIF_FILTER_ORDER_3RD,
    .cmdec               = 3,
    .sde                 = 0,
    .ocsinc              = DSMIF_FILTER_ORDER_3RD,
    .ocdec               = 3,
    .ocmptbl             = 0x0,
    .ocmptbh             = 0x0,
    .scntl               = 0x0,
    .scnth               = 0x0,
    .odel                = 0,
    .odeh                = 0,
    .cmsh                = (dsimf_data_shift_t)18,
    .ocsh                = (dsimf_data_shift_t)18
};
#define DSMIF_CHANNEL_00
dsmif_instance_ctrl_t g_dsmif0_ctrl;
#define DSMIF_MASK_UNIT_00  (DSMIF_UNIT0_MASK_00 | DSMIF_UNIT0_MASK_01 | DSMIF_UNIT0_MASK_02)
#define DSMIF_MASK_UNIT_01  (DSMIF_UNIT1_MASK_00 | DSMIF_UNIT1_MASK_01 | DSMIF_UNIT1_MASK_02)
/** DSMIF configuration extension. This extension is required and must be provided in adc_cfg_t::p_extend. */
const dsmif_extended_cfg_t g_dsmif0_cfg_extend =
{
    .isel                = 0,
    .iseh                = 0,
    .sedm                = DSMIF_SUM_ERR_DETECT_CHANNEL_0_2,
    .scmptbl             = 0x00000,
    .scmptbh             = 0x00000,
    .seel                = 0,
    .seeh                = 0,
    .cap_trig_a          = DSMIF_CAPTURE_TRIGGER_NOT,
    .cap_trig_b          = DSMIF_CAPTURE_TRIGGER_NOT,
    .cnt_init_trig       = DSMIF_COUNTER_INIT_TRIGGER_NOT,
    .edge                = DSMIF_CLOCK_EDGE_NEGATIVE,
#ifndef DSMIF_CHANNEL_00
#define DSMIF_UNIT0_MASK_00   (0)
#define DSMIF_UNIT1_MASK_00   (0)
    .p_channel_cfgs[0]   = NULL,
#else
#define DSMIF_UNIT0_MASK_00   (DSMIF_CHANNEL_MASK_0)
#define DSMIF_UNIT1_MASK_00   (DSMIF_CHANNEL_MASK_3)
    .p_channel_cfgs[0]   = &g_dsmif0_channel_cfg0,
#endif
#ifndef DSMIF_CHANNEL_01
#define DSMIF_UNIT0_MASK_01   (0)
#define DSMIF_UNIT1_MASK_01   (0)
    .p_channel_cfgs[1]   = NULL,
#else
#define DSMIF_UNIT0_MASK_01   (DSMIF_CHANNEL_MASK_1)
#define DSMIF_UNIT1_MASK_01   (DSMIF_CHANNEL_MASK_4)
    .p_channel_cfgs[1]   = &g_dsmif0_channel_cfg1,
#endif
#ifndef DSMIF_CHANNEL_02
#define DSMIF_UNIT0_MASK_02   (0)
#define DSMIF_UNIT1_MASK_02   (0)
    .p_channel_cfgs[2]   = NULL,
#else
#define DSMIF_UNIT0_MASK_02   (DSMIF_CHANNEL_MASK_2)
#define DSMIF_UNIT1_MASK_02   (DSMIF_CHANNEL_MASK_5)
    .p_channel_cfgs[2]   = &g_dsmif0_channel_cfg2,
#endif
    .channel_mask        = (dsmif_channel_mask_t)DSMIF_MASK_UNIT_00,
};
const adc_cfg_t g_dsmif0_cfg =
{
    .unit                = 0,
    .mode                = ADC_MODE_SYNCHRONIZE_SCAN,
#if defined(VECTOR_NUMBER_DSMIF0_CDRUI)
    .scan_end_irq        = VECTOR_NUMBER_DSMIF0_CDRUI,
#else
    .scan_end_irq        = FSP_INVALID_VECTOR,
#endif
    .scan_end_ipl           = (BSP_IRQ_DISABLED),
    .p_callback          = NULL,
    .p_context           = NULL,
    .p_extend            = &g_dsmif0_cfg_extend,
};
/* Instance structure to use this module. */
const adc_instance_t g_dsmif0 =
{
    .p_ctrl        = &g_dsmif0_ctrl,
    .p_cfg         = &g_dsmif0_cfg,
    .p_api         = &g_adc_on_dsmif
};
mtu3_instance_ctrl_t g_timer10_ctrl;
#if 0
const mtu3_extended_uvw_cfg_t g_timer10_uvw_extend =
{
    .tgru_val                       = 0x0000,
    .tgrv_val                       = 0x0000,
    .tgrw_val                       = 0x0000,
    .mtu3_clk_div_u                 = MTU3_DIV_UVW_PCLKH_1,
    .mtu3_clk_div_v                 = MTU3_DIV_UVW_PCLKH_1,
    .mtu3_clk_div_w                 = MTU3_DIV_UVW_PCLKH_1,
    .output_pin_level_u             = MTU3_IO_PIN_LEVEL_UVW_NO_FUNC,
    .output_pin_level_v             = MTU3_IO_PIN_LEVEL_UVW_NO_FUNC,
    .output_pin_level_w             = MTU3_IO_PIN_LEVEL_UVW_NO_FUNC,
    .noise_filter_mtioc_setting_uvw = (mtu3_noise_filter_setting_uvw_t)(MTU3_NOISE_FILTER_UVW_DISABLE),
    .noise_filter_mtioc_clk_uvw     = MTU3_NOISE_FILTER_CLOCK_PCLKH_DIV_1,

    .capture_u_ipl                  = (BSP_IRQ_DISABLED),
    .capture_v_ipl                  = (BSP_IRQ_DISABLED),
    .capture_w_ipl                  = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_TGIU5)
    .capture_u_irq                  = VECTOR_NUMBER_TGIU5,
#else
    .capture_u_irq                  = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_TGIV5)
    .capture_v_irq                  = VECTOR_NUMBER_TGIV5,
#else
    .capture_v_irq                  = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_TGIW5)
    .capture_w_irq                  = VECTOR_NUMBER_TGIW5,
#else
    .capture_w_irq                  = FSP_INVALID_VECTOR,
#endif
};
const mtu3_extended_pwm_cfg_t g_timer10_pwm_extend =
{
    .interrupt_skip_mode_a          = MTU3_INTERRUPT_SKIP_MODE_1,
    .interrupt_skip_mode_b          = MTU3_INTERRUPT_SKIP_MODE_1,
    .adc_a_compare_match            = 0x0000,
    .adc_b_compare_match            = 0x0000,
    .interrupt_skip_count_tciv4     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgia3     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tciv7     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgia6     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgr4an_bn = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgr7an_bn = MTU3_INTERRUPT_SKIP_COUNT_0,
};
#endif
const mtu3_extended_cfg_t g_timer10_extend =
{
    .tgra_val            = 0x0000,
    .tgrb_val            = 0x0000,
    .tgrc_val            = 0x0000,
    .tgrd_val            = 0x0000,
    .mtu3_clk_div        = MTU3_DIV_PCLKH_1,
    .clk_edge            = MTU3_CLOCK_EDGE_RISING,
    .mtu3_clear          = MTU3_TCNT_CLEAR_DISABLE,
    .mtioc_ctrl_setting  = { .output_pin_level_a = MTU3_IO_PIN_LEVEL_NO_OUTPUT,
                             .output_pin_level_b = MTU3_IO_PIN_LEVEL_NO_OUTPUT
                           },
    .capture_a_ipl       = (BSP_IRQ_DISABLED),
    .capture_b_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_TGIA2)
    .capture_a_irq       = VECTOR_NUMBER_TGIA2,
#else
    .capture_a_irq       = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_TGIB2)
    .capture_b_irq       = VECTOR_NUMBER_TGIB2,
#else
    .capture_b_irq       = FSP_INVALID_VECTOR,
#endif
    .noise_filter_mtioc_setting  = (mtu3_noise_filter_setting_t)( MTU3_NOISE_FILTER_DISABLE),
    .noise_filter_mtioc_clk      = MTU3_NOISE_FILTER_CLOCK_PCLKH_DIV_1,
    .noise_filter_mtclk_setting  = (mtu3_noise_filter_mtclk_setting_t)( MTU3_NOISE_FILTER_EXTERNAL_DISABLE),
    .noise_filter_mtclk_clk      = MTU3_NOISE_FILTER_EXTERNAL_CLOCK_PCLKH_DIV_1,
#if 0
    .p_uvw_cfg                   = &g_timer10_uvw_extend,
    .p_pwm_cfg                   = &g_timer10_pwm_extend,
#else
    .p_uvw_cfg                   = NULL,
    .p_pwm_cfg                   = NULL,
#endif
};
const timer_cfg_t g_timer10_cfg =
{
    .mode                = TIMER_MODE_PERIODIC,
    .channel             = 2,
    .p_callback          = NULL,
    .p_context           = NULL,
    .p_extend            = &g_timer10_extend,
    .cycle_end_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_TCIV2)
    .cycle_end_irq       = VECTOR_NUMBER_TCIV2,
#else
    .cycle_end_irq       = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const timer_instance_t g_timer10 =
{
    .p_ctrl        = &g_timer10_ctrl,
    .p_cfg         = &g_timer10_cfg,
    .p_api         = &g_timer_on_mtu3
};
mtu3_instance_ctrl_t g_timer9_ctrl;
#if 0
const mtu3_extended_uvw_cfg_t g_timer9_uvw_extend =
{
    .tgru_val                       = 0x0000,
    .tgrv_val                       = 0x0000,
    .tgrw_val                       = 0x0000,
    .mtu3_clk_div_u                 = MTU3_DIV_UVW_PCLKH_1,
    .mtu3_clk_div_v                 = MTU3_DIV_UVW_PCLKH_1,
    .mtu3_clk_div_w                 = MTU3_DIV_UVW_PCLKH_1,
    .output_pin_level_u             = MTU3_IO_PIN_LEVEL_UVW_NO_FUNC,
    .output_pin_level_v             = MTU3_IO_PIN_LEVEL_UVW_NO_FUNC,
    .output_pin_level_w             = MTU3_IO_PIN_LEVEL_UVW_NO_FUNC,
    .noise_filter_mtioc_setting_uvw = (mtu3_noise_filter_setting_uvw_t)(MTU3_NOISE_FILTER_UVW_DISABLE),
    .noise_filter_mtioc_clk_uvw     = MTU3_NOISE_FILTER_CLOCK_PCLKH_DIV_1,

    .capture_u_ipl                  = (BSP_IRQ_DISABLED),
    .capture_v_ipl                  = (BSP_IRQ_DISABLED),
    .capture_w_ipl                  = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_TGIU5)
    .capture_u_irq                  = VECTOR_NUMBER_TGIU5,
#else
    .capture_u_irq                  = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_TGIV5)
    .capture_v_irq                  = VECTOR_NUMBER_TGIV5,
#else
    .capture_v_irq                  = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_TGIW5)
    .capture_w_irq                  = VECTOR_NUMBER_TGIW5,
#else
    .capture_w_irq                  = FSP_INVALID_VECTOR,
#endif
};
const mtu3_extended_pwm_cfg_t g_timer9_pwm_extend =
{
    .interrupt_skip_mode_a          = MTU3_INTERRUPT_SKIP_MODE_1,
    .interrupt_skip_mode_b          = MTU3_INTERRUPT_SKIP_MODE_1,
    .adc_a_compare_match            = 0x0000,
    .adc_b_compare_match            = 0x0000,
    .interrupt_skip_count_tciv4     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgia3     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tciv7     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgia6     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgr4an_bn = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgr7an_bn = MTU3_INTERRUPT_SKIP_COUNT_0,
};
#endif
const mtu3_extended_cfg_t g_timer9_extend =
{
    .tgra_val            = 0x0000,
    .tgrb_val            = 0x0000,
    .tgrc_val            = 0x0000,
    .tgrd_val            = 0x0000,
    .mtu3_clk_div        = MTU3_DIV_PCLKH_1,
    .clk_edge            = MTU3_CLOCK_EDGE_RISING,
    .mtu3_clear          = MTU3_TCNT_CLEAR_DISABLE,
    .mtioc_ctrl_setting  = { .output_pin_level_a = MTU3_IO_PIN_LEVEL_NO_OUTPUT,
                             .output_pin_level_b = MTU3_IO_PIN_LEVEL_NO_OUTPUT
                           },
    .capture_a_ipl       = (BSP_IRQ_DISABLED),
    .capture_b_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_TGIA2)
    .capture_a_irq       = VECTOR_NUMBER_TGIA2,
#else
    .capture_a_irq       = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_TGIB2)
    .capture_b_irq       = VECTOR_NUMBER_TGIB2,
#else
    .capture_b_irq       = FSP_INVALID_VECTOR,
#endif
    .noise_filter_mtioc_setting  = (mtu3_noise_filter_setting_t)( MTU3_NOISE_FILTER_DISABLE),
    .noise_filter_mtioc_clk      = MTU3_NOISE_FILTER_CLOCK_PCLKH_DIV_1,
    .noise_filter_mtclk_setting  = (mtu3_noise_filter_mtclk_setting_t)( MTU3_NOISE_FILTER_EXTERNAL_DISABLE),
    .noise_filter_mtclk_clk      = MTU3_NOISE_FILTER_EXTERNAL_CLOCK_PCLKH_DIV_1,
#if 0
    .p_uvw_cfg                   = &g_timer9_uvw_extend,
    .p_pwm_cfg                   = &g_timer9_pwm_extend,
#else
    .p_uvw_cfg                   = NULL,
    .p_pwm_cfg                   = NULL,
#endif
};
const timer_cfg_t g_timer9_cfg =
{
    .mode                = TIMER_MODE_PERIODIC,
    .channel             = 2,
    .p_callback          = NULL,
    .p_context           = NULL,
    .p_extend            = &g_timer9_extend,
    .cycle_end_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_TCIV2)
    .cycle_end_irq       = VECTOR_NUMBER_TCIV2,
#else
    .cycle_end_irq       = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const timer_instance_t g_timer9 =
{
    .p_ctrl        = &g_timer9_ctrl,
    .p_cfg         = &g_timer9_cfg,
    .p_api         = &g_timer_on_mtu3
};
mtu3_phase_count_instance_ctrl_t g_mtu_phase_count_ch2_ctrl;
const mtu3_phase_count_cfg_t g_mtu_phase_count_ch2_cfg =
{
    .counting_mode       = MTU3_PHASE_COUNTING_MODE_1,
    .bit_mode            = MTU3_BIT_MODE_NORMAL_16BIT,
    .external_clock      = MTU3_EXTERAL_CLOCK_MTCLKC_D,
    .channel             = 2,
    .p_timer_instance    = &g_timer9,
    .channel_mask        = (1 << 2),
    .p_context           = NULL,
    .p_extend            = NULL,
};
/* Instance structure to use this module. */
const mtu3_phase_count_instance_t g_mtu_phase_count_ch2 =
{
    .p_ctrl        = &g_mtu_phase_count_ch2_ctrl,
    .p_cfg         = &g_mtu_phase_count_ch2_cfg,
    .p_api         = &g_mtu3_phase_count_on_mtu3_phase_count
};
mtu3_instance_ctrl_t g_timer8_ctrl;
#if 1
const mtu3_extended_uvw_cfg_t g_timer8_uvw_extend =
{
    .tgru_val                       = 0x0000,
    .tgrv_val                       = 0x0000,
    .tgrw_val                       = 0x0000,
    .mtu3_clk_div_u                 = MTU3_DIV_UVW_PCLKH_1,
    .mtu3_clk_div_v                 = MTU3_DIV_UVW_PCLKH_1,
    .mtu3_clk_div_w                 = MTU3_DIV_UVW_PCLKH_1,
    .output_pin_level_u             = MTU3_IO_PIN_LEVEL_UVW_NO_FUNC,
    .output_pin_level_v             = MTU3_IO_PIN_LEVEL_UVW_NO_FUNC,
    .output_pin_level_w             = MTU3_IO_PIN_LEVEL_UVW_NO_FUNC,
    .noise_filter_mtioc_setting_uvw = (mtu3_noise_filter_setting_uvw_t)(MTU3_NOISE_FILTER_UVW_DISABLE),
    .noise_filter_mtioc_clk_uvw     = MTU3_NOISE_FILTER_CLOCK_PCLKH_DIV_1,

    .capture_u_ipl                  = (BSP_IRQ_DISABLED),
    .capture_v_ipl                  = (BSP_IRQ_DISABLED),
    .capture_w_ipl                  = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_TGIU5)
    .capture_u_irq                  = VECTOR_NUMBER_TGIU5,
#else
    .capture_u_irq                  = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_TGIV5)
    .capture_v_irq                  = VECTOR_NUMBER_TGIV5,
#else
    .capture_v_irq                  = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_TGIW5)
    .capture_w_irq                  = VECTOR_NUMBER_TGIW5,
#else
    .capture_w_irq                  = FSP_INVALID_VECTOR,
#endif
};
const mtu3_extended_pwm_cfg_t g_timer8_pwm_extend =
{
    .interrupt_skip_mode_a          = MTU3_INTERRUPT_SKIP_MODE_1,
    .interrupt_skip_mode_b          = MTU3_INTERRUPT_SKIP_MODE_1,
    .adc_a_compare_match            = 0x0000,
    .adc_b_compare_match            = 0x0000,
    .interrupt_skip_count_tciv4     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgia3     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tciv7     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgia6     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgr4an_bn = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgr7an_bn = MTU3_INTERRUPT_SKIP_COUNT_0,
};
#endif
const mtu3_extended_cfg_t g_timer8_extend =
{
    .tgra_val            = 0x0000,
    .tgrb_val            = 0x0000,
    .tgrc_val            = 0x0000,
    .tgrd_val            = 0x0000,
    .mtu3_clk_div        = MTU3_DIV_PCLKH_1,
    .clk_edge            = MTU3_CLOCK_EDGE_RISING,
    .mtu3_clear          = MTU3_TCNT_CLEAR_DISABLE,
    .mtioc_ctrl_setting  = { .output_pin_level_a = MTU3_IO_PIN_LEVEL_NO_OUTPUT,
                             .output_pin_level_b = MTU3_IO_PIN_LEVEL_NO_OUTPUT
                           },
    .capture_a_ipl       = (BSP_IRQ_DISABLED),
    .capture_b_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_TGIA2)
    .capture_a_irq       = VECTOR_NUMBER_TGIA2,
#else
    .capture_a_irq       = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_TGIB2)
    .capture_b_irq       = VECTOR_NUMBER_TGIB2,
#else
    .capture_b_irq       = FSP_INVALID_VECTOR,
#endif
    .noise_filter_mtioc_setting  = (mtu3_noise_filter_setting_t)( MTU3_NOISE_FILTER_DISABLE),
    .noise_filter_mtioc_clk      = MTU3_NOISE_FILTER_CLOCK_PCLKH_DIV_1,
    .noise_filter_mtclk_setting  = (mtu3_noise_filter_mtclk_setting_t)( MTU3_NOISE_FILTER_EXTERNAL_DISABLE),
    .noise_filter_mtclk_clk      = MTU3_NOISE_FILTER_EXTERNAL_CLOCK_PCLKH_DIV_1,
#if 1
    .p_uvw_cfg                   = &g_timer8_uvw_extend,
    .p_pwm_cfg                   = &g_timer8_pwm_extend,
#else
    .p_uvw_cfg                   = NULL,
    .p_pwm_cfg                   = NULL,
#endif
};
const timer_cfg_t g_timer8_cfg =
{
    .mode                = TIMER_MODE_PERIODIC,
    .channel             = 2,
    .p_callback          = NULL,
    .p_context           = NULL,
    .p_extend            = &g_timer8_extend,
    .cycle_end_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_TCIV2)
    .cycle_end_irq       = VECTOR_NUMBER_TCIV2,
#else
    .cycle_end_irq       = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const timer_instance_t g_timer8 =
{
    .p_ctrl        = &g_timer8_ctrl,
    .p_cfg         = &g_timer8_cfg,
    .p_api         = &g_timer_on_mtu3
};
mtu3_instance_ctrl_t g_timer7_ctrl;
#if 1
const mtu3_extended_uvw_cfg_t g_timer7_uvw_extend =
{
    .tgru_val                       = 0x0000,
    .tgrv_val                       = 0x0000,
    .tgrw_val                       = 0x0000,
    .mtu3_clk_div_u                 = MTU3_DIV_UVW_PCLKH_1,
    .mtu3_clk_div_v                 = MTU3_DIV_UVW_PCLKH_1,
    .mtu3_clk_div_w                 = MTU3_DIV_UVW_PCLKH_1,
    .output_pin_level_u             = MTU3_IO_PIN_LEVEL_UVW_NO_FUNC,
    .output_pin_level_v             = MTU3_IO_PIN_LEVEL_UVW_NO_FUNC,
    .output_pin_level_w             = MTU3_IO_PIN_LEVEL_UVW_NO_FUNC,
    .noise_filter_mtioc_setting_uvw = (mtu3_noise_filter_setting_uvw_t)(MTU3_NOISE_FILTER_UVW_DISABLE),
    .noise_filter_mtioc_clk_uvw     = MTU3_NOISE_FILTER_CLOCK_PCLKH_DIV_1,

    .capture_u_ipl                  = (BSP_IRQ_DISABLED),
    .capture_v_ipl                  = (BSP_IRQ_DISABLED),
    .capture_w_ipl                  = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_TGIU5)
    .capture_u_irq                  = VECTOR_NUMBER_TGIU5,
#else
    .capture_u_irq                  = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_TGIV5)
    .capture_v_irq                  = VECTOR_NUMBER_TGIV5,
#else
    .capture_v_irq                  = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_TGIW5)
    .capture_w_irq                  = VECTOR_NUMBER_TGIW5,
#else
    .capture_w_irq                  = FSP_INVALID_VECTOR,
#endif
};
const mtu3_extended_pwm_cfg_t g_timer7_pwm_extend =
{
    .interrupt_skip_mode_a          = MTU3_INTERRUPT_SKIP_MODE_1,
    .interrupt_skip_mode_b          = MTU3_INTERRUPT_SKIP_MODE_1,
    .adc_a_compare_match            = 0x0000,
    .adc_b_compare_match            = 0x0000,
    .interrupt_skip_count_tciv4     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgia3     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tciv7     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgia6     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgr4an_bn = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgr7an_bn = MTU3_INTERRUPT_SKIP_COUNT_0,
};
#endif
const mtu3_extended_cfg_t g_timer7_extend =
{
    .tgra_val            = 0x0000,
    .tgrb_val            = 0x0000,
    .tgrc_val            = 0x0000,
    .tgrd_val            = 0x0000,
    .mtu3_clk_div        = MTU3_DIV_PCLKH_1,
    .clk_edge            = MTU3_CLOCK_EDGE_RISING,
    .mtu3_clear          = MTU3_TCNT_CLEAR_DISABLE,
    .mtioc_ctrl_setting  = { .output_pin_level_a = MTU3_IO_PIN_LEVEL_NO_OUTPUT,
                             .output_pin_level_b = MTU3_IO_PIN_LEVEL_NO_OUTPUT
                           },
    .capture_a_ipl       = (BSP_IRQ_DISABLED),
    .capture_b_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_TGIA1)
    .capture_a_irq       = VECTOR_NUMBER_TGIA1,
#else
    .capture_a_irq       = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_TGIB1)
    .capture_b_irq       = VECTOR_NUMBER_TGIB1,
#else
    .capture_b_irq       = FSP_INVALID_VECTOR,
#endif
    .noise_filter_mtioc_setting  = (mtu3_noise_filter_setting_t)( MTU3_NOISE_FILTER_DISABLE),
    .noise_filter_mtioc_clk      = MTU3_NOISE_FILTER_CLOCK_PCLKH_DIV_1,
    .noise_filter_mtclk_setting  = (mtu3_noise_filter_mtclk_setting_t)( MTU3_NOISE_FILTER_EXTERNAL_DISABLE),
    .noise_filter_mtclk_clk      = MTU3_NOISE_FILTER_EXTERNAL_CLOCK_PCLKH_DIV_1,
#if 1
    .p_uvw_cfg                   = &g_timer7_uvw_extend,
    .p_pwm_cfg                   = &g_timer7_pwm_extend,
#else
    .p_uvw_cfg                   = NULL,
    .p_pwm_cfg                   = NULL,
#endif
};
const timer_cfg_t g_timer7_cfg =
{
    .mode                = TIMER_MODE_PERIODIC,
    .channel             = 1,
    .p_callback          = NULL,
    .p_context           = NULL,
    .p_extend            = &g_timer7_extend,
    .cycle_end_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_TCIV1)
    .cycle_end_irq       = VECTOR_NUMBER_TCIV1,
#else
    .cycle_end_irq       = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const timer_instance_t g_timer7 =
{
    .p_ctrl        = &g_timer7_ctrl,
    .p_cfg         = &g_timer7_cfg,
    .p_api         = &g_timer_on_mtu3
};
mtu3_phase_count_instance_ctrl_t g_mtu_phase_count_ch1_ctrl;
const mtu3_phase_count_cfg_t g_mtu_phase_count_ch1_cfg =
{
    .counting_mode       = MTU3_PHASE_COUNTING_MODE_1,
    .bit_mode            = MTU3_BIT_MODE_NORMAL_16BIT,
    .external_clock      = MTU3_EXTERAL_CLOCK_MTCLKA_B,
    .channel             = 1,
    .p_timer_instance    = &g_timer7,
    .channel_mask        = (1 << 1),
    .p_context           = NULL,
    .p_extend            = NULL,
};
/* Instance structure to use this module. */
const mtu3_phase_count_instance_t g_mtu_phase_count_ch1 =
{
    .p_ctrl        = &g_mtu_phase_count_ch1_ctrl,
    .p_cfg         = &g_mtu_phase_count_ch1_cfg,
    .p_api         = &g_mtu3_phase_count_on_mtu3_phase_count
};
adc_instance_ctrl_t g_adc1_ctrl;
const adc_extended_cfg_t g_adc1_cfg_extend =
{
    .add_average_count   = ADC_ADD_OFF,
    .clearing            = ADC_CLEAR_AFTER_READ_ON,
    .trigger_group_b     = ADC_TRIGGER_SYNC_ELC,
    .double_trigger_mode = ADC_DOUBLE_TRIGGER_DISABLED,
    .adc_start_trigger_a  = ADC_ACTIVE_TRIGGER_DISABLED,
    .adc_start_trigger_b  = ADC_ACTIVE_TRIGGER_DISABLED,
    .adc_start_trigger_c_enabled = 0,
    .adc_start_trigger_c  = ADC_ACTIVE_TRIGGER_DISABLED,
    .adc_elc_ctrl        = ADC_ELC_SINGLE_SCAN,
};
const adc_cfg_t g_adc1_cfg =
{
    .unit                = 1,
    .mode                = ADC_MODE_SINGLE_SCAN,
    .resolution          = ADC_RESOLUTION_12_BIT,
    .alignment           = (adc_alignment_t)ADC_ALIGNMENT_RIGHT,
    .trigger             = ADC_TRIGGER_SOFTWARE,
    .p_callback          = adc1_sample_callback,
    .p_context           = NULL,
    .p_extend            = &g_adc1_cfg_extend,
#if defined(VECTOR_NUMBER_ADC1_ADI)
    .scan_end_irq        = VECTOR_NUMBER_ADC1_ADI,
#else
    .scan_end_irq        = FSP_INVALID_VECTOR,
#endif
    .scan_end_ipl        = (7),
#if defined(VECTOR_NUMBER_ADC1_GBADI)
    .scan_end_b_irq      = VECTOR_NUMBER_ADC1_GBADI,
#else
    .scan_end_b_irq      = FSP_INVALID_VECTOR,
#endif
    .scan_end_b_ipl      = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_ADC1_GCADI)
    .scan_end_c_irq      = VECTOR_NUMBER_ADC1_GCADI,
#else
    .scan_end_c_irq      = FSP_INVALID_VECTOR,
#endif
    .scan_end_c_ipl      = (BSP_IRQ_DISABLED),
};
const adc_channel_cfg_t g_adc1_channel_cfg =
{
    .scan_mask           = ADC_MASK_CHANNEL_0 | ADC_MASK_CHANNEL_1 | ADC_MASK_CHANNEL_2 | ADC_MASK_CHANNEL_3 | ADC_MASK_CHANNEL_4 | ADC_MASK_CHANNEL_5 | ADC_MASK_CHANNEL_6 | ADC_MASK_CHANNEL_7 |  0,
    .scan_mask_group_b   =  0,
    .priority_group_a    = ADC_GROUP_A_PRIORITY_OFF,
    .add_mask            =  0,
    .sample_hold_mask    =  0,
    .sample_hold_states  = 24,
    .scan_mask_group_c   =  0,
};
/* Instance structure to use this module. */
const adc_instance_t g_adc1 =
{
    .p_ctrl    = &g_adc1_ctrl,
    .p_cfg     = &g_adc1_cfg,
    .p_channel_cfg = &g_adc1_channel_cfg,
    .p_api     = &g_adc_on_adc
};
adc_instance_ctrl_t g_adc0_ctrl;
const adc_extended_cfg_t g_adc0_cfg_extend =
{
    .add_average_count   = ADC_ADD_OFF,
    .clearing            = ADC_CLEAR_AFTER_READ_ON,
    .trigger_group_b     = ADC_TRIGGER_SYNC_ELC,
    .double_trigger_mode = ADC_DOUBLE_TRIGGER_DISABLED,
    .adc_start_trigger_a  = ADC_ACTIVE_TRIGGER_DISABLED,
    .adc_start_trigger_b  = ADC_ACTIVE_TRIGGER_DISABLED,
    .adc_start_trigger_c_enabled = 0,
    .adc_start_trigger_c  = ADC_ACTIVE_TRIGGER_DISABLED,
    .adc_elc_ctrl        = ADC_ELC_SINGLE_SCAN,
};
const adc_cfg_t g_adc0_cfg =
{
    .unit                = 0,
    .mode                = ADC_MODE_SINGLE_SCAN,
    .resolution          = ADC_RESOLUTION_12_BIT,
    .alignment           = (adc_alignment_t)ADC_ALIGNMENT_RIGHT,
    .trigger             = ADC_TRIGGER_SOFTWARE,
    .p_callback          = adc0_sample_callback,
    .p_context           = NULL,
    .p_extend            = &g_adc0_cfg_extend,
#if defined(VECTOR_NUMBER_ADC0_ADI)
    .scan_end_irq        = VECTOR_NUMBER_ADC0_ADI,
#else
    .scan_end_irq        = FSP_INVALID_VECTOR,
#endif
    .scan_end_ipl        = (7),
#if defined(VECTOR_NUMBER_ADC0_GBADI)
    .scan_end_b_irq      = VECTOR_NUMBER_ADC0_GBADI,
#else
    .scan_end_b_irq      = FSP_INVALID_VECTOR,
#endif
    .scan_end_b_ipl      = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_ADC0_GCADI)
    .scan_end_c_irq      = VECTOR_NUMBER_ADC0_GCADI,
#else
    .scan_end_c_irq      = FSP_INVALID_VECTOR,
#endif
    .scan_end_c_ipl      = (BSP_IRQ_DISABLED),
};
const adc_channel_cfg_t g_adc0_channel_cfg =
{
    .scan_mask           = ADC_MASK_CHANNEL_0 | ADC_MASK_CHANNEL_1 | ADC_MASK_CHANNEL_2 | ADC_MASK_CHANNEL_3 | ADC_MASK_CHANNEL_4 | ADC_MASK_CHANNEL_5 | ADC_MASK_CHANNEL_6 | ADC_MASK_CHANNEL_7 |  0,
    .scan_mask_group_b   =  0,
    .priority_group_a    = ADC_GROUP_A_PRIORITY_OFF,
    .add_mask            =  0,
    .sample_hold_mask    =  0,
    .sample_hold_states  = 24,
    .scan_mask_group_c   =  0,
};
/* Instance structure to use this module. */
const adc_instance_t g_adc0 =
{
    .p_ctrl    = &g_adc0_ctrl,
    .p_cfg     = &g_adc0_cfg,
    .p_channel_cfg = &g_adc0_channel_cfg,
    .p_api     = &g_adc_on_adc
};
poe3_instance_ctrl_t g_mtu3_three_phase_poe_ctrl;
/** POE3 setting. */
const poe3_cfg_t g_mtu3_three_phase_poe_cfg =
{
    .poe0                  = {
                                 .mode                  = POE3_HIZ_MODE_FALLING_EDGE,
                                 .interrupt             = POE3_INTERRUPT_ENABLE_DISABLED,
                                 .mtioc3b_mtioc3d       = {
                                                              .mtioc3b_pin_select   = POE3_MTIOC3B_PIN_SELECT_P17_6,
                                                              .mtioc3d_pin_select   = POE3_MTIOC3D_PIN_SELECT_P18_1,
                                                              .mtioc3b_active_level = POE3_ACTIVE_LEVEL_LOW,
                                                              .mtioc3d_active_level = POE3_ACTIVE_LEVEL_LOW,
                                                              .hiz_output           = POE3_HIZ_OUTPUT_ENABLE_ENABLED
                                                          },
                                 .mtioc4b_mtioc4d       = {
                                                              .mtioc4b_pin_select   = POE3_MTIOC4B_PIN_SELECT_P18_2,
                                                              .mtioc4d_pin_select   = POE3_MTIOC4D_PIN_SELECT_P18_3,
                                                              .mtioc4b_active_level = POE3_ACTIVE_LEVEL_LOW,
                                                              .mtioc4d_active_level = POE3_ACTIVE_LEVEL_LOW,
                                                              .hiz_output           = POE3_HIZ_OUTPUT_ENABLE_ENABLED
                                                          },
                                 .mtioc4a_mtioc4c       = {
                                                              .mtioc4a_pin_select   = POE3_MTIOC4A_PIN_SELECT_P17_7,
                                                              .mtioc4c_pin_select   = POE3_MTIOC4C_PIN_SELECT_P18_0,
                                                              .mtioc4a_active_level = POE3_ACTIVE_LEVEL_LOW,
                                                              .mtioc4c_active_level = POE3_ACTIVE_LEVEL_LOW,
                                                              .hiz_output           = POE3_HIZ_OUTPUT_ENABLE_ENABLED
                                                          }
                             },
    .poe4                  = {
                                 .mode                  = POE3_HIZ_MODE_FALLING_EDGE,
                                 .interrupt             = POE3_INTERRUPT_ENABLE_DISABLED,
                                 .mtioc6b_mtioc6d       = {
                                                              .mtioc6b_pin_select   = POE3_MTIOC6B_PIN_SELECT_P19_3,
                                                              .mtioc6d_pin_select   = POE3_MTIOC6D_PIN_SELECT_P19_6,
                                                              .hiz_output           = POE3_HIZ_OUTPUT_ENABLE_ENABLED
                                                          },
                                 .mtioc7b_mtioc7d       = {
                                                              .mtioc7b_pin_select   = POE3_MTIOC7B_PIN_SELECT_P19_7,
                                                              .mtioc7d_pin_select   = POE3_MTIOC7D_PIN_SELECT_P20_0,
                                                              .hiz_output           = POE3_HIZ_OUTPUT_ENABLE_ENABLED
                                                          },
                                 .mtioc7a_mtioc7c       = {
                                                              .mtioc7a_pin_select   = POE3_MTIOC7A_PIN_SELECT_P19_4,
                                                              .mtioc7c_pin_select   = POE3_MTIOC7C_PIN_SELECT_P19_5,
                                                              .hiz_output           = POE3_HIZ_OUTPUT_ENABLE_ENABLED
                                                          }
                             },
    .poe8                  = {
                                 .mode                  = POE3_HIZ_MODE_FALLING_EDGE,
                                 .interrupt             = POE3_INTERRUPT_ENABLE_DISABLED,
                                 .mtioc0a               = {
                                                              .pin_select           = POE3_MTIOC0A_PIN_SELECT_P13_2,
                                                              .hiz_output           = POE3_HIZ_OUTPUT_ENABLE_DISABLED
                                                          },
                                 .mtioc0b               = {
                                                              .pin_select           = POE3_MTIOC0B_PIN_SELECT_P11_5,
                                                              .hiz_output           = POE3_HIZ_OUTPUT_ENABLE_DISABLED
                                                          },
                                 .mtioc0c               = {
                                                              .pin_select           = POE3_MTIOC0C_PIN_SELECT_P13_3,
                                                              .hiz_output           = POE3_HIZ_OUTPUT_ENABLE_DISABLED
                                                          },
                                 .mtioc0d               = {
                                                              .pin_select           = POE3_MTIOC0D_PIN_SELECT_P13_4,
                                                              .hiz_output           = POE3_HIZ_OUTPUT_ENABLE_DISABLED
                                                          }
                             },
    .oscillation_stop      = POE3_HIZ_OUTPUT_ENABLE_DISABLED,
#if BSP_FEATURE_POE3_ERROR_SIGNAL_TYPE == 2
    .dsmif0_error_1        = POE3_HIZ_OUTPUT_ENABLE_DISABLED,
    .dsmif1_error_1        = POE3_HIZ_OUTPUT_ENABLE_DISABLED,
#endif
    .dsmif0_error          = POE3_HIZ_OUTPUT_ENABLE_DISABLED,
    .dsmif1_error          = POE3_HIZ_OUTPUT_ENABLE_DISABLED,
    .short_circuit1        = { .interrupt  = POE3_INTERRUPT_ENABLE_DISABLED,
                               .hiz_output = POE3_HIZ_OUTPUT_ENABLE_DISABLED
                             },
    .short_circuit2        = { .interrupt  = POE3_INTERRUPT_ENABLE_DISABLED,
                               .hiz_output = POE3_HIZ_OUTPUT_ENABLE_DISABLED
                             },
    .p_context             = NULL,
};
/* Instance structure to use this module. */
const poe3_instance_t g_mtu3_three_phase_poe =
{
    .p_ctrl                = &g_mtu3_three_phase_poe_ctrl,
    .p_cfg                 = &g_mtu3_three_phase_poe_cfg,
    .p_api                 = &g_poe30_on_poe3
};
mtu3_instance_ctrl_t g_timer6_ctrl;
#if 1
const mtu3_extended_uvw_cfg_t g_timer6_uvw_extend =
{
    .tgru_val                       = 0x0000,
    .tgrv_val                       = 0x0000,
    .tgrw_val                       = 0x0000,
    .mtu3_clk_div_u                 = MTU3_DIV_UVW_PCLKH_1,
    .mtu3_clk_div_v                 = MTU3_DIV_UVW_PCLKH_1,
    .mtu3_clk_div_w                 = MTU3_DIV_UVW_PCLKH_1,
    .output_pin_level_u             = MTU3_IO_PIN_LEVEL_UVW_NO_FUNC,
    .output_pin_level_v             = MTU3_IO_PIN_LEVEL_UVW_NO_FUNC,
    .output_pin_level_w             = MTU3_IO_PIN_LEVEL_UVW_NO_FUNC,
    .noise_filter_mtioc_setting_uvw = (mtu3_noise_filter_setting_uvw_t)(MTU3_NOISE_FILTER_UVW_DISABLE),
    .noise_filter_mtioc_clk_uvw     = MTU3_NOISE_FILTER_CLOCK_PCLKH_DIV_1,

    .capture_u_ipl                  = (BSP_IRQ_DISABLED),
    .capture_v_ipl                  = (BSP_IRQ_DISABLED),
    .capture_w_ipl                  = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_TGIU5)
    .capture_u_irq                  = VECTOR_NUMBER_TGIU5,
#else
    .capture_u_irq                  = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_TGIV5)
    .capture_v_irq                  = VECTOR_NUMBER_TGIV5,
#else
    .capture_v_irq                  = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_TGIW5)
    .capture_w_irq                  = VECTOR_NUMBER_TGIW5,
#else
    .capture_w_irq                  = FSP_INVALID_VECTOR,
#endif
};
const mtu3_extended_pwm_cfg_t g_timer6_pwm_extend =
{
    .interrupt_skip_mode_a          = MTU3_INTERRUPT_SKIP_MODE_1,
    .interrupt_skip_mode_b          = MTU3_INTERRUPT_SKIP_MODE_1,
    .adc_a_compare_match            = 0x0000,
    .adc_b_compare_match            = 0x0000,
    .interrupt_skip_count_tciv4     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgia3     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tciv7     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgia6     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgr4an_bn = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgr7an_bn = MTU3_INTERRUPT_SKIP_COUNT_0,
};
#endif
const mtu3_extended_cfg_t g_timer6_extend =
{
    .tgra_val            = 0,
    .tgrb_val            = 0,
    .tgrc_val            = 0,
    .tgrd_val            = 0,
    .mtu3_clk_div        = MTU3_DIV_PCLKH_4,
    .clk_edge            = MTU3_CLOCK_EDGE_RISING,
    .mtu3_clear          = MTU3_TCNT_CLEAR_DISABLE,
    .mtioc_ctrl_setting  = { .output_pin_level_a = MTU3_IO_PIN_LEVEL_NO_OUTPUT,
                             .output_pin_level_b = MTU3_IO_PIN_LEVEL_NO_OUTPUT
                           },
    .capture_a_ipl       = (BSP_IRQ_DISABLED),
    .capture_b_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_TGIA7)
    .capture_a_irq       = VECTOR_NUMBER_TGIA7,
#else
    .capture_a_irq       = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_TGIB7)
    .capture_b_irq       = VECTOR_NUMBER_TGIB7,
#else
    .capture_b_irq       = FSP_INVALID_VECTOR,
#endif
    .noise_filter_mtioc_setting  = (mtu3_noise_filter_setting_t)( MTU3_NOISE_FILTER_DISABLE),
    .noise_filter_mtioc_clk      = MTU3_NOISE_FILTER_CLOCK_PCLKH_DIV_1,
    .noise_filter_mtclk_setting  = (mtu3_noise_filter_mtclk_setting_t)( MTU3_NOISE_FILTER_EXTERNAL_DISABLE),
    .noise_filter_mtclk_clk      = MTU3_NOISE_FILTER_EXTERNAL_CLOCK_PCLKH_DIV_1,
#if 1
    .p_uvw_cfg                   = &g_timer6_uvw_extend,
    .p_pwm_cfg                   = &g_timer6_pwm_extend,
#else
    .p_uvw_cfg                   = NULL,
    .p_pwm_cfg                   = NULL,
#endif
};
const timer_cfg_t g_timer6_cfg =
{
    .mode                = TIMER_MODE_PERIODIC,
    .channel             = 7,
    .p_callback          = NULL,
    .p_context           = NULL,
    .p_extend            = &g_timer6_extend,
    .cycle_end_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_TCIV7)
    .cycle_end_irq       = VECTOR_NUMBER_TCIV7,
#else
    .cycle_end_irq       = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const timer_instance_t g_timer6 =
{
    .p_ctrl        = &g_timer6_ctrl,
    .p_cfg         = &g_timer6_cfg,
    .p_api         = &g_timer_on_mtu3
};
mtu3_instance_ctrl_t g_timer5_ctrl;
#if 1
const mtu3_extended_uvw_cfg_t g_timer5_uvw_extend =
{
    .tgru_val                       = 0x0000,
    .tgrv_val                       = 0x0000,
    .tgrw_val                       = 0x0000,
    .mtu3_clk_div_u                 = MTU3_DIV_UVW_PCLKH_1,
    .mtu3_clk_div_v                 = MTU3_DIV_UVW_PCLKH_1,
    .mtu3_clk_div_w                 = MTU3_DIV_UVW_PCLKH_1,
    .output_pin_level_u             = MTU3_IO_PIN_LEVEL_UVW_NO_FUNC,
    .output_pin_level_v             = MTU3_IO_PIN_LEVEL_UVW_NO_FUNC,
    .output_pin_level_w             = MTU3_IO_PIN_LEVEL_UVW_NO_FUNC,
    .noise_filter_mtioc_setting_uvw = (mtu3_noise_filter_setting_uvw_t)(MTU3_NOISE_FILTER_UVW_DISABLE),
    .noise_filter_mtioc_clk_uvw     = MTU3_NOISE_FILTER_CLOCK_PCLKH_DIV_1,

    .capture_u_ipl                  = (BSP_IRQ_DISABLED),
    .capture_v_ipl                  = (BSP_IRQ_DISABLED),
    .capture_w_ipl                  = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_TGIU5)
    .capture_u_irq                  = VECTOR_NUMBER_TGIU5,
#else
    .capture_u_irq                  = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_TGIV5)
    .capture_v_irq                  = VECTOR_NUMBER_TGIV5,
#else
    .capture_v_irq                  = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_TGIW5)
    .capture_w_irq                  = VECTOR_NUMBER_TGIW5,
#else
    .capture_w_irq                  = FSP_INVALID_VECTOR,
#endif
};
const mtu3_extended_pwm_cfg_t g_timer5_pwm_extend =
{
    .interrupt_skip_mode_a          = MTU3_INTERRUPT_SKIP_MODE_1,
    .interrupt_skip_mode_b          = MTU3_INTERRUPT_SKIP_MODE_1,
    .adc_a_compare_match            = 0x0000,
    .adc_b_compare_match            = 0x0000,
    .interrupt_skip_count_tciv4     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgia3     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tciv7     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgia6     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgr4an_bn = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgr7an_bn = MTU3_INTERRUPT_SKIP_COUNT_0,
};
#endif
const mtu3_extended_cfg_t g_timer5_extend =
{
    .tgra_val            = 0xa28,
    .tgrb_val            = 0,
    .tgrc_val            = 0xa28,
    .tgrd_val            = 0,
    .mtu3_clk_div        = MTU3_DIV_PCLKH_4,
    .clk_edge            = MTU3_CLOCK_EDGE_RISING,
    .mtu3_clear          = MTU3_TCNT_CLEAR_DISABLE,
    .mtioc_ctrl_setting  = { .output_pin_level_a = MTU3_IO_PIN_LEVEL_NO_OUTPUT,
                             .output_pin_level_b = MTU3_IO_PIN_LEVEL_NO_OUTPUT
                           },
    .capture_a_ipl       = (BSP_IRQ_DISABLED),
    .capture_b_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_TGIA6)
    .capture_a_irq       = VECTOR_NUMBER_TGIA6,
#else
    .capture_a_irq       = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_TGIB6)
    .capture_b_irq       = VECTOR_NUMBER_TGIB6,
#else
    .capture_b_irq       = FSP_INVALID_VECTOR,
#endif
    .noise_filter_mtioc_setting  = (mtu3_noise_filter_setting_t)( MTU3_NOISE_FILTER_DISABLE),
    .noise_filter_mtioc_clk      = MTU3_NOISE_FILTER_CLOCK_PCLKH_DIV_1,
    .noise_filter_mtclk_setting  = (mtu3_noise_filter_mtclk_setting_t)( MTU3_NOISE_FILTER_EXTERNAL_DISABLE),
    .noise_filter_mtclk_clk      = MTU3_NOISE_FILTER_EXTERNAL_CLOCK_PCLKH_DIV_1,
#if 1
    .p_uvw_cfg                   = &g_timer5_uvw_extend,
    .p_pwm_cfg                   = &g_timer5_pwm_extend,
#else
    .p_uvw_cfg                   = NULL,
    .p_pwm_cfg                   = NULL,
#endif
};
const timer_cfg_t g_timer5_cfg =
{
    .mode                = TIMER_MODE_PERIODIC,
    .channel             = 6,
    .p_callback          = NULL,
    .p_context           = NULL,
    .p_extend            = &g_timer5_extend,
    .cycle_end_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_TCIV6)
    .cycle_end_irq       = VECTOR_NUMBER_TCIV6,
#else
    .cycle_end_irq       = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const timer_instance_t g_timer5 =
{
    .p_ctrl        = &g_timer5_ctrl,
    .p_cfg         = &g_timer5_cfg,
    .p_api         = &g_timer_on_mtu3
};
mtu3_three_phase_instance_ctrl_t g_mtu3_m1_3ph_drv_ctrl;
const mtu3_three_phase_extended_cfg_t g_mtu3_m1_3ph_drv_extend =
{
    .pwm_mode    = MTU3_THREE_PHASE_PWM_MODE_1,
    .period_half = 0x1388 / 2,
    .dead_time   = 0x64,
    .duty_val    =
    {
        0,
        0,
        0
    },
    .period_out  = MTU3_THREE_PHASE_PSYE_TOGGLE,
    .level       = MTU3_THREE_PHASE_OUTPUT_LEVEL_ACTIVE_HIGH,
    .duty_double =
    {
        0,
        0,
        0
    },
    .sync        = MTU3_THREE_PHASE_SYNCHRONOUS_CHANNEL_DISABLE
};
const three_phase_cfg_t g_mtu3_m1_3ph_drv_cfg =
{
    .buffer_mode         = THREE_PHASE_BUFFER_MODE_SINGLE,
    .p_timer_instance    =
    {
        &g_timer5,
        &g_timer6
    },
    .channel_mask        = (1 << 6) | (1 << 7),
    .p_context           = NULL,
    .p_extend            = &g_mtu3_m1_3ph_drv_extend,
};
/* Instance structure to use this module. */
const three_phase_instance_t g_mtu3_m1_3ph_drv =
{
    .p_ctrl        = &g_mtu3_m1_3ph_drv_ctrl,
    .p_cfg         = &g_mtu3_m1_3ph_drv_cfg,
    .p_api         = &g_three_phase_on_mtu3_three_phase
};
mtu3_instance_ctrl_t g_timer4_ctrl;
#if 1
const mtu3_extended_uvw_cfg_t g_timer4_uvw_extend =
{
    .tgru_val                       = 0x0000,
    .tgrv_val                       = 0x0000,
    .tgrw_val                       = 0x0000,
    .mtu3_clk_div_u                 = MTU3_DIV_UVW_PCLKH_1,
    .mtu3_clk_div_v                 = MTU3_DIV_UVW_PCLKH_1,
    .mtu3_clk_div_w                 = MTU3_DIV_UVW_PCLKH_1,
    .output_pin_level_u             = MTU3_IO_PIN_LEVEL_UVW_NO_FUNC,
    .output_pin_level_v             = MTU3_IO_PIN_LEVEL_UVW_NO_FUNC,
    .output_pin_level_w             = MTU3_IO_PIN_LEVEL_UVW_NO_FUNC,
    .noise_filter_mtioc_setting_uvw = (mtu3_noise_filter_setting_uvw_t)(MTU3_NOISE_FILTER_UVW_DISABLE),
    .noise_filter_mtioc_clk_uvw     = MTU3_NOISE_FILTER_CLOCK_PCLKH_DIV_1,

    .capture_u_ipl                  = (BSP_IRQ_DISABLED),
    .capture_v_ipl                  = (BSP_IRQ_DISABLED),
    .capture_w_ipl                  = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_TGIU5)
    .capture_u_irq                  = VECTOR_NUMBER_TGIU5,
#else
    .capture_u_irq                  = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_TGIV5)
    .capture_v_irq                  = VECTOR_NUMBER_TGIV5,
#else
    .capture_v_irq                  = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_TGIW5)
    .capture_w_irq                  = VECTOR_NUMBER_TGIW5,
#else
    .capture_w_irq                  = FSP_INVALID_VECTOR,
#endif
};
const mtu3_extended_pwm_cfg_t g_timer4_pwm_extend =
{
    .interrupt_skip_mode_a          = MTU3_INTERRUPT_SKIP_MODE_1,
    .interrupt_skip_mode_b          = MTU3_INTERRUPT_SKIP_MODE_1,
    .adc_a_compare_match            = 0x0000,
    .adc_b_compare_match            = 0x0000,
    .interrupt_skip_count_tciv4     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgia3     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tciv7     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgia6     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgr4an_bn = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgr7an_bn = MTU3_INTERRUPT_SKIP_COUNT_0,
};
#endif
const mtu3_extended_cfg_t g_timer4_extend =
{
    .tgra_val            = 0,
    .tgrb_val            = 0,
    .tgrc_val            = 0,
    .tgrd_val            = 0,
    .mtu3_clk_div        = MTU3_DIV_PCLKH_4,
    .clk_edge            = MTU3_CLOCK_EDGE_RISING,
    .mtu3_clear          = MTU3_TCNT_CLEAR_DISABLE,
    .mtioc_ctrl_setting  = { .output_pin_level_a = MTU3_IO_PIN_LEVEL_NO_OUTPUT,
                             .output_pin_level_b = MTU3_IO_PIN_LEVEL_NO_OUTPUT
                           },
    .capture_a_ipl       = (BSP_IRQ_DISABLED),
    .capture_b_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_TGIA4)
    .capture_a_irq       = VECTOR_NUMBER_TGIA4,
#else
    .capture_a_irq       = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_TGIB4)
    .capture_b_irq       = VECTOR_NUMBER_TGIB4,
#else
    .capture_b_irq       = FSP_INVALID_VECTOR,
#endif
    .noise_filter_mtioc_setting  = (mtu3_noise_filter_setting_t)( MTU3_NOISE_FILTER_DISABLE),
    .noise_filter_mtioc_clk      = MTU3_NOISE_FILTER_CLOCK_PCLKH_DIV_1,
    .noise_filter_mtclk_setting  = (mtu3_noise_filter_mtclk_setting_t)( MTU3_NOISE_FILTER_EXTERNAL_DISABLE),
    .noise_filter_mtclk_clk      = MTU3_NOISE_FILTER_EXTERNAL_CLOCK_PCLKH_DIV_1,
#if 1
    .p_uvw_cfg                   = &g_timer4_uvw_extend,
    .p_pwm_cfg                   = &g_timer4_pwm_extend,
#else
    .p_uvw_cfg                   = NULL,
    .p_pwm_cfg                   = NULL,
#endif
};
const timer_cfg_t g_timer4_cfg =
{
    .mode                = TIMER_MODE_PERIODIC,
    .channel             = 4,
    .p_callback          = NULL,
    .p_context           = NULL,
    .p_extend            = &g_timer4_extend,
    .cycle_end_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_TCIV4)
    .cycle_end_irq       = VECTOR_NUMBER_TCIV4,
#else
    .cycle_end_irq       = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const timer_instance_t g_timer4 =
{
    .p_ctrl        = &g_timer4_ctrl,
    .p_cfg         = &g_timer4_cfg,
    .p_api         = &g_timer_on_mtu3
};
mtu3_instance_ctrl_t g_timer3_ctrl;
#if 1
const mtu3_extended_uvw_cfg_t g_timer3_uvw_extend =
{
    .tgru_val                       = 0x0000,
    .tgrv_val                       = 0x0000,
    .tgrw_val                       = 0x0000,
    .mtu3_clk_div_u                 = MTU3_DIV_UVW_PCLKH_1,
    .mtu3_clk_div_v                 = MTU3_DIV_UVW_PCLKH_1,
    .mtu3_clk_div_w                 = MTU3_DIV_UVW_PCLKH_1,
    .output_pin_level_u             = MTU3_IO_PIN_LEVEL_UVW_NO_FUNC,
    .output_pin_level_v             = MTU3_IO_PIN_LEVEL_UVW_NO_FUNC,
    .output_pin_level_w             = MTU3_IO_PIN_LEVEL_UVW_NO_FUNC,
    .noise_filter_mtioc_setting_uvw = (mtu3_noise_filter_setting_uvw_t)(MTU3_NOISE_FILTER_UVW_DISABLE),
    .noise_filter_mtioc_clk_uvw     = MTU3_NOISE_FILTER_CLOCK_PCLKH_DIV_1,

    .capture_u_ipl                  = (BSP_IRQ_DISABLED),
    .capture_v_ipl                  = (BSP_IRQ_DISABLED),
    .capture_w_ipl                  = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_TGIU5)
    .capture_u_irq                  = VECTOR_NUMBER_TGIU5,
#else
    .capture_u_irq                  = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_TGIV5)
    .capture_v_irq                  = VECTOR_NUMBER_TGIV5,
#else
    .capture_v_irq                  = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_TGIW5)
    .capture_w_irq                  = VECTOR_NUMBER_TGIW5,
#else
    .capture_w_irq                  = FSP_INVALID_VECTOR,
#endif
};
const mtu3_extended_pwm_cfg_t g_timer3_pwm_extend =
{
    .interrupt_skip_mode_a          = MTU3_INTERRUPT_SKIP_MODE_1,
    .interrupt_skip_mode_b          = MTU3_INTERRUPT_SKIP_MODE_1,
    .adc_a_compare_match            = 0x0000,
    .adc_b_compare_match            = 0x0000,
    .interrupt_skip_count_tciv4     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgia3     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tciv7     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgia6     = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgr4an_bn = MTU3_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_count_tgr7an_bn = MTU3_INTERRUPT_SKIP_COUNT_0,
};
#endif
const mtu3_extended_cfg_t g_timer3_extend =
{
    .tgra_val            = 0xa28,
    .tgrb_val            = 0,
    .tgrc_val            = 0xa28,
    .tgrd_val            = 0,
    .mtu3_clk_div        = MTU3_DIV_PCLKH_4,
    .clk_edge            = MTU3_CLOCK_EDGE_RISING,
    .mtu3_clear          = MTU3_TCNT_CLEAR_DISABLE,
    .mtioc_ctrl_setting  = { .output_pin_level_a = MTU3_IO_PIN_LEVEL_NO_OUTPUT,
                             .output_pin_level_b = MTU3_IO_PIN_LEVEL_NO_OUTPUT
                           },
    .capture_a_ipl       = (BSP_IRQ_DISABLED),
    .capture_b_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_TGIA3)
    .capture_a_irq       = VECTOR_NUMBER_TGIA3,
#else
    .capture_a_irq       = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_TGIB3)
    .capture_b_irq       = VECTOR_NUMBER_TGIB3,
#else
    .capture_b_irq       = FSP_INVALID_VECTOR,
#endif
    .noise_filter_mtioc_setting  = (mtu3_noise_filter_setting_t)( MTU3_NOISE_FILTER_DISABLE),
    .noise_filter_mtioc_clk      = MTU3_NOISE_FILTER_CLOCK_PCLKH_DIV_1,
    .noise_filter_mtclk_setting  = (mtu3_noise_filter_mtclk_setting_t)( MTU3_NOISE_FILTER_EXTERNAL_DISABLE),
    .noise_filter_mtclk_clk      = MTU3_NOISE_FILTER_EXTERNAL_CLOCK_PCLKH_DIV_1,
#if 1
    .p_uvw_cfg                   = &g_timer3_uvw_extend,
    .p_pwm_cfg                   = &g_timer3_pwm_extend,
#else
    .p_uvw_cfg                   = NULL,
    .p_pwm_cfg                   = NULL,
#endif
};
const timer_cfg_t g_timer3_cfg =
{
    .mode                = TIMER_MODE_PERIODIC,
    .channel             = 3,
    .p_callback          = NULL,
    .p_context           = NULL,
    .p_extend            = &g_timer3_extend,
    .cycle_end_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_TCIV3)
    .cycle_end_irq       = VECTOR_NUMBER_TCIV3,
#else
    .cycle_end_irq       = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const timer_instance_t g_timer3 =
{
    .p_ctrl        = &g_timer3_ctrl,
    .p_cfg         = &g_timer3_cfg,
    .p_api         = &g_timer_on_mtu3
};
mtu3_three_phase_instance_ctrl_t g_mtu3_m0_3ph_drv_ctrl;
const mtu3_three_phase_extended_cfg_t g_mtu3_m0_3ph_drv_extend =
{
    .pwm_mode    = MTU3_THREE_PHASE_PWM_MODE_1,
    .period_half = 0x1388 / 2,
    .dead_time   = 0x64,
    .duty_val    =
    {
        0,
        0,
        0
    },
    .period_out  = MTU3_THREE_PHASE_PSYE_TOGGLE,
    .level       = MTU3_THREE_PHASE_OUTPUT_LEVEL_ACTIVE_HIGH,
    .duty_double =
    {
        0,
        0,
        0
    },
    .sync        = MTU3_THREE_PHASE_SYNCHRONOUS_CHANNEL_DISABLE
};
const three_phase_cfg_t g_mtu3_m0_3ph_drv_cfg =
{
    .buffer_mode         = THREE_PHASE_BUFFER_MODE_SINGLE,
    .p_timer_instance    =
    {
        &g_timer3,
        &g_timer4
    },
    .channel_mask        = (1 << 3) | (1 << 4),
    .p_context           = NULL,
    .p_extend            = &g_mtu3_m0_3ph_drv_extend,
};
/* Instance structure to use this module. */
const three_phase_instance_t g_mtu3_m0_3ph_drv =
{
    .p_ctrl        = &g_mtu3_m0_3ph_drv_ctrl,
    .p_cfg         = &g_mtu3_m0_3ph_drv_cfg,
    .p_api         = &g_three_phase_on_mtu3_three_phase
};
gpt_instance_ctrl_t g_gpt2_slowLoop_ctrl;
#if 0
const gpt_extended_pwm_cfg_t g_gpt2_slowLoop_pwm_extend =
{
    .trough_ipl          = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT2_UDF)
    .trough_irq          = VECTOR_NUMBER_GPT2_UDF,
#else
    .trough_irq          = FSP_INVALID_VECTOR,
#endif
    .poeg_link           = GPT_POEG_LINK_POEG0,
    .output_disable      =  GPT_OUTPUT_DISABLE_NONE,
    .adc_trigger         =  GPT_ADC_TRIGGER_NONE,
    .dead_time_count_up  = 0,
    .dead_time_count_down = 0,
    .adc_a_compare_match = 0,
    .adc_b_compare_match = 0,
    .interrupt_skip_source = GPT_INTERRUPT_SKIP_SOURCE_NONE,
    .interrupt_skip_count  = GPT_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_adc    = GPT_INTERRUPT_SKIP_ADC_NONE,
    .gtioca_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED,
    .gtiocb_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED,
    .interrupt_skip_source_ext1 = GPT_INTERRUPT_SKIP_SOURCE_NONE,
    .interrupt_skip_count_ext1  = GPT_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_source_ext2 = GPT_INTERRUPT_SKIP_SOURCE_NONE,
    .interrupt_skip_count_ext2  = GPT_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_func_ovf    = GPT_INTERRUPT_SKIP_SELECT_NONE,
    .interrupt_skip_func_unf    = GPT_INTERRUPT_SKIP_SELECT_NONE,
    .interrupt_skip_func_adc_a  = GPT_INTERRUPT_SKIP_SELECT_NONE,
    .interrupt_skip_func_adc_b  = GPT_INTERRUPT_SKIP_SELECT_NONE,
};
#endif
const gpt_extended_cfg_t g_gpt2_slowLoop_extend =
{
    .gtioca = { .output_enabled = false,
                .stop_level     = GPT_PIN_LEVEL_LOW
              },
    .gtiocb = { .output_enabled = false,
                .stop_level     = GPT_PIN_LEVEL_LOW
              },
    .start_source        = (gpt_source_t) ( GPT_SOURCE_NONE),
    .stop_source         = (gpt_source_t) ( GPT_SOURCE_NONE),
    .clear_source        = (gpt_source_t) ( GPT_SOURCE_NONE),
    .count_up_source     = (gpt_source_t) ( GPT_SOURCE_NONE),
    .count_down_source   = (gpt_source_t) ( GPT_SOURCE_NONE),
    .capture_a_source    = (gpt_source_t) ( GPT_SOURCE_NONE),
    .capture_b_source    = (gpt_source_t) ( GPT_SOURCE_NONE),
    .capture_a_ipl       = (BSP_IRQ_DISABLED),
    .capture_b_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT2_CCMPA)
    .capture_a_irq       = VECTOR_NUMBER_GPT2_CCMPA,
#else
    .capture_a_irq       = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_GPT2_CCMPB)
    .capture_b_irq       = VECTOR_NUMBER_GPT2_CCMPB,
#else
    .capture_b_irq       = FSP_INVALID_VECTOR,
#endif
    .capture_filter_gtioca       = GPT_CAPTURE_FILTER_NONE,
    .capture_filter_gtiocb       = GPT_CAPTURE_FILTER_NONE,
#if 0
    .p_pwm_cfg                   = &g_gpt2_slowLoop_pwm_extend,
#else
    .p_pwm_cfg                   = NULL,
#endif
    .dead_time_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT2_DTE)
    .dead_time_irq       = VECTOR_NUMBER_GPT2_DTE,
#else
    .dead_time_irq       = FSP_INVALID_VECTOR,
#endif
    .icds                = 0,
};
const timer_cfg_t g_gpt2_slowLoop_cfg =
{
    .mode                = TIMER_MODE_PERIODIC,
    /* Actual period: 10.73741824 seconds. Actual duty: 50%. */ .period_counts = (uint32_t) 0x100000000, .duty_cycle_counts = 0x80000000, .source_div = (timer_source_div_t)0,
    .channel             = 2,
    .p_callback          = NULL,
    .p_context           = NULL,
    .p_extend            = &g_gpt2_slowLoop_extend,
    .cycle_end_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT2_OVF)
    .cycle_end_irq       = VECTOR_NUMBER_GPT2_OVF,
#else
    .cycle_end_irq       = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const timer_instance_t g_gpt2_slowLoop =
{
    .p_ctrl        = &g_gpt2_slowLoop_ctrl,
    .p_cfg         = &g_gpt2_slowLoop_cfg,
    .p_api         = &g_timer_on_gpt
};
gpt_instance_ctrl_t g_gpt1_fastLoop_ctrl;
#if 0
const gpt_extended_pwm_cfg_t g_gpt1_fastLoop_pwm_extend =
{
    .trough_ipl          = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT1_UDF)
    .trough_irq          = VECTOR_NUMBER_GPT1_UDF,
#else
    .trough_irq          = FSP_INVALID_VECTOR,
#endif
    .poeg_link           = GPT_POEG_LINK_POEG0,
    .output_disable      =  GPT_OUTPUT_DISABLE_NONE,
    .adc_trigger         =  GPT_ADC_TRIGGER_NONE,
    .dead_time_count_up  = 0,
    .dead_time_count_down = 0,
    .adc_a_compare_match = 0,
    .adc_b_compare_match = 0,
    .interrupt_skip_source = GPT_INTERRUPT_SKIP_SOURCE_NONE,
    .interrupt_skip_count  = GPT_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_adc    = GPT_INTERRUPT_SKIP_ADC_NONE,
    .gtioca_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED,
    .gtiocb_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED,
    .interrupt_skip_source_ext1 = GPT_INTERRUPT_SKIP_SOURCE_NONE,
    .interrupt_skip_count_ext1  = GPT_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_source_ext2 = GPT_INTERRUPT_SKIP_SOURCE_NONE,
    .interrupt_skip_count_ext2  = GPT_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_func_ovf    = GPT_INTERRUPT_SKIP_SELECT_NONE,
    .interrupt_skip_func_unf    = GPT_INTERRUPT_SKIP_SELECT_NONE,
    .interrupt_skip_func_adc_a  = GPT_INTERRUPT_SKIP_SELECT_NONE,
    .interrupt_skip_func_adc_b  = GPT_INTERRUPT_SKIP_SELECT_NONE,
};
#endif
const gpt_extended_cfg_t g_gpt1_fastLoop_extend =
{
    .gtioca = { .output_enabled = false,
                .stop_level     = GPT_PIN_LEVEL_LOW
              },
    .gtiocb = { .output_enabled = false,
                .stop_level     = GPT_PIN_LEVEL_LOW
              },
    .start_source        = (gpt_source_t) ( GPT_SOURCE_NONE),
    .stop_source         = (gpt_source_t) ( GPT_SOURCE_NONE),
    .clear_source        = (gpt_source_t) ( GPT_SOURCE_NONE),
    .count_up_source     = (gpt_source_t) ( GPT_SOURCE_NONE),
    .count_down_source   = (gpt_source_t) ( GPT_SOURCE_NONE),
    .capture_a_source    = (gpt_source_t) ( GPT_SOURCE_NONE),
    .capture_b_source    = (gpt_source_t) ( GPT_SOURCE_NONE),
    .capture_a_ipl       = (BSP_IRQ_DISABLED),
    .capture_b_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT1_CCMPA)
    .capture_a_irq       = VECTOR_NUMBER_GPT1_CCMPA,
#else
    .capture_a_irq       = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_GPT1_CCMPB)
    .capture_b_irq       = VECTOR_NUMBER_GPT1_CCMPB,
#else
    .capture_b_irq       = FSP_INVALID_VECTOR,
#endif
    .capture_filter_gtioca       = GPT_CAPTURE_FILTER_NONE,
    .capture_filter_gtiocb       = GPT_CAPTURE_FILTER_NONE,
#if 0
    .p_pwm_cfg                   = &g_gpt1_fastLoop_pwm_extend,
#else
    .p_pwm_cfg                   = NULL,
#endif
    .dead_time_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT1_DTE)
    .dead_time_irq       = VECTOR_NUMBER_GPT1_DTE,
#else
    .dead_time_irq       = FSP_INVALID_VECTOR,
#endif
    .icds                = 0,
};
const timer_cfg_t g_gpt1_fastLoop_cfg =
{
    .mode                = TIMER_MODE_PERIODIC,
    /* Actual period: 10.73741824 seconds. Actual duty: 50%. */ .period_counts = (uint32_t) 0x100000000, .duty_cycle_counts = 0x80000000, .source_div = (timer_source_div_t)0,
    .channel             = 1,
    .p_callback          = NULL,
    .p_context           = NULL,
    .p_extend            = &g_gpt1_fastLoop_extend,
    .cycle_end_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT1_OVF)
    .cycle_end_irq       = VECTOR_NUMBER_GPT1_OVF,
#else
    .cycle_end_irq       = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const timer_instance_t g_gpt1_fastLoop =
{
    .p_ctrl        = &g_gpt1_fastLoop_ctrl,
    .p_cfg         = &g_gpt1_fastLoop_cfg,
    .p_api         = &g_timer_on_gpt
};
gpt_instance_ctrl_t g_gpt0_hyperLoop_ctrl;
#if 0
const gpt_extended_pwm_cfg_t g_gpt0_hyperLoop_pwm_extend =
{
    .trough_ipl          = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT0_UDF)
    .trough_irq          = VECTOR_NUMBER_GPT0_UDF,
#else
    .trough_irq          = FSP_INVALID_VECTOR,
#endif
    .poeg_link           = GPT_POEG_LINK_POEG0,
    .output_disable      =  GPT_OUTPUT_DISABLE_NONE,
    .adc_trigger         =  GPT_ADC_TRIGGER_NONE,
    .dead_time_count_up  = 0,
    .dead_time_count_down = 0,
    .adc_a_compare_match = 0,
    .adc_b_compare_match = 0,
    .interrupt_skip_source = GPT_INTERRUPT_SKIP_SOURCE_NONE,
    .interrupt_skip_count  = GPT_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_adc    = GPT_INTERRUPT_SKIP_ADC_NONE,
    .gtioca_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED,
    .gtiocb_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED,
    .interrupt_skip_source_ext1 = GPT_INTERRUPT_SKIP_SOURCE_NONE,
    .interrupt_skip_count_ext1  = GPT_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_source_ext2 = GPT_INTERRUPT_SKIP_SOURCE_NONE,
    .interrupt_skip_count_ext2  = GPT_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_func_ovf    = GPT_INTERRUPT_SKIP_SELECT_NONE,
    .interrupt_skip_func_unf    = GPT_INTERRUPT_SKIP_SELECT_NONE,
    .interrupt_skip_func_adc_a  = GPT_INTERRUPT_SKIP_SELECT_NONE,
    .interrupt_skip_func_adc_b  = GPT_INTERRUPT_SKIP_SELECT_NONE,
};
#endif
const gpt_extended_cfg_t g_gpt0_hyperLoop_extend =
{
    .gtioca = { .output_enabled = false,
                .stop_level     = GPT_PIN_LEVEL_LOW
              },
    .gtiocb = { .output_enabled = false,
                .stop_level     = GPT_PIN_LEVEL_LOW
              },
    .start_source        = (gpt_source_t) ( GPT_SOURCE_NONE),
    .stop_source         = (gpt_source_t) ( GPT_SOURCE_NONE),
    .clear_source        = (gpt_source_t) ( GPT_SOURCE_NONE),
    .count_up_source     = (gpt_source_t) ( GPT_SOURCE_NONE),
    .count_down_source   = (gpt_source_t) ( GPT_SOURCE_NONE),
    .capture_a_source    = (gpt_source_t) ( GPT_SOURCE_NONE),
    .capture_b_source    = (gpt_source_t) ( GPT_SOURCE_NONE),
    .capture_a_ipl       = (BSP_IRQ_DISABLED),
    .capture_b_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT0_CCMPA)
    .capture_a_irq       = VECTOR_NUMBER_GPT0_CCMPA,
#else
    .capture_a_irq       = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_GPT0_CCMPB)
    .capture_b_irq       = VECTOR_NUMBER_GPT0_CCMPB,
#else
    .capture_b_irq       = FSP_INVALID_VECTOR,
#endif
    .capture_filter_gtioca       = GPT_CAPTURE_FILTER_NONE,
    .capture_filter_gtiocb       = GPT_CAPTURE_FILTER_NONE,
#if 0
    .p_pwm_cfg                   = &g_gpt0_hyperLoop_pwm_extend,
#else
    .p_pwm_cfg                   = NULL,
#endif
    .dead_time_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT0_DTE)
    .dead_time_irq       = VECTOR_NUMBER_GPT0_DTE,
#else
    .dead_time_irq       = FSP_INVALID_VECTOR,
#endif
    .icds                = 0,
};
const timer_cfg_t g_gpt0_hyperLoop_cfg =
{
    .mode                = TIMER_MODE_PERIODIC,
    /* Actual period: 10.73741824 seconds. Actual duty: 50%. */ .period_counts = (uint32_t) 0x100000000, .duty_cycle_counts = 0x80000000, .source_div = (timer_source_div_t)0,
    .channel             = 0,
    .p_callback          = NULL,
    .p_context           = NULL,
    .p_extend            = &g_gpt0_hyperLoop_extend,
    .cycle_end_ipl       = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT0_OVF)
    .cycle_end_irq       = VECTOR_NUMBER_GPT0_OVF,
#else
    .cycle_end_irq       = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const timer_instance_t g_gpt0_hyperLoop =
{
    .p_ctrl        = &g_gpt0_hyperLoop_ctrl,
    .p_cfg         = &g_gpt0_hyperLoop_cfg,
    .p_api         = &g_timer_on_gpt
};
void g_hal_init(void) {
g_common_init();
}
