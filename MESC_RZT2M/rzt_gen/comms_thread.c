/* generated thread source file - do not edit */
#include "comms_thread.h"

#if 1
                static StaticTask_t comms_thread_memory;
                #if defined(__ARMCC_VERSION)           /* AC6 compiler */
                static uint8_t comms_thread_stack[512] BSP_PLACE_IN_SECTION(BSP_UNINIT_SECTION_PREFIX ".stack.thread") BSP_ALIGN_VARIABLE(BSP_STACK_ALIGNMENT);
                #else
                static uint8_t comms_thread_stack[512] BSP_PLACE_IN_SECTION(BSP_UNINIT_SECTION_PREFIX ".stack.comms_thread") BSP_ALIGN_VARIABLE(BSP_STACK_ALIGNMENT);
                #endif
                #endif
                TaskHandle_t comms_thread;
                void comms_thread_create(void);
                static void comms_thread_func(void * pvParameters);
                void rtos_startup_err_callback(void * p_instance, void * p_data);
                void rtos_startup_common_init(void);
spi_instance_ctrl_t g_spi3_ctrl;

/** SPI extended configuration for SPI HAL driver */
const spi_extended_cfg_t g_spi3_ext_cfg =
{
    .spi_clksyn              = SPI_SSL_MODE_CLK_SYN,
    .spi_comm                = SPI_COMMUNICATION_FULL_DUPLEX,
    .ssl_polarity            = SPI_SSLP_LOW,
    .ssl_select              = SPI_SSL_SELECT_SSL0,
    .mosi_idle               = SPI_MOSI_IDLE_VALUE_FIXING_DISABLE,
    .parity                  = SPI_PARITY_MODE_DISABLE,
    .byte_swap               = SPI_BYTE_SWAP_DISABLE,
    .spck_div                = {
        /* Actual calculated bitrate: 16000000. */ .spbr = 2, .brdv = 0
    },
    .spck_delay              = SPI_DELAY_COUNT_1,
    .ssl_negation_delay      = SPI_DELAY_COUNT_1,
    .next_access_delay       = SPI_DELAY_COUNT_1,
    .transmit_fifo_threshold = 0,
    .receive_fifo_threshold  = 0,
    .receive_data_ready_detect_adjustment  = 0,
    .sync_bypass             = SPI_SYNCHRONIZER_NOT_BYPASS
 };

/** SPI configuration for SPI HAL driver */
const spi_cfg_t g_spi3_cfg =
{
    .channel             = 3,

#if defined(VECTOR_NUMBER_SPI3_SPRI)
    .rxi_irq             = VECTOR_NUMBER_SPI3_SPRI,
#else
    .rxi_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SPI3_SPTI)
    .txi_irq             = VECTOR_NUMBER_SPI3_SPTI,
#else
    .txi_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SPI3_SPCEND)
    .tei_irq             = VECTOR_NUMBER_SPI3_SPCEND,
#else
    .tei_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SPI3_SPEI)
    .eri_irq             = VECTOR_NUMBER_SPI3_SPEI,
#else
    .eri_irq             = FSP_INVALID_VECTOR,
#endif

    .rxi_ipl             = (12),
    .txi_ipl             = (12),
    .tei_ipl             = (12),
    .eri_ipl             = (12),

    .operating_mode      = SPI_MODE_MASTER,

    .clk_phase           = SPI_CLK_PHASE_EDGE_ODD,
    .clk_polarity        = SPI_CLK_POLARITY_LOW,

    .mode_fault          = SPI_MODE_FAULT_ERROR_DISABLE,
    .bit_order           = SPI_BIT_ORDER_MSB_FIRST,
#define FSP_NOT_DEFINED (1)
#if (FSP_NOT_DEFINED == FSP_NOT_DEFINED)
    .p_transfer_tx       = NULL,
#else
    .p_transfer_tx       = &FSP_NOT_DEFINED,
#endif
#if (FSP_NOT_DEFINED == FSP_NOT_DEFINED)
    .p_transfer_rx       = NULL,
#else
    .p_transfer_rx       = &FSP_NOT_DEFINED,
#endif
#undef FSP_NOT_DEFINED
    .p_callback          = spi_callback,

    .p_context           = NULL,
    .p_extend            = (void *)&g_spi3_ext_cfg,
};

/* Instance structure to use this module. */
const spi_instance_t g_spi3 =
{
    .p_ctrl        = &g_spi3_ctrl,
    .p_cfg         = &g_spi3_cfg,
    .p_api         = &g_spi_on_spi
};
spi_instance_ctrl_t g_spi1_ctrl;

/** SPI extended configuration for SPI HAL driver */
const spi_extended_cfg_t g_spi1_ext_cfg =
{
    .spi_clksyn              = SPI_SSL_MODE_CLK_SYN,
    .spi_comm                = SPI_COMMUNICATION_FULL_DUPLEX,
    .ssl_polarity            = SPI_SSLP_LOW,
    .ssl_select              = SPI_SSL_SELECT_SSL0,
    .mosi_idle               = SPI_MOSI_IDLE_VALUE_FIXING_DISABLE,
    .parity                  = SPI_PARITY_MODE_DISABLE,
    .byte_swap               = SPI_BYTE_SWAP_DISABLE,
    .spck_div                = {
        /* Actual calculated bitrate: 16000000. */ .spbr = 2, .brdv = 0
    },
    .spck_delay              = SPI_DELAY_COUNT_1,
    .ssl_negation_delay      = SPI_DELAY_COUNT_1,
    .next_access_delay       = SPI_DELAY_COUNT_1,
    .transmit_fifo_threshold = 0,
    .receive_fifo_threshold  = 0,
    .receive_data_ready_detect_adjustment  = 0,
    .sync_bypass             = SPI_SYNCHRONIZER_NOT_BYPASS
 };

/** SPI configuration for SPI HAL driver */
const spi_cfg_t g_spi1_cfg =
{
    .channel             = 1,

#if defined(VECTOR_NUMBER_SPI1_SPRI)
    .rxi_irq             = VECTOR_NUMBER_SPI1_SPRI,
#else
    .rxi_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SPI1_SPTI)
    .txi_irq             = VECTOR_NUMBER_SPI1_SPTI,
#else
    .txi_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SPI1_SPCEND)
    .tei_irq             = VECTOR_NUMBER_SPI1_SPCEND,
#else
    .tei_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SPI1_SPEI)
    .eri_irq             = VECTOR_NUMBER_SPI1_SPEI,
#else
    .eri_irq             = FSP_INVALID_VECTOR,
#endif

    .rxi_ipl             = (12),
    .txi_ipl             = (12),
    .tei_ipl             = (12),
    .eri_ipl             = (12),

    .operating_mode      = SPI_MODE_MASTER,

    .clk_phase           = SPI_CLK_PHASE_EDGE_ODD,
    .clk_polarity        = SPI_CLK_POLARITY_LOW,

    .mode_fault          = SPI_MODE_FAULT_ERROR_DISABLE,
    .bit_order           = SPI_BIT_ORDER_MSB_FIRST,
#define FSP_NOT_DEFINED (1)
#if (FSP_NOT_DEFINED == FSP_NOT_DEFINED)
    .p_transfer_tx       = NULL,
#else
    .p_transfer_tx       = &FSP_NOT_DEFINED,
#endif
#if (FSP_NOT_DEFINED == FSP_NOT_DEFINED)
    .p_transfer_rx       = NULL,
#else
    .p_transfer_rx       = &FSP_NOT_DEFINED,
#endif
#undef FSP_NOT_DEFINED
    .p_callback          = spi_callback,

    .p_context           = NULL,
    .p_extend            = (void *)&g_spi1_ext_cfg,
};

/* Instance structure to use this module. */
const spi_instance_t g_spi1 =
{
    .p_ctrl        = &g_spi1_ctrl,
    .p_cfg         = &g_spi1_cfg,
    .p_api         = &g_spi_on_spi
};
spi_instance_ctrl_t g_spi0_ctrl;

/** SPI extended configuration for SPI HAL driver */
const spi_extended_cfg_t g_spi0_ext_cfg =
{
    .spi_clksyn              = SPI_SSL_MODE_CLK_SYN,
    .spi_comm                = SPI_COMMUNICATION_FULL_DUPLEX,
    .ssl_polarity            = SPI_SSLP_LOW,
    .ssl_select              = SPI_SSL_SELECT_SSL0,
    .mosi_idle               = SPI_MOSI_IDLE_VALUE_FIXING_DISABLE,
    .parity                  = SPI_PARITY_MODE_DISABLE,
    .byte_swap               = SPI_BYTE_SWAP_DISABLE,
    .spck_div                = {
        /* Actual calculated bitrate: 16000000. */ .spbr = 2, .brdv = 0
    },
    .spck_delay              = SPI_DELAY_COUNT_1,
    .ssl_negation_delay      = SPI_DELAY_COUNT_1,
    .next_access_delay       = SPI_DELAY_COUNT_1,
    .transmit_fifo_threshold = 0,
    .receive_fifo_threshold  = 0,
    .receive_data_ready_detect_adjustment  = 0,
    .sync_bypass             = SPI_SYNCHRONIZER_NOT_BYPASS
 };

/** SPI configuration for SPI HAL driver */
const spi_cfg_t g_spi0_cfg =
{
    .channel             = 0,

#if defined(VECTOR_NUMBER_SPI0_SPRI)
    .rxi_irq             = VECTOR_NUMBER_SPI0_SPRI,
#else
    .rxi_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SPI0_SPTI)
    .txi_irq             = VECTOR_NUMBER_SPI0_SPTI,
#else
    .txi_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SPI0_SPCEND)
    .tei_irq             = VECTOR_NUMBER_SPI0_SPCEND,
#else
    .tei_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SPI0_SPEI)
    .eri_irq             = VECTOR_NUMBER_SPI0_SPEI,
#else
    .eri_irq             = FSP_INVALID_VECTOR,
#endif

    .rxi_ipl             = (12),
    .txi_ipl             = (12),
    .tei_ipl             = (12),
    .eri_ipl             = (12),

    .operating_mode      = SPI_MODE_MASTER,

    .clk_phase           = SPI_CLK_PHASE_EDGE_ODD,
    .clk_polarity        = SPI_CLK_POLARITY_LOW,

    .mode_fault          = SPI_MODE_FAULT_ERROR_DISABLE,
    .bit_order           = SPI_BIT_ORDER_MSB_FIRST,
#define FSP_NOT_DEFINED (1)
#if (FSP_NOT_DEFINED == FSP_NOT_DEFINED)
    .p_transfer_tx       = NULL,
#else
    .p_transfer_tx       = &FSP_NOT_DEFINED,
#endif
#if (FSP_NOT_DEFINED == FSP_NOT_DEFINED)
    .p_transfer_rx       = NULL,
#else
    .p_transfer_rx       = &FSP_NOT_DEFINED,
#endif
#undef FSP_NOT_DEFINED
    .p_callback          = spi_callback,

    .p_context           = NULL,
    .p_extend            = (void *)&g_spi0_ext_cfg,
};

/* Instance structure to use this module. */
const spi_instance_t g_spi0 =
{
    .p_ctrl        = &g_spi0_ctrl,
    .p_cfg         = &g_spi0_cfg,
    .p_api         = &g_spi_on_spi
};
dmac_instance_ctrl_t g_transfer1_ctrl;

dmac_register_set_setting_t g_transfer1_next1_register_setting =
{
    .p_dest = NULL,
    .p_src  = NULL,
    .length = 1
};

dmac_extended_info_t g_transfer1_extend_info =
{
    .src_size            = DMAC_TRANSFER_SIZE_1_BYTE,
    .dest_size           = DMAC_TRANSFER_SIZE_1_BYTE,
    .p_next1_register_setting = &g_transfer1_next1_register_setting,
};

transfer_info_t g_transfer1_info =
{
    .dest_addr_mode      = TRANSFER_ADDR_MODE_INCREMENTED,
    .repeat_area         = (transfer_repeat_area_t) 0, // Unused
    .irq                 = (transfer_irq_t) 0, // Unused
    .chain_mode          = (transfer_chain_mode_t) 0, // Unused
    .src_addr_mode       = TRANSFER_ADDR_MODE_FIXED,
    .size                = (transfer_size_t) 0, // Unused
    .mode                = TRANSFER_MODE_NORMAL,
    .p_dest              = (void *) NULL,
    .p_src               = (void const *) NULL,
    .num_blocks          = 0, // Unused
    .length              = 0,
    .p_extend            = &g_transfer1_extend_info,
};

const dmac_extended_cfg_t g_transfer1_extend =
{
    .unit                = 0,
    .channel             = 1,
#if defined(VECTOR_NUMBER_DMAC0_INT1)
    .dmac_int_irq         = VECTOR_NUMBER_DMAC0_INT1,
#else
    .dmac_int_irq         = FSP_INVALID_VECTOR,
#endif
    .dmac_int_ipl         = (BSP_IRQ_DISABLED),
    .dmac_int_irq_detect_type = (0),

    .activation_source   = ELC_EVENT_SCI0_RXI,

    .ack_mode               = DMAC_ACK_MODE_BUS_CYCLE_MODE,
    .detection_mode         = (dmac_detection_t) ((0) << 2 | (1) << 1 | (0) << 0),
    .activation_request_source_select = DMAC_REQUEST_DIRECTION_SOURCE_MODULE,

    .next_register_operaion = DMAC_REGISTER_SELECT_REVERSE_DISABLE,

    .transfer_interval      = 0,
    .channel_scheduling     = DMAC_CHANNEL_SCHEDULING_FIXED,

    .p_callback          = NULL,
    .p_context           = NULL,

    .p_peripheral_module_handler = sci_uart_rxi_dmac_isr,
};
const transfer_cfg_t g_transfer1_cfg =
{
    .p_info              = &g_transfer1_info,
    .p_extend            = &g_transfer1_extend,
};
/* Instance structure to use this module. */
const transfer_instance_t g_transfer1 =
{
    .p_ctrl        = &g_transfer1_ctrl,
    .p_cfg         = &g_transfer1_cfg,
    .p_api         = &g_transfer_on_dmac
};
dmac_instance_ctrl_t g_transfer0_ctrl;

dmac_register_set_setting_t g_transfer0_next1_register_setting =
{
    .p_dest = NULL,
    .p_src  = NULL,
    .length = 1
};

dmac_extended_info_t g_transfer0_extend_info =
{
    .src_size            = DMAC_TRANSFER_SIZE_1_BYTE,
    .dest_size           = DMAC_TRANSFER_SIZE_1_BYTE,
    .p_next1_register_setting = &g_transfer0_next1_register_setting,
};

transfer_info_t g_transfer0_info =
{
    .dest_addr_mode      = TRANSFER_ADDR_MODE_FIXED,
    .repeat_area         = (transfer_repeat_area_t) 0, // Unused
    .irq                 = (transfer_irq_t) 0, // Unused
    .chain_mode          = (transfer_chain_mode_t) 0, // Unused
    .src_addr_mode       = TRANSFER_ADDR_MODE_INCREMENTED,
    .size                = (transfer_size_t) 0, // Unused
    .mode                = TRANSFER_MODE_NORMAL,
    .p_dest              = (void *) NULL,
    .p_src               = (void const *) NULL,
    .num_blocks          = 0, // Unused
    .length              = 0,
    .p_extend            = &g_transfer0_extend_info,
};

const dmac_extended_cfg_t g_transfer0_extend =
{
    .unit                = 0,
    .channel             = 0,
#if defined(VECTOR_NUMBER_DMAC0_INT0)
    .dmac_int_irq         = VECTOR_NUMBER_DMAC0_INT0,
#else
    .dmac_int_irq         = FSP_INVALID_VECTOR,
#endif
    .dmac_int_ipl         = (BSP_IRQ_DISABLED),
    .dmac_int_irq_detect_type = (0),

    .activation_source   = ELC_EVENT_SCI0_TXI,

    .ack_mode               = DMAC_ACK_MODE_BUS_CYCLE_MODE,
    .detection_mode         = (dmac_detection_t) ((0) << 2 | (1) << 1 | (0) << 0),
    .activation_request_source_select = DMAC_REQUEST_DIRECTION_DESTINATION_MODULE,

    .next_register_operaion = DMAC_REGISTER_SELECT_REVERSE_DISABLE,

    .transfer_interval      = 0,
    .channel_scheduling     = DMAC_CHANNEL_SCHEDULING_FIXED,

    .p_callback          = NULL,
    .p_context           = NULL,

    .p_peripheral_module_handler = sci_uart_txi_dmac_isr,
};
const transfer_cfg_t g_transfer0_cfg =
{
    .p_info              = &g_transfer0_info,
    .p_extend            = &g_transfer0_extend,
};
/* Instance structure to use this module. */
const transfer_instance_t g_transfer0 =
{
    .p_ctrl        = &g_transfer0_ctrl,
    .p_cfg         = &g_transfer0_cfg,
    .p_api         = &g_transfer_on_dmac
};
sci_uart_instance_ctrl_t     g_uart0_ctrl;

            baud_setting_t               g_uart0_baud_setting =
            {
                /* Baud rate calculated with 0.160% error. */ .abcse = 0, .abcs = 0, .bgdm = 1, .cks = 0, .brr = 51, .mddr = (uint8_t) 256, .brme = false
            };

            /** UART extended configuration for UARTonSCI HAL driver */
            const sci_uart_extended_cfg_t g_uart0_cfg_extend =
            {
                .clock                = SCI_UART_CLOCK_INT,
                .rx_edge_start          = SCI_UART_START_BIT_FALLING_EDGE,
                .noise_cancel         = SCI_UART_NOISE_CANCELLATION_DISABLE,
                .rx_fifo_trigger        = SCI_UART_RX_FIFO_TRIGGER_MAX,
                .p_baud_setting         = &g_uart0_baud_setting,
                .uart_mode              = UART_MODE_RS232,
                .ctsrts_en              = SCI_UART_CTSRTS_RTS_OUTPUT,
#if 0
                .flow_control_pin       = BSP_IO_PORT_00_PIN_00,
#else
                .flow_control_pin       = (bsp_io_port_pin_t) (0xFFFFU),
#endif
                .sync_bypass            = SCI_UART_SYNCHRONIZER_NOT_BYPASS,
            };

            /** UART interface configuration */
            const uart_cfg_t g_uart0_cfg =
            {
                .channel             = 0,
                .data_bits           = UART_DATA_BITS_8,
                .parity              = UART_PARITY_OFF,
                .stop_bits           = UART_STOP_BITS_1,
                .p_callback          = user_uart_callback,
                .p_context           = NULL,
                .p_extend            = &g_uart0_cfg_extend,
#define FSP_NOT_DEFINED (1)
#if (FSP_NOT_DEFINED == g_transfer0)
                .p_transfer_tx       = NULL,
#else
                .p_transfer_tx       = &g_transfer0,
#endif
#if (FSP_NOT_DEFINED == g_transfer1)
                .p_transfer_rx       = NULL,
#else
                .p_transfer_rx       = &g_transfer1,
#endif
#undef FSP_NOT_DEFINED
                .rxi_ipl             = (12),
                .txi_ipl             = (12),
                .tei_ipl             = (12),
                .eri_ipl             = (12),
#if defined(VECTOR_NUMBER_SCI0_RXI)
                .rxi_irq             = VECTOR_NUMBER_SCI0_RXI,
#else
                .rxi_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI0_TXI)
                .txi_irq             = VECTOR_NUMBER_SCI0_TXI,
#else
                .txi_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI0_TEI)
                .tei_irq             = VECTOR_NUMBER_SCI0_TEI,
#else
                .tei_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI0_ERI)
                .eri_irq             = VECTOR_NUMBER_SCI0_ERI,
#else
                .eri_irq             = FSP_INVALID_VECTOR,
#endif
            };

/* Instance structure to use this module. */
const uart_instance_t g_uart0 =
{
    .p_ctrl        = &g_uart0_ctrl,
    .p_cfg         = &g_uart0_cfg,
    .p_api         = &g_uart_on_sci
};
/* Nominal and Data bit timing configuration */

can_bit_timing_cfg_t g_canfd1_bit_timing_cfg =
{
    /* Actual bitrate: 500000 Hz. Actual sample point: 75 %. */
    .baud_rate_prescaler = 1,
    .time_segment_1 = 59,
    .time_segment_2 = 20,
    .synchronization_jump_width = 4
};

can_bit_timing_cfg_t g_canfd1_data_timing_cfg =
{
    /* Actual bitrate: 2000000 Hz. Actual sample point: 75 %. */
    .baud_rate_prescaler = 1,
    .time_segment_1 = 14,
    .time_segment_2 = 5,
    .synchronization_jump_width = 1
};


extern const canfd_afl_entry_t p_canfd1_afl[CANFD_CFG_AFL_CH1_RULE_NUM];

#ifndef CANFD_PRV_GLOBAL_CFG
#define CANFD_PRV_GLOBAL_CFG
canfd_global_cfg_t g_canfd_global_cfg =
{
    .global_interrupts = CANFD_CFG_GLOBAL_ERR_SOURCES,
    .global_config     = (CANFD_CFG_TX_PRIORITY | CANFD_CFG_DLC_CHECK | CANFD_CFD_CLOCK_SOURCE | CANFD_CFG_FD_OVERFLOW),
    .rx_mb_config      = (CANFD_CFG_RXMB_NUMBER | (CANFD_CFG_RXMB_SIZE << R_CANFD_CFDRMNB_RMPLS_Pos)),
    .global_err_ipl = CANFD_CFG_GLOBAL_ERR_IPL,
    .rx_fifo_ipl    = CANFD_CFG_RX_FIFO_IPL,
    .rx_fifo_config    =
    {
        ((CANFD_CFG_RXFIFO0_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO0_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO0_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO0_INT_MODE) | (CANFD_CFG_RXFIFO0_ENABLE)),
        ((CANFD_CFG_RXFIFO1_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO1_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO1_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO1_INT_MODE) | (CANFD_CFG_RXFIFO1_ENABLE)),
        ((CANFD_CFG_RXFIFO2_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO2_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO2_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO2_INT_MODE) | (CANFD_CFG_RXFIFO2_ENABLE)),
        ((CANFD_CFG_RXFIFO3_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO3_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO3_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO3_INT_MODE) | (CANFD_CFG_RXFIFO3_ENABLE)),
        ((CANFD_CFG_RXFIFO4_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO4_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO4_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO4_INT_MODE) | (CANFD_CFG_RXFIFO4_ENABLE)),
        ((CANFD_CFG_RXFIFO5_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO5_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO5_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO5_INT_MODE) | (CANFD_CFG_RXFIFO5_ENABLE)),
        ((CANFD_CFG_RXFIFO6_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO6_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO6_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO6_INT_MODE) | (CANFD_CFG_RXFIFO6_ENABLE)),
        ((CANFD_CFG_RXFIFO7_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO7_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO7_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO7_INT_MODE) | (CANFD_CFG_RXFIFO7_ENABLE)),
    },
};
#endif

canfd_extended_cfg_t g_canfd1_extended_cfg =
{
    .p_afl              = p_canfd1_afl,
    .txmb_txi_enable    = ( 0ULL),
    .error_interrupts   = ( 0U),
    .p_data_timing      = &g_canfd1_data_timing_cfg,
    .delay_compensation = (1),
    .p_global_cfg       = &g_canfd_global_cfg,
};

canfd_instance_ctrl_t g_canfd1_ctrl;
const can_cfg_t g_canfd1_cfg =
{
    .channel                = 1,
    .p_bit_timing           = &g_canfd1_bit_timing_cfg,
    .p_callback             = canfd1_callback,
    .p_extend               = &g_canfd1_extended_cfg,
    .p_context              = NULL,
    .ipl                    = (12),
#if defined(VECTOR_NUMBER_CAN1_TX)
    .tx_irq             = VECTOR_NUMBER_CAN1_TX,
#else
    .tx_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_CAN1_CHERR)
    .error_irq             = VECTOR_NUMBER_CAN1_CHERR,
#else
    .error_irq             = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const can_instance_t g_canfd1 =
{
    .p_ctrl        = &g_canfd1_ctrl,
    .p_cfg         = &g_canfd1_cfg,
    .p_api         = &g_canfd_on_canfd
};
/* Nominal and Data bit timing configuration */

can_bit_timing_cfg_t g_canfd0_bit_timing_cfg =
{
    /* Actual bitrate: 500000 Hz. Actual sample point: 75 %. */
    .baud_rate_prescaler = 1,
    .time_segment_1 = 59,
    .time_segment_2 = 20,
    .synchronization_jump_width = 4
};

can_bit_timing_cfg_t g_canfd0_data_timing_cfg =
{
    /* Actual bitrate: 2000000 Hz. Actual sample point: 75 %. */
    .baud_rate_prescaler = 1,
    .time_segment_1 = 14,
    .time_segment_2 = 5,
    .synchronization_jump_width = 1
};


extern const canfd_afl_entry_t p_canfd0_afl[CANFD_CFG_AFL_CH0_RULE_NUM];

#ifndef CANFD_PRV_GLOBAL_CFG
#define CANFD_PRV_GLOBAL_CFG
canfd_global_cfg_t g_canfd_global_cfg =
{
    .global_interrupts = CANFD_CFG_GLOBAL_ERR_SOURCES,
    .global_config     = (CANFD_CFG_TX_PRIORITY | CANFD_CFG_DLC_CHECK | CANFD_CFD_CLOCK_SOURCE | CANFD_CFG_FD_OVERFLOW),
    .rx_mb_config      = (CANFD_CFG_RXMB_NUMBER | (CANFD_CFG_RXMB_SIZE << R_CANFD_CFDRMNB_RMPLS_Pos)),
    .global_err_ipl = CANFD_CFG_GLOBAL_ERR_IPL,
    .rx_fifo_ipl    = CANFD_CFG_RX_FIFO_IPL,
    .rx_fifo_config    =
    {
        ((CANFD_CFG_RXFIFO0_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO0_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO0_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO0_INT_MODE) | (CANFD_CFG_RXFIFO0_ENABLE)),
        ((CANFD_CFG_RXFIFO1_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO1_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO1_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO1_INT_MODE) | (CANFD_CFG_RXFIFO1_ENABLE)),
        ((CANFD_CFG_RXFIFO2_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO2_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO2_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO2_INT_MODE) | (CANFD_CFG_RXFIFO2_ENABLE)),
        ((CANFD_CFG_RXFIFO3_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO3_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO3_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO3_INT_MODE) | (CANFD_CFG_RXFIFO3_ENABLE)),
        ((CANFD_CFG_RXFIFO4_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO4_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO4_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO4_INT_MODE) | (CANFD_CFG_RXFIFO4_ENABLE)),
        ((CANFD_CFG_RXFIFO5_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO5_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO5_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO5_INT_MODE) | (CANFD_CFG_RXFIFO5_ENABLE)),
        ((CANFD_CFG_RXFIFO6_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO6_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO6_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO6_INT_MODE) | (CANFD_CFG_RXFIFO6_ENABLE)),
        ((CANFD_CFG_RXFIFO7_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO7_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO7_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO7_INT_MODE) | (CANFD_CFG_RXFIFO7_ENABLE)),
    },
};
#endif

canfd_extended_cfg_t g_canfd0_extended_cfg =
{
    .p_afl              = p_canfd0_afl,
    .txmb_txi_enable    = ( 0ULL),
    .error_interrupts   = ( 0U),
    .p_data_timing      = &g_canfd0_data_timing_cfg,
    .delay_compensation = (1),
    .p_global_cfg       = &g_canfd_global_cfg,
};

canfd_instance_ctrl_t g_canfd0_ctrl;
const can_cfg_t g_canfd0_cfg =
{
    .channel                = 0,
    .p_bit_timing           = &g_canfd0_bit_timing_cfg,
    .p_callback             = canfd0_callback,
    .p_extend               = &g_canfd0_extended_cfg,
    .p_context              = NULL,
    .ipl                    = (12),
#if defined(VECTOR_NUMBER_CAN0_TX)
    .tx_irq             = VECTOR_NUMBER_CAN0_TX,
#else
    .tx_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_CAN0_CHERR)
    .error_irq             = VECTOR_NUMBER_CAN0_CHERR,
#else
    .error_irq             = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const can_instance_t g_canfd0 =
{
    .p_ctrl        = &g_canfd0_ctrl,
    .p_cfg         = &g_canfd0_cfg,
    .p_api         = &g_canfd_on_canfd
};
extern uint32_t g_fsp_common_thread_count;

                const rm_freertos_port_parameters_t comms_thread_parameters =
                {
                    .p_context = (void *) NULL,
                };

                void comms_thread_create (void)
                {
                    /* Increment count so we will know the number of threads created in the FSP Configuration editor. */
                    g_fsp_common_thread_count++;

                    /* Initialize each kernel object. */
                    

                    #if 1
                    comms_thread = xTaskCreateStatic(
                    #else
                    BaseType_t comms_thread_create_err = xTaskCreate(
                    #endif
                        comms_thread_func,
                        (const char *)"Comms Thread",
                        512/4, // In words, not bytes
                        (void *) &comms_thread_parameters, //pvParameters
                        1,
                        #if 1
                        (StackType_t *)&comms_thread_stack,
                        (StaticTask_t *)&comms_thread_memory
                        #else
                        & comms_thread
                        #endif
                    );

                    #if 1
                    if (NULL == comms_thread)
                    {
                        rtos_startup_err_callback(comms_thread, 0);
                    }
                    #else
                    if (pdPASS != comms_thread_create_err)
                    {
                        rtos_startup_err_callback(comms_thread, 0);
                    }
                    #endif
                }
                static void comms_thread_func (void * pvParameters)
                {
                    /* Initialize common components */
                    rtos_startup_common_init();

                    /* Initialize each module instance. */
                    

                    /* Enter user code for this thread. Pass task handle. */
                    comms_thread_entry(pvParameters);
                }
