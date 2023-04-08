/* generated vector source file - do not edit */
        #include "bsp_api.h"
        /* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
        #if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_MAX_ENTRIES] =
        {
                        [251] = gmac_isr_pmt, /* GMAC_PMT (GMAC1 power management) */
            [252] = gmac_isr_sbd, /* GMAC_SBD (GMAC1 general interrupt) */
            [253] = ethsw_isr_intr, /* ETHSW_INTR (Ethernet Switch interrupt) */
            [288] = sci_uart_eri_isr, /* SCI0_ERI (SCI0 Receive error) */
            [289] = sci_uart_rxi_isr, /* SCI0_RXI (SCI0 Receive data full) */
            [290] = sci_uart_txi_isr, /* SCI0_TXI (SCI0 Transmit data empty) */
            [291] = sci_uart_tei_isr, /* SCI0_TEI (SCI0 Transmit end) */
            [316] = canfd_rx_fifo_isr, /* CAN_RXF (CANFD RX FIFO interrupt) */
            [317] = canfd_error_isr, /* CAN_GLERR (CANFD Global error interrupt) */
            [318] = canfd_channel_tx_isr, /* CAN0_TX (CANFD0 Channel TX interrupt) */
            [319] = canfd_error_isr, /* CAN0_CHERR (CANFD0 Channel CAN error interrupt) */
            [321] = canfd_channel_tx_isr, /* CAN1_TX (CANFD1 Channel TX interrupt) */
            [322] = canfd_error_isr, /* CAN1_CHERR (CANFD1 Channel CAN error interrupt) */
            [324] = spi_rxi_isr, /* SPI0_SPRI (SPI0 Reception buffer full) */
            [325] = spi_txi_isr, /* SPI0_SPTI (SPI0 Transmit buffer empty) */
            [327] = spi_eri_isr, /* SPI0_SPEI (SPI0 errors) */
            [328] = spi_tei_isr, /* SPI0_SPCEND (SPI0 Communication complete) */
            [329] = spi_rxi_isr, /* SPI1_SPRI (SPI1 Reception buffer full) */
            [330] = spi_txi_isr, /* SPI1_SPTI (SPI1 Transmit buffer empty) */
            [332] = spi_eri_isr, /* SPI1_SPEI (SPI1 errors) */
            [333] = spi_tei_isr, /* SPI1_SPCEND (SPI1 Communication complete) */
            [345] = adc_scan_end_isr, /* ADC0_ADI (ADC0 A/D scan end interrupt) */
            [350] = adc_scan_end_isr, /* ADC1_ADI (ADC1 A/D scan end interrupt) */
            [443] = spi_rxi_isr, /* SPI3_SPRI (SPI3 Reception buffer full) */
            [444] = spi_txi_isr, /* SPI3_SPTI (SPI3 Transmit buffer empty) */
            [446] = spi_eri_isr, /* SPI3_SPEI (SPI3 errors) */
            [447] = spi_tei_isr, /* SPI3_SPCEND (SPI3 Communication complete) */
        };
        #endif