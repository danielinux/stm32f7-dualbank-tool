/*  
 *      This is free software: you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License version 2, as 
 *      published by the Free Software Foundation.
 *
 *      This program is distributed in the hope that it will be useful,
 *      but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *      GNU General Public License for more details.
 *
 *      You should have received a copy of the GNU General Public License
 *      along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *      Author: Daniele Lacamera
 *      
 *
 */  
extern unsigned int _stored_data;
extern unsigned int _start_data;
extern unsigned int _end_data;
extern unsigned int _start_bss;
extern unsigned int _end_bss;
extern unsigned int _end_stack;
extern unsigned int _start_heap;


static volatile unsigned int avail_mem = 0;
static unsigned int sp;

extern void main(void);

void isr_reset(void) {
    register unsigned int *src, *dst;
    src = (unsigned int *) &_stored_data;
    dst = (unsigned int *) &_start_data;
    while (dst < (unsigned int *)&_end_data) {
        *dst = *src;
        dst++;
        src++;
    }

    dst = &_start_bss;
    while (dst < (unsigned int *)&_end_bss) {
        *dst = 0U;
        dst++;
    }

    avail_mem = &_end_stack - &_start_heap;
    main();
}

void isr_fault(void)
{
    /* Panic. */
    while(1) ;;

}

void isr_memfault(void)
{
    /* Panic. */
    while(1) ;;
}

void isr_busfault(void)
{
    /* Panic. */
    while(1) ;;
}

void isr_usagefault(void)
{
    /* Panic. */
    while(1) ;;
}
        

void isr_empty(void)
{
    while(1);

}



__attribute__ ((section(".isr_vector")))
void (* const IV[])(void) =
{
	(void (*)(void))(&_end_stack),
	isr_reset,                   // Reset
	isr_fault,                   // NMI
	isr_fault,                   // HardFault
	isr_memfault,                // MemFault
	isr_busfault,                // BusFault
	isr_usagefault,              // UsageFault
	0, 0, 0, 0,                  // 4x reserved
	isr_empty,                   // SVC
	isr_empty,                   // DebugMonitor
	0,                           // reserved
	isr_empty,                   // PendSV
	isr_empty,                   // SysTick

    /* F7 Specific */

    isr_empty,                                 // nvic_wwdg_isr(void);
    isr_empty,                                 // pvd_isr(void);
    isr_empty,                                 // tamp_stamp_isr(void);
    isr_empty,                                 // rtc_wkup_isr(void);
    isr_empty,                                 // flash_isr(void);
    isr_empty,                                 // rcc_isr(void);
    isr_empty,                                 // exti0_isr(void);
    isr_empty,                                 // exti1_isr(void);
    isr_empty,                                 // exti2_isr(void);
    isr_empty,                                 // exti3_isr(void);
    isr_empty,                                 // exti4_isr(void);
    isr_empty,                                 // dma1_stream0_isr(void);
    isr_empty,                                 // dma1_stream1_isr(void);
    isr_empty,                                 // dma1_stream2_isr(void);
    isr_empty,                                 // dma1_stream3_isr(void);
    isr_empty,                                 // dma1_stream4_isr(void);
    isr_empty,                                 // dma1_stream5_isr(void);
    isr_empty,                                 // dma1_stream6_isr(void);
    isr_empty,                                 // adc_isr(void);
    isr_empty,                                 // can1_tx_isr(void);
    isr_empty,                                 // can1_rx0_isr(void);
    isr_empty,                                 // can1_rx1_isr(void);
    isr_empty,                                 // can1_sce_isr(void);
    isr_empty,                                 // exti9_5_isr(void);
    isr_empty,                                 // tim1_brk_tim9_isr(void);
    isr_empty,                                 // tim1_up_tim10_isr(void);
    isr_empty,                                 // tim1_trg_com_tim11_isr(void);
    isr_empty,                                 // tim1_cc_isr(void);
    isr_empty,                                 // tim2_isr(void);
    isr_empty,                                 // tim3_isr(void);
    isr_empty,                                 // tim4_isr(void);
    isr_empty,                                 // i2c1_ev_isr(void);
    isr_empty,                                 // i2c1_er_isr(void);
    isr_empty,                                 // i2c2_ev_isr(void);
    isr_empty,                                 // i2c2_er_isr(void);
    isr_empty,                                 // spi1_isr(void);
    isr_empty,                                 // spi2_isr(void);
    isr_empty,                                 // usart1_isr(void);
    isr_empty,                                 // usart2_isr(void);
    isr_empty,                                 // usart3_isr(void);
    isr_empty,                             // exti15_10_isr(void);
    isr_empty,                                 // rtc_alarm_isr(void);
    isr_empty,                                 // usb_fs_wkup_isr(void);
    isr_empty,                                 // tim8_brk_tim12_isr(void);
    isr_empty,                                 // tim8_up_tim13_isr(void);
    isr_empty,                                 // tim8_trg_com_tim14_isr(void);
    isr_empty,                                 // tim8_cc_isr(void);
    isr_empty,                                 // dma1_stream7_isr(void);
    isr_empty,                                 // fsmc_isr(void);
    isr_empty,                                 // sdmmc1_isr(void);
    isr_empty,                                 // tim5_isr(void);
    isr_empty,                                 // spi3_isr(void);
    isr_empty,                                 // uart4_isr(void);
    isr_empty,                                 // uart5_isr(void);
    isr_empty,                                 // tim6_dac_isr(void);
    isr_empty,                                 // tim7_isr(void);
    isr_empty,                                 // dma2_stream0_isr(void);
    isr_empty,                                 // dma2_stream1_isr(void);
    isr_empty,                                 // dma2_stream2_isr(void);
    isr_empty,                                 // dma2_stream3_isr(void);
    isr_empty,                                 // dma2_stream4_isr(void);
    isr_empty,                                 // eth_isr(void);
    isr_empty,                                 // eth_wkup_isr(void);
    isr_empty,                                 // can2_tx_isr(void);
    isr_empty,                                 // can2_rx0_isr(void);
    isr_empty,                                 // can2_rx1_isr(void);
    isr_empty,                                 // can2_sce_isr(void);
    isr_empty,                                 // otg_fs_isr(void);
    isr_empty,                                 // dma2_stream5_isr(void);
    isr_empty,                                 // dma2_stream6_isr(void);
    isr_empty,                                 // dma2_stream7_isr(void);
    isr_empty,                                 // usart6_isr(void);
    isr_empty,                                 // i2c3_ev_isr(void);
    isr_empty,                                 // i2c3_er_isr(void);
    isr_empty,                                 // otg_hs_ep1_out_isr(void);
    isr_empty,                                 // otg_hs_ep1_in_isr(void);
    isr_empty,                                 // otg_hs_wkup_isr(void);
    isr_empty,                                 // otg_hs_isr(void);
    isr_empty,                                 // dcmi_isr(void);
    isr_empty,                                 // cryp_isr(void);
    isr_empty,                                 // hash_rng_isr(void);
    isr_empty,                                 // fpu_isr(void);
    isr_empty,                                 // uart7_isr(void);
    isr_empty,                                 // uart8_isr(void);
    isr_empty,                                 // spi4_isr(void);
    isr_empty,                                 // spi5_isr(void);
    isr_empty,                                 // spi6_isr(void);
    isr_empty,                                 // sai1_isr(void);
    isr_empty,                                 // lcd_tft_isr(void);
    isr_empty,                                 // lcd_tft_err_isr(void);
    isr_empty,                                 // dma2d_isr(void);
    isr_empty,                                 // sai2_isr(void);
    isr_empty,                                 // quadspi_isr(void);
    isr_empty,                                 // i2c4_ev_isr(void);
    isr_empty,                                 // i2c4_er_isr(void);
    isr_empty,                                 // spdifrx_isr(void);

};
