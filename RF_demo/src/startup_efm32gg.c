/**
*****************************************************************************
**
**  File        : startup.c
**
**  Abstract    : This file contains interrupt vector and startup code.
**
**  Functions   : Reset_Handler
**
**  Target      : Energy Micro EFM32 Giant Gecko microcontrollers
**
**  Environment : Atollic TrueSTUDIO(R)
**
**  Distribution: The file is distributed “as is,” without any warranty
**                of any kind.
**
**  (c)Copyright Atollic AB.
**  You may use this file as-is or modify it according to the needs of your
**  project. Distribution of this file (unmodified or modified) is not
**  permitted. Atollic AB permit registered Atollic TrueSTUDIO(R) users the
**  rights to distribute the assembled, compiled & linked contents of this
**  file as part of an application binary file, provided that it is built
**  using the Atollic TrueSTUDIO(R) toolchain.
**
*****************************************************************************
*/

/**
**===========================================================================
**  Revisions
**===========================================================================
**       Date        Modification
**       2011-11-11  First issue.
**===========================================================================
*/

/**
**===========================================================================
**  External declarations
**===========================================================================
*/
extern unsigned long _sdata, _edata, _sidata, _sbss, _ebss;
extern unsigned long _estack;
extern void __libc_init_array();
extern void SystemInit();
extern void main();

/**
**===========================================================================
**  Default interrupt handler
**===========================================================================
*/
void Default_Handler()
{
	/* Hang here */
	while(1)
	{
	}
}

/**
**===========================================================================
**  Reset handler
**===========================================================================
*/
__attribute__((naked)) void Reset_Handler()
{
	/* Data and BSS variables */
	unsigned long *srcdata, *dstdata, *sbss;

	/* Set up the stack pointer */
	asm("ldr sp,=_estack\n\t");

	/* Initialize System */
	SystemInit();

	srcdata = &_sidata;
	dstdata = &_sdata;
	sbss = &_sbss;

	/* Copy data */
	while(dstdata != &_edata)
	{
		*(dstdata++) = *(srcdata++);
	}

	/* Clear BSS */
	while(sbss != &_ebss)
	{
		*(sbss++) = '\0';
	}

	/* Run static constructors */
	__libc_init_array();

	/* Jump to main */
	main();

	/* In case main returns, use default handler */
	Default_Handler();
}


/**
**===========================================================================
**  Weak definitions of handlers point to Default_Handler if not implemented
**===========================================================================
*/
void NMI_Handler() __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler() __attribute__ ((weak, alias("Default_Handler")));
void MemManage_Handler() __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler() __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler() __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler() __attribute__ ((weak, alias("Default_Handler")));
void DebugMonitor_Handler() __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler() __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler() __attribute__ ((weak, alias("Default_Handler")));
void DMA_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void GPIO_EVEN_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void TIMER0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void USART0_RX_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void USART0_TX_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void USB_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void ACMP0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void ADC0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void DAC0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void I2C0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void I2C1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void GPIO_ODD_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void TIMER1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void TIMER2_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void TIMER3_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void USART1_RX_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void USART1_TX_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void LESENSE_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void USART2_RX_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void USART2_TX_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void UART0_RX_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void UART0_TX_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void UART1_RX_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void UART1_TX_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void LEUART0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void LEUART1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void LETIMER0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void PCNT0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void PCNT1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void PCNT2_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void RTC_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void BURTC_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void CMU_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void VCMP_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void LCD_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void MSC_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void AES_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void EBI_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void EMU_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));

/**
**===========================================================================
**  Interrupt Vector Table
**===========================================================================
*/
void (* const InterruptVector[])() __attribute__ ((section(".isr_vector"))) = {
    /* Processor exceptions */
    (void (*)(void))&_estack,  /* 0 - Stack pointer */
    Reset_Handler,             /* 1 - Reset */
    NMI_Handler,               /* 2 - NMI  */
    HardFault_Handler,         /* 3 - Hard fault */
    MemManage_Handler,         /* 4 - Memory management fault */
    BusFault_Handler,          /* 5 - Bus fault */
    UsageFault_Handler,        /* 6 - Usage fault */
    0,                         /* 7 - Reserved */
    0,                         /* 8 - Reserved */
    0,                         /* 9 - Reserved */
    0,                         /* 10 - Reserved */
    SVC_Handler,               /* 11 - SVCall */
    DebugMonitor_Handler,      /* 12 - Reservered for Debug */
    0,                         /* 13 - Reserved */
    PendSV_Handler,            /* 14 - PendSV */
    SysTick_Handler,           /* 15 - Systick */
    /* External Interrupts */
    DMA_IRQHandler,            /* 0 - DMA */
    GPIO_EVEN_IRQHandler,      /* 1 - GPIO_EVEN */
    TIMER0_IRQHandler,         /* 2 - TIMER0 */
    USART0_RX_IRQHandler,      /* 3 - USART0_RX */
    USART0_TX_IRQHandler,      /* 4 - USART0_TX */
    USB_IRQHandler,            /* 5 - USB */
    ACMP0_IRQHandler,          /* 6 - ACMP0 */
    ADC0_IRQHandler,           /* 7 - ADC0 */
    DAC0_IRQHandler,           /* 8 - DAC0 */
    I2C0_IRQHandler,           /* 9 - I2C0 */
    I2C1_IRQHandler,           /* 10 - I2C1 */
    GPIO_ODD_IRQHandler,       /* 11 - GPIO_ODD */
    TIMER1_IRQHandler,         /* 12 - TIMER1 */
    TIMER2_IRQHandler,         /* 13 - TIMER2 */
    TIMER3_IRQHandler,         /* 14 - TIMER3 */
    USART1_RX_IRQHandler,      /* 15 - USART1_RX */
    USART1_TX_IRQHandler,      /* 16 - USART1_TX */
    LESENSE_IRQHandler,        /* 17 - LESENSE */
    USART2_RX_IRQHandler,      /* 18 - USART2_RX */
    USART2_TX_IRQHandler,      /* 19 - USART2_TX */
    UART0_RX_IRQHandler,       /* 20 - UART0_RX */
    UART0_TX_IRQHandler,       /* 21 - UART0_TX */
    UART1_RX_IRQHandler,       /* 22 - UART1_RX */
    UART1_TX_IRQHandler,       /* 23 - UART1_TX */
    LEUART0_IRQHandler,        /* 24 - LEUART0 */
    LEUART1_IRQHandler,        /* 25 - LEUART1 */
    LETIMER0_IRQHandler,       /* 26 - LETIMER0 */
    PCNT0_IRQHandler,          /* 27 - PCNT0 */
    PCNT1_IRQHandler,          /* 28 - PCNT1 */
    PCNT2_IRQHandler,          /* 29 - PCNT2 */
    RTC_IRQHandler,            /* 30 - RTC */
    BURTC_IRQHandler,          /* 31 - BURTC */
    CMU_IRQHandler,            /* 32 - CMU */
    VCMP_IRQHandler,           /* 33 - VCMP */
    LCD_IRQHandler,            /* 34 - LCD */
    MSC_IRQHandler,            /* 35 - MSC */
    AES_IRQHandler,            /* 36 - AES */
    EBI_IRQHandler,            /* 37 - EBI */
    EMU_IRQHandler             /* 38 - EMU */
};
