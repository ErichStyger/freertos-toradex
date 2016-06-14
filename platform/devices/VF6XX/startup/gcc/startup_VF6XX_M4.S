/* ---------------------------------------------------------------------------------------*/
/*  @file:    startup_VF6XX_M4.s                                                        */
/*  @purpose: CMSIS Cortex-M4 Core Device Startup File                                    */
/*            VF6XX_M4                                                                    */
/* ---------------------------------------------------------------------------------------*/
/*                                                                                        */
/* Copyright (c) 2015, Freescale Semiconductor, Inc.                                      */
/* Copyright (c) 2016, Toradex AG                                                   */
/* All rights reserved.                                                                   */
/*                                                                                        */
/* Redistribution and use in source and binary forms, with or without modification,       */
/* are permitted provided that the following conditions are met:                          */
/*                                                                                        */
/* o Redistributions of source code must retain the above copyright notice, this list     */
/*   of conditions and the following disclaimer.                                          */
/*                                                                                        */
/* o Redistributions in binary form must reproduce the above copyright notice, this       */
/*   list of conditions and the following disclaimer in the documentation and/or          */
/*   other materials provided with the distribution.                                      */
/*                                                                                        */
/* o Neither the name of Freescale Semiconductor, Inc. nor the names of its               */
/*   contributors may be used to endorse or promote products derived from this            */
/*   software without specific prior written permission.                                  */
/*                                                                                        */
/* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND        */
/* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED          */
/* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                 */
/* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR       */
/* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES         */
/* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;           */
/* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON         */
/* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                */
/* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS          */
/* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                           */
/*****************************************************************************/
/* Version: GCC for ARM Embedded Processors                                  */
/*****************************************************************************/

    .word __etext
    .word __data_start__
    .word __data_end__
    .word __bss_end__


    .syntax unified
    .arch armv7-m

    .section .isr_vector, "a"
    .align 2
    .globl __isr_vector
__isr_vector:
    .long   __stack			/* Initial Stack pointer */
    .long   Reset_Handler		/* Reset Handler */
    .long   NMI_Handler			/* NMI Handler */
    .long   HardFault_Handler		/* Hard Fault Handler */
    .long   MemManage_Handler		/* MPU Fault Handler */
    .long   BusFault_Handler		/* Bus Fault Handler */
    .long   UsageFault_Handler		/* Usage Fault Handler */
    .long   0				/* Reserved */
    .long   0				/* Reserved */
    .long   0				/* Reserved */
    .long   0				/* Reserved */
    .long   SVC_Handler			/* SV Call */
    .long   DebugMon_Handler		/* Debug Monitor Handler */
    .long   0				/* Reserved */
    .long   PendSV_Handler		/* PendableSrvReq Handler */
    .long   SysTick_Handler		/* Sys Tick Handler */

/* External Interrupts */
    .long   CPU2CPUInt0_Handler
    .long   CPU2CPUInt1_Handler
    .long   CPU2CPUInt2_Handler
    .long   CPU2CPUInt3_Handler
    .long   DIRECTED0_SEMA4_Handler
    .long   DIRECTED1_MCM_Handler
    .long   DIRECTED2_Handler
    .long   DIRECTED3_Handler
    .long   DMA0_Handler
    .long   DMA0_ERROR_Handler
    .long   DMA1_Handler
    .long   DMA1_ERROR_Handler
    .long   0
    .long   0
    .long   MSCM_ECC0_Handler
    .long   MSCM_ECC1_Handler
    .long   CSU_ALARM_Handler
    .long   0
    .long   MSCM_ACTZS_Handler
    .long   0
    .long   WDOG_A5_Handler
    .long   WDOG_M4_Handler
    .long   WDOG_SNVS_Handler
    .long   CP1_BOOT_FAIL_Handler
    .long   QSPI0_Handler
    .long   QSPI1_Handler
    .long   DDRMC_Handler
    .long   SDHC0_Handler
    .long   SDHC1_Handler
    .long   0
    .long   DCU0_Handler
    .long   DCU1_Handler
    .long   VIU_Handler
    .long   0
    .long   0
    .long   RLE_Handler
    .long   SEG_LCD_Handler
    .long   0
    .long   0
    .long   PIT_Handler
    .long   LPTIMER0_Handler
    .long   0
    .long   FLEXTIMER0_Handler
    .long   FLEXTIMER1_Handler
    .long   FLEXTIMER3_Handler
    .long   0
    .long   0
    .long   0
    .long   0
    .long   USBPHY0_Handler
    .long   USBPHY1_Handler
    .long   0
    .long   ADC0_Handler
    .long   ADC1_Handler
    .long   DAC0_Handler
    .long   DAC1_Handler
    .long   0
    .long   FLEXCAN0_Handler
    .long   FLEXCAN1_Handler
    .long   0
    .long   UART0_Handler
    .long   UART1_Handler
    .long   UART2_Handler
    .long   UART3_Handler
    .long   UART4_Handler
    .long   UART5_Handler
    .long   SPI0_Handler
    .long   SPI1_Handler
    .long   SPI2_Handler
    .long   SPI3_Handler
    .long   I2C0_Handler
    .long   I2C1_Handler
    .long   I2C2_Handler
    .long   I2C3_Handler
    .long   USBC0_Handler
    .long   USBC1_Handler
    .long   0
    .long   ENET0_Handler
    .long   ENET1_Handler
    .long   ENET0_1588_Handler
    .long   ENET1_1588_Handler
    .long   ENET_SWITCH_Handler
    .long   NFC_Handler
    .long   SAI0_Handler
    .long   SAI1_Handler
    .long   SAI2_Handler
    .long   SAI3_Handler
    .long   ESAI_BIFIFO_Handler
    .long   SPDIF_Handler
    .long   ASRC_Handler
    .long   VREG_Handler
    .long   WKPU0_Handler
    .long   0
    .long   CCM_FXOSC_Handler
    .long   CCM_Handler
    .long   SRC_Handler
    .long   PDB_Handler
    .long   EWM_Handler
    .long   0
    .long   0
    .long   0
    .long   0
    .long   0
    .long   0
    .long   0
    .long   0
    .long   GPIO0_Handler
    .long   GPIO1_Handler
    .long   GPIO2_Handler
    .long   GPIO3_Handler
    .long   GPIO4_Handler
    .size    __isr_vector, . - __isr_vector


    .text
    .thumb

/* Reset Handler */
    .thumb_func
    .align 2
    .globl   Reset_Handler
    .weak    Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
   /*
    * For Vybrid we need to set the stack pointer manually
    * since the boot ROM has its own stack
    */
    ldr sp,=__stack;
	cpsid	i
#ifndef __NO_SYSTEM_INIT
    bl SystemInit
#endif
    /* data copy */
    ldr r0,=__DATA_ROM
    subs r0,r0,#0x1
    ldr r1,=__data_start__
    subs r1,r1,#0x1
    ldr r2,=__data_end__
    subs r3,r2,r1
    b  Copy_init_data
  Loop_copy_init_data:
    adds r1,r1,#0x1
    adds r0,r0,#0x1
    ldrb r4,[r0]
    str r4,[r1]

  Copy_init_data:
    subs r3,r3,#0x1
    cmp r3,#0x0
    bne Loop_copy_init_data

    cpsie   i               /* Unmask interrupts */
    bl _start
    .pool
    .size Reset_Handler, . - Reset_Handler

    .align    1
    .thumb_func
    .weak DefaultISR
    .type DefaultISR, %function
DefaultISR:
    b    DefaultISR
    .size DefaultISR, . - DefaultISR

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro def_irq_handler    handler_name
    .weak \handler_name
    .set  \handler_name, DefaultISR
    .endm

/* Exception Handlers */
    def_irq_handler	NMI_Handler
    def_irq_handler	HardFault_Handler
    def_irq_handler	MemManage_Handler
    def_irq_handler	BusFault_Handler
    def_irq_handler	UsageFault_Handler
    def_irq_handler	SVC_Handler
    def_irq_handler	DebugMon_Handler
    def_irq_handler	PendSV_Handler
    def_irq_handler	SysTick_Handler
    def_irq_handler	CPU2CPUInt0_Handler
    def_irq_handler	CPU2CPUInt1_Handler
    def_irq_handler	CPU2CPUInt2_Handler
    def_irq_handler	CPU2CPUInt3_Handler
    def_irq_handler	DIRECTED0_SEMA4_Handler
    def_irq_handler	DIRECTED1_MCM_Handler
    def_irq_handler	DIRECTED2_Handler
    def_irq_handler	DIRECTED3_Handler
    def_irq_handler	DMA0_Handler
    def_irq_handler	DMA0_ERROR_Handler
    def_irq_handler	DMA1_Handler
    def_irq_handler	DMA1_ERROR_Handler
    def_irq_handler	MSCM_ECC0_Handler
    def_irq_handler	MSCM_ECC1_Handler
    def_irq_handler	CSU_ALARM_Handler
    def_irq_handler	MSCM_ACTZS_Handler
    def_irq_handler	WDOG_A5_Handler
    def_irq_handler	WDOG_M4_Handler
    def_irq_handler	WDOG_SNVS_Handler
    def_irq_handler	CP1_BOOT_FAIL_Handler
    def_irq_handler	QSPI0_Handler
    def_irq_handler	QSPI1_Handler
    def_irq_handler	DDRMC_Handler
    def_irq_handler	SDHC0_Handler
    def_irq_handler	SDHC1_Handler
    def_irq_handler	DCU0_Handler
    def_irq_handler	DCU1_Handler
    def_irq_handler	VIU_Handler
    def_irq_handler	RLE_Handler
    def_irq_handler	SEG_LCD_Handler
    def_irq_handler	PIT_Handler
    def_irq_handler	LPTIMER0_Handler
    def_irq_handler	FLEXTIMER0_Handler
    def_irq_handler	FLEXTIMER1_Handler
    def_irq_handler	FLEXTIMER2_Handler
    def_irq_handler	FLEXTIMER3_Handler
    def_irq_handler	USBPHY0_Handler
    def_irq_handler	USBPHY1_Handler
    def_irq_handler	ADC0_Handler
    def_irq_handler	ADC1_Handler
    def_irq_handler	DAC0_Handler
    def_irq_handler	DAC1_Handler
    def_irq_handler	FLEXCAN0_Handler
    def_irq_handler	FLEXCAN1_Handler
    def_irq_handler	UART0_Handler
    def_irq_handler	UART1_Handler
    def_irq_handler	UART2_Handler
    def_irq_handler	UART3_Handler
    def_irq_handler	UART4_Handler
    def_irq_handler	UART5_Handler
    def_irq_handler	SPI0_Handler
    def_irq_handler	SPI1_Handler
    def_irq_handler	SPI2_Handler
    def_irq_handler	SPI3_Handler
    def_irq_handler	I2C0_Handler
    def_irq_handler	I2C1_Handler
    def_irq_handler	I2C2_Handler
    def_irq_handler	I2C3_Handler
    def_irq_handler	USBC0_Handler
    def_irq_handler	USBC1_Handler
    def_irq_handler	ENET0_Handler
    def_irq_handler	ENET1_Handler
    def_irq_handler	ENET0_1588_Handler
    def_irq_handler	ENET1_1588_Handler
    def_irq_handler	ENET_SWITCH_Handler
    def_irq_handler	NFC_Handler
    def_irq_handler	SAI0_Handler
    def_irq_handler	SAI1_Handler
    def_irq_handler	SAI2_Handler
    def_irq_handler	SAI3_Handler
    def_irq_handler	ESAI_BIFIFO_Handler
    def_irq_handler	SPDIF_Handler
    def_irq_handler	ASRC_Handler
    def_irq_handler	VREG_Handler
    def_irq_handler	WKPU0_Handler
    def_irq_handler	CCM_FXOSC_Handler
    def_irq_handler	CCM_Handler
    def_irq_handler	SRC_Handler
    def_irq_handler	PDB_Handler
    def_irq_handler	EWM_Handler
    def_irq_handler	GPIO0_Handler
    def_irq_handler	GPIO1_Handler
    def_irq_handler	GPIO2_Handler
    def_irq_handler	GPIO3_Handler
    def_irq_handler	GPIO4_Handler
    .end
