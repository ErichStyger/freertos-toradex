/*
 *     Copyright (c) 1997 - 2015 Freescale Semiconductor, Inc.
 *     Copyright (c) 2016, Toradex AG
 *     All rights reserved.
 *
 *     Redistribution and use in source and binary forms, with or without modification,
 *     are permitted provided that the following conditions are met:
 *
 *     o Redistributions of source code must retain the above copyright notice, this list
 *       of conditions and the following disclaimer.
 *
 *     o Redistributions in binary form must reproduce the above copyright notice, this
 *       list of conditions and the following disclaimer in the documentation and/or
 *       other materials provided with the distribution.
 *
 *     o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from this
 *       software without specific prior written permission.
 *
 *     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 *     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** Interrupt Number Definitions */
#define NUMBER_OF_INT_VECTORS			16		/**< Number of interrupts in the Vector table */

typedef enum IRQn {
	/* Auxiliary constants */
	NotAvail_IRQn				= -128,		/**< Not available device specific interrupt */

	/* Core interrupts */
	NonMaskableInt_IRQn			= -14,		/**< Non Maskable Interrupt */
	HardFault_IRQn				= -13,		/**< Cortex-M4 SV Hard Fault Interrupt */
	MemoryManagement_IRQn			= -12,		/**< Cortex-M4 Memory Management Interrupt */
	BusFault_IRQn				= -11,		/**< Cortex-M4 Bus Fault Interrupt */
	UsageFault_IRQn				= -10,		/**< Cortex-M4 Usage Fault Interrupt */
	SVCall_IRQn				= -5,		/**< Cortex-M4 SV Call Interrupt */
	DebugMonitor_IRQn			= -4,		/**< Cortex-M4 Debug Monitor Interrupt */
	PendSV_IRQn				= -2,		/**< Cortex-M4 Pend SV Interrupt */
	SysTick_IRQn				= -1,		/**< Cortex-M4 System Tick Interrupt */

	/* Device specific interrupts */
	CPU2CPU_INT0_IRQ			= 0,
	CPU2CPU_INT1_IRQ			= 1,
	CPU2CPU_INT2_IRQ			= 2,
	CPU2CPU_INT3_IRQ			= 3,
	DIRECTED0_SEMA4_IRQ			= 4,
	DIRECTED1_MCM_IRQ			= 5,
	DIRECTED2_IRQ				= 6,
	DIRECTED3_IRQ				= 7,
	DMA0_IRQ				= 8,
	DMA0_ERROR_IRQ				= 9,
	DMA1_IRQ				= 10,
	DMA1_ERROR_IRQ				= 11,
	RESERVED0_IRQ				= 12,
	RESERVED1_IRQ				= 13,
	MSCM_ECC0_IRQ				= 14,
	MSCM_ECC1_IRQ				= 15,
	CSU_ALARM_IRQ				= 16,
	RESERVED2_IRQ				= 17,
	MSCM_ACTZS_IRQ				= 18,
	RESERVED3_IRQ				= 19,
	WDOG_A5_IRQ				= 20,
	WDOG_M4_IRQ				= 21,
	WDOG_SNVS_IRQ				= 22,
	CP1_BOOT_FAIL_IRQ			= 23,
	QSPI0_IRQ				= 24,
	QSPI1_IRQ				= 25,
	DDRMC_IRQ				= 26,
	SDHC0_IRQ				= 27,
	SDHC1_IRQ				= 28,
	RESERVED4_IRQ				= 29,
	DCU0_IRQ				= 30,
	DCU1_IRQ				= 31,
	VIU_IRQ					= 32,
	RESERVED5_IRQ				= 33,
	RESERVED6_IRQ				= 34,
	RLE_IRQ					= 35,
	SEG_LCD_IRQ				= 36,
	RESERVED7_IRQ				= 37,
	RESERVED8_IRQ				= 38,
	PIT_IRQ					= 39,
	LPTIMER0_IRQ				= 40,
	RESERVED9_IRQ				= 41,
	FLEXTIMER0_IRQ				= 42,
	FLEXTIMER1_IRQ				= 43,
	FLEXTIMER2_IRQ				= 44,
	FLEXTIMER3_IRQ				= 45,
	RESERVED10_IRQ				= 46,
	RESERVED11_IRQ				= 47,
	RESERVED12_IRQ				= 48,
	RESERVED13_IRQ				= 49,
	USBPHY0_IRQ				= 50,
	USBPHY1_IRQ				= 51,
	RESERVED14_IRQ				= 52,
	ADC0_IRQ				= 53,
	ADC1_IRQ				= 54,
	DAC0_IRQ				= 55,
	DAC1_IRQ				= 56,
	RESERVED15_IRQ				= 57,
	FLEXCAN0_IRQ				= 58,
	FLEXCAN1_IRQ				= 59,
	RESERVED16_IRQ				= 60,
	UART0_IRQ				= 61,
	UART1_IRQ				= 62,
	UART2_IRQ				= 63,
	UART3_IRQ				= 64,
	UART4_IRQ				= 65,
	UART5_IRQ				= 66,
	SPI0_IRQ				= 67,
	SPI1_IRQ				= 68,
	SPI2_IRQ				= 69,
	SPI3_IRQ				= 70,
	I2C0_IRQ				= 71,
	I2C1_IRQ				= 72,
	I2C2_IRQ				= 73,
	I2C3_IRQ				= 74,
	USBC0_IRQ				= 75,
	USBC1_IRQ				= 76,
	RESERVED17_IRQ				= 77,
	ENET0_IRQ				= 78,
	ENET1_IRQ				= 79,
	ENET0_1588_IRQ				= 80,
	ENET1_1588_IRQ				= 81,
	ENET_SWITCH_IRQ				= 82,
	NFC_IRQ					= 83,
	SAI0_IRQ				= 84,
	SAI1_IRQ				= 85,
	SAI2_IRQ				= 86,
	SAI3_IRQ				= 87,
	ESAI_BIFIFO_IRQ				= 88,
	SPDIF_IRQ				= 89,
	ASRC_IRQ				= 90,
	VREG_IRQ				= 91,
	WKPU0_IRQ				= 92,
	RESERVED18_IRQ				= 93,
	CCM_FXOSC_IRQ				= 94,
	CCM_IRQ					= 95,
	SRC_IRQ					= 96,
	PDB_IRQ					= 97,
	EWM_IRQ					= 98,
	RESERVED19_IRQ				= 99,
	RESERVED20_IRQ				= 100,
	RESERVED21_IRQ				= 101,
	RESERVED22_IRQ				= 102,
	RESERVED23_IRQ				= 103,
	RESERVED24_IRQ				= 104,
	RESERVED25_IRQ				= 105,
	RESERVED26_IRQ				= 106,
	GPIO0_IRQ				= 107,
	GPIO1_IRQ				= 108,
	GPIO2_IRQ				= 109,
	GPIO3_IRQ				= 110,
	GPIO4_IRQ				= 111,
} IRQn_Type;

/* ----------------------------------------------------------------------------
   -- Cortex M4 Core Configuration
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Cortex_Core_Configuration Cortex M4 Core Configuration
 * @{
 */

#define __MPU_PRESENT			0         /**< Defines if an MPU is present or not */
#define __NVIC_PRIO_BITS		4         /**< Number of priority bits implemented in the NVIC */
#define __Vendor_SysTickConfig		0         /**< Vendor specific implementation of SysTickConfig is defined */
#define __FPU_PRESENT			1         /**< Defines if an FPU is present or not */

#include "core_cm4.h"                  /* Core Peripheral Access Layer */

/* Memory map for all busses */
#define PERIPH_BASE			(0x40000000U)
#define PERIPH_BASE_AIPS0		(PERIPH_BASE + 0x00000)
#define PERIPH_BASE_AIPS1		(PERIPH_BASE + 0x80000)

/* Pheripheral addresses */

/* AIPS0 */
#define MSCM_BASE			(PERIPH_BASE_AIPS0 + 0x01000)
#define SEMA4_BASE			(PERIPH_BASE_AIPS0 + 0x1D000)
#define UART0_BASE			(PERIPH_BASE_AIPS0 + 0x27000)
#define UART1_BASE			(PERIPH_BASE_AIPS0 + 0x28000)
#define UART2_BASE			(PERIPH_BASE_AIPS0 + 0x29000)
#define UART3_BASE			(PERIPH_BASE_AIPS0 + 0x2A000)
#define SPI0_BASE			(PERIPH_BASE_AIPS0 + 0x2C000)
#define SPI1_BASE			(PERIPH_BASE_AIPS0 + 0x2D000)
#define IOMUXC_BASE			(PERIPH_BASE_AIPS0 + 0x48000)
#define PORTA_MUX_BASE			(PERIPH_BASE_AIPS0 + 0x49000)
#define PORTB_MUX_BASE			(PERIPH_BASE_AIPS0 + 0x4A000)
#define PORTC_MUX_BASE			(PERIPH_BASE_AIPS0 + 0x4B000)
#define PORTD_MUX_BASE			(PERIPH_BASE_AIPS0 + 0x4C000)
#define PORTE_MUX_BASE			(PERIPH_BASE_AIPS0 + 0x4D000)
#define ANADIG_BASE			(PERIPH_BASE_AIPS0 + 0x50000)
#define CCM_BASE			(PERIPH_BASE_AIPS0 + 0x6B000)

/* AIPS1 */
#define UART4_BASE			(PERIPH_BASE_AIPS1 + 0x29000)
#define UART5_BASE			(PERIPH_BASE_AIPS1 + 0x2A000)
#define GPIO_BASE			(PERIPH_BASE + 0xff000)

#define GPIO0_BASE			(GPIO_BASE + 0x000)
#define GPIO1_BASE			(GPIO_BASE + 0x040)
#define GPIO2_BASE			(GPIO_BASE + 0x080)
#define GPIO3_BASE			(GPIO_BASE + 0x0C0)
#define GPIO4_BASE			(GPIO_BASE + 0x100)

/* --- ANADIG registers ---------------------------------------------------- */

/** ANADIG - Register Layout Typedef */
typedef struct {
	uint8_t RESERVED_0[16];
	__IO  uint32_t	ANADIG_PLL3_CTRL;
	uint8_t RESERVED_1[12];
	__IO  uint32_t	ANADIG_PLL7_CTRL;
	uint8_t RESERVED_2[12];
	__IO  uint32_t	ANADIG_PLL2_CTRL;
	uint8_t RESERVED_3[12];
	__IO  uint32_t	ANADIG_PLL2_SS;
	uint8_t RESERVED_4[12];
	__IO  uint32_t	ANADIG_PLL2_NUM;
	uint8_t RESERVED_5[12];
	__IO  uint32_t	ANADIG_PLL2_DENOM;
	uint8_t RESERVED_6[12];
	__IO  uint32_t	ANADIG_PLL4_CTRL;
	uint8_t RESERVED_7[12];
	__IO  uint32_t	ANADIG_PLL4_NUM;
	uint8_t RESERVED_8[12];
	__IO  uint32_t	ANADIG_PLL4_DENOM;
	uint8_t RESERVED_9[12];
	__IO  uint32_t	ANADIG_PLL6_CTRL;
	uint8_t RESERVED_10[12];
	__IO  uint32_t	ANADIG_PLL6_NUM;
	uint8_t RESERVED_12[12];
	__IO  uint32_t	ANADIG_PLL6_DENOM;
	uint8_t RESERVED_13[28];
	__IO  uint32_t	ANADIG_PLL5_CTRL;
	uint8_t RESERVED_14[12];
	__IO  uint32_t	ANADIG_PLL3_PFD;
	uint8_t RESERVED_15[12];
	__IO  uint32_t	ANADIG_PLL2_PFD;
	uint8_t RESERVED_16[12];
	__IO  uint32_t	ANADIG_1P1;
	uint8_t RESERVED_17[12];
	__IO  uint32_t	ANADIG_REGU_3P0;
	uint8_t RESERVED_18[12];
	__IO  uint32_t	ANADIG_REGU_2P5;
	uint8_t RESERVED_19[28];
	__IO  uint32_t	ANADIG_ANA_MISC0;
	uint8_t RESERVED_20[12];
	__IO  uint32_t	ANADIG_ANA_MISC1;
	uint8_t RESERVED_21[252];
	__IO  uint32_t	ANADIG_DIGPROG;
	uint8_t RESERVED_22[12];
	__IO  uint32_t	ANADIG_PLL1_CTRL;
	uint8_t RESERVED_23[12];
	__IO  uint32_t	ANADIG_PLL1_SS;
	uint8_t RESERVED_24[12];
	__IO  uint32_t	ANADIG_PLL1_NUM;
	uint8_t RESERVED_25[12];
	__IO  uint32_t	ANADIG_PLL1_DENOM;
	uint8_t RESERVED_26[12];
	__IO  uint32_t	ANADIG_PLL1_PFD;
	uint8_t RESERVED_27[12];
	__IO  uint32_t	ANADIG_PLL_LOCK;
} ANADIG_Type, *ANADIG_MemMapPtr;

#define ANADIG_PLL3_CTRL_REG(base)			((base)->ANADIG_PLL3_CTRL)
#define ANADIG_PLL7_CTRL_REG(base)			((base)->ANADIG_PLL7_CTRL)
#define ANADIG_PLL2_CTRL_REG(base)			((base)->ANADIG_PLL2_CTRL)
#define ANADIG_PLL2_SS_REG(base)			((base)->ANADIG_PLL2_SS)
#define ANADIG_PLL2_NUM_REG(base)			((base)->ANADIG_PLL2_NUM)
#define ANADIG_PLL2_DENOM_REG(base)			((base)->ANADIG_PLL2_DENOM)
#define ANADIG_PLL4_CTRL_REG(base)			((base)->ANADIG_PLL4_CTRL)
#define ANADIG_PLL4_NUM_REG(base)			((base)->ANADIG_PLL4_NUM)
#define ANADIG_PLL4_DENOM_REG(base)			((base)->ANADIG_PLL4_DENOM)
#define ANADIG_PLL6_CTRL_REG(base)			((base)->ANADIG_PLL6_CTRL)
#define ANADIG_PLL6_NUM_REG(base)			((base)->ANADIG_PLL6_NUM)
#define ANADIG_PLL6_DENOM_REG(base)			((base)->ANADIG_PLL6_DENOM)
#define ANADIG_PLL5_CTRL_REG(base)			((base)->ANADIG_PLL5_CTRL)
#define ANADIG_PLL3_PFD_REG(base)			((base)->ANADIG_PLL3_PFD)
#define ANADIG_PLL2_PFD_REG(base)			((base)->ANADIG_PLL2_PFD)
#define ANADIG_REGU_1P1_REG(base)			((base)->ANADIG_REGU_1P1)
#define ANADIG_REGU_3P0_REG(base)			((base)->ANADIG_REGU_3P0)
#define ANADIG_REGU_2P5_REG(base)			((base)->ANADIG_REGU_2P5)
#define ANADIG_ANA_MISC0_REG(base)			((base)->ANADIG_ANA_MISC0)
#define ANADIG_ANA_MISC1_REG(base)			((base)->ANADIG_ANA_MISC1)
#define ANADIG_DIGPROG_REG(base)			((base)->ANADIG_DIGPROG)
#define ANADIG_PLL1_CTRL_REG(base)			((base)->ANADIG_PLL1_CTRL)
#define ANADIG_PLL1_SS_REG(base)			((base)->ANADIG_PLL1_SS)
#define ANADIG_PLL1_NUM_REG(base)			((base)->ANADIG_PLL1_NUM)
#define ANADIG_PLL1_DENOM_REG(base)			((base)->ANADIG_PLL1_DENOM)
#define ANADIG_PLL1_PFD_REG(base)			((base)->ANADIG_PLL1_PFD)
#define ANADIG_PLL_LOCK_REG(base)			((base)->ANADIG_PLL_LOCK)

/* ANADIG - Peripheral instance base addresses */
/** Peripheral CCM_ANALOG base pointer */
#define ANADIG                               ((ANADIG_Type *)ANADIG_BASE)
#define ANADIG_BASE_PTR                      (ANADIG)
/** Array initializer of CCM_ANALOG peripheral base adresses */
#define ANADIG_BASE_ADDRS                    { ANADIG_BASE }
/** Array initializer of CCM_ANALOG peripheral base pointers */
#define ANADIG_BASE_PTRS                     { ANADIG }

/* ANADIG - Register instance definitions */
#define  ANADIG_PLL3_CTRL		ANADIG_PLL3_CTRL_REG(ANADIG_BASE_PTR)
#define  ANADIG_PLL7_CTRL		ANADIG_PLL7_CTRL_REG(ANADIG_BASE_PTR)
#define  ANADIG_PLL2_CTRL		ANADIG_PLL2_CTRL_REG(ANADIG_BASE_PTR)
#define  ANADIG_PLL2_SS		ANADIG_PLL2_SS_REG(ANADIG_BASE_PTR)
#define  ANADIG_PLL2_NUM		ANADIG_PLL2_NUM_REG(ANADIG_BASE_PTR)
#define  ANADIG_PLL2_DENOM		ANADIG_PLL2_DENOM_REG(ANADIG_BASE_PTR)
#define  ANADIG_PLL4_CTRL		ANADIG_PLL4_CTRL_REG(ANADIG_BASE_PTR)
#define  ANADIG_PLL4_NUM		ANADIG_PLL4_NUM_REG(ANADIG_BASE_PTR)
#define  ANADIG_PLL4_DENOM		ANADIG_PLL4_DENOM_REG(ANADIG_BASE_PTR)
#define  ANADIG_PLL6_CTRL		ANADIG_PLL6_CTRL_REG(ANADIG_BASE_PTR)
#define  ANADIG_PLL6_NUM		ANADIG_PLL6_NUM_REG(ANADIG_BASE_PTR))
#define  ANADIG_PLL6_DENOM		ANADIG_PLL6_DENOM_REG(ANADIG_BASE_PTR)
#define  ANADIG_PLL5_CTRL		ANADIG_PLL5_CTRL_REG(ANADIG_BASE_PTR)
#define  ANADIG_PLL3_PFD		ANADIG_PLL3_PFD_REG(ANADIG_BASE_PTR)
#define  ANADIG_PLL2_PFD		ANADIG_PLL2_PFD_REG(ANADIG_BASE_PTR)
#define  ANADIG_REGU_1P1		ANADIG_REGU_1P1_REG(ANADIG_BASE_PTR)
#define  ANADIG__REGU_3P0		ANADIG__REGU_3P0_REG(ANADIG_BASE_PTR)
#define  ANADIG_REGU_2P5		ANADIG_REGU_2P5_REG(ANADIG_BASE_PTR)
#define  ANADIG_ANA_MISC0		ANADIG_ANA_MISC0_REG(ANADIG_BASE_PTR)
#define  ANADIG_ANA_MISC1		ANADIG_ANA_MISC1_REG(ANADIG_BASE_PTR)
#define  ANADIG_DIGPROG		ANADIG_DIGPROG_REG(ANADIG_BASE_PTR)
#define  ANADIG_PLL1_CTRL		ANADIG_PLL1_CTRL_REG(ANADIG_BASE_PTR)
#define  ANADIG_PLL1_SS		ANADIG_PLL1_SS_REG(ANADIG_BASE_PTR)
#define  ANADIG_PLL1_NUM		ANADIG_PLL1_NUM_REG(ANADIG_BASE_PTR)
#define  ANADIG_PLL1_DENO		ANADIG_PLL1_DENOM_REG(ANADIG_BASE_PTR)
#define  ANADIG_PLL1_PFD		ANADIG_PLL1_PFD_REG(ANADIG_BASE_PTR)
#define  ANADIG_PLL_LOCK		ANADIG_PLL_LOCK_REG(ANADIG_BASE_PTR)

/* --- ANADIG values -....-------------------------------------------------- */

/* ANADIG_PLL3_CTRL: PLL3 Control Register (480MHz PLL of USB0) */
#define ANADIG_PLL3_CTRL_LOCK			(1 << 31)
#define ANADIG_PLL3_CTRL_BYPASS		(1 << 16)
#define ANADIG_PLL3_CTRL_BYPASS_CLK_SRC	(1 << 14)
#define ANADIG_PLL3_CTRL_ENABLE		(1 << 13)
#define ANADIG_PLL3_CTRL_POWER			(1 << 12)
#define ANADIG_PLL3_CTRL_EN_USB_CLKS		(1 << 6)
#define ANADIG_PLL3_CTRL_DIV_SELECT		(1 << 1)

/* ANADIG_PLL7_CTRL: PLL7 Control Register (480MHz PLL of USB1) */
#define ANADIG_PLL7_CTRL_LOCK			(1 << 31)
#define ANADIG_PLL7_CTRL_BYPASS		(1 << 16)
#define ANADIG_PLL7_CTRL_BYPASS_CLK_SRC	(1 << 14)
#define ANADIG_PLL7_CTRL_ENABLE		(1 << 13)
#define ANADIG_PLL7_CTRL_POWER			(1 << 12)
#define ANADIG_PLL7_CTRL_EN_USB_CLKS		(1 << 6)
#define ANADIG_PLL7_CTRL_DIV_SELECT		(1 << 1)

/* ANADIG_PLL2_CTRL: PLL2 Control Register (528MHz PLL) */
#define ANADIG_PLL2_CTRL_LOCK			(1 << 31)
#define ANADIG_PLL2_CTRL_PFD_OFFSET_EN		(1 << 18)
#define ANADIG_PLL2_CTRL_DITHER_ENABLE		(1 << 17)
#define ANADIG_PLL2_CTRL_BYPASS		(1 << 16)
#define ANADIG_PLL2_CTRL_BYPASS_CLK_SRC	(1 << 14)
#define ANADIG_PLL2_CTRL_ENABLE		(1 << 13)
#define ANADIG_PLL2_CTRL_POWERDOWN		(1 << 12)
#define ANADIG_PLL2_CTRL_DIV_SELECT		(1 << 1)

/* ANADIG_PLL2_SS: PLL2 Spread Spectrum definition register */
#define ANADIG_PLL2_SS_STOP_MASK		(0xffff << 16)
#define ANADIG_PLL2_SS_ENABLE			(1 << 15)
#define ANADIG_PLL2_SS_STEP_MASK		0x8fff

/* ANADIG_PLL2_NUM: PLL2 Numerator definition register */
#define ANADIG_PLL2_NUM_MFN_MASK		0x3fffffff

/* ANADIG_PLL2_DENOM: PLL2 Denominator definition register */
#define ANADIG_PLL2_DENOM_MFN_MASK		0x3fffffff

/* ANADIG_PLL4_CTRL: PLL4 Control Register (audio PLL) */
#define ANADIG_PLL4_CTRL_LOCK			(1 << 31)
#define ANADIG_PLL4_CTRL_PFD_OFFSET_EN		(1 << 18)
#define ANADIG_PLL4_CTRL_DITHER_ENABLE		(1 << 17)
#define ANADIG_PLL4_CTRL_BYPASS		(1 << 16)
#define ANADIG_PLL4_CTRL_BYPASS_CLK_SRC	(1 << 14)
#define ANADIG_PLL4_CTRL_ENABLE		(1 << 13)
#define ANADIG_PLL4_CTRL_POWERDOWN		(1 << 12)
#define ANADIG_PLL4_CTRL_DIV_SELECT_MASK	(0x7f)

/* ANADIG_PLL4_NUM: PLL4 Numerator definition register */
#define ANADIG_PLL4_NUM_MFN_MASK		0x3fffffff

/* ANADIG_PLL4_DENOM: PLL4 Denominator definition register */
#define ANADIG_PLL4_DENOM_MFN_MASK		0x3fffffff

/* ANADIG_PLL6_CTRL: PLL6 Control Register (video PLL) */
#define ANADIG_PLL6_CTRL_LOCK			(1 << 31)
#define ANADIG_PLL6_CTRL_PFD_OFFSET_EN		(1 << 18)
#define ANADIG_PLL6_CTRL_DITHER_ENABLE		(1 << 17)
#define ANADIG_PLL6_CTRL_BYPASS		(1 << 16)
#define ANADIG_PLL6_CTRL_BYPASS_CLK_SRC	(1 << 14)
#define ANADIG_PLL6_CTRL_ENABLE		(1 << 13)
#define ANADIG_PLL6_CTRL_POWERDOWN		(1 << 12)
#define ANADIG_PLL6_CTRL_DIV_SELECT_MASK	(0x7f)

/* ANADIG_PLL6_NUM: PLL6 Numerator definition register */
#define ANADIG_PLL6_NUM_MFN_MASK		0x3fffffff

/* ANADIG_PLL6_DENOM: PLL6 Denominator definition register */
#define ANADIG_PLL6_DENOM_MFN_MASK		0x3fffffff

/* ANADIG_PLL5_CTRL: PLL5 Control Register (video PLL) */
#define ANADIG_PLL5_CTRL_LOCK			(1 << 31)
#define ANADIG_PLL5_CTRL_PFD_OFFSET_EN		(1 << 18)
#define ANADIG_PLL5_CTRL_DITHER_ENABLE		(1 << 17)
#define ANADIG_PLL5_CTRL_BYPASS		(1 << 16)
#define ANADIG_PLL5_CTRL_BYPASS_CLK_SRC	(1 << 14)
#define ANADIG_PLL5_CTRL_ENABLE		(1 << 13)
#define ANADIG_PLL5_CTRL_POWERDOWN		(1 << 12)
#define ANADIG_PLL5_CTRL_DIV_SELECT_MASK	(0x3)

/* ANADIG_PLL_PFD: PLL1/PLL2/PLL3 PFD Clocks */
#define ANADIG_PLL_PFD4_CLKGATE		(1 << 31)
#define ANADIG_PLL_PFD4_STABLE			(1 << 30)
#define ANADIG_PLL_PFD4_FRAC_SHIFT		24
#define ANADIG_PLL_PFD4_FRAC_MASK		(0x3f << 24)
#define ANADIG_PLL_PFD3_CLKGATE		(1 << 23)
#define ANADIG_PLL_PFD3_STABLE			(1 << 22)
#define ANADIG_PLL_PFD3_FRAC_SHIFT		16
#define ANADIG_PLL_PFD3_FRAC_MASK		(0x3f << 16)
#define ANADIG_PLL_PFD2_CLKGATE		(1 << 15)
#define ANADIG_PLL_PFD2_STABLE			(1 << 14)
#define ANADIG_PLL_PFD2_FRAC_SHIFT		8
#define ANADIG_PLL_PFD2_FRAC_MASK		(0x3f << 8)
#define ANADIG_PLL_PFD1_CLKGATE		(1 << 7)
#define ANADIG_PLL_PFD1_STABLE			(1 << 6)
#define ANADIG_PLL_PFD1_FRAC_SHIFT		0
#define ANADIG_PLL_PFD1_FRAC_MASK		(0x3f << 0)

/* AANADIG_ANA_MISC0: miscellaneous analog blocks */
#define ANADIG_ANA_MISC0_OSC_XTALOK_EN		(1 << 17)
#define ANADIG_ANA_MISC0_OSC_XTALOK		(1 << 16)
#define ANADIG_ANA_MISC0_CLK_24M_IRC_XTAL_SEL	(1 << 13)
#define ANADIG_ANA_MISC0_STOP_MODE_CONFIG	(1 << 12)
#define ANADIG_ANA_MISC0_REFTOP_VBGUP		(1 << 7)
#define ANADIG_ANA_MISC0_REFTOP_SELBIASOFF	(1 << 3)
#define ANADIG_ANA_MISC0_REFTOP_LOWPOWER	(1 << 2)
#define ANADIG_ANA_MISC0_REFTOP_PWDVBGUP	(1 << 1)
#define ANADIG_ANA_MISC0_REFTOP_PWD		(1 << 0)

/* AANADIG_ANA_MISC0: miscellaneous analog blocks */
#define ANADIG_ANA_MISC1_IRQ_ANA_BO		(1 << 30)
#define ANADIG_ANA_MISC1_IRQ_TEMPSENSE		(1 << 29)
#define ANADIG_ANA_MISC1_LVDSCLK1_IBEN		(1 << 12)
#define ANADIG_ANA_MISC1_LVDSCLK1_OBEN		(1 << 10)

/* AANADIG_ANA_DIGPROG: Digital Program register */
#define ANADIG_ANADIG_DIGPROG_MAJOR_MASK	(0xffff << 8)
#define ANADIG_ANADIG_DIGPROG_MINOR_MASK	(0xff << 0)

/* ANADIG_PLL1_CTRL: PLL1 Control Register (video PLL) */
#define ANADIG_PLL1_CTRL_LOCK			(1 << 31)
#define ANADIG_PLL1_CTRL_PFD_OFFSET_EN		(1 << 18)
#define ANADIG_PLL1_CTRL_DITHER_ENABLE		(1 << 17)
#define ANADIG_PLL1_CTRL_BYPASS		(1 << 16)
#define ANADIG_PLL1_CTRL_BYPASS_CLK_SRC	(1 << 14)
#define ANADIG_PLL1_CTRL_ENABLE		(1 << 13)
#define ANADIG_PLL1_CTRL_POWERDOWN		(1 << 12)
#define ANADIG_PLL1_CTRL_DIV_SELECT		(1 << 1)

/* ANADIG_PLL1_SS: PLL1 Spread Spectrum definition register */
#define ANADIG_PLL1_SS_STOP_MASK		(0xffff << 16)
#define ANADIG_PLL1_SS_ENABLE			(1 << 15)
#define ANADIG_PLL1_SS_STEP_MASK		0x8fff

/* ANADIG_PLL1_NUM: PLL1 Numerator definition register */
#define ANADIG_PLL1_NUM_MFN_MASK		0x3fffffff

/* ANADIG_PLL1_DENOM: PLL1 Denominator definition register */
#define ANADIG_PLL1_DENOM_MFN_MASK		0x3fffffff

/* ANADIG_PLL_LOCK: PLL Lock Register */
#define ANADIG_PLL_LOCK_PLL1			(1 << 6)
#define ANADIG_PLL_LOCK_PLL2			(1 << 5)
#define ANADIG_PLL_LOCK_PLL4			(1 << 4)
#define ANADIG_PLL_LOCK_PLL6			(1 << 3)
#define ANADIG_PLL_LOCK_PLL5			(1 << 2)
#define ANADIG_PLL_LOCK_PLL3			(1 << 1)
#define ANADIG_PLL_LOCK_PLL7			(1 << 0)

/** CCM - Register Layout Typedef */
typedef struct {
  __IO  uint32_t CCM_CCR;
  __IO  uint32_t CCM_CSR;
  __IO  uint32_t CCM_CCSR;
  __IO  uint32_t CCM_CACRR;
  __IO  uint32_t CCM_CSCMR1;
  __IO  uint32_t CCM_CSCDR1;
  __IO  uint32_t CCM_CSCDR2;
  __IO  uint32_t CCM_CSCDR3;
  __IO  uint32_t CCM_CSCMR2;
  __IO	uint8_t RESERVED_0[4];
  __IO  uint32_t CCM_CTOR;
  __IO  uint32_t CCM_CLPCR;
  __IO  uint32_t CCM_CISR;
  __IO  uint32_t CCM_CIMR;
  __IO  uint32_t CCM_CCOSR;
  __IO  uint32_t CCM_CGPR;
  __IO  uint32_t CCM_CCGR0;
  __IO  uint32_t CCM_CCGR1;
  __IO  uint32_t CCM_CCGR2;
  __IO  uint32_t CCM_CCGR3;
  __IO  uint32_t CCM_CCGR4;
  __IO  uint32_t CCM_CCGR5;
  __IO  uint32_t CCM_CCGR6;
  __IO  uint32_t CCM_CCGR7;
  __IO  uint32_t CCM_CCGR8;
  __IO  uint32_t CCM_CCGR9;
  __IO  uint32_t CCM_CCGR10;
  __IO  uint32_t CCM_CCGR11;
  __IO  uint32_t CCM_CMEOR0;
  __IO  uint32_t CCM_CMEOR1;
  __IO  uint32_t CCM_CMEOR2;
  __IO  uint32_t CCM_CMEOR3;
  __IO  uint32_t CCM_CMEOR4;
  __IO  uint32_t CCM_CMEOR5;
  __IO  uint32_t CCM_CPPDSR;
  __IO  uint32_t CCM_CCOWR;
  __IO  uint32_t CCM_CCPGR0;
  __IO  uint32_t CCM_CCPGR1;
  __IO  uint32_t CCM_CCPGR2;
  __IO  uint32_t CCM_CCPGR3;
} CCM_Type, *CCM_MemMapPtr;

#define CCM_CCR_REG(base)		((base)->CCM_CCR)
#define CCM_CSR_REG(base)		((base)->CCM_CSR)
#define CCM_CCSR_REG(base)		((base)->CCM_CCSR)
#define CCM_CACRR_REG(base)		((base)->CCM_CACRR)
#define CCM_CSCMR1_REG(base)		((base)->CCM_CACMR1)
#define CCM_CSCDR1_REG(base)		((base)->CCM_CSCDR1)
#define CCM_CSCDR2_REG(base)		((base)->CCM_CSCDR2)
#define CCM_CSCDR3_REG(base)		((base)->CCM_CSCDR3)
#define CCM_CSCMR2_REG(base)		((base)->CCM_CSCMR2)
#define CCM_CTOR_REG(base)		((base)->CCM_CTOR)
#define CCM_CLPCR_REG(base)		((base)->CCM_CLPCR)
#define CCM_CISR_REG(base)		((base)->CCM_CISR)
#define CCM_CIMR_REG(base)		((base)->CCM_CIMR)
#define CCM_CCOSR_REG(base)		((base)->CCM_CCOSR)
#define CCM_CGPR_REG(base)		((base)->CCM_CGRP)
#define CCM_CCGR0_REG(base)		((base)->CCM_CCGR0)
#define CCM_CCGR1_REG(base)		((base)->CCM_CCGR1)
#define CCM_CCGR2_REG(base)		((base)->CCM_CCGR2)
#define CCM_CCGR3_REG(base)		((base)->CCM_CCGR3)
#define CCM_CCGR4_REG(base)		((base)->CCM_CCGR4)
#define CCM_CCGR5_REG(base)		((base)->CCM_CCGR5)
#define CCM_CCGR6_REG(base)		((base)->CCM_CCGR6)
#define CCM_CCGR7_REG(base)		((base)->CCM_CCGR7)
#define CCM_CCGR8_REG(base)		((base)->CCM_CCGR8)
#define CCM_CCGR9_REG(base)		((base)->CCM_CCGR9)
#define CCM_CCGR10_REG(base)		((base)->CCM_CCGR10)
#define CCM_CCGR11_REG(base)		((base)->CCM_CCGR11)
#define CCM_CMEOR0_REG(base)		((base)->CCM_CMEOR0)
#define CCM_CMEOR1_REG(base)		((base)->CCM_CMEOR1)
#define CCM_CMEOR2_REG(base)		((base)->CCM_CMEOR2)
#define CCM_CMEOR3_REG(base)		((base)->CCM_CMEOR3)
#define CCM_CMEOR4_REG(base)		((base)->CCM_CMEOR4)
#define CCM_CMEOR5_REG(base)		((base)->CCM_CMEOR5)
#define CCM_CPPDSR_REG(base)		((base)->CCM_CPPDSR)
#define CCM_CCOWR_REG(base)		((base)->CCM_CCOWR)
#define CCM_CCPGR0_REG(base)		((base)->CCM_CCPGR0)
#define CCM_CCPGR1_REG(base)		((base)->CCM_CCPGR1)
#define CCM_CCPGR2_REG(base)		((base)->CCM_CCPGR2)
#define CCM_CCPGR3_REG(base)		((base)->CCM_CCRGR3)

/* CCM - Peripheral instance base addresses */
/** Peripheral CCM base pointer */
#define CCM			((CCM_Type *)CCM_BASE)
#define CCM_BASE_PTR		(CCM)
/** Array initializer of CCM peripheral base adresses */
#define CCM_BASE_ADDRS		{ CCM_BASE }
/** Array initializer of CCM peripheral base pointers */
#define CCM_BASE_PTRS		{ CCM }

/* --- CCM registers ------------------------------------------------------- */
#define CCM_CCR			CCM_CCR_REG(CCM_BASE_PTR)
#define CCM_CSR			CCM_CSR_REG(CCM_BASE_PTR)
#define CCM_CCSR			CCM_CCSR_REG(CCM_BASE_PTR)
#define CCM_CACRR			CCM_CACRR_REG(CCM_BASE_PTR)
#define CCM_CSCMR1			CCM_CSCMR1_REG(CCM_BASE_PTR)
#define CCM_CSCDR1			CCM_CSCDR1_REG(CCM_BASE_PTR)
#define CCM_CSCDR2			CCM_CSCDR2_REG(CCM_BASE_PTR)
#define CCM_CSCDR3			CCM_CSCDR3_REG(CCM_BASE_PTR)
#define CCM_CSCMR2			CCM_CSCMR2_REG(CCM_BASE_PTR)
#define CCM_CTOR			CCM_CTOR_REG(CCM_BASE_PTR)
#define CCM_CLPCR			CCM_CLPCR_REG(CCM_BASE_PTR)
#define CCM_CISR			CCM_CISR_REG(CCM_BASE_PTR)
#define CCM_CIMR			CCM_CIMR_REG(CCM_BASE_PTR)
#define CCM_CCOSR			CCM_CCOSR_REG(CCM_BASE_PTR)
#define CCM_CGPR			CCM_CGPR_REG(CCM_BASE_PTR)
#define CCM_CCGR0			CCM_CCGR0_REG(CCM_BASE_PTR)
#define CCM_CCGR1			CCM_CCGR1_REG(CCM_BASE_PTR)
#define CCM_CCGR2			CCM_CCGR2_REG(CCM_BASE_PTR)
#define CCM_CCGR3			CCM_CCGR3_REG(CCM_BASE_PTR)
#define CCM_CCGR4			CCM_CCGR4_REG(CCM_BASE_PTR)
#define CCM_CCGR5			CCM_CCGR5_REG(CCM_BASE_PTR)
#define CCM_CCGR6			CCM_CCGR6_REG(CCM_BASE_PTR)
#define CCM_CCGR7			CCM_CCGR7_REG(CCM_BASE_PTR)
#define CCM_CCGR8			CCM_CCGR8_REG(CCM_BASE_PTR)
#define CCM_CCGR9			CCM_CCGR9_REG(CCM_BASE_PTR)
#define CCM_CCGR10			CCM_CCGR10_REG(CCM_BASE_PTR)
#define CCM_CCGR11			CCM_CCGR11_REG(CCM_BASE_PTR)
#define CCM_CMEOR0			CCM_CMEOR0_REG(CCM_BASE_PTR)
#define CCM_CMEOR1			CCM_CMEOR1_REG(CCM_BASE_PTR)
#define CCM_CMEOR2			CCM_CMEOR2_REG(CCM_BASE_PTR)
#define CCM_CMEOR3			CCM_CMEOR3_REG(CCM_BASE_PTR)
#define CCM_CMEOR4			CCM_CMEOR4_REG(CCM_BASE_PTR)
#define CCM_CMEOR5			CCM_CMEOR5_REG(CCM_BASE_PTR)
#define CCM_CPPDSR			CCM_CPPDSR_REG(CCM_BASE_PTR)
#define CCM_CCOWR			CCM_CCOWR_REG(CCM_BASE_PTR)
#define CCM_CCPGR0			CCM_CCPGR0_REG(CCM_BASE_PTR)
#define CCM_CCPGR1			CCM_CCPGR1_REG(CCM_BASE_PTR)
#define CCM_CCPGR2			CCM_CCPGR2_REG(CCM_BASE_PTR)
#define CCM_CCPGR3			CCM_CCPGR3_REG(CCM_BASE_PTR)

/* --- CCM values -....----------------------------------------------------- */

/* CCR: CCM Control Register */
#define CCM_CCR_FIRC_EN		(1 << 16)
#define CCM_CCR_FXOSC_EN		(1 << 12)
#define CCM_CCR_OSCNT_MASK		0xff

/* CSR: CCM Status Register */
#define CCM_CSR_FXOSC_RDY		(1 << 5)

/* CCSR: CCM Clock Switcher Register */
#define CCM_CCSR_PLL3_PFDN4_EN		(1 << 31)
#define CCM_CCSR_PLL3_PFDN3_EN		(1 << 30)
#define CCM_CCSR_PLL3_PFDN2_EN		(1 << 29)
#define CCM_CCSR_PLL3_PFDN1_EN		(1 << 28)

#define CCM_CCSR_DAP_EN		(1 << 24)

/* PLL1/PLL2 PFD SEL definition */
#define CCM_CCSR_PLL2_PFD_CLK_SEL_SHIFT	19
#define CCM_CCSR_PLL2_PFD_CLK_SEL_MASK		(0x7 << 19)
#define CCM_CCSR_PLL1_PFD_CLK_SEL_SHIFT	16
#define CCM_CCSR_PLL1_PFD_CLK_SEL_MASK		(0x7 << 16)

#define CCM_CCSR_PLL_PFD_CLK_SEL_MAIN		0x0
#define CCM_CCSR_PLL_PFD_CLK_SEL_PFD1		0x1
#define CCM_CCSR_PLL_PFD_CLK_SEL_PFD2		0x2
#define CCM_CCSR_PLL_PFD_CLK_SEL_PFD3		0x3
#define CCM_CCSR_PLL_PFD_CLK_SEL_PFD4		0x4

#define CCM_CCSR_PLL2_PFDN4_EN			(1 << 15)
#define CCM_CCSR_PLL2_PFDN3_EN			(1 << 14)
#define CCM_CCSR_PLL2_PFDN2_EN			(1 << 13)
#define CCM_CCSR_PLL2_PFDN1_EN			(1 << 12)

#define CCM_CCSR_PLL1_PFDN4_EN			(1 << 11)
#define CCM_CCSR_PLL1_PFDN3_EN			(1 << 10)
#define CCM_CCSR_PLL1_PFDN2_EN			(1 << 9)
#define CCM_CCSR_PLL1_PFDN1_EN			(1 << 8)

#define CCM_CCSR_DDRC_CLK_SEL			(1 << 7)
#define CCM_CCSR_FAST_CLK_SEL			(1 << 6)
#define CCM_CCSR_SLOW_CLK_SEL			(1 << 5)

#define CCM_CCSR_SYS_CLK_SEL_SHIFT		0
#define CCM_CCSR_SYS_CLK_SEL_MASK		0x7
#define CCM_CCSR_SYS_CLK_SEL_FAST		0x0
#define CCM_CCSR_SYS_CLK_SEL_SLOW		0x1
#define CCM_CCSR_SYS_CLK_SEL_PLL2_PFD		0x2
#define CCM_CCSR_SYS_CLK_SEL_PLL2		0x3
#define CCM_CCSR_SYS_CLK_SEL_PLL1_PFD		0x4
#define CCM_CCSR_SYS_CLK_SEL_PLL3		0x5

/* CACRR: ARM Clock Root Register */
#define CCM_CACRR_FLEX_CLK_DIV_SHIFT		22
#define CCM_CACRR_FLEX_CLK_DIV_MASK		(0x7 << 22)
#define CCM_CACRR_PLL6_CLK_DIV			(1 << 21)
#define CCM_CACRR_PLL3_CLK_DIV			(1 << 20)
#define CCM_CACRR_PLL1_PFD_CLK_DIV_SHIFT	16
#define CCM_CACRR_PLL1_PFD_CLK_DIV_MASK	(0x3 << 16)
#define CCM_CACRR_IPG_CLK_DIV_SHIFT		11
#define CCM_CACRR_IPG_CLK_DIV_MASK		(0x3 << 11)
#define CCM_CACRR_PLL4_CLK_DIV_SHIFT		6
#define CCM_CACRR_PLL4_CLK_DIV_MASK		(0x7 << 6)
#define CCM_CACRR_BUS_CLK_DIV_SHIFT		3
#define CCM_CACRR_BUS_CLK_DIV_MASK		(0x7 << 3)
#define CCM_CACRR_ARM_CLK_DIV_SHIFT		0
#define CCM_CACRR_ARM_CLK_DIV_MASK		(0x7 << 0)

#define CCM_CGR_MASK(gate)			((gate % 16) << 1)

/* ----------------------------------------------------------------------------
   -- GPIO Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Peripheral_Access_Layer GPIO Peripheral Access Layer
 * @{
 */

/** GPIO - Register Layout Typedef */
typedef struct {
  __IO  uint32_t PDOR;
  __IO  uint32_t PSOR;
  __IO  uint32_t PCOR;
  __IO  uint32_t PTOR;
  __IO  uint32_t PDIR;
} GPIO_Type, *GPIO_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- GPIO - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Register_Accessor_Macros GPIO - Register accessor macros
 * @{
 */

/* GPIO - Register accessors */
#define GPIO_PDOR_REG(base)			((base)->PDOR)
#define GPIO_PSOR_REG(base)			((base)->PSOR)
#define GPIO_PCOR_REG(base)			((base)->PCOR)
#define GPIO_PTOR_REG(base)			((base)->PTOR)
#define GPIO_PDIR_REG(base)			((base)->PDIR)

/*!
 * @}
 */ /* end of group GPIO_Register_Accessor_Macros */

/* Calculate GPIO port base based on GPIO pin */
#define GPIO(pin)			(GPIO_Type *)(GPIO_BASE + ((pin >> 5) << 6))
#define GPIO_OFFSET(gpio)		(0x1 << (gpio & 0x1F))

/* GPIO - Peripheral instance base addresses */
/** Peripheral GPIO0 base pointer */
#define GPIO0                                    ((GPIO_Type *)GPIO0_BASE)
#define GPIO0_BASE_PTR                           (GPIO0)
/** Peripheral GPIO1 base pointer */
#define GPIO1                                    ((GPIO_Type *)GPIO1_BASE)
#define GPIO1_BASE_PTR                           (GPIO1)
/** Peripheral GPIO2 base pointer */
#define GPIO2                                    ((GPIO_Type *)GPIO2_BASE)
#define GPIO2_BASE_PTR                           (GPIO2)
/** Peripheral GPIO3 base pointer */
#define GPIO3                                    ((GPIO_Type *)GPIO3_BASE)
#define GPIO3_BASE_PTR                           (GPIO3)
/** Peripheral GPIO4 base pointer */
#define GPIO4                                    ((GPIO_Type *)GPIO4_BASE)
#define GPIO4_BASE_PTR                           (GPIO4)
/** Array initializer of GPIO peripheral base adresses */
#define GPIO_BASE_ADDRS                          { GPIO0_BASE, GPIO1_BASE, GPIO2_BASE, GPIO3_BASE, GPIO4_BASE }
/** Array initializer of GPIO peripheral base pointers */
#define GPIO_BASE_PTRS                           { GPIO0, GPIO1, GPIO2, GPIO3, GPIO4 }

/* ----------------------------------------------------------------------------
   -- UART Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup UART_Peripheral_Access_Layer UART Peripheral Access Layer
 * @{
 */

/** UART - Register Layout Typedef */
typedef struct {
	__IO   uint8_t UART_BDH;
	__IO   uint8_t UART_BDL;
	__IO   uint8_t UART_C1;
	__IO   uint8_t UART_C2;
	__IO   uint8_t UART_S1;
	__IO   uint8_t UART_S2;
	__IO   uint8_t UART_C3;
	__IO   uint8_t UART_D;
	__IO   uint8_t UART_MA1;
	__IO   uint8_t UART_MA2;
	__IO   uint8_t UART_C4;
	__IO   uint8_t UART_C5;
	__IO   uint8_t UART_ED;
	__IO   uint8_t UART_MODEM;
	__IO   uint8_t UART_IR;
	__IO   uint8_t UART_PFIFO;
	__IO   uint8_t UART_CFIFO;
	__IO   uint8_t UART_SFIFO;
	__IO   uint8_t UART_TWFIFO;
	__IO   uint8_t UART_TCFIFO;
	__IO   uint8_t UART_RWFIFO;
	__IO   uint8_t UART_RCFIFO;
	__IO   uint8_t UART_C7816;
	__IO   uint8_t UART_IS7816;
	__IO   uint8_t UART_WP7816T0;
	__IO   uint8_t UART_WP7816T1;
	__IO   uint8_t UART_WN7816;
	__IO   uint8_t UART_WF7816;
	__IO   uint8_t UART_ET7816;
	__IO   uint8_t UART_TL7816;
} UART_Type, *UART_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- UART - Register accessor macros
   ---------------------------------------------------------------------------- */

/* UART - Peripheral instance base addresses */
/** Peripheral UART1 base pointer */
#define UART1                                    ((UART_Type *)UART1_BASE)
#define UART1_BASE_PTR                           (UART1)
/** Peripheral UART2 base pointer */
#define UART2                                    ((UART_Type *)UART2_BASE)
#define UART2_BASE_PTR                           (UART2)
/** Peripheral UART3 base pointer */
#define UART3                                    ((UART_Type *)UART3_BASE)
#define UART3_BASE_PTR                           (UART3)
/** Peripheral UART4 base pointer */
#define UART4                                    ((UART_Type *)UART4_BASE)
#define UART4_BASE_PTR                           (UART4)
/** Peripheral UART5 base pointer */
#define UART5                                    ((UART_Type *)UART5_BASE)
#define UART5_BASE_PTR                           (UART5)
/** Array initializer of UART peripheral base adresses */
#define UART_BASE_ADDRS                          { UART1_BASE, UART2_BASE, UART3_BASE, UART4_BASE, UART5_BASE }
/** Array initializer of UART peripheral base pointers */
#define UART_BASE_PTRS                           { UART1, UART2, UART3, UART4, UART5 }
/** Interrupt vectors for the UART peripheral type */
#define UART_IRQS                                { UART0_IRQ, UART1_IRQ, UART2_IRQ, UART3_IRQ, UART4_IRQ, UART5_IRQ }

/*!
 * @addtogroup UART_Register_Accessor_Macros UART - Register accessor macros
 * @{
 */

/* UART - Register accessors */
#define UART_BDH_REG(base)			((base)->UART_BDH)
#define UART_BDL_REG(base)			((base)->UART_BDL)
#define UART_C1_REG(base)			((base)->UART_C1)
#define UART_C2_REG(base)			((base)->UART_C2)
#define UART_S1_REG(base)			((base)->UART_S1)
#define UART_S2_REG(base)			((base)->UART_S2)
#define UART_C3_REG(base)			((base)->UART_C3)
#define UART_D_REG(base)			((base)->UART_D)
#define UART_MA1_REG(base)			((base)->UART_MA1)
#define UART_MA2_REG(base)			((base)->UART_MA2)
#define UART_C4_REG(base)			((base)->UART_C4)
#define UART_C5_REG(base)			((base)->UART_C5)
#define UART_ED_REG(base)			((base)->UART_ED)
#define UART_MODEM_REG(base)			((base)->UART_MODEM)
#define UART_IR_REG(base)			((base)->UART_IR)
#define UART_PFIFO_REG(base)			((base)->UART_PFIFO)
#define UART_CFIFO_REG(base)			((base)->UART_CFIFO)
#define UART_SFIFO_REG(base)			((base)->UART_SFIFO)
#define UART_TWFIFO_REG(base)			((base)->UART_TWFIFO)
#define UART_TCFIFO_REG(base)			((base)->UART_TCFIFO)
#define UART_RWFIFO_REG(base)			((base)->UART_RWFIFO)
#define UART_RCFIFO_REG(base)			((base)->UART_RCFIFO)
#define UART_C7816_REG(base)			((base)->UART_C7816)
#define UART_IS7816_REG(base)			((base)->UART_IS7816)
#define UART_WP7816T0_REG(base)		((base)->UART_WP7816T0)
#define UART_WP7816T1_REG(base)		((base)->UART_WP7816T1)
#define UART_WN7816_REG(base)			((base)->UART_WN7816)
#define UART_WF7816_REG(base)			((base)->UART_WF7816)
#define UART_ET7816_REG(base)			((base)->UART_ET7816)
#define UART_TL7816_REG(base)			((base)->UART_TL7816)

/* BDH: Baud Rate Register High */
#define UART_BDH_LBKDIE			(1 << 7)
#define UART_BDH_RXEDGIE			(1 << 6)
#define UART_BDH_SBR_MASK			0x1f
#define UART_C4_BRFA_MASK			0x1f

/* BDL: Baud Rate Register Low */
#define UART_BDL_SBR_MASK			0xff

/* C1: Control register 1 */
#define UART_UCR1_LOOPS			(1 << 7)
#define UART_UCR1_RSRC				(1 << 5)
#define UART_UCR1_M				(1 << 4)
#define UART_UCR1_WAKE				(1 << 3)
#define UART_UCR1_ILT				(1 << 2)
#define UART_UCR1_PE				(1 << 1)
#define UART_UCR1_PT				(1 << 0)

/* C2: Control register 2 */
#define UART_UCR2_TIE				(1 << 7)
#define UART_UCR2_TCIE				(1 << 6)
#define UART_UCR2_RIE				(1 << 5)
#define UART_UCR2_ILIE				(1 << 4)
#define UART_UCR2_TE				(1 << 3)
#define UART_UCR2_RE				(1 << 2)
#define UART_UCR2_RWU				(1 << 1)
#define UART_UCR2_SBK				(1 << 0)

/* S1: Status register 1 */
#define UART_S1_TDRE				(1 << 7)
#define UART_S1_TC				(1 << 6)
#define UART_S1_RDRF				(1 << 5)
#define UART_S1_IDLE				(1 << 4)
#define UART_S1_OR				(1 << 3)
#define UART_S1_NF				(1 << 2)
#define UART_S1_FE				(1 << 1)
#define UART_S1_PF				(1 << 0)

/* S2: Status register 2 */
#define UART_S2_LBKDIF				(1 << 7)
#define UART_S2_RXEDGIF			(1 << 6)
#define UART_S2_MSBF				(1 << 5)
#define UART_S2_RXINV				(1 << 4)
#define UART_S2_RWUID				(1 << 3)
#define UART_S2_BRK13				(1 << 2)
#define UART_S2_LBKDE				(1 << 1)
#define UART_S2_RAF				(1 << 0)

/* C3: Control register 3 */
#define UART_C3_R8				(1 << 7)
#define UART_C3_T8				(1 << 6)
#define UART_C3_TXDIR				(1 << 5)
#define UART_C3_TXINV				(1 << 4)
#define UART_C3_ORIE				(1 << 3)
#define UART_C3_NEIE				(1 << 2)
#define UART_C3_FEIE				(1 << 1)
#define UART_C3_PEIE				(1 << 0)

/* MODEM: Modem configuration register */
#define UART_MODEM_RXRTSE			(1 << 3)
#define UART_MODEM_TXRTSPOL			(1 << 2)
#define UART_MODEM_TXRTSE			(1 << 1)
#define UART_MODEM_TXCTSE			(1 << 0)

/****************************************************************************/
/** @defgroup uart_parity UART Parity Selection
@ingroup VF6xx_uart_defines
@{*/
#define UART_PARITY_NONE		0x00
#define UART_PARITY_EVEN		UART_C1_PE
#define UART_PARITY_ODD		(UART_C1_PE | UART_C1_PT)
/**@}*/
#define UART_PARITY_MASK		0x3

/* CR3_CTSE/CR3_RTSE combined values */
/****************************************************************************/
/** @defgroup usart_cr3_flowcontrol USART Hardware Flow Control Selection
@ingroup STM32F_usart_defines
@{*/
#define UART_FLOWCONTROL_NONE		0x00
#define UART_FLOWCONTROL_RTS		UART_MODEM_RXRTSE
#define UART_FLOWCONTROL_CTS		UART_MODEM_TXCTSE
#define UART_FLOWCONTROL_RTS_CTS	(UART_MODEM_RXRTSE | UART_MODEM_TXCTSE)
/**@}*/
#define UART_FLOWCONTROL_MASK		(UART_MODEM_RXRTSE | UART_MODEM_TXCTSE)

/*!
 * @}
 */ /* end of group UART_Peripheral */


/* ----------------------------------------------------------------------------
   -- IOMUXC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup IOMUXC_Peripheral_Access_Layer IOMUXC Peripheral Access Layer
 * @{
 */

/** IOMUXC - Register Layout Typedef */
typedef struct {
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTA6;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTA8;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTA9;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTA10;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTA11;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTA12;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTA16;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTA17;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTA18;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTA19;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTA20;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTA21;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTA22;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTA23;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTA24;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTA25;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTA26;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTA27;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTA28;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTA29;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTA30;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTA31;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB0;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB1;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB2;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB3;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB4;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB5;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB6;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB7;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB8;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB9;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB10;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB11;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB12;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB13;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB14;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB15;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB16;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB17;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB18;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB19;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB20;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB21;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB22;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTC0;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTC1;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTC2;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTC3;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTC4;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTC5;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTC6;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTC7;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTC8;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTC9;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTC10;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTC11;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTC12;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTC13;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTC14;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTC15;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTC16;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTC17;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD31;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD30;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD29;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD28;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD27;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD26;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD25;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD24;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD23;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD22;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD21;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD20;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD19;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD18;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD17;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD16;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD0;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD1;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD2;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD3;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD4;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD5;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD6;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD7;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD8;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD9;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD10;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD11;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD12;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTD13;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB23;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB24;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB25;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB26;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB27;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTB28;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTC26;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTC27;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTC28;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTC29;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTC30;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTC31;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE0;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE1;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE2;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE3;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE4;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE5;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE6;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE7;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE8;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE9;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE10;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE11;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE12;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE13;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE14;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE15;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE16;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE17;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE18;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE19;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE20;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE21;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE22;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE23;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE24;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE25;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE26;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE27;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTE28;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_PTA7;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_RESETB;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_A15;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_A14;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_A13;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_A12;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_A11;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_A10;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_A9;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_A8;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_A7;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_A6;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_A5;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_A4;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_A3;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_A2;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_A1;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_A0;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_BA2;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_BA1;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_BA0;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_CAS_B;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_CKE0;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_CLK0;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_CS_B0;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D15;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D14;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D13;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D12;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D11;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D10;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D9;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D8;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D7;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D6;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D5;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D4;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D3;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D2;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D1;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D0;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_DQM1;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_DQM0;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_DQS1;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_DQS0;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_RAS_B;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_WE_B;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_ODT0;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_ODT1;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_DDRBYTE1;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DDR_DDRBYTE2;
	__IO  uint32_t RESERVED_0[2];
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_CCM_AUD_EXT_CLK_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_CCM_ENET_EXT_CLK_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_CCM_ENET_TS_CLK_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DSPI1_IPP_IND_SCK_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DSPI1_IPP_IND_SIN_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_DSPI1_IPP_IND_SS_B_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_ENET_SWIAHB_IPP_IND_MAC0_TIMER_0_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_ENET_SWIAHB_IPP_IND_MAC0_TIMER_1_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_ESAI_IPP_IND_FST_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_ESAI_IPP_IND_SCKT_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_ESAI_IPP_IND_SDO0_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_ESAI_IPP_IND_SDO1_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_ESAI_IPP_IND_SDO2_SDI3_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_ESAI_IPP_IND_SDO3_SDI2_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_ESAI_IPP_IND_SDO4_SDI1_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_ESAI_IPP_IND_SDO5_SDI0_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_FLEXTIMER1_IPP_IND_FTM_CH_0_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_FLEXTIMER1_IPP_IND_FTM_CH_1_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_FLEXTIMER1_IPP_IND_FTM_PHA_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_FLEXTIMER1_IPP_IND_FTM_PHB_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_I2C0_IPP_SCL_IND_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_I2C0_IPP_SDA_IND_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_I2C1_IPP_SCL_IND_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_I2C1_IPP_SDA_IND_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_I2C2_IPP_SCL_IND_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_I2C2_IPP_SDA_IND_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_MLB_TOP_MLBCLK_IN_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_MLB_TOP_MLBDAT_IN_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_MLB_TOP_MLBSIG_IN_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_SAI1_IPP_IND_SAI_TXSYNC_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_SAI2_IPP_IND_SAI_RXBCLK_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_SAI2_IPP_IND_SAI_RXDATA_0_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_SAI2_IPP_IND_SAI_RXSYNC_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_SAI2_IPP_IND_SAI_TXBCLK_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_SAI2_IPP_IND_SAI_TXSYNC_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_SCI_FLX1_IPP_IND_CTS_B_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_SCI_FLX1_IPP_IND_SCI_RX_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_SCI_FLX1_IPP_IND_SCI_TX_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_SCI_FLX2_IPP_IND_CTS_B_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_SCI_FLX2_IPP_IND_SCI_RX_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_SCI_FLX2_IPP_IND_SCI_TX_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_SCI_FLX3_IPP_IND_SCI_RX_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_SCI_FLX3_IPP_IND_SCI_TX_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_SRC_IPP_BOOT_CFG_18_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_SRC_IPP_BOOT_CFG_19_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_SRC_IPP_BOOT_CFG_20_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_VIDEO_IN0_IPP_IND_DE_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_VIDEO_IN0_IPP_IND_FID_SELECT_INPUT;
	__IO  uint32_t SW_MUX_CTL_PAD_IOMUXC_VIDEO_IN0_IPP_IND_PIX_CLK_SELECT_INPUT;
} IOMUXC_Type, *IOMUXC_MemMapPtr;

#define IOMUXC_SW_MUX_CTL_PAD_PTA6_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTA6)
#define IOMUXC_SW_MUX_CTL_PAD_PTA8_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTA8)
#define IOMUXC_SW_MUX_CTL_PAD_PTA9_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTA9)
#define IOMUXC_SW_MUX_CTL_PAD_PTA10_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTA10)
#define IOMUXC_SW_MUX_CTL_PAD_PTA11_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTA11)
#define IOMUXC_SW_MUX_CTL_PAD_PTA12_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTA12)
#define IOMUXC_SW_MUX_CTL_PAD_PTA16_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTA16)
#define IOMUXC_SW_MUX_CTL_PAD_PTA17_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTA17)
#define IOMUXC_SW_MUX_CTL_PAD_PTA18_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTA18)
#define IOMUXC_SW_MUX_CTL_PAD_PTA19_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTA19)
#define IOMUXC_SW_MUX_CTL_PAD_PTA20_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTA20)
#define IOMUXC_SW_MUX_CTL_PAD_PTA21_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTA21)
#define IOMUXC_SW_MUX_CTL_PAD_PTA22_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTA22)
#define IOMUXC_SW_MUX_CTL_PAD_PTA23_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTA23)
#define IOMUXC_SW_MUX_CTL_PAD_PTA24_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTA24)
#define IOMUXC_SW_MUX_CTL_PAD_PTA25_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTA25)
#define IOMUXC_SW_MUX_CTL_PAD_PTA26_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTA26)
#define IOMUXC_SW_MUX_CTL_PAD_PTA27_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTA27)
#define IOMUXC_SW_MUX_CTL_PAD_PTA28_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTA28)
#define IOMUXC_SW_MUX_CTL_PAD_PTA29_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTA29)
#define IOMUXC_SW_MUX_CTL_PAD_PTA30_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTA30)
#define IOMUXC_SW_MUX_CTL_PAD_PTA31_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTA31)
#define IOMUXC_SW_MUX_CTL_PAD_PTB0_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB0)
#define IOMUXC_SW_MUX_CTL_PAD_PTB1_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB1)
#define IOMUXC_SW_MUX_CTL_PAD_PTB2_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB2)
#define IOMUXC_SW_MUX_CTL_PAD_PTB3_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB3)
#define IOMUXC_SW_MUX_CTL_PAD_PTB4_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB4)
#define IOMUXC_SW_MUX_CTL_PAD_PTB5_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB5)
#define IOMUXC_SW_MUX_CTL_PAD_PTB6_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB6)
#define IOMUXC_SW_MUX_CTL_PAD_PTB7_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB7)
#define IOMUXC_SW_MUX_CTL_PAD_PTB8_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB8)
#define IOMUXC_SW_MUX_CTL_PAD_PTB9_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB9)
#define IOMUXC_SW_MUX_CTL_PAD_PTB10_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB10)
#define IOMUXC_SW_MUX_CTL_PAD_PTB11_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB11)
#define IOMUXC_SW_MUX_CTL_PAD_PTB12_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB12)
#define IOMUXC_SW_MUX_CTL_PAD_PTB13_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB13)
#define IOMUXC_SW_MUX_CTL_PAD_PTB14_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB14)
#define IOMUXC_SW_MUX_CTL_PAD_PTB15_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB15)
#define IOMUXC_SW_MUX_CTL_PAD_PTB16_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB16)
#define IOMUXC_SW_MUX_CTL_PAD_PTB17_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB17)
#define IOMUXC_SW_MUX_CTL_PAD_PTB18_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB18)
#define IOMUXC_SW_MUX_CTL_PAD_PTB19_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB19)
#define IOMUXC_SW_MUX_CTL_PAD_PTB20_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB20)
#define IOMUXC_SW_MUX_CTL_PAD_PTB21_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB21)
#define IOMUXC_SW_MUX_CTL_PAD_PTB22_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB22)
#define IOMUXC_SW_MUX_CTL_PAD_PTC0_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTC0)
#define IOMUXC_SW_MUX_CTL_PAD_PTC1_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTC1)
#define IOMUXC_SW_MUX_CTL_PAD_PTC2_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTC2)
#define IOMUXC_SW_MUX_CTL_PAD_PTC3_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTC3)
#define IOMUXC_SW_MUX_CTL_PAD_PTC4_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTC4)
#define IOMUXC_SW_MUX_CTL_PAD_PTC5_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTC5)
#define IOMUXC_SW_MUX_CTL_PAD_PTC6_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTC6)
#define IOMUXC_SW_MUX_CTL_PAD_PTC7_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTC7)
#define IOMUXC_SW_MUX_CTL_PAD_PTC8_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTC8)
#define IOMUXC_SW_MUX_CTL_PAD_PTC9_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTC9)
#define IOMUXC_SW_MUX_CTL_PAD_PTC10_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTC10)
#define IOMUXC_SW_MUX_CTL_PAD_PTC11_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTC11)
#define IOMUXC_SW_MUX_CTL_PAD_PTC12_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTC12)
#define IOMUXC_SW_MUX_CTL_PAD_PTC13_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTC13)
#define IOMUXC_SW_MUX_CTL_PAD_PTC14_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTC14)
#define IOMUXC_SW_MUX_CTL_PAD_PTC15_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTC15)
#define IOMUXC_SW_MUX_CTL_PAD_PTC16_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTC16)
#define IOMUXC_SW_MUX_CTL_PAD_PTC17_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTC17)
#define IOMUXC_SW_MUX_CTL_PAD_PTD31_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD31)
#define IOMUXC_SW_MUX_CTL_PAD_PTD30_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD30)
#define IOMUXC_SW_MUX_CTL_PAD_PTD29_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD29)
#define IOMUXC_SW_MUX_CTL_PAD_PTD28_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD28)
#define IOMUXC_SW_MUX_CTL_PAD_PTD27_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD27)
#define IOMUXC_SW_MUX_CTL_PAD_PTD26_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD26)
#define IOMUXC_SW_MUX_CTL_PAD_PTD25_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD25)
#define IOMUXC_SW_MUX_CTL_PAD_PTD24_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD24)
#define IOMUXC_SW_MUX_CTL_PAD_PTD23_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD23)
#define IOMUXC_SW_MUX_CTL_PAD_PTD22_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD22)
#define IOMUXC_SW_MUX_CTL_PAD_PTD21_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD21)
#define IOMUXC_SW_MUX_CTL_PAD_PTD20_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD20)
#define IOMUXC_SW_MUX_CTL_PAD_PTD19_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD19)
#define IOMUXC_SW_MUX_CTL_PAD_PTD18_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD18)
#define IOMUXC_SW_MUX_CTL_PAD_PTD17_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD17)
#define IOMUXC_SW_MUX_CTL_PAD_PTD16_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD16)
#define IOMUXC_SW_MUX_CTL_PAD_PTD0_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD0)
#define IOMUXC_SW_MUX_CTL_PAD_PTD1_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD1)
#define IOMUXC_SW_MUX_CTL_PAD_PTD2_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD2)
#define IOMUXC_SW_MUX_CTL_PAD_PTD3_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD3)
#define IOMUXC_SW_MUX_CTL_PAD_PTD4_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD4)
#define IOMUXC_SW_MUX_CTL_PAD_PTD5_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD5)
#define IOMUXC_SW_MUX_CTL_PAD_PTD6_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD6)
#define IOMUXC_SW_MUX_CTL_PAD_PTD7_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD7)
#define IOMUXC_SW_MUX_CTL_PAD_PTD8_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD8)
#define IOMUXC_SW_MUX_CTL_PAD_PTD9_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD9)
#define IOMUXC_SW_MUX_CTL_PAD_PTD10_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD10)
#define IOMUXC_SW_MUX_CTL_PAD_PTD11_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD11)
#define IOMUXC_SW_MUX_CTL_PAD_PTD12_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD12)
#define IOMUXC_SW_MUX_CTL_PAD_PTD13_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTD13)
#define IOMUXC_SW_MUX_CTL_PAD_PTB23_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB23)
#define IOMUXC_SW_MUX_CTL_PAD_PTB24_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB24)
#define IOMUXC_SW_MUX_CTL_PAD_PTB25_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB25)
#define IOMUXC_SW_MUX_CTL_PAD_PTB26_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB26)
#define IOMUXC_SW_MUX_CTL_PAD_PTB27_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB27)
#define IOMUXC_SW_MUX_CTL_PAD_PTB28_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTB28)
#define IOMUXC_SW_MUX_CTL_PAD_PTC26_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTC26)
#define IOMUXC_SW_MUX_CTL_PAD_PTC27_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTC27)
#define IOMUXC_SW_MUX_CTL_PAD_PTC28_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTC28)
#define IOMUXC_SW_MUX_CTL_PAD_PTC29_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTC29)
#define IOMUXC_SW_MUX_CTL_PAD_PTC30_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTC30)
#define IOMUXC_SW_MUX_CTL_PAD_PTC31_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTC31)
#define IOMUXC_SW_MUX_CTL_PAD_PTE0_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE0)
#define IOMUXC_SW_MUX_CTL_PAD_PTE1_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE1)
#define IOMUXC_SW_MUX_CTL_PAD_PTE2_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE2)
#define IOMUXC_SW_MUX_CTL_PAD_PTE3_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE3)
#define IOMUXC_SW_MUX_CTL_PAD_PTE4_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE4)
#define IOMUXC_SW_MUX_CTL_PAD_PTE5_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE5)
#define IOMUXC_SW_MUX_CTL_PAD_PTE6_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE6)
#define IOMUXC_SW_MUX_CTL_PAD_PTE7_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE7)
#define IOMUXC_SW_MUX_CTL_PAD_PTE8_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE8)
#define IOMUXC_SW_MUX_CTL_PAD_PTE9_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE9)
#define IOMUXC_SW_MUX_CTL_PAD_PTE10_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE10)
#define IOMUXC_SW_MUX_CTL_PAD_PTE11_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE11)
#define IOMUXC_SW_MUX_CTL_PAD_PTE12_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE12)
#define IOMUXC_SW_MUX_CTL_PAD_PTE13_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE13)
#define IOMUXC_SW_MUX_CTL_PAD_PTE14_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE14)
#define IOMUXC_SW_MUX_CTL_PAD_PTE15_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE15)
#define IOMUXC_SW_MUX_CTL_PAD_PTE16_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE16)
#define IOMUXC_SW_MUX_CTL_PAD_PTE17_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE17)
#define IOMUXC_SW_MUX_CTL_PAD_PTE18_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE18)
#define IOMUXC_SW_MUX_CTL_PAD_PTE19_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE19)
#define IOMUXC_SW_MUX_CTL_PAD_PTE20_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE20)
#define IOMUXC_SW_MUX_CTL_PAD_PTE21_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE21)
#define IOMUXC_SW_MUX_CTL_PAD_PTE22_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE22)
#define IOMUXC_SW_MUX_CTL_PAD_PTE23_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE23)
#define IOMUXC_SW_MUX_CTL_PAD_PTE24_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE24)
#define IOMUXC_SW_MUX_CTL_PAD_PTE25_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE25)
#define IOMUXC_SW_MUX_CTL_PAD_PTE26_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE26)
#define IOMUXC_SW_MUX_CTL_PAD_PTE27_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE27)
#define IOMUXC_SW_MUX_CTL_PAD_PTE28_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTE28)
#define IOMUXC_SW_MUX_CTL_PAD_PTA7_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_PTA7)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_RESETB_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_RESETB)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_A15_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_A15)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_A14_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_A14)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_A13_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_A13)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_A12_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_A12)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_A11_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_A11)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_A10_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_A10)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_A9_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_A9)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_A8_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_A8)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_A7_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_A7)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_A6_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_A6)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_A5_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_A5)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_A4_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_A4)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_A3_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_A3)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_A2_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_A2)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_A1_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_A1)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_A0_REG(base)						((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_A0)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_BA2_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_BA2)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_BA1_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_BA1)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_BA0_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_BA0)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_CAS_B_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_CAS_B)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_CKE0_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_CKE0)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_CLK0_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_CLK0)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_CS_B0_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_CS_B0)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D15_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D15)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D14_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D14)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D13_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D13)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D12_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D12)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D11_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D11)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D10_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D10)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D9_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D9)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D8_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D8)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D7_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D7)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D6_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D6)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D5_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D5)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D4_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D4)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D3_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D3)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D2_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D2)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D1_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D1)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D0_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_CS_D0)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_DQM1_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_DQM1)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_DQM0_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_DQM0)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_DQS1_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_DQS1)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_DQS0_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_DQS0)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_RAS_B_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_RAS_B)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_WE_B_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_WE_B)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_ODT0_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_ODT0)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_ODT1_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_ODT1)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_DDRBYTE1_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_DDRBYTE1)
#define IOMUXC_SW_MUX_CTL_PAD_DDR_DDRBYTE2_REG(base)					((base)->SW_MUX_CTL_PAD_IOMUXC_DDR_DDRBYTE2)
#define IOMUXC_SW_MUX_CTL_PAD_CCM_AUD_EXT_CLK_SELECT_INPUT_REG(base)			((base)->SW_MUX_CTL_PAD_IOMUXC_CCM_AUD_EXT_CLK_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_CCM_ENET_EXT_CLK_SELECT_INPUT_REG(base)			((base)->SW_MUX_CTL_PAD_IOMUXC_CCM_ENET_EXT_CLK_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_CCM_ENET_TS_CLK_SELECT_INPUT_REG(base)			((base)->SW_MUX_CTL_PAD_IOMUXC_CCM_ENET_TS_CLK_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_DSPI1_IPP_IND_SCK_SELECT_INPUT_REG(base)			((base)->SW_MUX_CTL_PAD_IOMUXC_DSPI1_IPP_IND_SCK_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_DSPI1_IPP_IND_SIN_SELECT_INPUT_REG(base)			((base)->SW_MUX_CTL_PAD_IOMUXC_DSPI1_IPP_IND_SIN_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_DSPI1_IPP_IND_SS_B_SELECT_INPUT_REG(base)		((base)->SW_MUX_CTL_PAD_IOMUXC_DSPI1_IPP_IND_SS_B_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_MAC0_TIMER_0_SELECT_INPUT_REG(base)			((base)->SW_MUX_CTL_PAD_IOMUXC_MAC0_TIMER_0_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_MAC0_TIMER_1_SELECT_INPUT_REG(base)			((base)->SW_MUX_CTL_PAD_IOMUXC_MAC0_TIMER_1_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_ESAI_IPP_IND_FST_SELECT_INPUT_REG(base)			((base)->SW_MUX_CTL_PAD_IOMUXC_ESAI_IPP_IND_FST_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_ESAI_IPP_IND_SCKT_SELECT_INPUT_REG(base)			((base)->SW_MUX_CTL_PAD_IOMUXC_ESAI_IPP_IND_SCKT_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_ESAI_IPP_IND_SDO0_SELECT_INPUT_REG(base)			((base)->SW_MUX_CTL_PAD_IOMUXC_ESAI_IPP_IND_SDO0_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_ESAI_IPP_IND_SDO1_SELECT_INPUT_REG(base)			((base)->SW_MUX_CTL_PAD_IOMUXC_ESAI_IPP_IND_SDO1_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_ESAI_IPP_IND_SDO2_SDI3_SELECT_INPUT_REG(base)		((base)->SW_MUX_CTL_PAD_IOMUXC_ESAI_IPP_IND_SDO2_SDI3_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_ESAI_IPP_IND_SDO3_SDI2_SELECT_INPUT_REG(base)		((base)->SW_MUX_CTL_PAD_IOMUXC_ESAI_IPP_IND_SDO3_SDI2_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_ESAI_IPP_IND_SDO4_SDI1_SELECT_INPUT_REG(base)		((base)->SW_MUX_CTL_PAD_IOMUXC_ESAI_IPP_IND_SDO4_SDI1_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_ESAI_IPP_IND_SDO5_SDI0_SELECT_INPUT_REG(base)		((base)->SW_MUX_CTL_PAD_IOMUXC_ESAI_IPP_IND_SDO5_SDI0_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_FLEXTIMER1_IPP_IND_FTM_CH_0_SELECT_INPUT_REG(base)	((base)->SW_MUX_CTL_PAD_IOMUXC_FLEXTIMER1_IPP_IND_FTM_CH_0_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_FLEXTIMER1_IPP_IND_FTM_CH_1_SELECT_INPUT_REG(base)	((base)->SW_MUX_CTL_PAD_IOMUXC_FLEXTIMER1_IPP_IND_FTM_CH_1_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_FLEXTIMER1_IPP_IND_FTM_PHA_SELECT_INPUT_REG(base)	((base)->SW_MUX_CTL_PAD_IOMUXC_FLEXTIMER1_IPP_IND_FTM_PHA_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_FLEXTIMER1_IPP_IND_FTM_PHB_SELECT_INPUT_REG(base)	((base)->SW_MUX_CTL_PAD_IOMUXC_FLEXTIMER1_IPP_IND_FTM_PHB_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_I2C0_IPP_SCL_IND_SELECT_INPUT_REG(base)			((base)->SW_MUX_CTL_PAD_IOMUXC_I2C0_IPP_SCL_IND_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_I2C0_IPP_SDA_IND_SELECT_INPUT_REG(base)			((base)->SW_MUX_CTL_PAD_IOMUXC_I2C0_IPP_SDA_IND_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_I2C1_IPP_SCL_IND_SELECT_INPUT_REG(base)			((base)->SW_MUX_CTL_PAD_IOMUXC_I2C1_IPP_SCL_IND_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_I2C1_IPP_SDA_IND_SELECT_INPUT_REG(base)			((base)->SW_MUX_CTL_PAD_IOMUXC_I2C1_IPP_SDA_IND_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_I2C2_IPP_SCL_IND_SELECT_INPUT_REG(base)			((base)->SW_MUX_CTL_PAD_IOMUXC_I2C2_IPP_SCL_IND_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_I2C2_IPP_SDA_IND_SELECT_INPUT_REG(base)			((base)->SW_MUX_CTL_PAD_IOMUXC_I2C2_IPP_SDA_IND_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_MLB_TOP_MLBCLK_IN_SELECT_INPUT_REG(base)			((base)->SW_MUX_CTL_PAD_IOMUXC_MLB_TOP_MLBCLK_IN_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_MLB_TOP_MLBDAT_IN_SELECT_INPUT_REG(base)			((base)->SW_MUX_CTL_PAD_IOMUXC_MLB_TOP_MLBDAT_IN_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_MLB_TOP_MLBSIG_IN_SELECT_INPUT_REG(base)			((base)->SW_MUX_CTL_PAD_IOMUXC_MLB_TOP_MLBSIG_IN_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_SAI1_IPP_IND_SAI_TXSYNC_SELECT_INPUT_REG(base)		((base)->SW_MUX_CTL_PAD_IOMUXC_SAI1_IPP_IND_SAI_TXSYNC_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_SAI2_IPP_IND_SAI_RXBCLK_SELECT_INPUT_REG(base)		((base)->SW_MUX_CTL_PAD_IOMUXC_SAI2_IPP_IND_SAI_RXBCLK_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_SAI2_IPP_IND_SAI_RXDATA_0_SELECT_INPUT_REG(base)		((base)->SW_MUX_CTL_PAD_IOMUXC_SAI2_IPP_IND_SAI_RXDATA_0_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_SAI2_IPP_IND_SAI_RXSYNC_SELECT_INPUT_REG(base)		((base)->SW_MUX_CTL_PAD_IOMUXC_SAI2_IPP_IND_SAI_RXSYNC_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_SAI2_IPP_IND_SAI_TXBCLK_SELECT_INPUT_REG(base)		((base)->SW_MUX_CTL_PAD_IOMUXC_SAI2_IPP_IND_SAI_TXBCLK_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_SAI2_IPP_IND_SAI_TXSYNC_SELECT_INPUT_REG(base)		((base)->SW_MUX_CTL_PAD_IOMUXC_SAI2_IPP_IND_SAI_TXSYNC_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_SCI_FLX1_IPP_IND_CTS_B_SELECT_INPUT_REG(base)		((base)->SW_MUX_CTL_PAD_IOMUXC_SCI_FLX1_IPP_IND_CTS_B_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_SCI_FLX1_IPP_IND_SCI_RX_SELECT_INPUT_REG(base)		((base)->SW_MUX_CTL_PAD_IOMUXC_SCI_FLX1_IPP_IND_SCI_RX_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_SCI_FLX1_IPP_IND_SCI_TX_SELECT_INPUT_REG(base)		((base)->SW_MUX_CTL_PAD_IOMUXC_SCI_FLX1_IPP_IND_SCI_TX_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_SCI_FLX2_IPP_IND_CTS_B_SELECT_INPUT_REG(base)		((base)->SW_MUX_CTL_PAD_IOMUXC_SCI_FLX2_IPP_IND_CTS_B_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_SCI_FLX2_IPP_IND_SCI_RX_SELECT_INPUT_REG(base)		((base)->SW_MUX_CTL_PAD_IOMUXC_SCI_FLX2_IPP_IND_SCI_RX_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_SCI_FLX2_IPP_IND_SCI_TX_SELECT_INPUT_REG(base)		((base)->SW_MUX_CTL_PAD_IOMUXC_SCI_FLX2_IPP_IND_SCI_TX_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_SCI_FLX3_IPP_IND_SCI_RX_SELECT_INPUT_REG(base)		((base)->SW_MUX_CTL_PAD_IOMUXC_SCI_FLX3_IPP_IND_SCI_RX_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_SCI_FLX3_IPP_IND_SCI_TX_SELECT_INPUT_REG(base)		((base)->SW_MUX_CTL_PAD_IOMUXC_SCI_FLX3_IPP_IND_SCI_TX_SELECT_INPUT))
#define IOMUXC_SW_MUX_CTL_PAD_SRC_IPP_BOOT_CFG_18_SELECT_INPUT_REG(base)		((base)->SW_MUX_CTL_PAD_IOMUXC_SRC_IPP_BOOT_CFG_18_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_SRC_IPP_BOOT_CFG_19_SELECT_INPUT_REG(base)		((base)->SW_MUX_CTL_PAD_IOMUXC_SRC_IPP_BOOT_CFG_19_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_SRC_IPP_BOOT_CFG_20_SELECT_INPUT_REG(base)		((base)->SW_MUX_CTL_PAD_IOMUXC_SRC_IPP_BOOT_CFG_20_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_VIDEO_IN0_IPP_IND_DE_SELECT_INPUT_REG(base)		((base)->SW_MUX_CTL_PAD_IOMUXC_VIDEO_IN0_IPP_IND_DE_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_VIDEO_IN0_IPP_IND_FID_SELECT_INPUT_REG(base)		((base)->SW_MUX_CTL_PAD_IOMUXC_VIDEO_IN0_IPP_IND_FID_SELECT_INPUT)
#define IOMUXC_SW_MUX_CTL_PAD_VIDEO_IN0_IPP_IND_PIX_CLK_SELECT_INPUT_REG(base)		((base)->SW_MUX_CTL_PAD_IOMUXC_VIDEO_IN0_IPP_IND_PIX_CLK_SELECT_INPUT)

/* IOMUXC - Peripheral instance base addresses */
/** Peripheral IOMUXC base pointer */
#define IOMUXC                                   ((IOMUXC_Type *)IOMUXC_BASE)
#define IOMUXC_BASE_PTR                          (IOMUXC)
/** Array initializer of IOMUXC peripheral base adresses */
#define IOMUXC_BASE_ADDRS                        { IOMUXC_BASE }
/** Array initializer of IOMUXC peripheral base pointers */
#define IOMUXC_BASE_PTRS                         { IOMUXC }

#define IOMUXC_PTA6							IOMUXC_SW_MUX_CTL_PAD_PTA6_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTA8							IOMUXC_SW_MUX_CTL_PAD_PTA8_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTA9							IOMUXC_SW_MUX_CTL_PAD_PTA9_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTA10							IOMUXC_SW_MUX_CTL_PAD_PTA10_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTA11							IOMUXC_SW_MUX_CTL_PAD_PTA11_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTA12							IOMUXC_SW_MUX_CTL_PAD_PTA12_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTA16							IOMUXC_SW_MUX_CTL_PAD_PTA16_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTA17							IOMUXC_SW_MUX_CTL_PAD_PTA17_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTA18							IOMUXC_SW_MUX_CTL_PAD_PTA18_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTA19							IOMUXC_SW_MUX_CTL_PAD_PTA19_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTA20							IOMUXC_SW_MUX_CTL_PAD_PTA20_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTA21							IOMUXC_SW_MUX_CTL_PAD_PTA21_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTA22							IOMUXC_SW_MUX_CTL_PAD_PTA22_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTA23							IOMUXC_SW_MUX_CTL_PAD_PTA23_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTA24							IOMUXC_SW_MUX_CTL_PAD_PTA24_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTA25							IOMUXC_SW_MUX_CTL_PAD_PTA25_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTA26							IOMUXC_SW_MUX_CTL_PAD_PTA26_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTA27							IOMUXC_SW_MUX_CTL_PAD_PTA27_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTA28							IOMUXC_SW_MUX_CTL_PAD_PTA28_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTA29							IOMUXC_SW_MUX_CTL_PAD_PTA29_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTA30							IOMUXC_SW_MUX_CTL_PAD_PTA30_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTA31							IOMUXC_SW_MUX_CTL_PAD_PTA31_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB0							IOMUXC_SW_MUX_CTL_PAD_PTB0_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB1							IOMUXC_SW_MUX_CTL_PAD_PTB1_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB2							IOMUXC_SW_MUX_CTL_PAD_PTB2_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB3							IOMUXC_SW_MUX_CTL_PAD_PTB3_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB4							IOMUXC_SW_MUX_CTL_PAD_PTB4_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB5							IOMUXC_SW_MUX_CTL_PAD_PTB5_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB6							IOMUXC_SW_MUX_CTL_PAD_PTB6_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB7							IOMUXC_SW_MUX_CTL_PAD_PTB7_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB8							IOMUXC_SW_MUX_CTL_PAD_PTB8_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB9							IOMUXC_SW_MUX_CTL_PAD_PTB9_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB10							IOMUXC_SW_MUX_CTL_PAD_PTB10_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB11							IOMUXC_SW_MUX_CTL_PAD_PTB11_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB12							IOMUXC_SW_MUX_CTL_PAD_PTB12_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB13							IOMUXC_SW_MUX_CTL_PAD_PTB13_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB14							IOMUXC_SW_MUX_CTL_PAD_PTB14_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB15							IOMUXC_SW_MUX_CTL_PAD_PTB15_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB16							IOMUXC_SW_MUX_CTL_PAD_PTB16_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB17							IOMUXC_SW_MUX_CTL_PAD_PTB17_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB18							IOMUXC_SW_MUX_CTL_PAD_PTB18_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB19							IOMUXC_SW_MUX_CTL_PAD_PTB19_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB20							IOMUXC_SW_MUX_CTL_PAD_PTB20_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB21							IOMUXC_SW_MUX_CTL_PAD_PTB21_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB22							IOMUXC_SW_MUX_CTL_PAD_PTB22_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTC0							IOMUXC_SW_MUX_CTL_PAD_PTC0_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTC1							IOMUXC_SW_MUX_CTL_PAD_PTC1_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTC2							IOMUXC_SW_MUX_CTL_PAD_PTC2_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTC3							IOMUXC_SW_MUX_CTL_PAD_PTC3_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTC4							IOMUXC_SW_MUX_CTL_PAD_PTC4_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTC5							IOMUXC_SW_MUX_CTL_PAD_PTC5_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTC6							IOMUXC_SW_MUX_CTL_PAD_PTC6_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTC7							IOMUXC_SW_MUX_CTL_PAD_PTC7_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTC8							IOMUXC_SW_MUX_CTL_PAD_PTC8_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTC9							IOMUXC_SW_MUX_CTL_PAD_PTC9_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTC10							IOMUXC_SW_MUX_CTL_PAD_PTC10_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTC11							IOMUXC_SW_MUX_CTL_PAD_PTC11_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTC12							IOMUXC_SW_MUX_CTL_PAD_PTC12_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTC13							IOMUXC_SW_MUX_CTL_PAD_PTC13_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTC14							IOMUXC_SW_MUX_CTL_PAD_PTC14_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTC15							IOMUXC_SW_MUX_CTL_PAD_PTC15_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTC16							IOMUXC_SW_MUX_CTL_PAD_PTC16_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTC17							IOMUXC_SW_MUX_CTL_PAD_PTC17_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD31							IOMUXC_SW_MUX_CTL_PAD_PTD31_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD30							IOMUXC_SW_MUX_CTL_PAD_PTD30_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD29							IOMUXC_SW_MUX_CTL_PAD_PTD29_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD28							IOMUXC_SW_MUX_CTL_PAD_PTD28_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD27							IOMUXC_SW_MUX_CTL_PAD_PTD27_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD26							IOMUXC_SW_MUX_CTL_PAD_PTD26_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD25							IOMUXC_SW_MUX_CTL_PAD_PTD25_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD24							IOMUXC_SW_MUX_CTL_PAD_PTD24_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD23							IOMUXC_SW_MUX_CTL_PAD_PTD23_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD22							IOMUXC_SW_MUX_CTL_PAD_PTD22_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD21							IOMUXC_SW_MUX_CTL_PAD_PTD21_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD20							IOMUXC_SW_MUX_CTL_PAD_PTD20_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD19							IOMUXC_SW_MUX_CTL_PAD_PTD19_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD18							IOMUXC_SW_MUX_CTL_PAD_PTD18_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD17							IOMUXC_SW_MUX_CTL_PAD_PTD17_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD16							IOMUXC_SW_MUX_CTL_PAD_PTD16_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD0							IOMUXC_SW_MUX_CTL_PAD_PTD0_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD1							IOMUXC_SW_MUX_CTL_PAD_PTD1_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD2							IOMUXC_SW_MUX_CTL_PAD_PTD2_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD3							IOMUXC_SW_MUX_CTL_PAD_PTD3_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD4							IOMUXC_SW_MUX_CTL_PAD_PTD4_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD5							IOMUXC_SW_MUX_CTL_PAD_PTD5_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD6							IOMUXC_SW_MUX_CTL_PAD_PTD6_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD7							IOMUXC_SW_MUX_CTL_PAD_PTD7_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD8							IOMUXC_SW_MUX_CTL_PAD_PTD8_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD9							IOMUXC_SW_MUX_CTL_PAD_PTD9_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD10							IOMUXC_SW_MUX_CTL_PAD_PTD10_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD11							IOMUXC_SW_MUX_CTL_PAD_PTD11_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD12							IOMUXC_SW_MUX_CTL_PAD_PTD12_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTD13							IOMUXC_SW_MUX_CTL_PAD_PTD13_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB23							IOMUXC_SW_MUX_CTL_PAD_PTB23_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB24							IOMUXC_SW_MUX_CTL_PAD_PTB24_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB25							IOMUXC_SW_MUX_CTL_PAD_PTB25_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB26							IOMUXC_SW_MUX_CTL_PAD_PTB26_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB27							IOMUXC_SW_MUX_CTL_PAD_PTB27_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTB28							IOMUXC_SW_MUX_CTL_PAD_PTB28_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTC26							IOMUXC_SW_MUX_CTL_PAD_PTC26_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTC27							IOMUXC_SW_MUX_CTL_PAD_PTC27_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTC28							IOMUXC_SW_MUX_CTL_PAD_PTC28_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTC29							IOMUXC_SW_MUX_CTL_PAD_PTC29_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTC30							IOMUXC_SW_MUX_CTL_PAD_PTC30_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTC31							IOMUXC_SW_MUX_CTL_PAD_PTC31_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE0							IOMUXC_SW_MUX_CTL_PAD_PTE0_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE1							IOMUXC_SW_MUX_CTL_PAD_PTE1_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE2							IOMUXC_SW_MUX_CTL_PAD_PTE2_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE3							IOMUXC_SW_MUX_CTL_PAD_PTE3_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE4							IOMUXC_SW_MUX_CTL_PAD_PTE4_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE5							IOMUXC_SW_MUX_CTL_PAD_PTE5_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE6							IOMUXC_SW_MUX_CTL_PAD_PTE6_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE7							IOMUXC_SW_MUX_CTL_PAD_PTE7_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE8							IOMUXC_SW_MUX_CTL_PAD_PTE8_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE9							IOMUXC_SW_MUX_CTL_PAD_PTE9_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE10							IOMUXC_SW_MUX_CTL_PAD_PTE10_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE11							IOMUXC_SW_MUX_CTL_PAD_PTE11_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE12							IOMUXC_SW_MUX_CTL_PAD_PTE12_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE13							IOMUXC_SW_MUX_CTL_PAD_PTE13_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE14							IOMUXC_SW_MUX_CTL_PAD_PTE14_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE15							IOMUXC_SW_MUX_CTL_PAD_PTE15_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE16							IOMUXC_SW_MUX_CTL_PAD_PTE16_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE17							IOMUXC_SW_MUX_CTL_PAD_PTE17_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE18							IOMUXC_SW_MUX_CTL_PAD_PTE18_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE19							IOMUXC_SW_MUX_CTL_PAD_PTE19_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE20							IOMUXC_SW_MUX_CTL_PAD_PTE20_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE21							IOMUXC_SW_MUX_CTL_PAD_PTE21_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE22							IOMUXC_SW_MUX_CTL_PAD_PTE22_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE23							IOMUXC_SW_MUX_CTL_PAD_PTE23_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE24							IOMUXC_SW_MUX_CTL_PAD_PTE24_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE25							IOMUXC_SW_MUX_CTL_PAD_PTE25_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE26							IOMUXC_SW_MUX_CTL_PAD_PTE26_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE27							IOMUXC_SW_MUX_CTL_PAD_PTE27_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTE28							IOMUXC_SW_MUX_CTL_PAD_PTE28_REG(IOMUXC_BASE_PTR)
#define IOMUXC_PTA7							IOMUXC_SW_MUX_CTL_PAD_PTA7_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_RESETB						IOMUXC_SW_MUX_CTL_PAD_DDR_RESETB_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_A15							IOMUXC_SW_MUX_CTL_PAD_DDR_A15_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_A14							IOMUXC_SW_MUX_CTL_PAD_DDR_A14_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_A13							IOMUXC_SW_MUX_CTL_PAD_DDR_A13_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_A12							IOMUXC_SW_MUX_CTL_PAD_DDR_A12_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_A11							IOMUXC_SW_MUX_CTL_PAD_DDR_A11_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_A10							IOMUXC_SW_MUX_CTL_PAD_DDR_A10_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_A9							IOMUXC_SW_MUX_CTL_PAD_DDR_A9_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_A8							IOMUXC_SW_MUX_CTL_PAD_DDR_A8_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_A7							IOMUXC_SW_MUX_CTL_PAD_DDR_A7_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_A6							IOMUXC_SW_MUX_CTL_PAD_DDR_A6_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_A5							IOMUXC_SW_MUX_CTL_PAD_DDR_A5_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_A4							IOMUXC_SW_MUX_CTL_PAD_DDR_A4_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_A3							IOMUXC_SW_MUX_CTL_PAD_DDR_A3_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_A2							IOMUXC_SW_MUX_CTL_PAD_DDR_A2_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_A1							IOMUXC_SW_MUX_CTL_PAD_DDR_A1_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_A0							IOMUXC_SW_MUX_CTL_PAD_DDR_A0_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_BA2							IOMUXC_SW_MUX_CTL_PAD_DDR_BA2_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_BA1							IOMUXC_SW_MUX_CTL_PAD_DDR_BA1_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_BA0							IOMUXC_SW_MUX_CTL_PAD_DDR_BA0_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_CAS_B						IOMUXC_SW_MUX_CTL_PAD_DDR_CAS_B_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_CKE0						IOMUXC_SW_MUX_CTL_PAD_DDR_CKE0_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_CLK0						IOMUXC_SW_MUX_CTL_PAD_DDR_CLK0_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_CS_B0						IOMUXC_SW_MUX_CTL_PAD_DDR_CS_B0_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_CS_D15						IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D15_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_CS_D14						IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D14_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_CS_D13						IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D13_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_CS_D12						IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D12_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_CS_D11						IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D11_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_CS_D10						IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D10_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_CS_D9						IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D9_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_CS_D8						IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D8_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_CS_D7						IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D7_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_CS_D6						IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D6_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_CS_D5						IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D5_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_CS_D4						IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D4_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_CS_D3						IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D3_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_CS_D2						IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D2_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_CS_D1						IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D1_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_CS_D0						IOMUXC_SW_MUX_CTL_PAD_DDR_CS_D0_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_DQM1						IOMUXC_SW_MUX_CTL_PAD_DDR_DQM1_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_DQM0						IOMUXC_SW_MUX_CTL_PAD_DDR_DQM0_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_DQS1						IOMUXC_SW_MUX_CTL_PAD_DDR_DQS1_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_DQS0						IOMUXC_SW_MUX_CTL_PAD_DDR_DQS0_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_RAS_B						IOMUXC_SW_MUX_CTL_PAD_DDR_RAS_B_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_WE_B						IOMUXC_SW_MUX_CTL_PAD_DDR_WE_B_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_ODT0						IOMUXC_SW_MUX_CTL_PAD_DDR_ODT0_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_ODT1						IOMUXC_SW_MUX_CTL_PAD_DDR_ODT1_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_DDRBYTE1						IOMUXC_SW_MUX_CTL_PAD_DDR_DDRBYTE1_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DDR_DDRBYTE2						IOMUXC_SW_MUX_CTL_PAD_DDR_DDRBYTE2_REG(IOMUXC_BASE_PTR)
#define IOMUXC_CCM_AUD_EXT_CLK_SELECT_INPUT				IOMUXC_SW_MUX_CTL_PAD_CCM_AUD_EXT_CLK_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_CCM_ENET_EXT_CLK_SELECT_INPUT				IOMUXC_SW_MUX_CTL_PAD_CCM_ENET_EXT_CLK_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_CCM_ENET_TS_CLK_SELECT_INPUT				IOMUXC_SW_MUX_CTL_PAD_CCM_ENET_TS_CLK_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DSPI1_IPP_IND_SCK_SELECT_INPUT				IOMUXC_SW_MUX_CTL_PAD_DSPI1_IPP_IND_SCK_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DSPI1_IPP_IND_SIN_SELECT_INPUT				IOMUXC_SW_MUX_CTL_PAD_DSPI1_IPP_IND_SIN_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_DSPI1_IPP_IND_SS_B_SELECT_INPUT				IOMUXC_SW_MUX_CTL_PAD_DSPI1_IPP_IND_SS_B_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_MAC0_TIMER_0_SELECT_INPUT				IOMUXC_SW_MUX_CTL_PAD_MAC0_TIMER_0_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_MAC0_TIMER_1_SELECT_INPUT				IOMUXC_SW_MUX_CTL_PAD_MAC0_TIMER_1_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_ESAI_IPP_IND_FST_SELECT_INPUT				IOMUXC_SW_MUX_CTL_PAD_ESAI_IPP_IND_FST_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_ESAI_IPP_IND_SCKT_SELECT_INPUT				IOMUXC_SW_MUX_CTL_PAD_ESAI_IPP_IND_SCKT_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_ESAI_IPP_IND_SDO0_SELECT_INPUT				IOMUXC_SW_MUX_CTL_PAD_ESAI_IPP_IND_SDO0_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_ESAI_IPP_IND_SDO1_SELECT_INPUT				IOMUXC_SW_MUX_CTL_PAD_ESAI_IPP_IND_SDO1_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_ESAI_IPP_IND_SDO2_SDI3_SELECT_INPUT			IOMUXC_SW_MUX_CTL_PAD_ESAI_IPP_IND_SDO2_SDI3_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_ESAI_IPP_IND_SDO3_SDI2_SELECT_INPUT			IOMUXC_SW_MUX_CTL_PAD_ESAI_IPP_IND_SDO3_SDI2_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_ESAI_IPP_IND_SDO4_SDI1_SELECT_INPUT			IOMUXC_SW_MUX_CTL_PAD_ESAI_IPP_IND_SDO4_SDI1_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_ESAI_IPP_IND_SDO5_SDI0_SELECT_INPUT			IOMUXC_SW_MUX_CTL_PAD_ESAI_IPP_IND_SDO5_SDI0_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_FLEXTIMER1_IPP_IND_FTM_CH_0_SELECT_INPUT		IOMUXC_SW_MUX_CTL_PAD_FLEXTIMER1_IPP_IND_FTM_CH_0_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_FLEXTIMER1_IPP_IND_FTM_CH_1_SELECT_INPUT		IOMUXC_SW_MUX_CTL_PAD_FLEXTIMER1_IPP_IND_FTM_CH_1_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_FLEXTIMER1_IPP_IND_FTM_PHA_SELECT_INPUT			IOMUXC_SW_MUX_CTL_PAD_FLEXTIMER1_IPP_IND_FTM_PHA_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_FLEXTIMER1_IPP_IND_FTM_PHB_SELECT_INPUT			IOMUXC_SW_MUX_CTL_PAD_FLEXTIMER1_IPP_IND_FTM_PHB_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_I2C0_IPP_SCL_IND_SELECT_INPUT				IOMUXC_SW_MUX_CTL_PAD_I2C0_IPP_SCL_IND_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_I2C0_IPP_SDA_IND_SELECT_INPUT				IOMUXC_SW_MUX_CTL_PAD_I2C0_IPP_SDA_IND_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_I2C1_IPP_SCL_IND_SELECT_INPUT				IOMUXC_SW_MUX_CTL_PAD_I2C1_IPP_SCL_IND_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_I2C1_IPP_SDA_IND_SELECT_INPUT				IOMUXC_SW_MUX_CTL_PAD_I2C1_IPP_SDA_IND_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_I2C2_IPP_SCL_IND_SELECT_INPUT				IOMUXC_SW_MUX_CTL_PAD_I2C2_IPP_SCL_IND_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_I2C2_IPP_SDA_IND_SELECT_INPUT				IOMUXC_SW_MUX_CTL_PAD_I2C2_IPP_SDA_IND_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_MLB_TOP_MLBCLK_IN_SELECT_INPUT				IOMUXC_SW_MUX_CTL_PAD_MLB_TOP_MLBCLK_IN_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_MLB_TOP_MLBDAT_IN_SELECT_INPUT				IOMUXC_SW_MUX_CTL_PAD_MLB_TOP_MLBDAT_IN_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_MLB_TOP_MLBSIG_IN_SELECT_INPUT				IOMUXC_SW_MUX_CTL_PAD_MLB_TOP_MLBSIG_IN_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_SAI1_IPP_IND_SAI_TXSYNC_SELECT_INPUT			IOMUXC_SW_MUX_CTL_PAD_SAI1_IPP_IND_SAI_TXSYNC_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_SAI2_IPP_IND_SAI_RXBCLK_SELECT_INPUT			IOMUXC_SW_MUX_CTL_PAD_SAI2_IPP_IND_SAI_RXBCLK_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_SAI2_IPP_IND_SAI_RXDATA_0_SELECT_INPUT			IOMUXC_SW_MUX_CTL_PAD_SAI2_IPP_IND_SAI_RXDATA_0_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_SAI2_IPP_IND_SAI_RXSYNC_SELECT_INPUT			IOMUXC_SW_MUX_CTL_PAD_SAI2_IPP_IND_SAI_RXSYNC_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_SAI2_IPP_IND_SAI_TXBCLK_SELECT_INPUT			IOMUXC_SW_MUX_CTL_PAD_SAI2_IPP_IND_SAI_TXBCLK_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_SAI2_IPP_IND_SAI_TXSYNC_SELECT_INPUT			IOMUXC_SW_MUX_CTL_PAD_SAI2_IPP_IND_SAI_TXSYNC_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_SCI_FLX1_IPP_IND_CTS_B_SELECT_INPUT			IOMUXC_SW_MUX_CTL_PAD_SCI_FLX1_IPP_IND_CTS_B_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_SCI_FLX1_IPP_IND_SCI_RX_SELECT_INPUT			IOMUXC_SW_MUX_CTL_PAD_SCI_FLX1_IPP_IND_SCI_RX_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_SCI_FLX1_IPP_IND_SCI_TX_SELECT_INPUT			IOMUXC_SW_MUX_CTL_PAD_SCI_FLX1_IPP_IND_SCI_TX_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_SCI_FLX2_IPP_IND_CTS_B_SELECT_INPUT			IOMUXC_SW_MUX_CTL_PAD_SCI_FLX2_IPP_IND_CTS_B_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_SCI_FLX2_IPP_IND_SCI_RX_SELECT_INPUT			IOMUXC_SW_MUX_CTL_PAD_SCI_FLX2_IPP_IND_SCI_RX_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_SCI_FLX2_IPP_IND_SCI_TX_SELECT_INPUT			IOMUXC_SW_MUX_CTL_PAD_SCI_FLX2_IPP_IND_SCI_TX_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_SCI_FLX3_IPP_IND_SCI_RX_SELECT_INPUT			IOMUXC_SW_MUX_CTL_PAD_SCI_FLX3_IPP_IND_SCI_RX_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_SCI_FLX3_IPP_IND_SCI_TX_SELECT_INPUT			IOMUXC_SW_MUX_CTL_PAD_SCI_FLX3_IPP_IND_SCI_TX_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_SRC_IPP_BOOT_CFG_18_SELECT_INPUT			IOMUXC_SW_MUX_CTL_PAD_SRC_IPP_BOOT_CFG_18_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_SRC_IPP_BOOT_CFG_19_SELECT_INPUT			IOMUXC_SW_MUX_CTL_PAD_SRC_IPP_BOOT_CFG_19_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_SRC_IPP_BOOT_CFG_20_SELECT_INPUT			IOMUXC_SW_MUX_CTL_PAD_SRC_IPP_BOOT_CFG_20_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_VIDEO_IN0_IPP_IND_DE_SELECT_INPUT			IOMUXC_SW_MUX_CTL_PAD_VIDEO_IN0_IPP_IND_DE_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_VIDEO_IN0_IPP_IND_FID_SELECT_INPUT			IOMUXC_SW_MUX_CTL_PAD_VIDEO_IN0_IPP_IND_FID_SELECT_INPUT_REG(IOMUXC_BASE_PTR)
#define IOMUXC_VIDEO_IN0_IPP_IND_PIX_CLK_SELECT_INPUT			IOMUXC_SW_MUX_CTL_PAD_VIDEO_IN0_IPP_IND_PIX_CLK_SELECT_INPUT_REG(IOMUXC_BASE_PTR)

#define IOMUXC_PAD(mode, speed, dse, pus, flags) \
	((IOMUXC_##mode) << IOMUXC_MUX_MODE_SHIFT | \
	 (IOMUXC_##speed) << IOMUXC_SPEED_SHIFT | \
	 (IOMUXC_##dse) << IOMUXC_DSE_SHIFT | \
	 (IOMUXC_##pus) << IOMUXC_PUS_SHIFT | \
	 (flags))

#define IOMUXC_MUX_MODE_SHIFT		20
#define IOMUXC_MUX_MODE_MASK		(0x7 << 20)
#define IOMUXC_MUX_MODE_ALT0		0x0
#define IOMUXC_MUX_MODE_ALT1		0x1
#define IOMUXC_MUX_MODE_ALT2		0x2
#define IOMUXC_MUX_MODE_ALT3		0x3
#define IOMUXC_MUX_MODE_ALT4		0x4
#define IOMUXC_MUX_MODE_ALT5		0x5
#define IOMUXC_MUX_MODE_ALT6		0x6
#define IOMUXC_MUX_MODE_ALT7		0x7
#define IOMUXC_SPEED_SHIFT		12
#define IOMUXC_SPEED_MASK		(0x3 << 12)
#define IOMUXC_SPEED_LOW		0x0
#define IOMUXC_SPEED_MEDIUM		0x1
#define IOMUXC_SPEED_HIGH		0x3
#define IOMUXC_SRE			(0x1 << 11)
#define IOMUXC_ODE			(0x1 << 10)
#define IOMUXC_HYS			(0x1 << 9)
#define IOMUXC_DSE_SHIFT		6
#define IOMUXC_DSE_MASK		(0x7 << 6)
#define IOMUXC_DSE_OFF			0x0
#define IOMUXC_DSE_150OHM		0x1
#define IOMUXC_DSE_75OHM		0x2
#define IOMUXC_DSE_50OHM		0x3
#define IOMUXC_DSE_37OHM		0x4
#define IOMUXC_DSE_30OHM		0x5
#define IOMUXC_DSE_25OHM		0x6
#define IOMUXC_DSE_20OHM		0x7
#define IOMUXC_PUS_SHIFT		4
#define IOMUXC_PUS_MASK		(0x3 << 4)
#define IOMUXC_PUS_PD_100KOHM		0x0
#define IOMUXC_PUS_PU_47KOHM		0x1
#define IOMUXC_PUS_PU_100KOHM		0x2
#define IOMUXC_PUS_PU_22KOHM		0x3
#define IOMUXC_PKE			(0x1 << 3)
#define IOMUXC_PUE			(0x1 << 2)
#define IOMUXC_OBE			(0x1 << 1)
#define IOMUXC_IBE			(0x1 << 0)

/* ----------------------------------------------------------------------------
   -- SEMA4 Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SEMA4_Peripheral_Access_Layer SEMA4 Peripheral Access Layer
 * @{
 */

typedef struct {
	__IO uint8_t GATE03;		/**< Semaphores GATE 03 Register, offset: 0x0 */
	__IO uint8_t GATE02;		/**< Semaphores GATE 02 Register, offset: 0x1 */
	__IO uint8_t GATE01;		/**< Semaphores GATE 01 Register, offset: 0x2 */
	__IO uint8_t GATE00;		/**< Semaphores GATE 00 Register, offset: 0x3 */
	__IO uint8_t GATE07;		/**< Semaphores GATE 07 Register, offset: 0x4 */
	__IO uint8_t GATE06;		/**< Semaphores GATE 06 Register, offset: 0x5 */
	__IO uint8_t GATE05;		/**< Semaphores GATE 05 Register, offset: 0x6 */
	__IO uint8_t GATE04;		/**< Semaphores GATE 04 Register, offset: 0x7 */
	__IO uint8_t GATE11;		/**< Semaphores GATE 11 Register, offset: 0x8 */
	__IO uint8_t GATE10;		/**< Semaphores GATE 10 Register, offset: 0x9 */
	__IO uint8_t GATE09;		/**< Semaphores GATE 09 Register, offset: 0xA */
	__IO uint8_t GATE08;		/**< Semaphores GATE 08 Register, offset: 0xB */
	__IO uint8_t GATE15;		/**< Semaphores GATE 15 Register, offset: 0xC */
	__IO uint8_t GATE14;		/**< Semaphores GATE 14 Register, offset: 0xD */
	__IO uint8_t GATE13;		/**< Semaphores GATE 13 Register, offset: 0xE */
	__IO uint8_t GATE12;		/**< Semaphores GATE 12 Register, offset: 0xF */
	uint8_t RESERVED_0[48];
	struct {			/* offset: 0x40, array step: 0x8 */
		__IO uint16_t INE;		/**< Semaphores Processor n IRQ Notification Enable, array offset: 0x40, array step: 0x8 */
		uint8_t RESERVED_0[6];
	} CPnINE[2];
	uint8_t RESERVED_1[48];
	struct {			/* offset: 0x80, array step: 0x8 */
		__IO uint16_t NTF;		/**< Semaphores Processor n IRQ Notification, array offset: 0x80, array step: 0x8 */
		uint8_t RESERVED_0[6];
	} CPnNTF[2];
	uint8_t RESERVED_2[112];
	__IO uint16_t RSTGT;		/**< Semaphores (Secure) Reset Gate n, offset: 0x100 */
	uint8_t RESERVED_3[2];
	__IO uint16_t RSTNTF;		/**< Semaphores (Secure) Reset IRQ Notification, offset: 0x104 */
} SEMA4_Type, *SEMA4_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- SEMA4 - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SEMA4_Register_Accessor_Macros SEMA4 - Register accessor macros
 * @{
 */

/* SEMA4 - Register accessors */
#define SEMA4_GATE03_REG(base)                   ((base)->Gate03)
#define SEMA4_GATE02_REG(base)                   ((base)->Gate02)
#define SEMA4_GATE01_REG(base)                   ((base)->Gate01)
#define SEMA4_GATE00_REG(base)                   ((base)->Gate00)
#define SEMA4_GATE07_REG(base)                   ((base)->Gate07)
#define SEMA4_GATE06_REG(base)                   ((base)->Gate06)
#define SEMA4_GATE05_REG(base)                   ((base)->Gate05)
#define SEMA4_GATE04_REG(base)                   ((base)->Gate04)
#define SEMA4_GATE11_REG(base)                   ((base)->Gate11)
#define SEMA4_GATE10_REG(base)                   ((base)->Gate10)
#define SEMA4_GATE09_REG(base)                   ((base)->Gate09)
#define SEMA4_GATE08_REG(base)                   ((base)->Gate08)
#define SEMA4_GATE15_REG(base)                   ((base)->Gate15)
#define SEMA4_GATE14_REG(base)                   ((base)->Gate14)
#define SEMA4_GATE13_REG(base)                   ((base)->Gate13)
#define SEMA4_GATE12_REG(base)                   ((base)->Gate12)
#define SEMA4_CPINE_REG(base,index)              ((base)->CPINE[index].CPINE)
#define SEMA4_CPNTF_REG(base,index)              ((base)->CPNTF[index].CPNTF)
#define SEMA4_RSTGT_REG(base)                    ((base)->RSTGT)
#define SEMA4_RSTNTF_REG(base)                   ((base)->RSTNTF)

/*!
 * @}
 */ /* end of group SEMA4_Register_Accessor_Macros */

/* ----------------------------------------------------------------------------
   -- SEMA4 Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SEMA4_Register_Masks SEMA4 Register Masks
 * @{
 */

/* Gate03 Bit Fields */
#define SEMA4_GATE03_GTFSM_MASK                  0x3u
#define SEMA4_GATE03_GTFSM_SHIFT                 0
#define SEMA4_GATE03_GTFSM(x)                    (((uint8_t)(((uint8_t)(x))<<SEMA4_GATE03_GTFSM_SHIFT))&SEMA4_GATE03_GTFSM_MASK)
/* Gate02 Bit Fields */
#define SEMA4_GATE02_GTFSM_MASK                  0x3u
#define SEMA4_GATE02_GTFSM_SHIFT                 0
#define SEMA4_GATE02_GTFSM(x)                    (((uint8_t)(((uint8_t)(x))<<SEMA4_GATE02_GTFSM_SHIFT))&SEMA4_GATE02_GTFSM_MASK)
/* Gate01 Bit Fields */
#define SEMA4_GATE01_GTFSM_MASK                  0x3u
#define SEMA4_GATE01_GTFSM_SHIFT                 0
#define SEMA4_GATE01_GTFSM(x)                    (((uint8_t)(((uint8_t)(x))<<SEMA4_GATE01_GTFSM_SHIFT))&SEMA4_GATE01_GTFSM_MASK)
/* Gate00 Bit Fields */
#define SEMA4_GATE00_GTFSM_MASK                  0x3u
#define SEMA4_GATE00_GTFSM_SHIFT                 0
#define SEMA4_GATE00_GTFSM(x)                    (((uint8_t)(((uint8_t)(x))<<SEMA4_GATE00_GTFSM_SHIFT))&SEMA4_GATE00_GTFSM_MASK)
/* Gate07 Bit Fields */
#define SEMA4_GATE07_GTFSM_MASK                  0x3u
#define SEMA4_GATE07_GTFSM_SHIFT                 0
#define SEMA4_GATE07_GTFSM(x)                    (((uint8_t)(((uint8_t)(x))<<SEMA4_GATE07_GTFSM_SHIFT))&SEMA4_GATE07_GTFSM_MASK)
/* Gate06 Bit Fields */
#define SEMA4_GATE06_GTFSM_MASK                  0x3u
#define SEMA4_GATE06_GTFSM_SHIFT                 0
#define SEMA4_GATE06_GTFSM(x)                    (((uint8_t)(((uint8_t)(x))<<SEMA4_GATE06_GTFSM_SHIFT))&SEMA4_GATE06_GTFSM_MASK)
/* Gate05 Bit Fields */
#define SEMA4_GATE05_GTFSM_MASK                  0x3u
#define SEMA4_GATE05_GTFSM_SHIFT                 0
#define SEMA4_GATE05_GTFSM(x)                    (((uint8_t)(((uint8_t)(x))<<SEMA4_GATE05_GTFSM_SHIFT))&SEMA4_GATE05_GTFSM_MASK)
/* Gate04 Bit Fields */
#define SEMA4_GATE04_GTFSM_MASK                  0x3u
#define SEMA4_GATE04_GTFSM_SHIFT                 0
#define SEMA4_GATE04_GTFSM(x)                    (((uint8_t)(((uint8_t)(x))<<SEMA4_GATE04_GTFSM_SHIFT))&SEMA4_GATE04_GTFSM_MASK)
/* Gate11 Bit Fields */
#define SEMA4_GATE11_GTFSM_MASK                  0x3u
#define SEMA4_GATE11_GTFSM_SHIFT                 0
#define SEMA4_GATE11_GTFSM(x)                    (((uint8_t)(((uint8_t)(x))<<SEMA4_GATE11_GTFSM_SHIFT))&SEMA4_GATE11_GTFSM_MASK)
/* Gate10 Bit Fields */
#define SEMA4_GATE10_GTFSM_MASK                  0x3u
#define SEMA4_GATE10_GTFSM_SHIFT                 0
#define SEMA4_GATE10_GTFSM(x)                    (((uint8_t)(((uint8_t)(x))<<SEMA4_GATE10_GTFSM_SHIFT))&SEMA4_GATE10_GTFSM_MASK)
/* Gate09 Bit Fields */
#define SEMA4_GATE09_GTFSM_MASK                  0x3u
#define SEMA4_GATE09_GTFSM_SHIFT                 0
#define SEMA4_GATE09_GTFSM(x)                    (((uint8_t)(((uint8_t)(x))<<SEMA4_GATE09_GTFSM_SHIFT))&SEMA4_GATE09_GTFSM_MASK)
/* Gate08 Bit Fields */
#define SEMA4_GATE08_GTFSM_MASK                  0x3u
#define SEMA4_GATE08_GTFSM_SHIFT                 0
#define SEMA4_GATE08_GTFSM(x)                    (((uint8_t)(((uint8_t)(x))<<SEMA4_GATE08_GTFSM_SHIFT))&SEMA4_GATE08_GTFSM_MASK)
/* Gate15 Bit Fields */
#define SEMA4_GATE15_GTFSM_MASK                  0x3u
#define SEMA4_GATE15_GTFSM_SHIFT                 0
#define SEMA4_GATE15_GTFSM(x)                    (((uint8_t)(((uint8_t)(x))<<SEMA4_GATE15_GTFSM_SHIFT))&SEMA4_GATE15_GTFSM_MASK)
/* Gate14 Bit Fields */
#define SEMA4_GATE14_GTFSM_MASK                  0x3u
#define SEMA4_GATE14_GTFSM_SHIFT                 0
#define SEMA4_GATE14_GTFSM(x)                    (((uint8_t)(((uint8_t)(x))<<SEMA4_GATE14_GTFSM_SHIFT))&SEMA4_GATE14_GTFSM_MASK)
/* Gate13 Bit Fields */
#define SEMA4_GATE13_GTFSM_MASK                  0x3u
#define SEMA4_GATE13_GTFSM_SHIFT                 0
#define SEMA4_GATE13_GTFSM(x)                    (((uint8_t)(((uint8_t)(x))<<SEMA4_GATE13_GTFSM_SHIFT))&SEMA4_GATE13_GTFSM_MASK)
/* Gate12 Bit Fields */
#define SEMA4_GATE12_GTFSM_MASK                  0x3u
#define SEMA4_GATE12_GTFSM_SHIFT                 0
#define SEMA4_GATE12_GTFSM(x)                    (((uint8_t)(((uint8_t)(x))<<SEMA4_GATE12_GTFSM_SHIFT))&SEMA4_GATE12_GTFSM_MASK)
/* CPINE Bit Fields */
#define SEMA4_CPINE_INE7_MASK                    0x1u
#define SEMA4_CPINE_INE7_SHIFT                   0
#define SEMA4_CPINE_INE6_MASK                    0x2u
#define SEMA4_CPINE_INE6_SHIFT                   1
#define SEMA4_CPINE_INE5_MASK                    0x4u
#define SEMA4_CPINE_INE5_SHIFT                   2
#define SEMA4_CPINE_INE4_MASK                    0x8u
#define SEMA4_CPINE_INE4_SHIFT                   3
#define SEMA4_CPINE_INE3_MASK                    0x10u
#define SEMA4_CPINE_INE3_SHIFT                   4
#define SEMA4_CPINE_INE2_MASK                    0x20u
#define SEMA4_CPINE_INE2_SHIFT                   5
#define SEMA4_CPINE_INE1_MASK                    0x40u
#define SEMA4_CPINE_INE1_SHIFT                   6
#define SEMA4_CPINE_INE0_MASK                    0x80u
#define SEMA4_CPINE_INE0_SHIFT                   7
#define SEMA4_CPINE_INE15_MASK                   0x100u
#define SEMA4_CPINE_INE15_SHIFT                  8
#define SEMA4_CPINE_INE14_MASK                   0x200u
#define SEMA4_CPINE_INE14_SHIFT                  9
#define SEMA4_CPINE_INE13_MASK                   0x400u
#define SEMA4_CPINE_INE13_SHIFT                  10
#define SEMA4_CPINE_INE12_MASK                   0x800u
#define SEMA4_CPINE_INE12_SHIFT                  11
#define SEMA4_CPINE_INE11_MASK                   0x1000u
#define SEMA4_CPINE_INE11_SHIFT                  12
#define SEMA4_CPINE_INE10_MASK                   0x2000u
#define SEMA4_CPINE_INE10_SHIFT                  13
#define SEMA4_CPINE_INE9_MASK                    0x4000u
#define SEMA4_CPINE_INE9_SHIFT                   14
#define SEMA4_CPINE_INE8_MASK                    0x8000u
#define SEMA4_CPINE_INE8_SHIFT                   15
/* CPNTF Bit Fields */
#define SEMA4_CPNTF_GN7_MASK                     0x1u
#define SEMA4_CPNTF_GN7_SHIFT                    0
#define SEMA4_CPNTF_GN6_MASK                     0x2u
#define SEMA4_CPNTF_GN6_SHIFT                    1
#define SEMA4_CPNTF_GN5_MASK                     0x4u
#define SEMA4_CPNTF_GN5_SHIFT                    2
#define SEMA4_CPNTF_GN4_MASK                     0x8u
#define SEMA4_CPNTF_GN4_SHIFT                    3
#define SEMA4_CPNTF_GN3_MASK                     0x10u
#define SEMA4_CPNTF_GN3_SHIFT                    4
#define SEMA4_CPNTF_GN2_MASK                     0x20u
#define SEMA4_CPNTF_GN2_SHIFT                    5
#define SEMA4_CPNTF_GN1_MASK                     0x40u
#define SEMA4_CPNTF_GN1_SHIFT                    6
#define SEMA4_CPNTF_GN0_MASK                     0x80u
#define SEMA4_CPNTF_GN0_SHIFT                    7
#define SEMA4_CPNTF_GN15_MASK                    0x100u
#define SEMA4_CPNTF_GN15_SHIFT                   8
#define SEMA4_CPNTF_GN14_MASK                    0x200u
#define SEMA4_CPNTF_GN14_SHIFT                   9
#define SEMA4_CPNTF_GN13_MASK                    0x400u
#define SEMA4_CPNTF_GN13_SHIFT                   10
#define SEMA4_CPNTF_GN12_MASK                    0x800u
#define SEMA4_CPNTF_GN12_SHIFT                   11
#define SEMA4_CPNTF_GN11_MASK                    0x1000u
#define SEMA4_CPNTF_GN11_SHIFT                   12
#define SEMA4_CPNTF_GN10_MASK                    0x2000u
#define SEMA4_CPNTF_GN10_SHIFT                   13
#define SEMA4_CPNTF_GN9_MASK                     0x4000u
#define SEMA4_CPNTF_GN9_SHIFT                    14
#define SEMA4_CPNTF_GN8_MASK                     0x8000u
#define SEMA4_CPNTF_GN8_SHIFT                    15
/* RSTGT Bit Fields */
#define SEMA4_RSTGT_RSTGSM_RSTGMS_RSTGDP_MASK    0xFFu
#define SEMA4_RSTGT_RSTGSM_RSTGMS_RSTGDP_SHIFT   0
#define SEMA4_RSTGT_RSTGSM_RSTGMS_RSTGDP(x)      (((uint16_t)(((uint16_t)(x))<<SEMA4_RSTGT_RSTGSM_RSTGMS_RSTGDP_SHIFT))&SEMA4_RSTGT_RSTGSM_RSTGMS_RSTGDP_MASK)
#define SEMA4_RSTGT_RSTGTN_MASK                  0xFF00u
#define SEMA4_RSTGT_RSTGTN_SHIFT                 8
#define SEMA4_RSTGT_RSTGTN(x)                    (((uint16_t)(((uint16_t)(x))<<SEMA4_RSTGT_RSTGTN_SHIFT))&SEMA4_RSTGT_RSTGTN_MASK)
/* RSTNTF Bit Fields */
#define SEMA4_RSTNTF_RSTNSM_RSTNMS_RSTNDP_MASK   0xFFu
#define SEMA4_RSTNTF_RSTNSM_RSTNMS_RSTNDP_SHIFT  0
#define SEMA4_RSTNTF_RSTNSM_RSTNMS_RSTNDP(x)     (((uint16_t)(((uint16_t)(x))<<SEMA4_RSTNTF_RSTNSM_RSTNMS_RSTNDP_SHIFT))&SEMA4_RSTNTF_RSTNSM_RSTNMS_RSTNDP_MASK)
#define SEMA4_RSTNTF_RSTNTN_MASK                 0xFF00u
#define SEMA4_RSTNTF_RSTNTN_SHIFT                8
#define SEMA4_RSTNTF_RSTNTN(x)                   (((uint16_t)(((uint16_t)(x))<<SEMA4_RSTNTF_RSTNTN_SHIFT))&SEMA4_RSTNTF_RSTNTN_MASK)

/*!
 * @}
 */ /* end of group SEMA4_Register_Masks */

/** Peripheral SEMA4 base pointer */
#define SEMA4                                    ((SEMA4_Type *)SEMA4_BASE)
#define SEMA4_BASE_PTR                           (SEMA4)
/** Array initializer of SEMA4 peripheral base adresses */
#define SEMA4_BASE_ADDRS                         { SEMA4_BASE }
/** Array initializer of SEMA4 peripheral base pointers */
#define SEMA4_BASE_PTRS                          { SEMA4 }

/* ----------------------------------------------------------------------------
   -- SEMA4 - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SEMA4_Register_Accessor_Macros SEMA4 - Register accessor macros
 * @{
 */

/* SEMA4 - Register instance definitions */
/* SEMA4 */
#define SEMA4_GATE03		SEMA4_GATE03_REG(SEMA4_BASE_PTR)
#define SEMA4_GATE02		SEMA4_GATE02_REG(SEMA4_BASE_PTR)
#define SEMA4_GATE01		SEMA4_GATE01_REG(SEMA4_BASE_PTR)
#define SEMA4_GATE00		SEMA4_GATE00_REG(SEMA4_BASE_PTR)
#define SEMA4_GATE07		SEMA4_GATE07_REG(SEMA4_BASE_PTR)
#define SEMA4_GATE06		SEMA4_GATE06_REG(SEMA4_BASE_PTR)
#define SEMA4_GATE05		SEMA4_GATE05_REG(SEMA4_BASE_PTR)
#define SEMA4_GATE04		SEMA4_GATE04_REG(SEMA4_BASE_PTR)
#define SEMA4_GATE11		SEMA4_GATE11_REG(SEMA4_BASE_PTR)
#define SEMA4_GATE10		SEMA4_GATE10_REG(SEMA4_BASE_PTR)
#define SEMA4_GATE09		SEMA4_GATE09_REG(SEMA4_BASE_PTR)
#define SEMA4_GATE08		SEMA4_GATE08_REG(SEMA4_BASE_PTR)
#define SEMA4_GATE15		SEMA4_GATE15_REG(SEMA4_BASE_PTR)
#define SEMA4_GATE14		SEMA4_GATE14_REG(SEMA4_BASE_PTR)
#define SEMA4_GATE13		SEMA4_GATE13_REG(SEMA4_BASE_PTR)
#define SEMA4_GATE12		SEMA4_GATE12_REG(SEMA4_BASE_PTR)
#define SEMA4_CP0INE		SEMA4_CPINE_REG(SEMA4_BASE_PTR,0)
#define SEMA4_CP1INE		SEMA4_CPINE_REG(SEMA4_BASE_PTR,1)
#define SEMA4_CP0NTF		SEMA4_CPNTF_REG(SEMA4_BASE_PTR,0)
#define SEMA4_CP1NTF		SEMA4_CPNTF_REG(SEMA4_BASE_PTR,1)
#define SEMA4_RSTGT		SEMA4_RSTGT_REG(SEMA4_BASE_PTR)
#define SEMA4_RSTNTF		SEMA4_RSTNTF_REG(SEMA4_BASE_PTR)

/* SEMA4 - Register array accessors */
#define SEMA4_CPINE(index)		SEMA4_CPINE_REG(SEMA4_BASE_PTR,index)
#define SEMA4_CPNTF(index)		SEMA4_CPNTF_REG(SEMA4_BASE_PTR,index)

#define SEMA4_PROCESSOR_SELF	(1)

/*!
 * @}
 */ /* end of group SEMA4_Register_Accessor_Macros */

/*!
 * @}
 */ /* end of group SEMA4_Peripheral */

/* ----------------------------------------------------------------------------
   -- ADC
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Peripheral ADC
 * @{
 */

/** ADC - Peripheral register structure */
typedef struct ADC_MemMap {
  uint32_t HC[2];                                  /**< Control register for hardware triggers, array offset: 0x0, array step: 0x4 */
  uint32_t HS;                                     /**< Status register for HW triggers, offset: 0x8 */
  uint32_t R[2];                                   /**< Data result register for HW triggers, array offset: 0xC, array step: 0x4 */
  uint32_t CFG;                                    /**< Configuration register, offset: 0x14 */
  uint32_t GC;                                     /**< General control register, offset: 0x18 */
  uint32_t GS;                                     /**< General status register, offset: 0x1C */
  uint32_t CV;                                     /**< Compare value register, offset: 0x20 */
  uint32_t OFS;                                    /**< Offset correction value register, offset: 0x24 */
  uint32_t CAL;                                    /**< Calibration value register, offset: 0x28 */
  uint8_t RESERVED_0[4];
  uint32_t PCTL;                                   /**< Pin control register, offset: 0x30 */
} volatile *ADC_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- ADC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Register_Accessor_Macros ADC - Register accessor macros
 * @{
 */


/* ADC - Register accessors */
#define ADC_HC_REG(base,index)                   ((base)->HC[index])
#define ADC_HS_REG(base)                         ((base)->HS)
#define ADC_R_REG(base,index)                    ((base)->R[index])
#define ADC_CFG_REG(base)                        ((base)->CFG)
#define ADC_GC_REG(base)                         ((base)->GC)
#define ADC_GS_REG(base)                         ((base)->GS)
#define ADC_CV_REG(base)                         ((base)->CV)
#define ADC_OFS_REG(base)                        ((base)->OFS)
#define ADC_CAL_REG(base)                        ((base)->CAL)
#define ADC_PCTL_REG(base)                       ((base)->PCTL)

/*!
 * @}
 */ /* end of group ADC_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- ADC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Register_Masks ADC Register Masks
 * @{
 */

/* HC Bit Fields */
#define ADC_HC_ADCH_MASK                         0x1Fu
#define ADC_HC_ADCH_SHIFT                        0
#define ADC_HC_ADCH(x)                           (((uint32_t)(((uint32_t)(x))<<ADC_HC_ADCH_SHIFT))&ADC_HC_ADCH_MASK)
#define ADC_HC_AIEN_MASK                         0x80u
#define ADC_HC_AIEN_SHIFT                        7
/* HS Bit Fields */
#define ADC_HS_COCO0_MASK                        0x1u
#define ADC_HS_COCO0_SHIFT                       0
#define ADC_HS_COCO1_MASK                        0x2u
#define ADC_HS_COCO1_SHIFT                       1
/* R Bit Fields */
#define ADC_R_D_MASK                             0xFFFu
#define ADC_R_D_SHIFT                            0
#define ADC_R_D(x)                               (((uint32_t)(((uint32_t)(x))<<ADC_R_D_SHIFT))&ADC_R_D_MASK)
/* CFG Bit Fields */
#define ADC_CFG_ADICLK_MASK                      0x3u
#define ADC_CFG_ADICLK_SHIFT                     0
#define ADC_CFG_ADICLK(x)                        (((uint32_t)(((uint32_t)(x))<<ADC_CFG_ADICLK_SHIFT))&ADC_CFG_ADICLK_MASK)
#define ADC_CFG_MODE_MASK                        0xCu
#define ADC_CFG_MODE_SHIFT                       2
#define ADC_CFG_MODE(x)                          (((uint32_t)(((uint32_t)(x))<<ADC_CFG_MODE_SHIFT))&ADC_CFG_MODE_MASK)
#define ADC_CFG_ADLSMP_MASK                      0x10u
#define ADC_CFG_ADLSMP_SHIFT                     4
#define ADC_CFG_ADIV_MASK                        0x60u
#define ADC_CFG_ADIV_SHIFT                       5
#define ADC_CFG_ADIV(x)                          (((uint32_t)(((uint32_t)(x))<<ADC_CFG_ADIV_SHIFT))&ADC_CFG_ADIV_MASK)
#define ADC_CFG_ADLPC_MASK                       0x80u
#define ADC_CFG_ADLPC_SHIFT                      7
#define ADC_CFG_ADSTS_MASK                       0x300u
#define ADC_CFG_ADSTS_SHIFT                      8
#define ADC_CFG_ADSTS(x)                         (((uint32_t)(((uint32_t)(x))<<ADC_CFG_ADSTS_SHIFT))&ADC_CFG_ADSTS_MASK)
#define ADC_CFG_ADHSC_MASK                       0x400u
#define ADC_CFG_ADHSC_SHIFT                      10
#define ADC_CFG_REFSEL_MASK                      0x1800u
#define ADC_CFG_REFSEL_SHIFT                     11
#define ADC_CFG_REFSEL(x)                        (((uint32_t)(((uint32_t)(x))<<ADC_CFG_REFSEL_SHIFT))&ADC_CFG_REFSEL_MASK)
#define ADC_CFG_ADTRG_MASK                       0x2000u
#define ADC_CFG_ADTRG_SHIFT                      13
#define ADC_CFG_AVGS_MASK                        0xC000u
#define ADC_CFG_AVGS_SHIFT                       14
#define ADC_CFG_AVGS(x)                          (((uint32_t)(((uint32_t)(x))<<ADC_CFG_AVGS_SHIFT))&ADC_CFG_AVGS_MASK)
#define ADC_CFG_OVWREN_MASK                      0x10000u
#define ADC_CFG_OVWREN_SHIFT                     16
/* GC Bit Fields */
#define ADC_GC_ADACKEN_MASK                      0x1u
#define ADC_GC_ADACKEN_SHIFT                     0
#define ADC_GC_DMAEN_MASK                        0x2u
#define ADC_GC_DMAEN_SHIFT                       1
#define ADC_GC_ACREN_MASK                        0x4u
#define ADC_GC_ACREN_SHIFT                       2
#define ADC_GC_ACFGT_MASK                        0x8u
#define ADC_GC_ACFGT_SHIFT                       3
#define ADC_GC_ACFE_MASK                         0x10u
#define ADC_GC_ACFE_SHIFT                        4
#define ADC_GC_AVGE_MASK                         0x20u
#define ADC_GC_AVGE_SHIFT                        5
#define ADC_GC_ADCO_MASK                         0x40u
#define ADC_GC_ADCO_SHIFT                        6
#define ADC_GC_CAL_MASK                          0x80u
#define ADC_GC_CAL_SHIFT                         7
/* GS Bit Fields */
#define ADC_GS_ADACT_MASK                        0x1u
#define ADC_GS_ADACT_SHIFT                       0
#define ADC_GS_CALF_MASK                         0x2u
#define ADC_GS_CALF_SHIFT                        1
#define ADC_GS_AWKST_MASK                        0x4u
#define ADC_GS_AWKST_SHIFT                       2
/* CV Bit Fields */
#define ADC_CV_CV1_MASK                          0xFFFu
#define ADC_CV_CV1_SHIFT                         0
#define ADC_CV_CV1(x)                            (((uint32_t)(((uint32_t)(x))<<ADC_CV_CV1_SHIFT))&ADC_CV_CV1_MASK)
#define ADC_CV_CV2_MASK                          0xFFF0000u
#define ADC_CV_CV2_SHIFT                         16
#define ADC_CV_CV2(x)                            (((uint32_t)(((uint32_t)(x))<<ADC_CV_CV2_SHIFT))&ADC_CV_CV2_MASK)
/* OFS Bit Fields */
#define ADC_OFS_OFS_MASK                         0xFFFu
#define ADC_OFS_OFS_SHIFT                        0
#define ADC_OFS_OFS(x)                           (((uint32_t)(((uint32_t)(x))<<ADC_OFS_OFS_SHIFT))&ADC_OFS_OFS_MASK)
#define ADC_OFS_SIGN_MASK                        0x1000u
#define ADC_OFS_SIGN_SHIFT                       12
/* CAL Bit Fields */
#define ADC_CAL_CAL_CODE_MASK                    0xFu
#define ADC_CAL_CAL_CODE_SHIFT                   0
#define ADC_CAL_CAL_CODE(x)                      (((uint32_t)(((uint32_t)(x))<<ADC_CAL_CAL_CODE_SHIFT))&ADC_CAL_CAL_CODE_MASK)
/* PCTL Bit Fields */
#define ADC_PCTL_ADPC0_MASK                      0x1u
#define ADC_PCTL_ADPC0_SHIFT                     0
#define ADC_PCTL_ADPC1_MASK                      0x2u
#define ADC_PCTL_ADPC1_SHIFT                     1
#define ADC_PCTL_ADPC2_MASK                      0x4u
#define ADC_PCTL_ADPC2_SHIFT                     2
#define ADC_PCTL_ADPC3_MASK                      0x8u
#define ADC_PCTL_ADPC3_SHIFT                     3
#define ADC_PCTL_ADPC4_MASK                      0x10u
#define ADC_PCTL_ADPC4_SHIFT                     4
#define ADC_PCTL_ADPC5_MASK                      0x20u
#define ADC_PCTL_ADPC5_SHIFT                     5
#define ADC_PCTL_ADPC6_MASK                      0x40u
#define ADC_PCTL_ADPC6_SHIFT                     6
#define ADC_PCTL_ADPC7_MASK                      0x80u
#define ADC_PCTL_ADPC7_SHIFT                     7
#define ADC_PCTL_ADPC8_MASK                      0x100u
#define ADC_PCTL_ADPC8_SHIFT                     8
#define ADC_PCTL_ADPC9_MASK                      0x200u
#define ADC_PCTL_ADPC9_SHIFT                     9
#define ADC_PCTL_ADPC10_MASK                     0x400u
#define ADC_PCTL_ADPC10_SHIFT                    10
#define ADC_PCTL_ADPC11_MASK                     0x800u
#define ADC_PCTL_ADPC11_SHIFT                    11
#define ADC_PCTL_ADPC12_MASK                     0x1000u
#define ADC_PCTL_ADPC12_SHIFT                    12
#define ADC_PCTL_ADPC13_MASK                     0x2000u
#define ADC_PCTL_ADPC13_SHIFT                    13
#define ADC_PCTL_ADPC14_MASK                     0x4000u
#define ADC_PCTL_ADPC14_SHIFT                    14
#define ADC_PCTL_ADPC15_MASK                     0x8000u
#define ADC_PCTL_ADPC15_SHIFT                    15
#define ADC_PCTL_ADPC16_MASK                     0x10000u
#define ADC_PCTL_ADPC16_SHIFT                    16
#define ADC_PCTL_ADPC17_MASK                     0x20000u
#define ADC_PCTL_ADPC17_SHIFT                    17
#define ADC_PCTL_ADPC18_MASK                     0x40000u
#define ADC_PCTL_ADPC18_SHIFT                    18
#define ADC_PCTL_ADPC19_MASK                     0x80000u
#define ADC_PCTL_ADPC19_SHIFT                    19
#define ADC_PCTL_ADPC20_MASK                     0x100000u
#define ADC_PCTL_ADPC20_SHIFT                    20
#define ADC_PCTL_ADPC21_MASK                     0x200000u
#define ADC_PCTL_ADPC21_SHIFT                    21
#define ADC_PCTL_ADPC22_MASK                     0x400000u
#define ADC_PCTL_ADPC22_SHIFT                    22
#define ADC_PCTL_ADPC23_MASK                     0x800000u
#define ADC_PCTL_ADPC23_SHIFT                    23

/*!
 * @}
 */ /* end of group ADC_Register_Masks */


/* ADC - Peripheral instance base addresses */
/** Peripheral ADC0 base pointer */
#define ADC0_BASE_PTR                            ((ADC_MemMapPtr)0x4003B000u)
/** Peripheral ADC1 base pointer */
#define ADC1_BASE_PTR                            ((ADC_MemMapPtr)0x400BB000u)
/** Array initializer of ADC peripheral base pointers */
#define ADC_BASE_PTRS                            { ADC0_BASE_PTR, ADC1_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- ADC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Register_Accessor_Macros ADC - Register accessor macros
 * @{
 */


/* ADC - Register instance definitions */
/* ADC0 */
#define ADC0_HC0                                 ADC_HC_REG(ADC0_BASE_PTR,0)
#define ADC0_HC1                                 ADC_HC_REG(ADC0_BASE_PTR,1)
#define ADC0_HS                                  ADC_HS_REG(ADC0_BASE_PTR)
#define ADC0_R0                                  ADC_R_REG(ADC0_BASE_PTR,0)
#define ADC0_R1                                  ADC_R_REG(ADC0_BASE_PTR,1)
#define ADC0_CFG                                 ADC_CFG_REG(ADC0_BASE_PTR)
#define ADC0_GC                                  ADC_GC_REG(ADC0_BASE_PTR)
#define ADC0_GS                                  ADC_GS_REG(ADC0_BASE_PTR)
#define ADC0_CV                                  ADC_CV_REG(ADC0_BASE_PTR)
#define ADC0_OFS                                 ADC_OFS_REG(ADC0_BASE_PTR)
#define ADC0_CAL                                 ADC_CAL_REG(ADC0_BASE_PTR)
#define ADC0_PCTL                                ADC_PCTL_REG(ADC0_BASE_PTR)
/* ADC1 */
#define ADC1_HC0                                 ADC_HC_REG(ADC1_BASE_PTR,0)
#define ADC1_HC1                                 ADC_HC_REG(ADC1_BASE_PTR,1)
#define ADC1_HS                                  ADC_HS_REG(ADC1_BASE_PTR)
#define ADC1_R0                                  ADC_R_REG(ADC1_BASE_PTR,0)
#define ADC1_R1                                  ADC_R_REG(ADC1_BASE_PTR,1)
#define ADC1_CFG                                 ADC_CFG_REG(ADC1_BASE_PTR)
#define ADC1_GC                                  ADC_GC_REG(ADC1_BASE_PTR)
#define ADC1_GS                                  ADC_GS_REG(ADC1_BASE_PTR)
#define ADC1_CV                                  ADC_CV_REG(ADC1_BASE_PTR)
#define ADC1_OFS                                 ADC_OFS_REG(ADC1_BASE_PTR)
#define ADC1_CAL                                 ADC_CAL_REG(ADC1_BASE_PTR)
#define ADC1_PCTL                                ADC_PCTL_REG(ADC1_BASE_PTR)

/* ADC - Register array accessors */
#define ADC0_HC(index)                           ADC_HC_REG(ADC0_BASE_PTR,index)
#define ADC1_HC(index)                           ADC_HC_REG(ADC1_BASE_PTR,index)
#define ADC0_R(index)                            ADC_R_REG(ADC0_BASE_PTR,index)
#define ADC1_R(index)                            ADC_R_REG(ADC1_BASE_PTR,index)

/*!
 * @}
 */ /* end of group ADC_Register_Accessor_Macros */

/* ----------------------------------------------------------------------------
   -- PDB
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PDB_Peripheral PDB
 * @{
 */

/** PDB - Peripheral register structure */
typedef struct PDB_MemMap {
  uint32_t SC;                                     /**< Status and Control register, offset: 0x0 */
  uint32_t MOD;                                    /**< Modulus register, offset: 0x4 */
  uint32_t CNT;                                    /**< Counter register, offset: 0x8 */
  uint32_t IDLY;                                   /**< Interrupt Delay register, offset: 0xC */
  struct {                                         /* offset: 0x10, array step: 0x28 */
    uint32_t C1;                                   /**< Channel n Control register 1, array offset: 0x10, array step: 0x28 */
    uint32_t S;                                      /**< Channel n Status register, array offset: 0x14, array step: 0x28 */
    uint32_t DLY[2];                                 /**< Channel n Delay 0 register..Channel n Delay 1 register, array offset: 0x18, array step: index*0x28, index2*0x4 */
    uint8_t RESERVED_0[24];
  } CH[2];
  uint8_t RESERVED_0[240];
  struct {                                         /* offset: 0x150, array step: 0x8 */
    uint32_t INTC;                                   /**< DAC Interval Trigger n Control register, array offset: 0x150, array step: 0x8 */
    uint32_t INT;                                    /**< DAC Interval n register, array offset: 0x154, array step: 0x8 */
  } DAC[2];
} volatile *PDB_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- PDB - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PDB_Register_Accessor_Macros PDB - Register accessor macros
 * @{
 */


/* PDB - Register accessors */
#define PDB_SC_REG(base)                         ((base)->SC)
#define PDB_MOD_REG(base)                        ((base)->MOD)
#define PDB_CNT_REG(base)                        ((base)->CNT)
#define PDB_IDLY_REG(base)                       ((base)->IDLY)
#define PDB_C1_REG(base,index)                   ((base)->CH[index].C1)
#define PDB_S_REG(base,index)                    ((base)->CH[index].S)
#define PDB_DLY_REG(base,index,index2)           ((base)->CH[index].DLY[index2])
#define PDB_INTC_REG(base,index)                 ((base)->DAC[index].INTC)
#define PDB_INT_REG(base,index)                  ((base)->DAC[index].INT)

/*!
 * @}
 */ /* end of group PDB_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- PDB Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PDB_Register_Masks PDB Register Masks
 * @{
 */

/* SC Bit Fields */
#define PDB_SC_LDOK_MASK                         0x1u
#define PDB_SC_LDOK_SHIFT                        0
#define PDB_SC_CONT_MASK                         0x2u
#define PDB_SC_CONT_SHIFT                        1
#define PDB_SC_MULT_MASK                         0xCu
#define PDB_SC_MULT_SHIFT                        2
#define PDB_SC_MULT(x)                           (((uint32_t)(((uint32_t)(x))<<PDB_SC_MULT_SHIFT))&PDB_SC_MULT_MASK)
#define PDB_SC_PDBIE_MASK                        0x20u
#define PDB_SC_PDBIE_SHIFT                       5
#define PDB_SC_PDBIF_MASK                        0x40u
#define PDB_SC_PDBIF_SHIFT                       6
#define PDB_SC_PDBEN_MASK                        0x80u
#define PDB_SC_PDBEN_SHIFT                       7
#define PDB_SC_TRGSEL_MASK                       0xF00u
#define PDB_SC_TRGSEL_SHIFT                      8
#define PDB_SC_TRGSEL(x)                         (((uint32_t)(((uint32_t)(x))<<PDB_SC_TRGSEL_SHIFT))&PDB_SC_TRGSEL_MASK)
#define PDB_SC_PRESCALER_MASK                    0x7000u
#define PDB_SC_PRESCALER_SHIFT                   12
#define PDB_SC_PRESCALER(x)                      (((uint32_t)(((uint32_t)(x))<<PDB_SC_PRESCALER_SHIFT))&PDB_SC_PRESCALER_MASK)
#define PDB_SC_DMAEN_MASK                        0x8000u
#define PDB_SC_DMAEN_SHIFT                       15
#define PDB_SC_SWTRIG_MASK                       0x10000u
#define PDB_SC_SWTRIG_SHIFT                      16
#define PDB_SC_PDBEIE_MASK                       0x20000u
#define PDB_SC_PDBEIE_SHIFT                      17
#define PDB_SC_LDMOD_MASK                        0xC0000u
#define PDB_SC_LDMOD_SHIFT                       18
#define PDB_SC_LDMOD(x)                          (((uint32_t)(((uint32_t)(x))<<PDB_SC_LDMOD_SHIFT))&PDB_SC_LDMOD_MASK)
/* MOD Bit Fields */
#define PDB_MOD_MOD_MASK                         0xFFFFu
#define PDB_MOD_MOD_SHIFT                        0
#define PDB_MOD_MOD(x)                           (((uint32_t)(((uint32_t)(x))<<PDB_MOD_MOD_SHIFT))&PDB_MOD_MOD_MASK)
/* CNT Bit Fields */
#define PDB_CNT_CNT_MASK                         0xFFFFu
#define PDB_CNT_CNT_SHIFT                        0
#define PDB_CNT_CNT(x)                           (((uint32_t)(((uint32_t)(x))<<PDB_CNT_CNT_SHIFT))&PDB_CNT_CNT_MASK)
/* IDLY Bit Fields */
#define PDB_IDLY_IDLY_MASK                       0xFFFFu
#define PDB_IDLY_IDLY_SHIFT                      0
#define PDB_IDLY_IDLY(x)                         (((uint32_t)(((uint32_t)(x))<<PDB_IDLY_IDLY_SHIFT))&PDB_IDLY_IDLY_MASK)
/* C1 Bit Fields */
#define PDB_C1_EN_MASK                           0xFFu
#define PDB_C1_EN_SHIFT                          0
#define PDB_C1_EN(x)                             (((uint32_t)(((uint32_t)(x))<<PDB_C1_EN_SHIFT))&PDB_C1_EN_MASK)
#define PDB_C1_TOS_MASK                          0xFF00u
#define PDB_C1_TOS_SHIFT                         8
#define PDB_C1_TOS(x)                            (((uint32_t)(((uint32_t)(x))<<PDB_C1_TOS_SHIFT))&PDB_C1_TOS_MASK)
#define PDB_C1_BB_MASK                           0xFF0000u
#define PDB_C1_BB_SHIFT                          16
#define PDB_C1_BB(x)                             (((uint32_t)(((uint32_t)(x))<<PDB_C1_BB_SHIFT))&PDB_C1_BB_MASK)
/* S Bit Fields */
#define PDB_S_ERR_MASK                           0xFFu
#define PDB_S_ERR_SHIFT                          0
#define PDB_S_ERR(x)                             (((uint32_t)(((uint32_t)(x))<<PDB_S_ERR_SHIFT))&PDB_S_ERR_MASK)
#define PDB_S_CF_MASK                            0xFF0000u
#define PDB_S_CF_SHIFT                           16
#define PDB_S_CF(x)                              (((uint32_t)(((uint32_t)(x))<<PDB_S_CF_SHIFT))&PDB_S_CF_MASK)
/* DLY Bit Fields */
#define PDB_DLY_DLY_MASK                         0xFFFFu
#define PDB_DLY_DLY_SHIFT                        0
#define PDB_DLY_DLY(x)                           (((uint32_t)(((uint32_t)(x))<<PDB_DLY_DLY_SHIFT))&PDB_DLY_DLY_MASK)
/* INTC Bit Fields */
#define PDB_INTC_TOE_MASK                        0x1u
#define PDB_INTC_TOE_SHIFT                       0
#define PDB_INTC_EXT_MASK                        0x2u
#define PDB_INTC_EXT_SHIFT                       1
/* INT Bit Fields */
#define PDB_INT_INT_MASK                         0xFFFFu
#define PDB_INT_INT_SHIFT                        0
#define PDB_INT_INT(x)                           (((uint32_t)(((uint32_t)(x))<<PDB_INT_INT_SHIFT))&PDB_INT_INT_MASK)

/*!
 * @}
 */ /* end of group PDB_Register_Masks */


/* PDB - Peripheral instance base addresses */
/** Peripheral PDB base pointer */
#define PDB_BASE_PTR                             ((PDB_MemMapPtr)0x40036000u)
/** Array initializer of PDB peripheral base pointers */
#define PDB_BASE_PTRS                            { PDB_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- PDB - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PDB_Register_Accessor_Macros PDB - Register accessor macros
 * @{
 */


/* PDB - Register instance definitions */
/* PDB */
#define PDB_SC                                   PDB_SC_REG(PDB_BASE_PTR)
#define PDB_MOD                                  PDB_MOD_REG(PDB_BASE_PTR)
#define PDB_CNT                                  PDB_CNT_REG(PDB_BASE_PTR)
#define PDB_IDLY                                 PDB_IDLY_REG(PDB_BASE_PTR)
#define PDB_CH0C1                                PDB_C1_REG(PDB_BASE_PTR,0)
#define PDB_CH0S                                 PDB_S_REG(PDB_BASE_PTR,0)
#define PDB_CH0DLY0                              PDB_DLY_REG(PDB_BASE_PTR,0,0)
#define PDB_CH0DLY1                              PDB_DLY_REG(PDB_BASE_PTR,0,1)
#define PDB_CH1C1                                PDB_C1_REG(PDB_BASE_PTR,1)
#define PDB_CH1S                                 PDB_S_REG(PDB_BASE_PTR,1)
#define PDB_CH1DLY0                              PDB_DLY_REG(PDB_BASE_PTR,1,0)
#define PDB_CH1DLY1                              PDB_DLY_REG(PDB_BASE_PTR,1,1)
#define PDB_DACINTC0                             PDB_INTC_REG(PDB_BASE_PTR,0)
#define PDB_DACINT0                              PDB_INT_REG(PDB_BASE_PTR,0)
#define PDB_DACINTC1                             PDB_INTC_REG(PDB_BASE_PTR,1)
#define PDB_DACINT1                              PDB_INT_REG(PDB_BASE_PTR,1)

/* PDB - Register array accessors */
#define PDB_C1(index)                            PDB_C1_REG(PDB_BASE_PTR,index)
#define PDB_S(index)                             PDB_S_REG(PDB_BASE_PTR,index)
#define PDB_DLY(index,index2)                    PDB_DLY_REG(PDB_BASE_PTR,index,index2)
#define PDB_INTC(index)                          PDB_INTC_REG(PDB_BASE_PTR,index)
#define PDB_INT(index)                           PDB_INT_REG(PDB_BASE_PTR,index)

/*!
 * @}
 */ /* end of group PDB_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group PDB_Peripheral */


/*!
 * @}
 */ /* end of group ADC_Peripheral */

/* ----------------------------------------------------------------------------
   -- PIT
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PIT_Peripheral PIT
 * @{
 */

/** PIT - Peripheral register structure */
typedef struct PIT_MemMap {
  uint32_t MCR;                                    /**< PIT Module Control Register, offset: 0x0 */
  uint8_t RESERVED_0[220];
  uint32_t LTMR64H;                                /**< PIT Upper Lifetime Timer Register, offset: 0xE0 */
  uint32_t LTMR64L;                                /**< PIT Lower Lifetime Timer Register, offset: 0xE4 */
  uint8_t RESERVED_1[24];
  struct {                                         /* offset: 0x100, array step: 0x10 */
    uint32_t LDVAL;                                  /**< Timer Load Value Register, array offset: 0x100, array step: 0x10 */
    uint32_t CVAL;                                   /**< Current Timer Value Register, array offset: 0x104, array step: 0x10 */
    uint32_t TCTRL;                                  /**< Timer Control Register, array offset: 0x108, array step: 0x10 */
    uint32_t TFLG;                                   /**< Timer Flag Register, array offset: 0x10C, array step: 0x10 */
  } CHANNEL[8];
} volatile *PIT_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- PIT - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PIT_Register_Accessor_Macros PIT - Register accessor macros
 * @{
 */


/* PIT - Register accessors */
#define PIT_MCR_REG(base)                        ((base)->MCR)
#define PIT_LTMR64H_REG(base)                    ((base)->LTMR64H)
#define PIT_LTMR64L_REG(base)                    ((base)->LTMR64L)
#define PIT_LDVAL_REG(base,index)                ((base)->CHANNEL[index].LDVAL)
#define PIT_CVAL_REG(base,index)                 ((base)->CHANNEL[index].CVAL)
#define PIT_TCTRL_REG(base,index)                ((base)->CHANNEL[index].TCTRL)
#define PIT_TFLG_REG(base,index)                 ((base)->CHANNEL[index].TFLG)

/*!
 * @}
 */ /* end of group PIT_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- PIT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PIT_Register_Masks PIT Register Masks
 * @{
 */

/* MCR Bit Fields */
#define PIT_MCR_FRZ_MASK                         0x1u
#define PIT_MCR_FRZ_SHIFT                        0
#define PIT_MCR_MDIS_MASK                        0x2u
#define PIT_MCR_MDIS_SHIFT                       1
/* LTMR64H Bit Fields */
#define PIT_LTMR64H_LTH_MASK                     0xFFFFFFFFu
#define PIT_LTMR64H_LTH_SHIFT                    0
#define PIT_LTMR64H_LTH(x)                       (((uint32_t)(((uint32_t)(x))<<PIT_LTMR64H_LTH_SHIFT))&PIT_LTMR64H_LTH_MASK)
/* LTMR64L Bit Fields */
#define PIT_LTMR64L_LTL_MASK                     0xFFFFFFFFu
#define PIT_LTMR64L_LTL_SHIFT                    0
#define PIT_LTMR64L_LTL(x)                       (((uint32_t)(((uint32_t)(x))<<PIT_LTMR64L_LTL_SHIFT))&PIT_LTMR64L_LTL_MASK)
/* LDVAL Bit Fields */
#define PIT_LDVAL_TSV_MASK                       0xFFFFFFFFu
#define PIT_LDVAL_TSV_SHIFT                      0
#define PIT_LDVAL_TSV(x)                         (((uint32_t)(((uint32_t)(x))<<PIT_LDVAL_TSV_SHIFT))&PIT_LDVAL_TSV_MASK)
/* CVAL Bit Fields */
#define PIT_CVAL_TVL_MASK                        0xFFFFFFFFu
#define PIT_CVAL_TVL_SHIFT                       0
#define PIT_CVAL_TVL(x)                          (((uint32_t)(((uint32_t)(x))<<PIT_CVAL_TVL_SHIFT))&PIT_CVAL_TVL_MASK)
/* TCTRL Bit Fields */
#define PIT_TCTRL_TEN_MASK                       0x1u
#define PIT_TCTRL_TEN_SHIFT                      0
#define PIT_TCTRL_TIE_MASK                       0x2u
#define PIT_TCTRL_TIE_SHIFT                      1
#define PIT_TCTRL_CHN_MASK                       0x4u
#define PIT_TCTRL_CHN_SHIFT                      2
/* TFLG Bit Fields */
#define PIT_TFLG_TIF_MASK                        0x1u
#define PIT_TFLG_TIF_SHIFT                       0

/*!
 * @}
 */ /* end of group PIT_Register_Masks */


/* PIT - Peripheral instance base addresses */
/** Peripheral PIT base pointer */
#define PIT_BASE_PTR                             ((PIT_MemMapPtr)0x40037000u)
/** Array initializer of PIT peripheral base pointers */
#define PIT_BASE_PTRS                            { PIT_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- PIT - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PIT_Register_Accessor_Macros PIT - Register accessor macros
 * @{
 */


/* PIT - Register instance definitions */
/* PIT */
#define PIT_MCR                                  PIT_MCR_REG(PIT_BASE_PTR)
#define PIT_LTMR64H                              PIT_LTMR64H_REG(PIT_BASE_PTR)
#define PIT_LTMR64L                              PIT_LTMR64L_REG(PIT_BASE_PTR)
#define PIT_LDVAL0                               PIT_LDVAL_REG(PIT_BASE_PTR,0)
#define PIT_CVAL0                                PIT_CVAL_REG(PIT_BASE_PTR,0)
#define PIT_TCTRL0                               PIT_TCTRL_REG(PIT_BASE_PTR,0)
#define PIT_TFLG0                                PIT_TFLG_REG(PIT_BASE_PTR,0)
#define PIT_LDVAL1                               PIT_LDVAL_REG(PIT_BASE_PTR,1)
#define PIT_CVAL1                                PIT_CVAL_REG(PIT_BASE_PTR,1)
#define PIT_TCTRL1                               PIT_TCTRL_REG(PIT_BASE_PTR,1)
#define PIT_TFLG1                                PIT_TFLG_REG(PIT_BASE_PTR,1)
#define PIT_LDVAL2                               PIT_LDVAL_REG(PIT_BASE_PTR,2)
#define PIT_CVAL2                                PIT_CVAL_REG(PIT_BASE_PTR,2)
#define PIT_TCTRL2                               PIT_TCTRL_REG(PIT_BASE_PTR,2)
#define PIT_TFLG2                                PIT_TFLG_REG(PIT_BASE_PTR,2)
#define PIT_LDVAL3                               PIT_LDVAL_REG(PIT_BASE_PTR,3)
#define PIT_CVAL3                                PIT_CVAL_REG(PIT_BASE_PTR,3)
#define PIT_TCTRL3                               PIT_TCTRL_REG(PIT_BASE_PTR,3)
#define PIT_TFLG3                                PIT_TFLG_REG(PIT_BASE_PTR,3)
#define PIT_LDVAL4                               PIT_LDVAL_REG(PIT_BASE_PTR,4)
#define PIT_CVAL4                                PIT_CVAL_REG(PIT_BASE_PTR,4)
#define PIT_TCTRL4                               PIT_TCTRL_REG(PIT_BASE_PTR,4)
#define PIT_TFLG4                                PIT_TFLG_REG(PIT_BASE_PTR,4)
#define PIT_LDVAL5                               PIT_LDVAL_REG(PIT_BASE_PTR,5)
#define PIT_CVAL5                                PIT_CVAL_REG(PIT_BASE_PTR,5)
#define PIT_TCTRL5                               PIT_TCTRL_REG(PIT_BASE_PTR,5)
#define PIT_TFLG5                                PIT_TFLG_REG(PIT_BASE_PTR,5)
#define PIT_LDVAL6                               PIT_LDVAL_REG(PIT_BASE_PTR,6)
#define PIT_CVAL6                                PIT_CVAL_REG(PIT_BASE_PTR,6)
#define PIT_TCTRL6                               PIT_TCTRL_REG(PIT_BASE_PTR,6)
#define PIT_TFLG6                                PIT_TFLG_REG(PIT_BASE_PTR,6)
#define PIT_LDVAL7                               PIT_LDVAL_REG(PIT_BASE_PTR,7)
#define PIT_CVAL7                                PIT_CVAL_REG(PIT_BASE_PTR,7)
#define PIT_TCTRL7                               PIT_TCTRL_REG(PIT_BASE_PTR,7)
#define PIT_TFLG7                                PIT_TFLG_REG(PIT_BASE_PTR,7)

/* PIT - Register array accessors */
#define PIT_LDVAL(index)                         PIT_LDVAL_REG(PIT_BASE_PTR,index)
#define PIT_CVAL(index)                          PIT_CVAL_REG(PIT_BASE_PTR,index)
#define PIT_TCTRL(index)                         PIT_TCTRL_REG(PIT_BASE_PTR,index)
#define PIT_TFLG(index)                          PIT_TFLG_REG(PIT_BASE_PTR,index)

/*!
 * @}
 */ /* end of group PIT_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group PIT_Peripheral */



/* ----------------------------------------------------------------------------
   -- CAN
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CAN_Peripheral CAN
 * @{
 */

/** CAN - Peripheral register structure */
typedef struct CAN_MemMap {
  uint32_t MCR;                                    /**< Module Configuration Register, offset: 0x0 */
  uint32_t CTRL1;                                  /**< Control 1 register, offset: 0x4 */
  uint32_t TIMER;                                  /**< Free Running Timer, offset: 0x8 */
  uint8_t RESERVED_0[4];
  uint32_t RXMGMASK;                               /**< Rx Mailboxes Global Mask Register, offset: 0x10 */
  uint32_t RX14MASK;                               /**< Rx 14 Mask register, offset: 0x14 */
  uint32_t RX15MASK;                               /**< Rx 15 Mask register, offset: 0x18 */
  uint32_t ECR;                                    /**< Error Counter, offset: 0x1C */
  uint32_t ESR1;                                   /**< Error and Status 1 register, offset: 0x20 */
  uint32_t IMASK2;                                 /**< Interrupt Masks 2 register, offset: 0x24 */
  uint32_t IMASK1;                                 /**< Interrupt Masks 1 register, offset: 0x28 */
  uint32_t IFLAG2;                                 /**< Interrupt Flags 2 register, offset: 0x2C */
  uint32_t IFLAG1;                                 /**< Interrupt Flags 1 register, offset: 0x30 */
  uint32_t CTRL2;                                  /**< Control 2 register, offset: 0x34 */
  uint32_t ESR2;                                   /**< Error and Status 2 register, offset: 0x38 */
  uint8_t RESERVED_1[8];
  uint32_t CRCR;                                   /**< CRC Register, offset: 0x44 */
  uint32_t RXFGMASK;                               /**< Rx FIFO Global Mask register, offset: 0x48 */
  uint32_t RXFIR;                                  /**< Rx FIFO Information Register, offset: 0x4C */
  uint8_t RESERVED_2[48];
  struct {                                         /* offset: 0x80, array step: 0x10 */
    uint32_t CS;                                     /**< Message Buffer 0 CS Register..Message Buffer 63 CS Register, array offset: 0x80, array step: 0x10 */
    uint32_t ID;                                     /**< Message Buffer 0 ID Register..Message Buffer 63 ID Register, array offset: 0x84, array step: 0x10 */
    uint32_t WORD0;                                  /**< Message Buffer 0 WORD0 Register..Message Buffer 63 WORD0 Register, array offset: 0x88, array step: 0x10 */
    uint32_t WORD1;                                  /**< Message Buffer 0 WORD1 Register..Message Buffer 63 WORD1 Register, array offset: 0x8C, array step: 0x10 */
  } MB[64];
  uint8_t RESERVED_3[1024];
  uint32_t RXIMR[64];                              /**< Rx Individual Mask Registers, array offset: 0x880, array step: 0x4 */
  uint8_t RESERVED_4[352];
  uint32_t MECR;                                   /**< Memory Error Control Register, offset: 0xAE0 */
  uint32_t ERRIAR;                                 /**< Error Injection Address Register, offset: 0xAE4 */
  uint32_t ERRIDPR;                                /**< Error Injection Data Pattern Register, offset: 0xAE8 */
  uint32_t ERRIPPR;                                /**< Error Injection Parity Pattern Register, offset: 0xAEC */
  uint32_t RERRAR;                                 /**< Error Report Address Register, offset: 0xAF0 */
  uint32_t RERRDR;                                 /**< Error Report Data Register, offset: 0xAF4 */
  uint32_t RERRSYNR;                               /**< Error Report Syndrome Register, offset: 0xAF8 */
  uint32_t ERRSR;                                  /**< Error Status Register, offset: 0xAFC */
} volatile *CAN_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- CAN - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CAN_Register_Accessor_Macros CAN - Register accessor macros
 * @{
 */


/* CAN - Register accessors */
#define CAN_MCR_REG(base)                        ((base)->MCR)
#define CAN_CTRL1_REG(base)                      ((base)->CTRL1)
#define CAN_TIMER_REG(base)                      ((base)->TIMER)
#define CAN_RXMGMASK_REG(base)                   ((base)->RXMGMASK)
#define CAN_RX14MASK_REG(base)                   ((base)->RX14MASK)
#define CAN_RX15MASK_REG(base)                   ((base)->RX15MASK)
#define CAN_ECR_REG(base)                        ((base)->ECR)
#define CAN_ESR1_REG(base)                       ((base)->ESR1)
#define CAN_IMASK2_REG(base)                     ((base)->IMASK2)
#define CAN_IMASK1_REG(base)                     ((base)->IMASK1)
#define CAN_IFLAG2_REG(base)                     ((base)->IFLAG2)
#define CAN_IFLAG1_REG(base)                     ((base)->IFLAG1)
#define CAN_CTRL2_REG(base)                      ((base)->CTRL2)
#define CAN_ESR2_REG(base)                       ((base)->ESR2)
#define CAN_CRCR_REG(base)                       ((base)->CRCR)
#define CAN_RXFGMASK_REG(base)                   ((base)->RXFGMASK)
#define CAN_RXFIR_REG(base)                      ((base)->RXFIR)
#define CAN_CS_REG(base,index)                   ((base)->MB[index].CS)
#define CAN_ID_REG(base,index)                   ((base)->MB[index].ID)
#define CAN_WORD0_REG(base,index)                ((base)->MB[index].WORD0)
#define CAN_WORD1_REG(base,index)                ((base)->MB[index].WORD1)
#define CAN_RXIMR_REG(base,index)                ((base)->RXIMR[index])
#define CAN_MECR_REG(base)                       ((base)->MECR)
#define CAN_ERRIAR_REG(base)                     ((base)->ERRIAR)
#define CAN_ERRIDPR_REG(base)                    ((base)->ERRIDPR)
#define CAN_ERRIPPR_REG(base)                    ((base)->ERRIPPR)
#define CAN_RERRAR_REG(base)                     ((base)->RERRAR)
#define CAN_RERRDR_REG(base)                     ((base)->RERRDR)
#define CAN_RERRSYNR_REG(base)                   ((base)->RERRSYNR)
#define CAN_ERRSR_REG(base)                      ((base)->ERRSR)

/*!
 * @}
 */ /* end of group CAN_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- CAN Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CAN_Register_Masks CAN Register Masks
 * @{
 */

/* MCR Bit Fields */
#define CAN_MCR_MAXMB_MASK                       0x7Fu
#define CAN_MCR_MAXMB_SHIFT                      0
#define CAN_MCR_MAXMB(x)                         (((uint32_t)(((uint32_t)(x))<<CAN_MCR_MAXMB_SHIFT))&CAN_MCR_MAXMB_MASK)
#define CAN_MCR_IDAM_MASK                        0x300u
#define CAN_MCR_IDAM_SHIFT                       8
#define CAN_MCR_IDAM(x)                          (((uint32_t)(((uint32_t)(x))<<CAN_MCR_IDAM_SHIFT))&CAN_MCR_IDAM_MASK)
#define CAN_MCR_AEN_MASK                         0x1000u
#define CAN_MCR_AEN_SHIFT                        12
#define CAN_MCR_LPRIOEN_MASK                     0x2000u
#define CAN_MCR_LPRIOEN_SHIFT                    13
#define CAN_MCR_IRMQ_MASK                        0x10000u
#define CAN_MCR_IRMQ_SHIFT                       16
#define CAN_MCR_SRXDIS_MASK                      0x20000u
#define CAN_MCR_SRXDIS_SHIFT                     17
#define CAN_MCR_LPMACK_MASK                      0x100000u
#define CAN_MCR_LPMACK_SHIFT                     20
#define CAN_MCR_WRNEN_MASK                       0x200000u
#define CAN_MCR_WRNEN_SHIFT                      21
#define CAN_MCR_SUPV_MASK                        0x800000u
#define CAN_MCR_SUPV_SHIFT                       23
#define CAN_MCR_FRZACK_MASK                      0x1000000u
#define CAN_MCR_FRZACK_SHIFT                     24
#define CAN_MCR_SOFTRST_MASK                     0x2000000u
#define CAN_MCR_SOFTRST_SHIFT                    25
#define CAN_MCR_NOTRDY_MASK                      0x8000000u
#define CAN_MCR_NOTRDY_SHIFT                     27
#define CAN_MCR_HALT_MASK                        0x10000000u
#define CAN_MCR_HALT_SHIFT                       28
#define CAN_MCR_RFEN_MASK                        0x20000000u
#define CAN_MCR_RFEN_SHIFT                       29
#define CAN_MCR_FRZ_MASK                         0x40000000u
#define CAN_MCR_FRZ_SHIFT                        30
#define CAN_MCR_MDIS_MASK                        0x80000000u
#define CAN_MCR_MDIS_SHIFT                       31
/* CTRL1 Bit Fields */
#define CAN_CTRL1_PROPSEG_MASK                   0x7u
#define CAN_CTRL1_PROPSEG_SHIFT                  0
#define CAN_CTRL1_PROPSEG(x)                     (((uint32_t)(((uint32_t)(x))<<CAN_CTRL1_PROPSEG_SHIFT))&CAN_CTRL1_PROPSEG_MASK)
#define CAN_CTRL1_LOM_MASK                       0x8u
#define CAN_CTRL1_LOM_SHIFT                      3
#define CAN_CTRL1_LBUF_MASK                      0x10u
#define CAN_CTRL1_LBUF_SHIFT                     4
#define CAN_CTRL1_TSYN_MASK                      0x20u
#define CAN_CTRL1_TSYN_SHIFT                     5
#define CAN_CTRL1_BOFFREC_MASK                   0x40u
#define CAN_CTRL1_BOFFREC_SHIFT                  6
#define CAN_CTRL1_SMP_MASK                       0x80u
#define CAN_CTRL1_SMP_SHIFT                      7
#define CAN_CTRL1_RWRNMSK_MASK                   0x400u
#define CAN_CTRL1_RWRNMSK_SHIFT                  10
#define CAN_CTRL1_TWRNMSK_MASK                   0x800u
#define CAN_CTRL1_TWRNMSK_SHIFT                  11
#define CAN_CTRL1_LPB_MASK                       0x1000u
#define CAN_CTRL1_LPB_SHIFT                      12
#define CAN_CTRL1_CLKSRC_MASK                    0x2000u
#define CAN_CTRL1_CLKSRC_SHIFT                   13
#define CAN_CTRL1_ERRMSK_MASK                    0x4000u
#define CAN_CTRL1_ERRMSK_SHIFT                   14
#define CAN_CTRL1_BOFFMSK_MASK                   0x8000u
#define CAN_CTRL1_BOFFMSK_SHIFT                  15
#define CAN_CTRL1_PSEG2_MASK                     0x70000u
#define CAN_CTRL1_PSEG2_SHIFT                    16
#define CAN_CTRL1_PSEG2(x)                       (((uint32_t)(((uint32_t)(x))<<CAN_CTRL1_PSEG2_SHIFT))&CAN_CTRL1_PSEG2_MASK)
#define CAN_CTRL1_PSEG1_MASK                     0x380000u
#define CAN_CTRL1_PSEG1_SHIFT                    19
#define CAN_CTRL1_PSEG1(x)                       (((uint32_t)(((uint32_t)(x))<<CAN_CTRL1_PSEG1_SHIFT))&CAN_CTRL1_PSEG1_MASK)
#define CAN_CTRL1_RJW_MASK                       0xC00000u
#define CAN_CTRL1_RJW_SHIFT                      22
#define CAN_CTRL1_RJW(x)                         (((uint32_t)(((uint32_t)(x))<<CAN_CTRL1_RJW_SHIFT))&CAN_CTRL1_RJW_MASK)
#define CAN_CTRL1_PRESDIV_MASK                   0xFF000000u
#define CAN_CTRL1_PRESDIV_SHIFT                  24
#define CAN_CTRL1_PRESDIV(x)                     (((uint32_t)(((uint32_t)(x))<<CAN_CTRL1_PRESDIV_SHIFT))&CAN_CTRL1_PRESDIV_MASK)
/* TIMER Bit Fields */
#define CAN_TIMER_TIMER_MASK                     0xFFFFu
#define CAN_TIMER_TIMER_SHIFT                    0
#define CAN_TIMER_TIMER(x)                       (((uint32_t)(((uint32_t)(x))<<CAN_TIMER_TIMER_SHIFT))&CAN_TIMER_TIMER_MASK)
/* RXMGMASK Bit Fields */
#define CAN_RXMGMASK_MG_MASK                     0xFFFFFFFFu
#define CAN_RXMGMASK_MG_SHIFT                    0
#define CAN_RXMGMASK_MG(x)                       (((uint32_t)(((uint32_t)(x))<<CAN_RXMGMASK_MG_SHIFT))&CAN_RXMGMASK_MG_MASK)
/* RX14MASK Bit Fields */
#define CAN_RX14MASK_RX14M_MASK                  0xFFFFFFFFu
#define CAN_RX14MASK_RX14M_SHIFT                 0
#define CAN_RX14MASK_RX14M(x)                    (((uint32_t)(((uint32_t)(x))<<CAN_RX14MASK_RX14M_SHIFT))&CAN_RX14MASK_RX14M_MASK)
/* RX15MASK Bit Fields */
#define CAN_RX15MASK_RX15M_MASK                  0xFFFFFFFFu
#define CAN_RX15MASK_RX15M_SHIFT                 0
#define CAN_RX15MASK_RX15M(x)                    (((uint32_t)(((uint32_t)(x))<<CAN_RX15MASK_RX15M_SHIFT))&CAN_RX15MASK_RX15M_MASK)
/* ECR Bit Fields */
#define CAN_ECR_TXERRCNT_MASK                    0xFFu
#define CAN_ECR_TXERRCNT_SHIFT                   0
#define CAN_ECR_TXERRCNT(x)                      (((uint32_t)(((uint32_t)(x))<<CAN_ECR_TXERRCNT_SHIFT))&CAN_ECR_TXERRCNT_MASK)
#define CAN_ECR_RXERRCNT_MASK                    0xFF00u
#define CAN_ECR_RXERRCNT_SHIFT                   8
#define CAN_ECR_RXERRCNT(x)                      (((uint32_t)(((uint32_t)(x))<<CAN_ECR_RXERRCNT_SHIFT))&CAN_ECR_RXERRCNT_MASK)
/* ESR1 Bit Fields */
#define CAN_ESR1_ERRINT_MASK                     0x2u
#define CAN_ESR1_ERRINT_SHIFT                    1
#define CAN_ESR1_BOFFINT_MASK                    0x4u
#define CAN_ESR1_BOFFINT_SHIFT                   2
#define CAN_ESR1_RX_MASK                         0x8u
#define CAN_ESR1_RX_SHIFT                        3
#define CAN_ESR1_FLTCONF_MASK                    0x30u
#define CAN_ESR1_FLTCONF_SHIFT                   4
#define CAN_ESR1_FLTCONF(x)                      (((uint32_t)(((uint32_t)(x))<<CAN_ESR1_FLTCONF_SHIFT))&CAN_ESR1_FLTCONF_MASK)
#define CAN_ESR1_TX_MASK                         0x40u
#define CAN_ESR1_TX_SHIFT                        6
#define CAN_ESR1_IDLE_MASK                       0x80u
#define CAN_ESR1_IDLE_SHIFT                      7
#define CAN_ESR1_RXWRN_MASK                      0x100u
#define CAN_ESR1_RXWRN_SHIFT                     8
#define CAN_ESR1_TXWRN_MASK                      0x200u
#define CAN_ESR1_TXWRN_SHIFT                     9
#define CAN_ESR1_STFERR_MASK                     0x400u
#define CAN_ESR1_STFERR_SHIFT                    10
#define CAN_ESR1_FRMERR_MASK                     0x800u
#define CAN_ESR1_FRMERR_SHIFT                    11
#define CAN_ESR1_CRCERR_MASK                     0x1000u
#define CAN_ESR1_CRCERR_SHIFT                    12
#define CAN_ESR1_ACKERR_MASK                     0x2000u
#define CAN_ESR1_ACKERR_SHIFT                    13
#define CAN_ESR1_BIT0ERR_MASK                    0x4000u
#define CAN_ESR1_BIT0ERR_SHIFT                   14
#define CAN_ESR1_BIT1ERR_MASK                    0x8000u
#define CAN_ESR1_BIT1ERR_SHIFT                   15
#define CAN_ESR1_RWRNINT_MASK                    0x10000u
#define CAN_ESR1_RWRNINT_SHIFT                   16
#define CAN_ESR1_TWRNINT_MASK                    0x20000u
#define CAN_ESR1_TWRNINT_SHIFT                   17
#define CAN_ESR1_SYNCH_MASK                      0x40000u
#define CAN_ESR1_SYNCH_SHIFT                     18
/* IMASK2 Bit Fields */
#define CAN_IMASK2_BUFHM_MASK                    0xFFFFFFFFu
#define CAN_IMASK2_BUFHM_SHIFT                   0
#define CAN_IMASK2_BUFHM(x)                      (((uint32_t)(((uint32_t)(x))<<CAN_IMASK2_BUFHM_SHIFT))&CAN_IMASK2_BUFHM_MASK)
/* IMASK1 Bit Fields */
#define CAN_IMASK1_BUFLM_MASK                    0xFFFFFFFFu
#define CAN_IMASK1_BUFLM_SHIFT                   0
#define CAN_IMASK1_BUFLM(x)                      (((uint32_t)(((uint32_t)(x))<<CAN_IMASK1_BUFLM_SHIFT))&CAN_IMASK1_BUFLM_MASK)
/* IFLAG2 Bit Fields */
#define CAN_IFLAG2_BUFHI_MASK                    0xFFFFFFFFu
#define CAN_IFLAG2_BUFHI_SHIFT                   0
#define CAN_IFLAG2_BUFHI(x)                      (((uint32_t)(((uint32_t)(x))<<CAN_IFLAG2_BUFHI_SHIFT))&CAN_IFLAG2_BUFHI_MASK)
/* IFLAG1 Bit Fields */
#define CAN_IFLAG1_BUF0I_MASK                    0x1u
#define CAN_IFLAG1_BUF0I_SHIFT                   0
#define CAN_IFLAG1_BUF4TO1I_MASK                 0x1Eu
#define CAN_IFLAG1_BUF4TO1I_SHIFT                1
#define CAN_IFLAG1_BUF4TO1I(x)                   (((uint32_t)(((uint32_t)(x))<<CAN_IFLAG1_BUF4TO1I_SHIFT))&CAN_IFLAG1_BUF4TO1I_MASK)
#define CAN_IFLAG1_BUF5I_MASK                    0x20u
#define CAN_IFLAG1_BUF5I_SHIFT                   5
#define CAN_IFLAG1_BUF6I_MASK                    0x40u
#define CAN_IFLAG1_BUF6I_SHIFT                   6
#define CAN_IFLAG1_BUF7I_MASK                    0x80u
#define CAN_IFLAG1_BUF7I_SHIFT                   7
#define CAN_IFLAG1_BUF31TO8I_MASK                0xFFFFFF00u
#define CAN_IFLAG1_BUF31TO8I_SHIFT               8
#define CAN_IFLAG1_BUF31TO8I(x)                  (((uint32_t)(((uint32_t)(x))<<CAN_IFLAG1_BUF31TO8I_SHIFT))&CAN_IFLAG1_BUF31TO8I_MASK)
/* CTRL2 Bit Fields */
#define CAN_CTRL2_EACEN_MASK                     0x10000u
#define CAN_CTRL2_EACEN_SHIFT                    16
#define CAN_CTRL2_RRS_MASK                       0x20000u
#define CAN_CTRL2_RRS_SHIFT                      17
#define CAN_CTRL2_MRP_MASK                       0x40000u
#define CAN_CTRL2_MRP_SHIFT                      18
#define CAN_CTRL2_TASD_MASK                      0xF80000u
#define CAN_CTRL2_TASD_SHIFT                     19
#define CAN_CTRL2_TASD(x)                        (((uint32_t)(((uint32_t)(x))<<CAN_CTRL2_TASD_SHIFT))&CAN_CTRL2_TASD_MASK)
#define CAN_CTRL2_RFFN_MASK                      0xF000000u
#define CAN_CTRL2_RFFN_SHIFT                     24
#define CAN_CTRL2_RFFN(x)                        (((uint32_t)(((uint32_t)(x))<<CAN_CTRL2_RFFN_SHIFT))&CAN_CTRL2_RFFN_MASK)
#define CAN_CTRL2_WRMFRZ_MASK                    0x10000000u
#define CAN_CTRL2_WRMFRZ_SHIFT                   28
#define CAN_CTRL2_ECRWRE_MASK                    0x20000000u
#define CAN_CTRL2_ECRWRE_SHIFT                   29
/* ESR2 Bit Fields */
#define CAN_ESR2_IMB_MASK                        0x2000u
#define CAN_ESR2_IMB_SHIFT                       13
#define CAN_ESR2_VPS_MASK                        0x4000u
#define CAN_ESR2_VPS_SHIFT                       14
#define CAN_ESR2_LPTM_MASK                       0x7F0000u
#define CAN_ESR2_LPTM_SHIFT                      16
#define CAN_ESR2_LPTM(x)                         (((uint32_t)(((uint32_t)(x))<<CAN_ESR2_LPTM_SHIFT))&CAN_ESR2_LPTM_MASK)
/* CRCR Bit Fields */
#define CAN_CRCR_TXCRC_MASK                      0x7FFFu
#define CAN_CRCR_TXCRC_SHIFT                     0
#define CAN_CRCR_TXCRC(x)                        (((uint32_t)(((uint32_t)(x))<<CAN_CRCR_TXCRC_SHIFT))&CAN_CRCR_TXCRC_MASK)
#define CAN_CRCR_MBCRC_MASK                      0x7F0000u
#define CAN_CRCR_MBCRC_SHIFT                     16
#define CAN_CRCR_MBCRC(x)                        (((uint32_t)(((uint32_t)(x))<<CAN_CRCR_MBCRC_SHIFT))&CAN_CRCR_MBCRC_MASK)
/* RXFGMASK Bit Fields */
#define CAN_RXFGMASK_FGM_MASK                    0xFFFFFFFFu
#define CAN_RXFGMASK_FGM_SHIFT                   0
#define CAN_RXFGMASK_FGM(x)                      (((uint32_t)(((uint32_t)(x))<<CAN_RXFGMASK_FGM_SHIFT))&CAN_RXFGMASK_FGM_MASK)
/* RXFIR Bit Fields */
#define CAN_RXFIR_IDHIT_MASK                     0x1FFu
#define CAN_RXFIR_IDHIT_SHIFT                    0
#define CAN_RXFIR_IDHIT(x)                       (((uint32_t)(((uint32_t)(x))<<CAN_RXFIR_IDHIT_SHIFT))&CAN_RXFIR_IDHIT_MASK)
/* CS Bit Fields */
#define CAN_CS_TIME_STAMP_MASK                   0xFFFFu
#define CAN_CS_TIME_STAMP_SHIFT                  0
#define CAN_CS_TIME_STAMP(x)                     (((uint32_t)(((uint32_t)(x))<<CAN_CS_TIME_STAMP_SHIFT))&CAN_CS_TIME_STAMP_MASK)
#define CAN_CS_DLC_MASK                          0xF0000u
#define CAN_CS_DLC_SHIFT                         16
#define CAN_CS_DLC(x)                            (((uint32_t)(((uint32_t)(x))<<CAN_CS_DLC_SHIFT))&CAN_CS_DLC_MASK)
#define CAN_CS_RTR_MASK                          0x100000u
#define CAN_CS_RTR_SHIFT                         20
#define CAN_CS_IDE_MASK                          0x200000u
#define CAN_CS_IDE_SHIFT                         21
#define CAN_CS_SRR_MASK                          0x400000u
#define CAN_CS_SRR_SHIFT                         22
#define CAN_CS_CODE_MASK                         0xF000000u
#define CAN_CS_CODE_SHIFT                        24
#define CAN_CS_CODE(x)                           (((uint32_t)(((uint32_t)(x))<<CAN_CS_CODE_SHIFT))&CAN_CS_CODE_MASK)
/* ID Bit Fields */
#define CAN_ID_EXT_MASK                          0x3FFFFu
#define CAN_ID_EXT_SHIFT                         0
#define CAN_ID_EXT(x)                            (((uint32_t)(((uint32_t)(x))<<CAN_ID_EXT_SHIFT))&CAN_ID_EXT_MASK)
#define CAN_ID_STD_MASK                          0x1FFC0000u
#define CAN_ID_STD_SHIFT                         18
#define CAN_ID_STD(x)                            (((uint32_t)(((uint32_t)(x))<<CAN_ID_STD_SHIFT))&CAN_ID_STD_MASK)
#define CAN_ID_PRIO_MASK                         0xE0000000u
#define CAN_ID_PRIO_SHIFT                        29
#define CAN_ID_PRIO(x)                           (((uint32_t)(((uint32_t)(x))<<CAN_ID_PRIO_SHIFT))&CAN_ID_PRIO_MASK)
/* WORD0 Bit Fields */
#define CAN_WORD0_DATA_BYTE_3_MASK               0xFFu
#define CAN_WORD0_DATA_BYTE_3_SHIFT              0
#define CAN_WORD0_DATA_BYTE_3(x)                 (((uint32_t)(((uint32_t)(x))<<CAN_WORD0_DATA_BYTE_3_SHIFT))&CAN_WORD0_DATA_BYTE_3_MASK)
#define CAN_WORD0_DATA_BYTE_2_MASK               0xFF00u
#define CAN_WORD0_DATA_BYTE_2_SHIFT              8
#define CAN_WORD0_DATA_BYTE_2(x)                 (((uint32_t)(((uint32_t)(x))<<CAN_WORD0_DATA_BYTE_2_SHIFT))&CAN_WORD0_DATA_BYTE_2_MASK)
#define CAN_WORD0_DATA_BYTE_1_MASK               0xFF0000u
#define CAN_WORD0_DATA_BYTE_1_SHIFT              16
#define CAN_WORD0_DATA_BYTE_1(x)                 (((uint32_t)(((uint32_t)(x))<<CAN_WORD0_DATA_BYTE_1_SHIFT))&CAN_WORD0_DATA_BYTE_1_MASK)
#define CAN_WORD0_DATA_BYTE_0_MASK               0xFF000000u
#define CAN_WORD0_DATA_BYTE_0_SHIFT              24
#define CAN_WORD0_DATA_BYTE_0(x)                 (((uint32_t)(((uint32_t)(x))<<CAN_WORD0_DATA_BYTE_0_SHIFT))&CAN_WORD0_DATA_BYTE_0_MASK)
/* WORD1 Bit Fields */
#define CAN_WORD1_DATA_BYTE_7_MASK               0xFFu
#define CAN_WORD1_DATA_BYTE_7_SHIFT              0
#define CAN_WORD1_DATA_BYTE_7(x)                 (((uint32_t)(((uint32_t)(x))<<CAN_WORD1_DATA_BYTE_7_SHIFT))&CAN_WORD1_DATA_BYTE_7_MASK)
#define CAN_WORD1_DATA_BYTE_6_MASK               0xFF00u
#define CAN_WORD1_DATA_BYTE_6_SHIFT              8
#define CAN_WORD1_DATA_BYTE_6(x)                 (((uint32_t)(((uint32_t)(x))<<CAN_WORD1_DATA_BYTE_6_SHIFT))&CAN_WORD1_DATA_BYTE_6_MASK)
#define CAN_WORD1_DATA_BYTE_5_MASK               0xFF0000u
#define CAN_WORD1_DATA_BYTE_5_SHIFT              16
#define CAN_WORD1_DATA_BYTE_5(x)                 (((uint32_t)(((uint32_t)(x))<<CAN_WORD1_DATA_BYTE_5_SHIFT))&CAN_WORD1_DATA_BYTE_5_MASK)
#define CAN_WORD1_DATA_BYTE_4_MASK               0xFF000000u
#define CAN_WORD1_DATA_BYTE_4_SHIFT              24
#define CAN_WORD1_DATA_BYTE_4(x)                 (((uint32_t)(((uint32_t)(x))<<CAN_WORD1_DATA_BYTE_4_SHIFT))&CAN_WORD1_DATA_BYTE_4_MASK)
/* RXIMR Bit Fields */
#define CAN_RXIMR_MI_MASK                        0xFFFFFFFFu
#define CAN_RXIMR_MI_SHIFT                       0
#define CAN_RXIMR_MI(x)                          (((uint32_t)(((uint32_t)(x))<<CAN_RXIMR_MI_SHIFT))&CAN_RXIMR_MI_MASK)
/* MECR Bit Fields */
#define CAN_MECR_NCEFAFRZ_MASK                   0x80u
#define CAN_MECR_NCEFAFRZ_SHIFT                  7
#define CAN_MECR_ECCDIS_MASK                     0x100u
#define CAN_MECR_ECCDIS_SHIFT                    8
#define CAN_MECR_RERRDIS_MASK                    0x200u
#define CAN_MECR_RERRDIS_SHIFT                   9
#define CAN_MECR_EXTERRIE_MASK                   0x2000u
#define CAN_MECR_EXTERRIE_SHIFT                  13
#define CAN_MECR_FAERRIE_MASK                    0x4000u
#define CAN_MECR_FAERRIE_SHIFT                   14
#define CAN_MECR_HAERRIE_MASK                    0x8000u
#define CAN_MECR_HAERRIE_SHIFT                   15
#define CAN_MECR_CEI_MSK_MASK                    0x10000u
#define CAN_MECR_CEI_MSK_SHIFT                   16
#define CAN_MECR_FANCEI_MSK_MASK                 0x40000u
#define CAN_MECR_FANCEI_MSK_SHIFT                18
#define CAN_MECR_HANCEI_MSK_MASK                 0x80000u
#define CAN_MECR_HANCEI_MSK_SHIFT                19
#define CAN_MECR_ECRWRDIS_MASK                   0x80000000u
#define CAN_MECR_ECRWRDIS_SHIFT                  31
/* ERRIAR Bit Fields */
#define CAN_ERRIAR_INJADDR_MASK                  0x3FFFu
#define CAN_ERRIAR_INJADDR_SHIFT                 0
#define CAN_ERRIAR_INJADDR(x)                    (((uint32_t)(((uint32_t)(x))<<CAN_ERRIAR_INJADDR_SHIFT))&CAN_ERRIAR_INJADDR_MASK)
/* ERRIDPR Bit Fields */
#define CAN_ERRIDPR_DFLIP_MASK                   0xFFFFFFFFu
#define CAN_ERRIDPR_DFLIP_SHIFT                  0
#define CAN_ERRIDPR_DFLIP(x)                     (((uint32_t)(((uint32_t)(x))<<CAN_ERRIDPR_DFLIP_SHIFT))&CAN_ERRIDPR_DFLIP_MASK)
/* ERRIPPR Bit Fields */
#define CAN_ERRIPPR_PFLIP0_MASK                  0x1Fu
#define CAN_ERRIPPR_PFLIP0_SHIFT                 0
#define CAN_ERRIPPR_PFLIP0(x)                    (((uint32_t)(((uint32_t)(x))<<CAN_ERRIPPR_PFLIP0_SHIFT))&CAN_ERRIPPR_PFLIP0_MASK)
#define CAN_ERRIPPR_PFLIP1_MASK                  0x1F00u
#define CAN_ERRIPPR_PFLIP1_SHIFT                 8
#define CAN_ERRIPPR_PFLIP1(x)                    (((uint32_t)(((uint32_t)(x))<<CAN_ERRIPPR_PFLIP1_SHIFT))&CAN_ERRIPPR_PFLIP1_MASK)
#define CAN_ERRIPPR_PFLIP2_MASK                  0x1F0000u
#define CAN_ERRIPPR_PFLIP2_SHIFT                 16
#define CAN_ERRIPPR_PFLIP2(x)                    (((uint32_t)(((uint32_t)(x))<<CAN_ERRIPPR_PFLIP2_SHIFT))&CAN_ERRIPPR_PFLIP2_MASK)
#define CAN_ERRIPPR_PFLIP3_MASK                  0x1F000000u
#define CAN_ERRIPPR_PFLIP3_SHIFT                 24
#define CAN_ERRIPPR_PFLIP3(x)                    (((uint32_t)(((uint32_t)(x))<<CAN_ERRIPPR_PFLIP3_SHIFT))&CAN_ERRIPPR_PFLIP3_MASK)
/* RERRAR Bit Fields */
#define CAN_RERRAR_ERRADDR_MASK                  0x3FFFu
#define CAN_RERRAR_ERRADDR_SHIFT                 0
#define CAN_RERRAR_ERRADDR(x)                    (((uint32_t)(((uint32_t)(x))<<CAN_RERRAR_ERRADDR_SHIFT))&CAN_RERRAR_ERRADDR_MASK)
#define CAN_RERRAR_SAID_MASK                     0x70000u
#define CAN_RERRAR_SAID_SHIFT                    16
#define CAN_RERRAR_SAID(x)                       (((uint32_t)(((uint32_t)(x))<<CAN_RERRAR_SAID_SHIFT))&CAN_RERRAR_SAID_MASK)
#define CAN_RERRAR_NCE_MASK                      0x1000000u
#define CAN_RERRAR_NCE_SHIFT                     24
/* RERRDR Bit Fields */
#define CAN_RERRDR_RDATA_MASK                    0xFFFFFFFFu
#define CAN_RERRDR_RDATA_SHIFT                   0
#define CAN_RERRDR_RDATA(x)                      (((uint32_t)(((uint32_t)(x))<<CAN_RERRDR_RDATA_SHIFT))&CAN_RERRDR_RDATA_MASK)
/* RERRSYNR Bit Fields */
#define CAN_RERRSYNR_SYND0_MASK                  0x1Fu
#define CAN_RERRSYNR_SYND0_SHIFT                 0
#define CAN_RERRSYNR_SYND0(x)                    (((uint32_t)(((uint32_t)(x))<<CAN_RERRSYNR_SYND0_SHIFT))&CAN_RERRSYNR_SYND0_MASK)
#define CAN_RERRSYNR_BE0_MASK                    0x80u
#define CAN_RERRSYNR_BE0_SHIFT                   7
#define CAN_RERRSYNR_SYND1_MASK                  0x1F00u
#define CAN_RERRSYNR_SYND1_SHIFT                 8
#define CAN_RERRSYNR_SYND1(x)                    (((uint32_t)(((uint32_t)(x))<<CAN_RERRSYNR_SYND1_SHIFT))&CAN_RERRSYNR_SYND1_MASK)
#define CAN_RERRSYNR_BE1_MASK                    0x8000u
#define CAN_RERRSYNR_BE1_SHIFT                   15
#define CAN_RERRSYNR_SYND2_MASK                  0x1F0000u
#define CAN_RERRSYNR_SYND2_SHIFT                 16
#define CAN_RERRSYNR_SYND2(x)                    (((uint32_t)(((uint32_t)(x))<<CAN_RERRSYNR_SYND2_SHIFT))&CAN_RERRSYNR_SYND2_MASK)
#define CAN_RERRSYNR_BE2_MASK                    0x800000u
#define CAN_RERRSYNR_BE2_SHIFT                   23
#define CAN_RERRSYNR_SYND3_MASK                  0x1F000000u
#define CAN_RERRSYNR_SYND3_SHIFT                 24
#define CAN_RERRSYNR_SYND3(x)                    (((uint32_t)(((uint32_t)(x))<<CAN_RERRSYNR_SYND3_SHIFT))&CAN_RERRSYNR_SYND3_MASK)
#define CAN_RERRSYNR_BE3_MASK                    0x80000000u
#define CAN_RERRSYNR_BE3_SHIFT                   31
/* ERRSR Bit Fields */
#define CAN_ERRSR_CEIOF_MASK                     0x1u
#define CAN_ERRSR_CEIOF_SHIFT                    0
#define CAN_ERRSR_FANCEIOF_MASK                  0x4u
#define CAN_ERRSR_FANCEIOF_SHIFT                 2
#define CAN_ERRSR_HANCEIOF_MASK                  0x8u
#define CAN_ERRSR_HANCEIOF_SHIFT                 3
#define CAN_ERRSR_CEIF_MASK                      0x10000u
#define CAN_ERRSR_CEIF_SHIFT                     16
#define CAN_ERRSR_FANCEIF_MASK                   0x40000u
#define CAN_ERRSR_FANCEIF_SHIFT                  18
#define CAN_ERRSR_HANCEIF_MASK                   0x80000u
#define CAN_ERRSR_HANCEIF_SHIFT                  19

/*!
 * @}
 */ /* end of group CAN_Register_Masks */


/* CAN - Peripheral instance base addresses */
/** Peripheral CAN0 base pointer */
#define CAN0_BASE_PTR                            ((CAN_MemMapPtr)0x40020000u)
/** Peripheral CAN1 base pointer */
#define CAN1_BASE_PTR                            ((CAN_MemMapPtr)0x400D4000u)
/** Array initializer of CAN peripheral base pointers */
#define CAN_BASE_PTRS                            { CAN0_BASE_PTR, CAN1_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- CAN - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CAN_Register_Accessor_Macros CAN - Register accessor macros
 * @{
 */


/* CAN - Register instance definitions */
/* CAN0 */
#define CAN0_MCR                                 CAN_MCR_REG(CAN0_BASE_PTR)
#define CAN0_CTRL1                               CAN_CTRL1_REG(CAN0_BASE_PTR)
#define CAN0_TIMER                               CAN_TIMER_REG(CAN0_BASE_PTR)
#define CAN0_RXMGMASK                            CAN_RXMGMASK_REG(CAN0_BASE_PTR)
#define CAN0_RX14MASK                            CAN_RX14MASK_REG(CAN0_BASE_PTR)
#define CAN0_RX15MASK                            CAN_RX15MASK_REG(CAN0_BASE_PTR)
#define CAN0_ECR                                 CAN_ECR_REG(CAN0_BASE_PTR)
#define CAN0_ESR1                                CAN_ESR1_REG(CAN0_BASE_PTR)
#define CAN0_IMASK2                              CAN_IMASK2_REG(CAN0_BASE_PTR)
#define CAN0_IMASK1                              CAN_IMASK1_REG(CAN0_BASE_PTR)
#define CAN0_IFLAG2                              CAN_IFLAG2_REG(CAN0_BASE_PTR)
#define CAN0_IFLAG1                              CAN_IFLAG1_REG(CAN0_BASE_PTR)
#define CAN0_CTRL2                               CAN_CTRL2_REG(CAN0_BASE_PTR)
#define CAN0_ESR2                                CAN_ESR2_REG(CAN0_BASE_PTR)
#define CAN0_CRCR                                CAN_CRCR_REG(CAN0_BASE_PTR)
#define CAN0_RXFGMASK                            CAN_RXFGMASK_REG(CAN0_BASE_PTR)
#define CAN0_RXFIR                               CAN_RXFIR_REG(CAN0_BASE_PTR)
#define CAN0_CS0                                 CAN_CS_REG(CAN0_BASE_PTR,0)
#define CAN0_ID0                                 CAN_ID_REG(CAN0_BASE_PTR,0)
#define CAN0_WORD00                              CAN_WORD0_REG(CAN0_BASE_PTR,0)
#define CAN0_WORD10                              CAN_WORD1_REG(CAN0_BASE_PTR,0)
#define CAN0_CS1                                 CAN_CS_REG(CAN0_BASE_PTR,1)
#define CAN0_ID1                                 CAN_ID_REG(CAN0_BASE_PTR,1)
#define CAN0_WORD01                              CAN_WORD0_REG(CAN0_BASE_PTR,1)
#define CAN0_WORD11                              CAN_WORD1_REG(CAN0_BASE_PTR,1)
#define CAN0_CS2                                 CAN_CS_REG(CAN0_BASE_PTR,2)
#define CAN0_ID2                                 CAN_ID_REG(CAN0_BASE_PTR,2)
#define CAN0_WORD02                              CAN_WORD0_REG(CAN0_BASE_PTR,2)
#define CAN0_WORD12                              CAN_WORD1_REG(CAN0_BASE_PTR,2)
#define CAN0_CS3                                 CAN_CS_REG(CAN0_BASE_PTR,3)
#define CAN0_ID3                                 CAN_ID_REG(CAN0_BASE_PTR,3)
#define CAN0_WORD03                              CAN_WORD0_REG(CAN0_BASE_PTR,3)
#define CAN0_WORD13                              CAN_WORD1_REG(CAN0_BASE_PTR,3)
#define CAN0_CS4                                 CAN_CS_REG(CAN0_BASE_PTR,4)
#define CAN0_ID4                                 CAN_ID_REG(CAN0_BASE_PTR,4)
#define CAN0_WORD04                              CAN_WORD0_REG(CAN0_BASE_PTR,4)
#define CAN0_WORD14                              CAN_WORD1_REG(CAN0_BASE_PTR,4)
#define CAN0_CS5                                 CAN_CS_REG(CAN0_BASE_PTR,5)
#define CAN0_ID5                                 CAN_ID_REG(CAN0_BASE_PTR,5)
#define CAN0_WORD05                              CAN_WORD0_REG(CAN0_BASE_PTR,5)
#define CAN0_WORD15                              CAN_WORD1_REG(CAN0_BASE_PTR,5)
#define CAN0_CS6                                 CAN_CS_REG(CAN0_BASE_PTR,6)
#define CAN0_ID6                                 CAN_ID_REG(CAN0_BASE_PTR,6)
#define CAN0_WORD06                              CAN_WORD0_REG(CAN0_BASE_PTR,6)
#define CAN0_WORD16                              CAN_WORD1_REG(CAN0_BASE_PTR,6)
#define CAN0_CS7                                 CAN_CS_REG(CAN0_BASE_PTR,7)
#define CAN0_ID7                                 CAN_ID_REG(CAN0_BASE_PTR,7)
#define CAN0_WORD07                              CAN_WORD0_REG(CAN0_BASE_PTR,7)
#define CAN0_WORD17                              CAN_WORD1_REG(CAN0_BASE_PTR,7)
#define CAN0_CS8                                 CAN_CS_REG(CAN0_BASE_PTR,8)
#define CAN0_ID8                                 CAN_ID_REG(CAN0_BASE_PTR,8)
#define CAN0_WORD08                              CAN_WORD0_REG(CAN0_BASE_PTR,8)
#define CAN0_WORD18                              CAN_WORD1_REG(CAN0_BASE_PTR,8)
#define CAN0_CS9                                 CAN_CS_REG(CAN0_BASE_PTR,9)
#define CAN0_ID9                                 CAN_ID_REG(CAN0_BASE_PTR,9)
#define CAN0_WORD09                              CAN_WORD0_REG(CAN0_BASE_PTR,9)
#define CAN0_WORD19                              CAN_WORD1_REG(CAN0_BASE_PTR,9)
#define CAN0_CS10                                CAN_CS_REG(CAN0_BASE_PTR,10)
#define CAN0_ID10                                CAN_ID_REG(CAN0_BASE_PTR,10)
#define CAN0_WORD010                             CAN_WORD0_REG(CAN0_BASE_PTR,10)
#define CAN0_WORD110                             CAN_WORD1_REG(CAN0_BASE_PTR,10)
#define CAN0_CS11                                CAN_CS_REG(CAN0_BASE_PTR,11)
#define CAN0_ID11                                CAN_ID_REG(CAN0_BASE_PTR,11)
#define CAN0_WORD011                             CAN_WORD0_REG(CAN0_BASE_PTR,11)
#define CAN0_WORD111                             CAN_WORD1_REG(CAN0_BASE_PTR,11)
#define CAN0_CS12                                CAN_CS_REG(CAN0_BASE_PTR,12)
#define CAN0_ID12                                CAN_ID_REG(CAN0_BASE_PTR,12)
#define CAN0_WORD012                             CAN_WORD0_REG(CAN0_BASE_PTR,12)
#define CAN0_WORD112                             CAN_WORD1_REG(CAN0_BASE_PTR,12)
#define CAN0_CS13                                CAN_CS_REG(CAN0_BASE_PTR,13)
#define CAN0_ID13                                CAN_ID_REG(CAN0_BASE_PTR,13)
#define CAN0_WORD013                             CAN_WORD0_REG(CAN0_BASE_PTR,13)
#define CAN0_WORD113                             CAN_WORD1_REG(CAN0_BASE_PTR,13)
#define CAN0_CS14                                CAN_CS_REG(CAN0_BASE_PTR,14)
#define CAN0_ID14                                CAN_ID_REG(CAN0_BASE_PTR,14)
#define CAN0_WORD014                             CAN_WORD0_REG(CAN0_BASE_PTR,14)
#define CAN0_WORD114                             CAN_WORD1_REG(CAN0_BASE_PTR,14)
#define CAN0_CS15                                CAN_CS_REG(CAN0_BASE_PTR,15)
#define CAN0_ID15                                CAN_ID_REG(CAN0_BASE_PTR,15)
#define CAN0_WORD015                             CAN_WORD0_REG(CAN0_BASE_PTR,15)
#define CAN0_WORD115                             CAN_WORD1_REG(CAN0_BASE_PTR,15)
#define CAN0_CS16                                CAN_CS_REG(CAN0_BASE_PTR,16)
#define CAN0_ID16                                CAN_ID_REG(CAN0_BASE_PTR,16)
#define CAN0_WORD016                             CAN_WORD0_REG(CAN0_BASE_PTR,16)
#define CAN0_WORD116                             CAN_WORD1_REG(CAN0_BASE_PTR,16)
#define CAN0_CS17                                CAN_CS_REG(CAN0_BASE_PTR,17)
#define CAN0_ID17                                CAN_ID_REG(CAN0_BASE_PTR,17)
#define CAN0_WORD017                             CAN_WORD0_REG(CAN0_BASE_PTR,17)
#define CAN0_WORD117                             CAN_WORD1_REG(CAN0_BASE_PTR,17)
#define CAN0_CS18                                CAN_CS_REG(CAN0_BASE_PTR,18)
#define CAN0_ID18                                CAN_ID_REG(CAN0_BASE_PTR,18)
#define CAN0_WORD018                             CAN_WORD0_REG(CAN0_BASE_PTR,18)
#define CAN0_WORD118                             CAN_WORD1_REG(CAN0_BASE_PTR,18)
#define CAN0_CS19                                CAN_CS_REG(CAN0_BASE_PTR,19)
#define CAN0_ID19                                CAN_ID_REG(CAN0_BASE_PTR,19)
#define CAN0_WORD019                             CAN_WORD0_REG(CAN0_BASE_PTR,19)
#define CAN0_WORD119                             CAN_WORD1_REG(CAN0_BASE_PTR,19)
#define CAN0_CS20                                CAN_CS_REG(CAN0_BASE_PTR,20)
#define CAN0_ID20                                CAN_ID_REG(CAN0_BASE_PTR,20)
#define CAN0_WORD020                             CAN_WORD0_REG(CAN0_BASE_PTR,20)
#define CAN0_WORD120                             CAN_WORD1_REG(CAN0_BASE_PTR,20)
#define CAN0_CS21                                CAN_CS_REG(CAN0_BASE_PTR,21)
#define CAN0_ID21                                CAN_ID_REG(CAN0_BASE_PTR,21)
#define CAN0_WORD021                             CAN_WORD0_REG(CAN0_BASE_PTR,21)
#define CAN0_WORD121                             CAN_WORD1_REG(CAN0_BASE_PTR,21)
#define CAN0_CS22                                CAN_CS_REG(CAN0_BASE_PTR,22)
#define CAN0_ID22                                CAN_ID_REG(CAN0_BASE_PTR,22)
#define CAN0_WORD022                             CAN_WORD0_REG(CAN0_BASE_PTR,22)
#define CAN0_WORD122                             CAN_WORD1_REG(CAN0_BASE_PTR,22)
#define CAN0_CS23                                CAN_CS_REG(CAN0_BASE_PTR,23)
#define CAN0_ID23                                CAN_ID_REG(CAN0_BASE_PTR,23)
#define CAN0_WORD023                             CAN_WORD0_REG(CAN0_BASE_PTR,23)
#define CAN0_WORD123                             CAN_WORD1_REG(CAN0_BASE_PTR,23)
#define CAN0_CS24                                CAN_CS_REG(CAN0_BASE_PTR,24)
#define CAN0_ID24                                CAN_ID_REG(CAN0_BASE_PTR,24)
#define CAN0_WORD024                             CAN_WORD0_REG(CAN0_BASE_PTR,24)
#define CAN0_WORD124                             CAN_WORD1_REG(CAN0_BASE_PTR,24)
#define CAN0_CS25                                CAN_CS_REG(CAN0_BASE_PTR,25)
#define CAN0_ID25                                CAN_ID_REG(CAN0_BASE_PTR,25)
#define CAN0_WORD025                             CAN_WORD0_REG(CAN0_BASE_PTR,25)
#define CAN0_WORD125                             CAN_WORD1_REG(CAN0_BASE_PTR,25)
#define CAN0_CS26                                CAN_CS_REG(CAN0_BASE_PTR,26)
#define CAN0_ID26                                CAN_ID_REG(CAN0_BASE_PTR,26)
#define CAN0_WORD026                             CAN_WORD0_REG(CAN0_BASE_PTR,26)
#define CAN0_WORD126                             CAN_WORD1_REG(CAN0_BASE_PTR,26)
#define CAN0_CS27                                CAN_CS_REG(CAN0_BASE_PTR,27)
#define CAN0_ID27                                CAN_ID_REG(CAN0_BASE_PTR,27)
#define CAN0_WORD027                             CAN_WORD0_REG(CAN0_BASE_PTR,27)
#define CAN0_WORD127                             CAN_WORD1_REG(CAN0_BASE_PTR,27)
#define CAN0_CS28                                CAN_CS_REG(CAN0_BASE_PTR,28)
#define CAN0_ID28                                CAN_ID_REG(CAN0_BASE_PTR,28)
#define CAN0_WORD028                             CAN_WORD0_REG(CAN0_BASE_PTR,28)
#define CAN0_WORD128                             CAN_WORD1_REG(CAN0_BASE_PTR,28)
#define CAN0_CS29                                CAN_CS_REG(CAN0_BASE_PTR,29)
#define CAN0_ID29                                CAN_ID_REG(CAN0_BASE_PTR,29)
#define CAN0_WORD029                             CAN_WORD0_REG(CAN0_BASE_PTR,29)
#define CAN0_WORD129                             CAN_WORD1_REG(CAN0_BASE_PTR,29)
#define CAN0_CS30                                CAN_CS_REG(CAN0_BASE_PTR,30)
#define CAN0_ID30                                CAN_ID_REG(CAN0_BASE_PTR,30)
#define CAN0_WORD030                             CAN_WORD0_REG(CAN0_BASE_PTR,30)
#define CAN0_WORD130                             CAN_WORD1_REG(CAN0_BASE_PTR,30)
#define CAN0_CS31                                CAN_CS_REG(CAN0_BASE_PTR,31)
#define CAN0_ID31                                CAN_ID_REG(CAN0_BASE_PTR,31)
#define CAN0_WORD031                             CAN_WORD0_REG(CAN0_BASE_PTR,31)
#define CAN0_WORD131                             CAN_WORD1_REG(CAN0_BASE_PTR,31)
#define CAN0_CS32                                CAN_CS_REG(CAN0_BASE_PTR,32)
#define CAN0_ID32                                CAN_ID_REG(CAN0_BASE_PTR,32)
#define CAN0_WORD032                             CAN_WORD0_REG(CAN0_BASE_PTR,32)
#define CAN0_WORD132                             CAN_WORD1_REG(CAN0_BASE_PTR,32)
#define CAN0_CS33                                CAN_CS_REG(CAN0_BASE_PTR,33)
#define CAN0_ID33                                CAN_ID_REG(CAN0_BASE_PTR,33)
#define CAN0_WORD033                             CAN_WORD0_REG(CAN0_BASE_PTR,33)
#define CAN0_WORD133                             CAN_WORD1_REG(CAN0_BASE_PTR,33)
#define CAN0_CS34                                CAN_CS_REG(CAN0_BASE_PTR,34)
#define CAN0_ID34                                CAN_ID_REG(CAN0_BASE_PTR,34)
#define CAN0_WORD034                             CAN_WORD0_REG(CAN0_BASE_PTR,34)
#define CAN0_WORD134                             CAN_WORD1_REG(CAN0_BASE_PTR,34)
#define CAN0_CS35                                CAN_CS_REG(CAN0_BASE_PTR,35)
#define CAN0_ID35                                CAN_ID_REG(CAN0_BASE_PTR,35)
#define CAN0_WORD035                             CAN_WORD0_REG(CAN0_BASE_PTR,35)
#define CAN0_WORD135                             CAN_WORD1_REG(CAN0_BASE_PTR,35)
#define CAN0_CS36                                CAN_CS_REG(CAN0_BASE_PTR,36)
#define CAN0_ID36                                CAN_ID_REG(CAN0_BASE_PTR,36)
#define CAN0_WORD036                             CAN_WORD0_REG(CAN0_BASE_PTR,36)
#define CAN0_WORD136                             CAN_WORD1_REG(CAN0_BASE_PTR,36)
#define CAN0_CS37                                CAN_CS_REG(CAN0_BASE_PTR,37)
#define CAN0_ID37                                CAN_ID_REG(CAN0_BASE_PTR,37)
#define CAN0_WORD037                             CAN_WORD0_REG(CAN0_BASE_PTR,37)
#define CAN0_WORD137                             CAN_WORD1_REG(CAN0_BASE_PTR,37)
#define CAN0_CS38                                CAN_CS_REG(CAN0_BASE_PTR,38)
#define CAN0_ID38                                CAN_ID_REG(CAN0_BASE_PTR,38)
#define CAN0_WORD038                             CAN_WORD0_REG(CAN0_BASE_PTR,38)
#define CAN0_WORD138                             CAN_WORD1_REG(CAN0_BASE_PTR,38)
#define CAN0_CS39                                CAN_CS_REG(CAN0_BASE_PTR,39)
#define CAN0_ID39                                CAN_ID_REG(CAN0_BASE_PTR,39)
#define CAN0_WORD039                             CAN_WORD0_REG(CAN0_BASE_PTR,39)
#define CAN0_WORD139                             CAN_WORD1_REG(CAN0_BASE_PTR,39)
#define CAN0_CS40                                CAN_CS_REG(CAN0_BASE_PTR,40)
#define CAN0_ID40                                CAN_ID_REG(CAN0_BASE_PTR,40)
#define CAN0_WORD040                             CAN_WORD0_REG(CAN0_BASE_PTR,40)
#define CAN0_WORD140                             CAN_WORD1_REG(CAN0_BASE_PTR,40)
#define CAN0_CS41                                CAN_CS_REG(CAN0_BASE_PTR,41)
#define CAN0_ID41                                CAN_ID_REG(CAN0_BASE_PTR,41)
#define CAN0_WORD041                             CAN_WORD0_REG(CAN0_BASE_PTR,41)
#define CAN0_WORD141                             CAN_WORD1_REG(CAN0_BASE_PTR,41)
#define CAN0_CS42                                CAN_CS_REG(CAN0_BASE_PTR,42)
#define CAN0_ID42                                CAN_ID_REG(CAN0_BASE_PTR,42)
#define CAN0_WORD042                             CAN_WORD0_REG(CAN0_BASE_PTR,42)
#define CAN0_WORD142                             CAN_WORD1_REG(CAN0_BASE_PTR,42)
#define CAN0_CS43                                CAN_CS_REG(CAN0_BASE_PTR,43)
#define CAN0_ID43                                CAN_ID_REG(CAN0_BASE_PTR,43)
#define CAN0_WORD043                             CAN_WORD0_REG(CAN0_BASE_PTR,43)
#define CAN0_WORD143                             CAN_WORD1_REG(CAN0_BASE_PTR,43)
#define CAN0_CS44                                CAN_CS_REG(CAN0_BASE_PTR,44)
#define CAN0_ID44                                CAN_ID_REG(CAN0_BASE_PTR,44)
#define CAN0_WORD044                             CAN_WORD0_REG(CAN0_BASE_PTR,44)
#define CAN0_WORD144                             CAN_WORD1_REG(CAN0_BASE_PTR,44)
#define CAN0_CS45                                CAN_CS_REG(CAN0_BASE_PTR,45)
#define CAN0_ID45                                CAN_ID_REG(CAN0_BASE_PTR,45)
#define CAN0_WORD045                             CAN_WORD0_REG(CAN0_BASE_PTR,45)
#define CAN0_WORD145                             CAN_WORD1_REG(CAN0_BASE_PTR,45)
#define CAN0_CS46                                CAN_CS_REG(CAN0_BASE_PTR,46)
#define CAN0_ID46                                CAN_ID_REG(CAN0_BASE_PTR,46)
#define CAN0_WORD046                             CAN_WORD0_REG(CAN0_BASE_PTR,46)
#define CAN0_WORD146                             CAN_WORD1_REG(CAN0_BASE_PTR,46)
#define CAN0_CS47                                CAN_CS_REG(CAN0_BASE_PTR,47)
#define CAN0_ID47                                CAN_ID_REG(CAN0_BASE_PTR,47)
#define CAN0_WORD047                             CAN_WORD0_REG(CAN0_BASE_PTR,47)
#define CAN0_WORD147                             CAN_WORD1_REG(CAN0_BASE_PTR,47)
#define CAN0_CS48                                CAN_CS_REG(CAN0_BASE_PTR,48)
#define CAN0_ID48                                CAN_ID_REG(CAN0_BASE_PTR,48)
#define CAN0_WORD048                             CAN_WORD0_REG(CAN0_BASE_PTR,48)
#define CAN0_WORD148                             CAN_WORD1_REG(CAN0_BASE_PTR,48)
#define CAN0_CS49                                CAN_CS_REG(CAN0_BASE_PTR,49)
#define CAN0_ID49                                CAN_ID_REG(CAN0_BASE_PTR,49)
#define CAN0_WORD049                             CAN_WORD0_REG(CAN0_BASE_PTR,49)
#define CAN0_WORD149                             CAN_WORD1_REG(CAN0_BASE_PTR,49)
#define CAN0_CS50                                CAN_CS_REG(CAN0_BASE_PTR,50)
#define CAN0_ID50                                CAN_ID_REG(CAN0_BASE_PTR,50)
#define CAN0_WORD050                             CAN_WORD0_REG(CAN0_BASE_PTR,50)
#define CAN0_WORD150                             CAN_WORD1_REG(CAN0_BASE_PTR,50)
#define CAN0_CS51                                CAN_CS_REG(CAN0_BASE_PTR,51)
#define CAN0_ID51                                CAN_ID_REG(CAN0_BASE_PTR,51)
#define CAN0_WORD051                             CAN_WORD0_REG(CAN0_BASE_PTR,51)
#define CAN0_WORD151                             CAN_WORD1_REG(CAN0_BASE_PTR,51)
#define CAN0_CS52                                CAN_CS_REG(CAN0_BASE_PTR,52)
#define CAN0_ID52                                CAN_ID_REG(CAN0_BASE_PTR,52)
#define CAN0_WORD052                             CAN_WORD0_REG(CAN0_BASE_PTR,52)
#define CAN0_WORD152                             CAN_WORD1_REG(CAN0_BASE_PTR,52)
#define CAN0_CS53                                CAN_CS_REG(CAN0_BASE_PTR,53)
#define CAN0_ID53                                CAN_ID_REG(CAN0_BASE_PTR,53)
#define CAN0_WORD053                             CAN_WORD0_REG(CAN0_BASE_PTR,53)
#define CAN0_WORD153                             CAN_WORD1_REG(CAN0_BASE_PTR,53)
#define CAN0_CS54                                CAN_CS_REG(CAN0_BASE_PTR,54)
#define CAN0_ID54                                CAN_ID_REG(CAN0_BASE_PTR,54)
#define CAN0_WORD054                             CAN_WORD0_REG(CAN0_BASE_PTR,54)
#define CAN0_WORD154                             CAN_WORD1_REG(CAN0_BASE_PTR,54)
#define CAN0_CS55                                CAN_CS_REG(CAN0_BASE_PTR,55)
#define CAN0_ID55                                CAN_ID_REG(CAN0_BASE_PTR,55)
#define CAN0_WORD055                             CAN_WORD0_REG(CAN0_BASE_PTR,55)
#define CAN0_WORD155                             CAN_WORD1_REG(CAN0_BASE_PTR,55)
#define CAN0_CS56                                CAN_CS_REG(CAN0_BASE_PTR,56)
#define CAN0_ID56                                CAN_ID_REG(CAN0_BASE_PTR,56)
#define CAN0_WORD056                             CAN_WORD0_REG(CAN0_BASE_PTR,56)
#define CAN0_WORD156                             CAN_WORD1_REG(CAN0_BASE_PTR,56)
#define CAN0_CS57                                CAN_CS_REG(CAN0_BASE_PTR,57)
#define CAN0_ID57                                CAN_ID_REG(CAN0_BASE_PTR,57)
#define CAN0_WORD057                             CAN_WORD0_REG(CAN0_BASE_PTR,57)
#define CAN0_WORD157                             CAN_WORD1_REG(CAN0_BASE_PTR,57)
#define CAN0_CS58                                CAN_CS_REG(CAN0_BASE_PTR,58)
#define CAN0_ID58                                CAN_ID_REG(CAN0_BASE_PTR,58)
#define CAN0_WORD058                             CAN_WORD0_REG(CAN0_BASE_PTR,58)
#define CAN0_WORD158                             CAN_WORD1_REG(CAN0_BASE_PTR,58)
#define CAN0_CS59                                CAN_CS_REG(CAN0_BASE_PTR,59)
#define CAN0_ID59                                CAN_ID_REG(CAN0_BASE_PTR,59)
#define CAN0_WORD059                             CAN_WORD0_REG(CAN0_BASE_PTR,59)
#define CAN0_WORD159                             CAN_WORD1_REG(CAN0_BASE_PTR,59)
#define CAN0_CS60                                CAN_CS_REG(CAN0_BASE_PTR,60)
#define CAN0_ID60                                CAN_ID_REG(CAN0_BASE_PTR,60)
#define CAN0_WORD060                             CAN_WORD0_REG(CAN0_BASE_PTR,60)
#define CAN0_WORD160                             CAN_WORD1_REG(CAN0_BASE_PTR,60)
#define CAN0_CS61                                CAN_CS_REG(CAN0_BASE_PTR,61)
#define CAN0_ID61                                CAN_ID_REG(CAN0_BASE_PTR,61)
#define CAN0_WORD061                             CAN_WORD0_REG(CAN0_BASE_PTR,61)
#define CAN0_WORD161                             CAN_WORD1_REG(CAN0_BASE_PTR,61)
#define CAN0_CS62                                CAN_CS_REG(CAN0_BASE_PTR,62)
#define CAN0_ID62                                CAN_ID_REG(CAN0_BASE_PTR,62)
#define CAN0_WORD062                             CAN_WORD0_REG(CAN0_BASE_PTR,62)
#define CAN0_WORD162                             CAN_WORD1_REG(CAN0_BASE_PTR,62)
#define CAN0_CS63                                CAN_CS_REG(CAN0_BASE_PTR,63)
#define CAN0_ID63                                CAN_ID_REG(CAN0_BASE_PTR,63)
#define CAN0_WORD063                             CAN_WORD0_REG(CAN0_BASE_PTR,63)
#define CAN0_WORD163                             CAN_WORD1_REG(CAN0_BASE_PTR,63)
#define CAN0_RXIMR0                              CAN_RXIMR_REG(CAN0_BASE_PTR,0)
#define CAN0_RXIMR1                              CAN_RXIMR_REG(CAN0_BASE_PTR,1)
#define CAN0_RXIMR2                              CAN_RXIMR_REG(CAN0_BASE_PTR,2)
#define CAN0_RXIMR3                              CAN_RXIMR_REG(CAN0_BASE_PTR,3)
#define CAN0_RXIMR4                              CAN_RXIMR_REG(CAN0_BASE_PTR,4)
#define CAN0_RXIMR5                              CAN_RXIMR_REG(CAN0_BASE_PTR,5)
#define CAN0_RXIMR6                              CAN_RXIMR_REG(CAN0_BASE_PTR,6)
#define CAN0_RXIMR7                              CAN_RXIMR_REG(CAN0_BASE_PTR,7)
#define CAN0_RXIMR8                              CAN_RXIMR_REG(CAN0_BASE_PTR,8)
#define CAN0_RXIMR9                              CAN_RXIMR_REG(CAN0_BASE_PTR,9)
#define CAN0_RXIMR10                             CAN_RXIMR_REG(CAN0_BASE_PTR,10)
#define CAN0_RXIMR11                             CAN_RXIMR_REG(CAN0_BASE_PTR,11)
#define CAN0_RXIMR12                             CAN_RXIMR_REG(CAN0_BASE_PTR,12)
#define CAN0_RXIMR13                             CAN_RXIMR_REG(CAN0_BASE_PTR,13)
#define CAN0_RXIMR14                             CAN_RXIMR_REG(CAN0_BASE_PTR,14)
#define CAN0_RXIMR15                             CAN_RXIMR_REG(CAN0_BASE_PTR,15)
#define CAN0_RXIMR16                             CAN_RXIMR_REG(CAN0_BASE_PTR,16)
#define CAN0_RXIMR17                             CAN_RXIMR_REG(CAN0_BASE_PTR,17)
#define CAN0_RXIMR18                             CAN_RXIMR_REG(CAN0_BASE_PTR,18)
#define CAN0_RXIMR19                             CAN_RXIMR_REG(CAN0_BASE_PTR,19)
#define CAN0_RXIMR20                             CAN_RXIMR_REG(CAN0_BASE_PTR,20)
#define CAN0_RXIMR21                             CAN_RXIMR_REG(CAN0_BASE_PTR,21)
#define CAN0_RXIMR22                             CAN_RXIMR_REG(CAN0_BASE_PTR,22)
#define CAN0_RXIMR23                             CAN_RXIMR_REG(CAN0_BASE_PTR,23)
#define CAN0_RXIMR24                             CAN_RXIMR_REG(CAN0_BASE_PTR,24)
#define CAN0_RXIMR25                             CAN_RXIMR_REG(CAN0_BASE_PTR,25)
#define CAN0_RXIMR26                             CAN_RXIMR_REG(CAN0_BASE_PTR,26)
#define CAN0_RXIMR27                             CAN_RXIMR_REG(CAN0_BASE_PTR,27)
#define CAN0_RXIMR28                             CAN_RXIMR_REG(CAN0_BASE_PTR,28)
#define CAN0_RXIMR29                             CAN_RXIMR_REG(CAN0_BASE_PTR,29)
#define CAN0_RXIMR30                             CAN_RXIMR_REG(CAN0_BASE_PTR,30)
#define CAN0_RXIMR31                             CAN_RXIMR_REG(CAN0_BASE_PTR,31)
#define CAN0_RXIMR32                             CAN_RXIMR_REG(CAN0_BASE_PTR,32)
#define CAN0_RXIMR33                             CAN_RXIMR_REG(CAN0_BASE_PTR,33)
#define CAN0_RXIMR34                             CAN_RXIMR_REG(CAN0_BASE_PTR,34)
#define CAN0_RXIMR35                             CAN_RXIMR_REG(CAN0_BASE_PTR,35)
#define CAN0_RXIMR36                             CAN_RXIMR_REG(CAN0_BASE_PTR,36)
#define CAN0_RXIMR37                             CAN_RXIMR_REG(CAN0_BASE_PTR,37)
#define CAN0_RXIMR38                             CAN_RXIMR_REG(CAN0_BASE_PTR,38)
#define CAN0_RXIMR39                             CAN_RXIMR_REG(CAN0_BASE_PTR,39)
#define CAN0_RXIMR40                             CAN_RXIMR_REG(CAN0_BASE_PTR,40)
#define CAN0_RXIMR41                             CAN_RXIMR_REG(CAN0_BASE_PTR,41)
#define CAN0_RXIMR42                             CAN_RXIMR_REG(CAN0_BASE_PTR,42)
#define CAN0_RXIMR43                             CAN_RXIMR_REG(CAN0_BASE_PTR,43)
#define CAN0_RXIMR44                             CAN_RXIMR_REG(CAN0_BASE_PTR,44)
#define CAN0_RXIMR45                             CAN_RXIMR_REG(CAN0_BASE_PTR,45)
#define CAN0_RXIMR46                             CAN_RXIMR_REG(CAN0_BASE_PTR,46)
#define CAN0_RXIMR47                             CAN_RXIMR_REG(CAN0_BASE_PTR,47)
#define CAN0_RXIMR48                             CAN_RXIMR_REG(CAN0_BASE_PTR,48)
#define CAN0_RXIMR49                             CAN_RXIMR_REG(CAN0_BASE_PTR,49)
#define CAN0_RXIMR50                             CAN_RXIMR_REG(CAN0_BASE_PTR,50)
#define CAN0_RXIMR51                             CAN_RXIMR_REG(CAN0_BASE_PTR,51)
#define CAN0_RXIMR52                             CAN_RXIMR_REG(CAN0_BASE_PTR,52)
#define CAN0_RXIMR53                             CAN_RXIMR_REG(CAN0_BASE_PTR,53)
#define CAN0_RXIMR54                             CAN_RXIMR_REG(CAN0_BASE_PTR,54)
#define CAN0_RXIMR55                             CAN_RXIMR_REG(CAN0_BASE_PTR,55)
#define CAN0_RXIMR56                             CAN_RXIMR_REG(CAN0_BASE_PTR,56)
#define CAN0_RXIMR57                             CAN_RXIMR_REG(CAN0_BASE_PTR,57)
#define CAN0_RXIMR58                             CAN_RXIMR_REG(CAN0_BASE_PTR,58)
#define CAN0_RXIMR59                             CAN_RXIMR_REG(CAN0_BASE_PTR,59)
#define CAN0_RXIMR60                             CAN_RXIMR_REG(CAN0_BASE_PTR,60)
#define CAN0_RXIMR61                             CAN_RXIMR_REG(CAN0_BASE_PTR,61)
#define CAN0_RXIMR62                             CAN_RXIMR_REG(CAN0_BASE_PTR,62)
#define CAN0_RXIMR63                             CAN_RXIMR_REG(CAN0_BASE_PTR,63)
#define CAN0_MECR                                CAN_MECR_REG(CAN0_BASE_PTR)
#define CAN0_ERRIAR                              CAN_ERRIAR_REG(CAN0_BASE_PTR)
#define CAN0_ERRIDPR                             CAN_ERRIDPR_REG(CAN0_BASE_PTR)
#define CAN0_ERRIPPR                             CAN_ERRIPPR_REG(CAN0_BASE_PTR)
#define CAN0_RERRAR                              CAN_RERRAR_REG(CAN0_BASE_PTR)
#define CAN0_RERRDR                              CAN_RERRDR_REG(CAN0_BASE_PTR)
#define CAN0_RERRSYNR                            CAN_RERRSYNR_REG(CAN0_BASE_PTR)
#define CAN0_ERRSR                               CAN_ERRSR_REG(CAN0_BASE_PTR)
/* CAN1 */
#define CAN1_MCR                                 CAN_MCR_REG(CAN1_BASE_PTR)
#define CAN1_CTRL1                               CAN_CTRL1_REG(CAN1_BASE_PTR)
#define CAN1_TIMER                               CAN_TIMER_REG(CAN1_BASE_PTR)
#define CAN1_RXMGMASK                            CAN_RXMGMASK_REG(CAN1_BASE_PTR)
#define CAN1_RX14MASK                            CAN_RX14MASK_REG(CAN1_BASE_PTR)
#define CAN1_RX15MASK                            CAN_RX15MASK_REG(CAN1_BASE_PTR)
#define CAN1_ECR                                 CAN_ECR_REG(CAN1_BASE_PTR)
#define CAN1_ESR1                                CAN_ESR1_REG(CAN1_BASE_PTR)
#define CAN1_IMASK2                              CAN_IMASK2_REG(CAN1_BASE_PTR)
#define CAN1_IMASK1                              CAN_IMASK1_REG(CAN1_BASE_PTR)
#define CAN1_IFLAG2                              CAN_IFLAG2_REG(CAN1_BASE_PTR)
#define CAN1_IFLAG1                              CAN_IFLAG1_REG(CAN1_BASE_PTR)
#define CAN1_CTRL2                               CAN_CTRL2_REG(CAN1_BASE_PTR)
#define CAN1_ESR2                                CAN_ESR2_REG(CAN1_BASE_PTR)
#define CAN1_CRCR                                CAN_CRCR_REG(CAN1_BASE_PTR)
#define CAN1_RXFGMASK                            CAN_RXFGMASK_REG(CAN1_BASE_PTR)
#define CAN1_RXFIR                               CAN_RXFIR_REG(CAN1_BASE_PTR)
#define CAN1_CS0                                 CAN_CS_REG(CAN1_BASE_PTR,0)
#define CAN1_ID0                                 CAN_ID_REG(CAN1_BASE_PTR,0)
#define CAN1_WORD00                              CAN_WORD0_REG(CAN1_BASE_PTR,0)
#define CAN1_WORD10                              CAN_WORD1_REG(CAN1_BASE_PTR,0)
#define CAN1_CS1                                 CAN_CS_REG(CAN1_BASE_PTR,1)
#define CAN1_ID1                                 CAN_ID_REG(CAN1_BASE_PTR,1)
#define CAN1_WORD01                              CAN_WORD0_REG(CAN1_BASE_PTR,1)
#define CAN1_WORD11                              CAN_WORD1_REG(CAN1_BASE_PTR,1)
#define CAN1_CS2                                 CAN_CS_REG(CAN1_BASE_PTR,2)
#define CAN1_ID2                                 CAN_ID_REG(CAN1_BASE_PTR,2)
#define CAN1_WORD02                              CAN_WORD0_REG(CAN1_BASE_PTR,2)
#define CAN1_WORD12                              CAN_WORD1_REG(CAN1_BASE_PTR,2)
#define CAN1_CS3                                 CAN_CS_REG(CAN1_BASE_PTR,3)
#define CAN1_ID3                                 CAN_ID_REG(CAN1_BASE_PTR,3)
#define CAN1_WORD03                              CAN_WORD0_REG(CAN1_BASE_PTR,3)
#define CAN1_WORD13                              CAN_WORD1_REG(CAN1_BASE_PTR,3)
#define CAN1_CS4                                 CAN_CS_REG(CAN1_BASE_PTR,4)
#define CAN1_ID4                                 CAN_ID_REG(CAN1_BASE_PTR,4)
#define CAN1_WORD04                              CAN_WORD0_REG(CAN1_BASE_PTR,4)
#define CAN1_WORD14                              CAN_WORD1_REG(CAN1_BASE_PTR,4)
#define CAN1_CS5                                 CAN_CS_REG(CAN1_BASE_PTR,5)
#define CAN1_ID5                                 CAN_ID_REG(CAN1_BASE_PTR,5)
#define CAN1_WORD05                              CAN_WORD0_REG(CAN1_BASE_PTR,5)
#define CAN1_WORD15                              CAN_WORD1_REG(CAN1_BASE_PTR,5)
#define CAN1_CS6                                 CAN_CS_REG(CAN1_BASE_PTR,6)
#define CAN1_ID6                                 CAN_ID_REG(CAN1_BASE_PTR,6)
#define CAN1_WORD06                              CAN_WORD0_REG(CAN1_BASE_PTR,6)
#define CAN1_WORD16                              CAN_WORD1_REG(CAN1_BASE_PTR,6)
#define CAN1_CS7                                 CAN_CS_REG(CAN1_BASE_PTR,7)
#define CAN1_ID7                                 CAN_ID_REG(CAN1_BASE_PTR,7)
#define CAN1_WORD07                              CAN_WORD0_REG(CAN1_BASE_PTR,7)
#define CAN1_WORD17                              CAN_WORD1_REG(CAN1_BASE_PTR,7)
#define CAN1_CS8                                 CAN_CS_REG(CAN1_BASE_PTR,8)
#define CAN1_ID8                                 CAN_ID_REG(CAN1_BASE_PTR,8)
#define CAN1_WORD08                              CAN_WORD0_REG(CAN1_BASE_PTR,8)
#define CAN1_WORD18                              CAN_WORD1_REG(CAN1_BASE_PTR,8)
#define CAN1_CS9                                 CAN_CS_REG(CAN1_BASE_PTR,9)
#define CAN1_ID9                                 CAN_ID_REG(CAN1_BASE_PTR,9)
#define CAN1_WORD09                              CAN_WORD0_REG(CAN1_BASE_PTR,9)
#define CAN1_WORD19                              CAN_WORD1_REG(CAN1_BASE_PTR,9)
#define CAN1_CS10                                CAN_CS_REG(CAN1_BASE_PTR,10)
#define CAN1_ID10                                CAN_ID_REG(CAN1_BASE_PTR,10)
#define CAN1_WORD010                             CAN_WORD0_REG(CAN1_BASE_PTR,10)
#define CAN1_WORD110                             CAN_WORD1_REG(CAN1_BASE_PTR,10)
#define CAN1_CS11                                CAN_CS_REG(CAN1_BASE_PTR,11)
#define CAN1_ID11                                CAN_ID_REG(CAN1_BASE_PTR,11)
#define CAN1_WORD011                             CAN_WORD0_REG(CAN1_BASE_PTR,11)
#define CAN1_WORD111                             CAN_WORD1_REG(CAN1_BASE_PTR,11)
#define CAN1_CS12                                CAN_CS_REG(CAN1_BASE_PTR,12)
#define CAN1_ID12                                CAN_ID_REG(CAN1_BASE_PTR,12)
#define CAN1_WORD012                             CAN_WORD0_REG(CAN1_BASE_PTR,12)
#define CAN1_WORD112                             CAN_WORD1_REG(CAN1_BASE_PTR,12)
#define CAN1_CS13                                CAN_CS_REG(CAN1_BASE_PTR,13)
#define CAN1_ID13                                CAN_ID_REG(CAN1_BASE_PTR,13)
#define CAN1_WORD013                             CAN_WORD0_REG(CAN1_BASE_PTR,13)
#define CAN1_WORD113                             CAN_WORD1_REG(CAN1_BASE_PTR,13)
#define CAN1_CS14                                CAN_CS_REG(CAN1_BASE_PTR,14)
#define CAN1_ID14                                CAN_ID_REG(CAN1_BASE_PTR,14)
#define CAN1_WORD014                             CAN_WORD0_REG(CAN1_BASE_PTR,14)
#define CAN1_WORD114                             CAN_WORD1_REG(CAN1_BASE_PTR,14)
#define CAN1_CS15                                CAN_CS_REG(CAN1_BASE_PTR,15)
#define CAN1_ID15                                CAN_ID_REG(CAN1_BASE_PTR,15)
#define CAN1_WORD015                             CAN_WORD0_REG(CAN1_BASE_PTR,15)
#define CAN1_WORD115                             CAN_WORD1_REG(CAN1_BASE_PTR,15)
#define CAN1_CS16                                CAN_CS_REG(CAN1_BASE_PTR,16)
#define CAN1_ID16                                CAN_ID_REG(CAN1_BASE_PTR,16)
#define CAN1_WORD016                             CAN_WORD0_REG(CAN1_BASE_PTR,16)
#define CAN1_WORD116                             CAN_WORD1_REG(CAN1_BASE_PTR,16)
#define CAN1_CS17                                CAN_CS_REG(CAN1_BASE_PTR,17)
#define CAN1_ID17                                CAN_ID_REG(CAN1_BASE_PTR,17)
#define CAN1_WORD017                             CAN_WORD0_REG(CAN1_BASE_PTR,17)
#define CAN1_WORD117                             CAN_WORD1_REG(CAN1_BASE_PTR,17)
#define CAN1_CS18                                CAN_CS_REG(CAN1_BASE_PTR,18)
#define CAN1_ID18                                CAN_ID_REG(CAN1_BASE_PTR,18)
#define CAN1_WORD018                             CAN_WORD0_REG(CAN1_BASE_PTR,18)
#define CAN1_WORD118                             CAN_WORD1_REG(CAN1_BASE_PTR,18)
#define CAN1_CS19                                CAN_CS_REG(CAN1_BASE_PTR,19)
#define CAN1_ID19                                CAN_ID_REG(CAN1_BASE_PTR,19)
#define CAN1_WORD019                             CAN_WORD0_REG(CAN1_BASE_PTR,19)
#define CAN1_WORD119                             CAN_WORD1_REG(CAN1_BASE_PTR,19)
#define CAN1_CS20                                CAN_CS_REG(CAN1_BASE_PTR,20)
#define CAN1_ID20                                CAN_ID_REG(CAN1_BASE_PTR,20)
#define CAN1_WORD020                             CAN_WORD0_REG(CAN1_BASE_PTR,20)
#define CAN1_WORD120                             CAN_WORD1_REG(CAN1_BASE_PTR,20)
#define CAN1_CS21                                CAN_CS_REG(CAN1_BASE_PTR,21)
#define CAN1_ID21                                CAN_ID_REG(CAN1_BASE_PTR,21)
#define CAN1_WORD021                             CAN_WORD0_REG(CAN1_BASE_PTR,21)
#define CAN1_WORD121                             CAN_WORD1_REG(CAN1_BASE_PTR,21)
#define CAN1_CS22                                CAN_CS_REG(CAN1_BASE_PTR,22)
#define CAN1_ID22                                CAN_ID_REG(CAN1_BASE_PTR,22)
#define CAN1_WORD022                             CAN_WORD0_REG(CAN1_BASE_PTR,22)
#define CAN1_WORD122                             CAN_WORD1_REG(CAN1_BASE_PTR,22)
#define CAN1_CS23                                CAN_CS_REG(CAN1_BASE_PTR,23)
#define CAN1_ID23                                CAN_ID_REG(CAN1_BASE_PTR,23)
#define CAN1_WORD023                             CAN_WORD0_REG(CAN1_BASE_PTR,23)
#define CAN1_WORD123                             CAN_WORD1_REG(CAN1_BASE_PTR,23)
#define CAN1_CS24                                CAN_CS_REG(CAN1_BASE_PTR,24)
#define CAN1_ID24                                CAN_ID_REG(CAN1_BASE_PTR,24)
#define CAN1_WORD024                             CAN_WORD0_REG(CAN1_BASE_PTR,24)
#define CAN1_WORD124                             CAN_WORD1_REG(CAN1_BASE_PTR,24)
#define CAN1_CS25                                CAN_CS_REG(CAN1_BASE_PTR,25)
#define CAN1_ID25                                CAN_ID_REG(CAN1_BASE_PTR,25)
#define CAN1_WORD025                             CAN_WORD0_REG(CAN1_BASE_PTR,25)
#define CAN1_WORD125                             CAN_WORD1_REG(CAN1_BASE_PTR,25)
#define CAN1_CS26                                CAN_CS_REG(CAN1_BASE_PTR,26)
#define CAN1_ID26                                CAN_ID_REG(CAN1_BASE_PTR,26)
#define CAN1_WORD026                             CAN_WORD0_REG(CAN1_BASE_PTR,26)
#define CAN1_WORD126                             CAN_WORD1_REG(CAN1_BASE_PTR,26)
#define CAN1_CS27                                CAN_CS_REG(CAN1_BASE_PTR,27)
#define CAN1_ID27                                CAN_ID_REG(CAN1_BASE_PTR,27)
#define CAN1_WORD027                             CAN_WORD0_REG(CAN1_BASE_PTR,27)
#define CAN1_WORD127                             CAN_WORD1_REG(CAN1_BASE_PTR,27)
#define CAN1_CS28                                CAN_CS_REG(CAN1_BASE_PTR,28)
#define CAN1_ID28                                CAN_ID_REG(CAN1_BASE_PTR,28)
#define CAN1_WORD028                             CAN_WORD0_REG(CAN1_BASE_PTR,28)
#define CAN1_WORD128                             CAN_WORD1_REG(CAN1_BASE_PTR,28)
#define CAN1_CS29                                CAN_CS_REG(CAN1_BASE_PTR,29)
#define CAN1_ID29                                CAN_ID_REG(CAN1_BASE_PTR,29)
#define CAN1_WORD029                             CAN_WORD0_REG(CAN1_BASE_PTR,29)
#define CAN1_WORD129                             CAN_WORD1_REG(CAN1_BASE_PTR,29)
#define CAN1_CS30                                CAN_CS_REG(CAN1_BASE_PTR,30)
#define CAN1_ID30                                CAN_ID_REG(CAN1_BASE_PTR,30)
#define CAN1_WORD030                             CAN_WORD0_REG(CAN1_BASE_PTR,30)
#define CAN1_WORD130                             CAN_WORD1_REG(CAN1_BASE_PTR,30)
#define CAN1_CS31                                CAN_CS_REG(CAN1_BASE_PTR,31)
#define CAN1_ID31                                CAN_ID_REG(CAN1_BASE_PTR,31)
#define CAN1_WORD031                             CAN_WORD0_REG(CAN1_BASE_PTR,31)
#define CAN1_WORD131                             CAN_WORD1_REG(CAN1_BASE_PTR,31)
#define CAN1_CS32                                CAN_CS_REG(CAN1_BASE_PTR,32)
#define CAN1_ID32                                CAN_ID_REG(CAN1_BASE_PTR,32)
#define CAN1_WORD032                             CAN_WORD0_REG(CAN1_BASE_PTR,32)
#define CAN1_WORD132                             CAN_WORD1_REG(CAN1_BASE_PTR,32)
#define CAN1_CS33                                CAN_CS_REG(CAN1_BASE_PTR,33)
#define CAN1_ID33                                CAN_ID_REG(CAN1_BASE_PTR,33)
#define CAN1_WORD033                             CAN_WORD0_REG(CAN1_BASE_PTR,33)
#define CAN1_WORD133                             CAN_WORD1_REG(CAN1_BASE_PTR,33)
#define CAN1_CS34                                CAN_CS_REG(CAN1_BASE_PTR,34)
#define CAN1_ID34                                CAN_ID_REG(CAN1_BASE_PTR,34)
#define CAN1_WORD034                             CAN_WORD0_REG(CAN1_BASE_PTR,34)
#define CAN1_WORD134                             CAN_WORD1_REG(CAN1_BASE_PTR,34)
#define CAN1_CS35                                CAN_CS_REG(CAN1_BASE_PTR,35)
#define CAN1_ID35                                CAN_ID_REG(CAN1_BASE_PTR,35)
#define CAN1_WORD035                             CAN_WORD0_REG(CAN1_BASE_PTR,35)
#define CAN1_WORD135                             CAN_WORD1_REG(CAN1_BASE_PTR,35)
#define CAN1_CS36                                CAN_CS_REG(CAN1_BASE_PTR,36)
#define CAN1_ID36                                CAN_ID_REG(CAN1_BASE_PTR,36)
#define CAN1_WORD036                             CAN_WORD0_REG(CAN1_BASE_PTR,36)
#define CAN1_WORD136                             CAN_WORD1_REG(CAN1_BASE_PTR,36)
#define CAN1_CS37                                CAN_CS_REG(CAN1_BASE_PTR,37)
#define CAN1_ID37                                CAN_ID_REG(CAN1_BASE_PTR,37)
#define CAN1_WORD037                             CAN_WORD0_REG(CAN1_BASE_PTR,37)
#define CAN1_WORD137                             CAN_WORD1_REG(CAN1_BASE_PTR,37)
#define CAN1_CS38                                CAN_CS_REG(CAN1_BASE_PTR,38)
#define CAN1_ID38                                CAN_ID_REG(CAN1_BASE_PTR,38)
#define CAN1_WORD038                             CAN_WORD0_REG(CAN1_BASE_PTR,38)
#define CAN1_WORD138                             CAN_WORD1_REG(CAN1_BASE_PTR,38)
#define CAN1_CS39                                CAN_CS_REG(CAN1_BASE_PTR,39)
#define CAN1_ID39                                CAN_ID_REG(CAN1_BASE_PTR,39)
#define CAN1_WORD039                             CAN_WORD0_REG(CAN1_BASE_PTR,39)
#define CAN1_WORD139                             CAN_WORD1_REG(CAN1_BASE_PTR,39)
#define CAN1_CS40                                CAN_CS_REG(CAN1_BASE_PTR,40)
#define CAN1_ID40                                CAN_ID_REG(CAN1_BASE_PTR,40)
#define CAN1_WORD040                             CAN_WORD0_REG(CAN1_BASE_PTR,40)
#define CAN1_WORD140                             CAN_WORD1_REG(CAN1_BASE_PTR,40)
#define CAN1_CS41                                CAN_CS_REG(CAN1_BASE_PTR,41)
#define CAN1_ID41                                CAN_ID_REG(CAN1_BASE_PTR,41)
#define CAN1_WORD041                             CAN_WORD0_REG(CAN1_BASE_PTR,41)
#define CAN1_WORD141                             CAN_WORD1_REG(CAN1_BASE_PTR,41)
#define CAN1_CS42                                CAN_CS_REG(CAN1_BASE_PTR,42)
#define CAN1_ID42                                CAN_ID_REG(CAN1_BASE_PTR,42)
#define CAN1_WORD042                             CAN_WORD0_REG(CAN1_BASE_PTR,42)
#define CAN1_WORD142                             CAN_WORD1_REG(CAN1_BASE_PTR,42)
#define CAN1_CS43                                CAN_CS_REG(CAN1_BASE_PTR,43)
#define CAN1_ID43                                CAN_ID_REG(CAN1_BASE_PTR,43)
#define CAN1_WORD043                             CAN_WORD0_REG(CAN1_BASE_PTR,43)
#define CAN1_WORD143                             CAN_WORD1_REG(CAN1_BASE_PTR,43)
#define CAN1_CS44                                CAN_CS_REG(CAN1_BASE_PTR,44)
#define CAN1_ID44                                CAN_ID_REG(CAN1_BASE_PTR,44)
#define CAN1_WORD044                             CAN_WORD0_REG(CAN1_BASE_PTR,44)
#define CAN1_WORD144                             CAN_WORD1_REG(CAN1_BASE_PTR,44)
#define CAN1_CS45                                CAN_CS_REG(CAN1_BASE_PTR,45)
#define CAN1_ID45                                CAN_ID_REG(CAN1_BASE_PTR,45)
#define CAN1_WORD045                             CAN_WORD0_REG(CAN1_BASE_PTR,45)
#define CAN1_WORD145                             CAN_WORD1_REG(CAN1_BASE_PTR,45)
#define CAN1_CS46                                CAN_CS_REG(CAN1_BASE_PTR,46)
#define CAN1_ID46                                CAN_ID_REG(CAN1_BASE_PTR,46)
#define CAN1_WORD046                             CAN_WORD0_REG(CAN1_BASE_PTR,46)
#define CAN1_WORD146                             CAN_WORD1_REG(CAN1_BASE_PTR,46)
#define CAN1_CS47                                CAN_CS_REG(CAN1_BASE_PTR,47)
#define CAN1_ID47                                CAN_ID_REG(CAN1_BASE_PTR,47)
#define CAN1_WORD047                             CAN_WORD0_REG(CAN1_BASE_PTR,47)
#define CAN1_WORD147                             CAN_WORD1_REG(CAN1_BASE_PTR,47)
#define CAN1_CS48                                CAN_CS_REG(CAN1_BASE_PTR,48)
#define CAN1_ID48                                CAN_ID_REG(CAN1_BASE_PTR,48)
#define CAN1_WORD048                             CAN_WORD0_REG(CAN1_BASE_PTR,48)
#define CAN1_WORD148                             CAN_WORD1_REG(CAN1_BASE_PTR,48)
#define CAN1_CS49                                CAN_CS_REG(CAN1_BASE_PTR,49)
#define CAN1_ID49                                CAN_ID_REG(CAN1_BASE_PTR,49)
#define CAN1_WORD049                             CAN_WORD0_REG(CAN1_BASE_PTR,49)
#define CAN1_WORD149                             CAN_WORD1_REG(CAN1_BASE_PTR,49)
#define CAN1_CS50                                CAN_CS_REG(CAN1_BASE_PTR,50)
#define CAN1_ID50                                CAN_ID_REG(CAN1_BASE_PTR,50)
#define CAN1_WORD050                             CAN_WORD0_REG(CAN1_BASE_PTR,50)
#define CAN1_WORD150                             CAN_WORD1_REG(CAN1_BASE_PTR,50)
#define CAN1_CS51                                CAN_CS_REG(CAN1_BASE_PTR,51)
#define CAN1_ID51                                CAN_ID_REG(CAN1_BASE_PTR,51)
#define CAN1_WORD051                             CAN_WORD0_REG(CAN1_BASE_PTR,51)
#define CAN1_WORD151                             CAN_WORD1_REG(CAN1_BASE_PTR,51)
#define CAN1_CS52                                CAN_CS_REG(CAN1_BASE_PTR,52)
#define CAN1_ID52                                CAN_ID_REG(CAN1_BASE_PTR,52)
#define CAN1_WORD052                             CAN_WORD0_REG(CAN1_BASE_PTR,52)
#define CAN1_WORD152                             CAN_WORD1_REG(CAN1_BASE_PTR,52)
#define CAN1_CS53                                CAN_CS_REG(CAN1_BASE_PTR,53)
#define CAN1_ID53                                CAN_ID_REG(CAN1_BASE_PTR,53)
#define CAN1_WORD053                             CAN_WORD0_REG(CAN1_BASE_PTR,53)
#define CAN1_WORD153                             CAN_WORD1_REG(CAN1_BASE_PTR,53)
#define CAN1_CS54                                CAN_CS_REG(CAN1_BASE_PTR,54)
#define CAN1_ID54                                CAN_ID_REG(CAN1_BASE_PTR,54)
#define CAN1_WORD054                             CAN_WORD0_REG(CAN1_BASE_PTR,54)
#define CAN1_WORD154                             CAN_WORD1_REG(CAN1_BASE_PTR,54)
#define CAN1_CS55                                CAN_CS_REG(CAN1_BASE_PTR,55)
#define CAN1_ID55                                CAN_ID_REG(CAN1_BASE_PTR,55)
#define CAN1_WORD055                             CAN_WORD0_REG(CAN1_BASE_PTR,55)
#define CAN1_WORD155                             CAN_WORD1_REG(CAN1_BASE_PTR,55)
#define CAN1_CS56                                CAN_CS_REG(CAN1_BASE_PTR,56)
#define CAN1_ID56                                CAN_ID_REG(CAN1_BASE_PTR,56)
#define CAN1_WORD056                             CAN_WORD0_REG(CAN1_BASE_PTR,56)
#define CAN1_WORD156                             CAN_WORD1_REG(CAN1_BASE_PTR,56)
#define CAN1_CS57                                CAN_CS_REG(CAN1_BASE_PTR,57)
#define CAN1_ID57                                CAN_ID_REG(CAN1_BASE_PTR,57)
#define CAN1_WORD057                             CAN_WORD0_REG(CAN1_BASE_PTR,57)
#define CAN1_WORD157                             CAN_WORD1_REG(CAN1_BASE_PTR,57)
#define CAN1_CS58                                CAN_CS_REG(CAN1_BASE_PTR,58)
#define CAN1_ID58                                CAN_ID_REG(CAN1_BASE_PTR,58)
#define CAN1_WORD058                             CAN_WORD0_REG(CAN1_BASE_PTR,58)
#define CAN1_WORD158                             CAN_WORD1_REG(CAN1_BASE_PTR,58)
#define CAN1_CS59                                CAN_CS_REG(CAN1_BASE_PTR,59)
#define CAN1_ID59                                CAN_ID_REG(CAN1_BASE_PTR,59)
#define CAN1_WORD059                             CAN_WORD0_REG(CAN1_BASE_PTR,59)
#define CAN1_WORD159                             CAN_WORD1_REG(CAN1_BASE_PTR,59)
#define CAN1_CS60                                CAN_CS_REG(CAN1_BASE_PTR,60)
#define CAN1_ID60                                CAN_ID_REG(CAN1_BASE_PTR,60)
#define CAN1_WORD060                             CAN_WORD0_REG(CAN1_BASE_PTR,60)
#define CAN1_WORD160                             CAN_WORD1_REG(CAN1_BASE_PTR,60)
#define CAN1_CS61                                CAN_CS_REG(CAN1_BASE_PTR,61)
#define CAN1_ID61                                CAN_ID_REG(CAN1_BASE_PTR,61)
#define CAN1_WORD061                             CAN_WORD0_REG(CAN1_BASE_PTR,61)
#define CAN1_WORD161                             CAN_WORD1_REG(CAN1_BASE_PTR,61)
#define CAN1_CS62                                CAN_CS_REG(CAN1_BASE_PTR,62)
#define CAN1_ID62                                CAN_ID_REG(CAN1_BASE_PTR,62)
#define CAN1_WORD062                             CAN_WORD0_REG(CAN1_BASE_PTR,62)
#define CAN1_WORD162                             CAN_WORD1_REG(CAN1_BASE_PTR,62)
#define CAN1_CS63                                CAN_CS_REG(CAN1_BASE_PTR,63)
#define CAN1_ID63                                CAN_ID_REG(CAN1_BASE_PTR,63)
#define CAN1_WORD063                             CAN_WORD0_REG(CAN1_BASE_PTR,63)
#define CAN1_WORD163                             CAN_WORD1_REG(CAN1_BASE_PTR,63)
#define CAN1_RXIMR0                              CAN_RXIMR_REG(CAN1_BASE_PTR,0)
#define CAN1_RXIMR1                              CAN_RXIMR_REG(CAN1_BASE_PTR,1)
#define CAN1_RXIMR2                              CAN_RXIMR_REG(CAN1_BASE_PTR,2)
#define CAN1_RXIMR3                              CAN_RXIMR_REG(CAN1_BASE_PTR,3)
#define CAN1_RXIMR4                              CAN_RXIMR_REG(CAN1_BASE_PTR,4)
#define CAN1_RXIMR5                              CAN_RXIMR_REG(CAN1_BASE_PTR,5)
#define CAN1_RXIMR6                              CAN_RXIMR_REG(CAN1_BASE_PTR,6)
#define CAN1_RXIMR7                              CAN_RXIMR_REG(CAN1_BASE_PTR,7)
#define CAN1_RXIMR8                              CAN_RXIMR_REG(CAN1_BASE_PTR,8)
#define CAN1_RXIMR9                              CAN_RXIMR_REG(CAN1_BASE_PTR,9)
#define CAN1_RXIMR10                             CAN_RXIMR_REG(CAN1_BASE_PTR,10)
#define CAN1_RXIMR11                             CAN_RXIMR_REG(CAN1_BASE_PTR,11)
#define CAN1_RXIMR12                             CAN_RXIMR_REG(CAN1_BASE_PTR,12)
#define CAN1_RXIMR13                             CAN_RXIMR_REG(CAN1_BASE_PTR,13)
#define CAN1_RXIMR14                             CAN_RXIMR_REG(CAN1_BASE_PTR,14)
#define CAN1_RXIMR15                             CAN_RXIMR_REG(CAN1_BASE_PTR,15)
#define CAN1_RXIMR16                             CAN_RXIMR_REG(CAN1_BASE_PTR,16)
#define CAN1_RXIMR17                             CAN_RXIMR_REG(CAN1_BASE_PTR,17)
#define CAN1_RXIMR18                             CAN_RXIMR_REG(CAN1_BASE_PTR,18)
#define CAN1_RXIMR19                             CAN_RXIMR_REG(CAN1_BASE_PTR,19)
#define CAN1_RXIMR20                             CAN_RXIMR_REG(CAN1_BASE_PTR,20)
#define CAN1_RXIMR21                             CAN_RXIMR_REG(CAN1_BASE_PTR,21)
#define CAN1_RXIMR22                             CAN_RXIMR_REG(CAN1_BASE_PTR,22)
#define CAN1_RXIMR23                             CAN_RXIMR_REG(CAN1_BASE_PTR,23)
#define CAN1_RXIMR24                             CAN_RXIMR_REG(CAN1_BASE_PTR,24)
#define CAN1_RXIMR25                             CAN_RXIMR_REG(CAN1_BASE_PTR,25)
#define CAN1_RXIMR26                             CAN_RXIMR_REG(CAN1_BASE_PTR,26)
#define CAN1_RXIMR27                             CAN_RXIMR_REG(CAN1_BASE_PTR,27)
#define CAN1_RXIMR28                             CAN_RXIMR_REG(CAN1_BASE_PTR,28)
#define CAN1_RXIMR29                             CAN_RXIMR_REG(CAN1_BASE_PTR,29)
#define CAN1_RXIMR30                             CAN_RXIMR_REG(CAN1_BASE_PTR,30)
#define CAN1_RXIMR31                             CAN_RXIMR_REG(CAN1_BASE_PTR,31)
#define CAN1_RXIMR32                             CAN_RXIMR_REG(CAN1_BASE_PTR,32)
#define CAN1_RXIMR33                             CAN_RXIMR_REG(CAN1_BASE_PTR,33)
#define CAN1_RXIMR34                             CAN_RXIMR_REG(CAN1_BASE_PTR,34)
#define CAN1_RXIMR35                             CAN_RXIMR_REG(CAN1_BASE_PTR,35)
#define CAN1_RXIMR36                             CAN_RXIMR_REG(CAN1_BASE_PTR,36)
#define CAN1_RXIMR37                             CAN_RXIMR_REG(CAN1_BASE_PTR,37)
#define CAN1_RXIMR38                             CAN_RXIMR_REG(CAN1_BASE_PTR,38)
#define CAN1_RXIMR39                             CAN_RXIMR_REG(CAN1_BASE_PTR,39)
#define CAN1_RXIMR40                             CAN_RXIMR_REG(CAN1_BASE_PTR,40)
#define CAN1_RXIMR41                             CAN_RXIMR_REG(CAN1_BASE_PTR,41)
#define CAN1_RXIMR42                             CAN_RXIMR_REG(CAN1_BASE_PTR,42)
#define CAN1_RXIMR43                             CAN_RXIMR_REG(CAN1_BASE_PTR,43)
#define CAN1_RXIMR44                             CAN_RXIMR_REG(CAN1_BASE_PTR,44)
#define CAN1_RXIMR45                             CAN_RXIMR_REG(CAN1_BASE_PTR,45)
#define CAN1_RXIMR46                             CAN_RXIMR_REG(CAN1_BASE_PTR,46)
#define CAN1_RXIMR47                             CAN_RXIMR_REG(CAN1_BASE_PTR,47)
#define CAN1_RXIMR48                             CAN_RXIMR_REG(CAN1_BASE_PTR,48)
#define CAN1_RXIMR49                             CAN_RXIMR_REG(CAN1_BASE_PTR,49)
#define CAN1_RXIMR50                             CAN_RXIMR_REG(CAN1_BASE_PTR,50)
#define CAN1_RXIMR51                             CAN_RXIMR_REG(CAN1_BASE_PTR,51)
#define CAN1_RXIMR52                             CAN_RXIMR_REG(CAN1_BASE_PTR,52)
#define CAN1_RXIMR53                             CAN_RXIMR_REG(CAN1_BASE_PTR,53)
#define CAN1_RXIMR54                             CAN_RXIMR_REG(CAN1_BASE_PTR,54)
#define CAN1_RXIMR55                             CAN_RXIMR_REG(CAN1_BASE_PTR,55)
#define CAN1_RXIMR56                             CAN_RXIMR_REG(CAN1_BASE_PTR,56)
#define CAN1_RXIMR57                             CAN_RXIMR_REG(CAN1_BASE_PTR,57)
#define CAN1_RXIMR58                             CAN_RXIMR_REG(CAN1_BASE_PTR,58)
#define CAN1_RXIMR59                             CAN_RXIMR_REG(CAN1_BASE_PTR,59)
#define CAN1_RXIMR60                             CAN_RXIMR_REG(CAN1_BASE_PTR,60)
#define CAN1_RXIMR61                             CAN_RXIMR_REG(CAN1_BASE_PTR,61)
#define CAN1_RXIMR62                             CAN_RXIMR_REG(CAN1_BASE_PTR,62)
#define CAN1_RXIMR63                             CAN_RXIMR_REG(CAN1_BASE_PTR,63)
#define CAN1_MECR                                CAN_MECR_REG(CAN1_BASE_PTR)
#define CAN1_ERRIAR                              CAN_ERRIAR_REG(CAN1_BASE_PTR)
#define CAN1_ERRIDPR                             CAN_ERRIDPR_REG(CAN1_BASE_PTR)
#define CAN1_ERRIPPR                             CAN_ERRIPPR_REG(CAN1_BASE_PTR)
#define CAN1_RERRAR                              CAN_RERRAR_REG(CAN1_BASE_PTR)
#define CAN1_RERRDR                              CAN_RERRDR_REG(CAN1_BASE_PTR)
#define CAN1_RERRSYNR                            CAN_RERRSYNR_REG(CAN1_BASE_PTR)
#define CAN1_ERRSR                               CAN_ERRSR_REG(CAN1_BASE_PTR)

/* CAN - Register array accessors */
#define CAN0_CS(index)                           CAN_CS_REG(CAN0_BASE_PTR,index)
#define CAN1_CS(index)                           CAN_CS_REG(CAN1_BASE_PTR,index)
#define CAN0_ID(index)                           CAN_ID_REG(CAN0_BASE_PTR,index)
#define CAN1_ID(index)                           CAN_ID_REG(CAN1_BASE_PTR,index)
#define CAN0_WORD0(index)                        CAN_WORD0_REG(CAN0_BASE_PTR,index)
#define CAN1_WORD0(index)                        CAN_WORD0_REG(CAN1_BASE_PTR,index)
#define CAN0_WORD1(index)                        CAN_WORD1_REG(CAN0_BASE_PTR,index)
#define CAN1_WORD1(index)                        CAN_WORD1_REG(CAN1_BASE_PTR,index)
#define CAN0_RXIMR(index)                        CAN_RXIMR_REG(CAN0_BASE_PTR,index)
#define CAN1_RXIMR(index)                        CAN_RXIMR_REG(CAN1_BASE_PTR,index)

/*!
 * @}
 */ /* end of group CAN_Register_Accessor_Macros */
/* ----------------------------------------------------------------------------
   -- CRC
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CRC_Peripheral CRC
 * @{
 */

/** CRC - Peripheral register structure */
typedef struct CRC_MemMap {
  union {                                          /* offset: 0x0 */
    struct {                                         /* offset: 0x0 */
      uint16_t DATAL;                                  /**< CRC_DATAL register., offset: 0x0 */
      uint16_t DATAH;                                  /**< CRC_DATAH register., offset: 0x2 */
    } ACCESS16BIT;
    uint32_t DATA;                                   /**< CRC Data register, offset: 0x0 */
    struct {                                         /* offset: 0x0 */
      uint8_t DATALL;                                  /**< CRC_DATALL register., offset: 0x0 */
      uint8_t DATALU;                                  /**< CRC_DATALU register., offset: 0x1 */
      uint8_t DATAHL;                                  /**< CRC_DATAHL register., offset: 0x2 */
      uint8_t DATAHU;                                  /**< CRC_DATAHU register., offset: 0x3 */
    } ACCESS8BIT;
  };
  union {                                          /* offset: 0x4 */
    struct {                                         /* offset: 0x4 */
      uint16_t GPOLYL;                                 /**< CRC_GPOLYL register., offset: 0x4 */
      uint16_t GPOLYH;                                 /**< CRC_GPOLYH register., offset: 0x6 */
    } GPOLY_ACCESS16BIT;
    uint32_t GPOLY;                                  /**< CRC Polynomial register, offset: 0x4 */
    struct {                                         /* offset: 0x4 */
      uint8_t GPOLYLL;                                 /**< CRC_GPOLYLL register., offset: 0x4 */
      uint8_t GPOLYLU;                                 /**< CRC_GPOLYLU register., offset: 0x5 */
      uint8_t GPOLYHL;                                 /**< CRC_GPOLYHL register., offset: 0x6 */
      uint8_t GPOLYHU;                                 /**< CRC_GPOLYHU register., offset: 0x7 */
    } GPOLY_ACCESS8BIT;
  };
  union {                                          /* offset: 0x8 */
    uint32_t CTRL;                                   /**< CRC Control register, offset: 0x8 */
    struct {                                         /* offset: 0x8 */
      uint8_t RESERVED_0[3];
      uint8_t CTRLHU;                                  /**< CRC_CTRLHU register., offset: 0xB */
    } CTRL_ACCESS8BIT;
  };
} volatile *CRC_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- CRC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CRC_Register_Accessor_Macros CRC - Register accessor macros
 * @{
 */


/* CRC - Register accessors */
#define CRC_DATAL_REG(base)                      ((base)->ACCESS16BIT.DATAL)
#define CRC_DATAH_REG(base)                      ((base)->ACCESS16BIT.DATAH)
#define CRC_DATA_REG(base)                       ((base)->DATA)
#define CRC_DATALL_REG(base)                     ((base)->ACCESS8BIT.DATALL)
#define CRC_DATALU_REG(base)                     ((base)->ACCESS8BIT.DATALU)
#define CRC_DATAHL_REG(base)                     ((base)->ACCESS8BIT.DATAHL)
#define CRC_DATAHU_REG(base)                     ((base)->ACCESS8BIT.DATAHU)
#define CRC_GPOLYL_REG(base)                     ((base)->GPOLY_ACCESS16BIT.GPOLYL)
#define CRC_GPOLYH_REG(base)                     ((base)->GPOLY_ACCESS16BIT.GPOLYH)
#define CRC_GPOLY_REG(base)                      ((base)->GPOLY)
#define CRC_GPOLYLL_REG(base)                    ((base)->GPOLY_ACCESS8BIT.GPOLYLL)
#define CRC_GPOLYLU_REG(base)                    ((base)->GPOLY_ACCESS8BIT.GPOLYLU)
#define CRC_GPOLYHL_REG(base)                    ((base)->GPOLY_ACCESS8BIT.GPOLYHL)
#define CRC_GPOLYHU_REG(base)                    ((base)->GPOLY_ACCESS8BIT.GPOLYHU)
#define CRC_CTRL_REG(base)                       ((base)->CTRL)
#define CRC_CTRLHU_REG(base)                     ((base)->CTRL_ACCESS8BIT.CTRLHU)

/*!
 * @}
 */ /* end of group CRC_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- CRC Register Masks
   ---------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------------
   -- CSU
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CSU_Peripheral CSU
 * @{
 */

/** CSU - Peripheral register structure */
typedef struct CSU_MemMap {
  uint32_t CSL[64];                                /**< Config Security Level, array offset: 0x0, array step: 0x4 */
  uint8_t RESERVED_0[256];
  uint32_t HP[2];                                  /**< Hprot Register, array offset: 0x200, array step: 0x4 */
  uint8_t RESERVED_1[16];
  uint32_t SA[2];                                  /**< Secure Access register, array offset: 0x218, array step: 0x4 */
  uint8_t RESERVED_2[16];
  uint32_t AMK[1];                                 /**< Alarm Mask register, array offset: 0x230, array step: 0x4 */
  uint8_t RESERVED_3[16];
  uint32_t AROUT[13];                              /**< Alarm Routing Register, array offset: 0x244, array step: 0x4 */
  uint8_t RESERVED_4[204];
  uint8_t ASOFT;                                   /**< Soft Alarm Register, offset: 0x344 */
  uint8_t RESERVED_5[3];
  uint16_t ACOUNTER;                               /**< Alarm Counter Register, offset: 0x348 */
  uint8_t RESERVED_6[2];
  uint16_t ACONTROL;                               /**< Alarm Control Register, offset: 0x34C */
  uint8_t RESERVED_7[10];
  uint32_t HPC[2];                                 /**< Hprot Control Register, array offset: 0x358, array step: 0x4 */
  uint8_t RESERVED_8[24];
  uint32_t ISR[1];                                 /**< Interrupt Status Register, array offset: 0x378, array step: 0x4 */
} volatile *CSU_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- CSU - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CSU_Register_Accessor_Macros CSU - Register accessor macros
 * @{
 */


/* CSU - Register accessors */
#define CSU_CSL_REG(base,index)                  ((base)->CSL[index])
#define CSU_HP_REG(base,index)                   ((base)->HP[index])
#define CSU_SA_REG(base,index)                   ((base)->SA[index])
#define CSU_AMK_REG(base,index)                  ((base)->AMK[index])
#define CSU_AROUT_REG(base,index)                ((base)->AROUT[index])
#define CSU_ASOFT_REG(base)                      ((base)->ASOFT)
#define CSU_ACOUNTER_REG(base)                   ((base)->ACOUNTER)
#define CSU_ACONTROL_REG(base)                   ((base)->ACONTROL)
#define CSU_HPC_REG(base,index)                  ((base)->HPC[index])
#define CSU_ISR_REG(base,index)                  ((base)->ISR[index])

/*!
 * @}
 */ /* end of group CSU_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- CSU Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CSU_Register_Masks CSU Register Masks
 * @{
 */

/* CSL Bit Fields */
#define CSU_CSL_CSL_2n_MASK                      0xFFu
#define CSU_CSL_CSL_2n_SHIFT                     0
#define CSU_CSL_CSL_2n(x)                        (((uint32_t)(((uint32_t)(x))<<CSU_CSL_CSL_2n_SHIFT))&CSU_CSL_CSL_2n_MASK)
#define CSU_CSL_L_2n_MASK                        0x100u
#define CSU_CSL_L_2n_SHIFT                       8
#define CSU_CSL_CSL_2n1_MASK                     0xFF0000u
#define CSU_CSL_CSL_2n1_SHIFT                    16
#define CSU_CSL_CSL_2n1(x)                       (((uint32_t)(((uint32_t)(x))<<CSU_CSL_CSL_2n1_SHIFT))&CSU_CSL_CSL_2n1_MASK)
#define CSU_CSL_L_2n1_MASK                       0x1000000u
#define CSU_CSL_L_2n1_SHIFT                      24
/* HP Bit Fields */
#define CSU_HP_HP0_n_MASK                        0x1u
#define CSU_HP_HP0_n_SHIFT                       0
#define CSU_HP_L0_n_MASK                         0x2u
#define CSU_HP_L0_n_SHIFT                        1
#define CSU_HP_HP1_n_MASK                        0x4u
#define CSU_HP_HP1_n_SHIFT                       2
#define CSU_HP_L1_n_MASK                         0x8u
#define CSU_HP_L1_n_SHIFT                        3
#define CSU_HP_HP2_n_MASK                        0x10u
#define CSU_HP_HP2_n_SHIFT                       4
#define CSU_HP_L2_n_MASK                         0x20u
#define CSU_HP_L2_n_SHIFT                        5
#define CSU_HP_HP3_n_MASK                        0x40u
#define CSU_HP_HP3_n_SHIFT                       6
#define CSU_HP_L3_n_MASK                         0x80u
#define CSU_HP_L3_n_SHIFT                        7
#define CSU_HP_HP4_n_MASK                        0x100u
#define CSU_HP_HP4_n_SHIFT                       8
#define CSU_HP_L4_n_MASK                         0x200u
#define CSU_HP_L4_n_SHIFT                        9
#define CSU_HP_HP5_n_MASK                        0x400u
#define CSU_HP_HP5_n_SHIFT                       10
#define CSU_HP_L5_n_MASK                         0x800u
#define CSU_HP_L5_n_SHIFT                        11
#define CSU_HP_HP6_n_MASK                        0x1000u
#define CSU_HP_HP6_n_SHIFT                       12
#define CSU_HP_L6_n_MASK                         0x2000u
#define CSU_HP_L6_n_SHIFT                        13
#define CSU_HP_HP7_n_MASK                        0x4000u
#define CSU_HP_HP7_n_SHIFT                       14
#define CSU_HP_L7_n_MASK                         0x8000u
#define CSU_HP_L7_n_SHIFT                        15
#define CSU_HP_HP8_n_MASK                        0x10000u
#define CSU_HP_HP8_n_SHIFT                       16
#define CSU_HP_L8_n_MASK                         0x20000u
#define CSU_HP_L8_n_SHIFT                        17
#define CSU_HP_HP9_n_MASK                        0x40000u
#define CSU_HP_HP9_n_SHIFT                       18
#define CSU_HP_L9_n_MASK                         0x80000u
#define CSU_HP_L9_n_SHIFT                        19
#define CSU_HP_HP10_n_MASK                       0x100000u
#define CSU_HP_HP10_n_SHIFT                      20
#define CSU_HP_L10_n_MASK                        0x200000u
#define CSU_HP_L10_n_SHIFT                       21
#define CSU_HP_HP11_n_MASK                       0x400000u
#define CSU_HP_HP11_n_SHIFT                      22
#define CSU_HP_L11_n_MASK                        0x800000u
#define CSU_HP_L11_n_SHIFT                       23
#define CSU_HP_HP12_n_MASK                       0x1000000u
#define CSU_HP_HP12_n_SHIFT                      24
#define CSU_HP_L12_n_MASK                        0x2000000u
#define CSU_HP_L12_n_SHIFT                       25
#define CSU_HP_HP13_n_MASK                       0x4000000u
#define CSU_HP_HP13_n_SHIFT                      26
#define CSU_HP_L13_n_MASK                        0x8000000u
#define CSU_HP_L13_n_SHIFT                       27
#define CSU_HP_HP14_n_MASK                       0x10000000u
#define CSU_HP_HP14_n_SHIFT                      28
#define CSU_HP_L14_n_MASK                        0x20000000u
#define CSU_HP_L14_n_SHIFT                       29
#define CSU_HP_HP15_n_MASK                       0x40000000u
#define CSU_HP_HP15_n_SHIFT                      30
#define CSU_HP_L15_n_MASK                        0x80000000u
#define CSU_HP_L15_n_SHIFT                       31
/* SA Bit Fields */
#define CSU_SA_SA0_n_MASK                        0x1u
#define CSU_SA_SA0_n_SHIFT                       0
#define CSU_SA_L0_n_MASK                         0x2u
#define CSU_SA_L0_n_SHIFT                        1
#define CSU_SA_SA1_n_MASK                        0x4u
#define CSU_SA_SA1_n_SHIFT                       2
#define CSU_SA_L1_n_MASK                         0x8u
#define CSU_SA_L1_n_SHIFT                        3
#define CSU_SA_SA2_n_MASK                        0x10u
#define CSU_SA_SA2_n_SHIFT                       4
#define CSU_SA_L2_n_MASK                         0x20u
#define CSU_SA_L2_n_SHIFT                        5
#define CSU_SA_SA3_n_MASK                        0x40u
#define CSU_SA_SA3_n_SHIFT                       6
#define CSU_SA_L3_n_MASK                         0x80u
#define CSU_SA_L3_n_SHIFT                        7
#define CSU_SA_SA4_n_MASK                        0x100u
#define CSU_SA_SA4_n_SHIFT                       8
#define CSU_SA_L4_n_MASK                         0x200u
#define CSU_SA_L4_n_SHIFT                        9
#define CSU_SA_SA5_n_MASK                        0x400u
#define CSU_SA_SA5_n_SHIFT                       10
#define CSU_SA_L5_n_MASK                         0x800u
#define CSU_SA_L5_n_SHIFT                        11
#define CSU_SA_SA6_n_MASK                        0x1000u
#define CSU_SA_SA6_n_SHIFT                       12
#define CSU_SA_L6_n_MASK                         0x2000u
#define CSU_SA_L6_n_SHIFT                        13
#define CSU_SA_SA7_n_MASK                        0x4000u
#define CSU_SA_SA7_n_SHIFT                       14
#define CSU_SA_L7_n_MASK                         0x8000u
#define CSU_SA_L7_n_SHIFT                        15
#define CSU_SA_SA8_n_MASK                        0x10000u
#define CSU_SA_SA8_n_SHIFT                       16
#define CSU_SA_L8_n_MASK                         0x20000u
#define CSU_SA_L8_n_SHIFT                        17
#define CSU_SA_SA9_n_MASK                        0x40000u
#define CSU_SA_SA9_n_SHIFT                       18
#define CSU_SA_L9_n_MASK                         0x80000u
#define CSU_SA_L9_n_SHIFT                        19
#define CSU_SA_SA10_n_MASK                       0x100000u
#define CSU_SA_SA10_n_SHIFT                      20
#define CSU_SA_L10_n_MASK                        0x200000u
#define CSU_SA_L10_n_SHIFT                       21
#define CSU_SA_SA11_n_MASK                       0x400000u
#define CSU_SA_SA11_n_SHIFT                      22
#define CSU_SA_L11_n_MASK                        0x800000u
#define CSU_SA_L11_n_SHIFT                       23
#define CSU_SA_SA12_n_MASK                       0x1000000u
#define CSU_SA_SA12_n_SHIFT                      24
#define CSU_SA_L12_n_MASK                        0x2000000u
#define CSU_SA_L12_n_SHIFT                       25
#define CSU_SA_SA13_n_MASK                       0x4000000u
#define CSU_SA_SA13_n_SHIFT                      26
#define CSU_SA_L13_n_MASK                        0x8000000u
#define CSU_SA_L13_n_SHIFT                       27
#define CSU_SA_SA14_n_MASK                       0x10000000u
#define CSU_SA_SA14_n_SHIFT                      28
#define CSU_SA_L14_n_MASK                        0x20000000u
#define CSU_SA_L14_n_SHIFT                       29
#define CSU_SA_SA15_n_MASK                       0x40000000u
#define CSU_SA_SA15_n_SHIFT                      30
#define CSU_SA_L15_n_MASK                        0x80000000u
#define CSU_SA_L15_n_SHIFT                       31
/* AMK Bit Fields */
#define CSU_AMK_AMK0_n_MASK                      0x1u
#define CSU_AMK_AMK0_n_SHIFT                     0
#define CSU_AMK_L0_n_MASK                        0x2u
#define CSU_AMK_L0_n_SHIFT                       1
#define CSU_AMK_AMK1_n_MASK                      0x4u
#define CSU_AMK_AMK1_n_SHIFT                     2
#define CSU_AMK_L1_n_MASK                        0x8u
#define CSU_AMK_L1_n_SHIFT                       3
#define CSU_AMK_AMK2_n_MASK                      0x10u
#define CSU_AMK_AMK2_n_SHIFT                     4
#define CSU_AMK_L2_n_MASK                        0x20u
#define CSU_AMK_L2_n_SHIFT                       5
#define CSU_AMK_AMK3_n_MASK                      0x40u
#define CSU_AMK_AMK3_n_SHIFT                     6
#define CSU_AMK_L3_n_MASK                        0x80u
#define CSU_AMK_L3_n_SHIFT                       7
#define CSU_AMK_AMK4_n_MASK                      0x100u
#define CSU_AMK_AMK4_n_SHIFT                     8
#define CSU_AMK_L4_n_MASK                        0x200u
#define CSU_AMK_L4_n_SHIFT                       9
#define CSU_AMK_AMK5_n_MASK                      0x400u
#define CSU_AMK_AMK5_n_SHIFT                     10
#define CSU_AMK_L5_n_MASK                        0x800u
#define CSU_AMK_L5_n_SHIFT                       11
#define CSU_AMK_AMK6_n_MASK                      0x1000u
#define CSU_AMK_AMK6_n_SHIFT                     12
#define CSU_AMK_L6_n_MASK                        0x2000u
#define CSU_AMK_L6_n_SHIFT                       13
#define CSU_AMK_AMK7_n_MASK                      0x4000u
#define CSU_AMK_AMK7_n_SHIFT                     14
#define CSU_AMK_L7_n_MASK                        0x8000u
#define CSU_AMK_L7_n_SHIFT                       15
#define CSU_AMK_AMK8_n_MASK                      0x10000u
#define CSU_AMK_AMK8_n_SHIFT                     16
#define CSU_AMK_L8_n_MASK                        0x20000u
#define CSU_AMK_L8_n_SHIFT                       17
#define CSU_AMK_AMK9_n_MASK                      0x40000u
#define CSU_AMK_AMK9_n_SHIFT                     18
#define CSU_AMK_L9_n_MASK                        0x80000u
#define CSU_AMK_L9_n_SHIFT                       19
#define CSU_AMK_AMK10_n_MASK                     0x100000u
#define CSU_AMK_AMK10_n_SHIFT                    20
#define CSU_AMK_L10_n_MASK                       0x200000u
#define CSU_AMK_L10_n_SHIFT                      21
#define CSU_AMK_AMK11_n_MASK                     0x400000u
#define CSU_AMK_AMK11_n_SHIFT                    22
#define CSU_AMK_L11_n_MASK                       0x800000u
#define CSU_AMK_L11_n_SHIFT                      23
#define CSU_AMK_AMK12_n_MASK                     0x1000000u
#define CSU_AMK_AMK12_n_SHIFT                    24
#define CSU_AMK_L12_n_MASK                       0x2000000u
#define CSU_AMK_L12_n_SHIFT                      25
#define CSU_AMK_AMK13_n_MASK                     0x4000000u
#define CSU_AMK_AMK13_n_SHIFT                    26
#define CSU_AMK_L13_n_MASK                       0x8000000u
#define CSU_AMK_L13_n_SHIFT                      27
#define CSU_AMK_AMK14_n_MASK                     0x10000000u
#define CSU_AMK_AMK14_n_SHIFT                    28
#define CSU_AMK_L14_n_MASK                       0x20000000u
#define CSU_AMK_L14_n_SHIFT                      29
#define CSU_AMK_AMK15_n_MASK                     0x40000000u
#define CSU_AMK_AMK15_n_SHIFT                    30
#define CSU_AMK_L15_n_MASK                       0x80000000u
#define CSU_AMK_L15_n_SHIFT                      31
/* AROUT Bit Fields */
#define CSU_AROUT_AROUT_n_MASK                   0x7FFFFFFFu
#define CSU_AROUT_AROUT_n_SHIFT                  0
#define CSU_AROUT_AROUT_n(x)                     (((uint32_t)(((uint32_t)(x))<<CSU_AROUT_AROUT_n_SHIFT))&CSU_AROUT_AROUT_n_MASK)
#define CSU_AROUT_L_n_MASK                       0x80000000u
#define CSU_AROUT_L_n_SHIFT                      31
/* ASOFT Bit Fields */
#define CSU_ASOFT_NALARM_MASK                    0x3u
#define CSU_ASOFT_NALARM_SHIFT                   0
#define CSU_ASOFT_NALARM(x)                      (((uint8_t)(((uint8_t)(x))<<CSU_ASOFT_NALARM_SHIFT))&CSU_ASOFT_NALARM_MASK)
#define CSU_ASOFT_SALARM_MASK                    0xCu
#define CSU_ASOFT_SALARM_SHIFT                   2
#define CSU_ASOFT_SALARM(x)                      (((uint8_t)(((uint8_t)(x))<<CSU_ASOFT_SALARM_SHIFT))&CSU_ASOFT_SALARM_MASK)
#define CSU_ASOFT_L_MASK                         0x10u
#define CSU_ASOFT_L_SHIFT                        4
/* ACOUNTER Bit Fields */
#define CSU_ACOUNTER_ACOUNTER_MASK               0x7FFFu
#define CSU_ACOUNTER_ACOUNTER_SHIFT              0
#define CSU_ACOUNTER_ACOUNTER(x)                 (((uint16_t)(((uint16_t)(x))<<CSU_ACOUNTER_ACOUNTER_SHIFT))&CSU_ACOUNTER_ACOUNTER_MASK)
#define CSU_ACOUNTER_L_MASK                      0x8000u
#define CSU_ACOUNTER_L_SHIFT                     15
/* ACONTROL Bit Fields */
#define CSU_ACONTROL_CE_MASK                     0x1u
#define CSU_ACONTROL_CE_SHIFT                    0
#define CSU_ACONTROL_SC_MASK                     0x2u
#define CSU_ACONTROL_SC_SHIFT                    1
#define CSU_ACONTROL_L_MASK                      0x4u
#define CSU_ACONTROL_L_SHIFT                     2
/* HPC Bit Fields */
#define CSU_HPC_HPC0_n_MASK                      0x1u
#define CSU_HPC_HPC0_n_SHIFT                     0
#define CSU_HPC_L0_n_MASK                        0x2u
#define CSU_HPC_L0_n_SHIFT                       1
#define CSU_HPC_HPC1_n_MASK                      0x4u
#define CSU_HPC_HPC1_n_SHIFT                     2
#define CSU_HPC_L1_n_MASK                        0x8u
#define CSU_HPC_L1_n_SHIFT                       3
#define CSU_HPC_HPC2_n_MASK                      0x10u
#define CSU_HPC_HPC2_n_SHIFT                     4
#define CSU_HPC_L2_n_MASK                        0x20u
#define CSU_HPC_L2_n_SHIFT                       5
#define CSU_HPC_HPC3_n_MASK                      0x40u
#define CSU_HPC_HPC3_n_SHIFT                     6
#define CSU_HPC_L3_n_MASK                        0x80u
#define CSU_HPC_L3_n_SHIFT                       7
#define CSU_HPC_HPC4_n_MASK                      0x100u
#define CSU_HPC_HPC4_n_SHIFT                     8
#define CSU_HPC_L4_n_MASK                        0x200u
#define CSU_HPC_L4_n_SHIFT                       9
#define CSU_HPC_HPC5_n_MASK                      0x400u
#define CSU_HPC_HPC5_n_SHIFT                     10
#define CSU_HPC_L5_n_MASK                        0x800u
#define CSU_HPC_L5_n_SHIFT                       11
#define CSU_HPC_HPC6_n_MASK                      0x1000u
#define CSU_HPC_HPC6_n_SHIFT                     12
#define CSU_HPC_L6_n_MASK                        0x2000u
#define CSU_HPC_L6_n_SHIFT                       13
#define CSU_HPC_HPC7_n_MASK                      0x4000u
#define CSU_HPC_HPC7_n_SHIFT                     14
#define CSU_HPC_L7_n_MASK                        0x8000u
#define CSU_HPC_L7_n_SHIFT                       15
#define CSU_HPC_HPC8_n_MASK                      0x10000u
#define CSU_HPC_HPC8_n_SHIFT                     16
#define CSU_HPC_L8_n_MASK                        0x20000u
#define CSU_HPC_L8_n_SHIFT                       17
#define CSU_HPC_HPC9_n_MASK                      0x40000u
#define CSU_HPC_HPC9_n_SHIFT                     18
#define CSU_HPC_L9_n_MASK                        0x80000u
#define CSU_HPC_L9_n_SHIFT                       19
#define CSU_HPC_HPC10_n_MASK                     0x100000u
#define CSU_HPC_HPC10_n_SHIFT                    20
#define CSU_HPC_L10_n_MASK                       0x200000u
#define CSU_HPC_L10_n_SHIFT                      21
#define CSU_HPC_HPC11_n_MASK                     0x400000u
#define CSU_HPC_HPC11_n_SHIFT                    22
#define CSU_HPC_L11_n_MASK                       0x800000u
#define CSU_HPC_L11_n_SHIFT                      23
#define CSU_HPC_HPC12_n_MASK                     0x1000000u
#define CSU_HPC_HPC12_n_SHIFT                    24
#define CSU_HPC_L12_n_MASK                       0x2000000u
#define CSU_HPC_L12_n_SHIFT                      25
#define CSU_HPC_HPC13_n_MASK                     0x4000000u
#define CSU_HPC_HPC13_n_SHIFT                    26
#define CSU_HPC_L13_n_MASK                       0x8000000u
#define CSU_HPC_L13_n_SHIFT                      27
#define CSU_HPC_HPC14_n_MASK                     0x10000000u
#define CSU_HPC_HPC14_n_SHIFT                    28
#define CSU_HPC_L14_n_MASK                       0x20000000u
#define CSU_HPC_L14_n_SHIFT                      29
#define CSU_HPC_HPC15_n_MASK                     0x40000000u
#define CSU_HPC_HPC15_n_SHIFT                    30
#define CSU_HPC_L15_n_MASK                       0x80000000u
#define CSU_HPC_L15_n_SHIFT                      31
/* ISR Bit Fields */
#define CSU_ISR_IS0_n_MASK                       0x1u
#define CSU_ISR_IS0_n_SHIFT                      0
#define CSU_ISR_IS1_n_MASK                       0x4u
#define CSU_ISR_IS1_n_SHIFT                      2
#define CSU_ISR_IS2_n_MASK                       0x10u
#define CSU_ISR_IS2_n_SHIFT                      4
#define CSU_ISR_IS3_n_MASK                       0x40u
#define CSU_ISR_IS3_n_SHIFT                      6
#define CSU_ISR_IS4_n_MASK                       0x100u
#define CSU_ISR_IS4_n_SHIFT                      8
#define CSU_ISR_IS5_n_MASK                       0x400u
#define CSU_ISR_IS5_n_SHIFT                      10
#define CSU_ISR_IS6_n_MASK                       0x1000u
#define CSU_ISR_IS6_n_SHIFT                      12
#define CSU_ISR_IS7_n_MASK                       0x4000u
#define CSU_ISR_IS7_n_SHIFT                      14
#define CSU_ISR_IS8_n_MASK                       0x10000u
#define CSU_ISR_IS8_n_SHIFT                      16
#define CSU_ISR_IS9_n_MASK                       0x40000u
#define CSU_ISR_IS9_n_SHIFT                      18
#define CSU_ISR_IS10_n_MASK                      0x100000u
#define CSU_ISR_IS10_n_SHIFT                     20
#define CSU_ISR_IS11_n_MASK                      0x400000u
#define CSU_ISR_IS11_n_SHIFT                     22
#define CSU_ISR_IS12_n_MASK                      0x1000000u
#define CSU_ISR_IS12_n_SHIFT                     24
#define CSU_ISR_IS13_n_MASK                      0x4000000u
#define CSU_ISR_IS13_n_SHIFT                     26
#define CSU_ISR_IS14_n_MASK                      0x10000000u
#define CSU_ISR_IS14_n_SHIFT                     28
#define CSU_ISR_IS15_n_MASK                      0x40000000u
#define CSU_ISR_IS15_n_SHIFT                     30

/*!
 * @}
 */ /* end of group CSU_Register_Masks */


/* CSU - Peripheral instance base addresses */
/** Peripheral CSU base pointer */
#define CSU_BASE_PTR                             ((CSU_MemMapPtr)0x40017000u)
/** Array initializer of CSU peripheral base pointers */
#define CSU_BASE_PTRS                            { CSU_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- CSU - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CSU_Register_Accessor_Macros CSU - Register accessor macros
 * @{
 */


/* CSU - Register instance definitions */
/* CSU */
#define CSU_CSL0                                 CSU_CSL_REG(CSU_BASE_PTR,0)
#define CSU_CSL1                                 CSU_CSL_REG(CSU_BASE_PTR,1)
#define CSU_CSL2                                 CSU_CSL_REG(CSU_BASE_PTR,2)
#define CSU_CSL3                                 CSU_CSL_REG(CSU_BASE_PTR,3)
#define CSU_CSL4                                 CSU_CSL_REG(CSU_BASE_PTR,4)
#define CSU_CSL5                                 CSU_CSL_REG(CSU_BASE_PTR,5)
#define CSU_CSL6                                 CSU_CSL_REG(CSU_BASE_PTR,6)
#define CSU_CSL7                                 CSU_CSL_REG(CSU_BASE_PTR,7)
#define CSU_CSL8                                 CSU_CSL_REG(CSU_BASE_PTR,8)
#define CSU_CSL9                                 CSU_CSL_REG(CSU_BASE_PTR,9)
#define CSU_CSL10                                CSU_CSL_REG(CSU_BASE_PTR,10)
#define CSU_CSL11                                CSU_CSL_REG(CSU_BASE_PTR,11)
#define CSU_CSL12                                CSU_CSL_REG(CSU_BASE_PTR,12)
#define CSU_CSL13                                CSU_CSL_REG(CSU_BASE_PTR,13)
#define CSU_CSL14                                CSU_CSL_REG(CSU_BASE_PTR,14)
#define CSU_CSL15                                CSU_CSL_REG(CSU_BASE_PTR,15)
#define CSU_CSL16                                CSU_CSL_REG(CSU_BASE_PTR,16)
#define CSU_CSL17                                CSU_CSL_REG(CSU_BASE_PTR,17)
#define CSU_CSL18                                CSU_CSL_REG(CSU_BASE_PTR,18)
#define CSU_CSL19                                CSU_CSL_REG(CSU_BASE_PTR,19)
#define CSU_CSL20                                CSU_CSL_REG(CSU_BASE_PTR,20)
#define CSU_CSL21                                CSU_CSL_REG(CSU_BASE_PTR,21)
#define CSU_CSL22                                CSU_CSL_REG(CSU_BASE_PTR,22)
#define CSU_CSL23                                CSU_CSL_REG(CSU_BASE_PTR,23)
#define CSU_CSL24                                CSU_CSL_REG(CSU_BASE_PTR,24)
#define CSU_CSL25                                CSU_CSL_REG(CSU_BASE_PTR,25)
#define CSU_CSL26                                CSU_CSL_REG(CSU_BASE_PTR,26)
#define CSU_CSL27                                CSU_CSL_REG(CSU_BASE_PTR,27)
#define CSU_CSL28                                CSU_CSL_REG(CSU_BASE_PTR,28)
#define CSU_CSL29                                CSU_CSL_REG(CSU_BASE_PTR,29)
#define CSU_CSL30                                CSU_CSL_REG(CSU_BASE_PTR,30)
#define CSU_CSL31                                CSU_CSL_REG(CSU_BASE_PTR,31)
#define CSU_CSL32                                CSU_CSL_REG(CSU_BASE_PTR,32)
#define CSU_CSL33                                CSU_CSL_REG(CSU_BASE_PTR,33)
#define CSU_CSL34                                CSU_CSL_REG(CSU_BASE_PTR,34)
#define CSU_CSL35                                CSU_CSL_REG(CSU_BASE_PTR,35)
#define CSU_CSL36                                CSU_CSL_REG(CSU_BASE_PTR,36)
#define CSU_CSL37                                CSU_CSL_REG(CSU_BASE_PTR,37)
#define CSU_CSL38                                CSU_CSL_REG(CSU_BASE_PTR,38)
#define CSU_CSL39                                CSU_CSL_REG(CSU_BASE_PTR,39)
#define CSU_CSL40                                CSU_CSL_REG(CSU_BASE_PTR,40)
#define CSU_CSL41                                CSU_CSL_REG(CSU_BASE_PTR,41)
#define CSU_CSL42                                CSU_CSL_REG(CSU_BASE_PTR,42)
#define CSU_CSL43                                CSU_CSL_REG(CSU_BASE_PTR,43)
#define CSU_CSL44                                CSU_CSL_REG(CSU_BASE_PTR,44)
#define CSU_CSL45                                CSU_CSL_REG(CSU_BASE_PTR,45)
#define CSU_CSL46                                CSU_CSL_REG(CSU_BASE_PTR,46)
#define CSU_CSL47                                CSU_CSL_REG(CSU_BASE_PTR,47)
#define CSU_CSL48                                CSU_CSL_REG(CSU_BASE_PTR,48)
#define CSU_CSL49                                CSU_CSL_REG(CSU_BASE_PTR,49)
#define CSU_CSL50                                CSU_CSL_REG(CSU_BASE_PTR,50)
#define CSU_CSL51                                CSU_CSL_REG(CSU_BASE_PTR,51)
#define CSU_CSL52                                CSU_CSL_REG(CSU_BASE_PTR,52)
#define CSU_CSL53                                CSU_CSL_REG(CSU_BASE_PTR,53)
#define CSU_CSL54                                CSU_CSL_REG(CSU_BASE_PTR,54)
#define CSU_CSL55                                CSU_CSL_REG(CSU_BASE_PTR,55)
#define CSU_CSL56                                CSU_CSL_REG(CSU_BASE_PTR,56)
#define CSU_CSL57                                CSU_CSL_REG(CSU_BASE_PTR,57)
#define CSU_CSL58                                CSU_CSL_REG(CSU_BASE_PTR,58)
#define CSU_CSL59                                CSU_CSL_REG(CSU_BASE_PTR,59)
#define CSU_CSL60                                CSU_CSL_REG(CSU_BASE_PTR,60)
#define CSU_CSL61                                CSU_CSL_REG(CSU_BASE_PTR,61)
#define CSU_CSL62                                CSU_CSL_REG(CSU_BASE_PTR,62)
#define CSU_CSL63                                CSU_CSL_REG(CSU_BASE_PTR,63)
#define CSU_HP0                                  CSU_HP_REG(CSU_BASE_PTR,0)
#define CSU_HP1                                  CSU_HP_REG(CSU_BASE_PTR,1)
#define CSU_SA0                                  CSU_SA_REG(CSU_BASE_PTR,0)
#define CSU_SA1                                  CSU_SA_REG(CSU_BASE_PTR,1)
#define CSU_AMK0                                 CSU_AMK_REG(CSU_BASE_PTR,0)
#define CSU_AROUT0                               CSU_AROUT_REG(CSU_BASE_PTR,0)
#define CSU_AROUT1                               CSU_AROUT_REG(CSU_BASE_PTR,1)
#define CSU_AROUT2                               CSU_AROUT_REG(CSU_BASE_PTR,2)
#define CSU_AROUT3                               CSU_AROUT_REG(CSU_BASE_PTR,3)
#define CSU_AROUT4                               CSU_AROUT_REG(CSU_BASE_PTR,4)
#define CSU_AROUT5                               CSU_AROUT_REG(CSU_BASE_PTR,5)
#define CSU_AROUT6                               CSU_AROUT_REG(CSU_BASE_PTR,6)
#define CSU_AROUT7                               CSU_AROUT_REG(CSU_BASE_PTR,7)
#define CSU_AROUT8                               CSU_AROUT_REG(CSU_BASE_PTR,8)
#define CSU_AROUT9                               CSU_AROUT_REG(CSU_BASE_PTR,9)
#define CSU_AROUT10                              CSU_AROUT_REG(CSU_BASE_PTR,10)
#define CSU_AROUT11                              CSU_AROUT_REG(CSU_BASE_PTR,11)
#define CSU_AROUT12                              CSU_AROUT_REG(CSU_BASE_PTR,12)
#define CSU_ASOFT                                CSU_ASOFT_REG(CSU_BASE_PTR)
#define CSU_ACOUNTER                             CSU_ACOUNTER_REG(CSU_BASE_PTR)
#define CSU_ACONTROL                             CSU_ACONTROL_REG(CSU_BASE_PTR)
#define CSU_HPC0                                 CSU_HPC_REG(CSU_BASE_PTR,0)
#define CSU_HPC1                                 CSU_HPC_REG(CSU_BASE_PTR,1)
#define CSU_ISR0                                 CSU_ISR_REG(CSU_BASE_PTR,0)

/* CSU - Register array accessors */
#define CSU_CSL(index)                           CSU_CSL_REG(CSU_BASE_PTR,index)
#define CSU_HP(index)                            CSU_HP_REG(CSU_BASE_PTR,index)
#define CSU_SA(index)                            CSU_SA_REG(CSU_BASE_PTR,index)
#define CSU_AMK(index)                           CSU_AMK_REG(CSU_BASE_PTR,index)
#define CSU_AROUT(index)                         CSU_AROUT_REG(CSU_BASE_PTR,index)
#define CSU_HPC(index)                           CSU_HPC_REG(CSU_BASE_PTR,index)
#define CSU_ISR(index)                           CSU_ISR_REG(CSU_BASE_PTR,index)

/*!
 * @}
 */ /* end of group CSU_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group CSU_Peripheral */


/* ----------------------------------------------------------------------------
   -- DAC
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DAC_Peripheral DAC
 * @{
 */

/** DAC - Peripheral register structure */
typedef struct DAC_MemMap {
  union {                                          /* offset: 0x0 */
    uint16_t DAT16[16];                              /**< DAC Data Register (16-bit), array offset: 0x0, array step: 0x2 */
    uint32_t DAT[8];                                 /**< DAC Data Register, array offset: 0x0, array step: 0x4 */
  };
  uint32_t STATCTRL;                               /**< DAC Status and Control Register, offset: 0x20 */
} volatile *DAC_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- DAC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DAC_Register_Accessor_Macros DAC - Register accessor macros
 * @{
 */


/* DAC - Register accessors */
#define DAC_DAT16_REG(base,index2)               ((base)->DAT16[index2])
#define DAC_DAT_REG(base,index2)                 ((base)->DAT[index2])
#define DAC_STATCTRL_REG(base)                   ((base)->STATCTRL)

/*!
 * @}
 */ /* end of group DAC_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- DAC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DAC_Register_Masks DAC Register Masks
 * @{
 */

/* DAT16 Bit Fields */
#define DAC_DAT16_DATA_MASK                      0xFFFu
#define DAC_DAT16_DATA_SHIFT                     0
#define DAC_DAT16_DATA(x)                        (((uint16_t)(((uint16_t)(x))<<DAC_DAT16_DATA_SHIFT))&DAC_DAT16_DATA_MASK)
/* DAT Bit Fields */
#define DAC_DAT_DATA0_MASK                       0xFFFu
#define DAC_DAT_DATA0_SHIFT                      0
#define DAC_DAT_DATA0(x)                         (((uint32_t)(((uint32_t)(x))<<DAC_DAT_DATA0_SHIFT))&DAC_DAT_DATA0_MASK)
#define DAC_DAT_DATA1_MASK                       0xFFF0000u
#define DAC_DAT_DATA1_SHIFT                      16
#define DAC_DAT_DATA1(x)                         (((uint32_t)(((uint32_t)(x))<<DAC_DAT_DATA1_SHIFT))&DAC_DAT_DATA1_MASK)
/* STATCTRL Bit Fields */
#define DAC_STATCTRL_DACBFRPBF_MASK              0x1u
#define DAC_STATCTRL_DACBFRPBF_SHIFT             0
#define DAC_STATCTRL_DACBFRPTF_MASK              0x2u
#define DAC_STATCTRL_DACBFRPTF_SHIFT             1
#define DAC_STATCTRL_DACBFWMF_MASK               0x4u
#define DAC_STATCTRL_DACBFWMF_SHIFT              2
#define DAC_STATCTRL_DACBBIEN_MASK               0x100u
#define DAC_STATCTRL_DACBBIEN_SHIFT              8
#define DAC_STATCTRL_DACBTIEN_MASK               0x200u
#define DAC_STATCTRL_DACBTIEN_SHIFT              9
#define DAC_STATCTRL_DACBWIEN_MASK               0x400u
#define DAC_STATCTRL_DACBWIEN_SHIFT              10
#define DAC_STATCTRL_LPEN_MASK                   0x800u
#define DAC_STATCTRL_LPEN_SHIFT                  11
#define DAC_STATCTRL_DACSWTRG_MASK               0x1000u
#define DAC_STATCTRL_DACSWTRG_SHIFT              12
#define DAC_STATCTRL_DACTRGSEL_MASK              0x2000u
#define DAC_STATCTRL_DACTRGSEL_SHIFT             13
#define DAC_STATCTRL_DACRFS_MASK                 0x4000u
#define DAC_STATCTRL_DACRFS_SHIFT                14
#define DAC_STATCTRL_DACEN_MASK                  0x8000u
#define DAC_STATCTRL_DACEN_SHIFT                 15
#define DAC_STATCTRL_DACBFEN_MASK                0x10000u
#define DAC_STATCTRL_DACBFEN_SHIFT               16
#define DAC_STATCTRL_DACBFMD_MASK                0x60000u
#define DAC_STATCTRL_DACBFMD_SHIFT               17
#define DAC_STATCTRL_DACBFMD(x)                  (((uint32_t)(((uint32_t)(x))<<DAC_STATCTRL_DACBFMD_SHIFT))&DAC_STATCTRL_DACBFMD_MASK)
#define DAC_STATCTRL_DACBFWM_MASK                0x180000u
#define DAC_STATCTRL_DACBFWM_SHIFT               19
#define DAC_STATCTRL_DACBFWM(x)                  (((uint32_t)(((uint32_t)(x))<<DAC_STATCTRL_DACBFWM_SHIFT))&DAC_STATCTRL_DACBFWM_MASK)
#define DAC_STATCTRL_DMAEN_MASK                  0x800000u
#define DAC_STATCTRL_DMAEN_SHIFT                 23
#define DAC_STATCTRL_DACBFUP_MASK                0xF000000u
#define DAC_STATCTRL_DACBFUP_SHIFT               24
#define DAC_STATCTRL_DACBFUP(x)                  (((uint32_t)(((uint32_t)(x))<<DAC_STATCTRL_DACBFUP_SHIFT))&DAC_STATCTRL_DACBFUP_MASK)
#define DAC_STATCTRL_DACBFRP_MASK                0xF0000000u
#define DAC_STATCTRL_DACBFRP_SHIFT               28
#define DAC_STATCTRL_DACBFRP(x)                  (((uint32_t)(((uint32_t)(x))<<DAC_STATCTRL_DACBFRP_SHIFT))&DAC_STATCTRL_DACBFRP_MASK)

/*!
 * @}
 */ /* end of group DAC_Register_Masks */


/* DAC - Peripheral instance base addresses */
/** Peripheral DAC0 base pointer */
#define DAC0_BASE_PTR                            ((DAC_MemMapPtr)0x400CC000u)
/** Peripheral DAC1 base pointer */
#define DAC1_BASE_PTR                            ((DAC_MemMapPtr)0x400CD000u)
/** Array initializer of DAC peripheral base pointers */
#define DAC_BASE_PTRS                            { DAC0_BASE_PTR, DAC1_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- DAC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DAC_Register_Accessor_Macros DAC - Register accessor macros
 * @{
 */


/* DAC - Register instance definitions */
/* DAC0 */
#define DAC0_DAT0                                DAC_DAT_REG(DAC0_BASE_PTR,0)
#define DAC0_DAT0L                               DAC_DAT16_REG(DAC0_BASE_PTR,0)
#define DAC0_DAT0H                               DAC_DAT16_REG(DAC0_BASE_PTR,1)
#define DAC0_DAT1                                DAC_DAT_REG(DAC0_BASE_PTR,1)
#define DAC0_DAT1L                               DAC_DAT16_REG(DAC0_BASE_PTR,2)
#define DAC0_DAT1H                               DAC_DAT16_REG(DAC0_BASE_PTR,3)
#define DAC0_DAT2                                DAC_DAT_REG(DAC0_BASE_PTR,2)
#define DAC0_DAT2L                               DAC_DAT16_REG(DAC0_BASE_PTR,4)
#define DAC0_DAT2H                               DAC_DAT16_REG(DAC0_BASE_PTR,5)
#define DAC0_DAT3                                DAC_DAT_REG(DAC0_BASE_PTR,3)
#define DAC0_DAT3L                               DAC_DAT16_REG(DAC0_BASE_PTR,6)
#define DAC0_DAT3H                               DAC_DAT16_REG(DAC0_BASE_PTR,7)
#define DAC0_DAT4                                DAC_DAT_REG(DAC0_BASE_PTR,4)
#define DAC0_DAT4L                               DAC_DAT16_REG(DAC0_BASE_PTR,8)
#define DAC0_DAT4H                               DAC_DAT16_REG(DAC0_BASE_PTR,9)
#define DAC0_DAT5                                DAC_DAT_REG(DAC0_BASE_PTR,5)
#define DAC0_DAT5L                               DAC_DAT16_REG(DAC0_BASE_PTR,10)
#define DAC0_DAT5H                               DAC_DAT16_REG(DAC0_BASE_PTR,11)
#define DAC0_DAT6                                DAC_DAT_REG(DAC0_BASE_PTR,6)
#define DAC0_DAT6L                               DAC_DAT16_REG(DAC0_BASE_PTR,12)
#define DAC0_DAT6H                               DAC_DAT16_REG(DAC0_BASE_PTR,13)
#define DAC0_DAT7                                DAC_DAT_REG(DAC0_BASE_PTR,7)
#define DAC0_DAT7L                               DAC_DAT16_REG(DAC0_BASE_PTR,14)
#define DAC0_DAT7H                               DAC_DAT16_REG(DAC0_BASE_PTR,15)
#define DAC0_STATCTRL                            DAC_STATCTRL_REG(DAC0_BASE_PTR)
/* DAC1 */
#define DAC1_DAT0                                DAC_DAT_REG(DAC1_BASE_PTR,0)
#define DAC1_DAT0L                               DAC_DAT16_REG(DAC1_BASE_PTR,0)
#define DAC1_DAT0H                               DAC_DAT16_REG(DAC1_BASE_PTR,1)
#define DAC1_DAT1                                DAC_DAT_REG(DAC1_BASE_PTR,1)
#define DAC1_DAT1L                               DAC_DAT16_REG(DAC1_BASE_PTR,2)
#define DAC1_DAT1H                               DAC_DAT16_REG(DAC1_BASE_PTR,3)
#define DAC1_DAT2                                DAC_DAT_REG(DAC1_BASE_PTR,2)
#define DAC1_DAT2L                               DAC_DAT16_REG(DAC1_BASE_PTR,4)
#define DAC1_DAT2H                               DAC_DAT16_REG(DAC1_BASE_PTR,5)
#define DAC1_DAT3                                DAC_DAT_REG(DAC1_BASE_PTR,3)
#define DAC1_DAT3L                               DAC_DAT16_REG(DAC1_BASE_PTR,6)
#define DAC1_DAT3H                               DAC_DAT16_REG(DAC1_BASE_PTR,7)
#define DAC1_DAT4                                DAC_DAT_REG(DAC1_BASE_PTR,4)
#define DAC1_DAT4L                               DAC_DAT16_REG(DAC1_BASE_PTR,8)
#define DAC1_DAT4H                               DAC_DAT16_REG(DAC1_BASE_PTR,9)
#define DAC1_DAT5                                DAC_DAT_REG(DAC1_BASE_PTR,5)
#define DAC1_DAT5L                               DAC_DAT16_REG(DAC1_BASE_PTR,10)
#define DAC1_DAT5H                               DAC_DAT16_REG(DAC1_BASE_PTR,11)
#define DAC1_DAT6                                DAC_DAT_REG(DAC1_BASE_PTR,6)
#define DAC1_DAT6L                               DAC_DAT16_REG(DAC1_BASE_PTR,12)
#define DAC1_DAT6H                               DAC_DAT16_REG(DAC1_BASE_PTR,13)
#define DAC1_DAT7                                DAC_DAT_REG(DAC1_BASE_PTR,7)
#define DAC1_DAT7L                               DAC_DAT16_REG(DAC1_BASE_PTR,14)
#define DAC1_DAT7H                               DAC_DAT16_REG(DAC1_BASE_PTR,15)
#define DAC1_STATCTRL                            DAC_STATCTRL_REG(DAC1_BASE_PTR)

/* DAC - Register array accessors */
#define DAC0_DAT16(index2)                       DAC_DAT16_REG(DAC0_BASE_PTR,index2)
#define DAC1_DAT16(index2)                       DAC_DAT16_REG(DAC1_BASE_PTR,index2)
#define DAC0_DAT(index2)                         DAC_DAT_REG(DAC0_BASE_PTR,index2)
#define DAC1_DAT(index2)                         DAC_DAT_REG(DAC1_BASE_PTR,index2)

/*!
 * @}
 */ /* end of group DAC_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group DAC_Peripheral */

/* ----------------------------------------------------------------------------
   -- DMA
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Peripheral DMA
 * @{
 */

/** DMA - Peripheral register structure */
typedef struct DMA_MemMap {
  uint32_t CR;                                     /**< Control Register, offset: 0x0 */
  uint32_t ES;                                     /**< Error Status Register, offset: 0x4 */
  uint8_t RESERVED_0[4];
  uint32_t ERQ;                                    /**< Enable Request Register, offset: 0xC */
  uint8_t RESERVED_1[4];
  uint32_t EEI;                                    /**< Enable Error Interrupt Register, offset: 0x14 */
  uint8_t CEEI;                                    /**< Clear Enable Error Interrupt Register, offset: 0x18 */
  uint8_t SEEI;                                    /**< Set Enable Error Interrupt Register, offset: 0x19 */
  uint8_t CERQ;                                    /**< Clear Enable Request Register, offset: 0x1A */
  uint8_t SERQ;                                    /**< Set Enable Request Register, offset: 0x1B */
  uint8_t CDNE;                                    /**< Clear DONE Status Bit Register, offset: 0x1C */
  uint8_t SSRT;                                    /**< Set START Bit Register, offset: 0x1D */
  uint8_t CERR;                                    /**< Clear Error Register, offset: 0x1E */
  uint8_t CINT;                                    /**< Clear Interrupt Request Register, offset: 0x1F */
  uint8_t RESERVED_2[4];
  uint32_t INT;                                    /**< Interrupt Request Register, offset: 0x24 */
  uint8_t RESERVED_3[4];
  uint32_t ERR;                                    /**< Error Register, offset: 0x2C */
  uint8_t RESERVED_4[4];
  uint32_t HRS;                                    /**< Hardware Request Status Register, offset: 0x34 */
  uint8_t RESERVED_5[200];
  uint8_t DCHPRI3;                                 /**< Channel n Priority Register, offset: 0x100 */
  uint8_t DCHPRI2;                                 /**< Channel n Priority Register, offset: 0x101 */
  uint8_t DCHPRI1;                                 /**< Channel n Priority Register, offset: 0x102 */
  uint8_t DCHPRI0;                                 /**< Channel n Priority Register, offset: 0x103 */
  uint8_t DCHPRI7;                                 /**< Channel n Priority Register, offset: 0x104 */
  uint8_t DCHPRI6;                                 /**< Channel n Priority Register, offset: 0x105 */
  uint8_t DCHPRI5;                                 /**< Channel n Priority Register, offset: 0x106 */
  uint8_t DCHPRI4;                                 /**< Channel n Priority Register, offset: 0x107 */
  uint8_t DCHPRI11;                                /**< Channel n Priority Register, offset: 0x108 */
  uint8_t DCHPRI10;                                /**< Channel n Priority Register, offset: 0x109 */
  uint8_t DCHPRI9;                                 /**< Channel n Priority Register, offset: 0x10A */
  uint8_t DCHPRI8;                                 /**< Channel n Priority Register, offset: 0x10B */
  uint8_t DCHPRI15;                                /**< Channel n Priority Register, offset: 0x10C */
  uint8_t DCHPRI14;                                /**< Channel n Priority Register, offset: 0x10D */
  uint8_t DCHPRI13;                                /**< Channel n Priority Register, offset: 0x10E */
  uint8_t DCHPRI12;                                /**< Channel n Priority Register, offset: 0x10F */
  uint8_t DCHPRI19;                                /**< Channel n Priority Register, offset: 0x110 */
  uint8_t DCHPRI18;                                /**< Channel n Priority Register, offset: 0x111 */
  uint8_t DCHPRI17;                                /**< Channel n Priority Register, offset: 0x112 */
  uint8_t DCHPRI16;                                /**< Channel n Priority Register, offset: 0x113 */
  uint8_t DCHPRI23;                                /**< Channel n Priority Register, offset: 0x114 */
  uint8_t DCHPRI22;                                /**< Channel n Priority Register, offset: 0x115 */
  uint8_t DCHPRI21;                                /**< Channel n Priority Register, offset: 0x116 */
  uint8_t DCHPRI20;                                /**< Channel n Priority Register, offset: 0x117 */
  uint8_t DCHPRI27;                                /**< Channel n Priority Register, offset: 0x118 */
  uint8_t DCHPRI26;                                /**< Channel n Priority Register, offset: 0x119 */
  uint8_t DCHPRI25;                                /**< Channel n Priority Register, offset: 0x11A */
  uint8_t DCHPRI24;                                /**< Channel n Priority Register, offset: 0x11B */
  uint8_t DCHPRI31;                                /**< Channel n Priority Register, offset: 0x11C */
  uint8_t DCHPRI30;                                /**< Channel n Priority Register, offset: 0x11D */
  uint8_t DCHPRI29;                                /**< Channel n Priority Register, offset: 0x11E */
  uint8_t DCHPRI28;                                /**< Channel n Priority Register, offset: 0x11F */
  uint8_t RESERVED_6[3808];
  struct {                                         /* offset: 0x1000, array step: 0x20 */
    uint32_t SADDR;                                  /**< TCD Source Address, array offset: 0x1000, array step: 0x20 */
    uint16_t SOFF;                                   /**< TCD Signed Source Address Offset, array offset: 0x1004, array step: 0x20 */
    uint16_t ATTR;                                   /**< TCD Transfer Attributes, array offset: 0x1006, array step: 0x20 */
    union {                                          /* offset: 0x1008, array step: 0x20 */
      uint32_t NBYTES_MLNO;                            /**< TCD Minor Byte Count (Minor Loop Disabled), array offset: 0x1008, array step: 0x20 */
      uint32_t NBYTES_MLOFFNO;                         /**< TCD Signed Minor Loop Offset (Minor Loop Enabled and Offset Disabled), array offset: 0x1008, array step: 0x20 */
      uint32_t NBYTES_MLOFFYES;                        /**< TCD Signed Minor Loop Offset (Minor Loop and Offset Enabled), array offset: 0x1008, array step: 0x20 */
    };
    uint32_t SLAST;                                  /**< TCD Last Source Address Adjustment, array offset: 0x100C, array step: 0x20 */
    uint32_t DADDR;                                  /**< TCD Destination Address, array offset: 0x1010, array step: 0x20 */
    uint16_t DOFF;                                   /**< TCD Signed Destination Address Offset, array offset: 0x1014, array step: 0x20 */
    union {                                          /* offset: 0x1016, array step: 0x20 */
      uint16_t CITER_ELINKNO;                          /**< TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x1016, array step: 0x20 */
      uint16_t CITER_ELINKYES;                         /**< TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x1016, array step: 0x20 */
    };
    uint32_t DLAST_SGA;                              /**< TCD Last Destination Address Adjustment/Scatter Gather Address, array offset: 0x1018, array step: 0x20 */
    uint16_t CSR;                                    /**< TCD Control and Status, array offset: 0x101C, array step: 0x20 */
    union {                                          /* offset: 0x101E, array step: 0x20 */
      uint16_t BITER_ELINKNO;                          /**< TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x101E, array step: 0x20 */
      uint16_t BITER_ELINKYES;                         /**< TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x101E, array step: 0x20 */
    };
  } TCD[32];
} volatile *DMA_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- DMA - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Register_Accessor_Macros DMA - Register accessor macros
 * @{
 */


/* DMA - Register accessors */
#define DMA_CR_REG(base)                         ((base)->CR)
#define DMA_ES_REG(base)                         ((base)->ES)
#define DMA_ERQ_REG(base)                        ((base)->ERQ)
#define DMA_EEI_REG(base)                        ((base)->EEI)
#define DMA_CEEI_REG(base)                       ((base)->CEEI)
#define DMA_SEEI_REG(base)                       ((base)->SEEI)
#define DMA_CERQ_REG(base)                       ((base)->CERQ)
#define DMA_SERQ_REG(base)                       ((base)->SERQ)
#define DMA_CDNE_REG(base)                       ((base)->CDNE)
#define DMA_SSRT_REG(base)                       ((base)->SSRT)
#define DMA_CERR_REG(base)                       ((base)->CERR)
#define DMA_CINT_REG(base)                       ((base)->CINT)
#define DMA_INT_REG(base)                        ((base)->INT)
#define DMA_ERR_REG(base)                        ((base)->ERR)
#define DMA_HRS_REG(base)                        ((base)->HRS)
#define DMA_DCHPRI3_REG(base)                    ((base)->DCHPRI3)
#define DMA_DCHPRI2_REG(base)                    ((base)->DCHPRI2)
#define DMA_DCHPRI1_REG(base)                    ((base)->DCHPRI1)
#define DMA_DCHPRI0_REG(base)                    ((base)->DCHPRI0)
#define DMA_DCHPRI7_REG(base)                    ((base)->DCHPRI7)
#define DMA_DCHPRI6_REG(base)                    ((base)->DCHPRI6)
#define DMA_DCHPRI5_REG(base)                    ((base)->DCHPRI5)
#define DMA_DCHPRI4_REG(base)                    ((base)->DCHPRI4)
#define DMA_DCHPRI11_REG(base)                   ((base)->DCHPRI11)
#define DMA_DCHPRI10_REG(base)                   ((base)->DCHPRI10)
#define DMA_DCHPRI9_REG(base)                    ((base)->DCHPRI9)
#define DMA_DCHPRI8_REG(base)                    ((base)->DCHPRI8)
#define DMA_DCHPRI15_REG(base)                   ((base)->DCHPRI15)
#define DMA_DCHPRI14_REG(base)                   ((base)->DCHPRI14)
#define DMA_DCHPRI13_REG(base)                   ((base)->DCHPRI13)
#define DMA_DCHPRI12_REG(base)                   ((base)->DCHPRI12)
#define DMA_DCHPRI19_REG(base)                   ((base)->DCHPRI19)
#define DMA_DCHPRI18_REG(base)                   ((base)->DCHPRI18)
#define DMA_DCHPRI17_REG(base)                   ((base)->DCHPRI17)
#define DMA_DCHPRI16_REG(base)                   ((base)->DCHPRI16)
#define DMA_DCHPRI23_REG(base)                   ((base)->DCHPRI23)
#define DMA_DCHPRI22_REG(base)                   ((base)->DCHPRI22)
#define DMA_DCHPRI21_REG(base)                   ((base)->DCHPRI21)
#define DMA_DCHPRI20_REG(base)                   ((base)->DCHPRI20)
#define DMA_DCHPRI27_REG(base)                   ((base)->DCHPRI27)
#define DMA_DCHPRI26_REG(base)                   ((base)->DCHPRI26)
#define DMA_DCHPRI25_REG(base)                   ((base)->DCHPRI25)
#define DMA_DCHPRI24_REG(base)                   ((base)->DCHPRI24)
#define DMA_DCHPRI31_REG(base)                   ((base)->DCHPRI31)
#define DMA_DCHPRI30_REG(base)                   ((base)->DCHPRI30)
#define DMA_DCHPRI29_REG(base)                   ((base)->DCHPRI29)
#define DMA_DCHPRI28_REG(base)                   ((base)->DCHPRI28)
#define DMA_SADDR_REG(base,index)                ((base)->TCD[index].SADDR)
#define DMA_SOFF_REG(base,index)                 ((base)->TCD[index].SOFF)
#define DMA_ATTR_REG(base,index)                 ((base)->TCD[index].ATTR)
#define DMA_NBYTES_MLNO_REG(base,index)          ((base)->TCD[index].NBYTES_MLNO)
#define DMA_NBYTES_MLOFFNO_REG(base,index)       ((base)->TCD[index].NBYTES_MLOFFNO)
#define DMA_NBYTES_MLOFFYES_REG(base,index)      ((base)->TCD[index].NBYTES_MLOFFYES)
#define DMA_SLAST_REG(base,index)                ((base)->TCD[index].SLAST)
#define DMA_DADDR_REG(base,index)                ((base)->TCD[index].DADDR)
#define DMA_DOFF_REG(base,index)                 ((base)->TCD[index].DOFF)
#define DMA_CITER_ELINKNO_REG(base,index)        ((base)->TCD[index].CITER_ELINKNO)
#define DMA_CITER_ELINKYES_REG(base,index)       ((base)->TCD[index].CITER_ELINKYES)
#define DMA_DLAST_SGA_REG(base,index)            ((base)->TCD[index].DLAST_SGA)
#define DMA_CSR_REG(base,index)                  ((base)->TCD[index].CSR)
#define DMA_BITER_ELINKNO_REG(base,index)        ((base)->TCD[index].BITER_ELINKNO)
#define DMA_BITER_ELINKYES_REG(base,index)       ((base)->TCD[index].BITER_ELINKYES)

/*!
 * @}
 */ /* end of group DMA_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- DMA Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Register_Masks DMA Register Masks
 * @{
 */

/* CR Bit Fields */
#define DMA_CR_EDBG_MASK                         0x2u
#define DMA_CR_EDBG_SHIFT                        1
#define DMA_CR_ERCA_MASK                         0x4u
#define DMA_CR_ERCA_SHIFT                        2
#define DMA_CR_ERGA_MASK                         0x8u
#define DMA_CR_ERGA_SHIFT                        3
#define DMA_CR_HOE_MASK                          0x10u
#define DMA_CR_HOE_SHIFT                         4
#define DMA_CR_HALT_MASK                         0x20u
#define DMA_CR_HALT_SHIFT                        5
#define DMA_CR_CLM_MASK                          0x40u
#define DMA_CR_CLM_SHIFT                         6
#define DMA_CR_EMLM_MASK                         0x80u
#define DMA_CR_EMLM_SHIFT                        7
#define DMA_CR_GRP0PRI_MASK                      0x100u
#define DMA_CR_GRP0PRI_SHIFT                     8
#define DMA_CR_GRP1PRI_MASK                      0x400u
#define DMA_CR_GRP1PRI_SHIFT                     10
#define DMA_CR_ECX_MASK                          0x10000u
#define DMA_CR_ECX_SHIFT                         16
#define DMA_CR_CX_MASK                           0x20000u
#define DMA_CR_CX_SHIFT                          17
/* ES Bit Fields */
#define DMA_ES_DBE_MASK                          0x1u
#define DMA_ES_DBE_SHIFT                         0
#define DMA_ES_SBE_MASK                          0x2u
#define DMA_ES_SBE_SHIFT                         1
#define DMA_ES_SGE_MASK                          0x4u
#define DMA_ES_SGE_SHIFT                         2
#define DMA_ES_NCE_MASK                          0x8u
#define DMA_ES_NCE_SHIFT                         3
#define DMA_ES_DOE_MASK                          0x10u
#define DMA_ES_DOE_SHIFT                         4
#define DMA_ES_DAE_MASK                          0x20u
#define DMA_ES_DAE_SHIFT                         5
#define DMA_ES_SOE_MASK                          0x40u
#define DMA_ES_SOE_SHIFT                         6
#define DMA_ES_SAE_MASK                          0x80u
#define DMA_ES_SAE_SHIFT                         7
#define DMA_ES_ERRCHN_MASK                       0x1F00u
#define DMA_ES_ERRCHN_SHIFT                      8
#define DMA_ES_ERRCHN(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_ES_ERRCHN_SHIFT))&DMA_ES_ERRCHN_MASK)
#define DMA_ES_CPE_MASK                          0x4000u
#define DMA_ES_CPE_SHIFT                         14
#define DMA_ES_GPE_MASK                          0x8000u
#define DMA_ES_GPE_SHIFT                         15
#define DMA_ES_ECX_MASK                          0x10000u
#define DMA_ES_ECX_SHIFT                         16
#define DMA_ES_VLD_MASK                          0x80000000u
#define DMA_ES_VLD_SHIFT                         31
/* ERQ Bit Fields */
#define DMA_ERQ_ERQ0_MASK                        0x1u
#define DMA_ERQ_ERQ0_SHIFT                       0
#define DMA_ERQ_ERQ1_MASK                        0x2u
#define DMA_ERQ_ERQ1_SHIFT                       1
#define DMA_ERQ_ERQ2_MASK                        0x4u
#define DMA_ERQ_ERQ2_SHIFT                       2
#define DMA_ERQ_ERQ3_MASK                        0x8u
#define DMA_ERQ_ERQ3_SHIFT                       3
#define DMA_ERQ_ERQ4_MASK                        0x10u
#define DMA_ERQ_ERQ4_SHIFT                       4
#define DMA_ERQ_ERQ5_MASK                        0x20u
#define DMA_ERQ_ERQ5_SHIFT                       5
#define DMA_ERQ_ERQ6_MASK                        0x40u
#define DMA_ERQ_ERQ6_SHIFT                       6
#define DMA_ERQ_ERQ7_MASK                        0x80u
#define DMA_ERQ_ERQ7_SHIFT                       7
#define DMA_ERQ_ERQ8_MASK                        0x100u
#define DMA_ERQ_ERQ8_SHIFT                       8
#define DMA_ERQ_ERQ9_MASK                        0x200u
#define DMA_ERQ_ERQ9_SHIFT                       9
#define DMA_ERQ_ERQ10_MASK                       0x400u
#define DMA_ERQ_ERQ10_SHIFT                      10
#define DMA_ERQ_ERQ11_MASK                       0x800u
#define DMA_ERQ_ERQ11_SHIFT                      11
#define DMA_ERQ_ERQ12_MASK                       0x1000u
#define DMA_ERQ_ERQ12_SHIFT                      12
#define DMA_ERQ_ERQ13_MASK                       0x2000u
#define DMA_ERQ_ERQ13_SHIFT                      13
#define DMA_ERQ_ERQ14_MASK                       0x4000u
#define DMA_ERQ_ERQ14_SHIFT                      14
#define DMA_ERQ_ERQ15_MASK                       0x8000u
#define DMA_ERQ_ERQ15_SHIFT                      15
#define DMA_ERQ_ERQ16_MASK                       0x10000u
#define DMA_ERQ_ERQ16_SHIFT                      16
#define DMA_ERQ_ERQ17_MASK                       0x20000u
#define DMA_ERQ_ERQ17_SHIFT                      17
#define DMA_ERQ_ERQ18_MASK                       0x40000u
#define DMA_ERQ_ERQ18_SHIFT                      18
#define DMA_ERQ_ERQ19_MASK                       0x80000u
#define DMA_ERQ_ERQ19_SHIFT                      19
#define DMA_ERQ_ERQ20_MASK                       0x100000u
#define DMA_ERQ_ERQ20_SHIFT                      20
#define DMA_ERQ_ERQ21_MASK                       0x200000u
#define DMA_ERQ_ERQ21_SHIFT                      21
#define DMA_ERQ_ERQ22_MASK                       0x400000u
#define DMA_ERQ_ERQ22_SHIFT                      22
#define DMA_ERQ_ERQ23_MASK                       0x800000u
#define DMA_ERQ_ERQ23_SHIFT                      23
#define DMA_ERQ_ERQ24_MASK                       0x1000000u
#define DMA_ERQ_ERQ24_SHIFT                      24
#define DMA_ERQ_ERQ25_MASK                       0x2000000u
#define DMA_ERQ_ERQ25_SHIFT                      25
#define DMA_ERQ_ERQ26_MASK                       0x4000000u
#define DMA_ERQ_ERQ26_SHIFT                      26
#define DMA_ERQ_ERQ27_MASK                       0x8000000u
#define DMA_ERQ_ERQ27_SHIFT                      27
#define DMA_ERQ_ERQ28_MASK                       0x10000000u
#define DMA_ERQ_ERQ28_SHIFT                      28
#define DMA_ERQ_ERQ29_MASK                       0x20000000u
#define DMA_ERQ_ERQ29_SHIFT                      29
#define DMA_ERQ_ERQ30_MASK                       0x40000000u
#define DMA_ERQ_ERQ30_SHIFT                      30
#define DMA_ERQ_ERQ31_MASK                       0x80000000u
#define DMA_ERQ_ERQ31_SHIFT                      31
/* EEI Bit Fields */
#define DMA_EEI_EEI0_MASK                        0x1u
#define DMA_EEI_EEI0_SHIFT                       0
#define DMA_EEI_EEI1_MASK                        0x2u
#define DMA_EEI_EEI1_SHIFT                       1
#define DMA_EEI_EEI2_MASK                        0x4u
#define DMA_EEI_EEI2_SHIFT                       2
#define DMA_EEI_EEI3_MASK                        0x8u
#define DMA_EEI_EEI3_SHIFT                       3
#define DMA_EEI_EEI4_MASK                        0x10u
#define DMA_EEI_EEI4_SHIFT                       4
#define DMA_EEI_EEI5_MASK                        0x20u
#define DMA_EEI_EEI5_SHIFT                       5
#define DMA_EEI_EEI6_MASK                        0x40u
#define DMA_EEI_EEI6_SHIFT                       6
#define DMA_EEI_EEI7_MASK                        0x80u
#define DMA_EEI_EEI7_SHIFT                       7
#define DMA_EEI_EEI8_MASK                        0x100u
#define DMA_EEI_EEI8_SHIFT                       8
#define DMA_EEI_EEI9_MASK                        0x200u
#define DMA_EEI_EEI9_SHIFT                       9
#define DMA_EEI_EEI10_MASK                       0x400u
#define DMA_EEI_EEI10_SHIFT                      10
#define DMA_EEI_EEI11_MASK                       0x800u
#define DMA_EEI_EEI11_SHIFT                      11
#define DMA_EEI_EEI12_MASK                       0x1000u
#define DMA_EEI_EEI12_SHIFT                      12
#define DMA_EEI_EEI13_MASK                       0x2000u
#define DMA_EEI_EEI13_SHIFT                      13
#define DMA_EEI_EEI14_MASK                       0x4000u
#define DMA_EEI_EEI14_SHIFT                      14
#define DMA_EEI_EEI15_MASK                       0x8000u
#define DMA_EEI_EEI15_SHIFT                      15
#define DMA_EEI_EEI16_MASK                       0x10000u
#define DMA_EEI_EEI16_SHIFT                      16
#define DMA_EEI_EEI17_MASK                       0x20000u
#define DMA_EEI_EEI17_SHIFT                      17
#define DMA_EEI_EEI18_MASK                       0x40000u
#define DMA_EEI_EEI18_SHIFT                      18
#define DMA_EEI_EEI19_MASK                       0x80000u
#define DMA_EEI_EEI19_SHIFT                      19
#define DMA_EEI_EEI20_MASK                       0x100000u
#define DMA_EEI_EEI20_SHIFT                      20
#define DMA_EEI_EEI21_MASK                       0x200000u
#define DMA_EEI_EEI21_SHIFT                      21
#define DMA_EEI_EEI22_MASK                       0x400000u
#define DMA_EEI_EEI22_SHIFT                      22
#define DMA_EEI_EEI23_MASK                       0x800000u
#define DMA_EEI_EEI23_SHIFT                      23
#define DMA_EEI_EEI24_MASK                       0x1000000u
#define DMA_EEI_EEI24_SHIFT                      24
#define DMA_EEI_EEI25_MASK                       0x2000000u
#define DMA_EEI_EEI25_SHIFT                      25
#define DMA_EEI_EEI26_MASK                       0x4000000u
#define DMA_EEI_EEI26_SHIFT                      26
#define DMA_EEI_EEI27_MASK                       0x8000000u
#define DMA_EEI_EEI27_SHIFT                      27
#define DMA_EEI_EEI28_MASK                       0x10000000u
#define DMA_EEI_EEI28_SHIFT                      28
#define DMA_EEI_EEI29_MASK                       0x20000000u
#define DMA_EEI_EEI29_SHIFT                      29
#define DMA_EEI_EEI30_MASK                       0x40000000u
#define DMA_EEI_EEI30_SHIFT                      30
#define DMA_EEI_EEI31_MASK                       0x80000000u
#define DMA_EEI_EEI31_SHIFT                      31
/* CEEI Bit Fields */
#define DMA_CEEI_CEEI_MASK                       0x1Fu
#define DMA_CEEI_CEEI_SHIFT                      0
#define DMA_CEEI_CEEI(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_CEEI_CEEI_SHIFT))&DMA_CEEI_CEEI_MASK)
#define DMA_CEEI_CAEE_MASK                       0x40u
#define DMA_CEEI_CAEE_SHIFT                      6
#define DMA_CEEI_NOP_MASK                        0x80u
#define DMA_CEEI_NOP_SHIFT                       7
/* SEEI Bit Fields */
#define DMA_SEEI_SEEI_MASK                       0x1Fu
#define DMA_SEEI_SEEI_SHIFT                      0
#define DMA_SEEI_SEEI(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_SEEI_SEEI_SHIFT))&DMA_SEEI_SEEI_MASK)
#define DMA_SEEI_SAEE_MASK                       0x40u
#define DMA_SEEI_SAEE_SHIFT                      6
#define DMA_SEEI_NOP_MASK                        0x80u
#define DMA_SEEI_NOP_SHIFT                       7
/* CERQ Bit Fields */
#define DMA_CERQ_CERQ_MASK                       0x1Fu
#define DMA_CERQ_CERQ_SHIFT                      0
#define DMA_CERQ_CERQ(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_CERQ_CERQ_SHIFT))&DMA_CERQ_CERQ_MASK)
#define DMA_CERQ_CAER_MASK                       0x40u
#define DMA_CERQ_CAER_SHIFT                      6
#define DMA_CERQ_NOP_MASK                        0x80u
#define DMA_CERQ_NOP_SHIFT                       7
/* SERQ Bit Fields */
#define DMA_SERQ_SERQ_MASK                       0x1Fu
#define DMA_SERQ_SERQ_SHIFT                      0
#define DMA_SERQ_SERQ(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_SERQ_SERQ_SHIFT))&DMA_SERQ_SERQ_MASK)
#define DMA_SERQ_SAER_MASK                       0x40u
#define DMA_SERQ_SAER_SHIFT                      6
#define DMA_SERQ_NOP_MASK                        0x80u
#define DMA_SERQ_NOP_SHIFT                       7
/* CDNE Bit Fields */
#define DMA_CDNE_CDNE_MASK                       0x1Fu
#define DMA_CDNE_CDNE_SHIFT                      0
#define DMA_CDNE_CDNE(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_CDNE_CDNE_SHIFT))&DMA_CDNE_CDNE_MASK)
#define DMA_CDNE_CADN_MASK                       0x40u
#define DMA_CDNE_CADN_SHIFT                      6
#define DMA_CDNE_NOP_MASK                        0x80u
#define DMA_CDNE_NOP_SHIFT                       7
/* SSRT Bit Fields */
#define DMA_SSRT_SSRT_MASK                       0x1Fu
#define DMA_SSRT_SSRT_SHIFT                      0
#define DMA_SSRT_SSRT(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_SSRT_SSRT_SHIFT))&DMA_SSRT_SSRT_MASK)
#define DMA_SSRT_SAST_MASK                       0x40u
#define DMA_SSRT_SAST_SHIFT                      6
#define DMA_SSRT_NOP_MASK                        0x80u
#define DMA_SSRT_NOP_SHIFT                       7
/* CERR Bit Fields */
#define DMA_CERR_CERR_MASK                       0x1Fu
#define DMA_CERR_CERR_SHIFT                      0
#define DMA_CERR_CERR(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_CERR_CERR_SHIFT))&DMA_CERR_CERR_MASK)
#define DMA_CERR_CAEI_MASK                       0x40u
#define DMA_CERR_CAEI_SHIFT                      6
#define DMA_CERR_NOP_MASK                        0x80u
#define DMA_CERR_NOP_SHIFT                       7
/* CINT Bit Fields */
#define DMA_CINT_CINT_MASK                       0x1Fu
#define DMA_CINT_CINT_SHIFT                      0
#define DMA_CINT_CINT(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_CINT_CINT_SHIFT))&DMA_CINT_CINT_MASK)
#define DMA_CINT_CAIR_MASK                       0x40u
#define DMA_CINT_CAIR_SHIFT                      6
#define DMA_CINT_NOP_MASK                        0x80u
#define DMA_CINT_NOP_SHIFT                       7
/* INT Bit Fields */
#define DMA_INT_INT0_MASK                        0x1u
#define DMA_INT_INT0_SHIFT                       0
#define DMA_INT_INT1_MASK                        0x2u
#define DMA_INT_INT1_SHIFT                       1
#define DMA_INT_INT2_MASK                        0x4u
#define DMA_INT_INT2_SHIFT                       2
#define DMA_INT_INT3_MASK                        0x8u
#define DMA_INT_INT3_SHIFT                       3
#define DMA_INT_INT4_MASK                        0x10u
#define DMA_INT_INT4_SHIFT                       4
#define DMA_INT_INT5_MASK                        0x20u
#define DMA_INT_INT5_SHIFT                       5
#define DMA_INT_INT6_MASK                        0x40u
#define DMA_INT_INT6_SHIFT                       6
#define DMA_INT_INT7_MASK                        0x80u
#define DMA_INT_INT7_SHIFT                       7
#define DMA_INT_INT8_MASK                        0x100u
#define DMA_INT_INT8_SHIFT                       8
#define DMA_INT_INT9_MASK                        0x200u
#define DMA_INT_INT9_SHIFT                       9
#define DMA_INT_INT10_MASK                       0x400u
#define DMA_INT_INT10_SHIFT                      10
#define DMA_INT_INT11_MASK                       0x800u
#define DMA_INT_INT11_SHIFT                      11
#define DMA_INT_INT12_MASK                       0x1000u
#define DMA_INT_INT12_SHIFT                      12
#define DMA_INT_INT13_MASK                       0x2000u
#define DMA_INT_INT13_SHIFT                      13
#define DMA_INT_INT14_MASK                       0x4000u
#define DMA_INT_INT14_SHIFT                      14
#define DMA_INT_INT15_MASK                       0x8000u
#define DMA_INT_INT15_SHIFT                      15
#define DMA_INT_INT16_MASK                       0x10000u
#define DMA_INT_INT16_SHIFT                      16
#define DMA_INT_INT17_MASK                       0x20000u
#define DMA_INT_INT17_SHIFT                      17
#define DMA_INT_INT18_MASK                       0x40000u
#define DMA_INT_INT18_SHIFT                      18
#define DMA_INT_INT19_MASK                       0x80000u
#define DMA_INT_INT19_SHIFT                      19
#define DMA_INT_INT20_MASK                       0x100000u
#define DMA_INT_INT20_SHIFT                      20
#define DMA_INT_INT21_MASK                       0x200000u
#define DMA_INT_INT21_SHIFT                      21
#define DMA_INT_INT22_MASK                       0x400000u
#define DMA_INT_INT22_SHIFT                      22
#define DMA_INT_INT23_MASK                       0x800000u
#define DMA_INT_INT23_SHIFT                      23
#define DMA_INT_INT24_MASK                       0x1000000u
#define DMA_INT_INT24_SHIFT                      24
#define DMA_INT_INT25_MASK                       0x2000000u
#define DMA_INT_INT25_SHIFT                      25
#define DMA_INT_INT26_MASK                       0x4000000u
#define DMA_INT_INT26_SHIFT                      26
#define DMA_INT_INT27_MASK                       0x8000000u
#define DMA_INT_INT27_SHIFT                      27
#define DMA_INT_INT28_MASK                       0x10000000u
#define DMA_INT_INT28_SHIFT                      28
#define DMA_INT_INT29_MASK                       0x20000000u
#define DMA_INT_INT29_SHIFT                      29
#define DMA_INT_INT30_MASK                       0x40000000u
#define DMA_INT_INT30_SHIFT                      30
#define DMA_INT_INT31_MASK                       0x80000000u
#define DMA_INT_INT31_SHIFT                      31
/* ERR Bit Fields */
#define DMA_ERR_ERR0_MASK                        0x1u
#define DMA_ERR_ERR0_SHIFT                       0
#define DMA_ERR_ERR1_MASK                        0x2u
#define DMA_ERR_ERR1_SHIFT                       1
#define DMA_ERR_ERR2_MASK                        0x4u
#define DMA_ERR_ERR2_SHIFT                       2
#define DMA_ERR_ERR3_MASK                        0x8u
#define DMA_ERR_ERR3_SHIFT                       3
#define DMA_ERR_ERR4_MASK                        0x10u
#define DMA_ERR_ERR4_SHIFT                       4
#define DMA_ERR_ERR5_MASK                        0x20u
#define DMA_ERR_ERR5_SHIFT                       5
#define DMA_ERR_ERR6_MASK                        0x40u
#define DMA_ERR_ERR6_SHIFT                       6
#define DMA_ERR_ERR7_MASK                        0x80u
#define DMA_ERR_ERR7_SHIFT                       7
#define DMA_ERR_ERR8_MASK                        0x100u
#define DMA_ERR_ERR8_SHIFT                       8
#define DMA_ERR_ERR9_MASK                        0x200u
#define DMA_ERR_ERR9_SHIFT                       9
#define DMA_ERR_ERR10_MASK                       0x400u
#define DMA_ERR_ERR10_SHIFT                      10
#define DMA_ERR_ERR11_MASK                       0x800u
#define DMA_ERR_ERR11_SHIFT                      11
#define DMA_ERR_ERR12_MASK                       0x1000u
#define DMA_ERR_ERR12_SHIFT                      12
#define DMA_ERR_ERR13_MASK                       0x2000u
#define DMA_ERR_ERR13_SHIFT                      13
#define DMA_ERR_ERR14_MASK                       0x4000u
#define DMA_ERR_ERR14_SHIFT                      14
#define DMA_ERR_ERR15_MASK                       0x8000u
#define DMA_ERR_ERR15_SHIFT                      15
#define DMA_ERR_ERR16_MASK                       0x10000u
#define DMA_ERR_ERR16_SHIFT                      16
#define DMA_ERR_ERR17_MASK                       0x20000u
#define DMA_ERR_ERR17_SHIFT                      17
#define DMA_ERR_ERR18_MASK                       0x40000u
#define DMA_ERR_ERR18_SHIFT                      18
#define DMA_ERR_ERR19_MASK                       0x80000u
#define DMA_ERR_ERR19_SHIFT                      19
#define DMA_ERR_ERR20_MASK                       0x100000u
#define DMA_ERR_ERR20_SHIFT                      20
#define DMA_ERR_ERR21_MASK                       0x200000u
#define DMA_ERR_ERR21_SHIFT                      21
#define DMA_ERR_ERR22_MASK                       0x400000u
#define DMA_ERR_ERR22_SHIFT                      22
#define DMA_ERR_ERR23_MASK                       0x800000u
#define DMA_ERR_ERR23_SHIFT                      23
#define DMA_ERR_ERR24_MASK                       0x1000000u
#define DMA_ERR_ERR24_SHIFT                      24
#define DMA_ERR_ERR25_MASK                       0x2000000u
#define DMA_ERR_ERR25_SHIFT                      25
#define DMA_ERR_ERR26_MASK                       0x4000000u
#define DMA_ERR_ERR26_SHIFT                      26
#define DMA_ERR_ERR27_MASK                       0x8000000u
#define DMA_ERR_ERR27_SHIFT                      27
#define DMA_ERR_ERR28_MASK                       0x10000000u
#define DMA_ERR_ERR28_SHIFT                      28
#define DMA_ERR_ERR29_MASK                       0x20000000u
#define DMA_ERR_ERR29_SHIFT                      29
#define DMA_ERR_ERR30_MASK                       0x40000000u
#define DMA_ERR_ERR30_SHIFT                      30
#define DMA_ERR_ERR31_MASK                       0x80000000u
#define DMA_ERR_ERR31_SHIFT                      31
/* HRS Bit Fields */
#define DMA_HRS_HRS0_MASK                        0x1u
#define DMA_HRS_HRS0_SHIFT                       0
#define DMA_HRS_HRS1_MASK                        0x2u
#define DMA_HRS_HRS1_SHIFT                       1
#define DMA_HRS_HRS2_MASK                        0x4u
#define DMA_HRS_HRS2_SHIFT                       2
#define DMA_HRS_HRS3_MASK                        0x8u
#define DMA_HRS_HRS3_SHIFT                       3
#define DMA_HRS_HRS4_MASK                        0x10u
#define DMA_HRS_HRS4_SHIFT                       4
#define DMA_HRS_HRS5_MASK                        0x20u
#define DMA_HRS_HRS5_SHIFT                       5
#define DMA_HRS_HRS6_MASK                        0x40u
#define DMA_HRS_HRS6_SHIFT                       6
#define DMA_HRS_HRS7_MASK                        0x80u
#define DMA_HRS_HRS7_SHIFT                       7
#define DMA_HRS_HRS8_MASK                        0x100u
#define DMA_HRS_HRS8_SHIFT                       8
#define DMA_HRS_HRS9_MASK                        0x200u
#define DMA_HRS_HRS9_SHIFT                       9
#define DMA_HRS_HRS10_MASK                       0x400u
#define DMA_HRS_HRS10_SHIFT                      10
#define DMA_HRS_HRS11_MASK                       0x800u
#define DMA_HRS_HRS11_SHIFT                      11
#define DMA_HRS_HRS12_MASK                       0x1000u
#define DMA_HRS_HRS12_SHIFT                      12
#define DMA_HRS_HRS13_MASK                       0x2000u
#define DMA_HRS_HRS13_SHIFT                      13
#define DMA_HRS_HRS14_MASK                       0x4000u
#define DMA_HRS_HRS14_SHIFT                      14
#define DMA_HRS_HRS15_MASK                       0x8000u
#define DMA_HRS_HRS15_SHIFT                      15
#define DMA_HRS_HRS16_MASK                       0x10000u
#define DMA_HRS_HRS16_SHIFT                      16
#define DMA_HRS_HRS17_MASK                       0x20000u
#define DMA_HRS_HRS17_SHIFT                      17
#define DMA_HRS_HRS18_MASK                       0x40000u
#define DMA_HRS_HRS18_SHIFT                      18
#define DMA_HRS_HRS19_MASK                       0x80000u
#define DMA_HRS_HRS19_SHIFT                      19
#define DMA_HRS_HRS20_MASK                       0x100000u
#define DMA_HRS_HRS20_SHIFT                      20
#define DMA_HRS_HRS21_MASK                       0x200000u
#define DMA_HRS_HRS21_SHIFT                      21
#define DMA_HRS_HRS22_MASK                       0x400000u
#define DMA_HRS_HRS22_SHIFT                      22
#define DMA_HRS_HRS23_MASK                       0x800000u
#define DMA_HRS_HRS23_SHIFT                      23
#define DMA_HRS_HRS24_MASK                       0x1000000u
#define DMA_HRS_HRS24_SHIFT                      24
#define DMA_HRS_HRS25_MASK                       0x2000000u
#define DMA_HRS_HRS25_SHIFT                      25
#define DMA_HRS_HRS26_MASK                       0x4000000u
#define DMA_HRS_HRS26_SHIFT                      26
#define DMA_HRS_HRS27_MASK                       0x8000000u
#define DMA_HRS_HRS27_SHIFT                      27
#define DMA_HRS_HRS28_MASK                       0x10000000u
#define DMA_HRS_HRS28_SHIFT                      28
#define DMA_HRS_HRS29_MASK                       0x20000000u
#define DMA_HRS_HRS29_SHIFT                      29
#define DMA_HRS_HRS30_MASK                       0x40000000u
#define DMA_HRS_HRS30_SHIFT                      30
#define DMA_HRS_HRS31_MASK                       0x80000000u
#define DMA_HRS_HRS31_SHIFT                      31
/* DCHPRI3 Bit Fields */
#define DMA_DCHPRI3_CHPRI_MASK                   0xFu
#define DMA_DCHPRI3_CHPRI_SHIFT                  0
#define DMA_DCHPRI3_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI3_CHPRI_SHIFT))&DMA_DCHPRI3_CHPRI_MASK)
#define DMA_DCHPRI3_GRPPRI_MASK                  0x30u
#define DMA_DCHPRI3_GRPPRI_SHIFT                 4
#define DMA_DCHPRI3_GRPPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI3_GRPPRI_SHIFT))&DMA_DCHPRI3_GRPPRI_MASK)
#define DMA_DCHPRI3_DPA_MASK                     0x40u
#define DMA_DCHPRI3_DPA_SHIFT                    6
#define DMA_DCHPRI3_ECP_MASK                     0x80u
#define DMA_DCHPRI3_ECP_SHIFT                    7
/* DCHPRI2 Bit Fields */
#define DMA_DCHPRI2_CHPRI_MASK                   0xFu
#define DMA_DCHPRI2_CHPRI_SHIFT                  0
#define DMA_DCHPRI2_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI2_CHPRI_SHIFT))&DMA_DCHPRI2_CHPRI_MASK)
#define DMA_DCHPRI2_GRPPRI_MASK                  0x30u
#define DMA_DCHPRI2_GRPPRI_SHIFT                 4
#define DMA_DCHPRI2_GRPPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI2_GRPPRI_SHIFT))&DMA_DCHPRI2_GRPPRI_MASK)
#define DMA_DCHPRI2_DPA_MASK                     0x40u
#define DMA_DCHPRI2_DPA_SHIFT                    6
#define DMA_DCHPRI2_ECP_MASK                     0x80u
#define DMA_DCHPRI2_ECP_SHIFT                    7
/* DCHPRI1 Bit Fields */
#define DMA_DCHPRI1_CHPRI_MASK                   0xFu
#define DMA_DCHPRI1_CHPRI_SHIFT                  0
#define DMA_DCHPRI1_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI1_CHPRI_SHIFT))&DMA_DCHPRI1_CHPRI_MASK)
#define DMA_DCHPRI1_GRPPRI_MASK                  0x30u
#define DMA_DCHPRI1_GRPPRI_SHIFT                 4
#define DMA_DCHPRI1_GRPPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI1_GRPPRI_SHIFT))&DMA_DCHPRI1_GRPPRI_MASK)
#define DMA_DCHPRI1_DPA_MASK                     0x40u
#define DMA_DCHPRI1_DPA_SHIFT                    6
#define DMA_DCHPRI1_ECP_MASK                     0x80u
#define DMA_DCHPRI1_ECP_SHIFT                    7
/* DCHPRI0 Bit Fields */
#define DMA_DCHPRI0_CHPRI_MASK                   0xFu
#define DMA_DCHPRI0_CHPRI_SHIFT                  0
#define DMA_DCHPRI0_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI0_CHPRI_SHIFT))&DMA_DCHPRI0_CHPRI_MASK)
#define DMA_DCHPRI0_GRPPRI_MASK                  0x30u
#define DMA_DCHPRI0_GRPPRI_SHIFT                 4
#define DMA_DCHPRI0_GRPPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI0_GRPPRI_SHIFT))&DMA_DCHPRI0_GRPPRI_MASK)
#define DMA_DCHPRI0_DPA_MASK                     0x40u
#define DMA_DCHPRI0_DPA_SHIFT                    6
#define DMA_DCHPRI0_ECP_MASK                     0x80u
#define DMA_DCHPRI0_ECP_SHIFT                    7
/* DCHPRI7 Bit Fields */
#define DMA_DCHPRI7_CHPRI_MASK                   0xFu
#define DMA_DCHPRI7_CHPRI_SHIFT                  0
#define DMA_DCHPRI7_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI7_CHPRI_SHIFT))&DMA_DCHPRI7_CHPRI_MASK)
#define DMA_DCHPRI7_GRPPRI_MASK                  0x30u
#define DMA_DCHPRI7_GRPPRI_SHIFT                 4
#define DMA_DCHPRI7_GRPPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI7_GRPPRI_SHIFT))&DMA_DCHPRI7_GRPPRI_MASK)
#define DMA_DCHPRI7_DPA_MASK                     0x40u
#define DMA_DCHPRI7_DPA_SHIFT                    6
#define DMA_DCHPRI7_ECP_MASK                     0x80u
#define DMA_DCHPRI7_ECP_SHIFT                    7
/* DCHPRI6 Bit Fields */
#define DMA_DCHPRI6_CHPRI_MASK                   0xFu
#define DMA_DCHPRI6_CHPRI_SHIFT                  0
#define DMA_DCHPRI6_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI6_CHPRI_SHIFT))&DMA_DCHPRI6_CHPRI_MASK)
#define DMA_DCHPRI6_GRPPRI_MASK                  0x30u
#define DMA_DCHPRI6_GRPPRI_SHIFT                 4
#define DMA_DCHPRI6_GRPPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI6_GRPPRI_SHIFT))&DMA_DCHPRI6_GRPPRI_MASK)
#define DMA_DCHPRI6_DPA_MASK                     0x40u
#define DMA_DCHPRI6_DPA_SHIFT                    6
#define DMA_DCHPRI6_ECP_MASK                     0x80u
#define DMA_DCHPRI6_ECP_SHIFT                    7
/* DCHPRI5 Bit Fields */
#define DMA_DCHPRI5_CHPRI_MASK                   0xFu
#define DMA_DCHPRI5_CHPRI_SHIFT                  0
#define DMA_DCHPRI5_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI5_CHPRI_SHIFT))&DMA_DCHPRI5_CHPRI_MASK)
#define DMA_DCHPRI5_GRPPRI_MASK                  0x30u
#define DMA_DCHPRI5_GRPPRI_SHIFT                 4
#define DMA_DCHPRI5_GRPPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI5_GRPPRI_SHIFT))&DMA_DCHPRI5_GRPPRI_MASK)
#define DMA_DCHPRI5_DPA_MASK                     0x40u
#define DMA_DCHPRI5_DPA_SHIFT                    6
#define DMA_DCHPRI5_ECP_MASK                     0x80u
#define DMA_DCHPRI5_ECP_SHIFT                    7
/* DCHPRI4 Bit Fields */
#define DMA_DCHPRI4_CHPRI_MASK                   0xFu
#define DMA_DCHPRI4_CHPRI_SHIFT                  0
#define DMA_DCHPRI4_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI4_CHPRI_SHIFT))&DMA_DCHPRI4_CHPRI_MASK)
#define DMA_DCHPRI4_GRPPRI_MASK                  0x30u
#define DMA_DCHPRI4_GRPPRI_SHIFT                 4
#define DMA_DCHPRI4_GRPPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI4_GRPPRI_SHIFT))&DMA_DCHPRI4_GRPPRI_MASK)
#define DMA_DCHPRI4_DPA_MASK                     0x40u
#define DMA_DCHPRI4_DPA_SHIFT                    6
#define DMA_DCHPRI4_ECP_MASK                     0x80u
#define DMA_DCHPRI4_ECP_SHIFT                    7
/* DCHPRI11 Bit Fields */
#define DMA_DCHPRI11_CHPRI_MASK                  0xFu
#define DMA_DCHPRI11_CHPRI_SHIFT                 0
#define DMA_DCHPRI11_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI11_CHPRI_SHIFT))&DMA_DCHPRI11_CHPRI_MASK)
#define DMA_DCHPRI11_GRPPRI_MASK                 0x30u
#define DMA_DCHPRI11_GRPPRI_SHIFT                4
#define DMA_DCHPRI11_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI11_GRPPRI_SHIFT))&DMA_DCHPRI11_GRPPRI_MASK)
#define DMA_DCHPRI11_DPA_MASK                    0x40u
#define DMA_DCHPRI11_DPA_SHIFT                   6
#define DMA_DCHPRI11_ECP_MASK                    0x80u
#define DMA_DCHPRI11_ECP_SHIFT                   7
/* DCHPRI10 Bit Fields */
#define DMA_DCHPRI10_CHPRI_MASK                  0xFu
#define DMA_DCHPRI10_CHPRI_SHIFT                 0
#define DMA_DCHPRI10_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI10_CHPRI_SHIFT))&DMA_DCHPRI10_CHPRI_MASK)
#define DMA_DCHPRI10_GRPPRI_MASK                 0x30u
#define DMA_DCHPRI10_GRPPRI_SHIFT                4
#define DMA_DCHPRI10_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI10_GRPPRI_SHIFT))&DMA_DCHPRI10_GRPPRI_MASK)
#define DMA_DCHPRI10_DPA_MASK                    0x40u
#define DMA_DCHPRI10_DPA_SHIFT                   6
#define DMA_DCHPRI10_ECP_MASK                    0x80u
#define DMA_DCHPRI10_ECP_SHIFT                   7
/* DCHPRI9 Bit Fields */
#define DMA_DCHPRI9_CHPRI_MASK                   0xFu
#define DMA_DCHPRI9_CHPRI_SHIFT                  0
#define DMA_DCHPRI9_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI9_CHPRI_SHIFT))&DMA_DCHPRI9_CHPRI_MASK)
#define DMA_DCHPRI9_GRPPRI_MASK                  0x30u
#define DMA_DCHPRI9_GRPPRI_SHIFT                 4
#define DMA_DCHPRI9_GRPPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI9_GRPPRI_SHIFT))&DMA_DCHPRI9_GRPPRI_MASK)
#define DMA_DCHPRI9_DPA_MASK                     0x40u
#define DMA_DCHPRI9_DPA_SHIFT                    6
#define DMA_DCHPRI9_ECP_MASK                     0x80u
#define DMA_DCHPRI9_ECP_SHIFT                    7
/* DCHPRI8 Bit Fields */
#define DMA_DCHPRI8_CHPRI_MASK                   0xFu
#define DMA_DCHPRI8_CHPRI_SHIFT                  0
#define DMA_DCHPRI8_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI8_CHPRI_SHIFT))&DMA_DCHPRI8_CHPRI_MASK)
#define DMA_DCHPRI8_GRPPRI_MASK                  0x30u
#define DMA_DCHPRI8_GRPPRI_SHIFT                 4
#define DMA_DCHPRI8_GRPPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI8_GRPPRI_SHIFT))&DMA_DCHPRI8_GRPPRI_MASK)
#define DMA_DCHPRI8_DPA_MASK                     0x40u
#define DMA_DCHPRI8_DPA_SHIFT                    6
#define DMA_DCHPRI8_ECP_MASK                     0x80u
#define DMA_DCHPRI8_ECP_SHIFT                    7
/* DCHPRI15 Bit Fields */
#define DMA_DCHPRI15_CHPRI_MASK                  0xFu
#define DMA_DCHPRI15_CHPRI_SHIFT                 0
#define DMA_DCHPRI15_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI15_CHPRI_SHIFT))&DMA_DCHPRI15_CHPRI_MASK)
#define DMA_DCHPRI15_GRPPRI_MASK                 0x30u
#define DMA_DCHPRI15_GRPPRI_SHIFT                4
#define DMA_DCHPRI15_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI15_GRPPRI_SHIFT))&DMA_DCHPRI15_GRPPRI_MASK)
#define DMA_DCHPRI15_DPA_MASK                    0x40u
#define DMA_DCHPRI15_DPA_SHIFT                   6
#define DMA_DCHPRI15_ECP_MASK                    0x80u
#define DMA_DCHPRI15_ECP_SHIFT                   7
/* DCHPRI14 Bit Fields */
#define DMA_DCHPRI14_CHPRI_MASK                  0xFu
#define DMA_DCHPRI14_CHPRI_SHIFT                 0
#define DMA_DCHPRI14_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI14_CHPRI_SHIFT))&DMA_DCHPRI14_CHPRI_MASK)
#define DMA_DCHPRI14_GRPPRI_MASK                 0x30u
#define DMA_DCHPRI14_GRPPRI_SHIFT                4
#define DMA_DCHPRI14_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI14_GRPPRI_SHIFT))&DMA_DCHPRI14_GRPPRI_MASK)
#define DMA_DCHPRI14_DPA_MASK                    0x40u
#define DMA_DCHPRI14_DPA_SHIFT                   6
#define DMA_DCHPRI14_ECP_MASK                    0x80u
#define DMA_DCHPRI14_ECP_SHIFT                   7
/* DCHPRI13 Bit Fields */
#define DMA_DCHPRI13_CHPRI_MASK                  0xFu
#define DMA_DCHPRI13_CHPRI_SHIFT                 0
#define DMA_DCHPRI13_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI13_CHPRI_SHIFT))&DMA_DCHPRI13_CHPRI_MASK)
#define DMA_DCHPRI13_GRPPRI_MASK                 0x30u
#define DMA_DCHPRI13_GRPPRI_SHIFT                4
#define DMA_DCHPRI13_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI13_GRPPRI_SHIFT))&DMA_DCHPRI13_GRPPRI_MASK)
#define DMA_DCHPRI13_DPA_MASK                    0x40u
#define DMA_DCHPRI13_DPA_SHIFT                   6
#define DMA_DCHPRI13_ECP_MASK                    0x80u
#define DMA_DCHPRI13_ECP_SHIFT                   7
/* DCHPRI12 Bit Fields */
#define DMA_DCHPRI12_CHPRI_MASK                  0xFu
#define DMA_DCHPRI12_CHPRI_SHIFT                 0
#define DMA_DCHPRI12_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI12_CHPRI_SHIFT))&DMA_DCHPRI12_CHPRI_MASK)
#define DMA_DCHPRI12_GRPPRI_MASK                 0x30u
#define DMA_DCHPRI12_GRPPRI_SHIFT                4
#define DMA_DCHPRI12_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI12_GRPPRI_SHIFT))&DMA_DCHPRI12_GRPPRI_MASK)
#define DMA_DCHPRI12_DPA_MASK                    0x40u
#define DMA_DCHPRI12_DPA_SHIFT                   6
#define DMA_DCHPRI12_ECP_MASK                    0x80u
#define DMA_DCHPRI12_ECP_SHIFT                   7
/* DCHPRI19 Bit Fields */
#define DMA_DCHPRI19_CHPRI_MASK                  0xFu
#define DMA_DCHPRI19_CHPRI_SHIFT                 0
#define DMA_DCHPRI19_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI19_CHPRI_SHIFT))&DMA_DCHPRI19_CHPRI_MASK)
#define DMA_DCHPRI19_GRPPRI_MASK                 0x30u
#define DMA_DCHPRI19_GRPPRI_SHIFT                4
#define DMA_DCHPRI19_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI19_GRPPRI_SHIFT))&DMA_DCHPRI19_GRPPRI_MASK)
#define DMA_DCHPRI19_DPA_MASK                    0x40u
#define DMA_DCHPRI19_DPA_SHIFT                   6
#define DMA_DCHPRI19_ECP_MASK                    0x80u
#define DMA_DCHPRI19_ECP_SHIFT                   7
/* DCHPRI18 Bit Fields */
#define DMA_DCHPRI18_CHPRI_MASK                  0xFu
#define DMA_DCHPRI18_CHPRI_SHIFT                 0
#define DMA_DCHPRI18_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI18_CHPRI_SHIFT))&DMA_DCHPRI18_CHPRI_MASK)
#define DMA_DCHPRI18_GRPPRI_MASK                 0x30u
#define DMA_DCHPRI18_GRPPRI_SHIFT                4
#define DMA_DCHPRI18_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI18_GRPPRI_SHIFT))&DMA_DCHPRI18_GRPPRI_MASK)
#define DMA_DCHPRI18_DPA_MASK                    0x40u
#define DMA_DCHPRI18_DPA_SHIFT                   6
#define DMA_DCHPRI18_ECP_MASK                    0x80u
#define DMA_DCHPRI18_ECP_SHIFT                   7
/* DCHPRI17 Bit Fields */
#define DMA_DCHPRI17_CHPRI_MASK                  0xFu
#define DMA_DCHPRI17_CHPRI_SHIFT                 0
#define DMA_DCHPRI17_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI17_CHPRI_SHIFT))&DMA_DCHPRI17_CHPRI_MASK)
#define DMA_DCHPRI17_GRPPRI_MASK                 0x30u
#define DMA_DCHPRI17_GRPPRI_SHIFT                4
#define DMA_DCHPRI17_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI17_GRPPRI_SHIFT))&DMA_DCHPRI17_GRPPRI_MASK)
#define DMA_DCHPRI17_DPA_MASK                    0x40u
#define DMA_DCHPRI17_DPA_SHIFT                   6
#define DMA_DCHPRI17_ECP_MASK                    0x80u
#define DMA_DCHPRI17_ECP_SHIFT                   7
/* DCHPRI16 Bit Fields */
#define DMA_DCHPRI16_CHPRI_MASK                  0xFu
#define DMA_DCHPRI16_CHPRI_SHIFT                 0
#define DMA_DCHPRI16_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI16_CHPRI_SHIFT))&DMA_DCHPRI16_CHPRI_MASK)
#define DMA_DCHPRI16_GRPPRI_MASK                 0x30u
#define DMA_DCHPRI16_GRPPRI_SHIFT                4
#define DMA_DCHPRI16_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI16_GRPPRI_SHIFT))&DMA_DCHPRI16_GRPPRI_MASK)
#define DMA_DCHPRI16_DPA_MASK                    0x40u
#define DMA_DCHPRI16_DPA_SHIFT                   6
#define DMA_DCHPRI16_ECP_MASK                    0x80u
#define DMA_DCHPRI16_ECP_SHIFT                   7
/* DCHPRI23 Bit Fields */
#define DMA_DCHPRI23_CHPRI_MASK                  0xFu
#define DMA_DCHPRI23_CHPRI_SHIFT                 0
#define DMA_DCHPRI23_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI23_CHPRI_SHIFT))&DMA_DCHPRI23_CHPRI_MASK)
#define DMA_DCHPRI23_GRPPRI_MASK                 0x30u
#define DMA_DCHPRI23_GRPPRI_SHIFT                4
#define DMA_DCHPRI23_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI23_GRPPRI_SHIFT))&DMA_DCHPRI23_GRPPRI_MASK)
#define DMA_DCHPRI23_DPA_MASK                    0x40u
#define DMA_DCHPRI23_DPA_SHIFT                   6
#define DMA_DCHPRI23_ECP_MASK                    0x80u
#define DMA_DCHPRI23_ECP_SHIFT                   7
/* DCHPRI22 Bit Fields */
#define DMA_DCHPRI22_CHPRI_MASK                  0xFu
#define DMA_DCHPRI22_CHPRI_SHIFT                 0
#define DMA_DCHPRI22_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI22_CHPRI_SHIFT))&DMA_DCHPRI22_CHPRI_MASK)
#define DMA_DCHPRI22_GRPPRI_MASK                 0x30u
#define DMA_DCHPRI22_GRPPRI_SHIFT                4
#define DMA_DCHPRI22_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI22_GRPPRI_SHIFT))&DMA_DCHPRI22_GRPPRI_MASK)
#define DMA_DCHPRI22_DPA_MASK                    0x40u
#define DMA_DCHPRI22_DPA_SHIFT                   6
#define DMA_DCHPRI22_ECP_MASK                    0x80u
#define DMA_DCHPRI22_ECP_SHIFT                   7
/* DCHPRI21 Bit Fields */
#define DMA_DCHPRI21_CHPRI_MASK                  0xFu
#define DMA_DCHPRI21_CHPRI_SHIFT                 0
#define DMA_DCHPRI21_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI21_CHPRI_SHIFT))&DMA_DCHPRI21_CHPRI_MASK)
#define DMA_DCHPRI21_GRPPRI_MASK                 0x30u
#define DMA_DCHPRI21_GRPPRI_SHIFT                4
#define DMA_DCHPRI21_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI21_GRPPRI_SHIFT))&DMA_DCHPRI21_GRPPRI_MASK)
#define DMA_DCHPRI21_DPA_MASK                    0x40u
#define DMA_DCHPRI21_DPA_SHIFT                   6
#define DMA_DCHPRI21_ECP_MASK                    0x80u
#define DMA_DCHPRI21_ECP_SHIFT                   7
/* DCHPRI20 Bit Fields */
#define DMA_DCHPRI20_CHPRI_MASK                  0xFu
#define DMA_DCHPRI20_CHPRI_SHIFT                 0
#define DMA_DCHPRI20_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI20_CHPRI_SHIFT))&DMA_DCHPRI20_CHPRI_MASK)
#define DMA_DCHPRI20_GRPPRI_MASK                 0x30u
#define DMA_DCHPRI20_GRPPRI_SHIFT                4
#define DMA_DCHPRI20_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI20_GRPPRI_SHIFT))&DMA_DCHPRI20_GRPPRI_MASK)
#define DMA_DCHPRI20_DPA_MASK                    0x40u
#define DMA_DCHPRI20_DPA_SHIFT                   6
#define DMA_DCHPRI20_ECP_MASK                    0x80u
#define DMA_DCHPRI20_ECP_SHIFT                   7
/* DCHPRI27 Bit Fields */
#define DMA_DCHPRI27_CHPRI_MASK                  0xFu
#define DMA_DCHPRI27_CHPRI_SHIFT                 0
#define DMA_DCHPRI27_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI27_CHPRI_SHIFT))&DMA_DCHPRI27_CHPRI_MASK)
#define DMA_DCHPRI27_GRPPRI_MASK                 0x30u
#define DMA_DCHPRI27_GRPPRI_SHIFT                4
#define DMA_DCHPRI27_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI27_GRPPRI_SHIFT))&DMA_DCHPRI27_GRPPRI_MASK)
#define DMA_DCHPRI27_DPA_MASK                    0x40u
#define DMA_DCHPRI27_DPA_SHIFT                   6
#define DMA_DCHPRI27_ECP_MASK                    0x80u
#define DMA_DCHPRI27_ECP_SHIFT                   7
/* DCHPRI26 Bit Fields */
#define DMA_DCHPRI26_CHPRI_MASK                  0xFu
#define DMA_DCHPRI26_CHPRI_SHIFT                 0
#define DMA_DCHPRI26_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI26_CHPRI_SHIFT))&DMA_DCHPRI26_CHPRI_MASK)
#define DMA_DCHPRI26_GRPPRI_MASK                 0x30u
#define DMA_DCHPRI26_GRPPRI_SHIFT                4
#define DMA_DCHPRI26_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI26_GRPPRI_SHIFT))&DMA_DCHPRI26_GRPPRI_MASK)
#define DMA_DCHPRI26_DPA_MASK                    0x40u
#define DMA_DCHPRI26_DPA_SHIFT                   6
#define DMA_DCHPRI26_ECP_MASK                    0x80u
#define DMA_DCHPRI26_ECP_SHIFT                   7
/* DCHPRI25 Bit Fields */
#define DMA_DCHPRI25_CHPRI_MASK                  0xFu
#define DMA_DCHPRI25_CHPRI_SHIFT                 0
#define DMA_DCHPRI25_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI25_CHPRI_SHIFT))&DMA_DCHPRI25_CHPRI_MASK)
#define DMA_DCHPRI25_GRPPRI_MASK                 0x30u
#define DMA_DCHPRI25_GRPPRI_SHIFT                4
#define DMA_DCHPRI25_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI25_GRPPRI_SHIFT))&DMA_DCHPRI25_GRPPRI_MASK)
#define DMA_DCHPRI25_DPA_MASK                    0x40u
#define DMA_DCHPRI25_DPA_SHIFT                   6
#define DMA_DCHPRI25_ECP_MASK                    0x80u
#define DMA_DCHPRI25_ECP_SHIFT                   7
/* DCHPRI24 Bit Fields */
#define DMA_DCHPRI24_CHPRI_MASK                  0xFu
#define DMA_DCHPRI24_CHPRI_SHIFT                 0
#define DMA_DCHPRI24_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI24_CHPRI_SHIFT))&DMA_DCHPRI24_CHPRI_MASK)
#define DMA_DCHPRI24_GRPPRI_MASK                 0x30u
#define DMA_DCHPRI24_GRPPRI_SHIFT                4
#define DMA_DCHPRI24_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI24_GRPPRI_SHIFT))&DMA_DCHPRI24_GRPPRI_MASK)
#define DMA_DCHPRI24_DPA_MASK                    0x40u
#define DMA_DCHPRI24_DPA_SHIFT                   6
#define DMA_DCHPRI24_ECP_MASK                    0x80u
#define DMA_DCHPRI24_ECP_SHIFT                   7
/* DCHPRI31 Bit Fields */
#define DMA_DCHPRI31_CHPRI_MASK                  0xFu
#define DMA_DCHPRI31_CHPRI_SHIFT                 0
#define DMA_DCHPRI31_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI31_CHPRI_SHIFT))&DMA_DCHPRI31_CHPRI_MASK)
#define DMA_DCHPRI31_GRPPRI_MASK                 0x30u
#define DMA_DCHPRI31_GRPPRI_SHIFT                4
#define DMA_DCHPRI31_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI31_GRPPRI_SHIFT))&DMA_DCHPRI31_GRPPRI_MASK)
#define DMA_DCHPRI31_DPA_MASK                    0x40u
#define DMA_DCHPRI31_DPA_SHIFT                   6
#define DMA_DCHPRI31_ECP_MASK                    0x80u
#define DMA_DCHPRI31_ECP_SHIFT                   7
/* DCHPRI30 Bit Fields */
#define DMA_DCHPRI30_CHPRI_MASK                  0xFu
#define DMA_DCHPRI30_CHPRI_SHIFT                 0
#define DMA_DCHPRI30_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI30_CHPRI_SHIFT))&DMA_DCHPRI30_CHPRI_MASK)
#define DMA_DCHPRI30_GRPPRI_MASK                 0x30u
#define DMA_DCHPRI30_GRPPRI_SHIFT                4
#define DMA_DCHPRI30_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI30_GRPPRI_SHIFT))&DMA_DCHPRI30_GRPPRI_MASK)
#define DMA_DCHPRI30_DPA_MASK                    0x40u
#define DMA_DCHPRI30_DPA_SHIFT                   6
#define DMA_DCHPRI30_ECP_MASK                    0x80u
#define DMA_DCHPRI30_ECP_SHIFT                   7
/* DCHPRI29 Bit Fields */
#define DMA_DCHPRI29_CHPRI_MASK                  0xFu
#define DMA_DCHPRI29_CHPRI_SHIFT                 0
#define DMA_DCHPRI29_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI29_CHPRI_SHIFT))&DMA_DCHPRI29_CHPRI_MASK)
#define DMA_DCHPRI29_GRPPRI_MASK                 0x30u
#define DMA_DCHPRI29_GRPPRI_SHIFT                4
#define DMA_DCHPRI29_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI29_GRPPRI_SHIFT))&DMA_DCHPRI29_GRPPRI_MASK)
#define DMA_DCHPRI29_DPA_MASK                    0x40u
#define DMA_DCHPRI29_DPA_SHIFT                   6
#define DMA_DCHPRI29_ECP_MASK                    0x80u
#define DMA_DCHPRI29_ECP_SHIFT                   7
/* DCHPRI28 Bit Fields */
#define DMA_DCHPRI28_CHPRI_MASK                  0xFu
#define DMA_DCHPRI28_CHPRI_SHIFT                 0
#define DMA_DCHPRI28_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI28_CHPRI_SHIFT))&DMA_DCHPRI28_CHPRI_MASK)
#define DMA_DCHPRI28_GRPPRI_MASK                 0x30u
#define DMA_DCHPRI28_GRPPRI_SHIFT                4
#define DMA_DCHPRI28_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI28_GRPPRI_SHIFT))&DMA_DCHPRI28_GRPPRI_MASK)
#define DMA_DCHPRI28_DPA_MASK                    0x40u
#define DMA_DCHPRI28_DPA_SHIFT                   6
#define DMA_DCHPRI28_ECP_MASK                    0x80u
#define DMA_DCHPRI28_ECP_SHIFT                   7
/* SADDR Bit Fields */
#define DMA_SADDR_SADDR_MASK                     0xFFFFFFFFu
#define DMA_SADDR_SADDR_SHIFT                    0
#define DMA_SADDR_SADDR(x)                       (((uint32_t)(((uint32_t)(x))<<DMA_SADDR_SADDR_SHIFT))&DMA_SADDR_SADDR_MASK)
/* SOFF Bit Fields */
#define DMA_SOFF_SOFF_MASK                       0xFFFFu
#define DMA_SOFF_SOFF_SHIFT                      0
#define DMA_SOFF_SOFF(x)                         (((uint16_t)(((uint16_t)(x))<<DMA_SOFF_SOFF_SHIFT))&DMA_SOFF_SOFF_MASK)
/* ATTR Bit Fields */
#define DMA_ATTR_DSIZE_MASK                      0x7u
#define DMA_ATTR_DSIZE_SHIFT                     0
#define DMA_ATTR_DSIZE(x)                        (((uint16_t)(((uint16_t)(x))<<DMA_ATTR_DSIZE_SHIFT))&DMA_ATTR_DSIZE_MASK)
#define DMA_ATTR_DMOD_MASK                       0xF8u
#define DMA_ATTR_DMOD_SHIFT                      3
#define DMA_ATTR_DMOD(x)                         (((uint16_t)(((uint16_t)(x))<<DMA_ATTR_DMOD_SHIFT))&DMA_ATTR_DMOD_MASK)
#define DMA_ATTR_SSIZE_MASK                      0x700u
#define DMA_ATTR_SSIZE_SHIFT                     8
#define DMA_ATTR_SSIZE(x)                        (((uint16_t)(((uint16_t)(x))<<DMA_ATTR_SSIZE_SHIFT))&DMA_ATTR_SSIZE_MASK)
#define DMA_ATTR_SMOD_MASK                       0xF800u
#define DMA_ATTR_SMOD_SHIFT                      11
#define DMA_ATTR_SMOD(x)                         (((uint16_t)(((uint16_t)(x))<<DMA_ATTR_SMOD_SHIFT))&DMA_ATTR_SMOD_MASK)
/* NBYTES_MLNO Bit Fields */
#define DMA_NBYTES_MLNO_NBYTES_MASK              0xFFFFFFFFu
#define DMA_NBYTES_MLNO_NBYTES_SHIFT             0
#define DMA_NBYTES_MLNO_NBYTES(x)                (((uint32_t)(((uint32_t)(x))<<DMA_NBYTES_MLNO_NBYTES_SHIFT))&DMA_NBYTES_MLNO_NBYTES_MASK)
/* NBYTES_MLOFFNO Bit Fields */
#define DMA_NBYTES_MLOFFNO_NBYTES_OE_MASK        0x3FFFFFFFu
#define DMA_NBYTES_MLOFFNO_NBYTES_OE_SHIFT       0
#define DMA_NBYTES_MLOFFNO_NBYTES_OE(x)          (((uint32_t)(((uint32_t)(x))<<DMA_NBYTES_MLOFFNO_NBYTES_OE_SHIFT))&DMA_NBYTES_MLOFFNO_NBYTES_OE_MASK)
#define DMA_NBYTES_MLOFFNO_DMLOE_MASK            0x40000000u
#define DMA_NBYTES_MLOFFNO_DMLOE_SHIFT           30
#define DMA_NBYTES_MLOFFNO_SMLOE_MASK            0x80000000u
#define DMA_NBYTES_MLOFFNO_SMLOE_SHIFT           31
/* NBYTES_MLOFFYES Bit Fields */
#define DMA_NBYTES_MLOFFYES_NBYTES_OD_MASK       0x3FFu
#define DMA_NBYTES_MLOFFYES_NBYTES_OD_SHIFT      0
#define DMA_NBYTES_MLOFFYES_NBYTES_OD(x)         (((uint32_t)(((uint32_t)(x))<<DMA_NBYTES_MLOFFYES_NBYTES_OD_SHIFT))&DMA_NBYTES_MLOFFYES_NBYTES_OD_MASK)
#define DMA_NBYTES_MLOFFYES_MLOFF_MASK           0x3FFFFC00u
#define DMA_NBYTES_MLOFFYES_MLOFF_SHIFT          10
#define DMA_NBYTES_MLOFFYES_MLOFF(x)             (((uint32_t)(((uint32_t)(x))<<DMA_NBYTES_MLOFFYES_MLOFF_SHIFT))&DMA_NBYTES_MLOFFYES_MLOFF_MASK)
#define DMA_NBYTES_MLOFFYES_DMLOE_MASK           0x40000000u
#define DMA_NBYTES_MLOFFYES_DMLOE_SHIFT          30
#define DMA_NBYTES_MLOFFYES_SMLOE_MASK           0x80000000u
#define DMA_NBYTES_MLOFFYES_SMLOE_SHIFT          31
/* SLAST Bit Fields */
#define DMA_SLAST_SLAST_MASK                     0xFFFFFFFFu
#define DMA_SLAST_SLAST_SHIFT                    0
#define DMA_SLAST_SLAST(x)                       (((uint32_t)(((uint32_t)(x))<<DMA_SLAST_SLAST_SHIFT))&DMA_SLAST_SLAST_MASK)
/* DADDR Bit Fields */
#define DMA_DADDR_DADDR_MASK                     0xFFFFFFFFu
#define DMA_DADDR_DADDR_SHIFT                    0
#define DMA_DADDR_DADDR(x)                       (((uint32_t)(((uint32_t)(x))<<DMA_DADDR_DADDR_SHIFT))&DMA_DADDR_DADDR_MASK)
/* DOFF Bit Fields */
#define DMA_DOFF_DOFF_MASK                       0xFFFFu
#define DMA_DOFF_DOFF_SHIFT                      0
#define DMA_DOFF_DOFF(x)                         (((uint16_t)(((uint16_t)(x))<<DMA_DOFF_DOFF_SHIFT))&DMA_DOFF_DOFF_MASK)
/* CITER_ELINKNO Bit Fields */
#define DMA_CITER_ELINKNO_CITER_MASK             0x7FFFu
#define DMA_CITER_ELINKNO_CITER_SHIFT            0
#define DMA_CITER_ELINKNO_CITER(x)               (((uint16_t)(((uint16_t)(x))<<DMA_CITER_ELINKNO_CITER_SHIFT))&DMA_CITER_ELINKNO_CITER_MASK)
#define DMA_CITER_ELINKNO_ELINK_MASK             0x8000u
#define DMA_CITER_ELINKNO_ELINK_SHIFT            15
/* CITER_ELINKYES Bit Fields */
#define DMA_CITER_ELINKYES_CITER_LE_MASK         0x1FFu
#define DMA_CITER_ELINKYES_CITER_LE_SHIFT        0
#define DMA_CITER_ELINKYES_CITER_LE(x)           (((uint16_t)(((uint16_t)(x))<<DMA_CITER_ELINKYES_CITER_LE_SHIFT))&DMA_CITER_ELINKYES_CITER_LE_MASK)
#define DMA_CITER_ELINKYES_LINKCH_MASK           0x3E00u
#define DMA_CITER_ELINKYES_LINKCH_SHIFT          9
#define DMA_CITER_ELINKYES_LINKCH(x)             (((uint16_t)(((uint16_t)(x))<<DMA_CITER_ELINKYES_LINKCH_SHIFT))&DMA_CITER_ELINKYES_LINKCH_MASK)
#define DMA_CITER_ELINKYES_ELINK_MASK            0x8000u
#define DMA_CITER_ELINKYES_ELINK_SHIFT           15
/* DLAST_SGA Bit Fields */
#define DMA_DLAST_SGA_DLASTSGA_MASK              0xFFFFFFFFu
#define DMA_DLAST_SGA_DLASTSGA_SHIFT             0
#define DMA_DLAST_SGA_DLASTSGA(x)                (((uint32_t)(((uint32_t)(x))<<DMA_DLAST_SGA_DLASTSGA_SHIFT))&DMA_DLAST_SGA_DLASTSGA_MASK)
/* CSR Bit Fields */
#define DMA_CSR_START_MASK                       0x1u
#define DMA_CSR_START_SHIFT                      0
#define DMA_CSR_INTMAJOR_MASK                    0x2u
#define DMA_CSR_INTMAJOR_SHIFT                   1
#define DMA_CSR_INTHALF_MASK                     0x4u
#define DMA_CSR_INTHALF_SHIFT                    2
#define DMA_CSR_DREQ_MASK                        0x8u
#define DMA_CSR_DREQ_SHIFT                       3
#define DMA_CSR_ESG_MASK                         0x10u
#define DMA_CSR_ESG_SHIFT                        4
#define DMA_CSR_MAJORELINK_MASK                  0x20u
#define DMA_CSR_MAJORELINK_SHIFT                 5
#define DMA_CSR_ACTIVE_MASK                      0x40u
#define DMA_CSR_ACTIVE_SHIFT                     6
#define DMA_CSR_DONE_MASK                        0x80u
#define DMA_CSR_DONE_SHIFT                       7
#define DMA_CSR_MAJORLINKCH_MASK                 0x1F00u
#define DMA_CSR_MAJORLINKCH_SHIFT                8
#define DMA_CSR_MAJORLINKCH(x)                   (((uint16_t)(((uint16_t)(x))<<DMA_CSR_MAJORLINKCH_SHIFT))&DMA_CSR_MAJORLINKCH_MASK)
#define DMA_CSR_BWC_MASK                         0xC000u
#define DMA_CSR_BWC_SHIFT                        14
#define DMA_CSR_BWC(x)                           (((uint16_t)(((uint16_t)(x))<<DMA_CSR_BWC_SHIFT))&DMA_CSR_BWC_MASK)
/* BITER_ELINKNO Bit Fields */
#define DMA_BITER_ELINKNO_BITER_MASK             0x7FFFu
#define DMA_BITER_ELINKNO_BITER_SHIFT            0
#define DMA_BITER_ELINKNO_BITER(x)               (((uint16_t)(((uint16_t)(x))<<DMA_BITER_ELINKNO_BITER_SHIFT))&DMA_BITER_ELINKNO_BITER_MASK)
#define DMA_BITER_ELINKNO_ELINK_MASK             0x8000u
#define DMA_BITER_ELINKNO_ELINK_SHIFT            15
/* BITER_ELINKYES Bit Fields */
#define DMA_BITER_ELINKYES_BITER_LE_MASK         0x1FFu
#define DMA_BITER_ELINKYES_BITER_LE_SHIFT        0
#define DMA_BITER_ELINKYES_BITER_LE(x)           (((uint16_t)(((uint16_t)(x))<<DMA_BITER_ELINKYES_BITER_LE_SHIFT))&DMA_BITER_ELINKYES_BITER_LE_MASK)
#define DMA_BITER_ELINKYES_LINKCH_MASK           0x3E00u
#define DMA_BITER_ELINKYES_LINKCH_SHIFT          9
#define DMA_BITER_ELINKYES_LINKCH(x)             (((uint16_t)(((uint16_t)(x))<<DMA_BITER_ELINKYES_LINKCH_SHIFT))&DMA_BITER_ELINKYES_LINKCH_MASK)
#define DMA_BITER_ELINKYES_ELINK_MASK            0x8000u
#define DMA_BITER_ELINKYES_ELINK_SHIFT           15

/*!
 * @}
 */ /* end of group DMA_Register_Masks */


/* DMA - Peripheral instance base addresses */
/** Peripheral DMA0 base pointer */
#define DMA0_BASE_PTR                            ((DMA_MemMapPtr)0x40018000u)
/** Peripheral DMA1 base pointer */
#define DMA1_BASE_PTR                            ((DMA_MemMapPtr)0x40098000u)
/** Array initializer of DMA peripheral base pointers */
#define DMA_BASE_PTRS                            { DMA0_BASE_PTR, DMA1_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- DMA - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Register_Accessor_Macros DMA - Register accessor macros
 * @{
 */


/* DMA - Register instance definitions */
/* DMA0 */
#define DMA0_CR                                  DMA_CR_REG(DMA0_BASE_PTR)
#define DMA0_ES                                  DMA_ES_REG(DMA0_BASE_PTR)
#define DMA0_ERQ                                 DMA_ERQ_REG(DMA0_BASE_PTR)
#define DMA0_EEI                                 DMA_EEI_REG(DMA0_BASE_PTR)
#define DMA0_CEEI                                DMA_CEEI_REG(DMA0_BASE_PTR)
#define DMA0_SEEI                                DMA_SEEI_REG(DMA0_BASE_PTR)
#define DMA0_CERQ                                DMA_CERQ_REG(DMA0_BASE_PTR)
#define DMA0_SERQ                                DMA_SERQ_REG(DMA0_BASE_PTR)
#define DMA0_CDNE                                DMA_CDNE_REG(DMA0_BASE_PTR)
#define DMA0_SSRT                                DMA_SSRT_REG(DMA0_BASE_PTR)
#define DMA0_CERR                                DMA_CERR_REG(DMA0_BASE_PTR)
#define DMA0_CINT                                DMA_CINT_REG(DMA0_BASE_PTR)
#define DMA0_INT                                 DMA_INT_REG(DMA0_BASE_PTR)
#define DMA0_ERR                                 DMA_ERR_REG(DMA0_BASE_PTR)
#define DMA0_HRS                                 DMA_HRS_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI3                             DMA_DCHPRI3_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI2                             DMA_DCHPRI2_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI1                             DMA_DCHPRI1_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI0                             DMA_DCHPRI0_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI7                             DMA_DCHPRI7_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI6                             DMA_DCHPRI6_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI5                             DMA_DCHPRI5_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI4                             DMA_DCHPRI4_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI11                            DMA_DCHPRI11_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI10                            DMA_DCHPRI10_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI9                             DMA_DCHPRI9_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI8                             DMA_DCHPRI8_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI15                            DMA_DCHPRI15_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI14                            DMA_DCHPRI14_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI13                            DMA_DCHPRI13_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI12                            DMA_DCHPRI12_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI19                            DMA_DCHPRI19_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI18                            DMA_DCHPRI18_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI17                            DMA_DCHPRI17_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI16                            DMA_DCHPRI16_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI23                            DMA_DCHPRI23_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI22                            DMA_DCHPRI22_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI21                            DMA_DCHPRI21_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI20                            DMA_DCHPRI20_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI27                            DMA_DCHPRI27_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI26                            DMA_DCHPRI26_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI25                            DMA_DCHPRI25_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI24                            DMA_DCHPRI24_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI31                            DMA_DCHPRI31_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI30                            DMA_DCHPRI30_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI29                            DMA_DCHPRI29_REG(DMA0_BASE_PTR)
#define DMA0_DCHPRI28                            DMA_DCHPRI28_REG(DMA0_BASE_PTR)
#define DMA0_TCD0_SADDR                          DMA_SADDR_REG(DMA0_BASE_PTR,0)
#define DMA0_TCD0_SOFF                           DMA_SOFF_REG(DMA0_BASE_PTR,0)
#define DMA0_TCD0_ATTR                           DMA_ATTR_REG(DMA0_BASE_PTR,0)
#define DMA0_TCD0_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,0)
#define DMA0_TCD0_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,0)
#define DMA0_TCD0_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,0)
#define DMA0_TCD0_SLAST                          DMA_SLAST_REG(DMA0_BASE_PTR,0)
#define DMA0_TCD0_DADDR                          DMA_DADDR_REG(DMA0_BASE_PTR,0)
#define DMA0_TCD0_DOFF                           DMA_DOFF_REG(DMA0_BASE_PTR,0)
#define DMA0_TCD0_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,0)
#define DMA0_TCD0_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,0)
#define DMA0_TCD0_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0_BASE_PTR,0)
#define DMA0_TCD0_CSR                            DMA_CSR_REG(DMA0_BASE_PTR,0)
#define DMA0_TCD0_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,0)
#define DMA0_TCD0_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,0)
#define DMA0_TCD1_SADDR                          DMA_SADDR_REG(DMA0_BASE_PTR,1)
#define DMA0_TCD1_SOFF                           DMA_SOFF_REG(DMA0_BASE_PTR,1)
#define DMA0_TCD1_ATTR                           DMA_ATTR_REG(DMA0_BASE_PTR,1)
#define DMA0_TCD1_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,1)
#define DMA0_TCD1_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,1)
#define DMA0_TCD1_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,1)
#define DMA0_TCD1_SLAST                          DMA_SLAST_REG(DMA0_BASE_PTR,1)
#define DMA0_TCD1_DADDR                          DMA_DADDR_REG(DMA0_BASE_PTR,1)
#define DMA0_TCD1_DOFF                           DMA_DOFF_REG(DMA0_BASE_PTR,1)
#define DMA0_TCD1_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,1)
#define DMA0_TCD1_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,1)
#define DMA0_TCD1_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0_BASE_PTR,1)
#define DMA0_TCD1_CSR                            DMA_CSR_REG(DMA0_BASE_PTR,1)
#define DMA0_TCD1_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,1)
#define DMA0_TCD1_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,1)
#define DMA0_TCD2_SADDR                          DMA_SADDR_REG(DMA0_BASE_PTR,2)
#define DMA0_TCD2_SOFF                           DMA_SOFF_REG(DMA0_BASE_PTR,2)
#define DMA0_TCD2_ATTR                           DMA_ATTR_REG(DMA0_BASE_PTR,2)
#define DMA0_TCD2_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,2)
#define DMA0_TCD2_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,2)
#define DMA0_TCD2_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,2)
#define DMA0_TCD2_SLAST                          DMA_SLAST_REG(DMA0_BASE_PTR,2)
#define DMA0_TCD2_DADDR                          DMA_DADDR_REG(DMA0_BASE_PTR,2)
#define DMA0_TCD2_DOFF                           DMA_DOFF_REG(DMA0_BASE_PTR,2)
#define DMA0_TCD2_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,2)
#define DMA0_TCD2_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,2)
#define DMA0_TCD2_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0_BASE_PTR,2)
#define DMA0_TCD2_CSR                            DMA_CSR_REG(DMA0_BASE_PTR,2)
#define DMA0_TCD2_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,2)
#define DMA0_TCD2_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,2)
#define DMA0_TCD3_SADDR                          DMA_SADDR_REG(DMA0_BASE_PTR,3)
#define DMA0_TCD3_SOFF                           DMA_SOFF_REG(DMA0_BASE_PTR,3)
#define DMA0_TCD3_ATTR                           DMA_ATTR_REG(DMA0_BASE_PTR,3)
#define DMA0_TCD3_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,3)
#define DMA0_TCD3_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,3)
#define DMA0_TCD3_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,3)
#define DMA0_TCD3_SLAST                          DMA_SLAST_REG(DMA0_BASE_PTR,3)
#define DMA0_TCD3_DADDR                          DMA_DADDR_REG(DMA0_BASE_PTR,3)
#define DMA0_TCD3_DOFF                           DMA_DOFF_REG(DMA0_BASE_PTR,3)
#define DMA0_TCD3_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,3)
#define DMA0_TCD3_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,3)
#define DMA0_TCD3_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0_BASE_PTR,3)
#define DMA0_TCD3_CSR                            DMA_CSR_REG(DMA0_BASE_PTR,3)
#define DMA0_TCD3_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,3)
#define DMA0_TCD3_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,3)
#define DMA0_TCD4_SADDR                          DMA_SADDR_REG(DMA0_BASE_PTR,4)
#define DMA0_TCD4_SOFF                           DMA_SOFF_REG(DMA0_BASE_PTR,4)
#define DMA0_TCD4_ATTR                           DMA_ATTR_REG(DMA0_BASE_PTR,4)
#define DMA0_TCD4_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,4)
#define DMA0_TCD4_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,4)
#define DMA0_TCD4_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,4)
#define DMA0_TCD4_SLAST                          DMA_SLAST_REG(DMA0_BASE_PTR,4)
#define DMA0_TCD4_DADDR                          DMA_DADDR_REG(DMA0_BASE_PTR,4)
#define DMA0_TCD4_DOFF                           DMA_DOFF_REG(DMA0_BASE_PTR,4)
#define DMA0_TCD4_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,4)
#define DMA0_TCD4_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,4)
#define DMA0_TCD4_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0_BASE_PTR,4)
#define DMA0_TCD4_CSR                            DMA_CSR_REG(DMA0_BASE_PTR,4)
#define DMA0_TCD4_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,4)
#define DMA0_TCD4_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,4)
#define DMA0_TCD5_SADDR                          DMA_SADDR_REG(DMA0_BASE_PTR,5)
#define DMA0_TCD5_SOFF                           DMA_SOFF_REG(DMA0_BASE_PTR,5)
#define DMA0_TCD5_ATTR                           DMA_ATTR_REG(DMA0_BASE_PTR,5)
#define DMA0_TCD5_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,5)
#define DMA0_TCD5_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,5)
#define DMA0_TCD5_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,5)
#define DMA0_TCD5_SLAST                          DMA_SLAST_REG(DMA0_BASE_PTR,5)
#define DMA0_TCD5_DADDR                          DMA_DADDR_REG(DMA0_BASE_PTR,5)
#define DMA0_TCD5_DOFF                           DMA_DOFF_REG(DMA0_BASE_PTR,5)
#define DMA0_TCD5_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,5)
#define DMA0_TCD5_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,5)
#define DMA0_TCD5_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0_BASE_PTR,5)
#define DMA0_TCD5_CSR                            DMA_CSR_REG(DMA0_BASE_PTR,5)
#define DMA0_TCD5_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,5)
#define DMA0_TCD5_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,5)
#define DMA0_TCD6_SADDR                          DMA_SADDR_REG(DMA0_BASE_PTR,6)
#define DMA0_TCD6_SOFF                           DMA_SOFF_REG(DMA0_BASE_PTR,6)
#define DMA0_TCD6_ATTR                           DMA_ATTR_REG(DMA0_BASE_PTR,6)
#define DMA0_TCD6_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,6)
#define DMA0_TCD6_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,6)
#define DMA0_TCD6_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,6)
#define DMA0_TCD6_SLAST                          DMA_SLAST_REG(DMA0_BASE_PTR,6)
#define DMA0_TCD6_DADDR                          DMA_DADDR_REG(DMA0_BASE_PTR,6)
#define DMA0_TCD6_DOFF                           DMA_DOFF_REG(DMA0_BASE_PTR,6)
#define DMA0_TCD6_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,6)
#define DMA0_TCD6_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,6)
#define DMA0_TCD6_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0_BASE_PTR,6)
#define DMA0_TCD6_CSR                            DMA_CSR_REG(DMA0_BASE_PTR,6)
#define DMA0_TCD6_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,6)
#define DMA0_TCD6_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,6)
#define DMA0_TCD7_SADDR                          DMA_SADDR_REG(DMA0_BASE_PTR,7)
#define DMA0_TCD7_SOFF                           DMA_SOFF_REG(DMA0_BASE_PTR,7)
#define DMA0_TCD7_ATTR                           DMA_ATTR_REG(DMA0_BASE_PTR,7)
#define DMA0_TCD7_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,7)
#define DMA0_TCD7_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,7)
#define DMA0_TCD7_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,7)
#define DMA0_TCD7_SLAST                          DMA_SLAST_REG(DMA0_BASE_PTR,7)
#define DMA0_TCD7_DADDR                          DMA_DADDR_REG(DMA0_BASE_PTR,7)
#define DMA0_TCD7_DOFF                           DMA_DOFF_REG(DMA0_BASE_PTR,7)
#define DMA0_TCD7_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,7)
#define DMA0_TCD7_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,7)
#define DMA0_TCD7_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0_BASE_PTR,7)
#define DMA0_TCD7_CSR                            DMA_CSR_REG(DMA0_BASE_PTR,7)
#define DMA0_TCD7_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,7)
#define DMA0_TCD7_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,7)
#define DMA0_TCD8_SADDR                          DMA_SADDR_REG(DMA0_BASE_PTR,8)
#define DMA0_TCD8_SOFF                           DMA_SOFF_REG(DMA0_BASE_PTR,8)
#define DMA0_TCD8_ATTR                           DMA_ATTR_REG(DMA0_BASE_PTR,8)
#define DMA0_TCD8_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,8)
#define DMA0_TCD8_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,8)
#define DMA0_TCD8_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,8)
#define DMA0_TCD8_SLAST                          DMA_SLAST_REG(DMA0_BASE_PTR,8)
#define DMA0_TCD8_DADDR                          DMA_DADDR_REG(DMA0_BASE_PTR,8)
#define DMA0_TCD8_DOFF                           DMA_DOFF_REG(DMA0_BASE_PTR,8)
#define DMA0_TCD8_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,8)
#define DMA0_TCD8_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,8)
#define DMA0_TCD8_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0_BASE_PTR,8)
#define DMA0_TCD8_CSR                            DMA_CSR_REG(DMA0_BASE_PTR,8)
#define DMA0_TCD8_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,8)
#define DMA0_TCD8_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,8)
#define DMA0_TCD9_SADDR                          DMA_SADDR_REG(DMA0_BASE_PTR,9)
#define DMA0_TCD9_SOFF                           DMA_SOFF_REG(DMA0_BASE_PTR,9)
#define DMA0_TCD9_ATTR                           DMA_ATTR_REG(DMA0_BASE_PTR,9)
#define DMA0_TCD9_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,9)
#define DMA0_TCD9_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,9)
#define DMA0_TCD9_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,9)
#define DMA0_TCD9_SLAST                          DMA_SLAST_REG(DMA0_BASE_PTR,9)
#define DMA0_TCD9_DADDR                          DMA_DADDR_REG(DMA0_BASE_PTR,9)
#define DMA0_TCD9_DOFF                           DMA_DOFF_REG(DMA0_BASE_PTR,9)
#define DMA0_TCD9_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,9)
#define DMA0_TCD9_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,9)
#define DMA0_TCD9_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0_BASE_PTR,9)
#define DMA0_TCD9_CSR                            DMA_CSR_REG(DMA0_BASE_PTR,9)
#define DMA0_TCD9_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,9)
#define DMA0_TCD9_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,9)
#define DMA0_TCD10_SADDR                         DMA_SADDR_REG(DMA0_BASE_PTR,10)
#define DMA0_TCD10_SOFF                          DMA_SOFF_REG(DMA0_BASE_PTR,10)
#define DMA0_TCD10_ATTR                          DMA_ATTR_REG(DMA0_BASE_PTR,10)
#define DMA0_TCD10_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,10)
#define DMA0_TCD10_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,10)
#define DMA0_TCD10_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,10)
#define DMA0_TCD10_SLAST                         DMA_SLAST_REG(DMA0_BASE_PTR,10)
#define DMA0_TCD10_DADDR                         DMA_DADDR_REG(DMA0_BASE_PTR,10)
#define DMA0_TCD10_DOFF                          DMA_DOFF_REG(DMA0_BASE_PTR,10)
#define DMA0_TCD10_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,10)
#define DMA0_TCD10_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,10)
#define DMA0_TCD10_DLASTSGA                      DMA_DLAST_SGA_REG(DMA0_BASE_PTR,10)
#define DMA0_TCD10_CSR                           DMA_CSR_REG(DMA0_BASE_PTR,10)
#define DMA0_TCD10_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,10)
#define DMA0_TCD10_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,10)
#define DMA0_TCD11_SADDR                         DMA_SADDR_REG(DMA0_BASE_PTR,11)
#define DMA0_TCD11_SOFF                          DMA_SOFF_REG(DMA0_BASE_PTR,11)
#define DMA0_TCD11_ATTR                          DMA_ATTR_REG(DMA0_BASE_PTR,11)
#define DMA0_TCD11_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,11)
#define DMA0_TCD11_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,11)
#define DMA0_TCD11_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,11)
#define DMA0_TCD11_SLAST                         DMA_SLAST_REG(DMA0_BASE_PTR,11)
#define DMA0_TCD11_DADDR                         DMA_DADDR_REG(DMA0_BASE_PTR,11)
#define DMA0_TCD11_DOFF                          DMA_DOFF_REG(DMA0_BASE_PTR,11)
#define DMA0_TCD11_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,11)
#define DMA0_TCD11_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,11)
#define DMA0_TCD11_DLASTSGA                      DMA_DLAST_SGA_REG(DMA0_BASE_PTR,11)
#define DMA0_TCD11_CSR                           DMA_CSR_REG(DMA0_BASE_PTR,11)
#define DMA0_TCD11_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,11)
#define DMA0_TCD11_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,11)
#define DMA0_TCD12_SADDR                         DMA_SADDR_REG(DMA0_BASE_PTR,12)
#define DMA0_TCD12_SOFF                          DMA_SOFF_REG(DMA0_BASE_PTR,12)
#define DMA0_TCD12_ATTR                          DMA_ATTR_REG(DMA0_BASE_PTR,12)
#define DMA0_TCD12_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,12)
#define DMA0_TCD12_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,12)
#define DMA0_TCD12_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,12)
#define DMA0_TCD12_SLAST                         DMA_SLAST_REG(DMA0_BASE_PTR,12)
#define DMA0_TCD12_DADDR                         DMA_DADDR_REG(DMA0_BASE_PTR,12)
#define DMA0_TCD12_DOFF                          DMA_DOFF_REG(DMA0_BASE_PTR,12)
#define DMA0_TCD12_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,12)
#define DMA0_TCD12_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,12)
#define DMA0_TCD12_DLASTSGA                      DMA_DLAST_SGA_REG(DMA0_BASE_PTR,12)
#define DMA0_TCD12_CSR                           DMA_CSR_REG(DMA0_BASE_PTR,12)
#define DMA0_TCD12_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,12)
#define DMA0_TCD12_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,12)
#define DMA0_TCD13_SADDR                         DMA_SADDR_REG(DMA0_BASE_PTR,13)
#define DMA0_TCD13_SOFF                          DMA_SOFF_REG(DMA0_BASE_PTR,13)
#define DMA0_TCD13_ATTR                          DMA_ATTR_REG(DMA0_BASE_PTR,13)
#define DMA0_TCD13_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,13)
#define DMA0_TCD13_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,13)
#define DMA0_TCD13_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,13)
#define DMA0_TCD13_SLAST                         DMA_SLAST_REG(DMA0_BASE_PTR,13)
#define DMA0_TCD13_DADDR                         DMA_DADDR_REG(DMA0_BASE_PTR,13)
#define DMA0_TCD13_DOFF                          DMA_DOFF_REG(DMA0_BASE_PTR,13)
#define DMA0_TCD13_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,13)
#define DMA0_TCD13_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,13)
#define DMA0_TCD13_DLASTSGA                      DMA_DLAST_SGA_REG(DMA0_BASE_PTR,13)
#define DMA0_TCD13_CSR                           DMA_CSR_REG(DMA0_BASE_PTR,13)
#define DMA0_TCD13_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,13)
#define DMA0_TCD13_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,13)
#define DMA0_TCD14_SADDR                         DMA_SADDR_REG(DMA0_BASE_PTR,14)
#define DMA0_TCD14_SOFF                          DMA_SOFF_REG(DMA0_BASE_PTR,14)
#define DMA0_TCD14_ATTR                          DMA_ATTR_REG(DMA0_BASE_PTR,14)
#define DMA0_TCD14_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,14)
#define DMA0_TCD14_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,14)
#define DMA0_TCD14_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,14)
#define DMA0_TCD14_SLAST                         DMA_SLAST_REG(DMA0_BASE_PTR,14)
#define DMA0_TCD14_DADDR                         DMA_DADDR_REG(DMA0_BASE_PTR,14)
#define DMA0_TCD14_DOFF                          DMA_DOFF_REG(DMA0_BASE_PTR,14)
#define DMA0_TCD14_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,14)
#define DMA0_TCD14_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,14)
#define DMA0_TCD14_DLASTSGA                      DMA_DLAST_SGA_REG(DMA0_BASE_PTR,14)
#define DMA0_TCD14_CSR                           DMA_CSR_REG(DMA0_BASE_PTR,14)
#define DMA0_TCD14_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,14)
#define DMA0_TCD14_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,14)
#define DMA0_TCD15_SADDR                         DMA_SADDR_REG(DMA0_BASE_PTR,15)
#define DMA0_TCD15_SOFF                          DMA_SOFF_REG(DMA0_BASE_PTR,15)
#define DMA0_TCD15_ATTR                          DMA_ATTR_REG(DMA0_BASE_PTR,15)
#define DMA0_TCD15_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,15)
#define DMA0_TCD15_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,15)
#define DMA0_TCD15_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,15)
#define DMA0_TCD15_SLAST                         DMA_SLAST_REG(DMA0_BASE_PTR,15)
#define DMA0_TCD15_DADDR                         DMA_DADDR_REG(DMA0_BASE_PTR,15)
#define DMA0_TCD15_DOFF                          DMA_DOFF_REG(DMA0_BASE_PTR,15)
#define DMA0_TCD15_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,15)
#define DMA0_TCD15_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,15)
#define DMA0_TCD15_DLASTSGA                      DMA_DLAST_SGA_REG(DMA0_BASE_PTR,15)
#define DMA0_TCD15_CSR                           DMA_CSR_REG(DMA0_BASE_PTR,15)
#define DMA0_TCD15_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,15)
#define DMA0_TCD15_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,15)
#define DMA0_TCD16_SADDR                         DMA_SADDR_REG(DMA0_BASE_PTR,16)
#define DMA0_TCD16_SOFF                          DMA_SOFF_REG(DMA0_BASE_PTR,16)
#define DMA0_TCD16_ATTR                          DMA_ATTR_REG(DMA0_BASE_PTR,16)
#define DMA0_TCD16_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,16)
#define DMA0_TCD16_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,16)
#define DMA0_TCD16_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,16)
#define DMA0_TCD16_SLAST                         DMA_SLAST_REG(DMA0_BASE_PTR,16)
#define DMA0_TCD16_DADDR                         DMA_DADDR_REG(DMA0_BASE_PTR,16)
#define DMA0_TCD16_DOFF                          DMA_DOFF_REG(DMA0_BASE_PTR,16)
#define DMA0_TCD16_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,16)
#define DMA0_TCD16_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,16)
#define DMA0_TCD16_DLASTSGA                      DMA_DLAST_SGA_REG(DMA0_BASE_PTR,16)
#define DMA0_TCD16_CSR                           DMA_CSR_REG(DMA0_BASE_PTR,16)
#define DMA0_TCD16_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,16)
#define DMA0_TCD16_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,16)
#define DMA0_TCD17_SADDR                         DMA_SADDR_REG(DMA0_BASE_PTR,17)
#define DMA0_TCD17_SOFF                          DMA_SOFF_REG(DMA0_BASE_PTR,17)
#define DMA0_TCD17_ATTR                          DMA_ATTR_REG(DMA0_BASE_PTR,17)
#define DMA0_TCD17_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,17)
#define DMA0_TCD17_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,17)
#define DMA0_TCD17_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,17)
#define DMA0_TCD17_SLAST                         DMA_SLAST_REG(DMA0_BASE_PTR,17)
#define DMA0_TCD17_DADDR                         DMA_DADDR_REG(DMA0_BASE_PTR,17)
#define DMA0_TCD17_DOFF                          DMA_DOFF_REG(DMA0_BASE_PTR,17)
#define DMA0_TCD17_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,17)
#define DMA0_TCD17_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,17)
#define DMA0_TCD17_DLASTSGA                      DMA_DLAST_SGA_REG(DMA0_BASE_PTR,17)
#define DMA0_TCD17_CSR                           DMA_CSR_REG(DMA0_BASE_PTR,17)
#define DMA0_TCD17_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,17)
#define DMA0_TCD17_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,17)
#define DMA0_TCD18_SADDR                         DMA_SADDR_REG(DMA0_BASE_PTR,18)
#define DMA0_TCD18_SOFF                          DMA_SOFF_REG(DMA0_BASE_PTR,18)
#define DMA0_TCD18_ATTR                          DMA_ATTR_REG(DMA0_BASE_PTR,18)
#define DMA0_TCD18_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,18)
#define DMA0_TCD18_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,18)
#define DMA0_TCD18_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,18)
#define DMA0_TCD18_SLAST                         DMA_SLAST_REG(DMA0_BASE_PTR,18)
#define DMA0_TCD18_DADDR                         DMA_DADDR_REG(DMA0_BASE_PTR,18)
#define DMA0_TCD18_DOFF                          DMA_DOFF_REG(DMA0_BASE_PTR,18)
#define DMA0_TCD18_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,18)
#define DMA0_TCD18_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,18)
#define DMA0_TCD18_DLASTSGA                      DMA_DLAST_SGA_REG(DMA0_BASE_PTR,18)
#define DMA0_TCD18_CSR                           DMA_CSR_REG(DMA0_BASE_PTR,18)
#define DMA0_TCD18_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,18)
#define DMA0_TCD18_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,18)
#define DMA0_TCD19_SADDR                         DMA_SADDR_REG(DMA0_BASE_PTR,19)
#define DMA0_TCD19_SOFF                          DMA_SOFF_REG(DMA0_BASE_PTR,19)
#define DMA0_TCD19_ATTR                          DMA_ATTR_REG(DMA0_BASE_PTR,19)
#define DMA0_TCD19_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,19)
#define DMA0_TCD19_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,19)
#define DMA0_TCD19_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,19)
#define DMA0_TCD19_SLAST                         DMA_SLAST_REG(DMA0_BASE_PTR,19)
#define DMA0_TCD19_DADDR                         DMA_DADDR_REG(DMA0_BASE_PTR,19)
#define DMA0_TCD19_DOFF                          DMA_DOFF_REG(DMA0_BASE_PTR,19)
#define DMA0_TCD19_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,19)
#define DMA0_TCD19_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,19)
#define DMA0_TCD19_DLASTSGA                      DMA_DLAST_SGA_REG(DMA0_BASE_PTR,19)
#define DMA0_TCD19_CSR                           DMA_CSR_REG(DMA0_BASE_PTR,19)
#define DMA0_TCD19_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,19)
#define DMA0_TCD19_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,19)
#define DMA0_TCD20_SADDR                         DMA_SADDR_REG(DMA0_BASE_PTR,20)
#define DMA0_TCD20_SOFF                          DMA_SOFF_REG(DMA0_BASE_PTR,20)
#define DMA0_TCD20_ATTR                          DMA_ATTR_REG(DMA0_BASE_PTR,20)
#define DMA0_TCD20_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,20)
#define DMA0_TCD20_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,20)
#define DMA0_TCD20_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,20)
#define DMA0_TCD20_SLAST                         DMA_SLAST_REG(DMA0_BASE_PTR,20)
#define DMA0_TCD20_DADDR                         DMA_DADDR_REG(DMA0_BASE_PTR,20)
#define DMA0_TCD20_DOFF                          DMA_DOFF_REG(DMA0_BASE_PTR,20)
#define DMA0_TCD20_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,20)
#define DMA0_TCD20_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,20)
#define DMA0_TCD20_DLASTSGA                      DMA_DLAST_SGA_REG(DMA0_BASE_PTR,20)
#define DMA0_TCD20_CSR                           DMA_CSR_REG(DMA0_BASE_PTR,20)
#define DMA0_TCD20_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,20)
#define DMA0_TCD20_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,20)
#define DMA0_TCD21_SADDR                         DMA_SADDR_REG(DMA0_BASE_PTR,21)
#define DMA0_TCD21_SOFF                          DMA_SOFF_REG(DMA0_BASE_PTR,21)
#define DMA0_TCD21_ATTR                          DMA_ATTR_REG(DMA0_BASE_PTR,21)
#define DMA0_TCD21_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,21)
#define DMA0_TCD21_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,21)
#define DMA0_TCD21_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,21)
#define DMA0_TCD21_SLAST                         DMA_SLAST_REG(DMA0_BASE_PTR,21)
#define DMA0_TCD21_DADDR                         DMA_DADDR_REG(DMA0_BASE_PTR,21)
#define DMA0_TCD21_DOFF                          DMA_DOFF_REG(DMA0_BASE_PTR,21)
#define DMA0_TCD21_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,21)
#define DMA0_TCD21_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,21)
#define DMA0_TCD21_DLASTSGA                      DMA_DLAST_SGA_REG(DMA0_BASE_PTR,21)
#define DMA0_TCD21_CSR                           DMA_CSR_REG(DMA0_BASE_PTR,21)
#define DMA0_TCD21_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,21)
#define DMA0_TCD21_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,21)
#define DMA0_TCD22_SADDR                         DMA_SADDR_REG(DMA0_BASE_PTR,22)
#define DMA0_TCD22_SOFF                          DMA_SOFF_REG(DMA0_BASE_PTR,22)
#define DMA0_TCD22_ATTR                          DMA_ATTR_REG(DMA0_BASE_PTR,22)
#define DMA0_TCD22_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,22)
#define DMA0_TCD22_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,22)
#define DMA0_TCD22_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,22)
#define DMA0_TCD22_SLAST                         DMA_SLAST_REG(DMA0_BASE_PTR,22)
#define DMA0_TCD22_DADDR                         DMA_DADDR_REG(DMA0_BASE_PTR,22)
#define DMA0_TCD22_DOFF                          DMA_DOFF_REG(DMA0_BASE_PTR,22)
#define DMA0_TCD22_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,22)
#define DMA0_TCD22_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,22)
#define DMA0_TCD22_DLASTSGA                      DMA_DLAST_SGA_REG(DMA0_BASE_PTR,22)
#define DMA0_TCD22_CSR                           DMA_CSR_REG(DMA0_BASE_PTR,22)
#define DMA0_TCD22_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,22)
#define DMA0_TCD22_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,22)
#define DMA0_TCD23_SADDR                         DMA_SADDR_REG(DMA0_BASE_PTR,23)
#define DMA0_TCD23_SOFF                          DMA_SOFF_REG(DMA0_BASE_PTR,23)
#define DMA0_TCD23_ATTR                          DMA_ATTR_REG(DMA0_BASE_PTR,23)
#define DMA0_TCD23_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,23)
#define DMA0_TCD23_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,23)
#define DMA0_TCD23_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,23)
#define DMA0_TCD23_SLAST                         DMA_SLAST_REG(DMA0_BASE_PTR,23)
#define DMA0_TCD23_DADDR                         DMA_DADDR_REG(DMA0_BASE_PTR,23)
#define DMA0_TCD23_DOFF                          DMA_DOFF_REG(DMA0_BASE_PTR,23)
#define DMA0_TCD23_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,23)
#define DMA0_TCD23_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,23)
#define DMA0_TCD23_DLASTSGA                      DMA_DLAST_SGA_REG(DMA0_BASE_PTR,23)
#define DMA0_TCD23_CSR                           DMA_CSR_REG(DMA0_BASE_PTR,23)
#define DMA0_TCD23_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,23)
#define DMA0_TCD23_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,23)
#define DMA0_TCD24_SADDR                         DMA_SADDR_REG(DMA0_BASE_PTR,24)
#define DMA0_TCD24_SOFF                          DMA_SOFF_REG(DMA0_BASE_PTR,24)
#define DMA0_TCD24_ATTR                          DMA_ATTR_REG(DMA0_BASE_PTR,24)
#define DMA0_TCD24_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,24)
#define DMA0_TCD24_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,24)
#define DMA0_TCD24_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,24)
#define DMA0_TCD24_SLAST                         DMA_SLAST_REG(DMA0_BASE_PTR,24)
#define DMA0_TCD24_DADDR                         DMA_DADDR_REG(DMA0_BASE_PTR,24)
#define DMA0_TCD24_DOFF                          DMA_DOFF_REG(DMA0_BASE_PTR,24)
#define DMA0_TCD24_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,24)
#define DMA0_TCD24_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,24)
#define DMA0_TCD24_DLASTSGA                      DMA_DLAST_SGA_REG(DMA0_BASE_PTR,24)
#define DMA0_TCD24_CSR                           DMA_CSR_REG(DMA0_BASE_PTR,24)
#define DMA0_TCD24_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,24)
#define DMA0_TCD24_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,24)
#define DMA0_TCD25_SADDR                         DMA_SADDR_REG(DMA0_BASE_PTR,25)
#define DMA0_TCD25_SOFF                          DMA_SOFF_REG(DMA0_BASE_PTR,25)
#define DMA0_TCD25_ATTR                          DMA_ATTR_REG(DMA0_BASE_PTR,25)
#define DMA0_TCD25_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,25)
#define DMA0_TCD25_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,25)
#define DMA0_TCD25_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,25)
#define DMA0_TCD25_SLAST                         DMA_SLAST_REG(DMA0_BASE_PTR,25)
#define DMA0_TCD25_DADDR                         DMA_DADDR_REG(DMA0_BASE_PTR,25)
#define DMA0_TCD25_DOFF                          DMA_DOFF_REG(DMA0_BASE_PTR,25)
#define DMA0_TCD25_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,25)
#define DMA0_TCD25_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,25)
#define DMA0_TCD25_DLASTSGA                      DMA_DLAST_SGA_REG(DMA0_BASE_PTR,25)
#define DMA0_TCD25_CSR                           DMA_CSR_REG(DMA0_BASE_PTR,25)
#define DMA0_TCD25_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,25)
#define DMA0_TCD25_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,25)
#define DMA0_TCD26_SADDR                         DMA_SADDR_REG(DMA0_BASE_PTR,26)
#define DMA0_TCD26_SOFF                          DMA_SOFF_REG(DMA0_BASE_PTR,26)
#define DMA0_TCD26_ATTR                          DMA_ATTR_REG(DMA0_BASE_PTR,26)
#define DMA0_TCD26_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,26)
#define DMA0_TCD26_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,26)
#define DMA0_TCD26_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,26)
#define DMA0_TCD26_SLAST                         DMA_SLAST_REG(DMA0_BASE_PTR,26)
#define DMA0_TCD26_DADDR                         DMA_DADDR_REG(DMA0_BASE_PTR,26)
#define DMA0_TCD26_DOFF                          DMA_DOFF_REG(DMA0_BASE_PTR,26)
#define DMA0_TCD26_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,26)
#define DMA0_TCD26_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,26)
#define DMA0_TCD26_DLASTSGA                      DMA_DLAST_SGA_REG(DMA0_BASE_PTR,26)
#define DMA0_TCD26_CSR                           DMA_CSR_REG(DMA0_BASE_PTR,26)
#define DMA0_TCD26_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,26)
#define DMA0_TCD26_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,26)
#define DMA0_TCD27_SADDR                         DMA_SADDR_REG(DMA0_BASE_PTR,27)
#define DMA0_TCD27_SOFF                          DMA_SOFF_REG(DMA0_BASE_PTR,27)
#define DMA0_TCD27_ATTR                          DMA_ATTR_REG(DMA0_BASE_PTR,27)
#define DMA0_TCD27_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,27)
#define DMA0_TCD27_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,27)
#define DMA0_TCD27_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,27)
#define DMA0_TCD27_SLAST                         DMA_SLAST_REG(DMA0_BASE_PTR,27)
#define DMA0_TCD27_DADDR                         DMA_DADDR_REG(DMA0_BASE_PTR,27)
#define DMA0_TCD27_DOFF                          DMA_DOFF_REG(DMA0_BASE_PTR,27)
#define DMA0_TCD27_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,27)
#define DMA0_TCD27_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,27)
#define DMA0_TCD27_DLASTSGA                      DMA_DLAST_SGA_REG(DMA0_BASE_PTR,27)
#define DMA0_TCD27_CSR                           DMA_CSR_REG(DMA0_BASE_PTR,27)
#define DMA0_TCD27_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,27)
#define DMA0_TCD27_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,27)
#define DMA0_TCD28_SADDR                         DMA_SADDR_REG(DMA0_BASE_PTR,28)
#define DMA0_TCD28_SOFF                          DMA_SOFF_REG(DMA0_BASE_PTR,28)
#define DMA0_TCD28_ATTR                          DMA_ATTR_REG(DMA0_BASE_PTR,28)
#define DMA0_TCD28_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,28)
#define DMA0_TCD28_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,28)
#define DMA0_TCD28_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,28)
#define DMA0_TCD28_SLAST                         DMA_SLAST_REG(DMA0_BASE_PTR,28)
#define DMA0_TCD28_DADDR                         DMA_DADDR_REG(DMA0_BASE_PTR,28)
#define DMA0_TCD28_DOFF                          DMA_DOFF_REG(DMA0_BASE_PTR,28)
#define DMA0_TCD28_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,28)
#define DMA0_TCD28_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,28)
#define DMA0_TCD28_DLASTSGA                      DMA_DLAST_SGA_REG(DMA0_BASE_PTR,28)
#define DMA0_TCD28_CSR                           DMA_CSR_REG(DMA0_BASE_PTR,28)
#define DMA0_TCD28_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,28)
#define DMA0_TCD28_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,28)
#define DMA0_TCD29_SADDR                         DMA_SADDR_REG(DMA0_BASE_PTR,29)
#define DMA0_TCD29_SOFF                          DMA_SOFF_REG(DMA0_BASE_PTR,29)
#define DMA0_TCD29_ATTR                          DMA_ATTR_REG(DMA0_BASE_PTR,29)
#define DMA0_TCD29_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,29)
#define DMA0_TCD29_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,29)
#define DMA0_TCD29_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,29)
#define DMA0_TCD29_SLAST                         DMA_SLAST_REG(DMA0_BASE_PTR,29)
#define DMA0_TCD29_DADDR                         DMA_DADDR_REG(DMA0_BASE_PTR,29)
#define DMA0_TCD29_DOFF                          DMA_DOFF_REG(DMA0_BASE_PTR,29)
#define DMA0_TCD29_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,29)
#define DMA0_TCD29_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,29)
#define DMA0_TCD29_DLASTSGA                      DMA_DLAST_SGA_REG(DMA0_BASE_PTR,29)
#define DMA0_TCD29_CSR                           DMA_CSR_REG(DMA0_BASE_PTR,29)
#define DMA0_TCD29_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,29)
#define DMA0_TCD29_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,29)
#define DMA0_TCD30_SADDR                         DMA_SADDR_REG(DMA0_BASE_PTR,30)
#define DMA0_TCD30_SOFF                          DMA_SOFF_REG(DMA0_BASE_PTR,30)
#define DMA0_TCD30_ATTR                          DMA_ATTR_REG(DMA0_BASE_PTR,30)
#define DMA0_TCD30_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,30)
#define DMA0_TCD30_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,30)
#define DMA0_TCD30_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,30)
#define DMA0_TCD30_SLAST                         DMA_SLAST_REG(DMA0_BASE_PTR,30)
#define DMA0_TCD30_DADDR                         DMA_DADDR_REG(DMA0_BASE_PTR,30)
#define DMA0_TCD30_DOFF                          DMA_DOFF_REG(DMA0_BASE_PTR,30)
#define DMA0_TCD30_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,30)
#define DMA0_TCD30_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,30)
#define DMA0_TCD30_DLASTSGA                      DMA_DLAST_SGA_REG(DMA0_BASE_PTR,30)
#define DMA0_TCD30_CSR                           DMA_CSR_REG(DMA0_BASE_PTR,30)
#define DMA0_TCD30_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,30)
#define DMA0_TCD30_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,30)
#define DMA0_TCD31_SADDR                         DMA_SADDR_REG(DMA0_BASE_PTR,31)
#define DMA0_TCD31_SOFF                          DMA_SOFF_REG(DMA0_BASE_PTR,31)
#define DMA0_TCD31_ATTR                          DMA_ATTR_REG(DMA0_BASE_PTR,31)
#define DMA0_TCD31_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,31)
#define DMA0_TCD31_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,31)
#define DMA0_TCD31_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,31)
#define DMA0_TCD31_SLAST                         DMA_SLAST_REG(DMA0_BASE_PTR,31)
#define DMA0_TCD31_DADDR                         DMA_DADDR_REG(DMA0_BASE_PTR,31)
#define DMA0_TCD31_DOFF                          DMA_DOFF_REG(DMA0_BASE_PTR,31)
#define DMA0_TCD31_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,31)
#define DMA0_TCD31_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,31)
#define DMA0_TCD31_DLASTSGA                      DMA_DLAST_SGA_REG(DMA0_BASE_PTR,31)
#define DMA0_TCD31_CSR                           DMA_CSR_REG(DMA0_BASE_PTR,31)
#define DMA0_TCD31_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,31)
#define DMA0_TCD31_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,31)
/* DMA1 */
#define DMA1_CR                                  DMA_CR_REG(DMA1_BASE_PTR)
#define DMA1_ES                                  DMA_ES_REG(DMA1_BASE_PTR)
#define DMA1_ERQ                                 DMA_ERQ_REG(DMA1_BASE_PTR)
#define DMA1_EEI                                 DMA_EEI_REG(DMA1_BASE_PTR)
#define DMA1_CEEI                                DMA_CEEI_REG(DMA1_BASE_PTR)
#define DMA1_SEEI                                DMA_SEEI_REG(DMA1_BASE_PTR)
#define DMA1_CERQ                                DMA_CERQ_REG(DMA1_BASE_PTR)
#define DMA1_SERQ                                DMA_SERQ_REG(DMA1_BASE_PTR)
#define DMA1_CDNE                                DMA_CDNE_REG(DMA1_BASE_PTR)
#define DMA1_SSRT                                DMA_SSRT_REG(DMA1_BASE_PTR)
#define DMA1_CERR                                DMA_CERR_REG(DMA1_BASE_PTR)
#define DMA1_CINT                                DMA_CINT_REG(DMA1_BASE_PTR)
#define DMA1_INT                                 DMA_INT_REG(DMA1_BASE_PTR)
#define DMA1_ERR                                 DMA_ERR_REG(DMA1_BASE_PTR)
#define DMA1_HRS                                 DMA_HRS_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI3                             DMA_DCHPRI3_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI2                             DMA_DCHPRI2_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI1                             DMA_DCHPRI1_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI0                             DMA_DCHPRI0_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI7                             DMA_DCHPRI7_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI6                             DMA_DCHPRI6_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI5                             DMA_DCHPRI5_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI4                             DMA_DCHPRI4_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI11                            DMA_DCHPRI11_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI10                            DMA_DCHPRI10_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI9                             DMA_DCHPRI9_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI8                             DMA_DCHPRI8_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI15                            DMA_DCHPRI15_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI14                            DMA_DCHPRI14_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI13                            DMA_DCHPRI13_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI12                            DMA_DCHPRI12_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI19                            DMA_DCHPRI19_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI18                            DMA_DCHPRI18_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI17                            DMA_DCHPRI17_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI16                            DMA_DCHPRI16_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI23                            DMA_DCHPRI23_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI22                            DMA_DCHPRI22_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI21                            DMA_DCHPRI21_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI20                            DMA_DCHPRI20_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI27                            DMA_DCHPRI27_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI26                            DMA_DCHPRI26_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI25                            DMA_DCHPRI25_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI24                            DMA_DCHPRI24_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI31                            DMA_DCHPRI31_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI30                            DMA_DCHPRI30_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI29                            DMA_DCHPRI29_REG(DMA1_BASE_PTR)
#define DMA1_DCHPRI28                            DMA_DCHPRI28_REG(DMA1_BASE_PTR)
#define DMA1_TCD0_SADDR                          DMA_SADDR_REG(DMA1_BASE_PTR,0)
#define DMA1_TCD0_SOFF                           DMA_SOFF_REG(DMA1_BASE_PTR,0)
#define DMA1_TCD0_ATTR                           DMA_ATTR_REG(DMA1_BASE_PTR,0)
#define DMA1_TCD0_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,0)
#define DMA1_TCD0_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,0)
#define DMA1_TCD0_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,0)
#define DMA1_TCD0_SLAST                          DMA_SLAST_REG(DMA1_BASE_PTR,0)
#define DMA1_TCD0_DADDR                          DMA_DADDR_REG(DMA1_BASE_PTR,0)
#define DMA1_TCD0_DOFF                           DMA_DOFF_REG(DMA1_BASE_PTR,0)
#define DMA1_TCD0_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,0)
#define DMA1_TCD0_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,0)
#define DMA1_TCD0_DLASTSGA                       DMA_DLAST_SGA_REG(DMA1_BASE_PTR,0)
#define DMA1_TCD0_CSR                            DMA_CSR_REG(DMA1_BASE_PTR,0)
#define DMA1_TCD0_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,0)
#define DMA1_TCD0_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,0)
#define DMA1_TCD1_SADDR                          DMA_SADDR_REG(DMA1_BASE_PTR,1)
#define DMA1_TCD1_SOFF                           DMA_SOFF_REG(DMA1_BASE_PTR,1)
#define DMA1_TCD1_ATTR                           DMA_ATTR_REG(DMA1_BASE_PTR,1)
#define DMA1_TCD1_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,1)
#define DMA1_TCD1_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,1)
#define DMA1_TCD1_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,1)
#define DMA1_TCD1_SLAST                          DMA_SLAST_REG(DMA1_BASE_PTR,1)
#define DMA1_TCD1_DADDR                          DMA_DADDR_REG(DMA1_BASE_PTR,1)
#define DMA1_TCD1_DOFF                           DMA_DOFF_REG(DMA1_BASE_PTR,1)
#define DMA1_TCD1_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,1)
#define DMA1_TCD1_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,1)
#define DMA1_TCD1_DLASTSGA                       DMA_DLAST_SGA_REG(DMA1_BASE_PTR,1)
#define DMA1_TCD1_CSR                            DMA_CSR_REG(DMA1_BASE_PTR,1)
#define DMA1_TCD1_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,1)
#define DMA1_TCD1_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,1)
#define DMA1_TCD2_SADDR                          DMA_SADDR_REG(DMA1_BASE_PTR,2)
#define DMA1_TCD2_SOFF                           DMA_SOFF_REG(DMA1_BASE_PTR,2)
#define DMA1_TCD2_ATTR                           DMA_ATTR_REG(DMA1_BASE_PTR,2)
#define DMA1_TCD2_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,2)
#define DMA1_TCD2_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,2)
#define DMA1_TCD2_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,2)
#define DMA1_TCD2_SLAST                          DMA_SLAST_REG(DMA1_BASE_PTR,2)
#define DMA1_TCD2_DADDR                          DMA_DADDR_REG(DMA1_BASE_PTR,2)
#define DMA1_TCD2_DOFF                           DMA_DOFF_REG(DMA1_BASE_PTR,2)
#define DMA1_TCD2_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,2)
#define DMA1_TCD2_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,2)
#define DMA1_TCD2_DLASTSGA                       DMA_DLAST_SGA_REG(DMA1_BASE_PTR,2)
#define DMA1_TCD2_CSR                            DMA_CSR_REG(DMA1_BASE_PTR,2)
#define DMA1_TCD2_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,2)
#define DMA1_TCD2_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,2)
#define DMA1_TCD3_SADDR                          DMA_SADDR_REG(DMA1_BASE_PTR,3)
#define DMA1_TCD3_SOFF                           DMA_SOFF_REG(DMA1_BASE_PTR,3)
#define DMA1_TCD3_ATTR                           DMA_ATTR_REG(DMA1_BASE_PTR,3)
#define DMA1_TCD3_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,3)
#define DMA1_TCD3_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,3)
#define DMA1_TCD3_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,3)
#define DMA1_TCD3_SLAST                          DMA_SLAST_REG(DMA1_BASE_PTR,3)
#define DMA1_TCD3_DADDR                          DMA_DADDR_REG(DMA1_BASE_PTR,3)
#define DMA1_TCD3_DOFF                           DMA_DOFF_REG(DMA1_BASE_PTR,3)
#define DMA1_TCD3_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,3)
#define DMA1_TCD3_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,3)
#define DMA1_TCD3_DLASTSGA                       DMA_DLAST_SGA_REG(DMA1_BASE_PTR,3)
#define DMA1_TCD3_CSR                            DMA_CSR_REG(DMA1_BASE_PTR,3)
#define DMA1_TCD3_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,3)
#define DMA1_TCD3_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,3)
#define DMA1_TCD4_SADDR                          DMA_SADDR_REG(DMA1_BASE_PTR,4)
#define DMA1_TCD4_SOFF                           DMA_SOFF_REG(DMA1_BASE_PTR,4)
#define DMA1_TCD4_ATTR                           DMA_ATTR_REG(DMA1_BASE_PTR,4)
#define DMA1_TCD4_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,4)
#define DMA1_TCD4_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,4)
#define DMA1_TCD4_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,4)
#define DMA1_TCD4_SLAST                          DMA_SLAST_REG(DMA1_BASE_PTR,4)
#define DMA1_TCD4_DADDR                          DMA_DADDR_REG(DMA1_BASE_PTR,4)
#define DMA1_TCD4_DOFF                           DMA_DOFF_REG(DMA1_BASE_PTR,4)
#define DMA1_TCD4_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,4)
#define DMA1_TCD4_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,4)
#define DMA1_TCD4_DLASTSGA                       DMA_DLAST_SGA_REG(DMA1_BASE_PTR,4)
#define DMA1_TCD4_CSR                            DMA_CSR_REG(DMA1_BASE_PTR,4)
#define DMA1_TCD4_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,4)
#define DMA1_TCD4_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,4)
#define DMA1_TCD5_SADDR                          DMA_SADDR_REG(DMA1_BASE_PTR,5)
#define DMA1_TCD5_SOFF                           DMA_SOFF_REG(DMA1_BASE_PTR,5)
#define DMA1_TCD5_ATTR                           DMA_ATTR_REG(DMA1_BASE_PTR,5)
#define DMA1_TCD5_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,5)
#define DMA1_TCD5_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,5)
#define DMA1_TCD5_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,5)
#define DMA1_TCD5_SLAST                          DMA_SLAST_REG(DMA1_BASE_PTR,5)
#define DMA1_TCD5_DADDR                          DMA_DADDR_REG(DMA1_BASE_PTR,5)
#define DMA1_TCD5_DOFF                           DMA_DOFF_REG(DMA1_BASE_PTR,5)
#define DMA1_TCD5_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,5)
#define DMA1_TCD5_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,5)
#define DMA1_TCD5_DLASTSGA                       DMA_DLAST_SGA_REG(DMA1_BASE_PTR,5)
#define DMA1_TCD5_CSR                            DMA_CSR_REG(DMA1_BASE_PTR,5)
#define DMA1_TCD5_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,5)
#define DMA1_TCD5_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,5)
#define DMA1_TCD6_SADDR                          DMA_SADDR_REG(DMA1_BASE_PTR,6)
#define DMA1_TCD6_SOFF                           DMA_SOFF_REG(DMA1_BASE_PTR,6)
#define DMA1_TCD6_ATTR                           DMA_ATTR_REG(DMA1_BASE_PTR,6)
#define DMA1_TCD6_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,6)
#define DMA1_TCD6_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,6)
#define DMA1_TCD6_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,6)
#define DMA1_TCD6_SLAST                          DMA_SLAST_REG(DMA1_BASE_PTR,6)
#define DMA1_TCD6_DADDR                          DMA_DADDR_REG(DMA1_BASE_PTR,6)
#define DMA1_TCD6_DOFF                           DMA_DOFF_REG(DMA1_BASE_PTR,6)
#define DMA1_TCD6_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,6)
#define DMA1_TCD6_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,6)
#define DMA1_TCD6_DLASTSGA                       DMA_DLAST_SGA_REG(DMA1_BASE_PTR,6)
#define DMA1_TCD6_CSR                            DMA_CSR_REG(DMA1_BASE_PTR,6)
#define DMA1_TCD6_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,6)
#define DMA1_TCD6_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,6)
#define DMA1_TCD7_SADDR                          DMA_SADDR_REG(DMA1_BASE_PTR,7)
#define DMA1_TCD7_SOFF                           DMA_SOFF_REG(DMA1_BASE_PTR,7)
#define DMA1_TCD7_ATTR                           DMA_ATTR_REG(DMA1_BASE_PTR,7)
#define DMA1_TCD7_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,7)
#define DMA1_TCD7_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,7)
#define DMA1_TCD7_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,7)
#define DMA1_TCD7_SLAST                          DMA_SLAST_REG(DMA1_BASE_PTR,7)
#define DMA1_TCD7_DADDR                          DMA_DADDR_REG(DMA1_BASE_PTR,7)
#define DMA1_TCD7_DOFF                           DMA_DOFF_REG(DMA1_BASE_PTR,7)
#define DMA1_TCD7_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,7)
#define DMA1_TCD7_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,7)
#define DMA1_TCD7_DLASTSGA                       DMA_DLAST_SGA_REG(DMA1_BASE_PTR,7)
#define DMA1_TCD7_CSR                            DMA_CSR_REG(DMA1_BASE_PTR,7)
#define DMA1_TCD7_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,7)
#define DMA1_TCD7_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,7)
#define DMA1_TCD8_SADDR                          DMA_SADDR_REG(DMA1_BASE_PTR,8)
#define DMA1_TCD8_SOFF                           DMA_SOFF_REG(DMA1_BASE_PTR,8)
#define DMA1_TCD8_ATTR                           DMA_ATTR_REG(DMA1_BASE_PTR,8)
#define DMA1_TCD8_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,8)
#define DMA1_TCD8_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,8)
#define DMA1_TCD8_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,8)
#define DMA1_TCD8_SLAST                          DMA_SLAST_REG(DMA1_BASE_PTR,8)
#define DMA1_TCD8_DADDR                          DMA_DADDR_REG(DMA1_BASE_PTR,8)
#define DMA1_TCD8_DOFF                           DMA_DOFF_REG(DMA1_BASE_PTR,8)
#define DMA1_TCD8_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,8)
#define DMA1_TCD8_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,8)
#define DMA1_TCD8_DLASTSGA                       DMA_DLAST_SGA_REG(DMA1_BASE_PTR,8)
#define DMA1_TCD8_CSR                            DMA_CSR_REG(DMA1_BASE_PTR,8)
#define DMA1_TCD8_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,8)
#define DMA1_TCD8_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,8)
#define DMA1_TCD9_SADDR                          DMA_SADDR_REG(DMA1_BASE_PTR,9)
#define DMA1_TCD9_SOFF                           DMA_SOFF_REG(DMA1_BASE_PTR,9)
#define DMA1_TCD9_ATTR                           DMA_ATTR_REG(DMA1_BASE_PTR,9)
#define DMA1_TCD9_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,9)
#define DMA1_TCD9_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,9)
#define DMA1_TCD9_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,9)
#define DMA1_TCD9_SLAST                          DMA_SLAST_REG(DMA1_BASE_PTR,9)
#define DMA1_TCD9_DADDR                          DMA_DADDR_REG(DMA1_BASE_PTR,9)
#define DMA1_TCD9_DOFF                           DMA_DOFF_REG(DMA1_BASE_PTR,9)
#define DMA1_TCD9_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,9)
#define DMA1_TCD9_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,9)
#define DMA1_TCD9_DLASTSGA                       DMA_DLAST_SGA_REG(DMA1_BASE_PTR,9)
#define DMA1_TCD9_CSR                            DMA_CSR_REG(DMA1_BASE_PTR,9)
#define DMA1_TCD9_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,9)
#define DMA1_TCD9_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,9)
#define DMA1_TCD10_SADDR                         DMA_SADDR_REG(DMA1_BASE_PTR,10)
#define DMA1_TCD10_SOFF                          DMA_SOFF_REG(DMA1_BASE_PTR,10)
#define DMA1_TCD10_ATTR                          DMA_ATTR_REG(DMA1_BASE_PTR,10)
#define DMA1_TCD10_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,10)
#define DMA1_TCD10_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,10)
#define DMA1_TCD10_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,10)
#define DMA1_TCD10_SLAST                         DMA_SLAST_REG(DMA1_BASE_PTR,10)
#define DMA1_TCD10_DADDR                         DMA_DADDR_REG(DMA1_BASE_PTR,10)
#define DMA1_TCD10_DOFF                          DMA_DOFF_REG(DMA1_BASE_PTR,10)
#define DMA1_TCD10_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,10)
#define DMA1_TCD10_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,10)
#define DMA1_TCD10_DLASTSGA                      DMA_DLAST_SGA_REG(DMA1_BASE_PTR,10)
#define DMA1_TCD10_CSR                           DMA_CSR_REG(DMA1_BASE_PTR,10)
#define DMA1_TCD10_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,10)
#define DMA1_TCD10_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,10)
#define DMA1_TCD11_SADDR                         DMA_SADDR_REG(DMA1_BASE_PTR,11)
#define DMA1_TCD11_SOFF                          DMA_SOFF_REG(DMA1_BASE_PTR,11)
#define DMA1_TCD11_ATTR                          DMA_ATTR_REG(DMA1_BASE_PTR,11)
#define DMA1_TCD11_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,11)
#define DMA1_TCD11_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,11)
#define DMA1_TCD11_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,11)
#define DMA1_TCD11_SLAST                         DMA_SLAST_REG(DMA1_BASE_PTR,11)
#define DMA1_TCD11_DADDR                         DMA_DADDR_REG(DMA1_BASE_PTR,11)
#define DMA1_TCD11_DOFF                          DMA_DOFF_REG(DMA1_BASE_PTR,11)
#define DMA1_TCD11_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,11)
#define DMA1_TCD11_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,11)
#define DMA1_TCD11_DLASTSGA                      DMA_DLAST_SGA_REG(DMA1_BASE_PTR,11)
#define DMA1_TCD11_CSR                           DMA_CSR_REG(DMA1_BASE_PTR,11)
#define DMA1_TCD11_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,11)
#define DMA1_TCD11_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,11)
#define DMA1_TCD12_SADDR                         DMA_SADDR_REG(DMA1_BASE_PTR,12)
#define DMA1_TCD12_SOFF                          DMA_SOFF_REG(DMA1_BASE_PTR,12)
#define DMA1_TCD12_ATTR                          DMA_ATTR_REG(DMA1_BASE_PTR,12)
#define DMA1_TCD12_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,12)
#define DMA1_TCD12_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,12)
#define DMA1_TCD12_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,12)
#define DMA1_TCD12_SLAST                         DMA_SLAST_REG(DMA1_BASE_PTR,12)
#define DMA1_TCD12_DADDR                         DMA_DADDR_REG(DMA1_BASE_PTR,12)
#define DMA1_TCD12_DOFF                          DMA_DOFF_REG(DMA1_BASE_PTR,12)
#define DMA1_TCD12_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,12)
#define DMA1_TCD12_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,12)
#define DMA1_TCD12_DLASTSGA                      DMA_DLAST_SGA_REG(DMA1_BASE_PTR,12)
#define DMA1_TCD12_CSR                           DMA_CSR_REG(DMA1_BASE_PTR,12)
#define DMA1_TCD12_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,12)
#define DMA1_TCD12_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,12)
#define DMA1_TCD13_SADDR                         DMA_SADDR_REG(DMA1_BASE_PTR,13)
#define DMA1_TCD13_SOFF                          DMA_SOFF_REG(DMA1_BASE_PTR,13)
#define DMA1_TCD13_ATTR                          DMA_ATTR_REG(DMA1_BASE_PTR,13)
#define DMA1_TCD13_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,13)
#define DMA1_TCD13_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,13)
#define DMA1_TCD13_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,13)
#define DMA1_TCD13_SLAST                         DMA_SLAST_REG(DMA1_BASE_PTR,13)
#define DMA1_TCD13_DADDR                         DMA_DADDR_REG(DMA1_BASE_PTR,13)
#define DMA1_TCD13_DOFF                          DMA_DOFF_REG(DMA1_BASE_PTR,13)
#define DMA1_TCD13_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,13)
#define DMA1_TCD13_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,13)
#define DMA1_TCD13_DLASTSGA                      DMA_DLAST_SGA_REG(DMA1_BASE_PTR,13)
#define DMA1_TCD13_CSR                           DMA_CSR_REG(DMA1_BASE_PTR,13)
#define DMA1_TCD13_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,13)
#define DMA1_TCD13_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,13)
#define DMA1_TCD14_SADDR                         DMA_SADDR_REG(DMA1_BASE_PTR,14)
#define DMA1_TCD14_SOFF                          DMA_SOFF_REG(DMA1_BASE_PTR,14)
#define DMA1_TCD14_ATTR                          DMA_ATTR_REG(DMA1_BASE_PTR,14)
#define DMA1_TCD14_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,14)
#define DMA1_TCD14_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,14)
#define DMA1_TCD14_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,14)
#define DMA1_TCD14_SLAST                         DMA_SLAST_REG(DMA1_BASE_PTR,14)
#define DMA1_TCD14_DADDR                         DMA_DADDR_REG(DMA1_BASE_PTR,14)
#define DMA1_TCD14_DOFF                          DMA_DOFF_REG(DMA1_BASE_PTR,14)
#define DMA1_TCD14_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,14)
#define DMA1_TCD14_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,14)
#define DMA1_TCD14_DLASTSGA                      DMA_DLAST_SGA_REG(DMA1_BASE_PTR,14)
#define DMA1_TCD14_CSR                           DMA_CSR_REG(DMA1_BASE_PTR,14)
#define DMA1_TCD14_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,14)
#define DMA1_TCD14_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,14)
#define DMA1_TCD15_SADDR                         DMA_SADDR_REG(DMA1_BASE_PTR,15)
#define DMA1_TCD15_SOFF                          DMA_SOFF_REG(DMA1_BASE_PTR,15)
#define DMA1_TCD15_ATTR                          DMA_ATTR_REG(DMA1_BASE_PTR,15)
#define DMA1_TCD15_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,15)
#define DMA1_TCD15_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,15)
#define DMA1_TCD15_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,15)
#define DMA1_TCD15_SLAST                         DMA_SLAST_REG(DMA1_BASE_PTR,15)
#define DMA1_TCD15_DADDR                         DMA_DADDR_REG(DMA1_BASE_PTR,15)
#define DMA1_TCD15_DOFF                          DMA_DOFF_REG(DMA1_BASE_PTR,15)
#define DMA1_TCD15_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,15)
#define DMA1_TCD15_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,15)
#define DMA1_TCD15_DLASTSGA                      DMA_DLAST_SGA_REG(DMA1_BASE_PTR,15)
#define DMA1_TCD15_CSR                           DMA_CSR_REG(DMA1_BASE_PTR,15)
#define DMA1_TCD15_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,15)
#define DMA1_TCD15_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,15)
#define DMA1_TCD16_SADDR                         DMA_SADDR_REG(DMA1_BASE_PTR,16)
#define DMA1_TCD16_SOFF                          DMA_SOFF_REG(DMA1_BASE_PTR,16)
#define DMA1_TCD16_ATTR                          DMA_ATTR_REG(DMA1_BASE_PTR,16)
#define DMA1_TCD16_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,16)
#define DMA1_TCD16_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,16)
#define DMA1_TCD16_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,16)
#define DMA1_TCD16_SLAST                         DMA_SLAST_REG(DMA1_BASE_PTR,16)
#define DMA1_TCD16_DADDR                         DMA_DADDR_REG(DMA1_BASE_PTR,16)
#define DMA1_TCD16_DOFF                          DMA_DOFF_REG(DMA1_BASE_PTR,16)
#define DMA1_TCD16_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,16)
#define DMA1_TCD16_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,16)
#define DMA1_TCD16_DLASTSGA                      DMA_DLAST_SGA_REG(DMA1_BASE_PTR,16)
#define DMA1_TCD16_CSR                           DMA_CSR_REG(DMA1_BASE_PTR,16)
#define DMA1_TCD16_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,16)
#define DMA1_TCD16_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,16)
#define DMA1_TCD17_SADDR                         DMA_SADDR_REG(DMA1_BASE_PTR,17)
#define DMA1_TCD17_SOFF                          DMA_SOFF_REG(DMA1_BASE_PTR,17)
#define DMA1_TCD17_ATTR                          DMA_ATTR_REG(DMA1_BASE_PTR,17)
#define DMA1_TCD17_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,17)
#define DMA1_TCD17_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,17)
#define DMA1_TCD17_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,17)
#define DMA1_TCD17_SLAST                         DMA_SLAST_REG(DMA1_BASE_PTR,17)
#define DMA1_TCD17_DADDR                         DMA_DADDR_REG(DMA1_BASE_PTR,17)
#define DMA1_TCD17_DOFF                          DMA_DOFF_REG(DMA1_BASE_PTR,17)
#define DMA1_TCD17_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,17)
#define DMA1_TCD17_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,17)
#define DMA1_TCD17_DLASTSGA                      DMA_DLAST_SGA_REG(DMA1_BASE_PTR,17)
#define DMA1_TCD17_CSR                           DMA_CSR_REG(DMA1_BASE_PTR,17)
#define DMA1_TCD17_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,17)
#define DMA1_TCD17_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,17)
#define DMA1_TCD18_SADDR                         DMA_SADDR_REG(DMA1_BASE_PTR,18)
#define DMA1_TCD18_SOFF                          DMA_SOFF_REG(DMA1_BASE_PTR,18)
#define DMA1_TCD18_ATTR                          DMA_ATTR_REG(DMA1_BASE_PTR,18)
#define DMA1_TCD18_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,18)
#define DMA1_TCD18_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,18)
#define DMA1_TCD18_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,18)
#define DMA1_TCD18_SLAST                         DMA_SLAST_REG(DMA1_BASE_PTR,18)
#define DMA1_TCD18_DADDR                         DMA_DADDR_REG(DMA1_BASE_PTR,18)
#define DMA1_TCD18_DOFF                          DMA_DOFF_REG(DMA1_BASE_PTR,18)
#define DMA1_TCD18_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,18)
#define DMA1_TCD18_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,18)
#define DMA1_TCD18_DLASTSGA                      DMA_DLAST_SGA_REG(DMA1_BASE_PTR,18)
#define DMA1_TCD18_CSR                           DMA_CSR_REG(DMA1_BASE_PTR,18)
#define DMA1_TCD18_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,18)
#define DMA1_TCD18_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,18)
#define DMA1_TCD19_SADDR                         DMA_SADDR_REG(DMA1_BASE_PTR,19)
#define DMA1_TCD19_SOFF                          DMA_SOFF_REG(DMA1_BASE_PTR,19)
#define DMA1_TCD19_ATTR                          DMA_ATTR_REG(DMA1_BASE_PTR,19)
#define DMA1_TCD19_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,19)
#define DMA1_TCD19_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,19)
#define DMA1_TCD19_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,19)
#define DMA1_TCD19_SLAST                         DMA_SLAST_REG(DMA1_BASE_PTR,19)
#define DMA1_TCD19_DADDR                         DMA_DADDR_REG(DMA1_BASE_PTR,19)
#define DMA1_TCD19_DOFF                          DMA_DOFF_REG(DMA1_BASE_PTR,19)
#define DMA1_TCD19_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,19)
#define DMA1_TCD19_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,19)
#define DMA1_TCD19_DLASTSGA                      DMA_DLAST_SGA_REG(DMA1_BASE_PTR,19)
#define DMA1_TCD19_CSR                           DMA_CSR_REG(DMA1_BASE_PTR,19)
#define DMA1_TCD19_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,19)
#define DMA1_TCD19_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,19)
#define DMA1_TCD20_SADDR                         DMA_SADDR_REG(DMA1_BASE_PTR,20)
#define DMA1_TCD20_SOFF                          DMA_SOFF_REG(DMA1_BASE_PTR,20)
#define DMA1_TCD20_ATTR                          DMA_ATTR_REG(DMA1_BASE_PTR,20)
#define DMA1_TCD20_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,20)
#define DMA1_TCD20_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,20)
#define DMA1_TCD20_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,20)
#define DMA1_TCD20_SLAST                         DMA_SLAST_REG(DMA1_BASE_PTR,20)
#define DMA1_TCD20_DADDR                         DMA_DADDR_REG(DMA1_BASE_PTR,20)
#define DMA1_TCD20_DOFF                          DMA_DOFF_REG(DMA1_BASE_PTR,20)
#define DMA1_TCD20_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,20)
#define DMA1_TCD20_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,20)
#define DMA1_TCD20_DLASTSGA                      DMA_DLAST_SGA_REG(DMA1_BASE_PTR,20)
#define DMA1_TCD20_CSR                           DMA_CSR_REG(DMA1_BASE_PTR,20)
#define DMA1_TCD20_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,20)
#define DMA1_TCD20_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,20)
#define DMA1_TCD21_SADDR                         DMA_SADDR_REG(DMA1_BASE_PTR,21)
#define DMA1_TCD21_SOFF                          DMA_SOFF_REG(DMA1_BASE_PTR,21)
#define DMA1_TCD21_ATTR                          DMA_ATTR_REG(DMA1_BASE_PTR,21)
#define DMA1_TCD21_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,21)
#define DMA1_TCD21_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,21)
#define DMA1_TCD21_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,21)
#define DMA1_TCD21_SLAST                         DMA_SLAST_REG(DMA1_BASE_PTR,21)
#define DMA1_TCD21_DADDR                         DMA_DADDR_REG(DMA1_BASE_PTR,21)
#define DMA1_TCD21_DOFF                          DMA_DOFF_REG(DMA1_BASE_PTR,21)
#define DMA1_TCD21_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,21)
#define DMA1_TCD21_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,21)
#define DMA1_TCD21_DLASTSGA                      DMA_DLAST_SGA_REG(DMA1_BASE_PTR,21)
#define DMA1_TCD21_CSR                           DMA_CSR_REG(DMA1_BASE_PTR,21)
#define DMA1_TCD21_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,21)
#define DMA1_TCD21_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,21)
#define DMA1_TCD22_SADDR                         DMA_SADDR_REG(DMA1_BASE_PTR,22)
#define DMA1_TCD22_SOFF                          DMA_SOFF_REG(DMA1_BASE_PTR,22)
#define DMA1_TCD22_ATTR                          DMA_ATTR_REG(DMA1_BASE_PTR,22)
#define DMA1_TCD22_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,22)
#define DMA1_TCD22_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,22)
#define DMA1_TCD22_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,22)
#define DMA1_TCD22_SLAST                         DMA_SLAST_REG(DMA1_BASE_PTR,22)
#define DMA1_TCD22_DADDR                         DMA_DADDR_REG(DMA1_BASE_PTR,22)
#define DMA1_TCD22_DOFF                          DMA_DOFF_REG(DMA1_BASE_PTR,22)
#define DMA1_TCD22_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,22)
#define DMA1_TCD22_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,22)
#define DMA1_TCD22_DLASTSGA                      DMA_DLAST_SGA_REG(DMA1_BASE_PTR,22)
#define DMA1_TCD22_CSR                           DMA_CSR_REG(DMA1_BASE_PTR,22)
#define DMA1_TCD22_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,22)
#define DMA1_TCD22_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,22)
#define DMA1_TCD23_SADDR                         DMA_SADDR_REG(DMA1_BASE_PTR,23)
#define DMA1_TCD23_SOFF                          DMA_SOFF_REG(DMA1_BASE_PTR,23)
#define DMA1_TCD23_ATTR                          DMA_ATTR_REG(DMA1_BASE_PTR,23)
#define DMA1_TCD23_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,23)
#define DMA1_TCD23_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,23)
#define DMA1_TCD23_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,23)
#define DMA1_TCD23_SLAST                         DMA_SLAST_REG(DMA1_BASE_PTR,23)
#define DMA1_TCD23_DADDR                         DMA_DADDR_REG(DMA1_BASE_PTR,23)
#define DMA1_TCD23_DOFF                          DMA_DOFF_REG(DMA1_BASE_PTR,23)
#define DMA1_TCD23_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,23)
#define DMA1_TCD23_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,23)
#define DMA1_TCD23_DLASTSGA                      DMA_DLAST_SGA_REG(DMA1_BASE_PTR,23)
#define DMA1_TCD23_CSR                           DMA_CSR_REG(DMA1_BASE_PTR,23)
#define DMA1_TCD23_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,23)
#define DMA1_TCD23_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,23)
#define DMA1_TCD24_SADDR                         DMA_SADDR_REG(DMA1_BASE_PTR,24)
#define DMA1_TCD24_SOFF                          DMA_SOFF_REG(DMA1_BASE_PTR,24)
#define DMA1_TCD24_ATTR                          DMA_ATTR_REG(DMA1_BASE_PTR,24)
#define DMA1_TCD24_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,24)
#define DMA1_TCD24_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,24)
#define DMA1_TCD24_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,24)
#define DMA1_TCD24_SLAST                         DMA_SLAST_REG(DMA1_BASE_PTR,24)
#define DMA1_TCD24_DADDR                         DMA_DADDR_REG(DMA1_BASE_PTR,24)
#define DMA1_TCD24_DOFF                          DMA_DOFF_REG(DMA1_BASE_PTR,24)
#define DMA1_TCD24_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,24)
#define DMA1_TCD24_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,24)
#define DMA1_TCD24_DLASTSGA                      DMA_DLAST_SGA_REG(DMA1_BASE_PTR,24)
#define DMA1_TCD24_CSR                           DMA_CSR_REG(DMA1_BASE_PTR,24)
#define DMA1_TCD24_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,24)
#define DMA1_TCD24_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,24)
#define DMA1_TCD25_SADDR                         DMA_SADDR_REG(DMA1_BASE_PTR,25)
#define DMA1_TCD25_SOFF                          DMA_SOFF_REG(DMA1_BASE_PTR,25)
#define DMA1_TCD25_ATTR                          DMA_ATTR_REG(DMA1_BASE_PTR,25)
#define DMA1_TCD25_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,25)
#define DMA1_TCD25_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,25)
#define DMA1_TCD25_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,25)
#define DMA1_TCD25_SLAST                         DMA_SLAST_REG(DMA1_BASE_PTR,25)
#define DMA1_TCD25_DADDR                         DMA_DADDR_REG(DMA1_BASE_PTR,25)
#define DMA1_TCD25_DOFF                          DMA_DOFF_REG(DMA1_BASE_PTR,25)
#define DMA1_TCD25_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,25)
#define DMA1_TCD25_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,25)
#define DMA1_TCD25_DLASTSGA                      DMA_DLAST_SGA_REG(DMA1_BASE_PTR,25)
#define DMA1_TCD25_CSR                           DMA_CSR_REG(DMA1_BASE_PTR,25)
#define DMA1_TCD25_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,25)
#define DMA1_TCD25_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,25)
#define DMA1_TCD26_SADDR                         DMA_SADDR_REG(DMA1_BASE_PTR,26)
#define DMA1_TCD26_SOFF                          DMA_SOFF_REG(DMA1_BASE_PTR,26)
#define DMA1_TCD26_ATTR                          DMA_ATTR_REG(DMA1_BASE_PTR,26)
#define DMA1_TCD26_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,26)
#define DMA1_TCD26_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,26)
#define DMA1_TCD26_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,26)
#define DMA1_TCD26_SLAST                         DMA_SLAST_REG(DMA1_BASE_PTR,26)
#define DMA1_TCD26_DADDR                         DMA_DADDR_REG(DMA1_BASE_PTR,26)
#define DMA1_TCD26_DOFF                          DMA_DOFF_REG(DMA1_BASE_PTR,26)
#define DMA1_TCD26_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,26)
#define DMA1_TCD26_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,26)
#define DMA1_TCD26_DLASTSGA                      DMA_DLAST_SGA_REG(DMA1_BASE_PTR,26)
#define DMA1_TCD26_CSR                           DMA_CSR_REG(DMA1_BASE_PTR,26)
#define DMA1_TCD26_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,26)
#define DMA1_TCD26_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,26)
#define DMA1_TCD27_SADDR                         DMA_SADDR_REG(DMA1_BASE_PTR,27)
#define DMA1_TCD27_SOFF                          DMA_SOFF_REG(DMA1_BASE_PTR,27)
#define DMA1_TCD27_ATTR                          DMA_ATTR_REG(DMA1_BASE_PTR,27)
#define DMA1_TCD27_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,27)
#define DMA1_TCD27_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,27)
#define DMA1_TCD27_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,27)
#define DMA1_TCD27_SLAST                         DMA_SLAST_REG(DMA1_BASE_PTR,27)
#define DMA1_TCD27_DADDR                         DMA_DADDR_REG(DMA1_BASE_PTR,27)
#define DMA1_TCD27_DOFF                          DMA_DOFF_REG(DMA1_BASE_PTR,27)
#define DMA1_TCD27_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,27)
#define DMA1_TCD27_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,27)
#define DMA1_TCD27_DLASTSGA                      DMA_DLAST_SGA_REG(DMA1_BASE_PTR,27)
#define DMA1_TCD27_CSR                           DMA_CSR_REG(DMA1_BASE_PTR,27)
#define DMA1_TCD27_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,27)
#define DMA1_TCD27_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,27)
#define DMA1_TCD28_SADDR                         DMA_SADDR_REG(DMA1_BASE_PTR,28)
#define DMA1_TCD28_SOFF                          DMA_SOFF_REG(DMA1_BASE_PTR,28)
#define DMA1_TCD28_ATTR                          DMA_ATTR_REG(DMA1_BASE_PTR,28)
#define DMA1_TCD28_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,28)
#define DMA1_TCD28_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,28)
#define DMA1_TCD28_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,28)
#define DMA1_TCD28_SLAST                         DMA_SLAST_REG(DMA1_BASE_PTR,28)
#define DMA1_TCD28_DADDR                         DMA_DADDR_REG(DMA1_BASE_PTR,28)
#define DMA1_TCD28_DOFF                          DMA_DOFF_REG(DMA1_BASE_PTR,28)
#define DMA1_TCD28_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,28)
#define DMA1_TCD28_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,28)
#define DMA1_TCD28_DLASTSGA                      DMA_DLAST_SGA_REG(DMA1_BASE_PTR,28)
#define DMA1_TCD28_CSR                           DMA_CSR_REG(DMA1_BASE_PTR,28)
#define DMA1_TCD28_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,28)
#define DMA1_TCD28_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,28)
#define DMA1_TCD29_SADDR                         DMA_SADDR_REG(DMA1_BASE_PTR,29)
#define DMA1_TCD29_SOFF                          DMA_SOFF_REG(DMA1_BASE_PTR,29)
#define DMA1_TCD29_ATTR                          DMA_ATTR_REG(DMA1_BASE_PTR,29)
#define DMA1_TCD29_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,29)
#define DMA1_TCD29_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,29)
#define DMA1_TCD29_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,29)
#define DMA1_TCD29_SLAST                         DMA_SLAST_REG(DMA1_BASE_PTR,29)
#define DMA1_TCD29_DADDR                         DMA_DADDR_REG(DMA1_BASE_PTR,29)
#define DMA1_TCD29_DOFF                          DMA_DOFF_REG(DMA1_BASE_PTR,29)
#define DMA1_TCD29_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,29)
#define DMA1_TCD29_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,29)
#define DMA1_TCD29_DLASTSGA                      DMA_DLAST_SGA_REG(DMA1_BASE_PTR,29)
#define DMA1_TCD29_CSR                           DMA_CSR_REG(DMA1_BASE_PTR,29)
#define DMA1_TCD29_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,29)
#define DMA1_TCD29_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,29)
#define DMA1_TCD30_SADDR                         DMA_SADDR_REG(DMA1_BASE_PTR,30)
#define DMA1_TCD30_SOFF                          DMA_SOFF_REG(DMA1_BASE_PTR,30)
#define DMA1_TCD30_ATTR                          DMA_ATTR_REG(DMA1_BASE_PTR,30)
#define DMA1_TCD30_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,30)
#define DMA1_TCD30_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,30)
#define DMA1_TCD30_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,30)
#define DMA1_TCD30_SLAST                         DMA_SLAST_REG(DMA1_BASE_PTR,30)
#define DMA1_TCD30_DADDR                         DMA_DADDR_REG(DMA1_BASE_PTR,30)
#define DMA1_TCD30_DOFF                          DMA_DOFF_REG(DMA1_BASE_PTR,30)
#define DMA1_TCD30_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,30)
#define DMA1_TCD30_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,30)
#define DMA1_TCD30_DLASTSGA                      DMA_DLAST_SGA_REG(DMA1_BASE_PTR,30)
#define DMA1_TCD30_CSR                           DMA_CSR_REG(DMA1_BASE_PTR,30)
#define DMA1_TCD30_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,30)
#define DMA1_TCD30_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,30)
#define DMA1_TCD31_SADDR                         DMA_SADDR_REG(DMA1_BASE_PTR,31)
#define DMA1_TCD31_SOFF                          DMA_SOFF_REG(DMA1_BASE_PTR,31)
#define DMA1_TCD31_ATTR                          DMA_ATTR_REG(DMA1_BASE_PTR,31)
#define DMA1_TCD31_NBYTES_MLNO                   DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,31)
#define DMA1_TCD31_NBYTES_MLOFFNO                DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,31)
#define DMA1_TCD31_NBYTES_MLOFFYES               DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,31)
#define DMA1_TCD31_SLAST                         DMA_SLAST_REG(DMA1_BASE_PTR,31)
#define DMA1_TCD31_DADDR                         DMA_DADDR_REG(DMA1_BASE_PTR,31)
#define DMA1_TCD31_DOFF                          DMA_DOFF_REG(DMA1_BASE_PTR,31)
#define DMA1_TCD31_CITER_ELINKNO                 DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,31)
#define DMA1_TCD31_CITER_ELINKYES                DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,31)
#define DMA1_TCD31_DLASTSGA                      DMA_DLAST_SGA_REG(DMA1_BASE_PTR,31)
#define DMA1_TCD31_CSR                           DMA_CSR_REG(DMA1_BASE_PTR,31)
#define DMA1_TCD31_BITER_ELINKNO                 DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,31)
#define DMA1_TCD31_BITER_ELINKYES                DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,31)

/* DMA - Register array accessors */
#define DMA0_SADDR(index)                        DMA_SADDR_REG(DMA0_BASE_PTR,index)
#define DMA1_SADDR(index)                        DMA_SADDR_REG(DMA1_BASE_PTR,index)
#define DMA0_SOFF(index)                         DMA_SOFF_REG(DMA0_BASE_PTR,index)
#define DMA1_SOFF(index)                         DMA_SOFF_REG(DMA1_BASE_PTR,index)
#define DMA0_ATTR(index)                         DMA_ATTR_REG(DMA0_BASE_PTR,index)
#define DMA1_ATTR(index)                         DMA_ATTR_REG(DMA1_BASE_PTR,index)
#define DMA0_NBYTES_MLNO(index)                  DMA_NBYTES_MLNO_REG(DMA0_BASE_PTR,index)
#define DMA1_NBYTES_MLNO(index)                  DMA_NBYTES_MLNO_REG(DMA1_BASE_PTR,index)
#define DMA0_NBYTES_MLOFFNO(index)               DMA_NBYTES_MLOFFNO_REG(DMA0_BASE_PTR,index)
#define DMA1_NBYTES_MLOFFNO(index)               DMA_NBYTES_MLOFFNO_REG(DMA1_BASE_PTR,index)
#define DMA0_NBYTES_MLOFFYES(index)              DMA_NBYTES_MLOFFYES_REG(DMA0_BASE_PTR,index)
#define DMA1_NBYTES_MLOFFYES(index)              DMA_NBYTES_MLOFFYES_REG(DMA1_BASE_PTR,index)
#define DMA0_SLAST(index)                        DMA_SLAST_REG(DMA0_BASE_PTR,index)
#define DMA1_SLAST(index)                        DMA_SLAST_REG(DMA1_BASE_PTR,index)
#define DMA0_DADDR(index)                        DMA_DADDR_REG(DMA0_BASE_PTR,index)
#define DMA1_DADDR(index)                        DMA_DADDR_REG(DMA1_BASE_PTR,index)
#define DMA0_DOFF(index)                         DMA_DOFF_REG(DMA0_BASE_PTR,index)
#define DMA1_DOFF(index)                         DMA_DOFF_REG(DMA1_BASE_PTR,index)
#define DMA0_CITER_ELINKNO(index)                DMA_CITER_ELINKNO_REG(DMA0_BASE_PTR,index)
#define DMA1_CITER_ELINKNO(index)                DMA_CITER_ELINKNO_REG(DMA1_BASE_PTR,index)
#define DMA0_CITER_ELINKYES(index)               DMA_CITER_ELINKYES_REG(DMA0_BASE_PTR,index)
#define DMA1_CITER_ELINKYES(index)               DMA_CITER_ELINKYES_REG(DMA1_BASE_PTR,index)
#define DMA0_DLAST_SGA(index)                    DMA_DLAST_SGA_REG(DMA0_BASE_PTR,index)
#define DMA1_DLAST_SGA(index)                    DMA_DLAST_SGA_REG(DMA1_BASE_PTR,index)
#define DMA0_CSR(index)                          DMA_CSR_REG(DMA0_BASE_PTR,index)
#define DMA1_CSR(index)                          DMA_CSR_REG(DMA1_BASE_PTR,index)
#define DMA0_BITER_ELINKNO(index)                DMA_BITER_ELINKNO_REG(DMA0_BASE_PTR,index)
#define DMA1_BITER_ELINKNO(index)                DMA_BITER_ELINKNO_REG(DMA1_BASE_PTR,index)
#define DMA0_BITER_ELINKYES(index)               DMA_BITER_ELINKYES_REG(DMA0_BASE_PTR,index)
#define DMA1_BITER_ELINKYES(index)               DMA_BITER_ELINKYES_REG(DMA1_BASE_PTR,index)

/*!
 * @}
 */ /* end of group DMA_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group DMA_Peripheral */


/* ----------------------------------------------------------------------------
   -- DMAMUX
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMAMUX_Peripheral DMAMUX
 * @{
 */

/** DMAMUX - Peripheral register structure */
typedef struct DMAMUX_MemMap {
  uint8_t CHCFG[16];                               /**< Channel Configuration register, array offset: 0x0, array step: 0x1 */
} volatile *DMAMUX_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- DMAMUX - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMAMUX_Register_Accessor_Macros DMAMUX - Register accessor macros
 * @{
 */


/* DMAMUX - Register accessors */
#define DMAMUX_CHCFG_REG(base,index)             ((base)->CHCFG[index])

/*!
 * @}
 */ /* end of group DMAMUX_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- DMAMUX Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMAMUX_Register_Masks DMAMUX Register Masks
 * @{
 */

/* CHCFG Bit Fields */
#define DMAMUX_CHCFG_SOURCE_MASK                 0x3Fu
#define DMAMUX_CHCFG_SOURCE_SHIFT                0
#define DMAMUX_CHCFG_SOURCE(x)                   (((uint8_t)(((uint8_t)(x))<<DMAMUX_CHCFG_SOURCE_SHIFT))&DMAMUX_CHCFG_SOURCE_MASK)
#define DMAMUX_CHCFG_TRIG_MASK                   0x40u
#define DMAMUX_CHCFG_TRIG_SHIFT                  6
#define DMAMUX_CHCFG_ENBL_MASK                   0x80u
#define DMAMUX_CHCFG_ENBL_SHIFT                  7

/*!
 * @}
 */ /* end of group DMAMUX_Register_Masks */


/* DMAMUX - Peripheral instance base addresses */
/** Peripheral DMAMUX0 base pointer */
#define DMAMUX0_BASE_PTR                         ((DMAMUX_MemMapPtr)0x40024000u)
/** Peripheral DMAMUX1 base pointer */
#define DMAMUX1_BASE_PTR                         ((DMAMUX_MemMapPtr)0x40025000u)
/** Peripheral DMAMUX2 base pointer */
#define DMAMUX2_BASE_PTR                         ((DMAMUX_MemMapPtr)0x400A1000u)
/** Peripheral DMAMUX3 base pointer */
#define DMAMUX3_BASE_PTR                         ((DMAMUX_MemMapPtr)0x400A2000u)
/** Array initializer of DMAMUX peripheral base pointers */
#define DMAMUX_BASE_PTRS                         { DMAMUX0_BASE_PTR, DMAMUX1_BASE_PTR, DMAMUX2_BASE_PTR, DMAMUX3_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- DMAMUX - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMAMUX_Register_Accessor_Macros DMAMUX - Register accessor macros
 * @{
 */


/* DMAMUX - Register instance definitions */
/* DMAMUX0 */
#define DMAMUX0_CHCFG0                           DMAMUX_CHCFG_REG(DMAMUX0_BASE_PTR,0)
#define DMAMUX0_CHCFG1                           DMAMUX_CHCFG_REG(DMAMUX0_BASE_PTR,1)
#define DMAMUX0_CHCFG2                           DMAMUX_CHCFG_REG(DMAMUX0_BASE_PTR,2)
#define DMAMUX0_CHCFG3                           DMAMUX_CHCFG_REG(DMAMUX0_BASE_PTR,3)
#define DMAMUX0_CHCFG4                           DMAMUX_CHCFG_REG(DMAMUX0_BASE_PTR,4)
#define DMAMUX0_CHCFG5                           DMAMUX_CHCFG_REG(DMAMUX0_BASE_PTR,5)
#define DMAMUX0_CHCFG6                           DMAMUX_CHCFG_REG(DMAMUX0_BASE_PTR,6)
#define DMAMUX0_CHCFG7                           DMAMUX_CHCFG_REG(DMAMUX0_BASE_PTR,7)
#define DMAMUX0_CHCFG8                           DMAMUX_CHCFG_REG(DMAMUX0_BASE_PTR,8)
#define DMAMUX0_CHCFG9                           DMAMUX_CHCFG_REG(DMAMUX0_BASE_PTR,9)
#define DMAMUX0_CHCFG10                          DMAMUX_CHCFG_REG(DMAMUX0_BASE_PTR,10)
#define DMAMUX0_CHCFG11                          DMAMUX_CHCFG_REG(DMAMUX0_BASE_PTR,11)
#define DMAMUX0_CHCFG12                          DMAMUX_CHCFG_REG(DMAMUX0_BASE_PTR,12)
#define DMAMUX0_CHCFG13                          DMAMUX_CHCFG_REG(DMAMUX0_BASE_PTR,13)
#define DMAMUX0_CHCFG14                          DMAMUX_CHCFG_REG(DMAMUX0_BASE_PTR,14)
#define DMAMUX0_CHCFG15                          DMAMUX_CHCFG_REG(DMAMUX0_BASE_PTR,15)
/* DMAMUX1 */
#define DMAMUX1_CHCFG0                           DMAMUX_CHCFG_REG(DMAMUX1_BASE_PTR,0)
#define DMAMUX1_CHCFG1                           DMAMUX_CHCFG_REG(DMAMUX1_BASE_PTR,1)
#define DMAMUX1_CHCFG2                           DMAMUX_CHCFG_REG(DMAMUX1_BASE_PTR,2)
#define DMAMUX1_CHCFG3                           DMAMUX_CHCFG_REG(DMAMUX1_BASE_PTR,3)
#define DMAMUX1_CHCFG4                           DMAMUX_CHCFG_REG(DMAMUX1_BASE_PTR,4)
#define DMAMUX1_CHCFG5                           DMAMUX_CHCFG_REG(DMAMUX1_BASE_PTR,5)
#define DMAMUX1_CHCFG6                           DMAMUX_CHCFG_REG(DMAMUX1_BASE_PTR,6)
#define DMAMUX1_CHCFG7                           DMAMUX_CHCFG_REG(DMAMUX1_BASE_PTR,7)
#define DMAMUX1_CHCFG8                           DMAMUX_CHCFG_REG(DMAMUX1_BASE_PTR,8)
#define DMAMUX1_CHCFG9                           DMAMUX_CHCFG_REG(DMAMUX1_BASE_PTR,9)
#define DMAMUX1_CHCFG10                          DMAMUX_CHCFG_REG(DMAMUX1_BASE_PTR,10)
#define DMAMUX1_CHCFG11                          DMAMUX_CHCFG_REG(DMAMUX1_BASE_PTR,11)
#define DMAMUX1_CHCFG12                          DMAMUX_CHCFG_REG(DMAMUX1_BASE_PTR,12)
#define DMAMUX1_CHCFG13                          DMAMUX_CHCFG_REG(DMAMUX1_BASE_PTR,13)
#define DMAMUX1_CHCFG14                          DMAMUX_CHCFG_REG(DMAMUX1_BASE_PTR,14)
#define DMAMUX1_CHCFG15                          DMAMUX_CHCFG_REG(DMAMUX1_BASE_PTR,15)
/* DMAMUX2 */
#define DMAMUX2_CHCFG0                           DMAMUX_CHCFG_REG(DMAMUX2_BASE_PTR,0)
#define DMAMUX2_CHCFG1                           DMAMUX_CHCFG_REG(DMAMUX2_BASE_PTR,1)
#define DMAMUX2_CHCFG2                           DMAMUX_CHCFG_REG(DMAMUX2_BASE_PTR,2)
#define DMAMUX2_CHCFG3                           DMAMUX_CHCFG_REG(DMAMUX2_BASE_PTR,3)
#define DMAMUX2_CHCFG4                           DMAMUX_CHCFG_REG(DMAMUX2_BASE_PTR,4)
#define DMAMUX2_CHCFG5                           DMAMUX_CHCFG_REG(DMAMUX2_BASE_PTR,5)
#define DMAMUX2_CHCFG6                           DMAMUX_CHCFG_REG(DMAMUX2_BASE_PTR,6)
#define DMAMUX2_CHCFG7                           DMAMUX_CHCFG_REG(DMAMUX2_BASE_PTR,7)
#define DMAMUX2_CHCFG8                           DMAMUX_CHCFG_REG(DMAMUX2_BASE_PTR,8)
#define DMAMUX2_CHCFG9                           DMAMUX_CHCFG_REG(DMAMUX2_BASE_PTR,9)
#define DMAMUX2_CHCFG10                          DMAMUX_CHCFG_REG(DMAMUX2_BASE_PTR,10)
#define DMAMUX2_CHCFG11                          DMAMUX_CHCFG_REG(DMAMUX2_BASE_PTR,11)
#define DMAMUX2_CHCFG12                          DMAMUX_CHCFG_REG(DMAMUX2_BASE_PTR,12)
#define DMAMUX2_CHCFG13                          DMAMUX_CHCFG_REG(DMAMUX2_BASE_PTR,13)
#define DMAMUX2_CHCFG14                          DMAMUX_CHCFG_REG(DMAMUX2_BASE_PTR,14)
#define DMAMUX2_CHCFG15                          DMAMUX_CHCFG_REG(DMAMUX2_BASE_PTR,15)
/* DMAMUX3 */
#define DMAMUX3_CHCFG0                           DMAMUX_CHCFG_REG(DMAMUX3_BASE_PTR,0)
#define DMAMUX3_CHCFG1                           DMAMUX_CHCFG_REG(DMAMUX3_BASE_PTR,1)
#define DMAMUX3_CHCFG2                           DMAMUX_CHCFG_REG(DMAMUX3_BASE_PTR,2)
#define DMAMUX3_CHCFG3                           DMAMUX_CHCFG_REG(DMAMUX3_BASE_PTR,3)
#define DMAMUX3_CHCFG4                           DMAMUX_CHCFG_REG(DMAMUX3_BASE_PTR,4)
#define DMAMUX3_CHCFG5                           DMAMUX_CHCFG_REG(DMAMUX3_BASE_PTR,5)
#define DMAMUX3_CHCFG6                           DMAMUX_CHCFG_REG(DMAMUX3_BASE_PTR,6)
#define DMAMUX3_CHCFG7                           DMAMUX_CHCFG_REG(DMAMUX3_BASE_PTR,7)
#define DMAMUX3_CHCFG8                           DMAMUX_CHCFG_REG(DMAMUX3_BASE_PTR,8)
#define DMAMUX3_CHCFG9                           DMAMUX_CHCFG_REG(DMAMUX3_BASE_PTR,9)
#define DMAMUX3_CHCFG10                          DMAMUX_CHCFG_REG(DMAMUX3_BASE_PTR,10)
#define DMAMUX3_CHCFG11                          DMAMUX_CHCFG_REG(DMAMUX3_BASE_PTR,11)
#define DMAMUX3_CHCFG12                          DMAMUX_CHCFG_REG(DMAMUX3_BASE_PTR,12)
#define DMAMUX3_CHCFG13                          DMAMUX_CHCFG_REG(DMAMUX3_BASE_PTR,13)
#define DMAMUX3_CHCFG14                          DMAMUX_CHCFG_REG(DMAMUX3_BASE_PTR,14)
#define DMAMUX3_CHCFG15                          DMAMUX_CHCFG_REG(DMAMUX3_BASE_PTR,15)

/* DMAMUX - Register array accessors */
#define DMAMUX0_CHCFG(index)                     DMAMUX_CHCFG_REG(DMAMUX0_BASE_PTR,index)
#define DMAMUX1_CHCFG(index)                     DMAMUX_CHCFG_REG(DMAMUX1_BASE_PTR,index)
#define DMAMUX2_CHCFG(index)                     DMAMUX_CHCFG_REG(DMAMUX2_BASE_PTR,index)
#define DMAMUX3_CHCFG(index)                     DMAMUX_CHCFG_REG(DMAMUX3_BASE_PTR,index)

/*!
 * @}
 */ /* end of group DMAMUX_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group DMAMUX_Peripheral */

/* ----------------------------------------------------------------------------
   -- FTM
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTM_Peripheral FTM
 * @{
 */

/** FTM - Peripheral register structure */
typedef struct FTM_MemMap {
  uint32_t SC;                                     /**< Status And Control, offset: 0x0 */
  uint32_t CNT;                                    /**< Counter, offset: 0x4 */
  uint32_t MOD;                                    /**< Modulo, offset: 0x8 */
  struct {                                         /* offset: 0xC, array step: 0x8 */
    uint32_t CnSC;                                   /**< Channel (n) Status And Control, array offset: 0xC, array step: 0x8 */
    uint32_t CnV;                                    /**< Channel (n) Value, array offset: 0x10, array step: 0x8 */
  } CONTROLS[8];
  uint32_t CNTIN;                                  /**< Counter Initial Value, offset: 0x4C */
  uint32_t STATUS;                                 /**< Capture And Compare Status, offset: 0x50 */
  uint32_t MODE;                                   /**< Features Mode Selection, offset: 0x54 */
  uint32_t SYNC;                                   /**< Synchronization, offset: 0x58 */
  uint32_t OUTINIT;                                /**< Initial State For Channels Output, offset: 0x5C */
  uint32_t OUTMASK;                                /**< Output Mask, offset: 0x60 */
  uint32_t COMBINE;                                /**< Function For Linked Channels, offset: 0x64 */
  uint32_t DEADTIME;                               /**< Deadtime Insertion Control, offset: 0x68 */
  uint32_t EXTTRIG;                                /**< FTM External Trigger, offset: 0x6C */
  uint32_t POL;                                    /**< Channels Polarity, offset: 0x70 */
  uint32_t FMS;                                    /**< Fault Mode Status, offset: 0x74 */
  uint32_t FILTER;                                 /**< Input Capture Filter Control, offset: 0x78 */
  uint32_t FLTCTRL;                                /**< Fault Control, offset: 0x7C */
  uint32_t QDCTRL;                                 /**< Quadrature Decoder Control And Status, offset: 0x80 */
  uint32_t CONF;                                   /**< Configuration, offset: 0x84 */
  uint32_t FLTPOL;                                 /**< FTM Fault Input Polarity, offset: 0x88 */
  uint32_t SYNCONF;                                /**< Synchronization Configuration, offset: 0x8C */
  uint32_t INVCTRL;                                /**< FTM Inverting Control, offset: 0x90 */
  uint32_t SWOCTRL;                                /**< FTM Software Output Control, offset: 0x94 */
  uint32_t PWMLOAD;                                /**< FTM PWM Load, offset: 0x98 */
} volatile *FTM_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- FTM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTM_Register_Accessor_Macros FTM - Register accessor macros
 * @{
 */


/* FTM - Register accessors */
#define FTM_SC_REG(base)                         ((base)->SC)
#define FTM_CNT_REG(base)                        ((base)->CNT)
#define FTM_MOD_REG(base)                        ((base)->MOD)
#define FTM_CnSC_REG(base,index)                 ((base)->CONTROLS[index].CnSC)
#define FTM_CnV_REG(base,index)                  ((base)->CONTROLS[index].CnV)
#define FTM_CNTIN_REG(base)                      ((base)->CNTIN)
#define FTM_STATUS_REG(base)                     ((base)->STATUS)
#define FTM_MODE_REG(base)                       ((base)->MODE)
#define FTM_SYNC_REG(base)                       ((base)->SYNC)
#define FTM_OUTINIT_REG(base)                    ((base)->OUTINIT)
#define FTM_OUTMASK_REG(base)                    ((base)->OUTMASK)
#define FTM_COMBINE_REG(base)                    ((base)->COMBINE)
#define FTM_DEADTIME_REG(base)                   ((base)->DEADTIME)
#define FTM_EXTTRIG_REG(base)                    ((base)->EXTTRIG)
#define FTM_POL_REG(base)                        ((base)->POL)
#define FTM_FMS_REG(base)                        ((base)->FMS)
#define FTM_FILTER_REG(base)                     ((base)->FILTER)
#define FTM_FLTCTRL_REG(base)                    ((base)->FLTCTRL)
#define FTM_QDCTRL_REG(base)                     ((base)->QDCTRL)
#define FTM_CONF_REG(base)                       ((base)->CONF)
#define FTM_FLTPOL_REG(base)                     ((base)->FLTPOL)
#define FTM_SYNCONF_REG(base)                    ((base)->SYNCONF)
#define FTM_INVCTRL_REG(base)                    ((base)->INVCTRL)
#define FTM_SWOCTRL_REG(base)                    ((base)->SWOCTRL)
#define FTM_PWMLOAD_REG(base)                    ((base)->PWMLOAD)

/*!
 * @}
 */ /* end of group FTM_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- FTM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTM_Register_Masks FTM Register Masks
 * @{
 */

/* SC Bit Fields */
#define FTM_SC_PS_MASK                           0x7u
#define FTM_SC_PS_SHIFT                          0
#define FTM_SC_PS(x)                             (((uint32_t)(((uint32_t)(x))<<FTM_SC_PS_SHIFT))&FTM_SC_PS_MASK)
#define FTM_SC_CLKS_MASK                         0x18u
#define FTM_SC_CLKS_SHIFT                        3
#define FTM_SC_CLKS(x)                           (((uint32_t)(((uint32_t)(x))<<FTM_SC_CLKS_SHIFT))&FTM_SC_CLKS_MASK)
#define FTM_SC_CPWMS_MASK                        0x20u
#define FTM_SC_CPWMS_SHIFT                       5
#define FTM_SC_TOIE_MASK                         0x40u
#define FTM_SC_TOIE_SHIFT                        6
#define FTM_SC_TOF_MASK                          0x80u
#define FTM_SC_TOF_SHIFT                         7
/* CNT Bit Fields */
#define FTM_CNT_COUNT_MASK                       0xFFFFu
#define FTM_CNT_COUNT_SHIFT                      0
#define FTM_CNT_COUNT(x)                         (((uint32_t)(((uint32_t)(x))<<FTM_CNT_COUNT_SHIFT))&FTM_CNT_COUNT_MASK)
/* MOD Bit Fields */
#define FTM_MOD_MOD_MASK                         0xFFFFu
#define FTM_MOD_MOD_SHIFT                        0
#define FTM_MOD_MOD(x)                           (((uint32_t)(((uint32_t)(x))<<FTM_MOD_MOD_SHIFT))&FTM_MOD_MOD_MASK)
/* CnSC Bit Fields */
#define FTM_CnSC_DMA_MASK                        0x1u
#define FTM_CnSC_DMA_SHIFT                       0
#define FTM_CnSC_ELSA_MASK                       0x4u
#define FTM_CnSC_ELSA_SHIFT                      2
#define FTM_CnSC_ELSB_MASK                       0x8u
#define FTM_CnSC_ELSB_SHIFT                      3
#define FTM_CnSC_MSA_MASK                        0x10u
#define FTM_CnSC_MSA_SHIFT                       4
#define FTM_CnSC_MSB_MASK                        0x20u
#define FTM_CnSC_MSB_SHIFT                       5
#define FTM_CnSC_CHIE_MASK                       0x40u
#define FTM_CnSC_CHIE_SHIFT                      6
#define FTM_CnSC_CHF_MASK                        0x80u
#define FTM_CnSC_CHF_SHIFT                       7
/* CnV Bit Fields */
#define FTM_CnV_VAL_MASK                         0xFFFFu
#define FTM_CnV_VAL_SHIFT                        0
#define FTM_CnV_VAL(x)                           (((uint32_t)(((uint32_t)(x))<<FTM_CnV_VAL_SHIFT))&FTM_CnV_VAL_MASK)
/* CNTIN Bit Fields */
#define FTM_CNTIN_INIT_MASK                      0xFFFFu
#define FTM_CNTIN_INIT_SHIFT                     0
#define FTM_CNTIN_INIT(x)                        (((uint32_t)(((uint32_t)(x))<<FTM_CNTIN_INIT_SHIFT))&FTM_CNTIN_INIT_MASK)
/* STATUS Bit Fields */
#define FTM_STATUS_CH0F_MASK                     0x1u
#define FTM_STATUS_CH0F_SHIFT                    0
#define FTM_STATUS_CH1F_MASK                     0x2u
#define FTM_STATUS_CH1F_SHIFT                    1
#define FTM_STATUS_CH2F_MASK                     0x4u
#define FTM_STATUS_CH2F_SHIFT                    2
#define FTM_STATUS_CH3F_MASK                     0x8u
#define FTM_STATUS_CH3F_SHIFT                    3
#define FTM_STATUS_CH4F_MASK                     0x10u
#define FTM_STATUS_CH4F_SHIFT                    4
#define FTM_STATUS_CH5F_MASK                     0x20u
#define FTM_STATUS_CH5F_SHIFT                    5
#define FTM_STATUS_CH6F_MASK                     0x40u
#define FTM_STATUS_CH6F_SHIFT                    6
#define FTM_STATUS_CH7F_MASK                     0x80u
#define FTM_STATUS_CH7F_SHIFT                    7
/* MODE Bit Fields */
#define FTM_MODE_FTMEN_MASK                      0x1u
#define FTM_MODE_FTMEN_SHIFT                     0
#define FTM_MODE_INIT_MASK                       0x2u
#define FTM_MODE_INIT_SHIFT                      1
#define FTM_MODE_WPDIS_MASK                      0x4u
#define FTM_MODE_WPDIS_SHIFT                     2
#define FTM_MODE_PWMSYNC_MASK                    0x8u
#define FTM_MODE_PWMSYNC_SHIFT                   3
#define FTM_MODE_CAPTEST_MASK                    0x10u
#define FTM_MODE_CAPTEST_SHIFT                   4
#define FTM_MODE_FAULTM_MASK                     0x60u
#define FTM_MODE_FAULTM_SHIFT                    5
#define FTM_MODE_FAULTM(x)                       (((uint32_t)(((uint32_t)(x))<<FTM_MODE_FAULTM_SHIFT))&FTM_MODE_FAULTM_MASK)
#define FTM_MODE_FAULTIE_MASK                    0x80u
#define FTM_MODE_FAULTIE_SHIFT                   7
/* SYNC Bit Fields */
#define FTM_SYNC_CNTMIN_MASK                     0x1u
#define FTM_SYNC_CNTMIN_SHIFT                    0
#define FTM_SYNC_CNTMAX_MASK                     0x2u
#define FTM_SYNC_CNTMAX_SHIFT                    1
#define FTM_SYNC_REINIT_MASK                     0x4u
#define FTM_SYNC_REINIT_SHIFT                    2
#define FTM_SYNC_SYNCHOM_MASK                    0x8u
#define FTM_SYNC_SYNCHOM_SHIFT                   3
#define FTM_SYNC_TRIG0_MASK                      0x10u
#define FTM_SYNC_TRIG0_SHIFT                     4
#define FTM_SYNC_TRIG1_MASK                      0x20u
#define FTM_SYNC_TRIG1_SHIFT                     5
#define FTM_SYNC_TRIG2_MASK                      0x40u
#define FTM_SYNC_TRIG2_SHIFT                     6
#define FTM_SYNC_SWSYNC_MASK                     0x80u
#define FTM_SYNC_SWSYNC_SHIFT                    7
/* OUTINIT Bit Fields */
#define FTM_OUTINIT_CH0OI_MASK                   0x1u
#define FTM_OUTINIT_CH0OI_SHIFT                  0
#define FTM_OUTINIT_CH1OI_MASK                   0x2u
#define FTM_OUTINIT_CH1OI_SHIFT                  1
#define FTM_OUTINIT_CH2OI_MASK                   0x4u
#define FTM_OUTINIT_CH2OI_SHIFT                  2
#define FTM_OUTINIT_CH3OI_MASK                   0x8u
#define FTM_OUTINIT_CH3OI_SHIFT                  3
#define FTM_OUTINIT_CH4OI_MASK                   0x10u
#define FTM_OUTINIT_CH4OI_SHIFT                  4
#define FTM_OUTINIT_CH5OI_MASK                   0x20u
#define FTM_OUTINIT_CH5OI_SHIFT                  5
#define FTM_OUTINIT_CH6OI_MASK                   0x40u
#define FTM_OUTINIT_CH6OI_SHIFT                  6
#define FTM_OUTINIT_CH7OI_MASK                   0x80u
#define FTM_OUTINIT_CH7OI_SHIFT                  7
/* OUTMASK Bit Fields */
#define FTM_OUTMASK_CH0OM_MASK                   0x1u
#define FTM_OUTMASK_CH0OM_SHIFT                  0
#define FTM_OUTMASK_CH1OM_MASK                   0x2u
#define FTM_OUTMASK_CH1OM_SHIFT                  1
#define FTM_OUTMASK_CH2OM_MASK                   0x4u
#define FTM_OUTMASK_CH2OM_SHIFT                  2
#define FTM_OUTMASK_CH3OM_MASK                   0x8u
#define FTM_OUTMASK_CH3OM_SHIFT                  3
#define FTM_OUTMASK_CH4OM_MASK                   0x10u
#define FTM_OUTMASK_CH4OM_SHIFT                  4
#define FTM_OUTMASK_CH5OM_MASK                   0x20u
#define FTM_OUTMASK_CH5OM_SHIFT                  5
#define FTM_OUTMASK_CH6OM_MASK                   0x40u
#define FTM_OUTMASK_CH6OM_SHIFT                  6
#define FTM_OUTMASK_CH7OM_MASK                   0x80u
#define FTM_OUTMASK_CH7OM_SHIFT                  7
/* COMBINE Bit Fields */
#define FTM_COMBINE_COMBINE0_MASK                0x1u
#define FTM_COMBINE_COMBINE0_SHIFT               0
#define FTM_COMBINE_COMP0_MASK                   0x2u
#define FTM_COMBINE_COMP0_SHIFT                  1
#define FTM_COMBINE_DECAPEN0_MASK                0x4u
#define FTM_COMBINE_DECAPEN0_SHIFT               2
#define FTM_COMBINE_DECAP0_MASK                  0x8u
#define FTM_COMBINE_DECAP0_SHIFT                 3
#define FTM_COMBINE_DTEN0_MASK                   0x10u
#define FTM_COMBINE_DTEN0_SHIFT                  4
#define FTM_COMBINE_SYNCEN0_MASK                 0x20u
#define FTM_COMBINE_SYNCEN0_SHIFT                5
#define FTM_COMBINE_FAULTEN0_MASK                0x40u
#define FTM_COMBINE_FAULTEN0_SHIFT               6
#define FTM_COMBINE_COMBINE1_MASK                0x100u
#define FTM_COMBINE_COMBINE1_SHIFT               8
#define FTM_COMBINE_COMP1_MASK                   0x200u
#define FTM_COMBINE_COMP1_SHIFT                  9
#define FTM_COMBINE_DECAPEN1_MASK                0x400u
#define FTM_COMBINE_DECAPEN1_SHIFT               10
#define FTM_COMBINE_DECAP1_MASK                  0x800u
#define FTM_COMBINE_DECAP1_SHIFT                 11
#define FTM_COMBINE_DTEN1_MASK                   0x1000u
#define FTM_COMBINE_DTEN1_SHIFT                  12
#define FTM_COMBINE_SYNCEN1_MASK                 0x2000u
#define FTM_COMBINE_SYNCEN1_SHIFT                13
#define FTM_COMBINE_FAULTEN1_MASK                0x4000u
#define FTM_COMBINE_FAULTEN1_SHIFT               14
#define FTM_COMBINE_COMBINE2_MASK                0x10000u
#define FTM_COMBINE_COMBINE2_SHIFT               16
#define FTM_COMBINE_COMP2_MASK                   0x20000u
#define FTM_COMBINE_COMP2_SHIFT                  17
#define FTM_COMBINE_DECAPEN2_MASK                0x40000u
#define FTM_COMBINE_DECAPEN2_SHIFT               18
#define FTM_COMBINE_DECAP2_MASK                  0x80000u
#define FTM_COMBINE_DECAP2_SHIFT                 19
#define FTM_COMBINE_DTEN2_MASK                   0x100000u
#define FTM_COMBINE_DTEN2_SHIFT                  20
#define FTM_COMBINE_SYNCEN2_MASK                 0x200000u
#define FTM_COMBINE_SYNCEN2_SHIFT                21
#define FTM_COMBINE_FAULTEN2_MASK                0x400000u
#define FTM_COMBINE_FAULTEN2_SHIFT               22
#define FTM_COMBINE_COMBINE3_MASK                0x1000000u
#define FTM_COMBINE_COMBINE3_SHIFT               24
#define FTM_COMBINE_COMP3_MASK                   0x2000000u
#define FTM_COMBINE_COMP3_SHIFT                  25
#define FTM_COMBINE_DECAPEN3_MASK                0x4000000u
#define FTM_COMBINE_DECAPEN3_SHIFT               26
#define FTM_COMBINE_DECAP3_MASK                  0x8000000u
#define FTM_COMBINE_DECAP3_SHIFT                 27
#define FTM_COMBINE_DTEN3_MASK                   0x10000000u
#define FTM_COMBINE_DTEN3_SHIFT                  28
#define FTM_COMBINE_SYNCEN3_MASK                 0x20000000u
#define FTM_COMBINE_SYNCEN3_SHIFT                29
#define FTM_COMBINE_FAULTEN3_MASK                0x40000000u
#define FTM_COMBINE_FAULTEN3_SHIFT               30
/* DEADTIME Bit Fields */
#define FTM_DEADTIME_DTVAL_MASK                  0x3Fu
#define FTM_DEADTIME_DTVAL_SHIFT                 0
#define FTM_DEADTIME_DTVAL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_DEADTIME_DTVAL_SHIFT))&FTM_DEADTIME_DTVAL_MASK)
#define FTM_DEADTIME_DTPS_MASK                   0xC0u
#define FTM_DEADTIME_DTPS_SHIFT                  6
#define FTM_DEADTIME_DTPS(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_DEADTIME_DTPS_SHIFT))&FTM_DEADTIME_DTPS_MASK)
/* EXTTRIG Bit Fields */
#define FTM_EXTTRIG_CH2TRIG_MASK                 0x1u
#define FTM_EXTTRIG_CH2TRIG_SHIFT                0
#define FTM_EXTTRIG_CH3TRIG_MASK                 0x2u
#define FTM_EXTTRIG_CH3TRIG_SHIFT                1
#define FTM_EXTTRIG_CH4TRIG_MASK                 0x4u
#define FTM_EXTTRIG_CH4TRIG_SHIFT                2
#define FTM_EXTTRIG_CH5TRIG_MASK                 0x8u
#define FTM_EXTTRIG_CH5TRIG_SHIFT                3
#define FTM_EXTTRIG_CH0TRIG_MASK                 0x10u
#define FTM_EXTTRIG_CH0TRIG_SHIFT                4
#define FTM_EXTTRIG_CH1TRIG_MASK                 0x20u
#define FTM_EXTTRIG_CH1TRIG_SHIFT                5
#define FTM_EXTTRIG_INITTRIGEN_MASK              0x40u
#define FTM_EXTTRIG_INITTRIGEN_SHIFT             6
#define FTM_EXTTRIG_TRIGF_MASK                   0x80u
#define FTM_EXTTRIG_TRIGF_SHIFT                  7
/* POL Bit Fields */
#define FTM_POL_POL0_MASK                        0x1u
#define FTM_POL_POL0_SHIFT                       0
#define FTM_POL_POL1_MASK                        0x2u
#define FTM_POL_POL1_SHIFT                       1
#define FTM_POL_POL2_MASK                        0x4u
#define FTM_POL_POL2_SHIFT                       2
#define FTM_POL_POL3_MASK                        0x8u
#define FTM_POL_POL3_SHIFT                       3
#define FTM_POL_POL4_MASK                        0x10u
#define FTM_POL_POL4_SHIFT                       4
#define FTM_POL_POL5_MASK                        0x20u
#define FTM_POL_POL5_SHIFT                       5
#define FTM_POL_POL6_MASK                        0x40u
#define FTM_POL_POL6_SHIFT                       6
#define FTM_POL_POL7_MASK                        0x80u
#define FTM_POL_POL7_SHIFT                       7
/* FMS Bit Fields */
#define FTM_FMS_FAULTF0_MASK                     0x1u
#define FTM_FMS_FAULTF0_SHIFT                    0
#define FTM_FMS_FAULTF1_MASK                     0x2u
#define FTM_FMS_FAULTF1_SHIFT                    1
#define FTM_FMS_FAULTF2_MASK                     0x4u
#define FTM_FMS_FAULTF2_SHIFT                    2
#define FTM_FMS_FAULTF3_MASK                     0x8u
#define FTM_FMS_FAULTF3_SHIFT                    3
#define FTM_FMS_FAULTIN_MASK                     0x20u
#define FTM_FMS_FAULTIN_SHIFT                    5
#define FTM_FMS_WPEN_MASK                        0x40u
#define FTM_FMS_WPEN_SHIFT                       6
#define FTM_FMS_FAULTF_MASK                      0x80u
#define FTM_FMS_FAULTF_SHIFT                     7
/* FILTER Bit Fields */
#define FTM_FILTER_CH0FVAL_MASK                  0xFu
#define FTM_FILTER_CH0FVAL_SHIFT                 0
#define FTM_FILTER_CH0FVAL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_FILTER_CH0FVAL_SHIFT))&FTM_FILTER_CH0FVAL_MASK)
#define FTM_FILTER_CH1FVAL_MASK                  0xF0u
#define FTM_FILTER_CH1FVAL_SHIFT                 4
#define FTM_FILTER_CH1FVAL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_FILTER_CH1FVAL_SHIFT))&FTM_FILTER_CH1FVAL_MASK)
#define FTM_FILTER_CH2FVAL_MASK                  0xF00u
#define FTM_FILTER_CH2FVAL_SHIFT                 8
#define FTM_FILTER_CH2FVAL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_FILTER_CH2FVAL_SHIFT))&FTM_FILTER_CH2FVAL_MASK)
#define FTM_FILTER_CH3FVAL_MASK                  0xF000u
#define FTM_FILTER_CH3FVAL_SHIFT                 12
#define FTM_FILTER_CH3FVAL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_FILTER_CH3FVAL_SHIFT))&FTM_FILTER_CH3FVAL_MASK)
/* FLTCTRL Bit Fields */
#define FTM_FLTCTRL_FAULT0EN_MASK                0x1u
#define FTM_FLTCTRL_FAULT0EN_SHIFT               0
#define FTM_FLTCTRL_FAULT1EN_MASK                0x2u
#define FTM_FLTCTRL_FAULT1EN_SHIFT               1
#define FTM_FLTCTRL_FAULT2EN_MASK                0x4u
#define FTM_FLTCTRL_FAULT2EN_SHIFT               2
#define FTM_FLTCTRL_FAULT3EN_MASK                0x8u
#define FTM_FLTCTRL_FAULT3EN_SHIFT               3
#define FTM_FLTCTRL_FFLTR0EN_MASK                0x10u
#define FTM_FLTCTRL_FFLTR0EN_SHIFT               4
#define FTM_FLTCTRL_FFLTR1EN_MASK                0x20u
#define FTM_FLTCTRL_FFLTR1EN_SHIFT               5
#define FTM_FLTCTRL_FFLTR2EN_MASK                0x40u
#define FTM_FLTCTRL_FFLTR2EN_SHIFT               6
#define FTM_FLTCTRL_FFLTR3EN_MASK                0x80u
#define FTM_FLTCTRL_FFLTR3EN_SHIFT               7
#define FTM_FLTCTRL_FFVAL_MASK                   0xF00u
#define FTM_FLTCTRL_FFVAL_SHIFT                  8
#define FTM_FLTCTRL_FFVAL(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_FLTCTRL_FFVAL_SHIFT))&FTM_FLTCTRL_FFVAL_MASK)
/* QDCTRL Bit Fields */
#define FTM_QDCTRL_QUADEN_MASK                   0x1u
#define FTM_QDCTRL_QUADEN_SHIFT                  0
#define FTM_QDCTRL_TOFDIR_MASK                   0x2u
#define FTM_QDCTRL_TOFDIR_SHIFT                  1
#define FTM_QDCTRL_QUADIR_MASK                   0x4u
#define FTM_QDCTRL_QUADIR_SHIFT                  2
#define FTM_QDCTRL_QUADMODE_MASK                 0x8u
#define FTM_QDCTRL_QUADMODE_SHIFT                3
#define FTM_QDCTRL_PHBPOL_MASK                   0x10u
#define FTM_QDCTRL_PHBPOL_SHIFT                  4
#define FTM_QDCTRL_PHAPOL_MASK                   0x20u
#define FTM_QDCTRL_PHAPOL_SHIFT                  5
#define FTM_QDCTRL_PHBFLTREN_MASK                0x40u
#define FTM_QDCTRL_PHBFLTREN_SHIFT               6
#define FTM_QDCTRL_PHAFLTREN_MASK                0x80u
#define FTM_QDCTRL_PHAFLTREN_SHIFT               7
/* CONF Bit Fields */
#define FTM_CONF_NUMTOF_MASK                     0x1Fu
#define FTM_CONF_NUMTOF_SHIFT                    0
#define FTM_CONF_NUMTOF(x)                       (((uint32_t)(((uint32_t)(x))<<FTM_CONF_NUMTOF_SHIFT))&FTM_CONF_NUMTOF_MASK)
#define FTM_CONF_BDMMODE_MASK                    0xC0u
#define FTM_CONF_BDMMODE_SHIFT                   6
#define FTM_CONF_BDMMODE(x)                      (((uint32_t)(((uint32_t)(x))<<FTM_CONF_BDMMODE_SHIFT))&FTM_CONF_BDMMODE_MASK)
#define FTM_CONF_GTBEEN_MASK                     0x200u
#define FTM_CONF_GTBEEN_SHIFT                    9
#define FTM_CONF_GTBEOUT_MASK                    0x400u
#define FTM_CONF_GTBEOUT_SHIFT                   10
/* FLTPOL Bit Fields */
#define FTM_FLTPOL_FLT0POL_MASK                  0x1u
#define FTM_FLTPOL_FLT0POL_SHIFT                 0
#define FTM_FLTPOL_FLT1POL_MASK                  0x2u
#define FTM_FLTPOL_FLT1POL_SHIFT                 1
#define FTM_FLTPOL_FLT2POL_MASK                  0x4u
#define FTM_FLTPOL_FLT2POL_SHIFT                 2
#define FTM_FLTPOL_FLT3POL_MASK                  0x8u
#define FTM_FLTPOL_FLT3POL_SHIFT                 3
/* SYNCONF Bit Fields */
#define FTM_SYNCONF_HWTRIGMODE_MASK              0x1u
#define FTM_SYNCONF_HWTRIGMODE_SHIFT             0
#define FTM_SYNCONF_CNTINC_MASK                  0x4u
#define FTM_SYNCONF_CNTINC_SHIFT                 2
#define FTM_SYNCONF_INVC_MASK                    0x10u
#define FTM_SYNCONF_INVC_SHIFT                   4
#define FTM_SYNCONF_SWOC_MASK                    0x20u
#define FTM_SYNCONF_SWOC_SHIFT                   5
#define FTM_SYNCONF_SYNCMODE_MASK                0x80u
#define FTM_SYNCONF_SYNCMODE_SHIFT               7
#define FTM_SYNCONF_SWRSTCNT_MASK                0x100u
#define FTM_SYNCONF_SWRSTCNT_SHIFT               8
#define FTM_SYNCONF_SWWRBUF_MASK                 0x200u
#define FTM_SYNCONF_SWWRBUF_SHIFT                9
#define FTM_SYNCONF_SWOM_MASK                    0x400u
#define FTM_SYNCONF_SWOM_SHIFT                   10
#define FTM_SYNCONF_SWINVC_MASK                  0x800u
#define FTM_SYNCONF_SWINVC_SHIFT                 11
#define FTM_SYNCONF_SWSOC_MASK                   0x1000u
#define FTM_SYNCONF_SWSOC_SHIFT                  12
#define FTM_SYNCONF_HWRSTCNT_MASK                0x10000u
#define FTM_SYNCONF_HWRSTCNT_SHIFT               16
#define FTM_SYNCONF_HWWRBUF_MASK                 0x20000u
#define FTM_SYNCONF_HWWRBUF_SHIFT                17
#define FTM_SYNCONF_HWOM_MASK                    0x40000u
#define FTM_SYNCONF_HWOM_SHIFT                   18
#define FTM_SYNCONF_HWINVC_MASK                  0x80000u
#define FTM_SYNCONF_HWINVC_SHIFT                 19
#define FTM_SYNCONF_HWSOC_MASK                   0x100000u
#define FTM_SYNCONF_HWSOC_SHIFT                  20
/* INVCTRL Bit Fields */
#define FTM_INVCTRL_INV0EN_MASK                  0x1u
#define FTM_INVCTRL_INV0EN_SHIFT                 0
#define FTM_INVCTRL_INV1EN_MASK                  0x2u
#define FTM_INVCTRL_INV1EN_SHIFT                 1
#define FTM_INVCTRL_INV2EN_MASK                  0x4u
#define FTM_INVCTRL_INV2EN_SHIFT                 2
#define FTM_INVCTRL_INV3EN_MASK                  0x8u
#define FTM_INVCTRL_INV3EN_SHIFT                 3
/* SWOCTRL Bit Fields */
#define FTM_SWOCTRL_CH0OC_MASK                   0x1u
#define FTM_SWOCTRL_CH0OC_SHIFT                  0
#define FTM_SWOCTRL_CH1OC_MASK                   0x2u
#define FTM_SWOCTRL_CH1OC_SHIFT                  1
#define FTM_SWOCTRL_CH2OC_MASK                   0x4u
#define FTM_SWOCTRL_CH2OC_SHIFT                  2
#define FTM_SWOCTRL_CH3OC_MASK                   0x8u
#define FTM_SWOCTRL_CH3OC_SHIFT                  3
#define FTM_SWOCTRL_CH4OC_MASK                   0x10u
#define FTM_SWOCTRL_CH4OC_SHIFT                  4
#define FTM_SWOCTRL_CH5OC_MASK                   0x20u
#define FTM_SWOCTRL_CH5OC_SHIFT                  5
#define FTM_SWOCTRL_CH6OC_MASK                   0x40u
#define FTM_SWOCTRL_CH6OC_SHIFT                  6
#define FTM_SWOCTRL_CH7OC_MASK                   0x80u
#define FTM_SWOCTRL_CH7OC_SHIFT                  7
#define FTM_SWOCTRL_CH0OCV_MASK                  0x100u
#define FTM_SWOCTRL_CH0OCV_SHIFT                 8
#define FTM_SWOCTRL_CH1OCV_MASK                  0x200u
#define FTM_SWOCTRL_CH1OCV_SHIFT                 9
#define FTM_SWOCTRL_CH2OCV_MASK                  0x400u
#define FTM_SWOCTRL_CH2OCV_SHIFT                 10
#define FTM_SWOCTRL_CH3OCV_MASK                  0x800u
#define FTM_SWOCTRL_CH3OCV_SHIFT                 11
#define FTM_SWOCTRL_CH4OCV_MASK                  0x1000u
#define FTM_SWOCTRL_CH4OCV_SHIFT                 12
#define FTM_SWOCTRL_CH5OCV_MASK                  0x2000u
#define FTM_SWOCTRL_CH5OCV_SHIFT                 13
#define FTM_SWOCTRL_CH6OCV_MASK                  0x4000u
#define FTM_SWOCTRL_CH6OCV_SHIFT                 14
#define FTM_SWOCTRL_CH7OCV_MASK                  0x8000u
#define FTM_SWOCTRL_CH7OCV_SHIFT                 15
/* PWMLOAD Bit Fields */
#define FTM_PWMLOAD_CH0SEL_MASK                  0x1u
#define FTM_PWMLOAD_CH0SEL_SHIFT                 0
#define FTM_PWMLOAD_CH1SEL_MASK                  0x2u
#define FTM_PWMLOAD_CH1SEL_SHIFT                 1
#define FTM_PWMLOAD_CH2SEL_MASK                  0x4u
#define FTM_PWMLOAD_CH2SEL_SHIFT                 2
#define FTM_PWMLOAD_CH3SEL_MASK                  0x8u
#define FTM_PWMLOAD_CH3SEL_SHIFT                 3
#define FTM_PWMLOAD_CH4SEL_MASK                  0x10u
#define FTM_PWMLOAD_CH4SEL_SHIFT                 4
#define FTM_PWMLOAD_CH5SEL_MASK                  0x20u
#define FTM_PWMLOAD_CH5SEL_SHIFT                 5
#define FTM_PWMLOAD_CH6SEL_MASK                  0x40u
#define FTM_PWMLOAD_CH6SEL_SHIFT                 6
#define FTM_PWMLOAD_CH7SEL_MASK                  0x80u
#define FTM_PWMLOAD_CH7SEL_SHIFT                 7
#define FTM_PWMLOAD_LDOK_MASK                    0x200u
#define FTM_PWMLOAD_LDOK_SHIFT                   9

/*!
 * @}
 */ /* end of group FTM_Register_Masks */


/* FTM - Peripheral instance base addresses */
/** Peripheral FTM0 base pointer */
#define FTM0_BASE_PTR                            ((FTM_MemMapPtr)0x40038000u)
/** Peripheral FTM1 base pointer */
#define FTM1_BASE_PTR                            ((FTM_MemMapPtr)0x40039000u)
/** Peripheral FTM2 base pointer */
#define FTM2_BASE_PTR                            ((FTM_MemMapPtr)0x400B8000u)
/** Peripheral FTM3 base pointer */
#define FTM3_BASE_PTR                            ((FTM_MemMapPtr)0x400B9000u)
/** Array initializer of FTM peripheral base pointers */
#define FTM_BASE_PTRS                            { FTM0_BASE_PTR, FTM1_BASE_PTR, FTM2_BASE_PTR, FTM3_BASE_PTR }

/*!
 * @}
 */ /* end of group FTM_Peripheral */

/* ----------------------------------------------------------------------------
   -- I2C
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2C_Peripheral I2C
 * @{
 */

/** I2C - Peripheral register structure */
typedef struct I2C_MemMap {
  uint8_t IBAD;                                    /**< I2C Bus Address Register, offset: 0x0 */
  uint8_t IBFD;                                    /**< I2C Bus Frequency Divider Register, offset: 0x1 */
  uint8_t IBCR;                                    /**< I2C Bus Control Register, offset: 0x2 */
  uint8_t IBSR;                                    /**< I2C Bus Status Register, offset: 0x3 */
  uint8_t IBDR;                                    /**< I2C Bus Data I/O Register, offset: 0x4 */
  uint8_t IBIC;                                    /**< I2C Bus Interrupt Config Register, offset: 0x5 */
  uint8_t IBDBG;                                   /**< I2C Bus Debug Register, offset: 0x6 */
} volatile *I2C_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- I2C - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2C_Register_Accessor_Macros I2C - Register accessor macros
 * @{
 */


/* I2C - Register accessors */
#define I2C_IBAD_REG(base)                       ((base)->IBAD)
#define I2C_IBFD_REG(base)                       ((base)->IBFD)
#define I2C_IBCR_REG(base)                       ((base)->IBCR)
#define I2C_IBSR_REG(base)                       ((base)->IBSR)
#define I2C_IBDR_REG(base)                       ((base)->IBDR)
#define I2C_IBIC_REG(base)                       ((base)->IBIC)
#define I2C_IBDBG_REG(base)                      ((base)->IBDBG)

/*!
 * @}
 */ /* end of group I2C_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- I2C Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2C_Register_Masks I2C Register Masks
 * @{
 */

/* IBAD Bit Fields */
#define I2C_IBAD_ADR_MASK                        0xFEu
#define I2C_IBAD_ADR_SHIFT                       1
#define I2C_IBAD_ADR(x)                          (((uint8_t)(((uint8_t)(x))<<I2C_IBAD_ADR_SHIFT))&I2C_IBAD_ADR_MASK)
/* IBFD Bit Fields */
#define I2C_IBFD_IBC_MASK                        0xFFu
#define I2C_IBFD_IBC_SHIFT                       0
#define I2C_IBFD_IBC(x)                          (((uint8_t)(((uint8_t)(x))<<I2C_IBFD_IBC_SHIFT))&I2C_IBFD_IBC_MASK)
/* IBCR Bit Fields */
#define I2C_IBCR_DMAEN_MASK                      0x2u
#define I2C_IBCR_DMAEN_SHIFT                     1
#define I2C_IBCR_RSTA_MASK                       0x4u
#define I2C_IBCR_RSTA_SHIFT                      2
#define I2C_IBCR_NOACK_MASK                      0x8u
#define I2C_IBCR_NOACK_SHIFT                     3
#define I2C_IBCR_TXRX_MASK                       0x10u
#define I2C_IBCR_TXRX_SHIFT                      4
#define I2C_IBCR_MSSL_MASK                       0x20u
#define I2C_IBCR_MSSL_SHIFT                      5
#define I2C_IBCR_IBIE_MASK                       0x40u
#define I2C_IBCR_IBIE_SHIFT                      6
#define I2C_IBCR_MDIS_MASK                       0x80u
#define I2C_IBCR_MDIS_SHIFT                      7
/* IBSR Bit Fields */
#define I2C_IBSR_RXAK_MASK                       0x1u
#define I2C_IBSR_RXAK_SHIFT                      0
#define I2C_IBSR_IBIF_MASK                       0x2u
#define I2C_IBSR_IBIF_SHIFT                      1
#define I2C_IBSR_SRW_MASK                        0x4u
#define I2C_IBSR_SRW_SHIFT                       2
#define I2C_IBSR_IBAL_MASK                       0x10u
#define I2C_IBSR_IBAL_SHIFT                      4
#define I2C_IBSR_IBB_MASK                        0x20u
#define I2C_IBSR_IBB_SHIFT                       5
#define I2C_IBSR_IAAS_MASK                       0x40u
#define I2C_IBSR_IAAS_SHIFT                      6
#define I2C_IBSR_TCF_MASK                        0x80u
#define I2C_IBSR_TCF_SHIFT                       7
/* IBDR Bit Fields */
#define I2C_IBDR_DATA_MASK                       0xFFu
#define I2C_IBDR_DATA_SHIFT                      0
#define I2C_IBDR_DATA(x)                         (((uint8_t)(((uint8_t)(x))<<I2C_IBDR_DATA_SHIFT))&I2C_IBDR_DATA_MASK)
/* IBIC Bit Fields */
#define I2C_IBIC_BIIE_MASK                       0x80u
#define I2C_IBIC_BIIE_SHIFT                      7
/* IBDBG Bit Fields */
#define I2C_IBDBG_IPG_DEBUG_EN_MASK              0x1u
#define I2C_IBDBG_IPG_DEBUG_EN_SHIFT             0
#define I2C_IBDBG_IPG_DEBUG_HALTED_MASK          0x2u
#define I2C_IBDBG_IPG_DEBUG_HALTED_SHIFT         1

/*!
 * @}
 */ /* end of group I2C_Register_Masks */


/* I2C - Peripheral instance base addresses */
/** Peripheral I2C0 base pointer */
#define I2C0_BASE_PTR                            ((I2C_MemMapPtr)0x40066000u)
/** Peripheral I2C1 base pointer */
#define I2C1_BASE_PTR                            ((I2C_MemMapPtr)0x40067000u)
/** Peripheral I2C2 base pointer */
#define I2C2_BASE_PTR                            ((I2C_MemMapPtr)0x400E6000u)
/** Peripheral I2C3 base pointer */
#define I2C3_BASE_PTR                            ((I2C_MemMapPtr)0x400E7000u)
/** Array initializer of I2C peripheral base pointers */
#define I2C_BASE_PTRS                            { I2C0_BASE_PTR, I2C1_BASE_PTR, I2C2_BASE_PTR, I2C3_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- LPTMR
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPTMR_Peripheral LPTMR
 * @{
 */

/** LPTMR - Peripheral register structure */
typedef struct LPTMR_MemMap {
  uint32_t CSR;                                    /**< Low Power Timer Control Status Register, offset: 0x0 */
  uint32_t PSR;                                    /**< Low Power Timer Prescale Register, offset: 0x4 */
  uint32_t CMR;                                    /**< Low Power Timer Compare Register, offset: 0x8 */
  uint32_t CNR;                                    /**< Low Power Timer Counter Register, offset: 0xC */
} volatile *LPTMR_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- LPTMR - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPTMR_Register_Accessor_Macros LPTMR - Register accessor macros
 * @{
 */


/* LPTMR - Register accessors */
#define LPTMR_CSR_REG(base)                      ((base)->CSR)
#define LPTMR_PSR_REG(base)                      ((base)->PSR)
#define LPTMR_CMR_REG(base)                      ((base)->CMR)
#define LPTMR_CNR_REG(base)                      ((base)->CNR)

/*!
 * @}
 */ /* end of group LPTMR_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- LPTMR Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPTMR_Register_Masks LPTMR Register Masks
 * @{
 */

/* CSR Bit Fields */
#define LPTMR_CSR_TEN_MASK                       0x1u
#define LPTMR_CSR_TEN_SHIFT                      0
#define LPTMR_CSR_TMS_MASK                       0x2u
#define LPTMR_CSR_TMS_SHIFT                      1
#define LPTMR_CSR_TFC_MASK                       0x4u
#define LPTMR_CSR_TFC_SHIFT                      2
#define LPTMR_CSR_TPP_MASK                       0x8u
#define LPTMR_CSR_TPP_SHIFT                      3
#define LPTMR_CSR_TPS_MASK                       0x30u
#define LPTMR_CSR_TPS_SHIFT                      4
#define LPTMR_CSR_TPS(x)                         (((uint32_t)(((uint32_t)(x))<<LPTMR_CSR_TPS_SHIFT))&LPTMR_CSR_TPS_MASK)
#define LPTMR_CSR_TIE_MASK                       0x40u
#define LPTMR_CSR_TIE_SHIFT                      6
#define LPTMR_CSR_TCF_MASK                       0x80u
#define LPTMR_CSR_TCF_SHIFT                      7
/* PSR Bit Fields */
#define LPTMR_PSR_PCS_MASK                       0x3u
#define LPTMR_PSR_PCS_SHIFT                      0
#define LPTMR_PSR_PCS(x)                         (((uint32_t)(((uint32_t)(x))<<LPTMR_PSR_PCS_SHIFT))&LPTMR_PSR_PCS_MASK)
#define LPTMR_PSR_PBYP_MASK                      0x4u
#define LPTMR_PSR_PBYP_SHIFT                     2
#define LPTMR_PSR_PRESCALE_MASK                  0x78u
#define LPTMR_PSR_PRESCALE_SHIFT                 3
#define LPTMR_PSR_PRESCALE(x)                    (((uint32_t)(((uint32_t)(x))<<LPTMR_PSR_PRESCALE_SHIFT))&LPTMR_PSR_PRESCALE_MASK)
/* CMR Bit Fields */
#define LPTMR_CMR_COMPARE_MASK                   0xFFFFu
#define LPTMR_CMR_COMPARE_SHIFT                  0
#define LPTMR_CMR_COMPARE(x)                     (((uint32_t)(((uint32_t)(x))<<LPTMR_CMR_COMPARE_SHIFT))&LPTMR_CMR_COMPARE_MASK)
/* CNR Bit Fields */
#define LPTMR_CNR_COUNTER_MASK                   0xFFFFu
#define LPTMR_CNR_COUNTER_SHIFT                  0
#define LPTMR_CNR_COUNTER(x)                     (((uint32_t)(((uint32_t)(x))<<LPTMR_CNR_COUNTER_SHIFT))&LPTMR_CNR_COUNTER_MASK)

/*!
 * @}
 */ /* end of group LPTMR_Register_Masks */


/* LPTMR - Peripheral instance base addresses */
/** Peripheral LPTMR0 base pointer */
#define LPTMR0_BASE_PTR                          ((LPTMR_MemMapPtr)0x40040000u)
/** Array initializer of LPTMR peripheral base pointers */
#define LPTMR_BASE_PTRS                          { LPTMR0_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- MCM
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCM_Peripheral MCM
 * @{
 */

/** MCM - Peripheral register structure */
typedef struct MCM_MemMap {
  uint8_t RESERVED_0[8];
  uint16_t PLASC;                                  /**< Crossbar Switch (AXBS) Slave Configuration, offset: 0x8 */
  uint16_t PLAMC;                                  /**< Crossbar Switch (AXBS) Master Configuration, offset: 0xA */
  uint32_t CR;                                     /**< Control Register, offset: 0xC */
  uint32_t ISCR;                                   /**< Interrupt Status and Control Register, offset: 0x10 */
  uint8_t RESERVED_1[12];
  uint32_t FADR;                                   /**< Fault address register, offset: 0x20 */
  uint32_t FATR;                                   /**< Fault attributes register, offset: 0x24 */
  uint32_t FDR;                                    /**< Fault data register, offset: 0x28 */
} volatile *MCM_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- MCM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCM_Register_Accessor_Macros MCM - Register accessor macros
 * @{
 */


/* MCM - Register accessors */
#define MCM_PLASC_REG(base)                      ((base)->PLASC)
#define MCM_PLAMC_REG(base)                      ((base)->PLAMC)
#define MCM_CR_REG(base)                         ((base)->CR)
#define MCM_ISCR_REG(base)                       ((base)->ISCR)
#define MCM_FADR_REG(base)                       ((base)->FADR)
#define MCM_FATR_REG(base)                       ((base)->FATR)
#define MCM_FDR_REG(base)                        ((base)->FDR)

/*!
 * @}
 */ /* end of group MCM_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- MCM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCM_Register_Masks MCM Register Masks
 * @{
 */

/* PLASC Bit Fields */
#define MCM_PLASC_ASC_MASK                       0xFFu
#define MCM_PLASC_ASC_SHIFT                      0
#define MCM_PLASC_ASC(x)                         (((uint16_t)(((uint16_t)(x))<<MCM_PLASC_ASC_SHIFT))&MCM_PLASC_ASC_MASK)
/* PLAMC Bit Fields */
#define MCM_PLAMC_AMC_MASK                       0xFFu
#define MCM_PLAMC_AMC_SHIFT                      0
#define MCM_PLAMC_AMC(x)                         (((uint16_t)(((uint16_t)(x))<<MCM_PLAMC_AMC_SHIFT))&MCM_PLAMC_AMC_MASK)
/* CR Bit Fields */
#define MCM_CR_SRAMUAP_MASK                      0x3000000u
#define MCM_CR_SRAMUAP_SHIFT                     24
#define MCM_CR_SRAMUAP(x)                        (((uint32_t)(((uint32_t)(x))<<MCM_CR_SRAMUAP_SHIFT))&MCM_CR_SRAMUAP_MASK)
#define MCM_CR_SRAMUWP_MASK                      0x4000000u
#define MCM_CR_SRAMUWP_SHIFT                     26
#define MCM_CR_SRAMLAP_MASK                      0x30000000u
#define MCM_CR_SRAMLAP_SHIFT                     28
#define MCM_CR_SRAMLAP(x)                        (((uint32_t)(((uint32_t)(x))<<MCM_CR_SRAMLAP_SHIFT))&MCM_CR_SRAMLAP_MASK)
#define MCM_CR_SRAMLWP_MASK                      0x40000000u
#define MCM_CR_SRAMLWP_SHIFT                     30
/* ISCR Bit Fields */
#define MCM_ISCR_DHREQ_MASK                      0x8u
#define MCM_ISCR_DHREQ_SHIFT                     3
#define MCM_ISCR_CWBER_MASK                      0x10u
#define MCM_ISCR_CWBER_SHIFT                     4
#define MCM_ISCR_FIOC_MASK                       0x100u
#define MCM_ISCR_FIOC_SHIFT                      8
#define MCM_ISCR_FDZC_MASK                       0x200u
#define MCM_ISCR_FDZC_SHIFT                      9
#define MCM_ISCR_FOFC_MASK                       0x400u
#define MCM_ISCR_FOFC_SHIFT                      10
#define MCM_ISCR_FUFC_MASK                       0x800u
#define MCM_ISCR_FUFC_SHIFT                      11
#define MCM_ISCR_FIXC_MASK                       0x1000u
#define MCM_ISCR_FIXC_SHIFT                      12
#define MCM_ISCR_FIDC_MASK                       0x8000u
#define MCM_ISCR_FIDC_SHIFT                      15
#define MCM_ISCR_CWBEE_MASK                      0x100000u
#define MCM_ISCR_CWBEE_SHIFT                     20
#define MCM_ISCR_FIOCE_MASK                      0x1000000u
#define MCM_ISCR_FIOCE_SHIFT                     24
#define MCM_ISCR_FDZCE_MASK                      0x2000000u
#define MCM_ISCR_FDZCE_SHIFT                     25
#define MCM_ISCR_FOFCE_MASK                      0x4000000u
#define MCM_ISCR_FOFCE_SHIFT                     26
#define MCM_ISCR_FUFCE_MASK                      0x8000000u
#define MCM_ISCR_FUFCE_SHIFT                     27
#define MCM_ISCR_FIXCE_MASK                      0x10000000u
#define MCM_ISCR_FIXCE_SHIFT                     28
#define MCM_ISCR_FIDCE_MASK                      0x80000000u
#define MCM_ISCR_FIDCE_SHIFT                     31
/* FADR Bit Fields */
#define MCM_FADR_ADDRESS_MASK                    0xFFFFFFFFu
#define MCM_FADR_ADDRESS_SHIFT                   0
#define MCM_FADR_ADDRESS(x)                      (((uint32_t)(((uint32_t)(x))<<MCM_FADR_ADDRESS_SHIFT))&MCM_FADR_ADDRESS_MASK)
/* FATR Bit Fields */
#define MCM_FATR_BEDA_MASK                       0x1u
#define MCM_FATR_BEDA_SHIFT                      0
#define MCM_FATR_BEMD_MASK                       0x2u
#define MCM_FATR_BEMD_SHIFT                      1
#define MCM_FATR_BESZ_MASK                       0x30u
#define MCM_FATR_BESZ_SHIFT                      4
#define MCM_FATR_BESZ(x)                         (((uint32_t)(((uint32_t)(x))<<MCM_FATR_BESZ_SHIFT))&MCM_FATR_BESZ_MASK)
#define MCM_FATR_BEWT_MASK                       0x80u
#define MCM_FATR_BEWT_SHIFT                      7
#define MCM_FATR_BEMN_MASK                       0xF00u
#define MCM_FATR_BEMN_SHIFT                      8
#define MCM_FATR_BEMN(x)                         (((uint32_t)(((uint32_t)(x))<<MCM_FATR_BEMN_SHIFT))&MCM_FATR_BEMN_MASK)
#define MCM_FATR_BEOVR_MASK                      0x80000000u
#define MCM_FATR_BEOVR_SHIFT                     31
/* FDR Bit Fields */
#define MCM_FDR_DATA_MASK                        0xFFFFFFFFu
#define MCM_FDR_DATA_SHIFT                       0
#define MCM_FDR_DATA(x)                          (((uint32_t)(((uint32_t)(x))<<MCM_FDR_DATA_SHIFT))&MCM_FDR_DATA_MASK)

/*!
 * @}
 */ /* end of group MCM_Register_Masks */


/* MCM - Peripheral instance base addresses */
/** Peripheral MCM base pointer */
#define MCM_BASE_PTR                             ((MCM_MemMapPtr)0xE0080000u)
/** Array initializer of MCM peripheral base pointers */
#define MCM_BASE_PTRS                            { MCM_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- MCM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCM_Register_Accessor_Macros MCM - Register accessor macros
 * @{
 */


/* MCM - Register instance definitions */
/* MCM */
#define MCM_PLASC                                MCM_PLASC_REG(MCM_BASE_PTR)
#define MCM_PLAMC                                MCM_PLAMC_REG(MCM_BASE_PTR)
#define MCM_CR                                   MCM_CR_REG(MCM_BASE_PTR)
#define MCM_ISCR                                 MCM_ISCR_REG(MCM_BASE_PTR)
#define MCM_FADR                                 MCM_FADR_REG(MCM_BASE_PTR)
#define MCM_FATR                                 MCM_FATR_REG(MCM_BASE_PTR)
#define MCM_FDR                                  MCM_FDR_REG(MCM_BASE_PTR)

/*!
 * @}
 */ /* end of group MCM_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group MCM_Peripheral */


/* ----------------------------------------------------------------------------
   -- MSCM
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MSCM_Peripheral MSCM
 * @{
 */

/** MSCM - Peripheral register structure */
typedef struct MSCM_MemMap {
  uint32_t CPxTYPE;                                /**< Processor X Type Register, offset: 0x0 */
  uint32_t CPxNUM;                                 /**< Processor X Number Register, offset: 0x4 */
  uint32_t CPxMASTER;                              /**< Processor X Master Register, offset: 0x8 */
  uint32_t CPxCOUNT;                               /**< Processor X Count Register, offset: 0xC */
  uint32_t CPxCFG[4];                              /**< Processor X Configuration Register, array offset: 0x10, array step: 0x4 */
  uint32_t CP0TYPE;                                /**< Processor 0 Type Register, offset: 0x20 */
  uint32_t CP0NUM;                                 /**< Processor 0 Number Register, offset: 0x24 */
  uint32_t CP0MASTER;                              /**< Processor 0 Master Register, offset: 0x28 */
  uint32_t CP0COUNT;                               /**< Processor 0 Count Register, offset: 0x2C */
  uint32_t CP0CFG[4];                              /**< Processor 0 Configuration Register, array offset: 0x30, array step: 0x4 */
  uint32_t CP1TYPE;                                /**< Processor 1 Type Register, offset: 0x40 */
  uint32_t CP1NUM;                                 /**< Processor 1 Number Register, offset: 0x44 */
  uint32_t CP1MASTER;                              /**< Processor 1 Master Register, offset: 0x48 */
  uint32_t CP1COUNT;                               /**< Processor 1 Count Register, offset: 0x4C */
  uint32_t CP1CFG[4];                              /**< Processor 1 Configuration Register, array offset: 0x50, array step: 0x4 */
  uint8_t RESERVED_0[1952];
  uint32_t IRCP0IR;                                /**< Interrupt Router CP0 Interrupt Register, offset: 0x800 */
  uint32_t IRCP1IR;                                /**< Interrupt Router CP1 Interrupt Register, offset: 0x804 */
  uint8_t RESERVED_1[24];
  uint32_t IRCPGIR;                                /**< Interrupt Router CPU Generate Interrupt Register, offset: 0x820 */
  uint8_t RESERVED_2[92];
  uint16_t IRSPRC[112];                            /**< Interrupt Router Shared Peripheral Routing Control Register n, array offset: 0x880, array step: 0x2 */
  uint8_t RESERVED_3[672];
  uint32_t TZENR;                                  /**< ACTZS TrustZone Enable Register, offset: 0xC00 */
  uint32_t TZIR;                                   /**< ACTZS TrustZone Interrupt Register, offset: 0xC04 */
  uint8_t RESERVED_4[8];
  uint32_t CSLIER;                                 /**< ACTZS CSLn Interrupt Enable Register, offset: 0xC10 */
  uint32_t CSLIR;                                  /**< ACTZS CSLn Interrupt Register, offset: 0xC14 */
  uint32_t CSOVR;                                  /**< ACTZS CSLn Interrupt Overrun Register, offset: 0xC18 */
  uint8_t RESERVED_5[228];
  struct {                                         /* offset: 0xD00, array step: 0x10 */
    uint32_t CSFAR;                                  /**< ACTZS CSLn Fail Status Address (Low) Register, array offset: 0xD00, array step: 0x10 */
    uint8_t RESERVED_0[4];
    uint32_t CSFCR;                                  /**< ACTZS CSLn Fail Status Control Register, array offset: 0xD08, array step: 0x10 */
    uint32_t CSFIR;                                  /**< ACTZS CSLn Fail Status Master ID Register, array offset: 0xD0C, array step: 0x10 */
  } CSFCAP[14];
} volatile *MSCM_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- MSCM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MSCM_Register_Accessor_Macros MSCM - Register accessor macros
 * @{
 */


/* MSCM - Register accessors */
#define MSCM_CPxTYPE_REG(base)                   ((base)->CPxTYPE)
#define MSCM_CPxNUM_REG(base)                    ((base)->CPxNUM)
#define MSCM_CPxMASTER_REG(base)                 ((base)->CPxMASTER)
#define MSCM_CPxCOUNT_REG(base)                  ((base)->CPxCOUNT)
#define MSCM_CPxCFG_REG(base,index)              ((base)->CPxCFG[index])
#define MSCM_CP0TYPE_REG(base)                   ((base)->CP0TYPE)
#define MSCM_CP0NUM_REG(base)                    ((base)->CP0NUM)
#define MSCM_CP0MASTER_REG(base)                 ((base)->CP0MASTER)
#define MSCM_CP0COUNT_REG(base)                  ((base)->CP0COUNT)
#define MSCM_CP0CFG_REG(base,index)              ((base)->CP0CFG[index])
#define MSCM_CP1TYPE_REG(base)                   ((base)->CP1TYPE)
#define MSCM_CP1NUM_REG(base)                    ((base)->CP1NUM)
#define MSCM_CP1MASTER_REG(base)                 ((base)->CP1MASTER)
#define MSCM_CP1COUNT_REG(base)                  ((base)->CP1COUNT)
#define MSCM_CP1CFG_REG(base,index)              ((base)->CP1CFG[index])
#define MSCM_IRCP0IR_REG(base)                   ((base)->IRCP0IR)
#define MSCM_IRCP1IR_REG(base)                   ((base)->IRCP1IR)
#define MSCM_IRCPGIR_REG(base)                   ((base)->IRCPGIR)
#define MSCM_IRSPRC_REG(base,index)              ((base)->IRSPRC[index])
#define MSCM_TZENR_REG(base)                     ((base)->TZENR)
#define MSCM_TZIR_REG(base)                      ((base)->TZIR)
#define MSCM_CSLIER_REG(base)                    ((base)->CSLIER)
#define MSCM_CSLIR_REG(base)                     ((base)->CSLIR)
#define MSCM_CSOVR_REG(base)                     ((base)->CSOVR)
#define MSCM_CSFAR_REG(base,index)               ((base)->CSFCAP[index].CSFAR)
#define MSCM_CSFCR_REG(base,index)               ((base)->CSFCAP[index].CSFCR)
#define MSCM_CSFIR_REG(base,index)               ((base)->CSFCAP[index].CSFIR)

/*!
 * @}
 */ /* end of group MSCM_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- MSCM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MSCM_Register_Masks MSCM Register Masks
 * @{
 */

/* CPxTYPE Bit Fields */
#define MSCM_CPxTYPE_rYpZ_MASK                   0xFFu
#define MSCM_CPxTYPE_rYpZ_SHIFT                  0
#define MSCM_CPxTYPE_rYpZ(x)                     (((uint32_t)(((uint32_t)(x))<<MSCM_CPxTYPE_rYpZ_SHIFT))&MSCM_CPxTYPE_rYpZ_MASK)
#define MSCM_CPxTYPE_Personality_MASK            0xFFFFFF00u
#define MSCM_CPxTYPE_Personality_SHIFT           8
#define MSCM_CPxTYPE_Personality(x)              (((uint32_t)(((uint32_t)(x))<<MSCM_CPxTYPE_Personality_SHIFT))&MSCM_CPxTYPE_Personality_MASK)
/* CPxNUM Bit Fields */
#define MSCM_CPxNUM_CPN_MASK                     0x1u
#define MSCM_CPxNUM_CPN_SHIFT                    0
/* CPxMASTER Bit Fields */
#define MSCM_CPxMASTER_PPN_MASK                  0x1Fu
#define MSCM_CPxMASTER_PPN_SHIFT                 0
#define MSCM_CPxMASTER_PPN(x)                    (((uint32_t)(((uint32_t)(x))<<MSCM_CPxMASTER_PPN_SHIFT))&MSCM_CPxMASTER_PPN_MASK)
/* CPxCOUNT Bit Fields */
#define MSCM_CPxCOUNT_PCNT_MASK                  0x1u
#define MSCM_CPxCOUNT_PCNT_SHIFT                 0
/* CPxCFG Bit Fields */
#define MSCM_CPxCFG_DCWY_MASK                    0xFFu
#define MSCM_CPxCFG_DCWY_SHIFT                   0
#define MSCM_CPxCFG_DCWY(x)                      (((uint32_t)(((uint32_t)(x))<<MSCM_CPxCFG_DCWY_SHIFT))&MSCM_CPxCFG_DCWY_MASK)
#define MSCM_CPxCFG_DCSZ_MASK                    0xFF00u
#define MSCM_CPxCFG_DCSZ_SHIFT                   8
#define MSCM_CPxCFG_DCSZ(x)                      (((uint32_t)(((uint32_t)(x))<<MSCM_CPxCFG_DCSZ_SHIFT))&MSCM_CPxCFG_DCSZ_MASK)
#define MSCM_CPxCFG_ICWY_MASK                    0xFF0000u
#define MSCM_CPxCFG_ICWY_SHIFT                   16
#define MSCM_CPxCFG_ICWY(x)                      (((uint32_t)(((uint32_t)(x))<<MSCM_CPxCFG_ICWY_SHIFT))&MSCM_CPxCFG_ICWY_MASK)
#define MSCM_CPxCFG_ICSZ_MASK                    0xFF000000u
#define MSCM_CPxCFG_ICSZ_SHIFT                   24
#define MSCM_CPxCFG_ICSZ(x)                      (((uint32_t)(((uint32_t)(x))<<MSCM_CPxCFG_ICSZ_SHIFT))&MSCM_CPxCFG_ICSZ_MASK)
/* CP0TYPE Bit Fields */
#define MSCM_CP0TYPE_rYpZ_MASK                   0xFFu
#define MSCM_CP0TYPE_rYpZ_SHIFT                  0
#define MSCM_CP0TYPE_rYpZ(x)                     (((uint32_t)(((uint32_t)(x))<<MSCM_CP0TYPE_rYpZ_SHIFT))&MSCM_CP0TYPE_rYpZ_MASK)
#define MSCM_CP0TYPE_Personality_MASK            0xFFFFFF00u
#define MSCM_CP0TYPE_Personality_SHIFT           8
#define MSCM_CP0TYPE_Personality(x)              (((uint32_t)(((uint32_t)(x))<<MSCM_CP0TYPE_Personality_SHIFT))&MSCM_CP0TYPE_Personality_MASK)
/* CP0NUM Bit Fields */
#define MSCM_CP0NUM_CPN_MASK                     0x1u
#define MSCM_CP0NUM_CPN_SHIFT                    0
/* CP0MASTER Bit Fields */
#define MSCM_CP0MASTER_PPN_MASK                  0x1Fu
#define MSCM_CP0MASTER_PPN_SHIFT                 0
#define MSCM_CP0MASTER_PPN(x)                    (((uint32_t)(((uint32_t)(x))<<MSCM_CP0MASTER_PPN_SHIFT))&MSCM_CP0MASTER_PPN_MASK)
/* CP0COUNT Bit Fields */
#define MSCM_CP0COUNT_PCNT_MASK                  0x1u
#define MSCM_CP0COUNT_PCNT_SHIFT                 0
/* CP0CFG Bit Fields */
#define MSCM_CP0CFG_DCWY_MASK                    0xFFu
#define MSCM_CP0CFG_DCWY_SHIFT                   0
#define MSCM_CP0CFG_DCWY(x)                      (((uint32_t)(((uint32_t)(x))<<MSCM_CP0CFG_DCWY_SHIFT))&MSCM_CP0CFG_DCWY_MASK)
#define MSCM_CP0CFG_DCSZ_MASK                    0xFF00u
#define MSCM_CP0CFG_DCSZ_SHIFT                   8
#define MSCM_CP0CFG_DCSZ(x)                      (((uint32_t)(((uint32_t)(x))<<MSCM_CP0CFG_DCSZ_SHIFT))&MSCM_CP0CFG_DCSZ_MASK)
#define MSCM_CP0CFG_ICWY_MASK                    0xFF0000u
#define MSCM_CP0CFG_ICWY_SHIFT                   16
#define MSCM_CP0CFG_ICWY(x)                      (((uint32_t)(((uint32_t)(x))<<MSCM_CP0CFG_ICWY_SHIFT))&MSCM_CP0CFG_ICWY_MASK)
#define MSCM_CP0CFG_ICSZ_MASK                    0xFF000000u
#define MSCM_CP0CFG_ICSZ_SHIFT                   24
#define MSCM_CP0CFG_ICSZ(x)                      (((uint32_t)(((uint32_t)(x))<<MSCM_CP0CFG_ICSZ_SHIFT))&MSCM_CP0CFG_ICSZ_MASK)
/* CP1TYPE Bit Fields */
#define MSCM_CP1TYPE_rYpZ_MASK                   0xFFu
#define MSCM_CP1TYPE_rYpZ_SHIFT                  0
#define MSCM_CP1TYPE_rYpZ(x)                     (((uint32_t)(((uint32_t)(x))<<MSCM_CP1TYPE_rYpZ_SHIFT))&MSCM_CP1TYPE_rYpZ_MASK)
#define MSCM_CP1TYPE_Personality_MASK            0xFFFFFF00u
#define MSCM_CP1TYPE_Personality_SHIFT           8
#define MSCM_CP1TYPE_Personality(x)              (((uint32_t)(((uint32_t)(x))<<MSCM_CP1TYPE_Personality_SHIFT))&MSCM_CP1TYPE_Personality_MASK)
/* CP1NUM Bit Fields */
#define MSCM_CP1NUM_CPN_MASK                     0x1u
#define MSCM_CP1NUM_CPN_SHIFT                    0
/* CP1MASTER Bit Fields */
#define MSCM_CP1MASTER_PPN_MASK                  0x1Fu
#define MSCM_CP1MASTER_PPN_SHIFT                 0
#define MSCM_CP1MASTER_PPN(x)                    (((uint32_t)(((uint32_t)(x))<<MSCM_CP1MASTER_PPN_SHIFT))&MSCM_CP1MASTER_PPN_MASK)
/* CP1COUNT Bit Fields */
#define MSCM_CP1COUNT_PCNT_MASK                  0x1u
#define MSCM_CP1COUNT_PCNT_SHIFT                 0
/* CP1CFG Bit Fields */
#define MSCM_CP1CFG_DCWY_MASK                    0xFFu
#define MSCM_CP1CFG_DCWY_SHIFT                   0
#define MSCM_CP1CFG_DCWY(x)                      (((uint32_t)(((uint32_t)(x))<<MSCM_CP1CFG_DCWY_SHIFT))&MSCM_CP1CFG_DCWY_MASK)
#define MSCM_CP1CFG_DCSZ_MASK                    0xFF00u
#define MSCM_CP1CFG_DCSZ_SHIFT                   8
#define MSCM_CP1CFG_DCSZ(x)                      (((uint32_t)(((uint32_t)(x))<<MSCM_CP1CFG_DCSZ_SHIFT))&MSCM_CP1CFG_DCSZ_MASK)
#define MSCM_CP1CFG_ICWY_MASK                    0xFF0000u
#define MSCM_CP1CFG_ICWY_SHIFT                   16
#define MSCM_CP1CFG_ICWY(x)                      (((uint32_t)(((uint32_t)(x))<<MSCM_CP1CFG_ICWY_SHIFT))&MSCM_CP1CFG_ICWY_MASK)
#define MSCM_CP1CFG_ICSZ_MASK                    0xFF000000u
#define MSCM_CP1CFG_ICSZ_SHIFT                   24
#define MSCM_CP1CFG_ICSZ(x)                      (((uint32_t)(((uint32_t)(x))<<MSCM_CP1CFG_ICSZ_SHIFT))&MSCM_CP1CFG_ICSZ_MASK)
/* IRCP0IR Bit Fields */
#define MSCM_IRCP0IR_INT0_MASK                   0x1u
#define MSCM_IRCP0IR_INT0_SHIFT                  0
#define MSCM_IRCP0IR_INT1_MASK                   0x2u
#define MSCM_IRCP0IR_INT1_SHIFT                  1
#define MSCM_IRCP0IR_INT2_MASK                   0x4u
#define MSCM_IRCP0IR_INT2_SHIFT                  2
#define MSCM_IRCP0IR_INT3_MASK                   0x8u
#define MSCM_IRCP0IR_INT3_SHIFT                  3
/* IRCP1IR Bit Fields */
#define MSCM_IRCP1IR_INT0_MASK                   0x1u
#define MSCM_IRCP1IR_INT0_SHIFT                  0
#define MSCM_IRCP1IR_INT1_MASK                   0x2u
#define MSCM_IRCP1IR_INT1_SHIFT                  1
#define MSCM_IRCP1IR_INT2_MASK                   0x4u
#define MSCM_IRCP1IR_INT2_SHIFT                  2
#define MSCM_IRCP1IR_INT3_MASK                   0x8u
#define MSCM_IRCP1IR_INT3_SHIFT                  3
/* IRCPGIR Bit Fields */
#define MSCM_IRCPGIR_INTID_MASK                  0x3u
#define MSCM_IRCPGIR_INTID_SHIFT                 0
#define MSCM_IRCPGIR_INTID(x)                    (((uint32_t)(((uint32_t)(x))<<MSCM_IRCPGIR_INTID_SHIFT))&MSCM_IRCPGIR_INTID_MASK)
#define MSCM_IRCPGIR_CPUTL_MASK                  0x30000u
#define MSCM_IRCPGIR_CPUTL_SHIFT                 16
#define MSCM_IRCPGIR_CPUTL(x)                    (((uint32_t)(((uint32_t)(x))<<MSCM_IRCPGIR_CPUTL_SHIFT))&MSCM_IRCPGIR_CPUTL_MASK)
#define MSCM_IRCPGIR_TLF_MASK                    0x3000000u
#define MSCM_IRCPGIR_TLF_SHIFT                   24
#define MSCM_IRCPGIR_TLF(x)                      (((uint32_t)(((uint32_t)(x))<<MSCM_IRCPGIR_TLF_SHIFT))&MSCM_IRCPGIR_TLF_MASK)
/* IRSPRC Bit Fields */
#define MSCM_IRSPRC_CP0En_MASK                   0x1u
#define MSCM_IRSPRC_CP0En_SHIFT                  0
#define MSCM_IRSPRC_CP1En_MASK                   0x2u
#define MSCM_IRSPRC_CP1En_SHIFT                  1
#define MSCM_IRSPRC_RO_MASK                      0x8000u
#define MSCM_IRSPRC_RO_SHIFT                     15
/* TZENR Bit Fields */
#define MSCM_TZENR_TZEN1_MASK                    0x2u
#define MSCM_TZENR_TZEN1_SHIFT                   1
#define MSCM_TZENR_TZEN3_MASK                    0x8u
#define MSCM_TZENR_TZEN3_SHIFT                   3
#define MSCM_TZENR_TZEN4_MASK                    0x10u
#define MSCM_TZENR_TZEN4_SHIFT                   4
#define MSCM_TZENR_TZEN5_MASK                    0x20u
#define MSCM_TZENR_TZEN5_SHIFT                   5
#define MSCM_TZENR_TZEN6_MASK                    0x40u
#define MSCM_TZENR_TZEN6_SHIFT                   6
#define MSCM_TZENR_TZEN7_MASK                    0x80u
#define MSCM_TZENR_TZEN7_SHIFT                   7
#define MSCM_TZENR_TZEN9_MASK                    0x200u
#define MSCM_TZENR_TZEN9_SHIFT                   9
#define MSCM_TZENR_TZEN11_MASK                   0x800u
#define MSCM_TZENR_TZEN11_SHIFT                  11
#define MSCM_TZENR_SBL_MASK                      0x10000000u
#define MSCM_TZENR_SBL_SHIFT                     28
#define MSCM_TZENR_RO_MASK                       0x80000000u
#define MSCM_TZENR_RO_SHIFT                      31
/* TZIR Bit Fields */
#define MSCM_TZIR_TZINT1_MASK                    0x2u
#define MSCM_TZIR_TZINT1_SHIFT                   1
#define MSCM_TZIR_TZINT3_MASK                    0x8u
#define MSCM_TZIR_TZINT3_SHIFT                   3
#define MSCM_TZIR_TZINT4_MASK                    0x10u
#define MSCM_TZIR_TZINT4_SHIFT                   4
#define MSCM_TZIR_TZINT5_MASK                    0x20u
#define MSCM_TZIR_TZINT5_SHIFT                   5
#define MSCM_TZIR_TZINT6_MASK                    0x40u
#define MSCM_TZIR_TZINT6_SHIFT                   6
#define MSCM_TZIR_TZINT7_MASK                    0x80u
#define MSCM_TZIR_TZINT7_SHIFT                   7
#define MSCM_TZIR_TZINT9_MASK                    0x200u
#define MSCM_TZIR_TZINT9_SHIFT                   9
#define MSCM_TZIR_TZINT11_MASK                   0x800u
#define MSCM_TZIR_TZINT11_SHIFT                  11
/* CSLIER Bit Fields */
#define MSCM_CSLIER_CIE0_MASK                    0x1u
#define MSCM_CSLIER_CIE0_SHIFT                   0
#define MSCM_CSLIER_CIE1_MASK                    0x2u
#define MSCM_CSLIER_CIE1_SHIFT                   1
#define MSCM_CSLIER_CIE2_MASK                    0x4u
#define MSCM_CSLIER_CIE2_SHIFT                   2
#define MSCM_CSLIER_CIE3_MASK                    0x8u
#define MSCM_CSLIER_CIE3_SHIFT                   3
#define MSCM_CSLIER_CIE4_MASK                    0x10u
#define MSCM_CSLIER_CIE4_SHIFT                   4
#define MSCM_CSLIER_CIE8_MASK                    0x100u
#define MSCM_CSLIER_CIE8_SHIFT                   8
#define MSCM_CSLIER_CIE9_MASK                    0x200u
#define MSCM_CSLIER_CIE9_SHIFT                   9
#define MSCM_CSLIER_CIE10_MASK                   0x400u
#define MSCM_CSLIER_CIE10_SHIFT                  10
#define MSCM_CSLIER_CIE12_MASK                   0x1000u
#define MSCM_CSLIER_CIE12_SHIFT                  12
#define MSCM_CSLIER_CIE13_MASK                   0x2000u
#define MSCM_CSLIER_CIE13_SHIFT                  13
#define MSCM_CSLIER_RO_MASK                      0x80000000u
#define MSCM_CSLIER_RO_SHIFT                     31
/* CSLIR Bit Fields */
#define MSCM_CSLIR_INT0_MASK                     0x1u
#define MSCM_CSLIR_INT0_SHIFT                    0
#define MSCM_CSLIR_INT1_MASK                     0x2u
#define MSCM_CSLIR_INT1_SHIFT                    1
#define MSCM_CSLIR_INT2_MASK                     0x4u
#define MSCM_CSLIR_INT2_SHIFT                    2
#define MSCM_CSLIR_INT3_MASK                     0x8u
#define MSCM_CSLIR_INT3_SHIFT                    3
#define MSCM_CSLIR_INT4_MASK                     0x10u
#define MSCM_CSLIR_INT4_SHIFT                    4
#define MSCM_CSLIR_INT8_MASK                     0x100u
#define MSCM_CSLIR_INT8_SHIFT                    8
#define MSCM_CSLIR_INT9_MASK                     0x200u
#define MSCM_CSLIR_INT9_SHIFT                    9
#define MSCM_CSLIR_INT10_MASK                    0x400u
#define MSCM_CSLIR_INT10_SHIFT                   10
#define MSCM_CSLIR_INT12_MASK                    0x1000u
#define MSCM_CSLIR_INT12_SHIFT                   12
#define MSCM_CSLIR_INT13_MASK                    0x2000u
#define MSCM_CSLIR_INT13_SHIFT                   13
/* CSOVR Bit Fields */
#define MSCM_CSOVR_OVR0_MASK                     0x1u
#define MSCM_CSOVR_OVR0_SHIFT                    0
#define MSCM_CSOVR_OVR1_MASK                     0x2u
#define MSCM_CSOVR_OVR1_SHIFT                    1
#define MSCM_CSOVR_OVR2_MASK                     0x4u
#define MSCM_CSOVR_OVR2_SHIFT                    2
#define MSCM_CSOVR_OVR3_MASK                     0x8u
#define MSCM_CSOVR_OVR3_SHIFT                    3
#define MSCM_CSOVR_OVR4_MASK                     0x10u
#define MSCM_CSOVR_OVR4_SHIFT                    4
#define MSCM_CSOVR_OVR8_MASK                     0x100u
#define MSCM_CSOVR_OVR8_SHIFT                    8
#define MSCM_CSOVR_OVR9_MASK                     0x200u
#define MSCM_CSOVR_OVR9_SHIFT                    9
#define MSCM_CSOVR_OVR10_MASK                    0x400u
#define MSCM_CSOVR_OVR10_SHIFT                   10
#define MSCM_CSOVR_OVR12_MASK                    0x1000u
#define MSCM_CSOVR_OVR12_SHIFT                   12
#define MSCM_CSOVR_OVR13_MASK                    0x2000u
#define MSCM_CSOVR_OVR13_SHIFT                   13
/* CSFAR Bit Fields */
#define MSCM_CSFAR_FAD_MASK                      0xFFFFFFFFu
#define MSCM_CSFAR_FAD_SHIFT                     0
#define MSCM_CSFAR_FAD(x)                        (((uint32_t)(((uint32_t)(x))<<MSCM_CSFAR_FAD_SHIFT))&MSCM_CSFAR_FAD_MASK)
/* CSFCR Bit Fields */
#define MSCM_CSFCR_FPR_MASK                      0x100000u
#define MSCM_CSFCR_FPR_SHIFT                     20
#define MSCM_CSFCR_FNS_MASK                      0x200000u
#define MSCM_CSFCR_FNS_SHIFT                     21
#define MSCM_CSFCR_FWT_MASK                      0x1000000u
#define MSCM_CSFCR_FWT_SHIFT                     24
/* CSFIR Bit Fields */
#define MSCM_CSFIR_FMID_MASK                     0x1Fu
#define MSCM_CSFIR_FMID_SHIFT                    0
#define MSCM_CSFIR_FMID(x)                       (((uint32_t)(((uint32_t)(x))<<MSCM_CSFIR_FMID_SHIFT))&MSCM_CSFIR_FMID_MASK)

/*!
 * @}
 */ /* end of group MSCM_Register_Masks */


/* MSCM - Peripheral instance base addresses */
/** Peripheral MSCM base pointer */
#define MSCM_BASE_PTR                            ((MSCM_MemMapPtr)0x40001000u)
/** Array initializer of MSCM peripheral base pointers */
#define MSCM_BASE_PTRS                           { MSCM_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- MSCM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MSCM_Register_Accessor_Macros MSCM - Register accessor macros
 * @{
 */


/* MSCM - Register instance definitions */
/* MSCM */
#define MSCM_CPxTYPE                             MSCM_CPxTYPE_REG(MSCM_BASE_PTR)
#define MSCM_CPxNUM                              MSCM_CPxNUM_REG(MSCM_BASE_PTR)
#define MSCM_CPxMASTER                           MSCM_CPxMASTER_REG(MSCM_BASE_PTR)
#define MSCM_CPxCOUNT                            MSCM_CPxCOUNT_REG(MSCM_BASE_PTR)
#define MSCM_CPxCFG0                             MSCM_CPxCFG_REG(MSCM_BASE_PTR,0)
#define MSCM_CPxCFG1                             MSCM_CPxCFG_REG(MSCM_BASE_PTR,1)
#define MSCM_CPxCFG2                             MSCM_CPxCFG_REG(MSCM_BASE_PTR,2)
#define MSCM_CPxCFG3                             MSCM_CPxCFG_REG(MSCM_BASE_PTR,3)
#define MSCM_CP0TYPE                             MSCM_CP0TYPE_REG(MSCM_BASE_PTR)
#define MSCM_CP0NUM                              MSCM_CP0NUM_REG(MSCM_BASE_PTR)
#define MSCM_CP0MASTER                           MSCM_CP0MASTER_REG(MSCM_BASE_PTR)
#define MSCM_CP0COUNT                            MSCM_CP0COUNT_REG(MSCM_BASE_PTR)
#define MSCM_CP0CFG0                             MSCM_CP0CFG_REG(MSCM_BASE_PTR,0)
#define MSCM_CP0CFG1                             MSCM_CP0CFG_REG(MSCM_BASE_PTR,1)
#define MSCM_CP0CFG2                             MSCM_CP0CFG_REG(MSCM_BASE_PTR,2)
#define MSCM_CP0CFG3                             MSCM_CP0CFG_REG(MSCM_BASE_PTR,3)
#define MSCM_CP1TYPE                             MSCM_CP1TYPE_REG(MSCM_BASE_PTR)
#define MSCM_CP1NUM                              MSCM_CP1NUM_REG(MSCM_BASE_PTR)
#define MSCM_CP1MASTER                           MSCM_CP1MASTER_REG(MSCM_BASE_PTR)
#define MSCM_CP1COUNT                            MSCM_CP1COUNT_REG(MSCM_BASE_PTR)
#define MSCM_CP1CFG0                             MSCM_CP1CFG_REG(MSCM_BASE_PTR,0)
#define MSCM_CP1CFG1                             MSCM_CP1CFG_REG(MSCM_BASE_PTR,1)
#define MSCM_CP1CFG2                             MSCM_CP1CFG_REG(MSCM_BASE_PTR,2)
#define MSCM_CP1CFG3                             MSCM_CP1CFG_REG(MSCM_BASE_PTR,3)
#define MSCM_IRCP0IR                             MSCM_IRCP0IR_REG(MSCM_BASE_PTR)
#define MSCM_IRCP1IR                             MSCM_IRCP1IR_REG(MSCM_BASE_PTR)
#define MSCM_IRCPGIR                             MSCM_IRCPGIR_REG(MSCM_BASE_PTR)
#define MSCM_IRSPRC0                             MSCM_IRSPRC_REG(MSCM_BASE_PTR,0)
#define MSCM_IRSPRC1                             MSCM_IRSPRC_REG(MSCM_BASE_PTR,1)
#define MSCM_IRSPRC2                             MSCM_IRSPRC_REG(MSCM_BASE_PTR,2)
#define MSCM_IRSPRC3                             MSCM_IRSPRC_REG(MSCM_BASE_PTR,3)
#define MSCM_IRSPRC4                             MSCM_IRSPRC_REG(MSCM_BASE_PTR,4)
#define MSCM_IRSPRC5                             MSCM_IRSPRC_REG(MSCM_BASE_PTR,5)
#define MSCM_IRSPRC6                             MSCM_IRSPRC_REG(MSCM_BASE_PTR,6)
#define MSCM_IRSPRC7                             MSCM_IRSPRC_REG(MSCM_BASE_PTR,7)
#define MSCM_IRSPRC8                             MSCM_IRSPRC_REG(MSCM_BASE_PTR,8)
#define MSCM_IRSPRC9                             MSCM_IRSPRC_REG(MSCM_BASE_PTR,9)
#define MSCM_IRSPRC10                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,10)
#define MSCM_IRSPRC11                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,11)
#define MSCM_IRSPRC12                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,12)
#define MSCM_IRSPRC13                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,13)
#define MSCM_IRSPRC14                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,14)
#define MSCM_IRSPRC15                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,15)
#define MSCM_IRSPRC16                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,16)
#define MSCM_IRSPRC17                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,17)
#define MSCM_IRSPRC18                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,18)
#define MSCM_IRSPRC19                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,19)
#define MSCM_IRSPRC20                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,20)
#define MSCM_IRSPRC21                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,21)
#define MSCM_IRSPRC22                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,22)
#define MSCM_IRSPRC23                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,23)
#define MSCM_IRSPRC24                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,24)
#define MSCM_IRSPRC25                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,25)
#define MSCM_IRSPRC26                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,26)
#define MSCM_IRSPRC27                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,27)
#define MSCM_IRSPRC28                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,28)
#define MSCM_IRSPRC29                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,29)
#define MSCM_IRSPRC30                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,30)
#define MSCM_IRSPRC31                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,31)
#define MSCM_IRSPRC32                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,32)
#define MSCM_IRSPRC33                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,33)
#define MSCM_IRSPRC34                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,34)
#define MSCM_IRSPRC35                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,35)
#define MSCM_IRSPRC36                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,36)
#define MSCM_IRSPRC37                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,37)
#define MSCM_IRSPRC38                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,38)
#define MSCM_IRSPRC39                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,39)
#define MSCM_IRSPRC40                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,40)
#define MSCM_IRSPRC41                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,41)
#define MSCM_IRSPRC42                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,42)
#define MSCM_IRSPRC43                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,43)
#define MSCM_IRSPRC44                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,44)
#define MSCM_IRSPRC45                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,45)
#define MSCM_IRSPRC46                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,46)
#define MSCM_IRSPRC47                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,47)
#define MSCM_IRSPRC48                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,48)
#define MSCM_IRSPRC49                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,49)
#define MSCM_IRSPRC50                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,50)
#define MSCM_IRSPRC51                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,51)
#define MSCM_IRSPRC52                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,52)
#define MSCM_IRSPRC53                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,53)
#define MSCM_IRSPRC54                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,54)
#define MSCM_IRSPRC55                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,55)
#define MSCM_IRSPRC56                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,56)
#define MSCM_IRSPRC57                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,57)
#define MSCM_IRSPRC58                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,58)
#define MSCM_IRSPRC59                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,59)
#define MSCM_IRSPRC60                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,60)
#define MSCM_IRSPRC61                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,61)
#define MSCM_IRSPRC62                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,62)
#define MSCM_IRSPRC63                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,63)
#define MSCM_IRSPRC64                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,64)
#define MSCM_IRSPRC65                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,65)
#define MSCM_IRSPRC66                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,66)
#define MSCM_IRSPRC67                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,67)
#define MSCM_IRSPRC68                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,68)
#define MSCM_IRSPRC69                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,69)
#define MSCM_IRSPRC70                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,70)
#define MSCM_IRSPRC71                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,71)
#define MSCM_IRSPRC72                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,72)
#define MSCM_IRSPRC73                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,73)
#define MSCM_IRSPRC74                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,74)
#define MSCM_IRSPRC75                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,75)
#define MSCM_IRSPRC76                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,76)
#define MSCM_IRSPRC77                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,77)
#define MSCM_IRSPRC78                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,78)
#define MSCM_IRSPRC79                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,79)
#define MSCM_IRSPRC80                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,80)
#define MSCM_IRSPRC81                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,81)
#define MSCM_IRSPRC82                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,82)
#define MSCM_IRSPRC83                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,83)
#define MSCM_IRSPRC84                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,84)
#define MSCM_IRSPRC85                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,85)
#define MSCM_IRSPRC86                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,86)
#define MSCM_IRSPRC87                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,87)
#define MSCM_IRSPRC88                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,88)
#define MSCM_IRSPRC89                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,89)
#define MSCM_IRSPRC90                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,90)
#define MSCM_IRSPRC91                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,91)
#define MSCM_IRSPRC92                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,92)
#define MSCM_IRSPRC93                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,93)
#define MSCM_IRSPRC94                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,94)
#define MSCM_IRSPRC95                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,95)
#define MSCM_IRSPRC96                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,96)
#define MSCM_IRSPRC97                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,97)
#define MSCM_IRSPRC98                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,98)
#define MSCM_IRSPRC99                            MSCM_IRSPRC_REG(MSCM_BASE_PTR,99)
#define MSCM_IRSPRC100                           MSCM_IRSPRC_REG(MSCM_BASE_PTR,100)
#define MSCM_IRSPRC101                           MSCM_IRSPRC_REG(MSCM_BASE_PTR,101)
#define MSCM_IRSPRC102                           MSCM_IRSPRC_REG(MSCM_BASE_PTR,102)
#define MSCM_IRSPRC103                           MSCM_IRSPRC_REG(MSCM_BASE_PTR,103)
#define MSCM_IRSPRC104                           MSCM_IRSPRC_REG(MSCM_BASE_PTR,104)
#define MSCM_IRSPRC105                           MSCM_IRSPRC_REG(MSCM_BASE_PTR,105)
#define MSCM_IRSPRC106                           MSCM_IRSPRC_REG(MSCM_BASE_PTR,106)
#define MSCM_IRSPRC107                           MSCM_IRSPRC_REG(MSCM_BASE_PTR,107)
#define MSCM_IRSPRC108                           MSCM_IRSPRC_REG(MSCM_BASE_PTR,108)
#define MSCM_IRSPRC109                           MSCM_IRSPRC_REG(MSCM_BASE_PTR,109)
#define MSCM_IRSPRC110                           MSCM_IRSPRC_REG(MSCM_BASE_PTR,110)
#define MSCM_IRSPRC111                           MSCM_IRSPRC_REG(MSCM_BASE_PTR,111)
#define MSCM_TZENR                               MSCM_TZENR_REG(MSCM_BASE_PTR)
#define MSCM_TZIR                                MSCM_TZIR_REG(MSCM_BASE_PTR)
#define MSCM_CSLIER                              MSCM_CSLIER_REG(MSCM_BASE_PTR)
#define MSCM_CSLIR                               MSCM_CSLIR_REG(MSCM_BASE_PTR)
#define MSCM_CSOVR                               MSCM_CSOVR_REG(MSCM_BASE_PTR)
#define MSCM_CSFAR0                              MSCM_CSFAR_REG(MSCM_BASE_PTR,0)
#define MSCM_CSFCR0                              MSCM_CSFCR_REG(MSCM_BASE_PTR,0)
#define MSCM_CSFIR0                              MSCM_CSFIR_REG(MSCM_BASE_PTR,0)
#define MSCM_CSFAR1                              MSCM_CSFAR_REG(MSCM_BASE_PTR,1)
#define MSCM_CSFCR1                              MSCM_CSFCR_REG(MSCM_BASE_PTR,1)
#define MSCM_CSFIR1                              MSCM_CSFIR_REG(MSCM_BASE_PTR,1)
#define MSCM_CSFAR2                              MSCM_CSFAR_REG(MSCM_BASE_PTR,2)
#define MSCM_CSFCR2                              MSCM_CSFCR_REG(MSCM_BASE_PTR,2)
#define MSCM_CSFIR2                              MSCM_CSFIR_REG(MSCM_BASE_PTR,2)
#define MSCM_CSFAR3                              MSCM_CSFAR_REG(MSCM_BASE_PTR,3)
#define MSCM_CSFCR3                              MSCM_CSFCR_REG(MSCM_BASE_PTR,3)
#define MSCM_CSFIR3                              MSCM_CSFIR_REG(MSCM_BASE_PTR,3)
#define MSCM_CSFAR4                              MSCM_CSFAR_REG(MSCM_BASE_PTR,4)
#define MSCM_CSFCR4                              MSCM_CSFCR_REG(MSCM_BASE_PTR,4)
#define MSCM_CSFIR4                              MSCM_CSFIR_REG(MSCM_BASE_PTR,4)
#define MSCM_CSFAR5                              MSCM_CSFAR_REG(MSCM_BASE_PTR,5)
#define MSCM_CSFCR5                              MSCM_CSFCR_REG(MSCM_BASE_PTR,5)
#define MSCM_CSFIR5                              MSCM_CSFIR_REG(MSCM_BASE_PTR,5)
#define MSCM_CSFAR6                              MSCM_CSFAR_REG(MSCM_BASE_PTR,6)
#define MSCM_CSFCR6                              MSCM_CSFCR_REG(MSCM_BASE_PTR,6)
#define MSCM_CSFIR6                              MSCM_CSFIR_REG(MSCM_BASE_PTR,6)
#define MSCM_CSFAR7                              MSCM_CSFAR_REG(MSCM_BASE_PTR,7)
#define MSCM_CSFCR7                              MSCM_CSFCR_REG(MSCM_BASE_PTR,7)
#define MSCM_CSFIR7                              MSCM_CSFIR_REG(MSCM_BASE_PTR,7)
#define MSCM_CSFAR8                              MSCM_CSFAR_REG(MSCM_BASE_PTR,8)
#define MSCM_CSFCR8                              MSCM_CSFCR_REG(MSCM_BASE_PTR,8)
#define MSCM_CSFIR8                              MSCM_CSFIR_REG(MSCM_BASE_PTR,8)
#define MSCM_CSFAR9                              MSCM_CSFAR_REG(MSCM_BASE_PTR,9)
#define MSCM_CSFCR9                              MSCM_CSFCR_REG(MSCM_BASE_PTR,9)
#define MSCM_CSFIR9                              MSCM_CSFIR_REG(MSCM_BASE_PTR,9)
#define MSCM_CSFAR10                             MSCM_CSFAR_REG(MSCM_BASE_PTR,10)
#define MSCM_CSFCR10                             MSCM_CSFCR_REG(MSCM_BASE_PTR,10)
#define MSCM_CSFIR10                             MSCM_CSFIR_REG(MSCM_BASE_PTR,10)
#define MSCM_CSFAR11                             MSCM_CSFAR_REG(MSCM_BASE_PTR,11)
#define MSCM_CSFCR11                             MSCM_CSFCR_REG(MSCM_BASE_PTR,11)
#define MSCM_CSFIR11                             MSCM_CSFIR_REG(MSCM_BASE_PTR,11)
#define MSCM_CSFAR12                             MSCM_CSFAR_REG(MSCM_BASE_PTR,12)
#define MSCM_CSFCR12                             MSCM_CSFCR_REG(MSCM_BASE_PTR,12)
#define MSCM_CSFIR12                             MSCM_CSFIR_REG(MSCM_BASE_PTR,12)
#define MSCM_CSFAR13                             MSCM_CSFAR_REG(MSCM_BASE_PTR,13)
#define MSCM_CSFCR13                             MSCM_CSFCR_REG(MSCM_BASE_PTR,13)
#define MSCM_CSFIR13                             MSCM_CSFIR_REG(MSCM_BASE_PTR,13)

/* MSCM - Register array accessors */
#define MSCM_CPxCFG(index)                       MSCM_CPxCFG_REG(MSCM_BASE_PTR,index)
#define MSCM_CP0CFG(index)                       MSCM_CP0CFG_REG(MSCM_BASE_PTR,index)
#define MSCM_CP1CFG(index)                       MSCM_CP1CFG_REG(MSCM_BASE_PTR,index)
#define MSCM_IRSPRC(index)                       MSCM_IRSPRC_REG(MSCM_BASE_PTR,index)
#define MSCM_CSFAR(index)                        MSCM_CSFAR_REG(MSCM_BASE_PTR,index)
#define MSCM_CSFCR(index)                        MSCM_CSFCR_REG(MSCM_BASE_PTR,index)
#define MSCM_CSFIR(index)                        MSCM_CSFIR_REG(MSCM_BASE_PTR,index)

/*!
 * @}
 */ /* end of group MSCM_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group MSCM_Peripheral */

/* ----------------------------------------------------------------------------
   -- PORT
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PORT_Peripheral PORT
 * @{
 */

/** PORT - Peripheral register structure */
typedef struct PORT_MemMap {
  uint32_t PCR[32];                                /**< Pin Control Register n, array offset: 0x0, array step: 0x4 */
  uint8_t RESERVED_0[32];
  uint32_t ISFR;                                   /**< Interrupt Status Flag Register, offset: 0xA0 */
  uint8_t RESERVED_1[28];
  uint32_t DFER;                                   /**< Digital Filter Enable Register, offset: 0xC0 */
  uint32_t DFCR;                                   /**< Digital Filter Clock Register, offset: 0xC4 */
  uint32_t DFWR;                                   /**< Digital Filter Width Register, offset: 0xC8 */
} volatile *PORT_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- PORT - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PORT_Register_Accessor_Macros PORT - Register accessor macros
 * @{
 */


/* PORT - Register accessors */
#define PORT_PCR_REG(base,index)                 ((base)->PCR[index])
#define PORT_ISFR_REG(base)                      ((base)->ISFR)
#define PORT_DFER_REG(base)                      ((base)->DFER)
#define PORT_DFCR_REG(base)                      ((base)->DFCR)
#define PORT_DFWR_REG(base)                      ((base)->DFWR)

/*!
 * @}
 */ /* end of group PORT_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- PORT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PORT_Register_Masks PORT Register Masks
 * @{
 */

/* PCR Bit Fields */
#define PORT_PCR_IRQC_MASK                       0xF0000u
#define PORT_PCR_IRQC_SHIFT                      16
#define PORT_PCR_IRQC(x)                         (((uint32_t)(((uint32_t)(x))<<PORT_PCR_IRQC_SHIFT))&PORT_PCR_IRQC_MASK)
#define PORT_PCR_ISF_MASK                        0x1000000u
#define PORT_PCR_ISF_SHIFT                       24
/* ISFR Bit Fields */
#define PORT_ISFR_ISF_MASK                       0xFFFFFFFFu
#define PORT_ISFR_ISF_SHIFT                      0
#define PORT_ISFR_ISF(x)                         (((uint32_t)(((uint32_t)(x))<<PORT_ISFR_ISF_SHIFT))&PORT_ISFR_ISF_MASK)
/* DFER Bit Fields */
#define PORT_DFER_DFE_MASK                       0xFFFFFFFFu
#define PORT_DFER_DFE_SHIFT                      0
#define PORT_DFER_DFE(x)                         (((uint32_t)(((uint32_t)(x))<<PORT_DFER_DFE_SHIFT))&PORT_DFER_DFE_MASK)
/* DFCR Bit Fields */
#define PORT_DFCR_CS_MASK                        0x1u
#define PORT_DFCR_CS_SHIFT                       0
/* DFWR Bit Fields */
#define PORT_DFWR_FILT_MASK                      0x1Fu
#define PORT_DFWR_FILT_SHIFT                     0
#define PORT_DFWR_FILT(x)                        (((uint32_t)(((uint32_t)(x))<<PORT_DFWR_FILT_SHIFT))&PORT_DFWR_FILT_MASK)

/*!
 * @}
 */ /* end of group PORT_Register_Masks */


/* PORT - Peripheral instance base addresses */
/** Peripheral PORT0 base pointer */
#define PORT0_BASE_PTR                           ((PORT_MemMapPtr)0x40049000u)
/** Peripheral PORT1 base pointer */
#define PORT1_BASE_PTR                           ((PORT_MemMapPtr)0x4004A000u)
/** Peripheral PORT2 base pointer */
#define PORT2_BASE_PTR                           ((PORT_MemMapPtr)0x4004B000u)
/** Peripheral PORT3 base pointer */
#define PORT3_BASE_PTR                           ((PORT_MemMapPtr)0x4004C000u)
/** Peripheral PORT4 base pointer */
#define PORT4_BASE_PTR                           ((PORT_MemMapPtr)0x4004D000u)
/** Array initializer of PORT peripheral base pointers */
#define PORT_BASE_PTRS                           { PORT0_BASE_PTR, PORT1_BASE_PTR, PORT2_BASE_PTR, PORT3_BASE_PTR, PORT4_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- PORT - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PORT_Register_Accessor_Macros PORT - Register accessor macros
 * @{
 */


/* PORT - Register instance definitions */
/* PORT0 */
#define PORT0_PCR0                               PORT_PCR_REG(PORT0_BASE_PTR,0)
#define PORT0_PCR1                               PORT_PCR_REG(PORT0_BASE_PTR,1)
#define PORT0_PCR2                               PORT_PCR_REG(PORT0_BASE_PTR,2)
#define PORT0_PCR3                               PORT_PCR_REG(PORT0_BASE_PTR,3)
#define PORT0_PCR4                               PORT_PCR_REG(PORT0_BASE_PTR,4)
#define PORT0_PCR5                               PORT_PCR_REG(PORT0_BASE_PTR,5)
#define PORT0_PCR6                               PORT_PCR_REG(PORT0_BASE_PTR,6)
#define PORT0_PCR7                               PORT_PCR_REG(PORT0_BASE_PTR,7)
#define PORT0_PCR8                               PORT_PCR_REG(PORT0_BASE_PTR,8)
#define PORT0_PCR9                               PORT_PCR_REG(PORT0_BASE_PTR,9)
#define PORT0_PCR10                              PORT_PCR_REG(PORT0_BASE_PTR,10)
#define PORT0_PCR11                              PORT_PCR_REG(PORT0_BASE_PTR,11)
#define PORT0_PCR12                              PORT_PCR_REG(PORT0_BASE_PTR,12)
#define PORT0_PCR13                              PORT_PCR_REG(PORT0_BASE_PTR,13)
#define PORT0_PCR14                              PORT_PCR_REG(PORT0_BASE_PTR,14)
#define PORT0_PCR15                              PORT_PCR_REG(PORT0_BASE_PTR,15)
#define PORT0_PCR16                              PORT_PCR_REG(PORT0_BASE_PTR,16)
#define PORT0_PCR17                              PORT_PCR_REG(PORT0_BASE_PTR,17)
#define PORT0_PCR18                              PORT_PCR_REG(PORT0_BASE_PTR,18)
#define PORT0_PCR19                              PORT_PCR_REG(PORT0_BASE_PTR,19)
#define PORT0_PCR20                              PORT_PCR_REG(PORT0_BASE_PTR,20)
#define PORT0_PCR21                              PORT_PCR_REG(PORT0_BASE_PTR,21)
#define PORT0_PCR22                              PORT_PCR_REG(PORT0_BASE_PTR,22)
#define PORT0_PCR23                              PORT_PCR_REG(PORT0_BASE_PTR,23)
#define PORT0_PCR24                              PORT_PCR_REG(PORT0_BASE_PTR,24)
#define PORT0_PCR25                              PORT_PCR_REG(PORT0_BASE_PTR,25)
#define PORT0_PCR26                              PORT_PCR_REG(PORT0_BASE_PTR,26)
#define PORT0_PCR27                              PORT_PCR_REG(PORT0_BASE_PTR,27)
#define PORT0_PCR28                              PORT_PCR_REG(PORT0_BASE_PTR,28)
#define PORT0_PCR29                              PORT_PCR_REG(PORT0_BASE_PTR,29)
#define PORT0_PCR30                              PORT_PCR_REG(PORT0_BASE_PTR,30)
#define PORT0_PCR31                              PORT_PCR_REG(PORT0_BASE_PTR,31)
#define PORT0_ISFR                               PORT_ISFR_REG(PORT0_BASE_PTR)
#define PORT0_DFER                               PORT_DFER_REG(PORT0_BASE_PTR)
#define PORT0_DFCR                               PORT_DFCR_REG(PORT0_BASE_PTR)
#define PORT0_DFWR                               PORT_DFWR_REG(PORT0_BASE_PTR)
/* PORT1 */
#define PORT1_PCR0                               PORT_PCR_REG(PORT1_BASE_PTR,0)
#define PORT1_PCR1                               PORT_PCR_REG(PORT1_BASE_PTR,1)
#define PORT1_PCR2                               PORT_PCR_REG(PORT1_BASE_PTR,2)
#define PORT1_PCR3                               PORT_PCR_REG(PORT1_BASE_PTR,3)
#define PORT1_PCR4                               PORT_PCR_REG(PORT1_BASE_PTR,4)
#define PORT1_PCR5                               PORT_PCR_REG(PORT1_BASE_PTR,5)
#define PORT1_PCR6                               PORT_PCR_REG(PORT1_BASE_PTR,6)
#define PORT1_PCR7                               PORT_PCR_REG(PORT1_BASE_PTR,7)
#define PORT1_PCR8                               PORT_PCR_REG(PORT1_BASE_PTR,8)
#define PORT1_PCR9                               PORT_PCR_REG(PORT1_BASE_PTR,9)
#define PORT1_PCR10                              PORT_PCR_REG(PORT1_BASE_PTR,10)
#define PORT1_PCR11                              PORT_PCR_REG(PORT1_BASE_PTR,11)
#define PORT1_PCR12                              PORT_PCR_REG(PORT1_BASE_PTR,12)
#define PORT1_PCR13                              PORT_PCR_REG(PORT1_BASE_PTR,13)
#define PORT1_PCR14                              PORT_PCR_REG(PORT1_BASE_PTR,14)
#define PORT1_PCR15                              PORT_PCR_REG(PORT1_BASE_PTR,15)
#define PORT1_PCR16                              PORT_PCR_REG(PORT1_BASE_PTR,16)
#define PORT1_PCR17                              PORT_PCR_REG(PORT1_BASE_PTR,17)
#define PORT1_PCR18                              PORT_PCR_REG(PORT1_BASE_PTR,18)
#define PORT1_PCR19                              PORT_PCR_REG(PORT1_BASE_PTR,19)
#define PORT1_PCR20                              PORT_PCR_REG(PORT1_BASE_PTR,20)
#define PORT1_PCR21                              PORT_PCR_REG(PORT1_BASE_PTR,21)
#define PORT1_PCR22                              PORT_PCR_REG(PORT1_BASE_PTR,22)
#define PORT1_PCR23                              PORT_PCR_REG(PORT1_BASE_PTR,23)
#define PORT1_PCR24                              PORT_PCR_REG(PORT1_BASE_PTR,24)
#define PORT1_PCR25                              PORT_PCR_REG(PORT1_BASE_PTR,25)
#define PORT1_PCR26                              PORT_PCR_REG(PORT1_BASE_PTR,26)
#define PORT1_PCR27                              PORT_PCR_REG(PORT1_BASE_PTR,27)
#define PORT1_PCR28                              PORT_PCR_REG(PORT1_BASE_PTR,28)
#define PORT1_PCR29                              PORT_PCR_REG(PORT1_BASE_PTR,29)
#define PORT1_PCR30                              PORT_PCR_REG(PORT1_BASE_PTR,30)
#define PORT1_PCR31                              PORT_PCR_REG(PORT1_BASE_PTR,31)
#define PORT1_ISFR                               PORT_ISFR_REG(PORT1_BASE_PTR)
#define PORT1_DFER                               PORT_DFER_REG(PORT1_BASE_PTR)
#define PORT1_DFCR                               PORT_DFCR_REG(PORT1_BASE_PTR)
#define PORT1_DFWR                               PORT_DFWR_REG(PORT1_BASE_PTR)
/* PORT2 */
#define PORT2_PCR0                               PORT_PCR_REG(PORT2_BASE_PTR,0)
#define PORT2_PCR1                               PORT_PCR_REG(PORT2_BASE_PTR,1)
#define PORT2_PCR2                               PORT_PCR_REG(PORT2_BASE_PTR,2)
#define PORT2_PCR3                               PORT_PCR_REG(PORT2_BASE_PTR,3)
#define PORT2_PCR4                               PORT_PCR_REG(PORT2_BASE_PTR,4)
#define PORT2_PCR5                               PORT_PCR_REG(PORT2_BASE_PTR,5)
#define PORT2_PCR6                               PORT_PCR_REG(PORT2_BASE_PTR,6)
#define PORT2_PCR7                               PORT_PCR_REG(PORT2_BASE_PTR,7)
#define PORT2_PCR8                               PORT_PCR_REG(PORT2_BASE_PTR,8)
#define PORT2_PCR9                               PORT_PCR_REG(PORT2_BASE_PTR,9)
#define PORT2_PCR10                              PORT_PCR_REG(PORT2_BASE_PTR,10)
#define PORT2_PCR11                              PORT_PCR_REG(PORT2_BASE_PTR,11)
#define PORT2_PCR12                              PORT_PCR_REG(PORT2_BASE_PTR,12)
#define PORT2_PCR13                              PORT_PCR_REG(PORT2_BASE_PTR,13)
#define PORT2_PCR14                              PORT_PCR_REG(PORT2_BASE_PTR,14)
#define PORT2_PCR15                              PORT_PCR_REG(PORT2_BASE_PTR,15)
#define PORT2_PCR16                              PORT_PCR_REG(PORT2_BASE_PTR,16)
#define PORT2_PCR17                              PORT_PCR_REG(PORT2_BASE_PTR,17)
#define PORT2_PCR18                              PORT_PCR_REG(PORT2_BASE_PTR,18)
#define PORT2_PCR19                              PORT_PCR_REG(PORT2_BASE_PTR,19)
#define PORT2_PCR20                              PORT_PCR_REG(PORT2_BASE_PTR,20)
#define PORT2_PCR21                              PORT_PCR_REG(PORT2_BASE_PTR,21)
#define PORT2_PCR22                              PORT_PCR_REG(PORT2_BASE_PTR,22)
#define PORT2_PCR23                              PORT_PCR_REG(PORT2_BASE_PTR,23)
#define PORT2_PCR24                              PORT_PCR_REG(PORT2_BASE_PTR,24)
#define PORT2_PCR25                              PORT_PCR_REG(PORT2_BASE_PTR,25)
#define PORT2_PCR26                              PORT_PCR_REG(PORT2_BASE_PTR,26)
#define PORT2_PCR27                              PORT_PCR_REG(PORT2_BASE_PTR,27)
#define PORT2_PCR28                              PORT_PCR_REG(PORT2_BASE_PTR,28)
#define PORT2_PCR29                              PORT_PCR_REG(PORT2_BASE_PTR,29)
#define PORT2_PCR30                              PORT_PCR_REG(PORT2_BASE_PTR,30)
#define PORT2_PCR31                              PORT_PCR_REG(PORT2_BASE_PTR,31)
#define PORT2_ISFR                               PORT_ISFR_REG(PORT2_BASE_PTR)
#define PORT2_DFER                               PORT_DFER_REG(PORT2_BASE_PTR)
#define PORT2_DFCR                               PORT_DFCR_REG(PORT2_BASE_PTR)
#define PORT2_DFWR                               PORT_DFWR_REG(PORT2_BASE_PTR)
/* PORT3 */
#define PORT3_PCR0                               PORT_PCR_REG(PORT3_BASE_PTR,0)
#define PORT3_PCR1                               PORT_PCR_REG(PORT3_BASE_PTR,1)
#define PORT3_PCR2                               PORT_PCR_REG(PORT3_BASE_PTR,2)
#define PORT3_PCR3                               PORT_PCR_REG(PORT3_BASE_PTR,3)
#define PORT3_PCR4                               PORT_PCR_REG(PORT3_BASE_PTR,4)
#define PORT3_PCR5                               PORT_PCR_REG(PORT3_BASE_PTR,5)
#define PORT3_PCR6                               PORT_PCR_REG(PORT3_BASE_PTR,6)
#define PORT3_PCR7                               PORT_PCR_REG(PORT3_BASE_PTR,7)
#define PORT3_PCR8                               PORT_PCR_REG(PORT3_BASE_PTR,8)
#define PORT3_PCR9                               PORT_PCR_REG(PORT3_BASE_PTR,9)
#define PORT3_PCR10                              PORT_PCR_REG(PORT3_BASE_PTR,10)
#define PORT3_PCR11                              PORT_PCR_REG(PORT3_BASE_PTR,11)
#define PORT3_PCR12                              PORT_PCR_REG(PORT3_BASE_PTR,12)
#define PORT3_PCR13                              PORT_PCR_REG(PORT3_BASE_PTR,13)
#define PORT3_PCR14                              PORT_PCR_REG(PORT3_BASE_PTR,14)
#define PORT3_PCR15                              PORT_PCR_REG(PORT3_BASE_PTR,15)
#define PORT3_PCR16                              PORT_PCR_REG(PORT3_BASE_PTR,16)
#define PORT3_PCR17                              PORT_PCR_REG(PORT3_BASE_PTR,17)
#define PORT3_PCR18                              PORT_PCR_REG(PORT3_BASE_PTR,18)
#define PORT3_PCR19                              PORT_PCR_REG(PORT3_BASE_PTR,19)
#define PORT3_PCR20                              PORT_PCR_REG(PORT3_BASE_PTR,20)
#define PORT3_PCR21                              PORT_PCR_REG(PORT3_BASE_PTR,21)
#define PORT3_PCR22                              PORT_PCR_REG(PORT3_BASE_PTR,22)
#define PORT3_PCR23                              PORT_PCR_REG(PORT3_BASE_PTR,23)
#define PORT3_PCR24                              PORT_PCR_REG(PORT3_BASE_PTR,24)
#define PORT3_PCR25                              PORT_PCR_REG(PORT3_BASE_PTR,25)
#define PORT3_PCR26                              PORT_PCR_REG(PORT3_BASE_PTR,26)
#define PORT3_PCR27                              PORT_PCR_REG(PORT3_BASE_PTR,27)
#define PORT3_PCR28                              PORT_PCR_REG(PORT3_BASE_PTR,28)
#define PORT3_PCR29                              PORT_PCR_REG(PORT3_BASE_PTR,29)
#define PORT3_PCR30                              PORT_PCR_REG(PORT3_BASE_PTR,30)
#define PORT3_PCR31                              PORT_PCR_REG(PORT3_BASE_PTR,31)
#define PORT3_ISFR                               PORT_ISFR_REG(PORT3_BASE_PTR)
#define PORT3_DFER                               PORT_DFER_REG(PORT3_BASE_PTR)
#define PORT3_DFCR                               PORT_DFCR_REG(PORT3_BASE_PTR)
#define PORT3_DFWR                               PORT_DFWR_REG(PORT3_BASE_PTR)
/* PORT4 */
#define PORT4_PCR0                               PORT_PCR_REG(PORT4_BASE_PTR,0)
#define PORT4_PCR1                               PORT_PCR_REG(PORT4_BASE_PTR,1)
#define PORT4_PCR2                               PORT_PCR_REG(PORT4_BASE_PTR,2)
#define PORT4_PCR3                               PORT_PCR_REG(PORT4_BASE_PTR,3)
#define PORT4_PCR4                               PORT_PCR_REG(PORT4_BASE_PTR,4)
#define PORT4_PCR5                               PORT_PCR_REG(PORT4_BASE_PTR,5)
#define PORT4_PCR6                               PORT_PCR_REG(PORT4_BASE_PTR,6)
#define PORT4_PCR7                               PORT_PCR_REG(PORT4_BASE_PTR,7)
#define PORT4_PCR8                               PORT_PCR_REG(PORT4_BASE_PTR,8)
#define PORT4_PCR9                               PORT_PCR_REG(PORT4_BASE_PTR,9)
#define PORT4_PCR10                              PORT_PCR_REG(PORT4_BASE_PTR,10)
#define PORT4_PCR11                              PORT_PCR_REG(PORT4_BASE_PTR,11)
#define PORT4_PCR12                              PORT_PCR_REG(PORT4_BASE_PTR,12)
#define PORT4_PCR13                              PORT_PCR_REG(PORT4_BASE_PTR,13)
#define PORT4_PCR14                              PORT_PCR_REG(PORT4_BASE_PTR,14)
#define PORT4_PCR15                              PORT_PCR_REG(PORT4_BASE_PTR,15)
#define PORT4_PCR16                              PORT_PCR_REG(PORT4_BASE_PTR,16)
#define PORT4_PCR17                              PORT_PCR_REG(PORT4_BASE_PTR,17)
#define PORT4_PCR18                              PORT_PCR_REG(PORT4_BASE_PTR,18)
#define PORT4_PCR19                              PORT_PCR_REG(PORT4_BASE_PTR,19)
#define PORT4_PCR20                              PORT_PCR_REG(PORT4_BASE_PTR,20)
#define PORT4_PCR21                              PORT_PCR_REG(PORT4_BASE_PTR,21)
#define PORT4_PCR22                              PORT_PCR_REG(PORT4_BASE_PTR,22)
#define PORT4_PCR23                              PORT_PCR_REG(PORT4_BASE_PTR,23)
#define PORT4_PCR24                              PORT_PCR_REG(PORT4_BASE_PTR,24)
#define PORT4_PCR25                              PORT_PCR_REG(PORT4_BASE_PTR,25)
#define PORT4_PCR26                              PORT_PCR_REG(PORT4_BASE_PTR,26)
#define PORT4_PCR27                              PORT_PCR_REG(PORT4_BASE_PTR,27)
#define PORT4_PCR28                              PORT_PCR_REG(PORT4_BASE_PTR,28)
#define PORT4_PCR29                              PORT_PCR_REG(PORT4_BASE_PTR,29)
#define PORT4_PCR30                              PORT_PCR_REG(PORT4_BASE_PTR,30)
#define PORT4_PCR31                              PORT_PCR_REG(PORT4_BASE_PTR,31)
#define PORT4_ISFR                               PORT_ISFR_REG(PORT4_BASE_PTR)
#define PORT4_DFER                               PORT_DFER_REG(PORT4_BASE_PTR)
#define PORT4_DFCR                               PORT_DFCR_REG(PORT4_BASE_PTR)
#define PORT4_DFWR                               PORT_DFWR_REG(PORT4_BASE_PTR)

/* PORT - Register array accessors */
#define PORT0_PCR(index)                         PORT_PCR_REG(PORT0_BASE_PTR,index)
#define PORT1_PCR(index)                         PORT_PCR_REG(PORT1_BASE_PTR,index)
#define PORT2_PCR(index)                         PORT_PCR_REG(PORT2_BASE_PTR,index)
#define PORT3_PCR(index)                         PORT_PCR_REG(PORT3_BASE_PTR,index)
#define PORT4_PCR(index)                         PORT_PCR_REG(PORT4_BASE_PTR,index)

/*!
 * @}
 */ /* end of group PORT_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group PORT_Peripheral */

/* ----------------------------------------------------------------------------
   -- SPI
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SPI_Peripheral SPI
 * @{
 */

/** SPI - Peripheral register structure */
typedef struct SPI_MemMap {
  uint32_t MCR;                                    /**< Module Configuration Register, offset: 0x0 */
  uint8_t RESERVED_0[4];
  uint32_t TCR;                                    /**< Transfer Count Register, offset: 0x8 */
  union {                                          /* offset: 0xC */
    uint32_t CTAR[4];                                /**< Clock and Transfer Attributes Register (In Master Mode), array offset: 0xC, array step: 0x4 */
    uint32_t CTAR_SLAVE[1];                          /**< Clock and Transfer Attributes Register (In Slave Mode), array offset: 0xC, array step: 0x4 */
  };
  uint8_t RESERVED_1[16];
  uint32_t SR;                                     /**< Status Register, offset: 0x2C */
  uint32_t RSER;                                   /**< DMA/Interrupt Request Select and Enable Register, offset: 0x30 */
  union {                                          /* offset: 0x34 */
    uint32_t PUSHR;                                  /**< PUSH TX FIFO Register In Master Mode, offset: 0x34 */
    uint32_t PUSHR_SLAVE;                            /**< PUSH TX FIFO Register In Slave Mode, offset: 0x34 */
  };
  uint32_t POPR;                                   /**< POP RX FIFO Register, offset: 0x38 */
  uint32_t TXFR[4];                                /**< Transmit FIFO Registers, array offset: 0x3C, array step: 0x4 */
  uint8_t RESERVED_2[48];
  uint32_t RXFR[4];                                /**< Receive FIFO Registers, array offset: 0x7C, array step: 0x4 */
} volatile *SPI_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- SPI - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SPI_Register_Accessor_Macros SPI - Register accessor macros
 * @{
 */


/* SPI - Register accessors */
#define SPI_MCR_REG(base)                        ((base)->MCR)
#define SPI_TCR_REG(base)                        ((base)->TCR)
#define SPI_CTAR_REG(base,index2)                ((base)->CTAR[index2])
#define SPI_CTAR_SLAVE_REG(base,index2)          ((base)->CTAR_SLAVE[index2])
#define SPI_SR_REG(base)                         ((base)->SR)
#define SPI_RSER_REG(base)                       ((base)->RSER)
#define SPI_PUSHR_REG(base)                      ((base)->PUSHR)
#define SPI_PUSHR_SLAVE_REG(base)                ((base)->PUSHR_SLAVE)
#define SPI_POPR_REG(base)                       ((base)->POPR)
#define SPI_TXFR_REG(base,index)                 ((base)->TXFR[index])
#define SPI_RXFR_REG(base,index)                 ((base)->RXFR[index])

/*!
 * @}
 */ /* end of group SPI_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- SPI Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SPI_Register_Masks SPI Register Masks
 * @{
 */

/* MCR Bit Fields */
#define SPI_MCR_HALT_MASK                        0x1u
#define SPI_MCR_HALT_SHIFT                       0
#define SPI_MCR_PES_MASK                         0x2u
#define SPI_MCR_PES_SHIFT                        1
#define SPI_MCR_FCPCS_MASK                       0x4u
#define SPI_MCR_FCPCS_SHIFT                      2
#define SPI_MCR_SMPL_PT_MASK                     0x300u
#define SPI_MCR_SMPL_PT_SHIFT                    8
#define SPI_MCR_SMPL_PT(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_MCR_SMPL_PT_SHIFT))&SPI_MCR_SMPL_PT_MASK)
#define SPI_MCR_CLR_RXF_MASK                     0x400u
#define SPI_MCR_CLR_RXF_SHIFT                    10
#define SPI_MCR_CLR_TXF_MASK                     0x800u
#define SPI_MCR_CLR_TXF_SHIFT                    11
#define SPI_MCR_DIS_RXF_MASK                     0x1000u
#define SPI_MCR_DIS_RXF_SHIFT                    12
#define SPI_MCR_DIS_TXF_MASK                     0x2000u
#define SPI_MCR_DIS_TXF_SHIFT                    13
#define SPI_MCR_MDIS_MASK                        0x4000u
#define SPI_MCR_MDIS_SHIFT                       14
#define SPI_MCR_PCSIS_MASK                       0x3F0000u
#define SPI_MCR_PCSIS_SHIFT                      16
#define SPI_MCR_PCSIS(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_MCR_PCSIS_SHIFT))&SPI_MCR_PCSIS_MASK)
#define SPI_MCR_ROOE_MASK                        0x1000000u
#define SPI_MCR_ROOE_SHIFT                       24
#define SPI_MCR_PCSSE_MASK                       0x2000000u
#define SPI_MCR_PCSSE_SHIFT                      25
#define SPI_MCR_MTFE_MASK                        0x4000000u
#define SPI_MCR_MTFE_SHIFT                       26
#define SPI_MCR_FRZ_MASK                         0x8000000u
#define SPI_MCR_FRZ_SHIFT                        27
#define SPI_MCR_DCONF_MASK                       0x30000000u
#define SPI_MCR_DCONF_SHIFT                      28
#define SPI_MCR_DCONF(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_MCR_DCONF_SHIFT))&SPI_MCR_DCONF_MASK)
#define SPI_MCR_CONT_SCKE_MASK                   0x40000000u
#define SPI_MCR_CONT_SCKE_SHIFT                  30
#define SPI_MCR_MSTR_MASK                        0x80000000u
#define SPI_MCR_MSTR_SHIFT                       31
/* TCR Bit Fields */
#define SPI_TCR_SPI_TCNT_MASK                    0xFFFF0000u
#define SPI_TCR_SPI_TCNT_SHIFT                   16
#define SPI_TCR_SPI_TCNT(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_TCR_SPI_TCNT_SHIFT))&SPI_TCR_SPI_TCNT_MASK)
/* CTAR Bit Fields */
#define SPI_CTAR_BR_MASK                         0xFu
#define SPI_CTAR_BR_SHIFT                        0
#define SPI_CTAR_BR(x)                           (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_BR_SHIFT))&SPI_CTAR_BR_MASK)
#define SPI_CTAR_DT_MASK                         0xF0u
#define SPI_CTAR_DT_SHIFT                        4
#define SPI_CTAR_DT(x)                           (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_DT_SHIFT))&SPI_CTAR_DT_MASK)
#define SPI_CTAR_ASC_MASK                        0xF00u
#define SPI_CTAR_ASC_SHIFT                       8
#define SPI_CTAR_ASC(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_ASC_SHIFT))&SPI_CTAR_ASC_MASK)
#define SPI_CTAR_CSSCK_MASK                      0xF000u
#define SPI_CTAR_CSSCK_SHIFT                     12
#define SPI_CTAR_CSSCK(x)                        (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_CSSCK_SHIFT))&SPI_CTAR_CSSCK_MASK)
#define SPI_CTAR_PBR_MASK                        0x30000u
#define SPI_CTAR_PBR_SHIFT                       16
#define SPI_CTAR_PBR(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_PBR_SHIFT))&SPI_CTAR_PBR_MASK)
#define SPI_CTAR_PDT_MASK                        0xC0000u
#define SPI_CTAR_PDT_SHIFT                       18
#define SPI_CTAR_PDT(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_PDT_SHIFT))&SPI_CTAR_PDT_MASK)
#define SPI_CTAR_PASC_MASK                       0x300000u
#define SPI_CTAR_PASC_SHIFT                      20
#define SPI_CTAR_PASC(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_PASC_SHIFT))&SPI_CTAR_PASC_MASK)
#define SPI_CTAR_PCSSCK_MASK                     0xC00000u
#define SPI_CTAR_PCSSCK_SHIFT                    22
#define SPI_CTAR_PCSSCK(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_PCSSCK_SHIFT))&SPI_CTAR_PCSSCK_MASK)
#define SPI_CTAR_LSBFE_MASK                      0x1000000u
#define SPI_CTAR_LSBFE_SHIFT                     24
#define SPI_CTAR_CPHA_MASK                       0x2000000u
#define SPI_CTAR_CPHA_SHIFT                      25
#define SPI_CTAR_CPOL_MASK                       0x4000000u
#define SPI_CTAR_CPOL_SHIFT                      26
#define SPI_CTAR_FMSZ_MASK                       0x78000000u
#define SPI_CTAR_FMSZ_SHIFT                      27
#define SPI_CTAR_FMSZ(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_FMSZ_SHIFT))&SPI_CTAR_FMSZ_MASK)
#define SPI_CTAR_DBR_MASK                        0x80000000u
#define SPI_CTAR_DBR_SHIFT                       31
/* CTAR_SLAVE Bit Fields */
#define SPI_CTAR_SLAVE_PP_MASK                   0x800000u
#define SPI_CTAR_SLAVE_PP_SHIFT                  23
#define SPI_CTAR_SLAVE_PE_MASK                   0x1000000u
#define SPI_CTAR_SLAVE_PE_SHIFT                  24
#define SPI_CTAR_SLAVE_CPHA_MASK                 0x2000000u
#define SPI_CTAR_SLAVE_CPHA_SHIFT                25
#define SPI_CTAR_SLAVE_CPOL_MASK                 0x4000000u
#define SPI_CTAR_SLAVE_CPOL_SHIFT                26
#define SPI_CTAR_SLAVE_FMSZ_MASK                 0xF8000000u
#define SPI_CTAR_SLAVE_FMSZ_SHIFT                27
#define SPI_CTAR_SLAVE_FMSZ(x)                   (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_SLAVE_FMSZ_SHIFT))&SPI_CTAR_SLAVE_FMSZ_MASK)
/* SR Bit Fields */
#define SPI_SR_POPNXTPTR_MASK                    0xFu
#define SPI_SR_POPNXTPTR_SHIFT                   0
#define SPI_SR_POPNXTPTR(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_SR_POPNXTPTR_SHIFT))&SPI_SR_POPNXTPTR_MASK)
#define SPI_SR_RXCTR_MASK                        0xF0u
#define SPI_SR_RXCTR_SHIFT                       4
#define SPI_SR_RXCTR(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_SR_RXCTR_SHIFT))&SPI_SR_RXCTR_MASK)
#define SPI_SR_TXNXTPTR_MASK                     0xF00u
#define SPI_SR_TXNXTPTR_SHIFT                    8
#define SPI_SR_TXNXTPTR(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_SR_TXNXTPTR_SHIFT))&SPI_SR_TXNXTPTR_MASK)
#define SPI_SR_TXCTR_MASK                        0xF000u
#define SPI_SR_TXCTR_SHIFT                       12
#define SPI_SR_TXCTR(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_SR_TXCTR_SHIFT))&SPI_SR_TXCTR_MASK)
#define SPI_SR_RFDF_MASK                         0x20000u
#define SPI_SR_RFDF_SHIFT                        17
#define SPI_SR_RFOF_MASK                         0x80000u
#define SPI_SR_RFOF_SHIFT                        19
#define SPI_SR_SPEF_MASK                         0x200000u
#define SPI_SR_SPEF_SHIFT                        21
#define SPI_SR_TFFF_MASK                         0x2000000u
#define SPI_SR_TFFF_SHIFT                        25
#define SPI_SR_TFUF_MASK                         0x8000000u
#define SPI_SR_TFUF_SHIFT                        27
#define SPI_SR_EOQF_MASK                         0x10000000u
#define SPI_SR_EOQF_SHIFT                        28
#define SPI_SR_TXRXS_MASK                        0x40000000u
#define SPI_SR_TXRXS_SHIFT                       30
#define SPI_SR_TCF_MASK                          0x80000000u
#define SPI_SR_TCF_SHIFT                         31
/* RSER Bit Fields */
#define SPI_RSER_RFDF_DIRS_MASK                  0x10000u
#define SPI_RSER_RFDF_DIRS_SHIFT                 16
#define SPI_RSER_RFDF_RE_MASK                    0x20000u
#define SPI_RSER_RFDF_RE_SHIFT                   17
#define SPI_RSER_RFOF_RE_MASK                    0x80000u
#define SPI_RSER_RFOF_RE_SHIFT                   19
#define SPI_RSER_SPEF_RE_MASK                    0x200000u
#define SPI_RSER_SPEF_RE_SHIFT                   21
#define SPI_RSER_TFFF_DIRS_MASK                  0x1000000u
#define SPI_RSER_TFFF_DIRS_SHIFT                 24
#define SPI_RSER_TFFF_RE_MASK                    0x2000000u
#define SPI_RSER_TFFF_RE_SHIFT                   25
#define SPI_RSER_TFUF_RE_MASK                    0x8000000u
#define SPI_RSER_TFUF_RE_SHIFT                   27
#define SPI_RSER_EOQF_RE_MASK                    0x10000000u
#define SPI_RSER_EOQF_RE_SHIFT                   28
#define SPI_RSER_TCF_RE_MASK                     0x80000000u
#define SPI_RSER_TCF_RE_SHIFT                    31
/* PUSHR Bit Fields */
#define SPI_PUSHR_TXDATA_MASK                    0xFFFFu
#define SPI_PUSHR_TXDATA_SHIFT                   0
#define SPI_PUSHR_TXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_PUSHR_TXDATA_SHIFT))&SPI_PUSHR_TXDATA_MASK)
#define SPI_PUSHR_PCS_MASK                       0x3F0000u
#define SPI_PUSHR_PCS_SHIFT                      16
#define SPI_PUSHR_PCS(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_PUSHR_PCS_SHIFT))&SPI_PUSHR_PCS_MASK)
#define SPI_PUSHR_PP_MCSC_MASK                   0x1000000u
#define SPI_PUSHR_PP_MCSC_SHIFT                  24
#define SPI_PUSHR_PE_MASC_MASK                   0x2000000u
#define SPI_PUSHR_PE_MASC_SHIFT                  25
#define SPI_PUSHR_CTCNT_MASK                     0x4000000u
#define SPI_PUSHR_CTCNT_SHIFT                    26
#define SPI_PUSHR_EOQ_MASK                       0x8000000u
#define SPI_PUSHR_EOQ_SHIFT                      27
#define SPI_PUSHR_CTAS_MASK                      0x70000000u
#define SPI_PUSHR_CTAS_SHIFT                     28
#define SPI_PUSHR_CTAS(x)                        (((uint32_t)(((uint32_t)(x))<<SPI_PUSHR_CTAS_SHIFT))&SPI_PUSHR_CTAS_MASK)
#define SPI_PUSHR_CONT_MASK                      0x80000000u
#define SPI_PUSHR_CONT_SHIFT                     31
/* PUSHR_SLAVE Bit Fields */
#define SPI_PUSHR_SLAVE_TXDATA_MASK              0xFFFFFFFFu
#define SPI_PUSHR_SLAVE_TXDATA_SHIFT             0
#define SPI_PUSHR_SLAVE_TXDATA(x)                (((uint32_t)(((uint32_t)(x))<<SPI_PUSHR_SLAVE_TXDATA_SHIFT))&SPI_PUSHR_SLAVE_TXDATA_MASK)
/* POPR Bit Fields */
#define SPI_POPR_RXDATA_MASK                     0xFFFFFFFFu
#define SPI_POPR_RXDATA_SHIFT                    0
#define SPI_POPR_RXDATA(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_POPR_RXDATA_SHIFT))&SPI_POPR_RXDATA_MASK)
/* TXFR Bit Fields */
#define SPI_TXFR_TXDATA_MASK                     0xFFFFu
#define SPI_TXFR_TXDATA_SHIFT                    0
#define SPI_TXFR_TXDATA(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_TXFR_TXDATA_SHIFT))&SPI_TXFR_TXDATA_MASK)
#define SPI_TXFR_TXCMD_TXDATA_MASK               0xFFFF0000u
#define SPI_TXFR_TXCMD_TXDATA_SHIFT              16
#define SPI_TXFR_TXCMD_TXDATA(x)                 (((uint32_t)(((uint32_t)(x))<<SPI_TXFR_TXCMD_TXDATA_SHIFT))&SPI_TXFR_TXCMD_TXDATA_MASK)
/* RXFR Bit Fields */
#define SPI_RXFR_RXDATA_MASK                     0xFFFFFFFFu
#define SPI_RXFR_RXDATA_SHIFT                    0
#define SPI_RXFR_RXDATA(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_RXFR_RXDATA_SHIFT))&SPI_RXFR_RXDATA_MASK)

/*!
 * @}
 */ /* end of group SPI_Register_Masks */


/* SPI - Peripheral instance base addresses */
/** Peripheral SPI0 base pointer */
#define SPI0_BASE_PTR                            ((SPI_MemMapPtr)0x4002C000u)
/** Peripheral SPI1 base pointer */
#define SPI1_BASE_PTR                            ((SPI_MemMapPtr)0x4002D000u)
/** Peripheral SPI2 base pointer */
#define SPI2_BASE_PTR                            ((SPI_MemMapPtr)0x400AC000u)
/** Peripheral SPI3 base pointer */
#define SPI3_BASE_PTR                            ((SPI_MemMapPtr)0x400AD000u)
/** Array initializer of SPI peripheral base pointers */
#define SPI_BASE_PTRS                            { SPI0_BASE_PTR, SPI1_BASE_PTR, SPI2_BASE_PTR, SPI3_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- SysTick
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SysTick_Peripheral SysTick
 * @{
 */

/** SysTick - Peripheral register structure */
typedef struct SysTick_MemMap {
  uint32_t CSR;                                    /**< SysTick Control and Status Register, offset: 0x0 */
  uint32_t RVR;                                    /**< SysTick Reload Value Register, offset: 0x4 */
  uint32_t CVR;                                    /**< SysTick Current Value Register, offset: 0x8 */
  uint32_t CALIB;                                  /**< SysTick Calibration Value Register, offset: 0xC */
} volatile *SysTick_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- SysTick - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SysTick_Register_Accessor_Macros SysTick - Register accessor macros
 * @{
 */


/* SysTick - Register accessors */
#define SysTick_CSR_REG(base)                    ((base)->CSR)
#define SysTick_RVR_REG(base)                    ((base)->RVR)
#define SysTick_CVR_REG(base)                    ((base)->CVR)
#define SysTick_CALIB_REG(base)                  ((base)->CALIB)

/*!
 * @}
 */ /* end of group SysTick_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- SysTick Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SysTick_Register_Masks SysTick Register Masks
 * @{
 */

/* CSR Bit Fields */
#define SysTick_CSR_ENABLE_MASK                  0x1u
#define SysTick_CSR_ENABLE_SHIFT                 0
#define SysTick_CSR_TICKINT_MASK                 0x2u
#define SysTick_CSR_TICKINT_SHIFT                1
#define SysTick_CSR_CLKSOURCE_MASK               0x4u
#define SysTick_CSR_CLKSOURCE_SHIFT              2
#define SysTick_CSR_COUNTFLAG_MASK               0x10000u
#define SysTick_CSR_COUNTFLAG_SHIFT              16
/* RVR Bit Fields */
#define SysTick_RVR_RELOAD_MASK                  0xFFFFFFu
#define SysTick_RVR_RELOAD_SHIFT                 0
#define SysTick_RVR_RELOAD(x)                    (((uint32_t)(((uint32_t)(x))<<SysTick_RVR_RELOAD_SHIFT))&SysTick_RVR_RELOAD_MASK)
/* CVR Bit Fields */
#define SysTick_CVR_CURRENT_MASK                 0xFFFFFFu
#define SysTick_CVR_CURRENT_SHIFT                0
#define SysTick_CVR_CURRENT(x)                   (((uint32_t)(((uint32_t)(x))<<SysTick_CVR_CURRENT_SHIFT))&SysTick_CVR_CURRENT_MASK)
/* CALIB Bit Fields */
#define SysTick_CALIB_TENMS_MASK                 0xFFFFFFu
#define SysTick_CALIB_TENMS_SHIFT                0
#define SysTick_CALIB_TENMS(x)                   (((uint32_t)(((uint32_t)(x))<<SysTick_CALIB_TENMS_SHIFT))&SysTick_CALIB_TENMS_MASK)
#define SysTick_CALIB_SKEW_MASK                  0x40000000u
#define SysTick_CALIB_SKEW_SHIFT                 30
#define SysTick_CALIB_NOREF_MASK                 0x80000000u
#define SysTick_CALIB_NOREF_SHIFT                31

/*!
 * @}
 */ /* end of group SysTick_Register_Masks */


/* SysTick - Peripheral instance base addresses */
/** Peripheral SysTick base pointer */
#define SysTick_BASE_PTR                         ((SysTick_MemMapPtr)0xE000E010u)
/** Array initializer of SysTick peripheral base pointers */
#define SysTick_BASE_PTRS                        { SysTick_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- SysTick - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SysTick_Register_Accessor_Macros SysTick - Register accessor macros
 * @{
 */


/* SysTick - Register instance definitions */
/* SysTick */
#define SYST_CSR                                 SysTick_CSR_REG(SysTick_BASE_PTR)
#define SYST_RVR                                 SysTick_RVR_REG(SysTick_BASE_PTR)
#define SYST_CVR                                 SysTick_CVR_REG(SysTick_BASE_PTR)
#define SYST_CALIB                               SysTick_CALIB_REG(SysTick_BASE_PTR)

/*!
 * @}
 */ /* end of group SysTick_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group SysTick_Peripheral */

/* ----------------------------------------------------------------------------
   -- WDOG
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup WDOG_Peripheral WDOG
 * @{
 */

/** WDOG - Peripheral register structure */
typedef struct WDOG_MemMap {
  uint16_t WCR;                                    /**< Watchdog Control Register, offset: 0x0 */
  uint16_t WSR;                                    /**< Watchdog Service Register, offset: 0x2 */
  uint16_t WRSR;                                   /**< Watchdog Reset Status Register, offset: 0x4 */
  uint16_t WICR;                                   /**< Watchdog Interrupt Control Register, offset: 0x6 */
  uint16_t WMCR;                                   /**< Watchdog Miscellaneous Control Register, offset: 0x8 */
} volatile *WDOG_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- WDOG - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup WDOG_Register_Accessor_Macros WDOG - Register accessor macros
 * @{
 */


/* WDOG - Register accessors */
#define WDOG_WCR_REG(base)                       ((base)->WCR)
#define WDOG_WSR_REG(base)                       ((base)->WSR)
#define WDOG_WRSR_REG(base)                      ((base)->WRSR)
#define WDOG_WICR_REG(base)                      ((base)->WICR)
#define WDOG_WMCR_REG(base)                      ((base)->WMCR)

/*!
 * @}
 */ /* end of group WDOG_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- WDOG Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup WDOG_Register_Masks WDOG Register Masks
 * @{
 */

/* WCR Bit Fields */
#define WDOG_WCR_WDZST_MASK                      0x1u
#define WDOG_WCR_WDZST_SHIFT                     0
#define WDOG_WCR_WDBG_MASK                       0x2u
#define WDOG_WCR_WDBG_SHIFT                      1
#define WDOG_WCR_WDE_MASK                        0x4u
#define WDOG_WCR_WDE_SHIFT                       2
#define WDOG_WCR_WDT_MASK                        0x8u
#define WDOG_WCR_WDT_SHIFT                       3
#define WDOG_WCR_SRS_MASK                        0x10u
#define WDOG_WCR_SRS_SHIFT                       4
#define WDOG_WCR_WDA_MASK                        0x20u
#define WDOG_WCR_WDA_SHIFT                       5
#define WDOG_WCR_SRE_MASK                        0x40u
#define WDOG_WCR_SRE_SHIFT                       6
#define WDOG_WCR_WDW_MASK                        0x80u
#define WDOG_WCR_WDW_SHIFT                       7
#define WDOG_WCR_WT_MASK                         0xFF00u
#define WDOG_WCR_WT_SHIFT                        8
#define WDOG_WCR_WT(x)                           (((uint16_t)(((uint16_t)(x))<<WDOG_WCR_WT_SHIFT))&WDOG_WCR_WT_MASK)
/* WSR Bit Fields */
#define WDOG_WSR_WSR_MASK                        0xFFFFu
#define WDOG_WSR_WSR_SHIFT                       0
#define WDOG_WSR_WSR(x)                          (((uint16_t)(((uint16_t)(x))<<WDOG_WSR_WSR_SHIFT))&WDOG_WSR_WSR_MASK)
/* WRSR Bit Fields */
#define WDOG_WRSR_SFTW_MASK                      0x1u
#define WDOG_WRSR_SFTW_SHIFT                     0
#define WDOG_WRSR_TOUT_MASK                      0x2u
#define WDOG_WRSR_TOUT_SHIFT                     1
#define WDOG_WRSR_POR_MASK                       0x10u
#define WDOG_WRSR_POR_SHIFT                      4
/* WICR Bit Fields */
#define WDOG_WICR_WICT_MASK                      0xFFu
#define WDOG_WICR_WICT_SHIFT                     0
#define WDOG_WICR_WICT(x)                        (((uint16_t)(((uint16_t)(x))<<WDOG_WICR_WICT_SHIFT))&WDOG_WICR_WICT_MASK)
#define WDOG_WICR_WTIS_MASK                      0x4000u
#define WDOG_WICR_WTIS_SHIFT                     14
#define WDOG_WICR_WIE_MASK                       0x8000u
#define WDOG_WICR_WIE_SHIFT                      15
/* WMCR Bit Fields */
#define WDOG_WMCR_PDE_MASK                       0x1u
#define WDOG_WMCR_PDE_SHIFT                      0

/*!
 * @}
 */ /* end of group WDOG_Register_Masks */


/* WDOG - Peripheral instance base addresses */
/** Peripheral WDOG_M4 base pointer */
#define WDOG_M4_BASE_PTR                         ((WDOG_MemMapPtr)0x4003F000u)
/** Array initializer of WDOG peripheral base pointers */
#define WDOG_BASE_PTRS                           { WDOG_A5_BASE_PTR, WDOG_M4_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- WDOG - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup WDOG_Register_Accessor_Macros WDOG - Register accessor macros
 * @{
 */


/* WDOG - Register instance definitions */
/* WDOG_M4 */
#define WDOG_M4_WCR                              WDOG_WCR_REG(WDOG_M4_BASE_PTR)
#define WDOG_M4_WSR                              WDOG_WSR_REG(WDOG_M4_BASE_PTR)
#define WDOG_M4_WRSR                             WDOG_WRSR_REG(WDOG_M4_BASE_PTR)
#define WDOG_M4_WICR                             WDOG_WICR_REG(WDOG_M4_BASE_PTR)
#define WDOG_M4_WMCR                             WDOG_WMCR_REG(WDOG_M4_BASE_PTR)

/*!
 * @}
 */ /* end of group WDOG_Register_Accessor_Macros */
