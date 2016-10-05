/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright (c) 2016, Toradex AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
**  @addtogroup pin_mux_module pin_mux module documentation
**  @{
*/

#ifndef __PIN_MUX__
#define __PIN_MUX__

/* MODULE pin_mux. */

#include "device_imx.h"

uint32_t iomux_gpio_pad_addr[] = {
	(uint32_t)(&IOMUXC_PTA6),
	(uint32_t)(&IOMUXC_PTA8),
	(uint32_t)(&IOMUXC_PTA9),
	(uint32_t)(&IOMUXC_PTA10),
	(uint32_t)(&IOMUXC_PTA11),
	(uint32_t)(&IOMUXC_PTA12),
	(uint32_t)(&IOMUXC_PTA16),
	(uint32_t)(&IOMUXC_PTA17),
	(uint32_t)(&IOMUXC_PTA18),
	(uint32_t)(&IOMUXC_PTA19),
	(uint32_t)(&IOMUXC_PTA20),
	(uint32_t)(&IOMUXC_PTA21),
	(uint32_t)(&IOMUXC_PTA22),
	(uint32_t)(&IOMUXC_PTA23),
	(uint32_t)(&IOMUXC_PTA24),
	(uint32_t)(&IOMUXC_PTA25),
	(uint32_t)(&IOMUXC_PTA26),
	(uint32_t)(&IOMUXC_PTA27),
	(uint32_t)(&IOMUXC_PTA28),
	(uint32_t)(&IOMUXC_PTA29),
	(uint32_t)(&IOMUXC_PTA30),
	(uint32_t)(&IOMUXC_PTA31),
	(uint32_t)(&IOMUXC_PTB0),
	(uint32_t)(&IOMUXC_PTB1),
	(uint32_t)(&IOMUXC_PTB2),
	(uint32_t)(&IOMUXC_PTB3),
	(uint32_t)(&IOMUXC_PTB4),
	(uint32_t)(&IOMUXC_PTB5),
	(uint32_t)(&IOMUXC_PTB6),
	(uint32_t)(&IOMUXC_PTB7),
	(uint32_t)(&IOMUXC_PTB8),
	(uint32_t)(&IOMUXC_PTB9),
	(uint32_t)(&IOMUXC_PTB10),
	(uint32_t)(&IOMUXC_PTB11),
	(uint32_t)(&IOMUXC_PTB12),
	(uint32_t)(&IOMUXC_PTB13),
	(uint32_t)(&IOMUXC_PTB14),
	(uint32_t)(&IOMUXC_PTB15),
	(uint32_t)(&IOMUXC_PTB16),
	(uint32_t)(&IOMUXC_PTB17),
	(uint32_t)(&IOMUXC_PTB18),
	(uint32_t)(&IOMUXC_PTB19),
	(uint32_t)(&IOMUXC_PTB20),
	(uint32_t)(&IOMUXC_PTB21),
	(uint32_t)(&IOMUXC_PTB22),
	(uint32_t)(&IOMUXC_PTC0),
	(uint32_t)(&IOMUXC_PTC1),
	(uint32_t)(&IOMUXC_PTC2),
	(uint32_t)(&IOMUXC_PTC3),
	(uint32_t)(&IOMUXC_PTC4),
	(uint32_t)(&IOMUXC_PTC5),
	(uint32_t)(&IOMUXC_PTC6),
	(uint32_t)(&IOMUXC_PTC7),
	(uint32_t)(&IOMUXC_PTC8),
	(uint32_t)(&IOMUXC_PTC9),
	(uint32_t)(&IOMUXC_PTC10),
	(uint32_t)(&IOMUXC_PTC11),
	(uint32_t)(&IOMUXC_PTC12),
	(uint32_t)(&IOMUXC_PTC13),
	(uint32_t)(&IOMUXC_PTC14),
	(uint32_t)(&IOMUXC_PTC15),
	(uint32_t)(&IOMUXC_PTC16),
	(uint32_t)(&IOMUXC_PTC17),
	(uint32_t)(&IOMUXC_PTD31),
	(uint32_t)(&IOMUXC_PTD30),
	(uint32_t)(&IOMUXC_PTD29),
	(uint32_t)(&IOMUXC_PTD28),
	(uint32_t)(&IOMUXC_PTD27),
	(uint32_t)(&IOMUXC_PTD26),
	(uint32_t)(&IOMUXC_PTD25),
	(uint32_t)(&IOMUXC_PTD24),
	(uint32_t)(&IOMUXC_PTD23),
	(uint32_t)(&IOMUXC_PTD22),
	(uint32_t)(&IOMUXC_PTD21),
	(uint32_t)(&IOMUXC_PTD20),
	(uint32_t)(&IOMUXC_PTD19),
	(uint32_t)(&IOMUXC_PTD18),
	(uint32_t)(&IOMUXC_PTD17),
	(uint32_t)(&IOMUXC_PTD16),
	(uint32_t)(&IOMUXC_PTD0),
	(uint32_t)(&IOMUXC_PTD1),
	(uint32_t)(&IOMUXC_PTD2),
	(uint32_t)(&IOMUXC_PTD3),
	(uint32_t)(&IOMUXC_PTD4),
	(uint32_t)(&IOMUXC_PTD5),
	(uint32_t)(&IOMUXC_PTD6),
	(uint32_t)(&IOMUXC_PTD7),
	(uint32_t)(&IOMUXC_PTD8),
	(uint32_t)(&IOMUXC_PTD9),
	(uint32_t)(&IOMUXC_PTD10),
	(uint32_t)(&IOMUXC_PTD11),
	(uint32_t)(&IOMUXC_PTD12),
	(uint32_t)(&IOMUXC_PTD13),
	(uint32_t)(&IOMUXC_PTB23),
	(uint32_t)(&IOMUXC_PTB24),
	(uint32_t)(&IOMUXC_PTB25),
	(uint32_t)(&IOMUXC_PTB26),
	(uint32_t)(&IOMUXC_PTB27),
	(uint32_t)(&IOMUXC_PTB28),
	(uint32_t)(&IOMUXC_PTC26),
	(uint32_t)(&IOMUXC_PTC27),
	(uint32_t)(&IOMUXC_PTC28),
	(uint32_t)(&IOMUXC_PTC29),
	(uint32_t)(&IOMUXC_PTC30),
	(uint32_t)(&IOMUXC_PTC31),
	(uint32_t)(&IOMUXC_PTE0),
	(uint32_t)(&IOMUXC_PTE1),
	(uint32_t)(&IOMUXC_PTE2),
	(uint32_t)(&IOMUXC_PTE3),
	(uint32_t)(&IOMUXC_PTE4),
	(uint32_t)(&IOMUXC_PTE5),
	(uint32_t)(&IOMUXC_PTE6),
	(uint32_t)(&IOMUXC_PTE7),
	(uint32_t)(&IOMUXC_PTE8),
	(uint32_t)(&IOMUXC_PTE9),
	(uint32_t)(&IOMUXC_PTE10),
	(uint32_t)(&IOMUXC_PTE11),
	(uint32_t)(&IOMUXC_PTE12),
	(uint32_t)(&IOMUXC_PTE13),
	(uint32_t)(&IOMUXC_PTE14),
	(uint32_t)(&IOMUXC_PTE15),
	(uint32_t)(&IOMUXC_PTE16),
	(uint32_t)(&IOMUXC_PTE17),
	(uint32_t)(&IOMUXC_PTE18),
	(uint32_t)(&IOMUXC_PTE19),
	(uint32_t)(&IOMUXC_PTE20),
	(uint32_t)(&IOMUXC_PTE21),
	(uint32_t)(&IOMUXC_PTE22),
	(uint32_t)(&IOMUXC_PTE23),
	(uint32_t)(&IOMUXC_PTE24),
	(uint32_t)(&IOMUXC_PTE25),
	(uint32_t)(&IOMUXC_PTE26),
	(uint32_t)(&IOMUXC_PTE27),
	(uint32_t)(&IOMUXC_PTE28),
	(uint32_t)(&IOMUXC_PTA7),
};

/*
** ===================================================================
**     Method      :  pin_mux_GPIO (component PinSettings)
*/
/*!
**     @brief
**         GPIO method sets registers according routing settings. Call
**         this method code to route desired pin as GPIO
**     @param
**         uint32_t gpio - gpio number 0-135
*/
/* ===================================================================*/
void configure_gpio_pin(uint32_t gpio);

/*
** ===================================================================
**     Method      :  pin_mux_UART (component PinSettings)
*/
/*!
**     @brief
**         UART method sets registers according routing settings. Call
**         this method code to route desired pins into:
**         UART1, UART2, UART3, UART4, UART5, UART6, UART7
**         peripherals.
**     @param
**         UART_Type* base - UART base address 1..7
*/
/* ===================================================================*/
void configure_uart_pins(UART_Type* base);

#endif /* __PIN_MUX__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
