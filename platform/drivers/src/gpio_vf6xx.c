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

#include "gpio_vf6xx.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/*******************************************************************************
 * GPIO Initialization and Configuration functions
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : GPIO_Init
 * Description   : Initializes the GPIO module according to the specified
 *                 parameters in the initStruct.
 *
 *END**************************************************************************/
void GPIO_Init(GPIO_Type* base, gpio_init_t* initStruct)
{
	uint32_t dir;
	uint32_t iomux_addr;

	/*
	 * Only on Vybrid the input/output buffer enable flags
	 * are part of the shared mux/conf register.
	 */
	iomux_addr = iomux_gpio_pad_addr[initStruct->pin];
	dir = *((volatile uint32_t *)((uint32_t)iomux_addr));

	dir &= ~(0x3);
	if (initStruct->direction == gpioDigitalOutput) {
		dir |= 0x2U;
	} else if (initStruct->direction == gpioDigitalInput) {
		dir |= 0x1U;
	}

	*((volatile uint32_t *)((uint32_t)iomux_addr)) = dir;
}

/*******************************************************************************
 * GPIO Read and Write Functions
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : GPIO_WritePinOutput
 * Description   : Sets the output level of the individual GPIO pin.
 *
 *END**************************************************************************/
void GPIO_WritePinOutput(GPIO_Type* base, uint32_t pin, gpio_pin_action_t pinVal)
{
	if (pinVal == gpioPinSet)
		GPIO_PSOR_REG(base) |= GPIO_OFFSET(pin);  /* Set pin output to high level.*/
	else
		GPIO_PSOR_REG(base) &= ~(GPIO_OFFSET(pin));  /* Set pin output to low level.*/
}

void GPIO_TogglePinOutput(GPIO_Type* base, uint32_t pin, gpio_pin_action_t pinVal)
{
	if (pinVal == gpioPinSet)
		GPIO_PTOR_REG(base) |= GPIO_OFFSET(pin);  /* Set pin output to high level.*/
	else
		GPIO_PTOR_REG(base) &= ~(GPIO_OFFSET(pin));  /* Set pin output to low level.*/
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
