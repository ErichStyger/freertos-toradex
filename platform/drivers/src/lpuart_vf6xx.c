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

#include <lpuart_vf6xx.h>

/*******************************************************************************
 * Code
 ******************************************************************************/

/*******************************************************************************
 * Initialization and Configuration functions
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_Init
 * Description   : This function initializes the module according to uart
 *                 initialize structure.
 *
 *END**************************************************************************/
void LPUART_Init(UART_Type* base, uart_init_config_t* initConfig)
{
	assert(initConfig);

	LPUART_Disable(base);
	LPUART_SetParity(base, initConfig->parity);
	LPUART_SetBaudRate(base, initConfig->clockRate, initConfig->baudRate);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_SetBaudRate
 * Description   : This function configures uart module baud rate.
 *
 *END**************************************************************************/
void LPUART_SetBaudRate(UART_Type* base, uint32_t clockRate, uint32_t baudRate)
{
	uint32_t sbr, brfa;
	uint8_t cr4 = 0, bdh = 0;

	sbr = clockRate / (16 * baudRate);
	brfa = ((clockRate - (16 * sbr * baudRate)) * 2) / baudRate;
	bdh &= ~UART_BDH_SBR_MASK;
	bdh |= (sbr >> 8) & UART_BDH_SBR_MASK;
	cr4 &= ~UART_C4_BRFA_MASK;
	brfa &= UART_C4_BRFA_MASK;

	UART_C4_REG(base) = (cr4 | brfa);
	UART_BDH_REG(base) = bdh;
	UART_BDL_REG(base) = sbr;
}

/*******************************************************************************
 * Hardware Flow control and Modem Signal functions
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_SetRtsFlowCtrl
 * Description   : This function is used to set the enable condition of RTS
 *                 auto control.
 *
 *END**************************************************************************/
void LPUART_SetRtsFlowCtrl(UART_Type* base, bool enable)
{
	if (enable)
		UART_MODEM_REG(base) &= ~UART_FLOWCONTROL_RTS;
	else
		UART_MODEM_REG(base) |= UART_FLOWCONTROL_RTS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_SetCtsFlowCtrl
 * Description   : This function is used to set the enable condition of CTS
 *                 auto control.
 *
 *END**************************************************************************/
void LPUART_SetCtsFlowCtrl(UART_Type* base, bool enable)
{
	if (enable)
		UART_MODEM_REG(base) |= UART_FLOWCONTROL_CTS;
	else
		UART_MODEM_REG(base) &= ~UART_FLOWCONTROL_CTS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_SetFlowCtrl
 * Description   : This function is used to set the enable condition of RTS, CTS
 *                 auto control.
 *
 *END**************************************************************************/
void LPUART_SetFlowCtrl(UART_Type* base, bool enable)
{
	if (enable)
		UART_MODEM_REG(base) |= UART_FLOWCONTROL_MASK;
	else
		UART_MODEM_REG(base) &= ~UART_FLOWCONTROL_MASK;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
