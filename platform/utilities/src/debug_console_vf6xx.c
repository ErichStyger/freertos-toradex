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

#include "debug_console_vf6xx.h"
#include "lpuart_vf6xx.h"

static void UART_SendDataBlocking(void *base, const uint8_t *txBuff, uint32_t txSize);
static void UART_ReceiveDataBlocking(void *base, uint8_t *rxBuff, uint32_t rxSize);

debug_console_state_t prv_debugConsole;

void vf6xx_DbgConsole_Init(UART_Type* base, uint32_t clockRate,
                                       uint32_t baudRate)
{
	prv_debugConsole.base = base;

	/* Setup UART init structure. */
	uart_init_config_t uart_init_str = {.clockRate  = clockRate,
		                        .baudRate   = baudRate,
		                        .wordLength = uartWordLength8Bits,
		                        .parity     = uartParityDisable};
	/* UART Init operation */
	LPUART_Init(prv_debugConsole.base, &uart_init_str);
	LPUART_Enable(prv_debugConsole.base);

	prv_debugConsole.ops.Send = UART_SendDataBlocking;
	prv_debugConsole.ops.Receive = UART_ReceiveDataBlocking;

	DbgConsole_Init(&prv_debugConsole);
}

void vf6xx_DbgConsole_DeInit(void)
{
    /* UART Deinit operation */
    LPUART_Disable(prv_debugConsole.base);
}

void UART_SendDataBlocking(void *base, const uint8_t *txBuff, uint32_t txSize)
{
    while (txSize--)
    {
	LPUART_SendBlocking((UART_Type*)base, *txBuff++);
    }
}

void UART_ReceiveDataBlocking(void *base, uint8_t *rxBuff, uint32_t rxSize)
{
    while (rxSize--)
    {
	*rxBuff = LPUART_RecvBlocking((UART_Type*)base);
        rxBuff++;
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
