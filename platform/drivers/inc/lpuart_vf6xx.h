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

#ifndef __LPUART_VF6XX_H__
#define __LPUART_VF6XX_H__

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "device_imx.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief UART module initialize structure. */
typedef struct _uart_init_config
{
    uint32_t clockRate;     /*!< Current UART module clock freq. */
    uint32_t baudRate;      /*!< Desired UART baud rate. */
    uint32_t wordLength;    /*!< Data bits in one frame. */
    uint32_t parity;        /*!< Parity error check mode of this module. */
} uart_init_config_t;

/*!
 * @brief UART number of data bits in a character.
 */
enum _uart_word_length
{
    uartWordLength8Bits     = 0x0,
    uartWordLength9Bits     = UART_UCR1_M,
};

/*!
 * @brief UART parity mode.
 */
enum _uart_partity_mode
{
    uartParityDisable       = 0x0,
    uartParityEven          = UART_UCR1_PE,
    uartParityOdd           = UART_UCR1_PE | UART_UCR1_PT
};

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name UART Initialization and Configuration functions
 * @{
 */

/*!
 * @brief Initialize UART module with given initialize structure.
 *
 * @param base UART base pointer.
 * @param initConfig UART initialize structure(see uart_init_config_t above).
 */
void LPUART_Init(UART_Type* base, uart_init_config_t* initConfig);

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_SetParity
 * Description   : This function configures uart module parity mode.
 *
 *END**************************************************************************/
static inline void LPUART_SetParity(UART_Type* base, uint32_t parityMode)
{
	UART_C1_REG(base) = parityMode;
}

/*!
 * @brief This function is used to set the baud rate of UART Module.
 *
 * @param base UART base pointer.
 * @param clockRate UART module clock frequency.
 * @param baudRate Desired UART module baud rate.
 */
void LPUART_SetBaudRate(UART_Type* base, uint32_t clockRate, uint32_t baudRate);

/*!
 * @brief This function is used to send data.
 *
 * @param base UART base pointer.
 * @param data Data to be set through Uart module.
 */
static inline void LPUART_PutChar(UART_Type* base, uint8_t data)
{
    UART_D_REG(base) = data;
}

/*!
 * @brief This function is used to receive data.
 *
 * @param base UART base pointer.
 * @return The data received from UART module.
 */
static inline uint8_t LPUART_GetChar(UART_Type* base)
{
    return UART_D_REG(base);
}

static inline void LPUART_Enable(UART_Type* base)
{
	UART_C2_REG(base) |= (UART_UCR2_TE | UART_UCR2_RE);
}

static inline void LPUART_Disable(UART_Type* base)
{
	UART_C2_REG(base) &= ~(UART_UCR2_TE | UART_UCR2_RE);
}

static inline void LPUART_WaitRecvReady(UART_Type* base)
{
	/* Wait until the data is ready to be received. */
	while ((UART_S1_REG(base) & UART_S1_RDRF) == 0);
}

static inline uint8_t LPUART_RecvBlocking(UART_Type* base)
{
	LPUART_WaitRecvReady(base);

	return LPUART_GetChar(base);
}

static inline void LPUART_WaitSendReady(UART_Type* base)
{
	/* Wait until the data has been transferred into the shift register. */
	while ((UART_S1_REG(base) & UART_S1_TC) == 0);
}

static inline void LPUART_SendBlocking(UART_Type* base, uint8_t data)
{
	LPUART_WaitSendReady(base);
	LPUART_PutChar(base, data);
}

#ifdef __cplusplus
}
#endif

/*! @}*/

#endif /* __LPUART_VF6XX_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
