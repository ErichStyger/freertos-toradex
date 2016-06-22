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

#ifndef __GPIO_VF6XX_H__
#define __GPIO_VF6XX_H__

#include <stdint.h>
#include <stdbool.h>
#include "device_imx.h"

/*!
 * @addtogroup gpio_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief GPIO direction definition */
typedef enum _gpio_pin_direction {
	gpioDigitalInput  = 0U,   /*!< Set current pin as digital input*/
	gpioDigitalOutput = 1U    /*!< Set current pin as digital output*/
} gpio_pin_direction_t;

/*! @brief GPIO pin(bit) value definition */
typedef enum _gpio_pin_action {
	gpioPinClear = 0U,
	gpioPinSet = 1U
} gpio_pin_action_t;

/*! @brief  GPIO Init structure definition */
typedef struct GpioInit
{
	uint32_t               pin;              /*!< Specifies the pin number. */
	gpio_pin_direction_t   direction;        /*!< Specifies the pin direction. */
} gpio_init_t;

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

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name GPIO Initialization and Configuration functions
 * @{
 */

/*!
 * @brief Initializes the GPIO peripheral according to the specified
 *        parameters in the initStruct.
 *
 * @param base GPIO base pointer (GPIO1, GPIO2, GPIO3, etc.).
 * @param initStruct pointer to a gpio_init_t structure that
 *         contains the configuration information.
 */
void GPIO_Init(GPIO_Type* base, gpio_init_t* initStruct);

void GPIO_TogglePinOutput(GPIO_Type* base, uint32_t pin, gpio_pin_action_t pinVal);

/*@}*/

/*!
 * @name GPIO Read and Write Functions
 * @{
 */

 /*!
 * @brief Reads the current input value of the pin when pin's direction is configured as input.
 *
 * @param base GPIO base pointer (GPIO1, GPIO2, GPIO3, etc.).
 * @param pin GPIO port pin number.
 * @return GPIO pin input value.
 *         - 0: Pin logic level is 0, or is not configured for use by digital function.
 *         - 1: Pin logic level is 1.
 */
static inline uint8_t GPIO_ReadPinInput(GPIO_Type* base, uint32_t pin)
{
	return (uint8_t)((GPIO_PDIR_REG(base) >> (pin & 0x1f)) & 1U);
}

/*!
 * @brief Reads the current input value of a specific GPIO port when port's direction are all configured as input.
 *        This function  gets all 32-pin input as a 32-bit integer.
 *
 * @param base GPIO base pointer(GPIO1, GPIO2, GPIO3, etc.)
 * @return GPIO port input data. Each bit represents one pin. For each bit:
 *         - 0: Pin logic level is 0, or is not configured for use by digital function.
 *         - 1: Pin logic level is 1.
 *         - LSB: pin 0
 *         - MSB: pin 31
 */
static inline uint32_t GPIO_ReadPortInput(GPIO_Type *base)
{
	return GPIO_PDIR_REG(base);
}

/*!
 * @brief Reads the current pin output.
 *
 * @param base GPIO base pointer(GPIO1, GPIO2, GPIO3, etc.)
 * @param pin GPIO port pin number.
 * @return current pin output value, 0 - Low logic, 1 - High logic.
 */
static inline uint8_t GPIO_ReadPinOutput(GPIO_Type* base, uint32_t pin)
{
	return (uint8_t)((GPIO_PDOR_REG(base) >> (pin & 0x1f)) & 0x1U);
}

/*!
 * @brief Reads out all pin output status of the current port.
 *        This function  operates all 32 port pins.
 *
 * @param base GPIO base pointer(GPIO1, GPIO2, GPIO3, etc.)
 * @return current port output status. Each bit represents one pin. For each bit:
 *        - 0: corresponding pin is outputting logic level 0
 *        - 1: corresponding pin is outputting logic level 1
 *        - LSB: pin 0
 *        - MSB: pin 31
 */
static inline uint32_t GPIO_ReadPortOutput(GPIO_Type* base)
{
	return GPIO_PDOR_REG(base);
}

/*!
 * @brief Sets the output level of the individual GPIO pin to logic 1 or 0.
 *
 * @param base GPIO base pointer(GPIO1, GPIO2, GPIO3, etc.)
 * @param pin GPIO port pin number.
 * @param pinVal pin output value, one of the follow.
 *        -gpioPinClear: logic 0;
 *        -gpioPinSet: logic 1.
 */
void GPIO_WritePinOutput(GPIO_Type* base, uint32_t pin, gpio_pin_action_t pinVal);

/*!
 * @brief Sets the output of the GPIO port pins to a specific logic value.
 *         This function  operates all 32 port pins.
 *
 * @param base GPIO base pointer(GPIO1, GPIO2, GPIO3, etc.)
 * @param portVal data to configure the GPIO output. Each bit represents one pin. For each bit:
 *        - 0: set logic level 0 to pin
 *        - 1: set logic level 1 to pin
 *        - LSB: pin 0
 *        - MSB: pin 31
 */
static inline void GPIO_WritePortOutput(GPIO_Type* base, uint32_t portVal)
{
	GPIO_PDOR_REG(base) = portVal;
}

/*@}*/

/*!
 * @name GPIO Read Pad Status Functions
 * @{
 */

 /*!
 * @brief Reads the current GPIO pin pad status.
 *
 * @param base GPIO base pointer (GPIO1, GPIO2, GPIO3, etc.).
 * @param pin GPIO port pin number.
 * @return GPIO pin pad status value.
 *         - 0: Pin pad status logic level is 0.
 *         - 1: Pin pad status logic level is 1.
 */
static inline uint8_t GPIO_ReadPadStatus(GPIO_Type* base, uint32_t pin)
{
	return (uint8_t)((GPIO_PSOR_REG(base) >> (pin & 0x1f)) & 1U);
}

/*@}*/

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* __GPIO_VF6XX_H__*/

/*******************************************************************************
 * EOF
 ******************************************************************************/
