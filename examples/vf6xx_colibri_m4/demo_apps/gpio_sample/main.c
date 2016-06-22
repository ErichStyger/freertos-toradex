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

#include <FreeRTOS.h>
#include <task.h>
#include <ccm_vf6xx.h>
#include <lpuart_vf6xx.h>
#include <gpio_vf6xx.h>
#include "debug_console_vf6xx.h"

struct hello_world {
	const char *text;
	int delay;
};

static struct hello_world task_a = {
	.text = "Hello, World!\r\n",
	.delay = 700,
};

void switch_task(void *p)
{
	struct hello_world *data = (struct hello_world *)p;

	for (;;) {
		if (GPIO_ReadPinInput(GPIO(38), 38))
			PRINTF(data->text);

		vTaskDelay(data->delay);
	}
}

void led_toggle_task(void *p)
{
	for (;;) {
		GPIO_TogglePinOutput(GPIO(39), 39, gpioPinSet);

		vTaskDelay(500);
	}
}

int main(void)
{
	/* Init Clock Control and UART */
	CCM_GetClocks();
	CCM_ControlGate(ccmCcgrGateUart2, ccmClockNeededAll);

	vf6xx_DbgConsole_Init(UART2, ccmIpgBusClk, 115200);

	gpio_init_t ledIn = {.pin = 38, .direction = gpioDigitalInput};
	gpio_init_t ledOut = {.pin = 39, .direction = gpioDigitalOutput};
	GPIO_Init(GPIO(38), &ledIn);
	GPIO_Init(GPIO(39), &ledOut);

	xTaskCreate(switch_task, "Task A", configMINIMAL_STACK_SIZE,
		    &task_a, tskIDLE_PRIORITY, NULL);

	xTaskCreate(led_toggle_task, "Task B", configMINIMAL_STACK_SIZE,
		    &task_a, tskIDLE_PRIORITY, NULL);

	vTaskStartScheduler();

	while (true);
}
