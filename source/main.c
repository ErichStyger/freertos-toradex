/*
 * Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
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

/**
 * This is template for main module created by New Kinetis SDK 2.x Project Wizard. Enjoy!
 **/

#include <string.h>

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_mpu.h"
#include "fsl_flexcan.h"
#include "fsl_dspi.h"
#include "fsl_gpio.h"
#include "usb_host_config.h"
#include "usb.h"
#include "usb_host.h"
#include "usb_host_hci.h"
#include "usb_host_devices.h"

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"


#if ((defined USB_HOST_CONFIG_KHCI) && (USB_HOST_CONFIG_KHCI))
#define CONTROLLER_ID kUSB_ControllerKhci0
#endif
#if ((defined USB_HOST_CONFIG_EHCI) && (USB_HOST_CONFIG_EHCI))
#define CONTROLLER_ID kUSB_ControllerEhci0
#endif

/* Task priorities. */
#define hello_task_PRIORITY (configMAX_PRIORITIES - 1)
#define USB_HOST_INTERRUPT_PRIORITY (5U)

/* Task handles */
TaskHandle_t can_task_handle;
TaskHandle_t usb_task_handle;
TaskHandle_t spi_task_handle;
TaskHandle_t hello_task_handle;

struct test_data {
	uint16_t vid;
	uint16_t pid;
	uint8_t  enumerated;
	uint8_t  can_test_status;
	uint8_t  ts_test_status;
} test_status;

/*!
 * @brief Task responsible for printing of "Hello world." message.
 */
static void hello_task(void *pvParameters) {
	for (;;) {
		PRINTF("Hello world.\r\n");
		/* Add your code here */
		vTaskDelay(5000);
	}
}

usb_host_handle g_HostHandle;

static usb_status_t USB_HostEvent(usb_device_handle deviceHandle,
		usb_host_configuration_handle configurationHandle,
		uint32_t eventCode)
{
	usb_status_t status = kStatus_USB_Success;
	uint32_t infoValue;

	switch (eventCode)
	{
	case kUSB_HostEventAttach:
		usb_echo("device attached.\r\n");
		break;

	case kUSB_HostEventNotSupported:
		usb_echo("device not supported.\r\n");
		break;

	case kUSB_HostEventEnumerationDone:
		usb_echo("device enumerated.\r\n");
		test_status.enumerated = 1;
		USB_HostHelperGetPeripheralInformation(deviceHandle, kUSB_HostGetDevicePID, &infoValue);
		usb_echo("PID = 0x%x ", infoValue);
		test_status.pid = infoValue;
		USB_HostHelperGetPeripheralInformation(deviceHandle, kUSB_HostGetDeviceVID, &infoValue);
		usb_echo("VID = 0x%x \r\n", infoValue);
		test_status.vid = infoValue;
		break;

	case kUSB_HostEventDetach:
		usb_echo("device removed.\r\n");
		USB_HostCloseDeviceInterface(deviceHandle, NULL);
		break;

	default:
		break;
	}
	return status;
}

/*!
 * @brief USB isr function.
 */
#if ((defined USB_HOST_CONFIG_KHCI) && (USB_HOST_CONFIG_KHCI))
void USB0_IRQHandler(void)
{
	USB_HostKhciIsrFunction(g_HostHandle);
}
#endif /* USB_HOST_CONFIG_KHCI */
#if ((defined USB_HOST_CONFIG_EHCI) && (USB_HOST_CONFIG_EHCI))
void USBHS_IRQHandler(void)
{
	USB_HostEhciIsrFunction(g_HostHandle);
}
#endif /* USB_HOST_CONFIG_EHCI */

static void USB_HostApplicationInit(void)
{
	usb_status_t status = kStatus_USB_Success;

#if ((defined USB_HOST_CONFIG_KHCI) && (USB_HOST_CONFIG_KHCI))
#if (defined(FSL_FEATURE_SOC_SCG_COUNT) && (FSL_FEATURE_SOC_SCG_COUNT > 0U))
	CLOCK_EnableUsbfs0Clock(kCLOCK_IpSrcFircAsync, CLOCK_GetFreq(kCLOCK_ScgFircAsyncDiv1Clk));
#else
	CLOCK_EnableUsbfs0Clock(kCLOCK_UsbSrcPll0, CLOCK_GetFreq(kCLOCK_PllFllSelClk));
#endif
#endif /* USB_HOST_CONFIG_KHCI */
#if ((defined USB_HOST_CONFIG_EHCI) && (USB_HOST_CONFIG_EHCI))
	IRQn_Type usbHsIrqs[] = USBHS_IRQS;
	usbIrq = usbHsIrqs[CONTROLLER_ID - kUSB_ControllerEhci0];
	CLOCK_EnableUsbhs0Clock(kCLOCK_UsbSrcPll0, CLOCK_GetFreq(kCLOCK_PllFllSelClk));
	USB_EhciPhyInit(CONTROLLER_ID, BOARD_XTAL0_CLK_HZ);
#endif /* USB_HOST_CONFIG_EHCI */
#if ((defined FSL_FEATURE_SOC_MPU_COUNT) && (FSL_FEATURE_SOC_MPU_COUNT))
	MPU_Enable(MPU, 0);
#endif /* FSL_FEATURE_SOC_MPU_COUNT */

	status = USB_HostInit(CONTROLLER_ID, &g_HostHandle, USB_HostEvent);
	if (status != kStatus_USB_Success)
	{
		usb_echo("host init error\r\n");
		return;
	}
	NVIC_SetPriority(USB0_IRQn, USB_HOST_INTERRUPT_PRIORITY);
	NVIC_EnableIRQ(USB0_IRQn);

	usb_echo("host init done\r\n");
}

static void USB_HostTask(void *param)
{
	while (1)
	{
#if ((defined USB_HOST_CONFIG_KHCI) && (USB_HOST_CONFIG_KHCI))
		USB_HostKhciTaskFunction(param);
#endif /* USB_HOST_CONFIG_KHCI */
#if ((defined USB_HOST_CONFIG_EHCI) && (USB_HOST_CONFIG_EHCI))
		USB_HostEhciTaskFunction(param);
#endif /* USB_HOST_CONFIG_EHCI */
	}
}


#define RX_MESSAGE_BUFFER_NUM (9)
#define TX_MESSAGE_BUFFER_NUM (8)
#define RX_MESSAGE_BUFFER_NUM1 (7)
#define TX_MESSAGE_BUFFER_NUM1 (6)

flexcan_handle_t flexcanHandle[2];
uint32_t txIdentifier[2];
uint32_t rxIdentifier[2];
volatile bool txComplete[2] = {false, false};
volatile bool rxComplete[2] = {false, false};

typedef struct _callback_message_t
{
	status_t async_status;
	SemaphoreHandle_t sem;
} callback_message_t;

static void flexcan_callback0(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData)
{
	callback_message_t * cb = (callback_message_t*) userData;
	BaseType_t reschedule = pdFALSE;

	switch (status)
	{
	case kStatus_FLEXCAN_RxIdle:
		if (RX_MESSAGE_BUFFER_NUM == result)
		{
			xSemaphoreGiveFromISR(cb->sem, &reschedule);
		}
		break;

	case kStatus_FLEXCAN_TxIdle:
		if (TX_MESSAGE_BUFFER_NUM == result)
		{
		}
		break;

	default:
		break;
	}
	portYIELD_FROM_ISR(reschedule);
}

static void flexcan_callback1(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData)
{
	callback_message_t * cb = (callback_message_t*) userData;
	BaseType_t reschedule = pdFALSE;

	switch (status)
	{
	case kStatus_FLEXCAN_RxIdle:
		if (RX_MESSAGE_BUFFER_NUM1 == result)
		{
			xSemaphoreGiveFromISR(cb->sem, &reschedule);
		}
		break;

	case kStatus_FLEXCAN_TxIdle:
		if (TX_MESSAGE_BUFFER_NUM1 == result)
		{
		}
		break;

	default:
		break;
	}
	portYIELD_FROM_ISR(reschedule);
}

void CAN_Init()
{
	flexcan_config_t flexcanConfig;
	flexcan_rx_mb_config_t mbConfig;

	txIdentifier[0] = 0x321;
	rxIdentifier[0] = 0x123;
	txIdentifier[1] = 0x123;
	rxIdentifier[1] = 0x321;

	/* Get FlexCAN module default Configuration. */
	/*
	 * flexcanConfig.clkSrc = kFLEXCAN_ClkSrcOsc;
	 * flexcanConfig.baudRate = 125000U;
	 * flexcanConfig.maxMbNum = 16;
	 * flexcanConfig.enableLoopBack = false;
	 * flexcanConfig.enableSelfWakeup = false;
	 * flexcanConfig.enableIndividMask = false;
	 * flexcanConfig.enableDoze = false;
	 */
	FLEXCAN_GetDefaultConfig(&flexcanConfig);

	/* Init FlexCAN module. */
	flexcanConfig.clkSrc = kFLEXCAN_ClkSrcPeri;
	FLEXCAN_Init(CAN0, &flexcanConfig, CLOCK_GetFreq(kCLOCK_BusClk));

	/* Create FlexCAN handle structure and set call back function. */
	FLEXCAN_TransferCreateHandle(CAN0, &flexcanHandle[0], flexcan_callback0, flexcanHandle[0].userData);

	/* Set Rx Masking mechanism. */
	FLEXCAN_SetRxMbGlobalMask(CAN0, FLEXCAN_RX_MB_STD_MASK(rxIdentifier[0], 0, 0));

	/* Setup Rx Message Buffer. */
	mbConfig.format = kFLEXCAN_FrameFormatStandard;
	mbConfig.type = kFLEXCAN_FrameTypeData;
	mbConfig.id = FLEXCAN_ID_STD(rxIdentifier[0]);
	FLEXCAN_SetRxMbConfig(CAN0, RX_MESSAGE_BUFFER_NUM, &mbConfig, true);

	/* Setup Tx Message Buffer. */
	FLEXCAN_SetTxMbConfig(CAN0, TX_MESSAGE_BUFFER_NUM, true);

	/* Get FlexCAN module default Configuration. */
	/*
	 * flexcanConfig.clkSrc = kFLEXCAN_ClkSrcOsc;
	 * flexcanConfig.baudRate = 125000U;
	 * flexcanConfig.maxMbNum = 16;
	 * flexcanConfig.enableLoopBack = false;
	 * flexcanConfig.enableSelfWakeup = false;
	 * flexcanConfig.enableIndividMask = false;
	 * flexcanConfig.enableDoze = false;
	 */
	FLEXCAN_GetDefaultConfig(&flexcanConfig);

	/* Init FlexCAN module. */
	flexcanConfig.clkSrc = kFLEXCAN_ClkSrcPeri;
	FLEXCAN_Init(CAN1, &flexcanConfig, CLOCK_GetFreq(kCLOCK_BusClk));

	/* Create FlexCAN handle structure and set call back function. */
	FLEXCAN_TransferCreateHandle(CAN1, &flexcanHandle[1], flexcan_callback1, flexcanHandle[1].userData);

	/* Set Rx Masking mechanism. */
	FLEXCAN_SetRxMbGlobalMask(CAN1, FLEXCAN_RX_MB_STD_MASK(rxIdentifier[1], 0, 0));

	/* Setup Rx Message Buffer. */
	mbConfig.format = kFLEXCAN_FrameFormatStandard;
	mbConfig.type = kFLEXCAN_FrameTypeData;
	mbConfig.id = FLEXCAN_ID_STD(rxIdentifier[1]);
	FLEXCAN_SetRxMbConfig(CAN1, RX_MESSAGE_BUFFER_NUM1, &mbConfig, true);

	/* Setup Tx Message Buffer. */
	FLEXCAN_SetTxMbConfig(CAN1, TX_MESSAGE_BUFFER_NUM1, true);
	PRINTF("CAN init done \r\n");
}


static void can_test_task(void *pvParameters) {
	flexcan_frame_t txFrame, rxFrame;
	flexcan_mb_transfer_t txXfer, rxXfer;
	callback_message_t cb_msg[2];

	cb_msg[0].sem = xSemaphoreCreateBinary();
	cb_msg[1].sem = xSemaphoreCreateBinary();
	flexcanHandle[0].userData = (void *) &cb_msg[0];
	flexcanHandle[1].userData = (void *) &cb_msg[1];
	CAN_Init();

	vTaskSuspend(NULL);
	rxXfer.frame = &rxFrame;
	rxXfer.mbIdx = RX_MESSAGE_BUFFER_NUM1;
	FLEXCAN_TransferReceiveNonBlocking(CAN1, &flexcanHandle[1], &rxXfer);

	txFrame.format = kFLEXCAN_FrameFormatStandard;
	txFrame.type = kFLEXCAN_FrameTypeData;
	txFrame.id = FLEXCAN_ID_STD(0x321);
	txFrame.length = 8;
	txFrame.dataWord0 = CAN_WORD0_DATA_BYTE_0(0x11) | CAN_WORD0_DATA_BYTE_1(0x22) | CAN_WORD0_DATA_BYTE_2(0x33) |
			CAN_WORD0_DATA_BYTE_3(0x44);
	txFrame.dataWord1 = CAN_WORD1_DATA_BYTE_4(0x55) | CAN_WORD1_DATA_BYTE_5(0x66) | CAN_WORD1_DATA_BYTE_6(0x77) |
			CAN_WORD1_DATA_BYTE_7(0x88);
	txXfer.frame = &txFrame;
	txXfer.mbIdx = TX_MESSAGE_BUFFER_NUM;

	PRINTF("\r\nCAN0->CAN1 ");

	FLEXCAN_TransferSendNonBlocking(CAN0, &flexcanHandle[0], &txXfer);
	if (xSemaphoreTake(cb_msg[1].sem, 1000u) == pdFALSE)
	{
		PRINTF("FAIL!\r\n");
		FLEXCAN_TransferAbortSend(CAN0, &flexcanHandle[0], txXfer.mbIdx);
		FLEXCAN_TransferAbortReceive(CAN1, &flexcanHandle[1], rxXfer.mbIdx);
		test_status.can_test_status = 0x0F;
	}
	else
	{
		PRINTF("SUCCESS!\r\n");
		test_status.can_test_status = 0x05;
	}

	rxXfer.frame = &rxFrame;
	rxXfer.mbIdx = RX_MESSAGE_BUFFER_NUM;
	FLEXCAN_TransferReceiveNonBlocking(CAN0, &flexcanHandle[0], &rxXfer);

	txFrame.format = kFLEXCAN_FrameFormatStandard;
	txFrame.type = kFLEXCAN_FrameTypeData;
	txFrame.id = FLEXCAN_ID_STD(0x123);
	txFrame.length = 8;
	txFrame.dataWord0 = CAN_WORD0_DATA_BYTE_0(0x11) | CAN_WORD0_DATA_BYTE_1(0x22) | CAN_WORD0_DATA_BYTE_2(0x33) |
			CAN_WORD0_DATA_BYTE_3(0x44);
	txFrame.dataWord1 = CAN_WORD1_DATA_BYTE_4(0x55) | CAN_WORD1_DATA_BYTE_5(0x66) | CAN_WORD1_DATA_BYTE_6(0x77) |
			CAN_WORD1_DATA_BYTE_7(0x88);
	txXfer.frame = &txFrame;
	txXfer.mbIdx = TX_MESSAGE_BUFFER_NUM1;

	PRINTF("CAN1->CAN0 ");

	FLEXCAN_TransferSendNonBlocking(CAN1, &flexcanHandle[1], &txXfer);
	if (xSemaphoreTake(cb_msg[0].sem, 1000u ) == pdFALSE)
	{
		PRINTF("FAIL!\r\n");
		FLEXCAN_TransferAbortSend(CAN1, &flexcanHandle[0], txXfer.mbIdx);
		FLEXCAN_TransferAbortReceive(CAN0, &flexcanHandle[1], rxXfer.mbIdx);
		test_status.can_test_status |= 0xF0;
	}
	else
	{
		PRINTF("SUCCESS!\r\n");
		test_status.can_test_status |= 0x50;
	}

	vSemaphoreDelete(cb_msg[0].sem);
	vSemaphoreDelete(cb_msg[1].sem);

	vTaskResume(spi_task_handle);
	vTaskDelete(NULL);
}

#define TRANSFER_SIZE 32U
dspi_slave_handle_t spi_handle;
uint8_t slaveRxData[TRANSFER_SIZE] = {0U};
uint8_t slaveTxData[TRANSFER_SIZE] = {0U};

void SPI_callback(SPI_Type *base, dspi_slave_handle_t *handle, status_t status, void *userData)
{
	callback_message_t * cb = (callback_message_t*) userData;
	BaseType_t reschedule = pdFALSE;

    if (status == kStatus_Success)
    {
    	xSemaphoreGiveFromISR(cb->sem, &reschedule);
    }

    if (status == kStatus_DSPI_Error)
    {
        __NOP();
    }
    portYIELD_FROM_ISR(reschedule);
}

void SPI_init() {
	dspi_slave_config_t slaveConfig;
	/* Slave config */
	slaveConfig.whichCtar = kDSPI_Ctar0;
	slaveConfig.ctarConfig.bitsPerFrame = 8;
	slaveConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh;
	slaveConfig.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge;
	slaveConfig.enableContinuousSCK = kDSPI_MsbFirst;
	slaveConfig.enableRxFifoOverWrite = false;
	slaveConfig.enableModifiedTimingFormat = false;
	slaveConfig.samplePoint = kDSPI_SckToSin0Clock;

	DSPI_SlaveInit(SPI2, &slaveConfig);
	DSPI_SlaveTransferCreateHandle(SPI2, &spi_handle, SPI_callback, spi_handle.userData);

	/* Set dspi slave interrupt priority higher. */
	NVIC_SetPriority(SPI2_IRQn, 5U);
	PRINTF("SPI init done \r\n");

}

static void spi_task(void *pvParameters) {
	callback_message_t cb_msg;
	dspi_transfer_t slaveXfer;

	cb_msg.sem = xSemaphoreCreateBinary();
	spi_handle.userData = &cb_msg;
	SPI_init();
	GPIO_ClearPinsOutput(GPIOA, 1u << 29u); // INT2 active

	while(1){
	    slaveXfer.txData = slaveTxData;
	    slaveXfer.rxData = slaveRxData;
	    slaveXfer.dataSize = 16;
	    slaveXfer.configFlags = kDSPI_SlaveCtar0;
	    //Wait for instructions from SoC
	    DSPI_SlaveTransferNonBlocking(SPI2, &spi_handle, &slaveXfer);
	    PRINTF("Waiting for SPI transfer\r\n");
	    xSemaphoreTake(cb_msg.sem, portMAX_DELAY);
	    for (int i = 0; i< 16; i++)
	    	PRINTF("Transfer received Rx[%d]= 0x%X\r\n", i, slaveRxData[i]);
		switch (slaveRxData[0]){
		case 0x01: //echo test echo remaining 15 characters
			slaveTxData[0] = 42;
			memcpy(&slaveTxData[1], &slaveRxData[1], 15);
			break;
		case 0x02: // forward USB state
			slaveTxData[0] = 0x02;
			slaveTxData[1] = test_status.enumerated;
			slaveTxData[2] = test_status.vid & 0xFF;
			slaveTxData[3] = (test_status.vid >> 8) & 0xFF;
			slaveTxData[4] = test_status.pid & 0xFF;
			slaveTxData[5] = (test_status.pid >> 8) & 0xFF;
			memset(&slaveTxData[6], 0, 10);
			break;
		case 0x03: // execute CAN test;
			vTaskResume(can_task_handle);
			vTaskSuspend(NULL); // wait for can_test to finish
			slaveTxData[0] = 0x03;
			slaveTxData[1] = test_status.can_test_status;
			memset(&slaveTxData[2], 0, 15);
			break;
		case 0x04: // execute touch screen test;
//			vTaskResume(ts_task_handle);
//			vTaskSuspend(NULL); // wait for ts_test to finish
			slaveTxData[0] = 0x04;
			slaveTxData[1] = 0x01;//test_status.ts_test_status;
			memset(&slaveTxData[2], 0, 14);
			break;
		default:
			memset(slaveTxData, 0x33, 16);
			;
		}
		//Prepare out transfer and signal on the INT pin
		DSPI_SlaveTransferNonBlocking(SPI2, &spi_handle, &slaveXfer);
		GPIO_ClearPinsOutput(GPIOA, 1u << 16u); // INT1 active
		xSemaphoreTake(cb_msg.sem, portMAX_DELAY);
		GPIO_SetPinsOutput(GPIOA, 1u << 16u); // INT1 idle
	}
}

/*!
 * @brief Application entry point.
 */
int main(void) {

	/* Init board hardware. */
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();
	PRINTF("Hello!\r\n");
	USB_HostApplicationInit();

	/* Add your code here */


	/* Create RTOS task */
	if (xTaskCreate(can_test_task, "can test task", 2000L / sizeof(portSTACK_TYPE), NULL, 4, &can_task_handle) != pdPASS)
	{
		usb_echo("create host task error\r\n");
	}
	if (xTaskCreate(USB_HostTask, "usb host task", 2000L / sizeof(portSTACK_TYPE), g_HostHandle, 4, &usb_task_handle) != pdPASS)
	{
		usb_echo("create host task error\r\n");
	}
	if(xTaskCreate(spi_task, "SPI_task", 2000L / sizeof(portSTACK_TYPE), NULL, 4, &spi_task_handle) != pdPASS)
	{
		usb_echo("create hello task error\r\n");
	}
	if(xTaskCreate(hello_task, "Hello_task", configMINIMAL_STACK_SIZE, NULL, hello_task_PRIORITY, &hello_task_handle) != pdPASS)
	{
		usb_echo("create hello task error\r\n");
	}

	NVIC_SetPriority(CAN0_ORed_Message_buffer_IRQn, 5u);
	NVIC_SetPriority(CAN1_ORed_Message_buffer_IRQn, 5u);
	NVIC_SetPriorityGrouping( 0 );
	vTaskStartScheduler();

	for(;;) { /* Infinite loop to avoid leaving the main function */
		__asm("NOP"); /* something to use as a breakpoint stop while looping */
	}
}



