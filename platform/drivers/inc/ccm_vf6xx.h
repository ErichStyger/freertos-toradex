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

#ifndef __CCM_VF6XX_H__
#define __CCM_VF6XX_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>
#include "device_imx.h"

#define PLL1_MAIN_CLK		528000000
#define PLL2_MAIN_CLK		528000000
#define PLL3_MAIN_CLK		480000000

#define FXOSC_CLK_24M		24000000
#define FXOSC_CLK_32K		32000

/* --- Variable definitions ------------------------------------------------ */
extern uint32_t ccmCoreClk;
extern uint32_t ccmPlatformBusClk;
extern uint32_t ccmIpgBusClk;

/*!
 * @brief CCM CCGR gate control
 */
enum ccm_clock_gate {
	ccmCcgrGateFlexCan0 = 0,
	ccmCg1Reserved,
	ccmCg2Reserved,
	ccmCg3Reserved,
	ccmCcgrGateDma0_mux,
	ccmCcgrGateDma1_mux,
	ccmCg6Reserved,
	ccmCcgrGateUart0,
	ccmCcgrGateUart1,
	ccmCcgrGateUart2,
	ccmCcgrGateUart3,
	ccmCg11Reserved,
	ccmCcgrGateSpi0,
	ccmCcgrGateSpi1,
	ccmCg14Reserved,
	ccmCcgrGateSai0,
	ccmCcgrGateSai1,
	ccmCcgrGateSai2,
	ccmCcgrGateSai3,
	ccmCcgrGateCrc,
	ccmCcgrGateUsbc0,
	ccmCg21Reserved,
	ccmCcgrGatePdb,
	ccmCcgrGatePit,
	ccmCcgrGateFtm0,
	ccmCcgrGateFtm1,
	ccmCg26Reserved,
	ccmCcgrGateAdc0,
	ccmCg28Reserved,
	ccmCcgrGate_Tcon0,
	ccmCcgrGateWdogA5,
	ccmCcgrGateWdogM4,
	ccmCcgrGateLptmr,
	ccmCg33Reserved,
	ccmCcgrGateRle,
	ccmCg35Reserved,
	ccmCcgrGateQspi0,
	ccmCg37Reserved,
	ccmCg38Reserved,
	ccmCg39Reserved,
	ccmCcgGateIomux,
	ccmCcgrGatePortA,
	ccmCcgrGatePortB,
	ccmCcgrGatePortC,
	ccmCcgrGatePortD,
	ccmCcgrGatePortE,
	ccmCg46Reserved,
	ccmCg47Reserved,
	ccmCcgrGateAnadig,
	ccmCg49Reserved,
	ccmCcgrGateScsm,
	ccmCg51Reserved,
	ccmCg52Reserved,
	ccmCg53Reserved,
	ccmCg54Reserved,
	ccmCg55Reserved,
	ccmCcgrGateDcu0,
	ccmCg57Reserved,
	ccmCg58Reserved,
	ccmCg59Reserved,
	ccmCg60Reserved,
	ccmCg61Reserved,
	ccmCg62Reserved,
	ccmCg63Reserved,
	ccmCcgrGateAsrc,
	ccmCcgrGateSpidf,
	ccmCcgrGateEsai,
	ccmCg67Reserved,
	ccmCg68Reserved,
	ccmCcgrGateEwm,
	ccmCcgrGateI2c0,
	ccmCcgrGateI2c1,
	ccmCg72Reserved,
	ccmCg73Reserved,
	ccmCcgrGateWkup,
	ccmCcgrGateCcm,
	ccmCcgrGateGpc,
	ccmCcgrGateVregDig,
	ccmCg78Reserved,
	ccmCcgrGateCmu,
	ccmCg80Notused,
	ccmCg81Notused,
	ccmCg82Notused,
	ccmCg83Notused,
	ccmCg84Notused,
	ccmCg85Notused,
	ccmCg86Notused,
	ccmCg87Notused,
	ccmCg88Notused,
	ccmCg89Notused,
	ccmCg90Notused,
	ccmCg91Notused,
	ccmCg92Notused,
	ccmCg93Notused,
	ccmCg94Notused,
	ccmCg95Notused,
	ccmCg96Reserved,
	ccmCcgrGateDma2Mux,
	ccmCcgrGateDma3Mux,
	ccmCg99Reserved,
	ccmCg100Reserved,
	ccmCcgrGateOtpCtrl,
	ccmCg102Reserved,
	ccmCg103Reserved,
	ccmCg104Reserved,
	ccmCcgrGateUart4,
	ccmCcgrGateUart5,
	ccmCg107Reserved,
	ccmCcgrGateSpi2,
	ccmCcgrGateSpi3,
	ccmCcgrGateDdrmc,
	ccmCg111Reserved,
	ccmCg112Reserved,
	ccmCcgrGateSdhc0,
	ccmCcgrGateDdhc1,
	ccmCg115Reserved,
	ccmCcgrGateUsbc1,
	ccmCg117Reserved,
	ccmCg118Reserved,
	ccmCg119Reserved,
	ccmCcgrGateFtm2,
	ccmCcgrGateFtm3,
	ccmCg122Reserved,
	ccmCcgrGateAdc1,
	ccmCg124Reserved,
	ccmCcgrGateTcon1,
	ccmCcgrGateSegLcd,
	ccmCg127Reserved,
	ccmCg128Reserved,
	ccmCg129Reserved,
	ccmCg130Reserved,
	ccmCg131Reserved,
	ccmCcgrGateQspi1,
	ccmCg133Reserved,
	ccmCg134Reserved,
	ccmCcgrGateVadc,
	ccmCcgrGateVdec,
	ccmCcgrGateViu3,
	ccmCg138Reserved,
	ccmCg139Reserved,
	ccmCcgrGateDac0,
	ccmCcgrGateDac1,
	ccmCg142Reserved,
	ccmCg143Notused,
	ccmCcgrGateEth01588,
	ccmCcgrGateEth11588,
	ccmCg146Reserved,
	ccmCg147Reserved,
	ccmCg148FlexCan1,
	ccmCg149Reserved,
	ccmCg150Reserved,
	ccmCg151Reserved,
	ccmCcgrGateDcu1,
	ccmCg153Reserved,
	ccmCg154Reserved,
	ccmCg155Reserved,
	ccmCg156Reserved,
	ccmCg157Reserved,
	ccmCg158Reserved,
	ccmCg159Reserved,
	ccmCcgrGateNfc,
	ccmCg161Reserved,
	ccmCg162Reserved,
	ccmCg163Reserved,
	ccmCg164Reserved,
	ccmCg165Reserved,
	ccmCcgrGateI2c2,
	ccmCcgrGateI2c3,
	ccmCcgrGateEthL2,
	ccmCg169Reserved,
	ccmCg170Reserved,
	ccmCg171RRserved,
	ccmCg172Reserved,
	ccmCg173Reserved,
	ccmCg174Reserved,
	ccmCg175Reserved,
	ccmCg176Reserved,
	ccmCg177Reserved,
	ccmCg178Reserved,
	ccmCg179Reserved,
	ccmCg180Reserved,
	ccmCg181Reserved,
	ccmCg182Reserved,
	ccmCg183Reserved,
	ccmCg184Reserved,
	ccmCg185Reserved,
	ccmCg186Reserved,
	ccmCg187Reserved,
	ccmCg188Reserved,
	ccmCg189Reserved,
	ccmCg190Reserved,
	ccmCg191Reserved
};

uint32_t ccm_ccgr_offset[] = {
	(uint32_t)(&CCM_CCGR0),
	(uint32_t)(&CCM_CCGR1),
	(uint32_t)(&CCM_CCGR2),
	(uint32_t)(&CCM_CCGR3),
	(uint32_t)(&CCM_CCGR4),
	(uint32_t)(&CCM_CCGR5),
	(uint32_t)(&CCM_CCGR6),
	(uint32_t)(&CCM_CCGR7),
	(uint32_t)(&CCM_CCGR8),
	(uint32_t)(&CCM_CCGR9),
	(uint32_t)(&CCM_CCGR10),
	(uint32_t)(&CCM_CCGR11),
};
/*!
 * @brief CCM gate control value
 */
enum _ccm_gate_value {
	ccmClockNotNeeded        = 0x0U,      /*!< Clock always disabled.*/
	ccmClockNeededRun        = 0x1111U,   /*!< Clock enabled when CPU is running.*/
	ccmClockNeededRunWait    = 0x2222U,   /*!< Clock enabled when CPU is running or in WAIT mode.*/
	ccmClockNeededAll        = 0x3333U    /*!< Clock always enabled.*/
};

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

void CCM_GetClocks(void);
uint32_t CCM_GetPllFreq(uint32_t pfdSel, uint32_t pllPfd, uint32_t pllClk);

static inline void CCM_ControlGate(uint32_t ccmGate, uint32_t control)
{
	*((volatile uint32_t *)((uint32_t)(ccm_ccgr_offset[(ccmGate >> 4)]))) |= \
							control << CCM_CGR_MASK(ccmGate);
}


#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __CCM_VF6XX_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
