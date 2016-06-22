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

#include <ccm_vf6xx.h>

/* ARM Cortex-A5 clock, core clock */
uint32_t ccmCoreClock = 0;
/* Platform bus clock and Cortex-M4 core clock */
uint32_t ccmPlatformBusClk = 0;
/* IPS bus clock */
uint32_t ccmIpgBusClk = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : CCM_GetPllFreq
 * Description   : Get PLL frequency
 *
 *END**************************************************************************/
uint32_t CCM_GetPllFreq(uint32_t pfdSel, uint32_t pllPfd, uint32_t pllClk)
{
	uint32_t frac = 0;

	switch (pfdSel) {
	case CCM_CCSR_PLL_PFD_CLK_SEL_MAIN:
		return pllClk;
	case CCM_CCSR_PLL_PFD_CLK_SEL_PFD1:
		frac = (pllPfd & ANADIG_PLL_PFD1_FRAC_MASK) >> ANADIG_PLL_PFD1_FRAC_SHIFT;
		break;
	case CCM_CCSR_PLL_PFD_CLK_SEL_PFD2:
		frac = (pllPfd & ANADIG_PLL_PFD2_FRAC_MASK) >> ANADIG_PLL_PFD2_FRAC_SHIFT;
		break;
	case CCM_CCSR_PLL_PFD_CLK_SEL_PFD3:
		frac = (pllPfd & ANADIG_PLL_PFD3_FRAC_MASK) >> ANADIG_PLL_PFD3_FRAC_SHIFT;
		break;
	case CCM_CCSR_PLL_PFD_CLK_SEL_PFD4:
		frac = (pllPfd & ANADIG_PLL_PFD4_FRAC_MASK) >> ANADIG_PLL_PFD4_FRAC_SHIFT;
		break;
	}

	return pllClk / frac * 18;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CCM_GetClocks
 * Description   : Get core clocks
 *
 *END**************************************************************************/
void CCM_GetClocks()
{
	uint32_t pllPfdSel;

	switch (CCM_CCSR & CCM_CCSR_SYS_CLK_SEL_MASK) {
	case CCM_CCSR_SYS_CLK_SEL_FAST:
		ccmCoreClock = FXOSC_CLK_24M;
		break;
	case CCM_CCSR_SYS_CLK_SEL_SLOW:
		ccmCoreClock = FXOSC_CLK_32K;
		break;
	case CCM_CCSR_SYS_CLK_SEL_PLL2_PFD:

		pllPfdSel = (CCM_CCSR & CCM_CCSR_PLL2_PFD_CLK_SEL_MASK) >> CCM_CCSR_PLL2_PFD_CLK_SEL_SHIFT;
		ccmCoreClock = CCM_GetPllFreq(pllPfdSel, ANADIG_PLL2_PFD, PLL2_MAIN_CLK);
		break;
	case CCM_CCSR_SYS_CLK_SEL_PLL2:
		ccmCoreClock = PLL2_MAIN_CLK;
		break;
	case CCM_CCSR_SYS_CLK_SEL_PLL1_PFD:
		pllPfdSel = (CCM_CCSR & CCM_CCSR_PLL1_PFD_CLK_SEL_MASK) >> CCM_CCSR_PLL1_PFD_CLK_SEL_SHIFT;

		ccmCoreClock = CCM_GetPllFreq(pllPfdSel, ANADIG_PLL1_PFD, PLL1_MAIN_CLK);
		break;
	case CCM_CCSR_SYS_CLK_SEL_PLL3:
		ccmCoreClock = PLL3_MAIN_CLK;
		break;
	}

	ccmCoreClock /= ((CCM_CACRR & CCM_CACRR_ARM_CLK_DIV_MASK) + 1);
	ccmPlatformBusClk = ccmCoreClock /
					(((CCM_CACRR & CCM_CACRR_BUS_CLK_DIV_MASK) >> CCM_CACRR_BUS_CLK_DIV_SHIFT) + 1);
	ccmIpgBusClk = ccmPlatformBusClk /
					(((CCM_CACRR & CCM_CACRR_IPG_CLK_DIV_MASK) >> CCM_CACRR_IPG_CLK_DIV_SHIFT) + 1);

	return;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
