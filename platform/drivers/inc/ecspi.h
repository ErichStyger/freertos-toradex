/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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

#ifndef __ECSPI_H__
#define __ECSPI_H__

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "device_imx.h"

/*!
 * @addtogroup ecspi_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief Channel select.
 */
enum _ecspi_channel_select {
    ecspiSelectChannel0 = 0U,                   /*!< Selecte Channel 0. Chip Select 0 (SS0) will be asserted.*/
    ecspiSelectChannel1 = 1U,                   /*!< Selecte Channel 1. Chip Select 1 (SS1) will be asserted.*/
    ecspiSelectChannel2 = 2U,                   /*!< Selecte Channel 2. Chip Select 2 (SS2) will be asserted.*/
    ecspiSelectChannel3 = 3U                    /*!< Selecte Channel 3. Chip Select 3 (SS3) will be asserted.*/
};

/*!
 * @brief Channel mode.
 */
enum _ecspi_master_slave_mode {
    ecspiSlaveMode = 0U,                        /*!< Set Slave Mode.*/
    ecspiMasterMode = 1U                        /*!< Set Master Mode.*/
};

/*!
 * @brief Clock phase.
 */
enum _ecspi_clock_phase {
    ecspiClockPhaseFirstEdge = 0U,              /*!< Data is captured on the leading edge of the SCK and
                                                   changed on the following edge.*/
    ecspiClockPhaseSecondEdge = 1U              /*!< Data is changed on the leading edge of the SCK and
                                                   captured on the following edge.*/
};

/*!
 * @brief Clock polarity.
 */
enum _ecspi_clock_polarity {
    ecspiClockPolarityActiveHigh = 0U,          /*!< Active-high ECSPI clock (idles low)*/
    ecspiClockPolarityActiveLow = 1U            /*!< Active-low ECSPI clock (idles high)*/
};

/*!
 * @brief SS signal polarity.
 */
enum _ecspi_ss_polarity {
    ecspiSSPolarityActiveLow = 0U,              /*!< Active-low, ECSPI SS signal*/
    ecspiSSPolarityActiveHigh = 1U              /*!< Active-high, ECSPI SS signal */
};

/*!
 * @brief Inactive state of data line.
 */
enum _ecspi_dataline_inactivestate {
    ecspiDataLineStayHigh = 0U,                 /*!< Data line inactive state stay high */
    ecspiDataLineStayLow = 1U                   /*!< Data line inactive state stay low */
};

/*!
 * @brief Inactive state of SCLK.
 */
enum _ecspi_sclk_inactivestate {
    ecspiSclkStayLow = 0U,                      /*!< SCLK inactive state stay low */
    ecspiSclkStayHigh = 1U                      /*!< SCLK line inactive state stay high */
};

/*!
 * @brief sample period counter clock source.
 */
enum _ecspi_sampleperiod_clocksource {
    ecspiSclk = 0U,                             /*!< SCLK */
    ecspiLowFreq32K = 1U                        /*!< Low-Frequency Reference Clock (32.768 KHz) */
};

/*!
 * @brief DMA Source definition.
 */
enum _ecspi_dma_source {
    ecspiDmaTxfifoEmpty = 7U,                   /*!< TXFIFO Empty DMA Request*/
    ecspiDmaRxfifoRequest = 23U,                /*!< RXFIFO DMA Request */
    ecspiDmaRxfifoTail = 31U,                   /*!< RXFIFO TAIL DMA Request */
};

/*!
 * @brief RXFIFO and TXFIFO threshold.
 */
enum _ecspi_fifothreshold {
    ecspiTxfifoThreshold = 0U,                 /*!< Defines the FIFO threshold that triggers a TX DMA/INT request */
    ecspiRxfifoThreshold = 16U                 /*!< defines the FIFO threshold that triggers a RX DMA/INT request. */
};

/*!
 * @brief Status flag.
 */
enum _ecspi_status_flag {
    ecspiFlagTxfifoEmpty          = 1U << 0,   /*!< TXFIFO Empty Flag */
    ecspiFlagTxfifoDataRequest    = 1U << 1,   /*!< TXFIFO Data Request Flag */
    ecspiFlagTxfifoFull           = 1U << 2,   /*!< TXFIFO Full Flag */
    ecspiFlagRxfifoReady          = 1U << 3,   /*!< RXFIFO Ready Flag */
    ecspiFlagRxfifoDataRequest    = 1U << 4,   /*!< RXFIFO Data Request Flag */
    ecspiFlagRxfifoFull           = 1U << 5,   /*!< RXFIFO Full Flag */
    ecspiFlagRxfifoOverflow       = 1U << 6,   /*!< RXFIFO Overflow Flag */
    ecspiFlagTxfifoTc             = 1U << 7    /*!< TXFIFO Transform Completed Flag */
};

/*!
 * @brief Data Ready Control.
 */
enum _ecspi_data_ready {
    ecspiRdyNoCare = 0U,                       /*!< The SPI_RDY signal is a don't care */
    ecspiRdyFallEdgeTrig = 1U,                 /*!< Burst will be triggered by the falling edge of the SPI_RDY signal (edge-triggered) */
    ecspiRdyLowLevelTrig = 2U,                 /*!< Burst will be triggered by a low level of the SPI_RDY signal (level-triggered) */
    ecspiRdyReserved = 3U,                     /*!< Reserved */
};

/*!
 * @brief Init structure.
 */
typedef struct EcspiInit
{
    uint32_t clockRate;                        /*!< Specifies ECSPII module clock freq. */
    uint32_t baudRate;                         /*!< Specifies desired ECSPI baud rate. */
    uint32_t channelSelect;                    /*!< Specifies the channel select */
    uint32_t mode;                             /*!< Specifies the mode */
    bool ecspiAutoStart;                       /*!< Specifies the start mode */
    uint32_t burstLength;                      /*!< Specifies the length of a burst to be transferred */
    uint32_t clockPhase;                       /*!< Specifies the clock phase */
    uint32_t clockPolarity;                    /*!< Specifies the clock polarity */
} ecspi_init_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name ECSPI Initialization and Configuration functions
 * @{
 */

 /*!
 * @brief Initializes the ECSPI module.
 *
 * @param base: ECSPI base pointer.
 * @param initStruct: pointer to a ecspi_init_t structure.
 */
void ECSPI_Init(ECSPI_Type* base, ecspi_init_t* initStruct);

 /*!
 * @brief Enables the specified ECSPI module.
 *
 * @param base ECSPI base pointer.
 */
static inline void ECSPI_Enable(ECSPI_Type* base)
{
    /* Enable the ECSPI */
    ECSPI_CONREG_REG(base) |= ECSPI_CONREG_EN_MASK;
}

 /*!
 * @brief Disable the specified ECSPI module.
 *
 * @param base ECSPI base pointer.
 */
static inline void ECSPI_Disable(ECSPI_Type* base)
{
    /* Enable the ECSPI */
    ECSPI_CONREG_REG(base) &= ~ECSPI_CONREG_EN_MASK;
}

/*!
 * @brief Insert the number of wait states to be inserted in data transfers.
 *
 * @param base ECSPI base pointer.
 * @param number the number of wait states.
 */
static inline void ECSPI_InsertWaitState(ECSPI_Type* base, uint32_t number)
{
    /* Configure the number of wait states inserted */
    ECSPI_PERIODREG_REG(base) = (ECSPI_PERIODREG_REG(base) & (~ECSPI_PERIODREG_SAMPLE_PERIOD_MASK)) |
                                ECSPI_PERIODREG_SAMPLE_PERIOD(number);
}

/*!
 * @brief Set the clock source for the sample period counter.
 *
 * @param base ECSPI base pointer.
 * @param source the clock source (see _ecspi_sampleperiod_clocksource).
 */
void ECSPI_SetSampClockSource(ECSPI_Type* base, uint32_t source);

/*!
 * @brief Set the ECSPI clocks inserte between the chip select's active edge
 *        and the first ECSPI clock edge
 *
 * @param base ECSPI base pointer.
 * @param delay the number of wait states.
 */
static inline void ECSPI_SetDelay(ECSPI_Type* base, uint32_t delay)
{
    /* Set the number of clocks inserte */
    ECSPI_PERIODREG_REG(base) = (ECSPI_PERIODREG_REG(base) & (~ECSPI_PERIODREG_CSD_CTL_MASK)) |
                                ECSPI_PERIODREG_CSD_CTL(delay);
}

/*!
 * @brief Set the inactive state of SCLK.
 *
 * @param base ECSPI base pointer.
 * @param channel ECSPI channel select (see _ecspi_channel_select).
 * @param state SCLK inactive state (see _ecspi_sclk_inactivestate).
 */
static inline void ECSPI_SetSCLKInactiveState(ECSPI_Type* base, uint32_t channel, uint32_t state)
{
    /* Configure the inactive state of SCLK */
    ECSPI_CONFIGREG_REG(base) = (ECSPI_CONFIGREG_REG(base) & (~ECSPI_CONFIGREG_SCLK_CTL(1 << channel))) |
                                ECSPI_CONFIGREG_SCLK_CTL((state & 1) << channel);
}

/*!
 * @brief Set the inactive state of data line.
 *
 * @param base ECSPI base pointer.
 * @param channel ECSPI channel select (see _ecspi_channel_select).
 * @param state Data line inactive state (see _ecspi_dataline_inactivestate).
 */
static inline void ECSPI_SetDataInactiveState(ECSPI_Type* base, uint32_t channel, uint32_t state)
{
    /* Set the inactive state of Data Line */
    ECSPI_CONFIGREG_REG(base) = (ECSPI_CONFIGREG_REG(base) & (~ECSPI_CONFIGREG_DATA_CTL(1 << channel))) |
                                ECSPI_CONFIGREG_DATA_CTL((state & 1) << channel);
}

/*!
 * @brief Trigger a burst.
 *
 * @param base ECSPI base pointer.
 */
static inline void ECSPI_StartBurst(ECSPI_Type* base)
{
    /* start a burst */
    ECSPI_CONREG_REG(base) |= ECSPI_CONREG_XCH_MASK;
}

/*!
 * @brief Set the burst length.
 *
 * @param base ECSPI base pointer.
 * @param length the value of burst length.
 */
static inline void ECSPI_SetBurstLength(ECSPI_Type* base, uint32_t length)
{
    /* Set the burst length according to length */
    ECSPI_CONREG_REG(base) = (ECSPI_CONREG_REG(base) & (~ECSPI_CONREG_BURST_LENGTH_MASK)) |
                            ECSPI_CONREG_BURST_LENGTH(length);
}

/*!
 * @brief Set ECSPI SS Wave Form.
 *
 * @param base ECSPI base pointer.
 * @param channel ECSPI channel selected (see _ecspi_channel_select).
 * @param ssMultiBurst For master mode, set true for multiple burst and false for one burst.
 *        For slave mode, set true to complete burst by SS signal edges and false to complete
 *        burst by number of bits received.
 */
static inline void ECSPI_SetSSMultipleBurst(ECSPI_Type* base, uint32_t channel, bool ssMultiBurst)
{
    /* Set the SS wave form. */
    ECSPI_CONFIGREG_REG(base) = (ECSPI_CONFIGREG_REG(base) & (~ECSPI_CONFIGREG_SS_CTL(1 << channel))) |
                                ECSPI_CONFIGREG_SS_CTL(ssMultiBurst << channel);
}

/*!
 * @brief Set ECSPI SS Polarity.
 *
 * @param base ECSPI base pointer.
 * @param channel ECSPI channel selected (see _ecspi_channel_select).
 * @param polarity set SS signal active logic (see _ecspi_ss_polarity).
 */
static inline void ECSPI_SetSSPolarity(ECSPI_Type* base, uint32_t channel, uint32_t polarity)
{
    /* Set the SS polarity. */
    ECSPI_CONFIGREG_REG(base) = (ECSPI_CONFIGREG_REG(base) & (~ECSPI_CONFIGREG_SS_POL(1 << channel))) |
                                ECSPI_CONFIGREG_SS_POL(polarity << channel);
}

/*!
 * @brief Set the Data Ready Control.
 *
 * @param base ECSPI base pointer.
 * @param spidataready ECSPI data ready control (see _ecspi_data_ready).
 */
static inline void ECSPI_SetSPIDataReady(ECSPI_Type* base, uint32_t spidataready)
{
    /* Set the Data Ready Control */
    ECSPI_CONREG_REG(base) = (ECSPI_CONREG_REG(base) & (~ECSPI_CONREG_DRCTL_MASK)) |
                             ECSPI_CONREG_DRCTL(spidataready);
}

/*!
 * @brief Calculated the ECSPI baud rate in bits per second.
 *        The calculated baud rate must not exceed the desired baud rate.
 *
 * @param base ECSPI base pointer.
 * @param sourceClockInHz ECSPI Clock(SCLK) (in Hz).
 * @param bitsPerSec the value of Baud Rate.
 * @return The calculated baud rate in bits-per-second, the nearest possible
 *         baud rate without exceeding the desired baud rate.
 */
uint32_t ECSPI_SetBaudRate(ECSPI_Type* base, uint32_t sourceClockInHz, uint32_t bitsPerSec);

/*@}*/

/*!
 * @name Data transfers functions
 * @{
 */

/*!
  * @brief Transmits a data to TXFIFO.
  *
  * @param base ECSPI base pointer.
  * @param data Data to be transmitted.
  */
static inline void ECSPI_SendData(ECSPI_Type* base, uint32_t data)
{
    /* Write data to Transmit Data Register */
    ECSPI_TXDATA_REG(base) = data;
}

/*!
  * @brief Receives a data from RXFIFO.
  * @param base ECSPI base pointer.
  * @return The value of received data.
  */
static inline uint32_t ECSPI_ReceiveData(ECSPI_Type* base)
{
    /* Read data from Receive Data Register */
    return ECSPI_RXDATA_REG(base);
}

/*!
  * @brief Read the number of words in the RXFIFO.
  *
  * @param base ECSPI base pointer.
  * @return The number of words in the RXFIFO.
  */
static inline uint32_t ECSPI_GetRxfifoCounter(ECSPI_Type* base)
{
    /* Get the number of words in the RXFIFO */
    return ((ECSPI_TESTREG_REG(base) & ECSPI_TESTREG_RXCNT_MASK) >> ECSPI_TESTREG_RXCNT_SHIFT);
}

/*!
  * @brief Read the number of words in the TXFIFO.
  *
  * @param base ECSPI base pointer.
  * @return The number of words in the TXFIFO.
  */
static inline uint32_t ECSPI_GetTxfifoCounter(ECSPI_Type* base)
{
    /* Get the number of words in the RXFIFO */
    return ((ECSPI_TESTREG_REG(base) & ECSPI_TESTREG_TXCNT_MASK) >> ECSPI_TESTREG_TXCNT_SHIFT);
}

/*@}*/

/*!
 * @name DMA management functions
 * @{
 */

/*!
 * @brief Enable or disable the specified DMA Source.
 *
 * @param base ECSPI base pointer.
 * @param source specifies DMA source (see _ecspi_dma_source).
 * @param enable True or False.
 */
void ECSPPI_SetDMACmd(ECSPI_Type* base, uint32_t source, bool enable);

/*!
  * @brief Set the burst length of a DMA operation.
  *
  * @param base ECSPI base pointer.
  * @param length specifies the burst length of a DMA operation.
  */
static inline void ECSPI_SetDMABurstLength(ECSPI_Type* base, uint32_t length)
{
    /* Configure the burst length of a DMA operation */
    ECSPI_DMAREG_REG(base) = (ECSPI_DMAREG_REG(base) & (~ECSPI_DMAREG_RX_DMA_LENGTH_MASK)) |
                             ECSPI_DMAREG_RX_DMA_LENGTH(length);
}

/*!
  * @brief Set the RXFIFO or TXFIFO threshold.
  *
  * @param base ECSPI base pointer.
  * @param fifo Data transfer fifo (see _ecspi_fifothreshold)
  * @param threshold Threshold value.
  */
void ECSPI_SetFIFOThreshold(ECSPI_Type* base, uint32_t fifo, uint32_t threshold);

/*@}*/

/*!
 * @name Interrupts and flags management functions
 * @{
 */

/*!
 * @brief Enable or disable the specified ECSPI interrupts.
 *
 * @param base ECSPI base pointer.
 * @param flags ECSPI status flag mask (see _ecspi_status_flag for bit definition).
 * @param enable Interrupt enable (true: enable, false: disable).
 */
void ECSPI_SetIntCmd(ECSPI_Type* base, uint32_t flags, bool enable);

/*!
 * @brief Checks whether the specified ECSPI flag is set or not.
 *
 * @param base ECSPI base pointer.
 * @param flags ECSPI status flag mask (see _ecspi_status_flag for bit definition).
 * @return ECSPI status, each bit represents one status flag.
 */
static inline uint32_t ECSPI_GetStatusFlag(ECSPI_Type* base, uint32_t flags)
{
    /* return the vale of ECSPI status */
    return ECSPI_STATREG_REG(base) & flags;
}

/*!
 * @brief Clear one or more ECSPI status flag.
 *
 * @param base ECSPI base pointer.
 * @param flags ECSPI status flag mask (see _ecspi_status_flag for bit definition).
 */
static inline void ECSPI_ClearStatusFlag(ECSPI_Type* base, uint32_t flags)
{
    /* Write 1 to the status bit */
    ECSPI_STATREG_REG(base) = flags;
}

/*@}*/

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /*__ECSPI_H__*/

/*******************************************************************************
 * EOF
 ******************************************************************************/
