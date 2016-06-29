/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ============================================================================
 *  @file       SPICC3200DMA.h
 *
 *  @brief      SPI driver implementation for a CC3200 SPI controller using the
 *              micro DMA controller.
 *
 *  The SPI header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/SPI.h>
 *  #include <ti/drivers/spi/SPICC3200DMA.h>
 *  @endcode
 *
 *  Refer to @ref SPI.h for a complete description of APIs & example of use.
 *
 *  This SPI driver implementation is designed to operate on a CC3200 SPI
 *  controller using a micro DMA controller.
 *
 *  ## SPI Chip Select #
 *  This SPI controller supports a hardware chip select pin. Refer to the
 *  device's user manual on how this hardware chip select pin behaves in regards
 *  to the SPI frame format.
 *
 *  <table>
 *  <tr>
 *  <th>Chip select type</th>
 *  <th>SPI_MASTER mode</th>
 *  <th>SPI_SLAVE mode</th>
 *  </tr>
 *  <tr>
 *  <td>Hardware chip select</td>
 *  <td>No action is needed by the application to select the peripheral.</td>
 *  <td>See the device documentation on it's chip select requirements.</td>
 *  </tr>
 *  <tr>
 *  <td>Software chip select</td>
 *  <td>The application is responsible to ensure that correct SPI slave is
 *      selected before performing a SPI_transfer().</td>
 *  <td>See the device documentation on it's chip select requirements.</td>
 *  </tr>
 *  </table>
 *
 *  ## DMA Interrupts #
 *  This driver is designed to operate with the micro DMA. The micro DMA
 *  generates IRQ on the perpheral's interrupt vector. This implementation
 *  automatically installs a DMA aware Hwi (interrupt) to service the assigned
 *  micro DMA channels.
 *
 *  ## SPI data frames #
 *  SPI data frames can be any size from 4-bits to 32-bits. If the dataSize in
 *  ::SPI_Params is greater than 8-bits, then the SPICC3200DMA driver
 *  implementation will assume that the ::SPI_Transaction txBuf and rxBuf
 *  point to an array of 16-bit uint16_t elements.  The driver will also assume
 *  the SPI_Transaction txBuf and rxBuf point to 32-bit uint32_t elements if
 *  dataSize is greater than 16-bits.
 *
 *  dataSize   | buffer element size |
 *  --------   | ------------------- |
 *  4-8 bits   | uint8_t             |
 *  9-16 bits  | uint16_t            |
 *  16-32 bits | uint32_t            |
 *
 *  ## DMA transfer size limit #
 *  The micro DMA controller only supports data transfers of up to 1024
 *  data frames. A data frame can be 4 to 32 bits in length.
 *
 *  ## DMA accessible memory #
 *  As this driver uses uDMA to transfer data/from data buffers, it is the
 *  responsibility of the application to ensure that theses buffers reside in
 *  memory that is accessible by the DMA.
 *
 *  ============================================================================
 */

#ifndef ti_drivers_spi_SPICC3200DMA__include
#define ti_drivers_spi_SPICC3200DMA__include

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drivers/ports/HwiP.h>
#include <ti/drivers/ports/SemaphoreP.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/dma/UDMACC3200.h>

/**
 *  @addtogroup SPI_STATUS
 *  SPICC3200DMA_STATUS_* macros are command codes only defined in the
 *  SPICC3200DMA.h driver implementation and need to:
 *  @code
 *  #include <ti/drivers/sdspi/SPICC3200DMA.h>
 *  @endcode
 *  @{
 */

/* Add SPICC3200DMA_STATUS_* macros here */

/** @}*/

/**
 *  @addtogroup SPI_CMD
 *  SPICC3200DMA_CMD_* macros are command codes only defined in the
 *  SPICC3200DMA.h driver implementation and need to:
 *  @code
 *  #include <ti/drivers/sdspi/SPICC3200DMA.h>
 *  @endcode
 *  @{
 */

/* Add SPICC3200DMA_CMD_* macros here */

/** @}*/

typedef unsigned long SPIBaseAddrType;
typedef unsigned long SPIDataType;

/* SPI function table pointer */
extern const SPI_FxnTable SPICC3200DMA_fxnTable;

/*!
 *  @brief
 *  SPICC3200DMA data frame size is used to determine how to configure the
 *  DMA data transfers. This field is to be only used internally.
 *
 *  SPICC3200DMA_8bit:  txBuf and rxBuf are arrays of uint8_t elements
 *  SPICC3200DMA_16bit: txBuf and rxBuf are arrays of uint16_t elements
 *  SPICC3200DMA_32bit: txBuf and rxBuf are arrays of uint32_t elements
 */
typedef enum SPICC3200DMA_FrameSize {
    SPICC3200DMA_8bit  = 0,
    SPICC3200DMA_16bit = 1,
    SPICC3200DMA_32bit = 2
} SPICC3200DMA_FrameSize;

/*!
 *  @brief  SPICC3200DMA Hardware attributes
 *
 *  These fields, with the exception of intPriority,
 *  are used by driverlib APIs and therefore must be populated by
 *  driverlib macro definitions. For CCWare these definitions are found in:
 *      - driverlib/prcm.h
 *      - driverlib/spi.h
 *      - driverlib/udma.h
 *      - inc/hw_memmap.h
 *      - inc/hw_ints.h
 *
 *  intPriority is the SPI peripheral's interrupt priority, as defined by the
 *  underlying OS.  It is passed unmodified to the underlying OS's interrupt
 *  handler creation code, so you need to refer to the OS documentation
 *  for usage.  For example, for SYS/BIOS applications, refer to the
 *  ti.sysbios.family.arm.m3.Hwi documentation for SYS/BIOS usage of
 *  interrupt priorities.  If the driver uses the ti.drivers.ports interface
 *  instead of making OS calls directly, then the HwiP port handles the
 *  interrupt priority in an OS specific way.  In the case of the SYS/BIOS
 *  port, intPriority is passed unmodified to Hwi_create().
 *
 *  A sample structure is shown below:
 *  @code
 *  #if defined(__TI_COMPILER_VERSION__)
 *  #pragma DATA_ALIGN(scratchBuf, 32)
 *  #elif defined(__IAR_SYSTEMS_ICC__)
 *  #pragma data_alignment=32
 *  #elif defined(__GNUC__)
 *  __attribute__ ((aligned (32)))
 *  #endif
 *  uint32_t scratchBuf;
 *
 *  const SPICC3200DMA_HWAttrs SPICC3200DMAHWAttrs[] = {
 *      {
 *          .baseAddr = GSPI_BASE,
 *          .intNum = INT_GSPI,
 *          .intPriority = (~0),
 *          .spiPRCM = PRCM_GSPI,
 *          .csControl = SPI_HW_CTRL_CS,
 *          .csPolarity = SPI_CS_ACTIVELOW,
 *          .pinMode = SPI_4PIN_MODE,
 *          .turboMode = SPI_TURBO_OFF,
 *          .scratchBufPtr = &scratchBuf,
 *          .defaultTxBufValue = 0,
 *          .rxChannelIndex = UDMA_CH6_GSPI_RX,
 *          .txChannelIndex = UDMA_CH7_GSPI_TX,
 *          .minDmaTransferSize = 100;
 *      },
 *      ...
 *  };
 *  @endcode
 */
typedef struct SPICC3200DMA_HWAttrs {
    /*! SPICC3200DMA Peripheral's base address */
    SPIBaseAddrType   baseAddr;

    /*! SPICC3200DMA Peripheral's interrupt vector */
    uint32_t   intNum;

    /*! SPICC3200DMA Peripheral's interrupt priority */
    uint32_t   intPriority;

    /*! SPI PRCM peripheral number */
    uint32_t   spiPRCM;

    /*! Specify if chip select line will be controlled by SW or HW */
    uint32_t   csControl;

    uint32_t   csPolarity;

    /*! Set peripheral to work in 3-pin or 4-pin mode */
    uint32_t   pinMode;

    /*! Enable or disable SPI TURBO mode */
    uint32_t   turboMode;

    /*! Address of a scratch buffer of size uint32_t */
    uint32_t  *scratchBufPtr;

    /*! Default TX value if txBuf == NULL */
    unsigned long   defaultTxBufValue;

    /*! uDMA RX channel index */
    uint32_t   rxChannelIndex;

    /*! uDMA TX channel index */
    uint32_t   txChannelIndex;

    /*! Minimum amout of data to start a uDMA transfer */
    uint32_t   minDmaTransferSize;
} SPICC3200DMA_HWAttrs;

/*!
 *  @brief  SPICC3200DMA Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct SPICC3200DMA_Object {
    HwiP_Handle            hwiHandle;
    SemaphoreP_Handle      transferComplete;

    Power_NotifyObj        notifyObj;
    unsigned int           powerMgrId;

    uint32_t               bitRate;        /*!< SPI bit rate in Hz */
    uint32_t               dataSize;       /*!< SPI data frame size in bits */
    SPI_CallbackFxn        transferCallbackFxn;
    SPI_Transaction       *transaction;

    void                 (*spiPollingFxn) (uint32_t baseAddr, void *rx,
                                   void *tx, uint8_t rxInc, uint8_t txInc,
                               size_t count);

    uint8_t                rxFifoTrigger;
    uint8_t                txFifoTrigger;
    SPI_Mode               spiMode;
    SPI_TransferMode       transferMode;
    SPI_FrameFormat        frameFormat;    /*!< SPI frame format */
    SPICC3200DMA_FrameSize frameSize;

    bool                   isOpen;

    /* UDMA */
    UDMACC3200_Handle      dmaHandle;
} SPICC3200DMA_Object, *SPICC3200DMA_Handle;

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_spi_SPICC3200DMA__include */
