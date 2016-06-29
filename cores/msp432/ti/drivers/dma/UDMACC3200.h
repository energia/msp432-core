/*
 * Copyright (c) 2016, Texas Instruments Incorporated
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
 *  @file       UDMACC3200.h
 *
 *  @brief      uDMA driver implementation for CC3200.
 *
 *  This driver is intended for use only by TI-RTOS drivers that use the uDMA
 *  peripheral (e.g., SPI and I2S).  This driver is mainly used for Power
 *  management of the UDMA peripheral.
 *
 *  The application should only define the memory for the control table and
 *  set up the UDMACC3200_HWAttrs and UDMACC3200_Config structures.
 *
 *  The UDMACC3200 header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/dma/UDMACC3200.h>
 *  @endcode
 *
 *  ============================================================================
 */

#ifndef ti_drivers_dma_UDMACC3200__include
#define ti_drivers_dma_UDMACC3200__include

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <ti/drivers/ports/HwiP.h>

/*!
 *  @brief      UDMA error function pointer
 */
typedef void (*UDMACC3200_ErrorFxn)(uintptr_t arg);

/*!
 *  @brief      UDMACC3200 Hardware attributes
 *
 *  This structure contains the base address of the uDMA control
 *  table, and uDMA error interrupt attributes.
 *
 *  The control table is used by the uDMA controller to store channel
 *  control structures.  The control table can be located anywhere in
 *  system memory, but must be contiguous and aligned on a 1024-byte boundary.
 *
 *  dmaErrorFxn is the uDMA peripheral's error interrupt handler.
 *
 *  intPriority is priority of the uDMA peripheral's error interrupt, as
 *  defined by the underlying OS.  It is passed unmodified to the
 *  underlying OS's interrupt handler creation code, so you need to
 *  refer to the OS documentation for usage.  For example, for
 *  SYS/BIOS applications, refer to the ti.sysbios.family.arm.m3.Hwi
 *  documentation for SYS/BIOS usage of interrupt priorities.  If the
 *  driver uses the ti.drivers.ports interface instead of making OS
 *  calls directly, then the HwiP port handles the interrupt priority
 *  in an OS specific way.  In the case of the SYS/BIOS port,
 *  intPriority is passed unmodified to Hwi_create().
 *
 *  A sample structure is shown below:
 *  @code
 *
 *  #include <driverlib/udma.h>
 *
 *  #if defined(__TI_COMPILER_VERSION__)
 *  #pragma DATA_ALIGN(dmaControlTable, 1024)
 *  #elif defined(__IAR_SYSTEMS_ICC__)
 *  #pragma data_alignment=1024
 *  #elif defined(__GNUC__)
 *  __attribute__ ((aligned (1024)))
 *  #endif
 *
 *  static tDMAControlTable dmaControlTable[64];
 *
 *  #include <ti/drivers/dma/UDMACC3200.h>
 *
 *  UDMACC3200_Object udmaCC3200Object;
 *
 *  const UDMACC3200_HWAttrs udmaCC3200HWAttrs = {
 *          .controlBaseAddr = (void *)dmaControlTable,
 *          .dmaErrorFxn = UDMACC3200_errorFxn,
 *          .intNum = INT_UDMAERR,
 *          .intPriority = (~0)
 *  };
 *  @endcode
 *
 */
typedef struct UDMACC3200_HWAttrs {
    void           *controlBaseAddr; /*!< uDMA control registers base address */
    UDMACC3200_ErrorFxn dmaErrorFxn; /*!< uDMA error interrupt handler */
    uint8_t         intNum;          /*!< uDMA error interrupt number */
    uint8_t         intPriority;     /*!< uDMA error interrupt priority. */
} UDMACC3200_HWAttrs;

/*!
 *  @brief      UDMACC3200 Global configuration
 *
 *  The UDMACC3200_Config structure contains pointers used by the UDMACC3200
 *  driver.
 *
 *  This structure needs to be defined before calling UDMACC3200_init() and
 *  it must not be changed thereafter.
 */
typedef struct UDMACC3200_Config {
    void              *object;            /*!< Pointer to UDMACC3200 object */
    void const        *hwAttrs;           /*!< Pointer to hardware attributes */
} UDMACC3200_Config;

/*!
 *  @brief      A handle that is returned from a UDMACC3200_open() call.
 */
typedef struct UDMACC3200_Config      *UDMACC3200_Handle;

/*!
 *  @brief  UDMACC3200 object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct UDMACC3200_Object {
    bool             isOpen;          /* Flag for open/close status */
    HwiP_Handle      hwiHandle;       /* DMA error Hwi */
} UDMACC3200_Object;

/*!
 *  @brief  Function to close the DMA driver.
 *
 *  This function releases Power dependency on UDMA that was previously
 *  set with a call to UDMACC3200_open(). If there is only one outstanding
 *  UDMACC3200_open() call (i.e. all but one UDMACC3200_open() calls have
 *  been matched by a corresponding call to UDMACC3200_close()), this
 *  function will disable the UDMA.
 *
 *  @pre    UDMACC3200_open() has to be called first.
 *          Calling context: Task
 *
 *  @param  handle  A UDMACC3200_Handle returned from UDMACC3200_open()
 *
 *  @return none
 *
 *  @sa     UDMACC3200_open
 */
extern void UDMACC3200_close(UDMACC3200_Handle handle);

/*!
 *  @brief  Function to initialize the CC3200 DMA driver
 *
 *  The function will set the isOpen flag to false, and should be called prior
 *  to opening the DMA driver.
 *
 *  @return none
 *
 *  @sa     UDMACC3200_open()
 */
extern void UDMACC3200_init();

/*!
 *  @brief  Function to initialize the CC3200 DMA peripheral
 *
 *  UDMACC3200_open() can be called multiple times.  Each time the
 *  function is called, it will set a dependency on the peripheral and
 *  enable the clock.  The Power dependency count on the UDMA will be
 *  equal to the number of outstanding calls to UDMACC3200_open().
 *  Calling UDMACC3200_close() will decrement the Power dependency count,
 *  and the last call to UDMACC3200_close() will disable the UDMA.
 *
 *  @pre    UDMACC3200_init() has to be called first.
 *          Calling context: Task
 *
 *  @return UDMACC3200_Handle on success or NULL if an error has occurred.
 *
 *  @sa     UDMACC3200_close()
 */
extern UDMACC3200_Handle UDMACC3200_open();

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_dma_UDMACC3200__include */
