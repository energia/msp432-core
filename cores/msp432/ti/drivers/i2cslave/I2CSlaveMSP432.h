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
 *  @file       I2CSlaveMSP432.h
 *
 *  @brief      I2CSlave driver implementation for the EUSCI controller on
 *  MSP432
 *
 *  This I2CSlave driver implementation is designed to operate on a EUCSI
 *  controller in I2CSlave mode.  The I2CSlaveMSP432 header file should be
 *  included in an application as follows:
 *  @code
 *  #include <ti/drivers/I2CSlave.h>
 *  #include <ti/drivers/I2CSlaveMSP432.h>
 *  @endcode
 *
 *  ============================================================================
 */

#ifndef ti_drivers_i2cslave_I2CSlaveMSP432__include
#define ti_drivers_i2cslave_I2CSlaveMSP432__include

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include <ti/drivers/I2CSlave.h>
#include <ti/drivers/ports/HwiP.h>
#include <ti/drivers/ports/SemaphoreP.h>
#include <ti/drivers/Power.h>

/* I2C function table pointer */
extern const I2CSlave_FxnTable I2CSlaveMSP432_fxnTable;

/*!
 *  @brief  I2CSlaveMSP432 Hardware attributes
 *  These fields are used by driverlib APIs and therefore must be populated by
 *  driverlib macro definitions. For MSP430Ware these definitions are found in:
 *      - i2cslave.h
 *
 *  A sample structure is shown below:
 *  @code
 *  const I2CSlaveMSP432_HWAttrs i2cslaveMSP432HWAttrs[] = {
 *      {
 *          .baseAddr = EUSCI_B0_BASE,
 *          .intNum = INT_EUSCIB0,
 *          .intPriority = ~0
 *
 *      }
 *  };
 *  @endcode
 */
typedef struct I2CSlaveMSP432_HWAttrs {
    uint32_t baseAddr;       /*! EUSCI_B_I2C Peripheral's base address */
    uint32_t intNum;         /*! EUSCI_B_I2C Peripheral's interrupt vector */
    uint32_t intPriority;    /*! EUSCI_B_I2C Peripheral's interrupt priority */
    uint32_t slaveAddress;   /*! EUSCI_B_I2C Peripheral's slave address */
} I2CSlaveMSP432_HWAttrs;

/*!
 *  @brief  I2CSlaveMSP432 Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct I2CSlaveMSP432_Object {
    SemaphoreP_Handle       mutex;               /* Grants exclusive access */
     /* Notify finished I2C transfer */
    SemaphoreP_Handle       transferComplete;

    HwiP_Handle             hwiHandle;

    I2CSlave_CallbackFxn    transferCallbackFxn; /* Callback function pointer */
    /* Internal inc. writeBuf index */
    const uint8_t           *writeBufferIdx;
    /* Internal inc. readBuf index */
    uint8_t                 *readBufferIdx;
    size_t                  countIdx;            /* Internal dec. writeCounter */

    volatile I2CSlave_Mode  mode;                /* Stores the I2CSlave state */
    I2CSlave_TransferMode   transferMode;        /* Blocking or Callback mode */

    /* To determine if the I2CSlave is open */
    bool                    isOpen;
    bool                    transferInProgress;
} I2CSlaveMSP432_Object;

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_i2cslave_I2CMSP432__include */
