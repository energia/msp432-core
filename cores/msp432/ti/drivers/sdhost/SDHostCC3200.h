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
 *  @file       SDHostCC3200.h
 *
 *  @brief      SDHost driver implementation for the CC3200.
 *
 *  The SDHost header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/sdhost/SDHostCC3200.h>
 *  @endcode
 *
 *  This SDHost driver implementation is designed to operate on a CC3200
 *  SD Host controller using interrupts.
 *
 *  # Operation #
 *
 *  The SDHost driver is a driver designed to hook into FatFs. It implements a
 *  set of functions that FatFs needs to call to perform basic block data
 *  transfers.
 *
 *  The only functions that can be called by the application are the
 *  standard driver framework functions (_open, _close, etc...).
 *
 *  Once the driver has been opened, the application may used the FatFs APIs or
 *  the standard C runtime file I/O calls (fopen, fclose, etc...). Once the
 *  driver has been closed, ensure the application does NOT make any file I/O
 *  calls.

 *  ## Opening the driver #
 *
 *  @code
 *  SDHostCC3200_Handle      handle;
 *  SDHostCC3200_Params      params;
 *
 *  SDHostCC3200_Params_init(&params);
 *  handle = SDHost_open(someSDHost_configIndexValue, driveNum, &params);
 *  if (!handle) {
 *      System_printf("SDHostCC3200 did not open");
 *  }
 *  @endcode
 *
 *  # Implementation #
 *
 *  The SDHost driver interface module is joined (at link time) to a NULL
 *  terminated array of SDHost_Config data structures named *SDHost_config*.
 *  *SDHost_config* is implemented in the application with each entry being an
 *  instance of a SDHost peripheral. Each entry in *SDHost_config* contains a:
 *  - (void *) data object that contains internal driver data structures
 *  - (void *) hardware attributes which are to be configured by the application
 *
 *  # Instrumentation #
 *
 *  The SDHost driver interface produces log statements if
 *  instrumentation is enabled.
 *
 *  Diagnostics Mask | Log details |
 *  ---------------- | ----------- |
 *  Diags_USER1      | basic operations performed |
 *  Diags_USER2      | detailed operations performed |
 *  ============================================================================
 */

#ifndef ti_drivers_sdhost_SDHostCC3200__include
#define ti_drivers_sdhost_SDHostCC3200__include

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <ti/drivers/ports/HwiP.h>
#include <ti/drivers/ports/SemaphoreP.h>
#include <ti/drivers/Power.h>
#include <ti/mw/fatfs/ff.h>
#include <ti/mw/fatfs/diskio.h>

/**
 *  @defgroup SDHostCC3200_CONTROL SDHostCC3200_control command and status codes
 *  @{
 */

/*!
 * Common SDHostCC3200_control command code reservation offset.
 * SDHostCC3200 driver implementations should offset command codes with
 * SDHostCC3200_CMD_RESERVED growing positively
 *
 * Example implementation specific command codes:
 * @code
 * #define SDHostCC3200XYZ_CMD_COMMAND0    SDHostCC3200_CMD_RESERVED + 0
 * #define SDHostCC3200XYZ_CMD_COMMAND1    SDHostCC3200_CMD_RESERVED + 1
 * @endcode
 */
#define SDHostCC3200_CMD_RESERVED                (32)

/*!
 * Common SDHostCC3200_control status code reservation offset.
 * SDHostCC3200 driver implementations should offset status codes with
 * SDHostCC3200_STATUS_RESERVED growing negatively.
 *
 * Example implementation specific status codes:
 * @code
 * #define SDHostCC3200XYZ_STATUS_ERROR0   (SDHostCC3200_STATUS_RESERVED - 0)
 * #define SDHostCC3200XYZ_STATUS_ERROR1   (SDHostCC3200_STATUS_RESERVED - 1)
 * #define SDHostCC3200XYZ_STATUS_ERROR2   (SDHostCC3200_STATUS_RESERVED - 2)
 * @endcode
 */
#define SDHostCC3200_STATUS_RESERVED             (-32)

/**
 *  @defgroup SDSPI_STATUS Status Codes
 *  SDSPI_STATUS_* macros are general status codes returned by SDSPI_control()
 *  @{
 *  @ingroup SDSPI_CONTROL
 */

/*!
 * @brief   Successful status code returned by SDHostCC3200_control().
 *
 * SDSPI_control() returns SDSPI_STATUS_SUCCESS if the control code was executed
 * successfully.
 *
 * Note: This status code is also used internally in the driver to indicate that
 * a command sent to the SD card was successful.
 */
#define SDHostCC3200_STATUS_SUCCESS              (0)

/*!
 * @brief   Generic error status code returned by SDHostCC3200_control().
 *
 * SDHostCC3200_control() returns SDHostCC3200_STATUS_ERROR if the control code
 *  was not executed successfully.
 *
 * Note: This status code is also used internally in the driver to indicate
 * if a command sent to the SD card succeeded.
 */
#define SDHostCC3200_STATUS_ERROR                (-1)

/*!
 * @brief   An error status code returned by SDHostCC3200_control() for
 * undefined command codes.
 *
 * SDHostCC3200_control() returns SDHostCC3200_STATUS_UNDEFINEDCMD if the
 * control code is not recognized by the driver implementation.
 */
#define SDHostCC3200_STATUS_UNDEFINEDCMD         (-2)
/** @}*/

/**
 *  @defgroup SDHostCC3200_CMD Command Codes
 *  SDHostCC3200_CMD_* macros are general command codes for
 *  SDHostCC3200_control(). Not all SDHostCC3200 driver implementations support
 *  these command codes.
 *  @{
 *  @ingroup SDHostCC3200_CONTROL
 */

/* Add SDHostCC3200_CMD_<commands> here */

/** @}*/

/** @}*/

/*!
 *  @brief  SD Card type inserted
 */
typedef enum SDHostCC3200_CardType {
    SDHostCC3200_NOCARD = 0, /*!< Unrecognized Card */
    SDHostCC3200_MMC = 1,    /*!< Multi-media Memory Card (MMC) */
    SDHostCC3200_SDSC = 2,   /*!< Standard SDCard (SDSC) */
    SDHostCC3200_SDHC = 3    /*!< High Capacity SDCard (SDHC) */
} SDHostCC3200_CardType;

/*!
 *  @brief  SDHostCC3200 Hardware attributes
 *
 *  The SDHostCC3200 configuration structure is passed to the SDHostCC3200
 *  driver implementation with hardware specifics regarding GPIO Pins and Ports
 *  to be used.
 *
 *  The SDHostCC3200 driver uses this information to:
 *  - Configure and reconfigure specific ports/pins to initialize the SD Card
 *    for SD mode
 *  - Identify which GPIO port and pin is used for the SDHost clock, data and
 *    command lines
 *
 *  These fields are used by driverlib APIs and therefore must be populated by
 *  driverlib macro definitions. For CC32xxWare these definitions are found in:
 *      - inc/hw_memmap.h
 *      - inc/hw_ints.h
 *      - driverlib/gpio.h
 *      - driverlib/pin.h
 *      - driverlib/prcm.h
 *      - driverlib/sdhost.h
 *
 *  @struct SDHostCC3200_HWAttrs
 *  An example configuration structure could look as the following:
 *  @code
 *  const SDHostCC3200_HWAttrs sdhostCC3200HWattrs[] = {
 *      {
 *          .clkRate = 8000000,
 *          .intPriority = ~0,
 *          .baseAddr = SDHOST_BASE,
 *          .dataPin = PIN_06,
 *          .dataPinMode = PIN_MODE_8,
 *          .cmdPin = PIN_08,
 *          .cmdPinMode = PIN_MODE_8,
 *          .clkPin = PIN_07,
 *          .clkPinMode = PIN_MODE_8,
 *      }
 *  };
 *  @endcode
 */
typedef struct SDHostCC3200_HWAttrs {
    /*!< SD interface clock rate */
    uint_fast32_t clkRate;

    /*!< Internal SDHost ISR command/transfer priorty */
    int_fast32_t intPriority;

    /*!< SDHost Peripheral base address */
    uint_fast32_t baseAddr;

    /*!< SD Host Data pin */
    int_least8_t dataPin;

    /*!< Pin Mode to use for SD Host Data line */
    int_least8_t dataPinMode;

    /*!< SD Host CMD pin */
    int_least8_t cmdPin;

    /*!< Pin Mode to use for SD Host cmd line */
    int_least8_t cmdPinMode;

    /*!< SD Host CLK pin */
    int_least8_t clkPin;

    /*!< Pin Mode to use for SD Host CLK line */
    int_least8_t clkPinMode;
} SDHostCC3200_HWAttrs;

/*!
 *  @brief  SDHostCC3200 Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct SDHostCC3200_Object {
    /* Disk status */
    DSTATUS                    diskState;
    /* FATFS data object */
    FATFS                      filesystem;
    /* Drive number (1 Supported) */
    uint_fast32_t              driveNumber;
    /* Number of sectors on the device */
    uint_fast32_t              numSec;
    /* Relative Card Address */
    uint_fast32_t              rca;
    /* SD Card command state */
    volatile int_fast32_t      stat;
    /* SDCard Card Command Class(CCC) */
    SDHostCC3200_CardType      cardType;
    /*!
     *  Semaphore to suspend thread execution when waiting for an SD Commands
     *  or data transfers to complete.
     */
    SemaphoreP_Handle          sdSem;
    /*!
     *  SD Card interrupt handle.
     */
    HwiP_Handle                hwiHandle;
    /* LPDS wake-up notify object */
    Power_NotifyObj            postNotify;
    /* Determined from base address */
    unsigned int               powerMgrId;
} SDHostCC3200_Object;

/*!
 *  @brief      A handle that is returned from a SDHost_open() call.
 */
typedef struct SDHostCC3200_Config      *SDHostCC3200_Handle;

/*!
 *  @brief  SDHost Global configuration
 *
 *  The SDHostCC3200_Config structure contains a set of pointers used
 *  to characterize the SDHost driver implementation.
 *
 *  This structure needs to be defined before calling SDHost_init() and it must
 *  not be changed thereafter.
 *
 *  @sa     SDHost_init()
 */
typedef struct SDHostCC3200_Config {
    /*! Pointer to a driver specific data object */
    void             *object;

    /*! Pointer to a driver specific hardware attributes structure */
    void const       *hwAttrs;
} SDHostCC3200_Config;

/*!
 *  @brief SDHostParameters
 *
 *  SDSPI Parameters are used to with the SDSPI_open() call. Default values for
 *  these parameters are set using SDSPI_Params_init().
 *
 *  @sa         SDHostCC3200_Params_init()
 */

 /* SDHostCC3200 functions */
typedef struct SDHostCC3200_Params {
    uintptr_t custom;  /*!< Custom argument used by driver implementation */
} SDHostCC3200_Params;

/*!
 *  Function to initialize any driver parameters.
 *  Note: No custom parameters are currently supported.
 */
void SDHostCC3200_Params_init(SDHostCC3200_Params *params);

/*!
 *  Function to unmount the FatFs filesystem and unregister the SDHostCC3200
 *  disk I/O functions from SYS/BIOS' FatFS module.
 *
 *  @param  handle      SDHostCC3200_Handle returned by SDHost_open()
 */
void SDHostCC3200_close(SDHostCC3200_Handle handle);

/*!
 *  Function to initialize the SDHost module
 */
void SDHostCC3200_init(SDHostCC3200_Handle handle);

/*!
 *  @pre    Function assumes that the handle is not NULL
 *  Note: No control codes are currently supported.
 */
int_fast8_t SDHostCC3200_control(SDHostCC3200_Handle handle, uint_fast32_t cmd,
    uintptr_t arg);

/*!
 *  Function to mount the FatFs filesystem and register the SDHostCC3200 disk
 *  I/O functions with SYS/BIOS' FatFS module.
 *
 *  This function also configures some basic Pin/Clock settings needed for the
 *  software chip select with the SDCard.
 *
 *  @param  index       Logical peripheral number indexed into the HWAttrs
 *                      table.
 *  @param  drv         Drive Number
 *  @param  params      SDHost parameters
 */
SDHostCC3200_Handle SDHostCC3200_open(uint_least8_t index, uint_least8_t drv,
    SDHostCC3200_Params *params);

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_sdhost_SDHostCC3200__include */
