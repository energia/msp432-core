/*
 * Copyright (c) 2015, Texas Instruments Incorporated
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
 *  @file       SDSPIEUSCIA.h
 *
 *  @brief      SDSPI driver implementation for a USCI SPI peripheral used
 *              with the SDSPI driver.
 *
 *  The SDSPI header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/SDSPI.h>
 *  #include <ti/drivers/sdspi/SDSPIEUSCIA.h>
 *  @endcode
 *
 *  Refer to @ref SDSPI.h for a complete description of APIs & example of use.
 *
 *  This SDSPI driver implementation is designed to operate on a USCI SPI
 *  controller in a simple polling method.
 *
 *  ============================================================================
 */

#ifndef ti_drivers_sdspi_SDSPIEUSCIA__include
#define ti_drivers_sdspi_SDSPIEUSCIA__include

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <ti/drivers/SDSPI.h>

/*
 * DIR gets defined in msp430.h and ff.h
 * We need the defined DIR data structure from ff.h
 */
#undef DIR

#include <ti/mw/fatfs/ff.h>
#include <ti/mw/fatfs/diskio.h>

/**
 *  @addtogroup SDSPI_STATUS
 *  SDSPIEUSCIA_STATUS_* macros are command codes only defined in the
 *  SDSPIEUSCIA.h driver implementation and need to:
 *  @code
 *  #include <ti/drivers/sdspi/SDSPIEUSCIA.h>
 *  @endcode
 *  @{
 */

/* Add SDSPIEUSCIA_STATUS_* macros here */

/** @}*/

/**
 *  @addtogroup SDSPI_CMD
 *  SDSPIEUSCIA_CMD_* macros are command codes only defined in the
 *  SDSPIEUSCIA.h driver implementation and need to:
 *  @code
 *  #include <ti/drivers/sdspi/SDSPIEUSCIA.h>
 *  @endcode
 *  @{
 */

/* Add SDSPIEUSCIA_CMD_* macros here */

/** @}*/

/* SDSPI function table */
extern const SDSPI_FxnTable SDSPIEUSCIA_fxnTable;

/*!
 *  @brief  SD Card type inserted
 */
typedef enum SDSPIEUSCIA_CardType {
    SDSPIEUSCIA_NOCARD = 0, /*!< Unrecognized Card */
    SDSPIEUSCIA_MMC = 1,    /*!< Multi-media Memory Card (MMC) */
    SDSPIEUSCIA_SDSC = 2,   /*!< Standard SDCard (SDSC) */
    SDSPIEUSCIA_SDHC = 3    /*!< High Capacity SDCard (SDHC) */
} SDSPIEUSCIA_CardType;

/*!
 *  @brief  SDSPIEUSCIA Hardware attributes
 *
 *  The SDSPIEUSCIA configuration structure describes to the SDSPIEUSCIA
 *  driver implementation hardware specifies on which SPI peripheral, GPIO Pins
 *  and Ports are to be used.
 *
 *  The SDSPIEUSCIA driver uses this information to:
 *  - configure and reconfigure specific ports/pins to initialize the SD Card
 *    for SPI mode
 *  - identify which SPI peripheral is used for data communications
 *  - identify which GPIO port and pin is used for the SPI chip select
 *    mechanism
 *  - identify which GPIO port and pin is concurrently located on the SPI's MOSI
 *    (TX) pin.
 *  - identify the current version of the driver's HWAttrs
 *
 *  @remark
 *  To initialize the SD Card into SPI mode, the SDSPI driver changes the SPI's
 *  MOSI pin into a GPIO pin so it can kept driven HIGH while the SPI SCK pin
 *  can toggle. After the initialization, the TX pin is reverted back to the SPI
 *  MOSI mode.
 *
 *  These fields are used by driverlib APIs and therefore must be populated by
 *  driverlib macro definitions. For TivaWare these definitions are found in:
 *      - inc/hw_memmap.h
 *      - eusci_a_spi.h
 *
 *  @struct SDSPIEUSCIA_HWAttrsV1
 *  An example configuration structure could look as the following:
 *  @code
 *  const SDSPIEUSCIA_HWAttrsV1 sdspiEUSCIAHWAttrs = {
 *      {
 *           .baseAddr = EUSCI_A0_BASE, // SPI Peripheral's base address
 *           .clockSource = EUSCI_A_SPI_CLOCKSOURCE_SMCLK, // Clock source
 *
 *           .portSCK = GPIO_PORT_P2,  // SPI SCK PORT
 *           .pinSCK = GPIO_PIN2,      // SPI SCK PIN
 *                                     // SPI SCK PIN Mode
 *           .pinSCKModeFunction = GPIO_SECONDARY_MODULE_FUNCTION,
 *
 *           .portMISO = GPIO_PORT_P1, // SPI MISO PORT
 *           .pinMISO = GPIO_PIN7,     // SPI MISO PIN
 *                                     // SPI MISO PIN Mode
 *           .pinMISOModeFunction = GPIO_SECONDARY_MODULE_FUNCTION,
 *
 *           .portMOSI = GPIO_PORT_P1, // SPI MOSI PORT
 *           .pinMOSI = GPIO_PIN6,     // SPI MOSI PIN
 *                                     // SPI MOSI PIN Mode
 *           .pinMOSIModeFunction = GPIO_SECONDARY_MODULE_FUNCTION,
 *
 *           .portCS = GPIO_PORT_P3,   // GPIO Chip select port
 *           .pinCS = GPIO_PIN4,       // GPIO Chip select pin
 *       },
 *  };
 *  @endcode
 */
typedef struct SDSPIEUSCIA_HWAttrsV1 {
    uint32_t   baseAddr; /*!< SPI Peripheral's base address */
    uint8_t    clockSource; /*!< SPIUSCI Clock source */

    uint8_t    portSCK;  /*!< SPI port uses for the SCK, MISO, and MOSI pins */
    uint32_t   pinSCK;   /*!< SPI SCK pin */
    uint8_t    pinSCKModeFunction;  /*!< SCK GPIO pin mode selection */

    uint8_t    portMISO; /*!< SPI port uses for the SCK, MISO, and MOSI pins */
    uint32_t   pinMISO;  /*!< SPI MISO pin */
    uint8_t    pinMISOModeFunction;  /*!< MISO GPIO pin mode selection */

    uint8_t    portMOSI; /*!< SPI port uses for the SCK, MISO, and MOSI pins */
    uint32_t   pinMOSI;  /*!< SPI MOSI pin */
    uint8_t    pinMOSIModeFunction;  /*!< MOSI GPIO pin mode selection */

    uint8_t    portCS;   /*!< GPIO Port used for the chip select */
    uint32_t   pinCS;    /*!< GPIO Pin used for the chip select */
} SDSPIEUSCIA_HWAttrsV1;

/*!
 *  @brief  SDSPIEUSCIA Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct SDSPIEUSCIA_Object {
    uint16_t             driveNumber;   /*!< Drive number used by FatFs */
    DSTATUS              diskState;     /*!< Disk status */
    SDSPIEUSCIA_CardType cardType;      /*!< SDCard Card Command Class (CCC) */
    uint32_t             bitRate;       /*!< SPI bus bit rate (Hz) */
    FATFS                filesystem;    /*!< FATFS data object */
} SDSPIEUSCIA_Object, *SDSPIEUSCIA_Handle;

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_sdspi_SDSPIEUSCIA__include */
