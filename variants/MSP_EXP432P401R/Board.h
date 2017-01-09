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
 *  @file       Board.h
 *
 *  @brief      MSP_EXP432P401R Launch Pad Board Specific APIs
 *
 *  The Board header file should be included in an application as
 *  follows:
 *  @code
 *  #include <Board.h>
 *  @endcode
 *
 *  ============================================================================
 */
#ifndef __Board_H
#define __Board_H

#include <ti/drivers/UART.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/SPI.h>

#ifdef __cplusplus
extern "C" {
#endif

/* LEDs on MSP_EXP432P401R Launch Pad are active high. */
#define Board_LED_OFF (0)
#define Board_LED_ON  (~0)

/*!
 *  @def    Board_GPIOName
 *  @brief  Enum of GPIO names on the MSP_EXP432P401R Launch Pad dev board
 */
typedef enum Board_GPIOName {
    Board_LED1 = 0,
    Board_LED_RED,
    Board_LED_GREEN,
    Board_LED_BLUE,
    Board_S1,
    Board_S2,

    Board_GPIOCOUNT
} Board_GPIOName;

/*!
 *  @def    Board_I2CName
 *  @brief  Enum of I2C names on the MSP_EXP432P401R Launch Pad dev board
 */
typedef enum Board_I2CName {
    Board_I2CB0 = 0,

    Board_I2CCOUNT
} Board_I2CName;

/*!
 *  @def    Board_PWMName
 *  @brief  Enum of PWM names on the MSP_EXP432P401R Launch Pad dev board
 */
typedef enum Board_PWMName {
    Board_PWM0 = 0,
    Board_PWM1,
    Board_PWM2,
    Board_PWM3,
    Board_PWM4,
    Board_PWM5,
    Board_PWM6,
    Board_PWM7,
    Board_PWM8,
    Board_PWM9,
    Board_PWM10,
    Board_PWM11,

    Board_PWMCOUNT
} Board_PWMName;

/*!
 *  @def    Board_SDSPIName
 *  @brief  Enum of SDSPI names on the MSP_EXP432P401R Launch Pad dev board
 */
typedef enum Board_SDSPIName {
    Board_SDSPIB0 = 0,

    Board_SDSPICOUNT
} EBoard_SDSPIName;

/*!
 *  @def    Board_SPIName
 *  @brief  Enum of SPI names on the MSP_EXP432P401R Launch Pad dev board
 */
typedef enum Board_SPIName {
    Board_SPIB0 = 0,
    Board_SPIB2,

    Board_SPICOUNT
} Board_SPIName;

/*!
 *  @def    Board_UARTName
 *  @brief  Enum of UART names on the MSP_EXP432P401R Launch Pad dev board
 */
typedef enum Board_UARTName {
    Board_UARTA0 = 0,
    Board_UARTA2,

    Board_UARTCOUNT
} Board_UARTName;

/*!
 *  @def    Board_WatchdogName
 *  @brief  Enum of Watchdog names on the MSP_EXP432P401R Launch Pad dev board
 */
typedef enum Board_WatchdogName {
    Board_WATCHDOG = 0,

    Board_WATCHDOGCOUNT
} Board_WatchdogName;

/*!
 *  @def    Board_WiFiName
 *  @brief  Enum of WiFi names on the MSP_EXP432P401R Launch Pad dev board
 */
typedef enum Board_WiFiName {
    Board_WIFI = 0,

    Board_WIFICOUNT
} Board_WiFiName;

/* Board specific I2C addresses */
#define Board_TMP006_ADDR           (0x40)
#define Board_RF430CL330_ADDR       (0x28)
#define Board_TPL0401_ADDR          (0x40)

#ifdef __cplusplus
}
#endif

#endif /* __Board_H */
