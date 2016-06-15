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
 *  @file       GPIOCC26XX.h
 *
 *  @brief      CC26XX GPIO driver
 *
 *  The GPIO header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/gpio/GPIOCC26XX.h>
 *  @endcode
 *
 *  # Operation #
 *
 *  The GPIO module allows you to manage General Purpose I/O pins via simple
 *  and portable APIs. The application is required to supply a GPIOCC26XX_Config
 *  structure to the module (see example below). This structure communicates to
 *  the GPIO module how to configure the pins used by the application (See the
 *  description of GPIO_PinConfig in the GPIO.h file).
 *
 *  The application is required to call GPIO_init(). This function will
 *  initialize all the GPIO pins defined in the GPIO_PinConfig table to the
 *  configurations specified. Once completed the other APIs can be used to
 *  access the pins.
 *
 *  Asserts are used to verify that the driver has been initialized, and is
 *  reading/writing a valid index.
 *
 *  The following is an example of the code required to use the 2 switches and
 *  4 LEDs on the CC26XX Launchpad board.
 *
 *  Board header:
 *  @code
 *  // Enum of GPIO names on the CC26XX_LP dev board
 *  typedef enum CC26XX_LP_GPIOName {
 *      // input pins with callbacks first
 *      CC26XX_LP_SW2 = 0,
 *      CC26XX_LP_SW3,
 *      // output pins
 *      CC26XX_LP_LED_D7,
 *      CC26XX_LP_LED_D6,
 *      CC26XX_LP_LED_D5,
 *
 *      CC26XX_LP_GPIOCOUNT
 *  } CC26XX_LP_GPIOName;
 *  @endcode
 *
 *  Board initialization code:
 *  @code
 *  #include <ti/drivers/GPIO.h>
 *  #include <ti/drivers/gpio/GPIOCC26XX.h>
 *
 *  // Array of Pin configurations
 *  // NOTE: The order of the pin configurations must coincide with what was
 *  //       defined in CC26XX_LP.h
 *  // NOTE: Pins not used for interrupts should be placed at the end of the
 *           array.  Callback entries can be omitted from callbacks array to
 *           reduce memory usage.
 *  const GPIO_PinConfig gpioPinConfigs[] = {
 *      // input pins with callbacks
 *
 *      // CC26XX_LP_SW2
 *      GPIOCC26XX_DIO_22 | GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING,
 *      // CC26XX_LP_SW3
 *      GPIOCC26XX_DIO_13 | GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING,
 *
 *      // output pins
 *
 *      // CC26XX_LP_LED_D7
 *      GPIOCC26XX_DIO_09 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_LOW,
 *      // CC26XX_LP_LED_D6
 *      GPIOCC26XX_DIO_10 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_LOW,
 *      // CC26XX_LP_LED_D5
 *      GPIOCC26XX_DIO_11 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_LOW,
 *  };
 *
 *  // Array of callback function pointers
 *  // NOTE: The order of the pin configurations must coincide with what was
 *  //       defined in CC26XX_LP.h
 *  // NOTE: Pins not used for interrupts can be omitted from callbacks array to
 *           reduce memory usage (if placed at end of gpioPinConfigs array).
 *  const GPIO_callbackFxn gpioCallbackFunctions[] = {
 *      // CC26XX_LP_SW2
 *      NULL,
 *      // CC26XX_LP_SW3
 *      NULL
 *  };
 *
 *  //
 *  // The device-specific GPIO_config structure
 *  //
 *  const GPIOCC26XX_Config GPIOCC26XX_config = {
 *      .pinConfigs = (GPIO_PinConfig *)gpioPinConfigs,
 *      .callbacks = (GPIO_CallbackFxn *)gpioCallbackFunctions,
 *      .numberOfPinConfigs =
 *               sizeof(gpioPinConfigs)/sizeof(GPIO_PinConfig),
 *      .numberOfCallbacks =
 *               sizeof(gpioCallbackFunctions)/sizeof(GPIO_CallbackFxn),
 *      .intPriority = ~0
 *  };
 *
 *  // Initialize peripheral and pins
 *  void CC26XX_initGPIO(void)
 *  {
 *      // set up initial GPIO pin configurations
 *      GPIO_init();
 *  }
 *  @endcode
 *
 *  Keep in mind that the callback functions will be called in the context of
 *  an interrupt service routine and should be designed accordingly.  When an
 *  interrupt is triggered, the interrupt status of all (interrupt enabled) pins
 *  on a port will be read, cleared, and the respective callbacks will be
 *  executed.  Callbacks will be called in order from least significant bit to
 *  most significant bit.
 *
 *  # Instrumentation #
 *  The GPIO driver interface produces log statements if instrumentation is
 *  enabled.
 *
 *  Diagnostics Mask | Log details |
 *  ---------------- | ----------- |
 *  Diags_USER1      | basic operations performed |
 *
 *  ============================================================================
 */

#ifndef ti_drivers_GPIOCC26XX__include
#define ti_drivers_GPIOCC26XX__include

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <driverlib/ioc.h>

#include <ti/drivers/GPIO.h>

/*!
 *  @brief  GPIO device specific driver configuration structure
 */
typedef struct GPIOCC26XX_Config {
    /*! Pointer to the board's GPIO_PinConfig array */
    GPIO_PinConfig  *pinConfigs;

    /*! Pointer to the board's GPIO_CallbackFxn array */
    GPIO_CallbackFxn  *callbacks;

    /*! Number of GPIO_PinConfigs defined */
    uint32_t numberOfPinConfigs;

    /*! Number of GPIO_Callbacks defined */
    uint32_t numberOfCallbacks;

    /*!
     *  Interrupt priority used for call back interrupts.
     *
     *  intPriority is the interrupt priority, as defined by the
     *  underlying OS.  It is passed unmodified to the underlying OS's
     *  interrupt handler creation code, so you need to refer to the OS
     *  documentation for usage.  For example, for SYS/BIOS applications,
     *  refer to the ti.sysbios.family.arm.m3.Hwi documentation for SYS/BIOS
     *  usage of interrupt priorities.  If the driver uses the ti.drivers.ports
     *  interface instead of making OS calls directly, then the HwiP port
     *  handles the interrupt priority in an OS specific way.  In the case
     *  of the SYS/BIOS port, intPriority is passed unmodified to Hwi_create().
     *
     *  Setting ~0 will configure the lowest possible priority
     */
    uint32_t intPriority;
} GPIOCC26XX_Config;

/*!
 *  @brief  Device specific port/pin definition macros
 *
 *  Below are the port/pin definitions to be used within
 *  the board's pin configuration table.
 */
#define GPIOCC26XX_EMPTY_PIN  0xffff

#define GPIOCC26XX_DIO_00    IOID_0
#define GPIOCC26XX_DIO_01    IOID_1
#define GPIOCC26XX_DIO_02    IOID_2
#define GPIOCC26XX_DIO_03    IOID_3
#define GPIOCC26XX_DIO_04    IOID_4
#define GPIOCC26XX_DIO_05    IOID_5
#define GPIOCC26XX_DIO_06    IOID_6
#define GPIOCC26XX_DIO_07    IOID_7

#define GPIOCC26XX_DIO_08    IOID_8
#define GPIOCC26XX_DIO_09    IOID_9
#define GPIOCC26XX_DIO_10    IOID_10
#define GPIOCC26XX_DIO_11    IOID_11
#define GPIOCC26XX_DIO_12    IOID_12
#define GPIOCC26XX_DIO_13    IOID_13
#define GPIOCC26XX_DIO_14    IOID_14
#define GPIOCC26XX_DIO_15    IOID_15

#define GPIOCC26XX_DIO_16    IOID_16
#define GPIOCC26XX_DIO_17    IOID_17
#define GPIOCC26XX_DIO_18    IOID_18
#define GPIOCC26XX_DIO_19    IOID_19
#define GPIOCC26XX_DIO_20    IOID_20
#define GPIOCC26XX_DIO_21    IOID_21
#define GPIOCC26XX_DIO_22    IOID_22
#define GPIOCC26XX_DIO_23    IOID_23

#define GPIOCC26XX_DIO_24    IOID_24
#define GPIOCC26XX_DIO_25    IOID_25
#define GPIOCC26XX_DIO_26    IOID_26
#define GPIOCC26XX_DIO_27    IOID_27
#define GPIOCC26XX_DIO_28    IOID_28
#define GPIOCC26XX_DIO_29    IOID_29
#define GPIOCC26XX_DIO_30    IOID_30
#define GPIOCC26XX_DIO_31    IOID_31

/*!
 *  @brief     Un-oonfigure a GPIO pin
 *
 *  Disables pin interrupt, clears callback, restores pin to default setting,
 *  removes pin from PIN object
 *
 *  @param      index    GPIO index
 */
extern void GPIOCC26xx_release(int index);

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_GPIOCC26XX__include */
