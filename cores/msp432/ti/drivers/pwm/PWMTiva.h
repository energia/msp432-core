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
 * @file       PWMTiva.h
 *
 * @brief      PWM driver implementation for Tiva PWM peripherals.
 *
 * The PWM header file should be included in an application as follows:
 * @code
 * #include <ti/drivers/PWM.h>
 * #include <ti/drivers/pwm/PWMTiva.h>
 * @endcode
 *
 *  Refer to @ref PWM.h for a complete description of APIs & example of use.
 *
 * ## Operation #
 * This driver implementation uses the Pulse Width Modulator (PWM) peripherals
 * present on Tiva devices to generate PWM signals.  Each PWM peripheral
 * instance contains 4 PWM signal generators, each controlling 2 PWM outputs
 * (8 PWM outputs total).  This driver manages each PWM output individually
 * (each output has it's own PWM handle/instance).  However since a single clock
 * prescalar is available for a peripheral and a generator is responsible for
 * producing 2 outputs, there are some limitations in place to ensure proper
 * operation:
 *     - The peripheral prescalar will be set according to the period of the
 *       first PWM instance opened.  Any subsequent outputs will fail to open if
 *       a greater prescalar is required to generate the PWM period.
 *     - A PWM generators period is set to the period of first the instance
 *       opened.  Opening the second output will fail if the period used is
 *       not the same as what was set by the first output.
 *     - A PWM generators options are set by the first instance opened.  Opening
 *       the second output will fail if the options are not the same as what was
 *       set by the first output.
 *
 * Since the period and duty registers are 16 bits wide the prescalar is used to
 * divide the input clock and allow for larger periods.  The maximum period
 * supported is calculated as:
 *     - MAX_PERIOD = (MAX_PRESCALAR * MAX_MATCH_VALUE) / CYCLES_PER_US
 *     - Ex:
 *         - 80 MHz clock: (64 * 65535) / 80 = 52428 microseconds
 *         - 120 MHz clock: (64 * 65535) / 120 = 34952 microseconds
 *
 * After opening, the PWM_setPeriodl() API can be used to change a PWM
 * generator period. However, the clock prescalar is shared by all generators
 * so the new period must be a value that can generated with the same
 * prescaler.  Also keep in mind that changing a period affects both generator
 * outputs, so the period must be larger than both duties.  The equation
 * below can be used to determine the prescalar for a given period (the
 * prescalar will be the following power of 2 (2^x)):
 *     - prescalar = (period * CYCLES_PER_US) / MAX_MATCH_VALUE
 *     - Ex:
 *         - 100 microseconds -> (100 * 80) / 65535 = (0.1220) = 1
 *         - 10000 microseconds -> (10000 * 80) / 65535 = (12.20) = 16
 *
 * =============================================================================
 */

#ifndef ti_driver_pwm_PWMTiva__include
#define ti_driver_pwm_PWMTiva__include

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <ti/drivers/PWM.h>

/**
 *  @addtogroup PWM_STATUS
 *  PWMTiva_STATUS_* macros are command codes only defined in the
 *  PWMTiva.h driver implementation and need to:
 *  @code
 *  #include <ti/drivers/pwm/PWMTiva.h>
 *  @endcode
 *  @{
 */

/* Add PWMTiva_STATUS_* macros here */

/** @}*/

/**
 *  @addtogroup PWM_CMD
 *  PWMTiva_CMD_* macros are command codes only defined in the
 *  PWMTiva.h driver implementation and need to:
 *  @code
 *  #include <ti/drivers/pwm/PWMTiva.h>
 *  @endcode
 *  @{
 */

/** @}*/

/* Number of PWM peripherals available on a device. */
#define PWMTiva_NUM_PWM_PERIPHERALS      (2)

/* Number of signal generator blocks per PWM peripheral. */
#define PWMTiva_NUM_PWM_GENERATORS       (4)

 /* Number of PWM signals a PWM peripheral can generate. */
#define PWMTiva_NUM_PWM_OUTPUTS          (8)

/* PWM function table pointer */
extern const PWM_FxnTable PWMTiva_fxnTable;

/*!
 *  @brief  PWMTiva Hardware attributes
 *
 *  These fields are used by driverlib APIs and therefore must be populated by
 *  driverlib macro definitions. For TivaWare these definitions are found in:
 *      - inc/hw_memmap.h
 *      - driverlib/gpio.h
 *      - driverlib/pwm.h
 *
 *  A sample structure is shown below:
 *  @code
 *  const PWMTiva_HWAttrs PWMTivaHWAttrs[] = {
 *      {
 *          .pwmBaseAddr = PWM0_BASE,
 *          .pwmOutput = PWM_OUT_0,
 *          .pwmGenOpts = PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DBG_RUN,
 *          .pinPwmMode = GPIO_PF0_M0PWM0,
 *          .gpioBaseAddr = GPIO_PORTF_BASE,
 *          .gpioPinIndex = GPIO_PIN_0
 *      }
 *  };
 *  @endcode
 */
typedef struct PWMTiva_HWAttrsV1 {
    /*!< PWM peripheral base address (ex.: PWM0_BASE, PWM1_BASE, etc.). */
    uint32_t pwmBaseAddr;
    /*!< Encoded PWM offset address (ex.: PWM_OUT_0, PWM_OUT_5, etc.). */
    uint32_t pwmOutput;
    /*!< Generator options for PWM (ex.: PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DBG_RUN). */
    uint32_t pwmGenOpts;
    /*!< PWM output pin mode (ex.: GPIO_PF0_M0PWM0, GPIO_PF2_M0PWM2, etc.). */
    uint32_t  pinPwmMode;
    /*!< GPIO port base address for the device pin (ex.: GPIO_PORTA_BASE, GPIO_PORTF_BASE, etc). */
    uint32_t gpioBaseAddr;
    /*!< GPIO port pin index (ex.: GPIO_PIN_0, GPIO_PIN_4, etc.) */
    uint8_t  gpioPinIndex;

} PWMTiva_HWAttrsV1;

/*!
 *  @brief PWMTiva_Status
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct PWMTiva_Status {
    uint32_t pwmDuties[PWMTiva_NUM_PWM_OUTPUTS];
    uint32_t genPeriods[PWMTiva_NUM_PWM_GENERATORS];
    uint8_t  prescalar;
    uint8_t  activeOutputs;
} PWMTiva_Status;

/*!
 *  @brief  PWMTiva Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct PWMTiva_Object {
    PWMTiva_Status   *pwmStatusStruct;
    PWM_Duty_Units    dutyUnits;
    PWM_Period_Units  periodUnits;
    PWM_IdleLevel     idleLevel;
    uint8_t           pwmOutputNum;
    uint8_t           pwmOutputBit;
    bool              isOpen;
} PWMTiva_Object;

#ifdef __cplusplus
}
#endif

#endif /* ti_driver_pwm_PWMTiva__include */
