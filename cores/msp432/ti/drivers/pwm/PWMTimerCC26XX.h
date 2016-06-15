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
 * @file       PWMTimerCC26XX.h
 *
 * @brief      PWM driver implementation using Tiva General Purpose
 *             Timer peripherals.
 *
 * The PWM header file should be included in an application as follows:
 * @code
 * #include <ti/drivers/PWM.h>
 * #include <ti/drivers/pwm/PWMTimerCC26XX.h>
 * @endcode
 *
 * ## Operation #
 * This driver configures a Tiva General Purpose Timer (GPT) in PWM mode.
 * Creating a PWM instance will make the corresponding timer unavailable to the
 * TI-RTOS kernel until the PWM instance is closed. Additionally, if the
 * required timer is already used by the kernel, the PWM instance will not be
 * opened.
 *
 * When in PWM mode, each GPT is divided into 2 PWM outputs.  This driver
 * manages each output as independent PWM instances.  The timer is automatically
 * configured in count-down mode using the system clock as the source.
 *
 * The period in the PWM_Params structure must be used to set the intial period.
 * After opening, the PWM_control() API can be used to change a period.  The
 * period and duty registers are 16 bits wide; thus, prescalars are used to
 * extend period and duty registers.  The maximum period supported is calculated
 * as:
 *     MAX_PERIOD = (((MAX_PRESCALAR + 1) * MAX_MATCH_VAL) / CYCLES_PER_US) - 1
 *     80 MHz clock: (((255 + 1) * 65535) / 80) - 1 = 209711 microseconds
 *     120 MHz clock: (((255 + 1) * 65535) / 120) - 1 = 139807 microseconds
 *
 * Below is an example of how to use the PWM_control() to change a period:
 *
 * @code
 * int rc = 0;
 * int newPeriod = 6000;   // Period in microseconds
 *
 * rc = PWM_control(pwmHandle, PWMTimerCC26XX_CHANGE_PERIOD, &newPeriod);
 * if (rc < 0) {
 *   // handle error condition
 * }
 * @endcode
 *
 * Updates to a PWM instance will occur instantaneously (i.e. GPT peripherals do
 * not have shadow registers).  Finally, if the duty supplied is greater than
 * the period, the output will remain in active state.
 *
 * =============================================================================
 */

#ifndef ti_driver_pwm_PWMTimerCC26XX__include
#define ti_driver_pwm_PWMTimerCC26XX__include

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drivers/PWM.h>

/**
 *  @addtogroup PWM_STATUS
 *  PWMTimerCC26XX_STATUS_* macros are command codes only defined in the
 *  PWMTimerCC26XX.h driver implementation and need to:
 *  @code
 *  #include <ti/drivers/pwm/PWMTimerCC26XX.h>
 *  @endcode
 *  @{
 */

/* Add PWMTimerCC26XX_STATUS_* macros here */

/** @}*/

/**
 *  @addtogroup PWM_CMD
 *  PWMTimerCC26XX_CMD_* macros are command codes only defined in the
 *  PWMTimerCC26XX.h driver implementation and need to:
 *  @code
 *  #include <ti/drivers/pwm/PWMTimerCC26XX.h>
 *  @endcode
 *  @{
 */

/*!
 *  @brief Control command to change the PWM period.
 */
#define PWMTimerCC26XX_CMD_CHANGE_PERIOD    PWM_CMD_RESERVED + 0
/** @}*/

/* BACKWARDS COMPATIBILITY */
#define PWMTimerCC26XX_CHANGE_PERIOD        PWMTimerCC26XX_CMD_CHANGE_PERIOD
/* END BACKWARDS COMPATIBILITY */

/* PWM function table pointer */
extern const PWM_FxnTable PWMTimerCC26XX_fxnTable;

/*!
 *  @brief  PWMTimerCC26XX PWM Pin Configuration
 *
 *  Pin configuration of the pin that the PWM signal should
 *  be output from.
 *
 *  Use the 'custom' field of the the PWM params passed to
 *  the PWM_open() call to override the default pwmPinId found in
 *  the HWAttrs structure.
 */
typedef struct PWMTimerCC26XX_PWMPinCfg {
    uint8_t pwmPinId;
 } PWMTimerCC26XX_PWMPinCfg;

 /*!
 *  @brief  PWMTimerCC26XX Hardware attributes
 *
 *  These fields are used by driverlib APIs and therefore must be populated by
 *  driverlib macro definitions. For CCWare these definitions are found in:
 *      - inc/hw_memmap.h
 *      - driverlib/timer.h
 *
 *  A sample structure is shown below:
 *  @code
 *  const PWMTimerCC26XX_HWAttrs PWMTimerCC26XXHWAttrs[] = {
 *      {TIMERA3_BASE, TIMER_A},
 *      {TIMERA3_BASE, TIMER_B},
 *  };
 *  @endcode
 */
typedef struct PWMTimerCC26XX_HWAttrs {
    uint32_t   baseAddr;        /*!< Timer peripheral base address */
    uint16_t   timer;           /*!< Half-timers to generate outputs */
    uint32_t   powerMngrId;     /*!< UART Peripheral's power manager ID */
    uint8_t    pwmPinId;        /*!< The PIN ID of the PWM output */
} PWMTimerCC26XX_HWAttrs;

/*!
 *  @brief  PWMTimerCC26XX Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct PWMTimerCC26XX_Object {
    uint32_t period;
    uint32_t duty;
    uint8_t  dutyMode;            /* Units in which duty is specified */
    uint8_t  cyclesPerMicroSec;
    uint8_t  timerNum;            /* 0-7 */
} PWMTimerCC26XX_Object;

#ifdef __cplusplus
}
#endif

#endif /* ti_driver_pwm_PWMTimerCC26XX__include */
