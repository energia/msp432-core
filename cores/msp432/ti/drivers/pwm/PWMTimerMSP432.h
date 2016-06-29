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
 * @file       PWMTimerMSP432.h
 *
 * @brief      PWM driver implementation using Timer_A peripherals
 *
 * The PWM header file should be included in an application as follows:
 * @code
 * #include <ti/drivers/PWM.h>
 * #include <ti/drivers/pwm/PWMTimerMSP432.h>
 * @endcode
 *
 * *  Refer to @ref PWM.h for a complete description of APIs & example of use.
 *
 * ## Operation #
 * This driver configures an MSP432 Timer_A peripheral for PWM. If the
 * timer is already in use (by the kernel for instance), PWM instances will
 * not be opened.
 *
 * When used for PWM generation, each Timer_A can produce up to 6 PWM outputs
 * and this driver manages each output as an independent PWM instance.  However
 * since a single period and prescalar are used for all Timer outputs, there are
 * limitations in place to ensure proper operation:
 *     - The PWM period and prescalar are calculated and set based on the first
 *       instance opened. Opening a second instance will fail if the period
 *       is not the same as what was set by the first instance.
 *
 * The timer is automatically configured in count-up mode using the clock
 * source specified in the hwAttrs structure.  In PWM mode, the timer
 * capture/compare register 0 is used as the period register and cannot be
 * used to generate a PWM output.
 *
 * The period and duty registers are 16 bits wide, thus a prescalar is used to
 * divide the input clock and allow for larger periods.  The maximum period
 * supported is calculated as:
 *     - MAX_PERIOD = (MAX_PRESCALAR * MAX_MATCH_VALUE) / CYCLES_PER_US
 *     - Ex:
 *           - 12 MHz clock: (64 * 65535) / 12 = 349520 microseconds
 *           - 6 MHz clock: (64 * 65535) / 6 = 699040 microseconds
 *           - 3 MHz clock: (64 * 65535) / 3 = 1398080 microseconds
 *
 * After opening, the PWM_setPeriod() API can be used to change the PWM period.
 * Keep in mind the period is shared by all other PWMs on the timer, so all
 * other PWM outputs on the timer will change.  Additionally, a call to
 * PWM_setPeriod() will fail if the new period requires a prescalar different
 * than the one set when initially configured.
 *
 * =============================================================================
 */

#ifndef ti_driver_pwm_PWMTimerMSP432__include
#define ti_driver_pwm_PWMTimerMSP432__include

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <ti/drivers/PWM.h>

/**
 *  @addtogroup PWM_STATUS
 *  PWMTimerMSP432_STATUS_* macros are command codes only defined in the
 *  PWMTimerMSP432.h driver implementation and need to:
 *  @code
 *  #include <ti/drivers/pwm/PWMTimerMSP432.h>
 *  @endcode
 *  @{
 */

/* Add PWMTimerMSP432_STATUS_* macros here */

/** @}*/

/**
 *  @addtogroup PWM_CMD
 *  PWMTimerMSP432_CMD_* macros are command codes only defined in the
 *  PWMTimerMSP432.h driver implementation and need to:
 *  @code
 *  #include <ti/drivers/pwm/PWMTimerMSP432.h>
 *  @endcode
 *  @{
 */

/* Add PWMTimerMSP432_CMD_* macros here */

/** @}*/

/* Number of Timer_A peripherals available on a device. */
#define PWMTimerMSP432_NUM_TIMERS       (4)

/* Number of PWM outputs a Timer_A peripheral can generate. */
#define PWMTimerMSP432_NUM_PWM_OUTPUTS  (6)

/* PWM function table pointer */
extern const PWM_FxnTable PWMTimerMSP432_fxnTable;

/*!
 *  @brief  PWMTimerMSP432 Hardware attributes
 *
 *  These fields are used by driverlib APIs and therefore must be populated by
 *  driverlib macro definitions. For msp432_driverlib these definitions are
 *  found in:
 *      - msp432p401r.h
 *      - gpio.h
 *      - timer_a.h
 *
 *  A sample structure is shown below:
 *  @code
 *  const PWMTimerMSP432_HWAttrs PWMTimerMSP432HWAttrs[] = {
 *      {
 *          .timerBaseAddr = TIMER_A1_BASE,
 *          .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
 *          .compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1,
 *          .gpioPort = GPIO_PORT_P2,
 *          .gpioPinIndex = GPIO_PIN1,
 *          .pwmMode = GPIO_PRIMARY_MODULE_FUNCTION
 *      },
 *  };
 *  @endcode
 */
typedef struct PWMTimerMSP432_HWAttrsV1 {
    uint32_t timerBaseAddr;           /*!< PWMTimer peripheral base address*/
    uint16_t clockSource;
    uint8_t  compareRegister;    /*!< Timer compare register for PWM */
    uint8_t  gpioPort;
    uint8_t  gpioPinIndex;
    uint8_t  pwmMode;
} PWMTimerMSP432_HWAttrsV1;

/*!
 *  @brief PWMTimerMSP432_Status
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct PWMTimerMSP432_Status {
    uint32_t duties[PWMTimerMSP432_NUM_PWM_OUTPUTS];
    uint32_t period;
    uint8_t  prescalar;
    uint8_t  activeOutputsMask;
} PWMTimerMSP432_Status;

/*!
 *  @brief  PWMTimerMSP432 Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct PWMTimerMSP432_Object {
    PWMTimerMSP432_Status *timerStatusStruct;
    PWM_Period_Units       periodUnits;
    PWM_Duty_Units         dutyUnits;
    PWM_IdleLevel          idleLevel;
    uint8_t                compareOutputNum;
    bool                   pwmStarted;
    bool                   isOpen;
} PWMTimerMSP432_Object;

#ifdef __cplusplus
}
#endif

#endif /* ti_driver_pwm_PWMTimerMSP432__include */
