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
 * @file       PWMTimerCC3200.h
 *
 * @brief      PWM driver implementation using CC3200 General Purpose Timers.
 *
 * The PWM header file should be included in an application as follows:
 * @code
 * #include <ti/drivers/PWM.h>
 * #include <ti/drivers/pwm/PWMTimerCC3200.h>
 * @endcode
 *
 * Refer to @ref PWM.h for a complete description of APIs & example of use.
 *
 * ## Operation #
 * This driver configures a CC3200 General Purpose Timer (GPT) in PWM mode.
 * When in PWM mode, each GPT is divided into 2 PWM outputs.  This driver
 * manages each output as an independent PWM instance.  The timer is
 * automatically configured in count-down mode using the system clock as
 * the source.
 *
 * The timers operate at the system clock frequency (80 MHz). So each timer
 * tick is 12.5 ns. The period and duty registers are 16 bits wide; thus
 * 8-bit prescalars are used to extend period and duty registers.  The
 * maximum value supported is 16777215 timer counts ((2^24) - 1) or
 * 209715 microseconds.  Updates to a PWM's period or duty will occur
 * instantaneously (GPT peripherals do not have shadow registers).
 *
 * Finally, when this driver is opened, it automatically changes the
 * PWM pin's parking configuration (used when entering low power modes) to
 * correspond with the PWM_IDLE_LEVEL set in the PWM_params.  However, this
 * is setting is not reverted once the driver is close, it is the users
 * responsibility to change the parking configuration if necessary.
 *
 * =============================================================================
 */

#ifndef ti_driver_pwm_PWMTimerCC3200__include
#define ti_driver_pwm_PWMTimerCC3200__include

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <ti/drivers/PWM.h>

/**
 *  @addtogroup PWM_STATUS
 *  PWMTimerCC3200_STATUS_* macros are command codes only defined in the
 *  PWMTimerCC3200.h driver implementation and need to:
 *  @code
 *  #include <ti/drivers/pwm/PWMTimerCC3200.h>
 *  @endcode
 *  @{
 */

/* Add PWMTimerCC3200_STATUS_* macros here */

/** @}*/

/**
 *  @addtogroup PWM_CMD
 *  PWMTimerCC3200_CMD_* macros are command codes only defined in the
 *  PWMTimerCC3200.h driver implementation and need to:
 *  @code
 *  #include <ti/drivers/pwm/PWMTimerCC3200.h>
 *  @endcode
 *  @{
 */

/* Add PWMTimerCC3200_CMD_* macros here */

/** @}*/

/* PWM function table pointer */
extern const PWM_FxnTable PWMTimerCC3200_fxnTable;

/*!
 *  @brief  PWMTimerCC3200 Hardware attributes
 *
 *  These fields are used by CC3200 driverlib APIs and therefore must be
 *  populated by driverlib macro definitions. For CC3200 driverlib these
 *  definitions are found in:
 *      - inc/hw_memmap.h
 *      - driverlib/gpio.h
 *      - driverlib/pin.h
 *      - driverlib/timer.h
 *
 *  A sample structure is shown below:
 *  @code
 *  const PWMTimerCC3200_HWAttrsV1 pwmTimerCC3200HWAttrs[] = {
 *      {
 *          .timerBaseAddr = TIMERA3_BASE,
 *          .halfTimer = TIMER_A,
 *          .pinTimerPwmMode = PIN_MODE_3,
 *          .pinId = PIN_01,
 *          .gpioBaseAddr = GPIOA1_BASE,
 *          .gpioPinIndex = GPIO_PIN_2
 *      },
 *      {
 *          .timerBaseAddr = TIMERA3_BASE,
 *          .halfTimer = TIMER_B,
 *          .pinTimerPwmMode = PIN_MODE_3,
 *          .pinId = PIN_02,
 *          .gpioBaseAddr = GPIOA1_BASE,
 *          .gpioPinIndex = GPIO_PIN_3
 *      }
 *  };
 *  @endcode
 */
typedef struct PWMTimerCC3200_HWAttrsV1 {
    /*!< Timer peripheral base address */
    uint32_t timerBaseAddr;
    /*!< Half-timer to generate outputs (ex.: TIMER_A or TIMER_B) */
    uint16_t halfTimer;
    /*!< Timer output pin mode (ex.: PIN_MODE_3, PIN_MODE_6, etc.) */
    uint8_t  pinTimerPwmMode;
    /*!< Pin number (ex.: PIN_01, PIN_02, etc.) */
    uint8_t  pinId;

    /*!< GPIO port base address for the device pin. */
    uint32_t gpioBaseAddr;
    /*!< GPIO port pin index (ex.: GPIO_PIN_1, GPIO_PIN_3, etc.) */
    uint8_t  gpioPinIndex;
} PWMTimerCC3200_HWAttrsV1;

/*!
 *  @brief  PWMTimerCC3200 Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct PWMTimerCC3200_Object {
    Power_NotifyObj  postNotify;
    unsigned int     gpioPowerMgrId;
    unsigned int     timerPowerMgrId;
    uint32_t         duty;
    uint32_t         period;
    PWM_Duty_Units   dutyUnits;
    PWM_Period_Units periodUnits;
    PWM_IdleLevel    idleLevel;
    bool             pwmStarted;
    bool             isOpen;
} PWMTimerCC3200_Object;

#ifdef __cplusplus
}
#endif

#endif /* ti_driver_pwm_PWMTimerCC3200__include */
