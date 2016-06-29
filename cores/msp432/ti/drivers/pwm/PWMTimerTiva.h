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
 * @file       PWMTimerTiva.h
 *
 * @brief      PWM driver implementation using Tiva General Purpose Timers.
 *
 * The PWM header file should be included in an application as follows:
 * @code
 * #include <ti/drivers/PWM.h>
 * #include <ti/drivers/pwm/PWMTimerTiva.h>
 * @endcode
 *
 * Refer to @ref PWM.h for a complete description of APIs & example of use.
 *
 * ## Operation #
 * This driver configures a Tiva General Purpose Timer (GPT) in PWM mode.
 * When in PWM mode, each GPT is divided into 2 PWM outputs.  This driver
 * manages each output as an independent PWM instance.  The timer is
 * automatically configured in count-down mode using the system clock as
 * the source.
 *
 * The timers operate at the system clock frequency (80 MHz or 120 MHz). So each
 * timer tick is 12.5 ns or 8.3 ns respectively. The period and duty registers
 * are 16 bits wide; thus 8-bit prescalars are used to extend period and duty
 * registers.  The maximum value supported is 16777215 ((2^24) - 1) timer
 * counts; 209715 microseconds @ 80 MHz or 139810 microseconds @ 120 MHz.
 * Updates to a PWM's period or duty will occur instantaneously
 * (GPT peripherals do not have shadow registers).
 *
 * =============================================================================
 */

#ifndef ti_driver_pwm_PWMTimerTiva__include
#define ti_driver_pwm_PWMTimerTiva__include

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <ti/drivers/PWM.h>

/**
 *  @addtogroup PWM_STATUS
 *  PWMTimerTiva_STATUS_* macros are command codes only defined in the
 *  PWMTimerTiva.h driver implementation and need to:
 *  @code
 *  #include <ti/drivers/pwm/PWMTimerTiva.h>
 *  @endcode
 *  @{
 */

/* Add PWMTimerTiva_STATUS_* macros here */

/** @}*/

/**
 *  @addtogroup PWM_CMD
 *  PWMTimerTiva_CMD_* macros are command codes only defined in the
 *  PWMTimerTiva.h driver implementation and need to:
 *  @code
 *  #include <ti/drivers/pwm/PWMTimerTiva.h>
 *  @endcode
 *  @{
 */

/* Add PWMTimerTiva_CMD_* macros here */

/** @}*/

/* PWM function table pointer */
extern const PWM_FxnTable PWMTimerTiva_fxnTable;

/*!
 *  @brief  PWMTimerTiva Hardware attributes
 *
 *  These fields are used by driverlib APIs and therefore must be populated by
 *  driverlib macro definitions. For TivaWare these definitions are found in:
 *      - inc/hw_memmap.h
 *      - driverlib/gpio.h
 *      - driverlib/pin_map.h
 *      - driverlib/timer.h
 *
 *  A sample structure is shown below:
 *  @code
 *  const PWMTimerTiva_HWAttrsV1 pwmTivaHWAttrs[] = {
 *      {
 *          .timerBaseAddr = TIMER2_BASE,
 *  	    .halfTimer = TIMER_A,
 *          .pinTimerPwmMode = GPIO_PA4_T2CCP0,
 *          .gpioBaseAddr = GPIO_PORTA_BASE,
 *          .gpioPinIndex = GPIO_PIN_4
 *      }
 *  };
 *  @endcode
 */
typedef struct PWMTimerTiva_HWAttrsV1 {
    /*!< Timer peripheral base address */
    uint32_t timerBaseAddr;
    /*!< Half-timer to generate outputs (ex.: TIMER_A or TIMER_B) */
    uint16_t halfTimer;
    /*!< Timer output pin mode (ex.: GPIO_PA0_T0CCP0, GPIO_PA1_T0CCP1, etc.) */
    uint32_t  pinTimerPwmMode;

    /*!< GPIO port base address for the device pin. */
    uint32_t gpioBaseAddr;
    /*!< GPIO port pin index (ex.: GPIO_PIN_0, GPIO_PIN_4, etc.) */
    uint8_t  gpioPinIndex;
} PWMTimerTiva_HWAttrsV1;

/*!
 *  @brief  PWMTimerTiva Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct PWMTimerTiva_Object {
    uint32_t         duty;
    uint32_t         period;
    PWM_Duty_Units   dutyUnits;
    PWM_Period_Units periodUnits;
    PWM_IdleLevel    idleLevel;
    bool             isOpen;
} PWMTimerTiva_Object;

#ifdef __cplusplus
}
#endif

#endif /* ti_driver_pwm_PWMTimerTiva__include */
