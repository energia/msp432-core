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
 *  @file       ADCMSP432.h
 *
 *  @brief      ADC driver implementation for the ADC peripheral on MSP432
 *
 *  This ADC driver implementation is designed to operate on a ADC14 peripheral
 *  for MSP432.  The ADC MSP432 header file should be included in an application
 *  as follows:
 *  @code
 *  #include <ti/drivers/ADC.h>
 *  #include <ti/drivers/ADCMSP432.h>
 *  @endcode
 *
 *  Refer to @ref ADC.h for a complete description of APIs & example of use.
 *
 *  ============================================================================
 */
#ifndef ti_drivers_adc_ADCMSP432__include
#define ti_drivers_adc_ADCMSP432__include

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include <ti/drivers/ADC.h>
#include <ti/drivers/ports/HwiP.h>
#include <ti/drivers/ports/SemaphoreP.h>
#include <ti/drivers/Power.h>

/* ADC function table pointer */
extern const ADC_FxnTable ADCMSP432_fxnTable;

/*!
 *  @brief  ADCMSP432 Hardware attributes
 *  These fields are used by driverlib APIs and therefore must be populated by
 *  driverlib macro definitions. For MSPWare these definitions are found in:
 *      - adc14.h
 *      - gpio.h
 *      - ref_a.h
 *
 *  A sample structure is shown below:
 *  @code
 *  const ADCMSP432_HWAttrs adcMSP432HWAttrs[Board_ADCCHANNELCOUNT] = {
 *      {
 *          .channel = ADC_INPUT_A0,
 *          .gpioPort = GPIO_PORT_P5,
 *          .gpioPin = GPIO_PIN5,
 *          .gpioMode = GPIO_TERTIARY_MODULE_FUNCTION,
 *          .refVoltage = REF_A_VREF2_5V,
 *          .resolution = ADC_14BIT
 *      }
 *  };
 *  @endcode
 */
typedef struct ADCMSP432_HWAttrs {
    uint_fast16_t  channel;    /*! ADC channel */
    uint_fast16_t  gpioPort;   /*! GPIO port for the ADC channel */
    uint_fast16_t  gpioPin;    /*! GPIO pin for the ADC channel */
    uint_fast16_t  gpioMode;   /*! GPIO function mode for the ADC channel */
    uint_fast16_t  refVoltage; /*! Reference voltage for ADC channel */
    uint_fast32_t  resolution; /*! ADC resolution for ADC channel  */
} ADCMSP432_HWAttrs;

/*!
 *  @brief  ADCMSP432 Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct ADCMSP432_Object {
    bool isOpen;               /* To determine if the ADC is open */
} ADCMSP432_Object;

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_adc_ADCMSP432__include */
