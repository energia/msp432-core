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

#ifndef WiringAnalog_h
#define WiringAnalog_h

#include <ti/runtime/wiring/Energia.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NOT_A_PIN       0
#define NOT_ON_ADC      0xff

/* PWM digital_pin_to_pwm_index table encodings */
#define PWM_MAPPABLE        0xfe
#define PWM_NOT_MAPPABLE    0xff

#define PWM_FIXED_INDEX_0   0
#define PWM_FIXED_INDEX_1   1
#define PWM_FIXED_INDEX_2   2
#define PWM_FIXED_INDEX_3   3
#define PWM_FIXED_INDEX_4   4
#define PWM_FIXED_INDEX_5   5
#define PWM_FIXED_INDEX_6   6
#define PWM_FIXED_INDEX_7   7
#define PWM_FIXED_INDEX_8   8
#define PWM_FIXED_INDEX_9   9
#define PWM_FIXED_INDEX_10  10
#define PWM_FIXED_INDEX_11  11
#define PWM_AVAILABLE_PWMS  12
#define PWM_MAX_MAPPABLE_INDEX   PWM_FIXED_INDEX_7

extern uint8_t digital_pin_to_pwm_index[];
extern const uint8_t digital_pin_to_adc_index[];

#ifdef __cplusplus
} // extern "C"
#endif

#endif
