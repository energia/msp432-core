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

#include "Energia.h"
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/drivers/GPIO.h>

void interrupts(void)
{
    Hwi_enable();
}

void noInterrupts(void)
{
    Hwi_disable();
}

void attachInterrupt(uint8_t pin, void (*userFunc)(void), int mode)
{
    GPIO_PinConfig intType;

    switch(mode) {
        case LOW:
            intType = GPIO_CFG_IN_INT_LOW;
            break;
        case CHANGE:
            intType = GPIO_CFG_IN_INT_BOTH_EDGES;
            break;
        case RISING:
            intType = GPIO_CFG_IN_INT_RISING;
            break;
        case FALLING:
            intType = GPIO_CFG_IN_INT_FALLING;
            break;
        case HIGH:
            intType = GPIO_CFG_IN_INT_HIGH;
            break;
    }

    GPIO_setConfig(pin, GPIO_CFG_IN_INT_ONLY | intType);

    GPIO_setCallback(pin, (GPIO_CallbackFxn)userFunc);

    GPIO_enableInt(pin);
}

void detachInterrupt(uint8_t pin) {
    GPIO_setCallback(pin, NULL);
}

void disablePinInterrupt(uint8_t pin) {
    GPIO_disableInt(pin);
}

void enablePinInterrupt(uint8_t pin) {
    GPIO_enableInt(pin);
}

