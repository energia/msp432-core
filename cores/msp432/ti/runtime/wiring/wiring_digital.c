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

#include "wiring_private.h"
#include <ti/drivers/GPIO.h>

/* device specific routine */
GPIO_PinConfig mode2gpioConfig(uint8_t pin, uint8_t mode)
{
   switch (mode) {
        case INPUT:
            digital_pin_to_pin_function[pin] = PIN_FUNC_DIGITAL_INPUT;
            return (GPIO_CFG_IN_NOPULL);
	    
        case INPUT_PULLUP:
            digital_pin_to_pin_function[pin] = PIN_FUNC_DIGITAL_INPUT;
            return (GPIO_CFG_IN_PU);

        case INPUT_PULLDOWN:
            digital_pin_to_pin_function[pin] = PIN_FUNC_DIGITAL_INPUT;
            return (GPIO_CFG_IN_PD);

        case OUTPUT:
            digital_pin_to_pin_function[pin] = PIN_FUNC_DIGITAL_OUTPUT;
            return (GPIO_CFG_OUT_STR_HIGH);
    }

    /* unknown mode */
    digital_pin_to_pin_function[pin] = PIN_FUNC_UNUSED;
    return (GPIO_DO_NOT_CONFIG);
}

void pinMode(uint8_t pin, uint8_t mode)
{
    /* undo any previous plumbing */
    switch (digital_pin_to_pin_function[pin])
    {
        case PIN_FUNC_ANALOG_OUTPUT:
            stopAnalogWrite(pin);
            break;

        case PIN_FUNC_ANALOG_INPUT:
            stopAnalogRead(pin);
            break;
    }

    GPIO_PinConfig gpioConfig = mode2gpioConfig(pin, mode);

    if (gpioConfig != GPIO_DO_NOT_CONFIG) {
        GPIO_setConfig(pin, gpioConfig);
    }
}

int digitalRead(uint8_t pin)
{
    if (digital_pin_to_pin_function[pin] != PIN_FUNC_DIGITAL_INPUT) {
        pinMode(pin, INPUT);
    }

    if (GPIO_read(pin)) {
        return (HIGH);
    }

    return (LOW);
}

void digitalWrite(uint8_t pin, uint8_t val)
{
    if (digital_pin_to_pin_function[pin] != PIN_FUNC_DIGITAL_OUTPUT) {
        pinMode(pin, OUTPUT);
    }

    GPIO_write(pin, val ? 1 : 0);
}
