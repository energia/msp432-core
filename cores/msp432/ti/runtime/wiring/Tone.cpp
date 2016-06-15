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

#include <ti/sysbios/hal/Timer.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/Error.h>

extern uint32_t toneTimerId;

static Timer_Params timerParams;
static Timer_Handle timerHandle;

static bool initTimer = true;
static bool playing = false;
static bool togglePin = true;
static uint8_t tonePin;
static bool toneState;
static unsigned long toneDuration;
static unsigned long mark;
static unsigned int currentFrequency;

static void ToneIntHandler(UArg arg0)
{
    if (millis() - mark > toneDuration && toneDuration != 0) {
        Timer_stop(timerHandle);
        digitalWrite(tonePin, LOW);
        playing = false;
        return;
    }

    if (togglePin) {
        digitalWrite(tonePin, toneState);
    }
    toneState = !toneState;
}

void tone(uint8_t _pin, unsigned int frequency, unsigned long duration)
{
    if (playing && tonePin != _pin) {
        return;
    }

    if (frequency > 20000) {
        return;
    }

    playing = true;
    toneDuration = duration;

    togglePin = true;

    if (frequency == 0) {
        togglePin = false;
        frequency = 1000;
    }

    currentFrequency = frequency;

    if (_pin != tonePin) {
        pinMode(tonePin, OUTPUT);
        tonePin = _pin;
    }

    if (initTimer) {
        Error_Block eb;
        Error_init(&eb);
        Timer_Params_init(&timerParams);
        timerParams.period = (1000000L / frequency) / 2;
        timerParams.runMode = Timer_RunMode_CONTINUOUS;
        timerHandle = Timer_create(toneTimerId, ToneIntHandler, &timerParams, &eb);
        initTimer = false;
    }

    Timer_stop(timerHandle);
    Timer_setPeriodMicroSecs(timerHandle, (1000000L / frequency) / 2);
    Timer_start(timerHandle);

    mark = millis();
}

void noTone(uint8_t _pin)
{
    if (!initTimer) {
        Timer_stop(timerHandle);
        digitalWrite(tonePin, LOW);
        playing = false;
    }
}
