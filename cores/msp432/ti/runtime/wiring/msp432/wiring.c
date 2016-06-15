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

#include <ti/runtime/wiring/Energia.h>
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>
#define ti_sysbios_knl_Clock__internalaccess
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>

#include <ti/sysbios/family/arm/m3/Hwi.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerMSP432.h>

#include <rom.h>
#include <rom_map.h>
#include <wdt_a.h>

static uint8_t delayMode = 0; /* determines  which tick source is driving Clock_tick */

/*
 *  ======== micros ========
 */
unsigned long micros(void)
{
    Types_FreqHz freq;
    Types_Timestamp64 time;
    uint64_t t64;

    Timestamp_getFreq(&freq);
    Timestamp_get64(&time);
    t64 = ((uint64_t)time.hi << 32) | time.lo;
    return (t64/(freq.lo/1000000));
}

/*
 *  ======== millis ========
 */
unsigned long millis(void)
{
    return (Clock_getTicks());
}

/*
 *  ======== delayMicroseconds ========
 *  Delay for the given number of microseconds.
 */
void delayMicroseconds(unsigned int us)
{
    if (us <7) {
        //The overhead in calling and returning from this function takes about 6us
    }
    else if (us <=20) {
        int time;
        for (time = 5*(us-6); time > 0; time--) {
            asm("   nop");
        }
    }
    else if (us < 70) {
        int time;
        for (time = 5*us; time > 0; time--) {
            asm("   nop");
        }
    }
    else {
        uint32_t t0, deltaT;
        Types_FreqHz freq;

        Timestamp_getFreq(&freq);
        deltaT = us * (freq.lo/1000000);

        t0 = Timestamp_get32();

        while ((Timestamp_get32()-t0) < deltaT) {
            ;
        }
    }
}

/*
 *  ======== clockTickFxn ========
 *
 *  250ms Watchdog Timer interrupt handler.
 */
static void clockTickFxn(uintptr_t arg)
{
    /*
     * Bump Clock tick count by 249. 
     * Clock_tick() will bump it one more
     */
    ((ti_sysbios_knl_Clock_Module_State *)(&ti_sysbios_knl_Clock_Module__state__V))->ticks += 249;

    Clock_tick();
}

/*
 *  ======== switchToWatchdogTimer ========
 *
 *  Use 250ms watchdog timer interrupt to drive the Clock tick
 *  Stop the default Timer_A then start the watchdog timer.
 */
static void switchToWatchdogTimer()
{
    Clock_TimerProxy_Handle clockTimer;
    static Hwi_Handle wdtHwi = NULL;

    /* Stop Timer_A currrently being used by Clock */
    clockTimer = Clock_getTimerHandle();
    Clock_TimerProxy_stop(clockTimer);

    MAP_WDT_A_holdTimer();

    if (wdtHwi == NULL) {
        /* Create watchdog Timer Hwi */
        wdtHwi = Hwi_create(19, clockTickFxn, NULL, NULL);
        
        /* set WDT to use 32KHz input, 250ms period */
        MAP_WDT_A_initIntervalTimer(WDT_A_CLOCKSOURCE_XCLK, WDT_A_CLOCKITERATIONS_8192);
    }

    /* don't allow deeper than DEEPSLEEP1 */
    Power_setConstraint(PowerMSP432_DISALLOW_DEEPSLEEP_1);

    /* Start watchdog Timer */
    MAP_WDT_A_clearTimer();
    MAP_WDT_A_startTimer();

    /* hence, Clock_tick() will be called from 250ms watchdog timer interrupt */
}

/*
 *  ======== switchToTimerA ========
 *
 *  Use 1ms Timer_A interrupt to drive the Clock tick
 *  By default, the Timer_A Hwi object has already been
 *  statically created and configured to call Clock_tick().
 *  Simply stop the watchdog timer and restart the Timer_A.
 */
static void switchToTimerA()
{
    Clock_TimerProxy_Handle clockTimer;

    /* Stop watchdog Timer */
    MAP_WDT_A_holdTimer();

    /* Re-start Timer_A */
    clockTimer = Clock_getTimerHandle();
    Clock_TimerProxy_start(clockTimer);

    /* hence, Clock_tick() will be called from 1ms Timer_A interrupt */
}

/* 
 *  ======== delay ========
 */
void delay(uint32_t milliseconds)
{
    if (milliseconds == 0) {
        Task_yield();
        return;
    }

    switch (delayMode) {
        /* using Timer_A, check for opportunity to transition to WDT */
        case 0:
            if ( (milliseconds >= 250) && (milliseconds % 250) == 0) {
                delayMode = 1;
                switchToWatchdogTimer();
            }
            else {
                delayMode = 2;
                switchToTimerA();
            }
            break;
        /* using WDT, check for need to transition to Timer_A */
        case 1:
            if ( (milliseconds >= 250) && (milliseconds % 250) == 0) {
                /* stay in mode 1 */
            }
            else {
                /* switch to Timer_A and never look back */
                delayMode = 2;
                switchToTimerA();
            }
            break;
        /* always using Timer_A */
        case 2:
            break;
    }

    /* timeout is always in milliseconds so that Clock_workFunc() behaves properly */
    Task_sleep(milliseconds); 
}

/* 
 *  ======== setDelayResolution ========
 *
 *  For now, only two resolutions are supported: 1ms and 250ms
 */
 void setDelayResolution(uint32_t milliseconds)
{
    if (milliseconds == 250) {
        if ((delayMode == 0) || (delayMode == 2)) {
            switchToWatchdogTimer();
            delayMode = 1;
        }
    }
    else {
        if (delayMode == 1) {
            switchToTimerA();
            delayMode = 2;
        }
    }
}
