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
 * EXEMPLARY, OR CONSEQueueNTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ============================================================================
 *  @file       PowerCC26XX.h
 *
 *  @brief      Power manager interface for CC26XX
 *
 *  The Power header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/Power.h>
 *  #include <ti/drivers/power/PowerCC26XX.h>
 *  @endcode
 *
 *  Refer to @ref Power.h for a complete description of APIs.
 *
 *  ## Implementation #
 *  This module defines the power resources, constraints, events, sleep
 *  states and transition latency times for CC26XX.
 *
 *  A default power policy is provided which can transition the MCU from the
 *  active state to one of the supported power saving states.
 *
 *  ============================================================================
 */

#ifndef ti_drivers_power_PowerCC26XX_
#define ti_drivers_power_PowerCC26XX_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/Power.h>

typedef uint8_t PowerCC26XX_Resource;

/* latency to resume from STANDBY (usec) */
#define PowerCC26XX_RESUMETIMESTANDBY  750

/* total latency for STANDBY (usec) */
#define PowerCC26XX_TOTALTIMESTANDBY   1000

/* initial wake delay from STANDBY (usec) */
#define PowerCC26XX_WAKEDELAYSTANDBY    130

/* initial wait time (usec) to see if RCOSC_LF is stable */
#define PowerCC26XX_INITIALWAITRCOSC_LF 1000

/* retry wait time (usec) to see if RCOSC_LF is stable */
#define PowerCC26XX_RETRYWAITRCOSC_LF   1000

/* initial wait time (usec) to see if XOSC_HF is stable */
#define PowerCC26XX_INITIALWAITXOSC_HF  50

/* retry wait time (usec) to see if XOSC_HF is stabe */
#define PowerCC26XX_RETRYWAITXOSC_HF    50

/* initial wait time (usec) to see if XOSC_LF is stable */
#define PowerCC26XX_INITIALWAITXOSC_LF  10000

/* retry wait time (usec) to see if XOSC_LF is stable */
#define PowerCC26XX_RETRYWAITXOSC_LF    5000

/* resource IDs */
#define PowerCC26XX_PERIPH_GPT0     0
#define PowerCC26XX_PERIPH_GPT1     1
#define PowerCC26XX_PERIPH_GPT2     2
#define PowerCC26XX_PERIPH_GPT3     3
#define PowerCC26XX_PERIPH_SSI0     4
#define PowerCC26XX_PERIPH_SSI1     5
#define PowerCC26XX_PERIPH_UART0    6
#define PowerCC26XX_PERIPH_I2C0     7
#define PowerCC26XX_PERIPH_TRNG     8
#define PowerCC26XX_PERIPH_GPIO     9
#define PowerCC26XX_PERIPH_UDMA     10
#define PowerCC26XX_PERIPH_CRYPTO   11
#define PowerCC26XX_PERIPH_I2S      12
#define PowerCC26XX_PERIPH_RFCORE   13
#define PowerCC26XX_XOSC_HF         14
#define PowerCC26XX_DOMAIN_PERIPH   15
#define PowerCC26XX_DOMAIN_SERIAL   16
#define PowerCC26XX_DOMAIN_RFCORE   17
#define PowerCC26XX_DOMAIN_SYSBUS   18
#define PowerCC26XX_NUMRESOURCES    19

/* resource is a peripheral */
#define PowerCC26XX_PERIPH          0x80

/* resource requires special handler */
#define PowerCC26XX_SPECIAL         0x40

 /* resource is a domain */
#define PowerCC26XX_DOMAIN          0x00

/* parent resource mask */
#define PowerCC26XX_PARENTMASK      0x3F

/* if resource has no parent */
#define PowerCC26XX_NOPARENT        0x3F

/* standby power state */
#define PowerCC26XX_STANDBY         0x1

#define PowerCC26XX_ENABLE          1
#define PowerCC26XX_DISABLE         0

/*
 *  Constraints
 */
#define PowerCC26XX_SB_VIMS_CACHE_RETAIN    0
#define PowerCC26XX_SD_DISALLOW             1
#define PowerCC26XX_SB_DISALLOW             2
#define PowerCC26XX_IDLE_PD_DISALLOW        3
#define PowerCC26XX_NEED_FLASH_IN_IDLE      4
#define PowerCC26XX_NUMCONSTRAINTS          5

/*
 *  Events
 *
 *  Each event must be a power of two and must be sequential
 *  without any gaps.
 */
#define PowerCC26XX_ENTERING_STANDBY    0x1
#define PowerCC26XX_ENTERING_SHUTDOWN   0x2
#define PowerCC26XX_AWAKE_STANDBY       0x4
#define PowerCC26XX_AWAKE_STANDBY_LATE  0x8
#define PowerCC26XX_XOSC_HF_SWITCHED    0x10
#define PowerCC26XX_NUMEVENTS           5

/*
 *  Calibration stages
 */
#define PowerCC26XX_SETUP_CALIBRATE     1
#define PowerCC26XX_INITIATE_CALIBRATE  2
#define PowerCC26XX_DO_CALIBRATE        3

/*! @brief Power resource database record format */
typedef struct PowerCC26XX_ResourceRecord {
    uint8_t flags;          /* resource type : first parent */
    uint8_t flags2;         /* second parent */
    uint16_t driverlibID;
} PowerCC26XX_ResourceRecord;


/*! @brief Global configuration structure */
typedef struct PowerCC26XX_Config {
    /*!
     *  @brief The power policy's initialization function
     *
     *  If the policy does not have an initialization function, 'NULL'
     *  should be specified.
     */
    Power_PolicyInitFxn policyInitFxn;
    /*!
     *  @brief The power policy function
     *
     *  When enabled, this function is called in the idle loop, to decide upon
     *  and activate sleep states.  Two reference power policies are provided:
     *
     *  PowerCC26XX_doWFI() - a simple policy that invokes wait for interrupt
     *   (WFI)
     *
     *  PowerCC26XX_standbyPolicy() - an agressive policy that considers
     *  constraints and latencies, and optionally puts the device into
     *  the standby state.
     *
     *  Custom policies can be written, and specified via this function pointer.
     */
    Power_PolicyFxn policyFxn;
    /*!
     *  @brief The function to be used for activating RC Oscillator (RCOSC)
     *  calibration
     *
     *  Calibration is normally enabled, via specification of the function
     *  PowerCC26XX_calibrate().  This enables high accuracy operation, and
     *  faster high frequency crystal oscillator (XOSC_HF) startups.
     *
     *  To disable RCOSC calibration the function PowerCC26XX_noCalibrate()
     *  should be specified.
     */
    bool (*calibrateFxn)(unsigned int);
    /*!
     *  @brief Boolean specifying if the power policy function is enabled
     *
     *  If 'TRUE', the policy function will be invoked once for each pass
     *  of the idle loop.
     *
     *  If 'FALSE', the policy will not run by default. But the policy can be
     *  enabled later at runtime, once the application has started, via
     *  a call to Power_enablePolicy().  Once it has been enabled,
     *  the policy will be invoked once for each pass of the idle loop.
     */
    bool enablePolicy;
    /*!
     *  @brief Boolean specifying whether the low frequency RCOSC
     * (RCOSC_LF) should be calibrated.
     *
     *  If RCOSC calibration is enabled (above, via specification of
     *  an appropriate calibrateFxn), this Boolean specifies whether
     *  RCOSC_LF should be calibrated.
     */
    bool calibrateRCOSC_LF;
    /*!
     *  @brief Boolean specifying whether the high frequency RCOSC
     * (RCOSC_HF) should be calibrated.
     *
     *  If RCOSC calibration is enabled (above, via specification of
     *  an appropriate calibrateFxn), this Boolean specifies whether
     *  RCOSC_HF should be calibrated.
     */
    bool calibrateRCOSC_HF;
} PowerCC26XX_Config;

/*! @brief Internal module state */
typedef struct PowerCC26XX_ModuleState {
    List_List notifyList;        /* event notification list */
    uint32_t constraintMask;     /* constraint mask */
    Clock_Struct clockObj;       /* Clock object for scheduling wakeups */
    Clock_Struct xoscClockObj;   /* Clock object for XOSC_HF switching */
    Clock_Struct lfClockObj;     /* Clock object for LF clock check */
    Clock_Struct calClockStruct; /* Clock object for RCOSC calibration */
    Hwi_Struct hwiStruct;        /* Hwi object for RCOSC calibration */
    int32_t nDeltaFreqCurr;      /* RCOSC calibration variable */
    int32_t nCtrimCurr;          /* RCOSC calibration variable */
    int32_t nCtrimFractCurr;     /* RCOSC calibration variable */
    int32_t nCtrimNew;           /* RCOSC calibration variable */
    int32_t nCtrimFractNew;      /* RCOSC calibration variable */
    int32_t nRtrimNew;           /* RCOSC calibration variable */
    int32_t nRtrimCurr;          /* RCOSC calibration variable */
    int32_t nDeltaFreqNew;       /* RCOSC calibration variable */
    bool bRefine;                /* RCOSC calibration variable */
    uint32_t state;              /* current transition state */
    bool xoscPending;            /* is XOSC_HF activation in progress? */
    bool calLF;                  /* calibrate RCOSC_LF? */
    uint8_t hwiState;            /* calibration AUX ISR state */
    bool busyCal;                /* already busy calibrating */
    uint8_t calStep;             /* current calibration step */
    bool firstLF;                /* is this first LF calibration? */
    bool enablePolicy;           /* enable policy state */
    bool initialized;            /* TRUE if Power_init has been called */
    uint8_t constraintCounts[PowerCC26XX_NUMCONSTRAINTS];
    uint8_t resourceCounts[PowerCC26XX_NUMRESOURCES];
    unsigned int (*resourceHandlers[3])(unsigned int);
    Power_PolicyFxn policyFxn;
} PowerCC26XX_ModuleState;

/*!
 *  @brief  RCOSC calibration
 *
 *  If this function is plugged into the PowerCC26XX_Config structure
 *  'calibrateFxn' field, RCOSC calibration is performed.
 *
 *  @param  arg      used internally
 *
 *  @return used internally
 */
bool PowerCC26XX_calibrate(unsigned int arg);

/*!
 *  @brief  Wait for interrupt
 *
 *  This function executes a Wait-For-Interrupt instruction.
 *  This can be used to set the PowerCC26XX_Config field 'policyFxn'.
 */
void PowerCC26XX_doWFI(void);

/*!
 *  @brief  Get the handle of a Clock object that a power policy can use
 *          to schedule wakeups
 *
 *  @return     The handle of the Clock object
 */
Clock_Handle PowerCC26XX_getClockHandle(void);

/*!
 *  @brief  Get the estimated crystal oscillator startup time in microseconds
 *
 *  @param timeUntilWakeupInMs  The estimated time until the next wakeup
 *                              event, in units of milliseconds
 *
 *  @return     The estimated crystal oscillator startup latency, in
 *              units of microseconds.
 */
uint32_t PowerCC26XX_getXoscStartupTime(uint32_t timeUntilWakeupInMs);

/*!
 *  @brief  Explicitly trigger RCOSC calibration
 *
 *  @return  TRUE if calibration was actually initiated otherwise FALSE
 */
bool PowerCC26XX_injectCalibration(void);

/*!
 *  @brief  Do not perform calibration function
 *
 *  If this function is plugged into the PowerCC26XX_Config structure
 *  'calibrateFxn' field, no calibration is performed.
 *
 *  @param  arg      used internally
 *
 *  @return used internally
 */
bool PowerCC26XX_noCalibrate(unsigned int arg);

/*!
 *  @brief  Switch the HF clock source to XOSC_HF
 */
void PowerCC26XX_switchXOSC_HF(void);

/*!
 *  @brief  Power standby policy function
 *
 *  This policy will transition the device into the PowerCC26XX_STANDBY sleep
 *  state during CPU idle time.  This is an agressive power policy function
 *  that will consider active constraints, sleep transition latencies, and
 *  the next expected wakeup, and automatically transition the device into
 *  the deepest sleep state possible.
 *
 *  The first goal is to enter PowerCC26XX_STANDBY; if that is not appropriate
 *  given current conditions (e.g., the sleep transition latency is greater
 *  greater than the time until the next scheduled Clock event), then
 *  the secondary goal is the IDLE_PD state; if that is disallowed (e.g.,
 *  if the Power_IDLE_PD_DISALLOW constraint is declared), then the policy
 *  will fallback and simply invoke wait-for-interrupt, to clock
 *  gate the CPU until the next interrupt.
 *
 *  In order for this power policy to run, it must be set to the 'policyFxn'
 *  field in the PowerCC26XX_Config structure, and either the 'enablePolicy'
 *  field in the PowerCC26XX_Config structure must be set or
 *  Power_enablePolicy() must be called at runtime.
 */
void PowerCC26XX_standbyPolicy(void);

#ifdef __cplusplus
}
#endif

#endif /* POWER_CC26XX_ */
