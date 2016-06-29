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
 *  @file       PowerCC3200.h
 *
 *  @brief      Power manager interface for the CC3200
 *
 *  The Power header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/Power.h>
 *  #include <ti/drivers/power/PowerCC3200.h>
 *  @endcode
 *
 *  Refer to @ref Power.h for a complete description of APIs.
 *
 *  ## Implementation #
 *  This module defines the power resources, constraints, events, sleep
 *  states and transition latency times for CC3200.
 *
 *  A default power policy is provided which can transition the MCU from the
 *  active state to one of two sleep states (LPDS or Sleep).
 *  The default policy looks at the amount of idle time remaining to determine
 *  which sleep state it can transition to.  It first checks for a tranistion
 *  into LPDS.  If it cannot transition into LPDS (e.g., not enough time, or
 *  a constraint has been set prohibiting LPDS), it goes into Sleep mode.
 *
 *  The function 'Power_idleFunc' needs to be added to the application's idle
 *  loop.
 *
 *  For a TI-RTOS application, the Clock module must be using the RTC timer.
 *  This timer can be selected using the following statements in the
 *  application configuration (*.cfg) file:
 *
 *  @code
 *  var Clock = xdc.useModule('ti.sysbios.knl.Clock');
 *  Clock.TimerProxy = xdc.useModule("ti.sysbios.family.arm.cc32xx.Timer");
 *  Clock.tickMode = Clock.TickMode_DYNAMIC;
 *
 *  var Idle = xdc.useModule('ti.sysbios.knl.Idle');
 *  Idle.addFunc('&Power_idleFunc');
 *  @endcode
 *
 *  ============================================================================
 */

#ifndef ti_drivers_power_PowerCC3200__include
#define ti_drivers_power_PowerCC3200__include

#include <stdint.h>
#include <ti/drivers/utils/List.h>
#include <ti/drivers/Power.h>

/* driverlib header files */
#include <inc/hw_types.h>
#include <driverlib/pin.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>

#ifdef __cplusplus
extern "C" {
#endif

/* values were gathered from a logic analyzer and rounded up */
/*! latency time for resuming from LPDS in microseconds */
#define PowerCC3200_RESUMETIMELPDS          2500

/*! total latency time for LPDS in microseconds */
#define PowerCC3200_TOTALTIMELPDS           20000

/*! total latency time for Shutdown in microseconds */
#define PowerCC3200_TOTALTIMESHUTDOWN       500000

/* Power resources */
#define PowerCC3200_PERIPH_CAMERA       0
#define PowerCC3200_PERIPH_I2S          1
#define PowerCC3200_PERIPH_SDHOST       2
#define PowerCC3200_PERIPH_GSPI         3
#define PowerCC3200_PERIPH_LSPI         4
#define PowerCC3200_PERIPH_UDMA         5
#define PowerCC3200_PERIPH_GPIOA0       6
#define PowerCC3200_PERIPH_GPIOA1       7
#define PowerCC3200_PERIPH_GPIOA2       8
#define PowerCC3200_PERIPH_GPIOA3       9
#define PowerCC3200_PERIPH_GPIOA4       10
#define PowerCC3200_PERIPH_WDT          11
#define PowerCC3200_PERIPH_UARTA0       12
#define PowerCC3200_PERIPH_UARTA1       13
#define PowerCC3200_PERIPH_TIMERA0      14
#define PowerCC3200_PERIPH_TIMERA1      15
#define PowerCC3200_PERIPH_TIMERA2      16
#define PowerCC3200_PERIPH_TIMERA3      17
#define PowerCC3200_PERIPH_DTHE         18
#define PowerCC3200_PERIPH_SSPI         19
#define PowerCC3200_PERIPH_I2CA0        20
#define PowerCC3200_NUMRESOURCES        21

/*
 *  Power constraints on the CC3200 device
 */
#define PowerCC3200_DISALLOW_LPDS       0   /*!< disallow entering LPDS */
#define PowerCC3200_DISALLOW_SHUTDOWN   1   /*!< disallow shutdown */
#define PowerCC3200_NUMCONSTRAINTS      2   /*!< number of constraints */

/*
 *  Power events on the CC3200 device
 *
 *  Each event must be a power of two and must be sequential
 *  without any gaps.
 */
#define PowerCC3200_ENTERING_LPDS       0x1  /*!< entering LPDS event */
#define PowerCC3200_ENTERING_SHUTDOWN   0x2  /*!< entering Shutdown event */
#define PowerCC3200_AWAKE_LPDS          0x4  /*!< awake from LPDS event */
#define PowerCC3200_NUMEVENTS           3    /*!< number of events */

/* Power sleep states */
#define PowerCC3200_LPDS                0x1 /*!< LPDS state */

/* Use by NVIC Register structure */
#define PowerCC3200_numNVICSetEnableRegs    6
#define PowerCC3200_numNVICIntPriority      49

/* Number of pins that can be parked in LPDS */
#define PowerCC3200_NUMPINS             34

/*! @brief  Used to specify parking of a pin during LPDS */
typedef struct PowerCC3200_ParkInfo {
    uint32_t pin;
    uint32_t parkState;
} PowerCC3200_ParkInfo;

/*! @brief Power global configuration */
typedef struct PowerCC3200_ConfigV1 {
    /*! Init function for power policy */
    Power_PolicyInitFxn policyInitFxn;
    /*! The power policy function */
    Power_PolicyFxn policyFxn;
    /*!
     *  @brief  Hook function called before entering LPDS
     *
     *  This function is called after any notifications are complete,
     *  and before any pins are parked, just before entry to LPDS.
     */
    void (*enterLPDSHookFxn)(void);
    /*!
     *  @brief  Hook function called when resuming from LPDS
     *
     *  This function is called early in the wake sequence, before any
     *  notification functions are run.
     */
    void (*resumeLPDSHookFxn)(void);
    /*! Determines whether to run the power policy function */
    bool enablePolicy;
    /*! Enable GPIO as a wakeup source for LPDS */
    bool enableGPIOWakeupLPDS;
    /*! Enable GPIO as a wakeup source for shutdown */
    bool enableGPIOWakeupShutdown;
    /*! Enable Network activity as a wakeup source for LPDS */
    bool enableNetworkWakeupLPDS;
    /*!
     *  @brief  The GPIO source for wakeup from LPDS
     *
     *  Only one of the following GPIOs {2,4,11,13,17,24,26} can
     *  be the wakeup source for LPDS. The following values are
     *  taken from the driverlib prcm.h file.
     *
     *  Value can be one of the following (defined in driverlib/prcm.h):
     *  PRCM_LPDS_GPIO2, PRCM_LPDS_GPIO4, PRCM_LPDS_GPIO11,
     *  PRCM_LPDS_GPIO13, PRCM_LPDS_GPIO17, PRCM_LPDS_GPIO24,
     *  PRCM_LPDS_GPIO26
     */
    uint32_t wakeupGPIOSourceLPDS;
    /*!
     *  @brief  The GPIO trigger type for wakeup from LPDS
     *
     *  Value can be one of the following (defined in driverlib/prcm.h):
     *  PRCM_LPDS_LOW_LEVEL, PRCM_LPDS_HIGH_LEVEL,
     *  PRCM_LPDS_FALL_EDGE, PRCM_LPDS_RISE_EDGE
     */
    uint32_t wakeupGPIOTypeLPDS;
    /*!
     *  @brief  Function to be called when the configured GPIO triggers wakeup
     *  from LPDS
     *
     *  During LPDS the internal GPIO module is powered off, and special
     *  periphery logic is used instead to detect the trigger and wake the
     *  device.  No GPIO interrupt service routine will be triggered in this
     *  case (even if an ISR is configured, and used normally to detect GPIO
     *  interrupts when not in LPDS).  This function can be used in lieu of a
     *  GPIO ISR, to take specific action upon LPDS wakeup.
     *
     *  A value of NULL indicates no GPIO wakeup function will be called.
     *
     *  An argument for this wakeup function can be specified via
     *  wakeupGPIOFxnLPDSArg.
     *
     *  Note that this wakeup function will be called as one of the last steps
     *  in Power_sleep(), after all notifications have been sent out, and after
     *  pins have been restored to their previous (non-parked) states.
     */
    void (*wakeupGPIOFxnLPDS)(unsigned int argument);
    /*!
     *  @brief  The argument to be passed to wakeupGPIOFxnLPDS()
     */
    unsigned int wakeupGPIOFxnLPDSArg;
    /*!
     *  @brief  The GPIO sources for wakeup from shutdown
     *
     *  Any one of the following GPIOs {2,4,11,13,17,24,26} can
     *  be a wakeup source for shutdown. The following values are
     *  taken from the driverlib prcm.h file
     *
     *  Value can be one or more of the following (defined in driverlib/prcm.h):
     *  PRCM_HIB_GPIO2, PRCM_HIB_GPIO4, PRCM_HIB_GPIO11,
     *  PRCM_HIB_GPIO13, PRCM_HIB_GPIO17, PRCM_HIB_GPIO24,
     *  PRCM_HIB_GPIO26
     */
    uint32_t wakeupGPIOSourceShutdown;
    /*!
     *  @brief  The GPIO trigger type for wakeup from shutdown
     *
     *  Value can be one of the following (defined in driverlib/prcm.h):
     *  PRCM_HIB_LOW_LEVEL, PRCM_HIB_HIGH_LEVEL,
     *  PRCM_HIB_FALL_EDGE, PRCM_HIB_RISE_EDGE
     */
    uint32_t wakeupGPIOTypeShutdown;
    /*!
     *  @brief  SRAM retention mask for LPDS
     *
     *  Value can be a mask of the following (defined in driverlib/prcm.h):
     *  PRCM_SRAM_COL_1, PRCM_SRAM_COL_2, PRCM_SRAM_COL_3,
     *  PRCM_SRAM_COL_4
     */
    uint32_t ramRetentionMaskLPDS;
    /*!
     *  @brief  Keep debug interface active during LPDS
     *
     *  This Boolean controls whether the debug interface will be left active
     *  when LPDS is entered.  For best power savings this flag should be set
     *  to false.  Setting the flag to true will enable better debug
     *  capability, but will prevent full LPDS, and will result in increased
     *  power consumption.
     */
    bool keepDebugActiveDuringLPDS;
    /*!
     *  @brief  IO retention mask for Shutdown
     *
     *  Value can be a mask of the following (defined in driverlib/prcm.h):
     *  PRCM_IO_RET_GRP_0, PRCM_IO_RET_GRP_1, PRCM_IO_RET_GRP_2
     *  PRCM_IO_RET_GRP_3
     */
    uint32_t ioRetentionShutdown;
    /*!
     *  @brief  Pointer to an array of pins to be parked during LPDS
     *
     *  A value of NULL will disable parking of any pins during LPDS
     */
    PowerCC3200_ParkInfo * pinParkDefs;
    /*!
     *  @brief  Number of pins to be parked during LPDS
     */
    uint32_t numPins;
} PowerCC3200_ConfigV1;

/*! @brief Internal module state */
typedef struct PowerCC3200_ModuleState {
    List_List notifyList;
    uint32_t constraintMask;
    uint32_t state;
    uint16_t dbRecords[PowerCC3200_NUMRESOURCES];
    bool enablePolicy;
    bool initialized;
    uint8_t refCount[PowerCC3200_NUMRESOURCES];
    uint8_t constraintCounts[PowerCC3200_NUMCONSTRAINTS];
    Power_PolicyFxn policyFxn;
    uint32_t pinType[PowerCC3200_NUMPINS];
    uint16_t pinDir[PowerCC3200_NUMPINS];
    uint8_t pinMode[PowerCC3200_NUMPINS];
    uint16_t stateAntPin29;
    uint16_t stateAntPin30;
    uint32_t pinLockMask;
    void (*wakeupGPIOFxnLPDS)(unsigned int);
    unsigned int wakeupGPIOFxnLPDSArg;
} PowerCC3200_ModuleState;

/*! @brief  NVIC registers that need to be save on entering LPDS */
typedef struct PowerCC3200_NVICRegisters {
    uint32_t vectorTable;
    uint32_t auxCtrl;
    uint32_t intCtrlState;
    uint32_t appInt;
    uint32_t sysCtrl;
    uint32_t configCtrl;
    uint32_t sysPri1;
    uint32_t sysPri2;
    uint32_t sysPri3;
    uint32_t sysHcrs;
    uint32_t systickCtrl;
    uint32_t systickReload;
    uint32_t systickCalib;
    uint32_t intSetEn[PowerCC3200_numNVICSetEnableRegs];
    uint32_t intPriority[PowerCC3200_numNVICIntPriority];
} PowerCC3200_NVICRegisters;

/*! @brief  MCU core registers that need to be save on entering LPDS */
typedef struct PowerCC3200_MCURegisters {
    uint32_t msp;
    uint32_t psp;
    uint32_t psr;
    uint32_t primask;
    uint32_t faultmask;
    uint32_t basepri;
    uint32_t control;
} PowerCC3200_MCURegisters;

/*! @brief  struct of context registers to save on entering LPDS */
typedef struct PowerCC3200_SaveRegisters {
    PowerCC3200_MCURegisters m4Regs;
    PowerCC3200_NVICRegisters nvicRegs;
} PowerCC3200_SaveRegisters;

/*! @brief enumeration of state for parked pins */
typedef enum {
    PowerCC3200_NO_PULL_HIZ = PIN_TYPE_STD,
    PowerCC3200_WEAK_PULL_UP_STD = PIN_TYPE_STD_PU,
    PowerCC3200_WEAK_PULL_DOWN_STD = PIN_TYPE_STD_PD,
    PowerCC3200_WEAK_PULL_UP_OPENDRAIN = PIN_TYPE_OD_PU,
    PowerCC3200_WEAK_PULL_DOWN_OPENDRAIN = PIN_TYPE_OD_PD,
    PowerCC3200_DRIVE_LOW,
    PowerCC3200_DRIVE_HIGH,
    PowerCC3200_DONT_PARK
} PowerCC3200_ParkState;

/*! @brief enumeration of pins that can be parked */
typedef enum {
    PowerCC3200_PIN01 = PIN_01,
    PowerCC3200_PIN02 = PIN_02,
    PowerCC3200_PIN03 = PIN_03,
    PowerCC3200_PIN04 = PIN_04,
    PowerCC3200_PIN05 = PIN_05,
    PowerCC3200_PIN06 = PIN_06,
    PowerCC3200_PIN07 = PIN_07,
    PowerCC3200_PIN08 = PIN_08,
    PowerCC3200_PIN11 = PIN_11,
    PowerCC3200_PIN12 = PIN_12,
    PowerCC3200_PIN13 = PIN_13,
    PowerCC3200_PIN14 = PIN_14,
    PowerCC3200_PIN15 = PIN_15,
    PowerCC3200_PIN16 = PIN_16,
    PowerCC3200_PIN17 = PIN_17,
    PowerCC3200_PIN18 = PIN_18,
    PowerCC3200_PIN19 = PIN_19,
    PowerCC3200_PIN20 = PIN_20,
    PowerCC3200_PIN21 = PIN_21,
    PowerCC3200_PIN29 = 0x1C,
    PowerCC3200_PIN30 = 0x1D,
    PowerCC3200_PIN45 = PIN_45,
    PowerCC3200_PIN50 = PIN_50,
    PowerCC3200_PIN52 = PIN_52,
    PowerCC3200_PIN53 = PIN_53,
    PowerCC3200_PIN55 = PIN_55,
    PowerCC3200_PIN57 = PIN_57,
    PowerCC3200_PIN58 = PIN_58,
    PowerCC3200_PIN59 = PIN_59,
    PowerCC3200_PIN60 = PIN_60,
    PowerCC3200_PIN61 = PIN_61,
    PowerCC3200_PIN62 = PIN_62,
    PowerCC3200_PIN63 = PIN_63,
    PowerCC3200_PIN64 = PIN_64
} PowerCC3200_Pin;

/*! @brief  For wakeup from LPDS or shutdown configuration */
typedef struct PowerCC3200_Wakeup {
    bool enableGPIOWakeupLPDS;
    bool enableGPIOWakeupShutdown;
    bool enableNetworkWakeupLPDS;
    uint32_t wakeupGPIOSourceLPDS;
    uint32_t wakeupGPIOTypeLPDS;
    void (*wakeupGPIOFxnLPDS)();
    unsigned int wakeupGPIOFxnLPDSArg;
    uint32_t wakeupGPIOSourceShutdown;
    uint32_t wakeupGPIOTypeShutdown;
} PowerCC3200_Wakeup;

/*!
 *  @brief  Function configures wakeup for LPDS and shutdown
 *
 *  This function allows the app to configure the GPIO source and
 *  type for waking up from LPDS and shutdown and the network host
 *  as a wakeup source for LPDS.  This overwrites any previous
 *  wakeup settings.
 *
 *  @param  wakeup      Settings applied to wakeup configuration
 *
 */
void PowerCC3200_configureWakeup(PowerCC3200_Wakeup *wakeup);

/*! OS specific default power policy init function */
void PowerCC3200_initPolicy(void);

/*! CC3200-specific function to dynamically set the LPDS park state for a pin */
void PowerCC3200_setParkState(PowerCC3200_Pin pin, uint32_t level);

/*!
 *  @brief  Function to disable IO retention and unlock pin groups following
 *  exit from Shutdown.
 *
 *  PowerCC3200_ConfigV1.ioRetentionShutdown can be used to specify locking and
 *  retention of pin groups during Shutdown.  Upon exit from Shutdown, and
 *  when appropriate, an application can call this function, to
 *  correspondingly disable IO retention, and unlock the specified pin groups.
 *
 *  @param  groupFlags     A logical OR of one or more of the following
 *  flags (defined in driverlib/prcm.h):
 *      PRCM_IO_RET_GRP_0 - all pins except sFlash and JTAG interface
 *      PRCM_IO_RET_GRP_1 - sFlash interface pins 11,12,13,14
 *      PRCM_IO_RET_GRP_2 - JTAG TDI and TDO interface pins 16,17
 *      PRCM_IO_RET_GRP_3 - JTAG TCK and TMS interface pins 19,20
 */
void PowerCC3200_disableIORetention(unsigned long groupFlags);

/*! OS specific default power policy function */
void PowerCC3200_sleepPolicy(void);

#define Power_getPerformanceLevel(void)   0
#define Power_setPerformanceLevel(level)  Power_EFAIL

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_power_PowerCC3200__include */
