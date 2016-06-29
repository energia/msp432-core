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
 *  @file       PWM.h
 *
 *  @brief      PWM driver interface
 *
 *  The PWM header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/PWM.h>
 *  @endcode
 *
 *  ## Operation #
 *
 *  The PWM driver in TI-RTOS facilitates the generation of Pulse Width
 *  Modulated signals via simple and portable APIs.  PWM instances must be
 *  opened by calling PWM_open() while passing in a PWM index and a parameters
 *  data structure.
 *
 *  The driver APIs serve as an interface to a typical TI-RTOS application.
 *  The specific peripheral implementations are responsible of creating all OS
 *  specific primitives to allow for thread-safe operation.
 *
 *  When a PWM instance is opened the period, duty cycle and idle level are
 *  configured and the PWM is stopped (waveforms not generated until PWM_start()
 *  is called).  The maximum period and duty supported is device dependent;
 *  refer to the implementation specific documentation for values.
 *
 *  PWM outputs are active-high, meaning the duty will control the duration of
 *  high output on the pin (at 0% duty output is always low, at 100% duty is
 *  always high).
 *
 *  A PWM instance can be configured to interpret the period as one of three
 *  units:
 *      - PWM_PERIOD_US: The period is in microseconds.
 *      - PWM_PERIOD_HZ: The period is in Hertz.
 *      - PWM_PERIOD_COUNTS: The period is in timer counts.
 *
 *  A PWM instance can be configured to interpret the duty as one of three
 *  units:
 *      - PWM_DUTY_US: The duty is in microseconds.
 *      - PWM_DUTY_FRACTION: The duty is in a fractional part of the period where
 *                         0 is 0% and PWM_DUTY_FRACTION_MAX is 100%.
 *      - PWM_DUTY_COUNTS: The period is in timer counts and must less than
 *                       the period.
 *
 *  The idle level parameter is used to set the output to high/low when the
 *  PWM is not running (stopped or not started).  The idle level can be
 *  set to:
 *      - PWM_IDLE_LOW
 *      - PWM_IDLE_HIGH
 *
 *  The default PWM configuration is to set a duty of 0% with a 1MHz frequency.
 *  The default period units are in PWM_PERIOD_HZ and the default duty units
 *  are in PWM_DUTY_FRACTION.  Finally, the default output idle level is
 *  PWM_IDLE_LOW.  It is the application's responsibility to set the duty for
 *  each PWM output used.
 *
 *  ## Opening the driver #
 *
 *  @code
 *  PWM_Handle handle;
 *  PWM_Params params;
 *
 *  PWM_init();
 *
 *  PWM_Params_init(&params);
 *  params.idleLevel = PWM_IDLE_LOW;      // Output low when PWM is not running
 *  params.period.unit = PWM_PERIOD_HZ;   // Period is in Hz
 *  params.period.value = 1e6;            // 1MHz
 *  params.duty.unit = PWM_DUTY_FRACTION; // Duty is fraction of period
 *  params.duty.value = 0;                // 0% duty cycle
 *
 *  handle = PWM_open(Board_PWM0, &params);
 *  if (handle == NULL) {
 *      System_printf("PWM did not open");
 *      Task_exit();
 *  }
 *  PWM_start(handle);
 *
 *  @endcode
 *
 *  ## Implementation #
 *
 *  This module serves as the main interface for TI-RTOS applications. Its
 *  purpose is to redirect the module APIs to specific peripheral
 *  implementations specified by a pointer to a PWM_FxnTable.
 *
 *  The PWM driver interface module is joined (at link time) to a
 *  NULL-terminated array of PWM_Config data structures named *PWM_config*.
 *  *PWM_config* is implemented in the application with each entry being an
 *  PWM instance. Each entry in *PWM_config* contains a:
 *  - (PWM_FxnTable *) to a set of functions that implement a PWM peripheral
 *  - (void *) data object that is associated with the device implementation
 *  - (void *) hardware attributes that are associated with the device implementation
 *
 *  # Instrumentation #
 *
 *  The PWM driver interface produces log statements if instrumentation is
 *  enabled.
 *
 *  Diagnostics Mask | Log details |
 *  ---------------- | ----------- |
 *  Diags_USER1      | basic operations performed |
 *  Diags_USER2      | detailed operations performed |
 *
 *  ============================================================================
 */

#ifndef ti_drivers_PWM__include
#define ti_drivers_PWM__include

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*!
 *  @brief Maximum duty (100%) when configuring duty cycle as a fraction of
 *  period.
 */
#define PWM_DUTY_FRACTION_MAX        ((uint32_t) ~0)

/*!
 * Common PWM_control command code reservation offset.
 * PWM driver implementations should offset command codes with PWM_CMD_RESERVED
 * growing positively.
 *
 * Example implementation specific command codes:
 * @code
 * #define PWMXYZ_COMMAND0         (PWM_CMD_RESERVED + 0)
 * #define PWMXYZ_COMMAND1         (PWM_CMD_RESERVED + 1)
 * @endcode
 */
#define PWM_CMD_RESERVED             (32)

/*!
 * Common PWM_control status code reservation offset.
 * PWM driver implementations should offset status codes with
 * PWM_STATUS_RESERVED growing negatively.
 *
 * Example implementation specific status codes:
 * @code
 * #define PWMXYZ_STATUS_ERROR0    (PWM_STATUS_RESERVED - 0)
 * #define PWMXYZ_STATUS_ERROR1    (PWM_STATUS_RESERVED - 1)
 * #define PWMXYZ_STATUS_ERROR2    (PWM_STATUS_RESERVED - 2)
 * @endcode
 */
#define PWM_STATUS_RESERVED          (-32)

/*!
 * @brief  Success status code returned by:
 * PWM_control(), PWM_setDuty(), PWM_setPeriod().
 *
 * Functions return PWM_STATUS_SUCCESS if the call was executed
 * successfully.
 */
#define PWM_STATUS_SUCCESS           (0)

/*!
 * @brief   Generic error status code returned by PWM_control().
 *
 * PWM_control() returns PWM_STATUS_ERROR if the control code was not executed
 * successfully.
 */
#define PWM_STATUS_ERROR             (-1)

/*!
 * @brief   An error status code returned by PWM_control() for undefined
 * command codes.
 *
 * PWM_control() returns PWM_STATUS_UNDEFINEDCMD if the control code is not
 * recognized by the driver implementation.
 */
#define PWM_STATUS_UNDEFINEDCMD      (-2)

/*!
 * @brief   An error status code returned by PWM_setPeriod().
 *
 * PWM_setPeriod() returns PWM_STATUS_INVALID_PERIOD if the period argument is
 * invalid for the current configuration.
 */
#define PWM_STATUS_INVALID_PERIOD    (-3)

/*!
 * @brief   An error status code returned by PWM_setDuty().
 *
 * PWM_setDuty() returns PWM_STATUS_INVALID_DUTY if the duty cycle argument is
 * invalid for the current configuration.
 */
#define PWM_STATUS_INVALID_DUTY      (-4)

/*!
 *  @brief   PWM period unit definitions.  Refer to device specific
 *  implementation if using PWM_PERIOD_COUNTS (raw PWM/Timer counts).
 */
typedef enum PWM_Period_Units {
    PWM_PERIOD_US,    /* Period in microseconds */
    PWM_PERIOD_HZ,    /* Period in Hertz (for example 2MHz = 0.5us period) */
    PWM_PERIOD_COUNTS /* Period in timer counts */
} PWM_Period_Units;

/*!
 *  @brief   PWM duty cycle unit definitions.  Refer to device specific
 *  implementation if using PWM_DUTY_COUNTS (raw PWM/Timer counts).
 */
typedef enum PWM_Duty_Units {
    PWM_DUTY_US,       /* Duty cycle in microseconds */
    PWM_DUTY_FRACTION, /* Duty as a fractional part of PWM_DUTY_FRACTION_MAX */
    PWM_DUTY_COUNTS    /* Duty in timer counts  */
} PWM_Duty_Units;

/*!
 *  @brief   Idle output level when PWM is not running (stopped / not started).
 */
typedef enum PWM_IdleLevel {
    PWM_IDLE_LOW  = 0,
    PWM_IDLE_HIGH = 1,
} PWM_IdleLevel;

/*!
 *  @brief PWM Parameters
 *
 *  PWM Parameters are used to with the PWM_open() call. Default values for
 *  these parameters are set using PWM_Params_init().
 *
 *  @sa     PWM_Params_init()
 */
typedef struct PWM_Params {
    PWM_Period_Units periodUnits; /*!< Units in which the period is specified */
    uint32_t         periodValue; /*!< PWM initial period */
    PWM_Duty_Units   dutyUnits;   /*!< Units in which the duty is specified */
    uint32_t         dutyValue;   /*!< PWM initial duty */
    PWM_IdleLevel    idleLevel;   /*!< Pin output when PWM is stopped. */
    uintptr_t        custom;      /*!< Custom argument used by driver
                                       implementation */
} PWM_Params;

/*!
 *  @brief      A handle that is returned from a PWM_open() call.
 */
typedef struct PWM_Config *PWM_Handle;

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              PWM_close().
 */
typedef void       (*PWM_CloseFxn)     (PWM_Handle handle);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              PWM_control().
 */
typedef int        (*PWM_ControlFxn)   (PWM_Handle handle,
                                        unsigned int cmd,
                                        void *arg);
/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              PWM_init().
 */
typedef void       (*PWM_InitFxn)      (PWM_Handle handle);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              PWM_open().
 */
typedef PWM_Handle (*PWM_OpenFxn)      (PWM_Handle handle,
                                        PWM_Params *params);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              PWM_setDuty().
 */
typedef int        (*PWM_SetDutyFxn)   (PWM_Handle handle,
                                        uint32_t dutyValue);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              PWM_setPeriod().
 */
typedef int        (*PWM_SetPeriodFxn) (PWM_Handle handle,
                                        uint32_t periodValue);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              PWM_start().
 */
typedef void       (*PWM_StartFxn)     (PWM_Handle handle);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              PWM_stop().
 */
typedef void       (*PWM_StopFxn)      (PWM_Handle handle);

/*!
 *  @brief      The definition of a PWM function table that contains the
 *              required set of functions to control a specific PWM driver
 *              implementation.
 */
typedef struct PWM_FxnTable {
    /*! Function to close the specified instance */
    PWM_CloseFxn     closeFxn;
    /*! Function to driver implementation specific control function */
    PWM_ControlFxn   controlFxn;
    /*! Function to initialize the given data object */
    PWM_InitFxn      initFxn;
    /*! Function to open the specified instance */
    PWM_OpenFxn      openFxn;
    /*! Function to set the duty cycle for a specific instance */
    PWM_SetDutyFxn   setDutyFxn;
    /*! Function to set the period for a specific instance */
    PWM_SetPeriodFxn setPeriodFxn;
    /*! Function to start the PWM output for a specific instance */
    PWM_StartFxn     startFxn;
    /*! Function to stop the PWM output for a specific instance */
    PWM_StopFxn      stopFxn;
} PWM_FxnTable;

/*!
 *  @brief  PWM Global configuration.
 *
 *  The PWM_Config structure contains a set of pointers used to characterize
 *  the PWM driver implementation.
 *
 */
typedef struct PWM_Config {
    /*! Pointer to a table of driver-specific implementations of PWM APIs */
    PWM_FxnTable const *fxnTablePtr;
    /*! Pointer to a driver specific data object */
    void               *object;
    /*! Pointer to a driver specific hardware attributes structure */
    void         const *hwAttrs;
} PWM_Config;

/*!
 *  @brief  Function to close a PWM instance specified by the PWM handle.
 *
 *  @pre    PWM_open() must have been called first.
 *  @pre    PWM_stop() must have been called first if PWM was started.
 *
 *  @param  handle A PWM handle returned from PWM_open().
 *
 *  @sa     PWM_open()
 *  @sa     PWM_start()
 *  @sa     PWM_stop()
 */
extern void PWM_close(PWM_Handle handle);

/*!
 *  @brief  Function performs implementation specific features on a given
 *          PWM_Handle.
 *
 *  @pre    PWM_open() must have been called first.
 *
 *  @param  handle      A PWM handle returned from PWM_open().
 *
 *  @param  cmd         A command value defined by the driver specific
 *                      implementation.
 *
 *  @param  arg         A pointer to an optional R/W (read/write) argument that
 *                      is accompanied with cmd.
 *
 *  @return A PWM_Status describing an error or success state. Negative values
 *          indicate an error occurred.
 *
 *  @sa     PWM_open()
 */
extern int PWM_control(PWM_Handle handle, unsigned int cmd, void *arg);

/*!
 *  @brief  This function initializes the PWM module.
 *
 *  @pre    The PWM_config structure must exist and be persistent before this
 *          function can be called. This function must be called before any
 *          other PWM driver APIs.  This function does not modify any peripheral
 *          registers & should only be called once.
 */
extern void PWM_init(void);

/*!
 *  @brief  This function opens a given PWM instance and sets the period,
 *          duty and idle level to those specified in the params argument.
 *
 *  @param  index         Logical instance number for the PWM indexed into
 *                        the PWM_config table.
 *
 *  @param  params        Pointer to an parameter structure.  If NULL default
 *                        values are used.
 *
 *  @return A PWM_Handle if successful or NULL on an error or if it has been
 *          opened already. If NULL is returned further PWM API calls will
 *          result in undefined behavior.
 *
 *  @sa     PWM_close()
 */
extern PWM_Handle PWM_open(unsigned int index, PWM_Params *params);

/*!
 *  @brief  Function to initialize the PWM_Params structure to default values.
 *
 *  @param  params      A pointer to PWM_Params structure for initialization.
 *
 *  Defaults values are:
 *      Period units: PWM_PERIOD_HZ
 *      Period: 1e6  (1MHz)
 *      Duty cycle units: PWM_DUTY_FRACTION
 *      Duty cycle: 0%
 *      Idle level: PWM_IDLE_LOW
 */
extern void PWM_Params_init(PWM_Params *params);

/*!
 *  @brief  Function to set the duty cycle of the specified PWM handle.  PWM
 *          instances run in active high output mode; 0% is always low output,
 *          100% is always high output.  This API can be called while the PWM
 *          is running & duty must always be lower than or equal to the period.
 *          If an error occurs while calling the function the PWM duty cycle
 *          will remain unchanged.
 *
 *  @pre    PWM_open() must have been called first.
 *
 *  @param  handle      A PWM handle returned from PWM_open().
 *
 *  @param  duty        Duty cycle in the units specified by the params used
 *                      in PWM_open().
 *
 *  @return A PWM status describing an error or success. Negative values
 *          indicate an error.
 *
 *  @sa     PWM_open()
 */
extern int PWM_setDuty(PWM_Handle handle, uint32_t duty);

/*!
 *  @brief  Function to set the period of the specified PWM handle. This API
 *          can be called while the PWM is running & the period must always be
 *          larger than the duty cycle.
 *          If an error occurs while calling the function the PWM period
 *          will remain unchanged.
 *
 *  @pre    PWM_open() must have been called first.
 *
 *  @param  handle      A PWM handle returned from PWM_open().
 *
 *  @param  period      Period in the units specified by the params used
 *                      in PWM_open().
 *
 *  @return A PWM status describing an error or success state. Negative values
 *          indicate an error.
 *
 *  @sa     PWM_open()
 */
extern int PWM_setPeriod(PWM_Handle handle, uint32_t period);

/*!
 *  @brief  Function to start the specified PWM handle with current settings.
 *
 *  @pre    PWM_open() has to have been called first.
 *
 *  @param  handle      A PWM handle returned from PWM_open().
 *
 *  @sa     PWM_open()
 *  @sa     PWM_stop()
 */
extern void PWM_start(PWM_Handle handle);

/*!
 *  @brief  Function to stop the specified PWM handle. Output will set to the
 *          idle level specified by params in PWM_open().
 *
 *  @pre    PWM_open() has to have been called first.
 *
 *  @param  handle      A PWM handle returned from PWM_open().
 *
 *  @sa     PWM_open()
 *  @sa     PWM_start()
 */
extern void PWM_stop(PWM_Handle handle);

#ifdef __cplusplus
}
#endif
#endif /* ti_drivers_PWM__include */
