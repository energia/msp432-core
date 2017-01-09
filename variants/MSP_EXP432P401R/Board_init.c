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

/*
 *  ======== Board_init.c ========
 *  This file is responsible for setting up the board specific items for the
 *  MSP_EXP432P401R Launch Pad board.
 */

#include <stdbool.h>

#include <xdc/std.h>

#include <xdc/runtime/System.h>

#include <ti/drivers/ports/DebugP.h>
#include <ti/drivers/ports/HwiP.h>

#include <msp432.h>
#include <driverlib/MSP432P4xx/rom.h>
#include <driverlib/MSP432P4xx/rom_map.h>
#include <driverlib/MSP432P4xx/dma.h>
#include <driverlib/MSP432P4xx/interrupt.h>
#include <driverlib/MSP432P4xx/gpio.h>
#include <driverlib/MSP432P4xx/i2c.h>
#include <driverlib/MSP432P4xx/spi.h>
#include <driverlib/MSP432P4xx/timer_a.h>
#include <driverlib/MSP432P4xx/uart.h>
#include <driverlib/MSP432P4xx/wdt_a.h>

#include "Board.h"

/*
 *  =============================== DMA ===============================
 */

#include <ti/drivers/dma/UDMAMSP432.h>


#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(dmaControlTable, 256)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=256
#elif defined(__GNUC__)
__attribute__ ((aligned (256)))
#endif
static DMA_ControlTable dmaControlTable[8];

/*
 *  ======== dmaErrorHwi ========
 *  This is the handler for the uDMA error interrupt.
 */
static void dmaErrorHwi(uintptr_t arg)
{
    int status = MAP_DMA_getErrorStatus();
    MAP_DMA_clearErrorStatus();

    /* Suppress unused variable warning */
    (void)status;

    while (1);
}

UDMAMSP432_Object udmaMSP432Object;

const UDMAMSP432_HWAttrs udmaMSP432HWAttrs = {
    .controlBaseAddr = (void *)dmaControlTable,
    .dmaErrorFxn = (UDMAMSP432_ErrorFxn)dmaErrorHwi,
    .intNum = INT_DMA_ERR,
    .intPriority = (~0)
};

const UDMAMSP432_Config UDMAMSP432_config = {
    .object = &udmaMSP432Object,
    .hwAttrs = &udmaMSP432HWAttrs
};

/*
 *  =============================== General ===============================
 */

/*
 *  ======== Board_initGeneral ========
 */
void Board_initGeneral(void) {
}

/*
 *  =============================== GPIO ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(GPIOMSP432_config, ".const:GPIOMSP432_config")
#pragma DATA_SECTION(gpioPinConfigs, ".data:gpioPinConfigs")
#pragma DATA_SECTION(gpioCallbackFunctions, ".data:gpioCallbackFunctions")
#endif

#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOMSP432.h>

/*
 * Array of Pin configurations
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in MSP_EXP432P401R.h
 * NOTE: Pins not used for interrupts should be placed at the end of the
 *       array.  Callback entries can be omitted from callbacks array to
 *       reduce memory usage.
 */
GPIO_PinConfig gpioPinConfigs[] = {
    /* port_pin */
    GPIOMSP432_EMPTY_PIN | GPIO_DO_NOT_CONFIG,  /*  0  - dummy */

    /* pins 1-10 */
    GPIOMSP432_EMPTY_PIN | GPIO_DO_NOT_CONFIG,  /*  1  - 3.3V */
    GPIOMSP432_P6_0 | GPIO_DO_NOT_CONFIG,       /*  2  - P6.0_A15 */
    GPIOMSP432_P3_2 | GPIO_DO_NOT_CONFIG,       /*  3  - P3.2_URXD */
    GPIOMSP432_P3_3 | GPIO_DO_NOT_CONFIG,       /*  4  - P3.3_UTXD */
    GPIOMSP432_P4_1 | GPIO_DO_NOT_CONFIG,       /*  5  - P4.1_IO_A12 */
    GPIOMSP432_P4_3 | GPIO_DO_NOT_CONFIG,       /*  6  - P4.3_A10 */
    GPIOMSP432_P1_5 | GPIO_DO_NOT_CONFIG,       /*  7  - P1.5_SPICLK */
    GPIOMSP432_P4_6 | GPIO_DO_NOT_CONFIG,       /*  8  - P4.6_IO_A7 */
    GPIOMSP432_P6_5 | GPIO_DO_NOT_CONFIG,       /*  9  - P6.5_I2CSCL */
    GPIOMSP432_P6_4 | GPIO_DO_NOT_CONFIG,       /*  10 - P6.4_I2CSDA */

    /* pins 11-20 */
    GPIOMSP432_P3_6 | GPIO_DO_NOT_CONFIG,       /*  11 - P3.6_IO */
    GPIOMSP432_P5_2 | GPIO_DO_NOT_CONFIG,       /*  12 - P5.2_IO */
    GPIOMSP432_P5_0 | GPIO_DO_NOT_CONFIG,       /*  13 - P5.0_IO */
    GPIOMSP432_P1_7 | GPIO_DO_NOT_CONFIG,       /*  14 - P1.7_SPIMISO */
    GPIOMSP432_P1_6 | GPIO_DO_NOT_CONFIG,       /*  15 - P1.6_SPIMOSI */
    GPIOMSP432_EMPTY_PIN | GPIO_DO_NOT_CONFIG,  /*  16 - RESET */
    GPIOMSP432_P5_7 | GPIO_DO_NOT_CONFIG,       /*  17 - P5.7_IO */
    GPIOMSP432_P3_0 | GPIO_DO_NOT_CONFIG,       /*  18 - P3.0_IO */
    GPIOMSP432_P2_5 | GPIO_DO_NOT_CONFIG,       /*  19 - P2.5_IO_PWM */
    GPIOMSP432_EMPTY_PIN | GPIO_DO_NOT_CONFIG,  /*  20 - GND */

    /* pins 21-30 */
    GPIOMSP432_EMPTY_PIN | GPIO_DO_NOT_CONFIG,  /*  21 - 5V */
    GPIOMSP432_EMPTY_PIN | GPIO_DO_NOT_CONFIG,  /*  22 - GND */
    GPIOMSP432_P6_1 | GPIO_DO_NOT_CONFIG,       /*  23 - P6.1_A14 */
    GPIOMSP432_P4_0 | GPIO_DO_NOT_CONFIG,       /*  24 - P4.0_A13 */
    GPIOMSP432_P4_2 | GPIO_DO_NOT_CONFIG,       /*  25 - P4.2_A11 */
    GPIOMSP432_P4_4 | GPIO_DO_NOT_CONFIG,       /*  26 - P4.4_A9 */
    GPIOMSP432_P4_5 | GPIO_DO_NOT_CONFIG,       /*  27 - P4.5_A8 */
    GPIOMSP432_P4_7 | GPIO_DO_NOT_CONFIG,       /*  28 - P4.7_A6 */
    GPIOMSP432_P5_4 | GPIO_DO_NOT_CONFIG,       /*  29 - P5.4_IO */
    GPIOMSP432_P5_5 | GPIO_DO_NOT_CONFIG,       /*  30 - P5.5_IO */

    /* pins 31-40 */
    GPIOMSP432_P3_7 | GPIO_DO_NOT_CONFIG,       /*  31 - P3.7_IO */
    GPIOMSP432_P3_5 | GPIO_DO_NOT_CONFIG,       /*  32 - P3.5_IO */
    GPIOMSP432_P5_1 | GPIO_DO_NOT_CONFIG,       /*  33 - P5.1_IO */
    GPIOMSP432_P2_3 | GPIO_DO_NOT_CONFIG,       /*  34 - P2.3_IO */
    GPIOMSP432_P6_7 | GPIO_DO_NOT_CONFIG,       /*  35 - P6.7_IO_CAPT */
    GPIOMSP432_P6_6 | GPIO_DO_NOT_CONFIG,       /*  36 - P6.6_IO_CAPT */
    GPIOMSP432_P5_6 | GPIO_DO_NOT_CONFIG,       /*  37 - P5.6_PWM */
    GPIOMSP432_P2_4 | GPIO_DO_NOT_CONFIG,       /*  38 - P2.4_PWM */
    GPIOMSP432_P2_6 | GPIO_DO_NOT_CONFIG,       /*  39 - P2.6_PWM */
    GPIOMSP432_P2_7 | GPIO_DO_NOT_CONFIG,       /*  40 - P2.7_PWM */

    /* bottom row pins 41-56 */
    GPIOMSP432_P8_5 | GPIO_DO_NOT_CONFIG,       /*  41 - P8.5 */
    GPIOMSP432_P9_0 | GPIO_DO_NOT_CONFIG,       /*  42 - P9.0 */
    GPIOMSP432_P8_4 | GPIO_DO_NOT_CONFIG,       /*  43 - P8.4 */
    GPIOMSP432_P8_2 | GPIO_DO_NOT_CONFIG,       /*  44 - P8.2 */
    GPIOMSP432_P9_2 | GPIO_DO_NOT_CONFIG,       /*  45 - P9.2 */
    GPIOMSP432_P6_2 | GPIO_DO_NOT_CONFIG,       /*  46 - P6.2 */
    GPIOMSP432_P7_3 | GPIO_DO_NOT_CONFIG,       /*  47 - P7.3 */
    GPIOMSP432_P7_1 | GPIO_DO_NOT_CONFIG,       /*  48 - P7.1 */
    GPIOMSP432_P9_4 | GPIO_DO_NOT_CONFIG,       /*  49 - P9.4 */
    GPIOMSP432_P9_6 | GPIO_DO_NOT_CONFIG,       /*  40 - P9.6 */
    GPIOMSP432_P8_0 | GPIO_DO_NOT_CONFIG,       /*  51 - P8.0 */
    GPIOMSP432_P7_4 | GPIO_DO_NOT_CONFIG,       /*  52 - P7.4 */
    GPIOMSP432_P7_6 | GPIO_DO_NOT_CONFIG,       /*  53 - P7.6 */
    GPIOMSP432_P10_0 | GPIO_DO_NOT_CONFIG,      /*  54 - P10.0 */
    GPIOMSP432_P10_2 | GPIO_DO_NOT_CONFIG,      /*  55 - P10_2 */
    GPIOMSP432_P10_4 | GPIO_DO_NOT_CONFIG,      /*  56 - P10.4 */

    /* bottom row pins 57-72 */
    GPIOMSP432_P8_6 | GPIO_DO_NOT_CONFIG,       /*  57 - P8.6 */
    GPIOMSP432_P8_7 | GPIO_DO_NOT_CONFIG,       /*  58 - P8.7 */
    GPIOMSP432_P9_1 | GPIO_DO_NOT_CONFIG,       /*  59 - P9.1 */
    GPIOMSP432_P8_3 | GPIO_DO_NOT_CONFIG,       /*  60 - P8.3 */
    GPIOMSP432_P5_3 | GPIO_DO_NOT_CONFIG,       /*  61 - P5.3 */
    GPIOMSP432_P9_3 | GPIO_DO_NOT_CONFIG,       /*  62 - P9.3 */
    GPIOMSP432_P6_3 | GPIO_DO_NOT_CONFIG,       /*  63 - P6.3 */
    GPIOMSP432_P7_2 | GPIO_DO_NOT_CONFIG,       /*  64 - P7.2 */
    GPIOMSP432_P7_0 | GPIO_DO_NOT_CONFIG,       /*  65 - P7.0 */
    GPIOMSP432_P9_5 | GPIO_DO_NOT_CONFIG,       /*  66 - P9.5 */
    GPIOMSP432_P9_7 | GPIO_DO_NOT_CONFIG,       /*  67 - P9.7 */
    GPIOMSP432_P7_5 | GPIO_DO_NOT_CONFIG,       /*  68 - P7.5 */
    GPIOMSP432_P7_7 | GPIO_DO_NOT_CONFIG,       /*  69 - P7.7 */
    GPIOMSP432_P10_1 | GPIO_DO_NOT_CONFIG,      /*  70 - P10.1 */
    GPIOMSP432_P10_3 | GPIO_DO_NOT_CONFIG,      /*  71 - P10.3 */
    GPIOMSP432_P10_5 | GPIO_DO_NOT_CONFIG,      /*  72 - P10.5 */

    /* virtual pins 73-78 */
    GPIOMSP432_P1_1 | GPIO_DO_NOT_CONFIG,       /*  73 - P1.1 SW1 */
    GPIOMSP432_P1_4 | GPIO_DO_NOT_CONFIG,       /*  74 - P1.4 SW2 */
    GPIOMSP432_P2_0 | GPIO_DO_NOT_CONFIG,       /*  75 - P2.0 RED_LED */
    GPIOMSP432_P2_1 | GPIO_DO_NOT_CONFIG,       /*  76 - P2.1 GREEN_LED */
    GPIOMSP432_P2_2 | GPIO_DO_NOT_CONFIG,       /*  77 - P2.2 BLUE_LED */
    GPIOMSP432_P1_0 | GPIO_DO_NOT_CONFIG,       /*  78 - P1.0 LED1 */
};

/*
 * Array of callback function pointers
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in MSP_EXP432P401R.h
 * NOTE: Pins not used for interrupts can be omitted from callbacks array to
 *       reduce memory usage (if placed at end of gpioPinConfigs array).
 */
GPIO_CallbackFxn gpioCallbackFunctions[] = {
    /* port_pin */
    NULL,  /*  0  - dummy */

    /* pins 1-10 */
    NULL,  /*  1  - 3.3V */
    NULL,  /*  2  - P6.0_A15 */
    NULL,  /*  3  - P3.2_URXD */
    NULL,  /*  4  - P3.3_UTXD */
    NULL,  /*  5  - P4.1_IO_A12 */
    NULL,  /*  6  - P4.3_A10 */
    NULL,  /*  7  - P1.5_SPICLK */
    NULL,  /*  8  - P4.6_IO_A7 */
    NULL,  /*  9  - P6.5_I2CSCL */
    NULL,  /*  10 - P6.4_I2CSDA */

    /* pins 11-20 */
    NULL,  /*  11 - P3.6_IO */
    NULL,  /*  12 - P5.2_IO */
    NULL,  /*  13 - P5.0_IO */
    NULL,  /*  14 - P1.7_SPIMISO */
    NULL,  /*  15 - P1.6_SPIMOSI */
    NULL,  /*  16 - RESET */
    NULL,  /*  17 - P5.7_IO */
    NULL,  /*  18 - P3.0_IO */
    NULL,  /*  19 - P2.5_IO_PWM */
    NULL,  /*  20 - GND */

    /* pins 21-30 */
    NULL,  /*  21 - 5V */
    NULL,  /*  22 - GND */
    NULL,  /*  23 - P6.1_A14 */
    NULL,  /*  24 - P4.0_A13 */
    NULL,  /*  25 - P4.2_A11 */
    NULL,  /*  26 - P4.4_A9 */
    NULL,  /*  27 - P4.5_A8 */
    NULL,  /*  28 - P4.7_A6 */
    NULL,  /*  29 - P5.4_IO */
    NULL,  /*  30 - P5.5_IO */

    /* pins 31-40 */
    NULL,  /*  31 - P3.7_IO */
    NULL,  /*  32 - P3.5_IO */
    NULL,  /*  33 - P5.1_IO */
    NULL,  /*  34 - P2.3_IO */
    NULL,  /*  35 - P6.7_IO_CAPT */
    NULL,  /*  36 - P6.6_IO_CAPT */
    NULL,  /*  37 - P5.6_PWM */
    NULL,  /*  38 - P2.4_PWM */
    NULL,  /*  39 - P2.6_PWM */
    NULL,  /*  40 - P2.7_PWM */

    /* pins 41-56 */
    NULL,  /*  41 - P8.5 */
    NULL,  /*  42 - P9.0 */
    NULL,  /*  43 - P8.4 */
    NULL,  /*  44 - P8.2 */
    NULL,  /*  45 - P9.2 */
    NULL,  /*  46 - P6.2 */
    NULL,  /*  47 - P7.3 */
    NULL,  /*  48 - P7.1 */
    NULL,  /*  49 - P9.4 */
    NULL,  /*  40 - P9.6 */
    NULL,  /*  51 - P8.0 */
    NULL,  /*  52 - P7.4 */
    NULL,  /*  53 - P7.6 */
    NULL,  /*  54 - P10.0 */
    NULL,  /*  55 - P10_2 */
    NULL,  /*  56 - P10.4 */

    /* pins 57-72 */
    NULL,  /*  57 - P8.6 */
    NULL,  /*  58 - P8.7 */
    NULL,  /*  59 - P9.1 */
    NULL,  /*  60 - P8.3 */
    NULL,  /*  61 - P5.3 */
    NULL,  /*  62 - P9.3 */
    NULL,  /*  63 - P6.3 */
    NULL,  /*  64 - P7.2 */
    NULL,  /*  65 - P7.0 */
    NULL,  /*  66 - P9.5 */
    NULL,  /*  67 - P9.7 */
    NULL,  /*  68 - P7.5 */
    NULL,  /*  69 - P7.7 */
    NULL,  /*  70 - P10.1 */
    NULL,  /*  71 - P10.3 */
    NULL,  /*  72 - P10.5 */

    /* virtual pins 73-78 */
    NULL,  /*  73 - P1.1 SW1 */
    NULL,  /*  74 - P1.4 SW2 */
    NULL,  /*  75 - P2.0 RED_LED */
    NULL,  /*  76 - P2.1 GREEN_LED */
    NULL,  /*  77 - P2.2 BLUE_LED */
    NULL,  /*  78 - P1.0 LED1 */
};

const GPIOMSP432_Config GPIOMSP432_config = {
    .pinConfigs = (GPIO_PinConfig *)gpioPinConfigs,
    .callbacks = (GPIO_CallbackFxn *)gpioCallbackFunctions,
    .numberOfPinConfigs = sizeof(gpioPinConfigs)/sizeof(GPIO_PinConfig),
    .numberOfCallbacks = sizeof(gpioCallbackFunctions)/sizeof(GPIO_CallbackFxn),
    .intPriority = (~0)
};

/* Not sure why the SVSLOFF bit is undefined in the latest driverlib msp432p401r.h file */

#define SVSLOFF 0x00000100

/*
 *  ======== Board_initGPIO ========
 */
void Board_initGPIO(void)
{
    /* Terminate all IO pins on the device */
    P1DIR |= 0xFF; P1OUT = 0;
    P2DIR |= 0xFF; P2OUT = 0;
    P3DIR |= 0xFF; P3OUT = 0;
    P4DIR |= 0xFF; P4OUT = 0;
    P5DIR |= 0xFF; P5OUT = 0;
    P6DIR |= 0xFF; P6OUT = 0;
    P7DIR |= 0xFF; P7OUT = 0;
    P8DIR |= 0xFF; P8OUT = 0;
    P9DIR |= 0xFF; P9OUT = 0;
    P10DIR |= 0xFF; P10OUT = 0;

    /* Configure Port PJ.2 and PJ.3 as GPIO and write 0 */
    PJDIR |= (BIT2 | BIT3); PJOUT &= ~(BIT2 | BIT3);

    /* PJ.0 & PJ.1 configured for LFXT IN/OUT */
    PJSEL0 |= BIT0 | BIT1;
    PJSEL1 &= ~(BIT0 | BIT1);

    /* Turn off PSS high-side & low-side supervisors */
    PSS->KEY = PSS_KEY_KEY_VAL;
    PSS->CTL0 |= PSS_CTL0_SVSMHOFF | SVSLOFF;         /* 823 (14) uA -> 809 (3) uA */
    PSS->KEY = 0;

    /* Configure Port PJ.4 and PJ.5 */
    ; /* do nothing (the reset default is to support JTAG) */

    /* set up initial TI-RTOS GPIO pin configurations */
    GPIO_init();
}

/*
 *  =============================== I2C ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(I2C_config, ".const:I2C_config")
#pragma DATA_SECTION(i2cMSP432HWAttrs, ".const:i2cMSP432HWAttrs")
#endif

#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CMSP432.h>

/* I2C objects */
I2CMSP432_Object i2cMSP432Objects[Board_I2CCOUNT];

/* I2C configuration structure */
const I2CMSP432_HWAttrs i2cMSP432HWAttrs[Board_I2CCOUNT] = {
    {
        .baseAddr = EUSCI_B1_BASE,
        .intNum = INT_EUSCIB1,
        .intPriority = (~0),
        .clockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK
    }
};

const I2C_Config I2C_config[] = {
    {
        .fxnTablePtr = &I2CMSP432_fxnTable,
        .object = &i2cMSP432Objects[0],
        .hwAttrs = &i2cMSP432HWAttrs[0]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== Board_openI2C ========
 *  Initialize the I2C driver.
 *  Initialize the I2C port's pins.
 *  Open the I2C port.
 */
I2C_Handle Board_openI2C(UInt i2cPortIndex, I2C_Params *i2cParams)
{
    
    /* Initialize the I2C driver */
    /* By design, I2C_init() is idempotent */
    I2C_init();
    
    /* initialize the pins associated with the respective I2C */
    switch(i2cPortIndex) {
        case 0:
            /* Configure Pins 6.4 & 6.5 as SDA & SCL, respectively. */
            MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6,
                                                           GPIO_PIN4 | GPIO_PIN5,
                                                           GPIO_PRIMARY_MODULE_FUNCTION);
            break;

        default:
            return (NULL);
    }

    /* open the I2C */
    return (I2C_open(i2cPortIndex, i2cParams));
}

/*
 *  =============================== Power ===============================
 */

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerMSP432.h>

const PowerMSP432_ConfigV1 PowerMSP432_config = {
    .policyInitFxn = PowerMSP432_initPolicy,
    .policyFxn = PowerMSP432_deepSleepPolicy,
    .initialPerfLevel = 2,
    .enablePolicy = true,
    .enablePerf = true,
    .enableParking = true
};

/*
 *  ======== Board_initPower ========
 */
void Board_initPower(void)
{
    Power_init();
}

/*
 *  =============================== PWM ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(PWM_config, ".const:PWM_config")
#pragma DATA_SECTION(pwmTimerMSP432HWAttrs, ".const:pwmTimerMSP432HWAttrs")
#endif

#include <ti/drivers/PWM.h>
#include <ti/drivers/pwm/PWMTimerMSP432.h>

PWMTimerMSP432_Object pwmTimerMSP432Objects[Board_PWMCOUNT];

/* PWM configuration structure */
PWMTimerMSP432_HWAttrsV1 pwmTimerMSP432HWAttrs[Board_PWMCOUNT] = {
    /* pin mappable PWM channels */
    {
        .timerBaseAddr = TIMER_A0_BASE,
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1,
        .gpioPort = GPIO_PORT_P2,
        .gpioPinIndex = GPIO_PIN1,
        .pwmMode = GPIO_PRIMARY_MODULE_FUNCTION
    },
    {
        .timerBaseAddr = TIMER_A0_BASE,
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2,
        .gpioPort = GPIO_PORT_P2,
        .gpioPinIndex = GPIO_PIN1,
        .pwmMode = GPIO_PRIMARY_MODULE_FUNCTION
    },
    {
        .timerBaseAddr = TIMER_A0_BASE,
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3,
        .gpioPort = GPIO_PORT_P2,
        .gpioPinIndex = GPIO_PIN1,
        .pwmMode = GPIO_PRIMARY_MODULE_FUNCTION
    },
    {
        .timerBaseAddr = TIMER_A0_BASE,
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4,
        .gpioPort = GPIO_PORT_P2,
        .gpioPinIndex = GPIO_PIN1,
        .pwmMode = GPIO_PRIMARY_MODULE_FUNCTION
    },
    {
        .timerBaseAddr = TIMER_A1_BASE,
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1,
        .gpioPort = GPIO_PORT_P2,
        .gpioPinIndex = GPIO_PIN1,
        .pwmMode = GPIO_PRIMARY_MODULE_FUNCTION
    },
    {
        .timerBaseAddr = TIMER_A1_BASE,
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2,
        .gpioPort = GPIO_PORT_P2,
        .gpioPinIndex = GPIO_PIN1,
        .pwmMode = GPIO_PRIMARY_MODULE_FUNCTION
    },
    {
        .timerBaseAddr = TIMER_A1_BASE,
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3,
        .gpioPort = GPIO_PORT_P2,
        .gpioPinIndex = GPIO_PIN1,
        .pwmMode = GPIO_PRIMARY_MODULE_FUNCTION
    },
    {
        .timerBaseAddr = TIMER_A1_BASE,
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4,
        .gpioPort = GPIO_PORT_P2,
        .gpioPinIndex = GPIO_PIN1,
        .pwmMode = GPIO_PRIMARY_MODULE_FUNCTION
    },
	/* fixed pin mapped PWM channels */
    {
        .timerBaseAddr = TIMER_A2_BASE,
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1,
        .gpioPort = GPIO_PORT_P2,
        .gpioPinIndex = GPIO_PIN1,
        .pwmMode = GPIO_PRIMARY_MODULE_FUNCTION
    },
    {
        .timerBaseAddr = TIMER_A2_BASE,
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2,
        .gpioPort = GPIO_PORT_P2,
        .gpioPinIndex = GPIO_PIN1,
        .pwmMode = GPIO_PRIMARY_MODULE_FUNCTION
    },
    {
        .timerBaseAddr = TIMER_A2_BASE,
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3,
        .gpioPort = GPIO_PORT_P2,
        .gpioPinIndex = GPIO_PIN1,
        .pwmMode = GPIO_PRIMARY_MODULE_FUNCTION
    },
    {
        .timerBaseAddr = TIMER_A2_BASE,
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4,
        .gpioPort = GPIO_PORT_P2,
        .gpioPinIndex = GPIO_PIN1,
        .pwmMode = GPIO_PRIMARY_MODULE_FUNCTION
    }
};

const PWM_Config PWM_config[] = {
    {
        .fxnTablePtr = &PWMTimerMSP432_fxnTable,
        .object = &pwmTimerMSP432Objects[0],
        .hwAttrs = &pwmTimerMSP432HWAttrs[0]
    },
    {
        .fxnTablePtr = &PWMTimerMSP432_fxnTable,
        .object = &pwmTimerMSP432Objects[1],
        .hwAttrs = &pwmTimerMSP432HWAttrs[1]
    },
    {
        .fxnTablePtr = &PWMTimerMSP432_fxnTable,
        .object = &pwmTimerMSP432Objects[2],
        .hwAttrs = &pwmTimerMSP432HWAttrs[2]
    },
    {
        .fxnTablePtr = &PWMTimerMSP432_fxnTable,
        .object = &pwmTimerMSP432Objects[3],
        .hwAttrs = &pwmTimerMSP432HWAttrs[3]
    },
    {
        .fxnTablePtr = &PWMTimerMSP432_fxnTable,
        .object = &pwmTimerMSP432Objects[4],
        .hwAttrs = &pwmTimerMSP432HWAttrs[4]
    },
    {
        .fxnTablePtr = &PWMTimerMSP432_fxnTable,
        .object = &pwmTimerMSP432Objects[5],
        .hwAttrs = &pwmTimerMSP432HWAttrs[5]
    },
    {
        .fxnTablePtr = &PWMTimerMSP432_fxnTable,
        .object = &pwmTimerMSP432Objects[6],
        .hwAttrs = &pwmTimerMSP432HWAttrs[6]
    },
    {
        .fxnTablePtr = &PWMTimerMSP432_fxnTable,
        .object = &pwmTimerMSP432Objects[7],
        .hwAttrs = &pwmTimerMSP432HWAttrs[7]
    },
    {
        .fxnTablePtr = &PWMTimerMSP432_fxnTable,
        .object = &pwmTimerMSP432Objects[8],
        .hwAttrs = &pwmTimerMSP432HWAttrs[8]
    },
    {
        .fxnTablePtr = &PWMTimerMSP432_fxnTable,
        .object = &pwmTimerMSP432Objects[9],
        .hwAttrs = &pwmTimerMSP432HWAttrs[9]
    },
    {
        .fxnTablePtr = &PWMTimerMSP432_fxnTable,
        .object = &pwmTimerMSP432Objects[10],
        .hwAttrs = &pwmTimerMSP432HWAttrs[10]
    },
    {
        .fxnTablePtr = &PWMTimerMSP432_fxnTable,
        .object = &pwmTimerMSP432Objects[11],
        .hwAttrs = &pwmTimerMSP432HWAttrs[11]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== Board_initPWM ========
 */
void Board_initPWM(void)
{
    PWM_init();
}

/*
 *  =============================== SDSPI ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(SDSPI_config, ".const:SDSPI_config")
#pragma DATA_SECTION(sdspiMSP432HWAttrs, ".const:sdspiMSP432HWAttrs")
#endif

#include <ti/drivers/SDSPI.h>
#include <ti/drivers/sdspi/SDSPIMSP432.h>

/* SDSPI objects */
SDSPIMSP432_Object sdspiMSP432Objects[Board_SDSPICOUNT];

/* SDSPI configuration structure, describing which pins are to be used */
const SDSPIMSP432_HWAttrs sdspiMSP432HWAttrs[Board_SDSPICOUNT] = {
    {
        .baseAddr = EUSCI_B0_BASE,
        .clockSource = EUSCI_B_SPI_CLOCKSOURCE_SMCLK,

        /* CLK, MOSI & MISO ports & pins */
        .portSCK = GPIO_PORT_P1,
        .pinSCK = GPIO_PIN5,
        .sckMode = GPIO_PRIMARY_MODULE_FUNCTION,

        .portMISO = GPIO_PORT_P1,
        .pinMISO = GPIO_PIN7,
        .misoMode = GPIO_PRIMARY_MODULE_FUNCTION,

        .portMOSI = GPIO_PORT_P1,
        .pinMOSI = GPIO_PIN6,
        .mosiMode = GPIO_PRIMARY_MODULE_FUNCTION,

        /* Chip select port & pin */
        .portCS = GPIO_PORT_P4,
        .pinCS = GPIO_PIN6
    }
};

const SDSPI_Config SDSPI_config[] = {
    {
        .fxnTablePtr = &SDSPIMSP432_fxnTable,
        .object = &sdspiMSP432Objects[0],
        .hwAttrs = &sdspiMSP432HWAttrs[0]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== Board_initSDSPI ========
 */
void Board_initSDSPI(void)
{
    SDSPI_init();
}

/*
 *  =============================== SPI ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(SPI_config, ".const:SPI_config")
#pragma DATA_SECTION(spiMSP432DMAHWAttrs, ".const:spiMSP432DMAHWAttrs")
#endif

#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPIMSP432DMA.h>

/* SPI objects */
SPIMSP432DMA_Object spiMSP432DMAObjects[Board_SPICOUNT];

/* SPI configuration structure, describing which pins are to be used */
const SPIMSP432DMA_HWAttrs spiMSP432DMAHWAttrs[Board_SPICOUNT] = {
    {
        .baseAddr = EUSCI_B0_BASE,
        .bitOrder = EUSCI_B_SPI_MSB_FIRST,
        .clockSource = EUSCI_B_SPI_CLOCKSOURCE_SMCLK,

        .defaultTxBufValue = 0,

        .dmaIntNum = INT_DMA_INT1,
        .intPriority = 0xC0,       /* make SPI interrupt one priority higher than default */
        .rxDMAChannelIndex = DMA_CH1_EUSCIB0RX0,
        .txDMAChannelIndex = DMA_CH0_EUSCIB0TX0
    },
    {
        .baseAddr = EUSCI_B2_BASE,
        .bitOrder = EUSCI_B_SPI_MSB_FIRST,
        .clockSource = EUSCI_B_SPI_CLOCKSOURCE_SMCLK,

        .defaultTxBufValue = 0,

        .dmaIntNum = INT_DMA_INT2,
        .intPriority = 0xC0,       /* make SPI interrupt one priority higher than default */
        .rxDMAChannelIndex = DMA_CH5_EUSCIB2RX0,
        .txDMAChannelIndex = DMA_CH4_EUSCIB2TX0
    }
};

const SPI_Config SPI_config[] = {
    {
        .fxnTablePtr = &SPIMSP432DMA_fxnTable,
        .object = &spiMSP432DMAObjects[0],
        .hwAttrs = &spiMSP432DMAHWAttrs[0]
    },
    {
        .fxnTablePtr = &SPIMSP432DMA_fxnTable,
        .object = &spiMSP432DMAObjects[1],
        .hwAttrs = &spiMSP432DMAHWAttrs[1]
    },
    {NULL, NULL, NULL},
};

/*
 *  ======== Board_openSPI ========
 */
SPI_Handle Board_openSPI(UInt spiPortIndex, SPI_Params *spiParams)
{
    /* Initialize the SPI driver */
    /* By design, SPI_init() is idempotent */
    SPI_init();

    /* initialize the pins associated with the respective UART */
    switch(spiPortIndex) {
        case 0:

            /* Configure CLK, MOSI & MISO for SPI0 (EUSCI_B0) */
            MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1,
                                                            GPIO_PIN5 | GPIO_PIN6,
                                                            GPIO_PRIMARY_MODULE_FUNCTION);
            MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                                                           GPIO_PIN7,
                                                           GPIO_PRIMARY_MODULE_FUNCTION);
            break;
            
        case 1:
            /* Configure CLK, MOSI & MISO for SPI1 (EUSCI_B2) */
            MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3,
                                                            GPIO_PIN5 | GPIO_PIN6,
                                                            GPIO_PRIMARY_MODULE_FUNCTION);
            MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
                                                           GPIO_PIN7,
                                                           GPIO_PRIMARY_MODULE_FUNCTION);
            break;

        default:
            return(NULL);
    }
    
    /* open the SPI port */
    return (SPI_open(spiPortIndex, spiParams));
}

/*
 *  =============================== UART ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(UART_config, ".const:UART_config")
#pragma DATA_SECTION(uartMSP432HWAttrs, ".const:uartMSP432HWAttrs")
#endif

#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTMSP432.h>

/* UART objects */
UARTMSP432_Object uartMSP432Objects[Board_UARTCOUNT];

/*
 * The baudrate dividers were determined by using the MSP430 baudrate
 * calculator
 * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 */
const UARTMSP432_BaudrateConfig uartMSP432Baudrates[] = {
    /* {baudrate, input clock, prescalar, UCBRFx, UCBRSx, oversampling} */
    {
        .outputBaudrate = 115200,
        .inputClockFreq = 12000000,
        .prescalar = 6,
        .hwRegUCBRFx = 8,
        .hwRegUCBRSx = 32,
        .oversampling = 1
    },
    {57600,  12000000,  13,  0,  37, 1},
    {38400,  12000000,  19,  8,  85, 1},
    {19200,  12000000,  39,  1,   0, 1},
    {9600,   12000000,  78,  2,   0, 1},
    {4800,   12000000, 156,  4,   0, 1},

    {115200, 6000000,    3,  4,   2, 1},
    {57600,  6000000,    6,  8,  32, 1},
    {38400,  6000000,    9, 12,  34, 1},
    {19200,  6000000,   19,  8,  85, 1},
    {9600,   6000000,   39,  1,   0, 1},
    {4800,   6000000,   78,  2,   0, 1},

    {115200, 3000000,    1, 10,   0, 1},
    {57600,  3000000,    3,  4,   2, 1},
    {38400,  3000000,    4, 14,   8, 1},
    {19200,  3000000,    9, 12,  34, 1},
    {9600,   3000000,   19,  8,  85, 1},
    {4800,   3000000,   39,  1,   0, 1},
};

unsigned char uartMSP432RingBuffer0[32];
unsigned char uartMSP432RingBuffer1[32];

/* UART configuration structure */
const UARTMSP432_HWAttrs uartMSP432HWAttrs[Board_UARTCOUNT] = {
    {
        .baseAddr = EUSCI_A0_BASE,
        .intNum = INT_EUSCIA0,
        .intPriority = (0xc0),
        .clockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK,
        .bitOrder = EUSCI_A_UART_LSB_FIRST,
        .numBaudrateEntries = sizeof(uartMSP432Baudrates) /
                              sizeof(UARTMSP432_BaudrateConfig),
        .baudrateLUT = uartMSP432Baudrates,
        .ringBufPtr  = uartMSP432RingBuffer0,
        .ringBufSize = sizeof(uartMSP432RingBuffer0)
    },
    {
        .baseAddr = EUSCI_A2_BASE,
        .intNum = INT_EUSCIA2,
        .intPriority = (0xc0),
        .clockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK,
        .bitOrder = EUSCI_A_UART_LSB_FIRST,
        .numBaudrateEntries = sizeof(uartMSP432Baudrates) /
            sizeof(UARTMSP432_BaudrateConfig),
        .baudrateLUT = uartMSP432Baudrates,
        .ringBufPtr  = uartMSP432RingBuffer1,
        .ringBufSize = sizeof(uartMSP432RingBuffer1)
    }
};

const UART_Config UART_config[] = {
    {
        .fxnTablePtr = &UARTMSP432_fxnTable,
        .object = &uartMSP432Objects[0],
        .hwAttrs = &uartMSP432HWAttrs[0]
    },
    {
        .fxnTablePtr = &UARTMSP432_fxnTable,
        .object = &uartMSP432Objects[1],
        .hwAttrs = &uartMSP432HWAttrs[1]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== Board_openUART ========
 *  Initialize the UART driver.
 *  Initialize the UART port's pins.
 *  Open the UART port.
 */
UART_Handle  Board_openUART(UInt uartPortIndex, UART_Params *uartParams)
{
    /* Initialize the UART driver */
    /* By design, UART_init() is idempotent */
    UART_init();

    /* initialize the pins associated with the respective UART */
    switch(uartPortIndex) {
        case 0:
            /* Set P1.2 & P1.3 in UART mode */
            MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                                                   GPIO_PIN2 | GPIO_PIN3,
                                                   GPIO_PRIMARY_MODULE_FUNCTION);
            break;

        case 1:
            /* Set P3.2 & P3.3 in UART mode */
            MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
                                                   GPIO_PIN2 | GPIO_PIN3,
                                                   GPIO_PRIMARY_MODULE_FUNCTION);
            break;

        default:
            return (NULL);
    }

    /* open the UART */
    return (UART_open(uartPortIndex, uartParams));
}

/*
 *  =============================== Watchdog ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(Watchdog_config, ".const:Watchdog_config")
#pragma DATA_SECTION(watchdogMSP432HWAttrs, ".const:watchdogMSP432HWAttrs")
#endif

#include <ti/drivers/Watchdog.h>
#include <ti/drivers/watchdog/WatchdogMSP432.h>

/* Watchdog objects */
WatchdogMSP432_Object watchdogMSP432Objects[Board_WATCHDOGCOUNT];

/* Watchdog configuration structure */
const WatchdogMSP432_HWAttrs watchdogMSP432HWAttrs[Board_WATCHDOGCOUNT] = {
    {
        .baseAddr = WDT_A_BASE,
        .intNum = INT_WDT_A,
        .intPriority = (~0),
        .clockSource = WDT_A_CLOCKSOURCE_SMCLK,
        .clockDivider = WDT_A_CLOCKDIVIDER_8192K
    },
};

const Watchdog_Config Watchdog_config[] = {
    {
        .fxnTablePtr = &WatchdogMSP432_fxnTable,
        .object = &watchdogMSP432Objects[0],
        .hwAttrs = &watchdogMSP432HWAttrs[0]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== Board_initWatchdog ========
 */
void Board_initWatchdog(void)
{
    /* Initialize the Watchdog driver */
    Watchdog_init();
}

#if 0
/*
 *  =============================== WiFi ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(WiFi_config, ".const:WiFi_config")
#pragma DATA_SECTION(wiFiCC3100HWAttrs, ".const:wiFiCC3100HWAttrs")
#endif
#include <ti/drivers/WiFi.h>
#include <ti/drivers/wifi/WiFiCC3100.h>

/* WiFi objects */
WiFiCC3100_Object wiFiCC3100Objects[Board_WIFICOUNT];

/* WiFi configuration structure */
const WiFiCC3100_HWAttrs wiFiCC3100HWAttrs[Board_WIFICOUNT] = {
    {
        .irqPort = GPIO_PORT_P2,
        .irqPin = GPIO_PIN5,
        .irqIntNum = INT_PORT2,

        .csPort = GPIO_PORT_P3,
        .csPin = GPIO_PIN0,

        .enPort = GPIO_PORT_P4,
        .enPin = GPIO_PIN1
    }
};

const WiFi_Config WiFi_config[] = {
    {
        .fxnTablePtr = &WiFiCC3100_fxnTable,
        .object = &wiFiCC3100Objects[0],
        .hwAttrs = &wiFiCC3100HWAttrs[0]
    },
    {NULL, NULL, NULL},
};

/*
 *  ======== Board_initWiFi ========
 */
void Board_initWiFi(void)
{
    /* Configure EN & CS pins to disable CC3100 */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN1);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);

    /* Configure CLK, MOSI & MISO for SPI0 (EUSCI_B0) */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1,
                                                    GPIO_PIN5 | GPIO_PIN6,
                                                    GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                                                   GPIO_PIN7,
                                                   GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configure IRQ pin */
    MAP_GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P2, GPIO_PIN5);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P2, GPIO_PIN5,
                                 GPIO_LOW_TO_HIGH_TRANSITION);

    /* Initialize SPI and WiFi drivers */
    SPI_init();
    WiFi_init();
}
#endif

/*
 *  ======== Board_init ========
 */
void Board_init(void) {
    Board_initGPIO();
    Board_initPWM();
    Board_initPower();
}

