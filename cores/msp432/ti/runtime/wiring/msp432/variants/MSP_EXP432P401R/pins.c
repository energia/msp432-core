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

#include <ti/runtime/wiring/wiring_private.h>
#include <ti/runtime/wiring/msp432/wiring_analog.h>

#include <adc14.h>

#include <ti/drivers/PWM.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOMSP432.h>

uint8_t digital_pin_to_pin_function[] = {
    PIN_FUNC_UNUSED,    /*  dummy */
    PIN_FUNC_UNUSED,    /*  1  - 3.3V */
    PIN_FUNC_UNUSED,    /*  2  - P6.0_A15 */
    PIN_FUNC_UNUSED,    /*  3  - P3.2_URXD */
    PIN_FUNC_UNUSED,    /*  4  - P3.3_UTXD */
    PIN_FUNC_UNUSED,    /*  5  - P4.1_IO_A12 */
    PIN_FUNC_UNUSED,    /*  6  - P4.3_A10 */
    PIN_FUNC_UNUSED,    /*  7  - P1.5_SPICLK */
    PIN_FUNC_UNUSED,    /*  8  - P4.6_IO_A7 */
    PIN_FUNC_UNUSED,    /*  9  - P6.5_I2CSCL */
    PIN_FUNC_UNUSED,    /*  10 - P6.4_I2CSDA */

    PIN_FUNC_UNUSED,    /*  11 - P3.6_IO */
    PIN_FUNC_UNUSED,    /*  12 - P5.2_IO */
    PIN_FUNC_UNUSED,    /*  13 - P5.0_IO */
    PIN_FUNC_UNUSED,    /*  14 - P1.7_SPIMISO */
    PIN_FUNC_UNUSED,    /*  15 - P1.6_SPIMOSI */
    PIN_FUNC_UNUSED,    /*  16 - RESET */
    PIN_FUNC_UNUSED,    /*  17 - P5.7_IO */
    PIN_FUNC_UNUSED,    /*  18 - P3.0_IO */
    PIN_FUNC_UNUSED,    /*  19 - P2.5_IO_PWM */
    PIN_FUNC_UNUSED,    /*  20 - GND */

    PIN_FUNC_UNUSED,    /*  21 - 5V */
    PIN_FUNC_UNUSED,    /*  22 - GND */
    PIN_FUNC_UNUSED,    /*  23 - P6.1_A14 */
    PIN_FUNC_UNUSED,    /*  24 - P4.0_A13 */
    PIN_FUNC_UNUSED,    /*  25 - P4.2_A11 */
    PIN_FUNC_UNUSED,    /*  26 - P4.4_A9 */
    PIN_FUNC_UNUSED,    /*  27 - P4.5_A8 */
    PIN_FUNC_UNUSED,    /*  28 - P4.7_A6 */
    PIN_FUNC_UNUSED,    /*  29 - P5.4_IO */
    PIN_FUNC_UNUSED,    /*  30 - P5.5_IO */

    PIN_FUNC_UNUSED,    /*  31 - P3.7_IO */
    PIN_FUNC_UNUSED,    /*  32 - P3.5_IO */
    PIN_FUNC_UNUSED,    /*  33 - P5.1_IO */
    PIN_FUNC_UNUSED,    /*  34 - P2.3_IO */
    PIN_FUNC_UNUSED,    /*  35 - P6.7_IO_CAPT */
    PIN_FUNC_UNUSED,    /*  36 - P6.6_IO_CAPT */
    PIN_FUNC_UNUSED,    /*  37 - P5.6_PWM */
    PIN_FUNC_UNUSED,    /*  38 - P2.4_PWM */
    PIN_FUNC_UNUSED,    /*  39 - P2.6_PWM */
    PIN_FUNC_UNUSED,    /*  40 - P2.7_PWM */

    /* pins 41-56 */
    PIN_FUNC_UNUSED,    /*  41 - P8.5 */
    PIN_FUNC_UNUSED,    /*  42 - P9.0 */
    PIN_FUNC_UNUSED,    /*  43 - P8.4 */
    PIN_FUNC_UNUSED,    /*  44 - P8.2 */
    PIN_FUNC_UNUSED,    /*  45 - P9.2 */
    PIN_FUNC_UNUSED,    /*  46 - P6.2 */
    PIN_FUNC_UNUSED,    /*  47 - P7.3 */
    PIN_FUNC_UNUSED,    /*  48 - P7.1 */
    PIN_FUNC_UNUSED,    /*  49 - P9.4 */
    PIN_FUNC_UNUSED,    /*  40 - P9.6 */
    PIN_FUNC_UNUSED,    /*  51 - P8.0 */
    PIN_FUNC_UNUSED,    /*  52 - P7.4 */
    PIN_FUNC_UNUSED,    /*  53 - P7.6 */
    PIN_FUNC_UNUSED,    /*  54 - P10.0 */
    PIN_FUNC_UNUSED,    /*  55 - P10_2 */
    PIN_FUNC_UNUSED,    /*  56 - P10.4 */

    /* pins 57-72 */
    PIN_FUNC_UNUSED,    /*  57 - P8.6 */
    PIN_FUNC_UNUSED,    /*  58 - P8.7 */
    PIN_FUNC_UNUSED,    /*  59 - P9.1 */
    PIN_FUNC_UNUSED,    /*  60 - P8.3 */
    PIN_FUNC_UNUSED,    /*  61 - P5.3 */
    PIN_FUNC_UNUSED,    /*  62 - P9.3 */
    PIN_FUNC_UNUSED,    /*  63 - P6.3 */
    PIN_FUNC_UNUSED,    /*  64 - P7.2 */
    PIN_FUNC_UNUSED,    /*  65 - P7.0 */
    PIN_FUNC_UNUSED,    /*  66 - P9.5 */
    PIN_FUNC_UNUSED,    /*  67 - P9.7 */
    PIN_FUNC_UNUSED,    /*  68 - P7.5 */
    PIN_FUNC_UNUSED,    /*  69 - P7.7 */
    PIN_FUNC_UNUSED,    /*  70 - P10.1 */
    PIN_FUNC_UNUSED,    /*  71 - P10.3 */
    PIN_FUNC_UNUSED,    /*  72 - P10.5 */

    /* virtual pins 73-78 */
    PIN_FUNC_UNUSED,    /*  73 - P1.1 SW1 */
    PIN_FUNC_UNUSED,    /*  74 - P1.4 SW2 */
    PIN_FUNC_UNUSED,    /*  75 - P2.0 RED_LED */
    PIN_FUNC_UNUSED,    /*  76 - P2.1 GREEN_LED */
    PIN_FUNC_UNUSED,    /*  77 - P2.2 BLUE_LED */
    PIN_FUNC_UNUSED,    /*  78 - P1.0 LED1 */
};

/*
 * When a mappable pin is being used for analogWrite(),
 * its corresponding entry in this table is replaced with the
 * PWM channel index it is using.
 *
 * If/when a pin is then changed back to a digitial pin, the
 * pin's entry in this table is restored to PWM_MAPPABLE.
 *
 * Fixed map entries are not modified.
 */
uint8_t digital_pin_to_pwm_index[] = {
    PWM_NOT_MAPPABLE,       /*  dummy */
    PWM_NOT_MAPPABLE,       /*  1  - 3.3V */
    PWM_NOT_MAPPABLE,       /*  2  - P6.0_A15 */
    PWM_MAPPABLE,           /*  3  - P3.2_URXD */
    PWM_MAPPABLE,           /*  4  - P3.3_UTXD */
    PWM_NOT_MAPPABLE,       /*  5  - P4.1_IO_A12 */
    PWM_NOT_MAPPABLE,       /*  6  - P4.3_A10 */
    PWM_NOT_MAPPABLE,       /*  7  - P1.5_SPICLK */
    PWM_NOT_MAPPABLE,       /*  8  - P4.6_IO_A7 */
    PWM_NOT_MAPPABLE,       /*  9  - P6.5_I2CSCL */
    PWM_NOT_MAPPABLE,       /*  10 - P6.4_I2CSDA */

    PWM_MAPPABLE,           /*  11 - P3.6_IO */
    PWM_NOT_MAPPABLE,       /*  12 - P5.2_IO */
    PWM_NOT_MAPPABLE,       /*  13 - P5.0_IO */
    PWM_NOT_MAPPABLE,       /*  14 - P1.7_SPIMISO */
    PWM_NOT_MAPPABLE,       /*  15 - P1.6_SPIMOSI */
    PWM_NOT_MAPPABLE,       /*  16 - RESET */
    PWM_FIXED_INDEX_9,      /*  17 - P5.7_IO */
    PWM_MAPPABLE,           /*  18 - P3.0_IO */
    PWM_MAPPABLE,           /*  19 - P2.5_IO_PWM */
    PWM_NOT_MAPPABLE,       /*  20 - GND */

    PWM_NOT_MAPPABLE,       /*  21 - 5V */
    PWM_NOT_MAPPABLE,       /*  22 - GND */
    PWM_NOT_MAPPABLE,       /*  23 - P6.1_A14 */
    PWM_NOT_MAPPABLE,       /*  24 - P4.0_A13 */
    PWM_NOT_MAPPABLE,       /*  25 - P4.2_A11 */
    PWM_NOT_MAPPABLE,       /*  26 - P4.4_A9 */
    PWM_NOT_MAPPABLE,       /*  27 - P4.5_A8 */
    PWM_NOT_MAPPABLE,       /*  28 - P4.7_A6 */
    PWM_NOT_MAPPABLE,       /*  29 - P5.4_IO */
    PWM_NOT_MAPPABLE,       /*  30 - P5.5_IO */

    PWM_MAPPABLE,           /*  31 - P3.7_IO */
    PWM_MAPPABLE,           /*  32 - P3.5_IO */
    PWM_NOT_MAPPABLE,       /*  33 - P5.1_IO */
    PWM_MAPPABLE,           /*  34 - P2.3_IO */
    PWM_FIXED_INDEX_11,     /*  35 - P6.7_IO_CAPT */
    PWM_FIXED_INDEX_10,     /*  36 - P6.6_IO_CAPT */
    PWM_FIXED_INDEX_8,      /*  37 - P5.6_PWM */
    PWM_MAPPABLE,           /*  38 - P2.4_PWM */
    PWM_MAPPABLE,           /*  39 - P2.6_PWM */
    PWM_MAPPABLE,           /*  40 - P2.7_PWM */

    /* pins 41-56 */
    PWM_NOT_MAPPABLE,       /*  41 - P8.5 */
    PWM_NOT_MAPPABLE,       /*  42 - P9.0 */
    PWM_NOT_MAPPABLE,       /*  43 - P8.4 */
    PWM_NOT_MAPPABLE,       /*  44 - P8.2 */
    PWM_NOT_MAPPABLE,       /*  45 - P9.2 */
    PWM_NOT_MAPPABLE,       /*  46 - P6.2 */
    PWM_MAPPABLE,           /*  47 - P7.3 */
    PWM_MAPPABLE,           /*  48 - P7.1 */
    PWM_NOT_MAPPABLE,       /*  49 - P9.4 */
    PWM_NOT_MAPPABLE,       /*  40 - P9.6 */
    PWM_NOT_MAPPABLE,       /*  51 - P8.0 */
    PWM_MAPPABLE,           /*  52 - P7.4 */
    PWM_MAPPABLE,           /*  53 - P7.6 */
    PWM_NOT_MAPPABLE,       /*  54 - P10.0 */
    PWM_NOT_MAPPABLE,       /*  55 - P10_2 */
    PWM_NOT_MAPPABLE,       /*  56 - P10.4 */

    /* pins 57-72 */
    PWM_NOT_MAPPABLE,       /*  57 - P8.6 */
    PWM_NOT_MAPPABLE,       /*  58 - P8.7 */
    PWM_NOT_MAPPABLE,       /*  59 - P9.1 */
    PWM_NOT_MAPPABLE,       /*  60 - P8.3 */
    PWM_NOT_MAPPABLE,       /*  61 - P5.3 */
    PWM_NOT_MAPPABLE,       /*  62 - P9.3 */
    PWM_NOT_MAPPABLE,       /*  63 - P6.3 */
    PWM_MAPPABLE,           /*  64 - P7.2 */
    PWM_MAPPABLE,           /*  65 - P7.0 */
    PWM_NOT_MAPPABLE,       /*  66 - P9.5 */
    PWM_NOT_MAPPABLE,       /*  67 - P9.7 */
    PWM_MAPPABLE,           /*  68 - P7.5 */
    PWM_MAPPABLE,           /*  69 - P7.7 */
    PWM_NOT_MAPPABLE,       /*  70 - P10.1 */
    PWM_NOT_MAPPABLE,       /*  71 - P10.3 */
    PWM_NOT_MAPPABLE,       /*  72 - P10.5 */

    /* virtual pins 73-78 */
    PWM_NOT_MAPPABLE,       /*  73 - P1.1 SW1 */
    PWM_NOT_MAPPABLE,       /*  74 - P1.4 SW2 */
    PWM_MAPPABLE,           /*  75 - P2.0 RED_LED */
    PWM_MAPPABLE,           /*  76 - P2.1 GREEN_LED */
    PWM_MAPPABLE,           /*  77 - P2.2 BLUE_LED */
    PWM_NOT_MAPPABLE,       /*  78 - P1.0 LED1 */
};

/*
 * mapping of pins to an ADC channel
 */
const uint8_t digital_pin_to_adc_index[] = {
    /* port_pin */
    NOT_ON_ADC,     /*  dummy */

    /* pins 1-10 */
    NOT_ON_ADC,     /*  1  - 3.3V */
    ADC_INPUT_A15,  /*  2  - P6.0_A15 */
    NOT_ON_ADC,     /*  3  - P3.2_URXD */
    NOT_ON_ADC,     /*  4  - P3.3_UTXD */
    ADC_INPUT_A12,  /*  5  - P4.1_IO_A12 */
    ADC_INPUT_A10,  /*  6  - P4.3_A10 */
    NOT_ON_ADC,     /*  7  - P1.5_SPICLK */
    ADC_INPUT_A7,   /*  8  - P4.6_IO_A7 */
    NOT_ON_ADC,     /*  9  - P6.5_I2CSCL */
    NOT_ON_ADC,     /*  10 - P6.4_I2CSDA */

    /* pins 11-20 */
    NOT_ON_ADC,     /*  11 - P3.6_IO */
    ADC_INPUT_A3,   /*  12 - P5.2_IO */
    ADC_INPUT_A5,   /*  13 - P5.0_IO */
    NOT_ON_ADC,     /*  14 - P1.7_SPIMISO */
    NOT_ON_ADC,     /*  15 - P1.6_SPIMOSI */
    NOT_ON_ADC,     /*  16 - RESET */
    NOT_ON_ADC,     /*  17 - P5.7_IO */
    NOT_ON_ADC,     /*  18 - P3.0_IO */
    NOT_ON_ADC,     /*  19 - P2.5_IO_PWM */
    NOT_ON_ADC,     /*  20 - GND */

    /* pins 21-30 */
    NOT_ON_ADC,     /*  21 - 5V */
    NOT_ON_ADC,     /*  22 - GND */
    ADC_INPUT_A14,  /*  23 - P6.1_A14 */
    ADC_INPUT_A13,  /*  24 - P4.0_A13 */
    ADC_INPUT_A11,  /*  25 - P4.2_A11 */
    ADC_INPUT_A9,   /*  26 - P4.4_A9 */
    ADC_INPUT_A8,   /*  27 - P4.5_A8 */
    ADC_INPUT_A6,   /*  28 - P4.7_A6 */
    ADC_INPUT_A1,   /*  29 - P5.4_IO */
    ADC_INPUT_A0,   /*  30 - P5.5_IO */

    /* pins 31-40 */
    NOT_ON_ADC,     /*  31 - P3.7_IO */
    NOT_ON_ADC,     /*  32 - P3.5_IO */
    ADC_INPUT_A4,   /*  33 - P5.1_IO */
    NOT_ON_ADC,     /*  34 - P2.3_IO */
    NOT_ON_ADC,     /*  35 - P6.7_IO_CAPT */
    NOT_ON_ADC,     /*  36 - P6.6_IO_CAPT */
    NOT_ON_ADC,     /*  37 - P5.6_PWM */
    NOT_ON_ADC,     /*  38 - P2.4_PWM */
    NOT_ON_ADC,     /*  39 - P2.6_PWM */
    NOT_ON_ADC,     /*  40 - P2.7_PWM */

    /* pins 41-56 */
    ADC_INPUT_A20,  /*  41 - P8.5 */
    ADC_INPUT_A17,  /*  42 - P9.0 */
    ADC_INPUT_A21,  /*  43 - P8.4 */
    ADC_INPUT_A23,  /*  44 - P8.2 */
    NOT_ON_ADC,     /*  45 - P9.2 */
    NOT_ON_ADC,     /*  46 - P6.2 */
    NOT_ON_ADC,     /*  47 - P7.3 */
    NOT_ON_ADC,     /*  48 - P7.1 */
    NOT_ON_ADC,     /*  49 - P9.4 */
    NOT_ON_ADC,     /*  40 - P9.6 */
    NOT_ON_ADC,     /*  51 - P8.0 */
    NOT_ON_ADC,     /*  52 - P7.4 */
    NOT_ON_ADC,     /*  53 - P7.6 */
    NOT_ON_ADC,     /*  54 - P10.0 */
    NOT_ON_ADC,     /*  55 - P10_2 */
    NOT_ON_ADC,     /*  56 - P10.4 */

    /* pins 57-72 */
    ADC_INPUT_A19,  /*  57 - P8.6 */
    ADC_INPUT_A18,  /*  58 - P8.7 */
    ADC_INPUT_A16,  /*  59 - P9.1 */
    ADC_INPUT_A22,  /*  60 - P8.3 */
    ADC_INPUT_A2,   /*  61 - P5.3 */
    NOT_ON_ADC,     /*  62 - P9.3 */
    NOT_ON_ADC,     /*  63 - P6.3 */
    NOT_ON_ADC,     /*  64 - P7.2 */
    NOT_ON_ADC,     /*  65 - P7.0 */
    NOT_ON_ADC,     /*  66 - P9.5 */
    NOT_ON_ADC,     /*  67 - P9.7 */
    NOT_ON_ADC,     /*  68 - P7.5 */
    NOT_ON_ADC,     /*  69 - P7.7 */
    NOT_ON_ADC,     /*  70 - P10.1 */
    NOT_ON_ADC,     /*  71 - P10.3 */
    NOT_ON_ADC,     /*  72 - P10.5 */

    /* virtual pins 73-78 */
    NOT_ON_ADC,     /*  73 - P1.1 SW1 */
    NOT_ON_ADC,     /*  73 - P1.4 SW2 */
    NOT_ON_ADC,     /*  75 - P2.0 RED_LED */
    NOT_ON_ADC,     /*  76 - P2.1 GREEN_LED */
    NOT_ON_ADC,     /*  77 - P2.2 BLUE_LED */
    NOT_ON_ADC,     /*  78 - P1.0 LED1 */
};

