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

#include <ti/sysbios/family/arm/m3/Hwi.h>

#include "wiring_private.h"
#include "SPI.h"

void spiTransferCallback(SPI_Handle handle,
                                        SPI_Transaction * transaction);
SPIClass::SPIClass(void)
{
    init(0);
}

SPIClass::SPIClass(unsigned long module)
{
    init(module);
}

/*
 * Private Methods
 */
void SPIClass::init(unsigned long module)
{
    spiModule = module;
    begun = FALSE;
    dataMode = SPI_MODE0;
    bitOrder = MSBFIRST;
    clockDivider = SPI_CLOCK_DIV4;
    numUsingInterrupts = 0;
}

/*
 * Public Methods
 */
void SPIClass::begin(uint8_t ssPin)
{
    /* return if SPI already started */
    if (begun == TRUE) {
        return;
    }

    SPI_Params_init(&params);

    params.bitRate = SPI_CLOCK_MAX / clockDivider;
    params.frameFormat = (SPI_FrameFormat) dataMode;
    params.transferMode = SPI_MODE_CALLBACK;
    params.transferCallbackFxn = spiTransferCallback;

    spi = Board_openSPI(spiModule, &params);

    if (spi != NULL) {
	/* 6/18/2015 no support for pin profiles, just save for now */
        slaveSelect = ssPin;
        GateMutex_construct(&gate, NULL);
        begun = TRUE;
    }
}

void SPIClass::begin()
{
    /* default CS is under user control */
    begin(0);
}

void SPIClass::end(uint8_t ssPin) {
    begun = FALSE;
    numUsingInterrupts = 0;
    SPI_close(spi);
}

void SPIClass::end()
{
    end(slaveSelect);
}

void SPIClass::setBitOrder(uint8_t ssPin, uint8_t bitOrder)
{
    if (bitOrder == LSBFIRST || bitOrder == MSBFIRST) {
        this->bitOrder = bitOrder;
    }
}

void SPIClass::setBitOrder(uint8_t bitOrder)
{
    setBitOrder(0, bitOrder);
}

void SPIClass::setDataMode(uint8_t mode)
{
    dataMode = mode;

    if (begun == TRUE) {
        SPI_close(spi);
        params.frameFormat = (SPI_FrameFormat) dataMode;
        spi = SPI_open(spiModule, &params);
    }
}

void SPIClass::setClockDivider(uint8_t divider)
{
    clockDivider = divider;

    if (begun == TRUE) {
        SPI_close(spi);
        params.bitRate = SPI_CLOCK_MAX / clockDivider;
        spi = SPI_open(spiModule, &params);
    }
}

uint8_t SPIClass::reverseBits(uint8_t rxtxData)
{
#if (defined(xdc_target__isaCompatible_v7M) || defined(xdc_target__isaCompatible_v7A))  \
     &&  defined(__TI_COMPILER_VERSION__)
	rxtxData = __rbit(rxtxData);
        rxtxData = __rev(rxtxData);
#elif (defined(xdc_target__isaCompatible_v7M) || defined(xdc_target__isaCompatible_v7A))  \
     && defined(__GNUC__)
        /* reverse order of 32 bits */
        asm("rbit %0, %1" : "=r" (rxtxData) : "r" (rxtxData));
        /* reverse order of bytes to get original bits into lowest byte */
        asm("rev %0, %1" : "=r" (rxtxData) : "r" (rxtxData));
#else
    static  const uint8_t reverse_data[] =
        { 0x0, 0x8, 0x4, 0xC,
          0x2, 0xA, 0x6, 0xE,
          0x1, 0x9, 0x5, 0xD,
          0x3, 0xB, 0x7, 0xF
        };

    uint8_t b = 0;

    b  = reverse_data[x & 0xF] << 4;
    b |= reverse_data[(x & 0xF0) >> 4];
    rxtxData = b;
#endif
    return (rxtxData);
}

uint8_t *SPIClass::transfer(uint8_t *buffer, size_t size)
{
    uint32_t taskKey, hwiKey;
    uint8_t i;

    if (spi == NULL) {
        return (0);
    }
    
    /* protect single 'transaction' content from re-rentrancy */
    taskKey = Task_disable();

    hwiKey = Hwi_disable();

    /* disable all interrupts registered with SPI.usingInterrupt() */
    for (i = 0; i < numUsingInterrupts; i++) {
        disablePinInterrupt(usingInterruptPins[i]);
    }

    Hwi_restore(hwiKey);

    if (bitOrder == LSBFIRST) {
        for (i = 0; i < size; i++) {
            buffer[i] = reverseBits(buffer[i]);
        }
    }

    transaction.txBuf = buffer;
    transaction.rxBuf = buffer;
    transaction.count = size;
    transferComplete = 0;

    /* kick off the SPI transaction */
    SPI_transfer(spi, &transaction);

    /* wait for transfer to complete (ie for callback to be called) */
    while (transferComplete == 0) {
        ;
    }

    /* now that the transaction is finished, allow other threads to pre-empt */

    hwiKey = Hwi_disable();

    /* re-enable all interrupts registered with SPI.usingInterrupt() */
    for (i = 0; i < numUsingInterrupts; i++) {
        enablePinInterrupt(usingInterruptPins[i]);
    }

    Hwi_restore(hwiKey);

    if (bitOrder == LSBFIRST) {
        for (i = 0; i < size; i++) {
            buffer[i] = reverseBits(buffer[i]);
        }
    }

    Task_restore(taskKey);

    return (buffer);
}

uint8_t SPIClass::transfer(uint8_t ssPin, uint8_t data_out, uint8_t transferMode)
{
    uint8_t data_in;
    uint8_t i;
    uint32_t taskKey, hwiKey;

    if (spi == NULL) {
        return (0);
    }
    
    if (bitOrder == LSBFIRST) {
        data_out = reverseBits(data_out);
    }

    /* protect single 'transaction' content from re-rentrancy */
    taskKey = Task_disable();

    hwiKey = Hwi_disable();

    /* disable all interrupts registered with SPI.usingInterrupt() */
    for (i = 0; i < numUsingInterrupts; i++) {
        disablePinInterrupt(usingInterruptPins[i]);
    }

    Hwi_restore(hwiKey);

    /* select SPI peripheral if ssPin was provided */
    if (ssPin != 0) {
        digitalWrite(ssPin, LOW);
    }

    transaction.txBuf = &data_out;
    transaction.rxBuf = &data_in;
    transaction.count = 1;
    transferComplete = 0;

    /* kick off the SPI transaction */
    SPI_transfer(spi, &transaction);

    /* wait for transfer to complete (ie for callback to be called) */
    while (transferComplete == 0) {
        ;
    }

    /* deselect SPI peripheral if ssPin was provided */
    if (transferMode == SPI_LAST && ssPin != 0) {
        digitalWrite(ssPin, HIGH);
    }

    /* now that the transaction is finished, allow other threads to pre-empt */

    hwiKey = Hwi_disable();

    /* re-enable all interrupts registered with SPI.usingInterrupt() */
    for (i = 0; i < numUsingInterrupts; i++) {
        enablePinInterrupt(usingInterruptPins[i]);
    }

    Hwi_restore(hwiKey);

    Task_restore(taskKey);

    if (bitOrder == LSBFIRST) {
        data_in = reverseBits(data_in);
    }

    return ((uint8_t)data_in);
}

uint8_t SPIClass::transfer(uint8_t ssPin, uint8_t data)
{
    return (transfer(ssPin, data, SPI_LAST));
}

uint8_t SPIClass::transfer(uint8_t data)
{
    return (transfer(0, data, SPI_LAST));
}

void SPIClass::setModule(uint8_t module)
{
    spiModule = module;
    begin(slaveSelect);
}

void SPIClass::usingInterrupt(uint8_t pin)
{
    if (numUsingInterrupts < 16) {
	usingInterruptPins[numUsingInterrupts++] = pin;
    }
}

/* C type function */
void spiTransferCallback(SPI_Handle spi, SPI_Transaction * transaction)
{
    SPI.transferComplete = 1;
}
