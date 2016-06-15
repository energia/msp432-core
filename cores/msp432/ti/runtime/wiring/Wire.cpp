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

#include <stdlib.h>
#include <string.h>
#include "wiring_private.h"
#include "Wire.h"

#include <xdc/runtime/Memory.h>

#include <ti/sysbios/knl/Task.h>


#define TX_BUFFER_EMPTY    (wc->txReadIndex == wc->txWriteIndex)
#define TX_BUFFER_FULL     (((wc->txWriteIndex + 1) % BUFFER_LENGTH) == wc->txReadIndex)

#define RX_BUFFER_EMPTY    (wc->rxReadIndex == wc->rxWriteIndex)
#define RX_BUFFER_FULL     (((wc->rxWriteIndex + 1) % BUFFER_LENGTH) == wc->rxReadIndex)

#define RUN_BIT     0x1
#define START_BIT   0x2
#define STOP_BIT    0x4
#define ACK_BIT     0x8

TwoWire::TwoWire()
{
    init(0);
}

TwoWire::TwoWire(unsigned long module)
{
    init(module);
}

/*
 * Private Methods
 */
void TwoWire::init(unsigned long module)
{
    i2cModule = module;
    begun = FALSE;
}

WireContext *TwoWire::getWireContext(void)
{
    WireContext *wc;

    wc = (WireContext *)Task_getEnv(Task_self());

    if (wc == NULL) {
        wc = (WireContext *)Memory_alloc(NULL, sizeof(WireContext), 4, NULL);

        wc->idle = true;

        wc->rxReadIndex = 0;
        wc->rxWriteIndex = 0;
        wc->txReadIndex = 0;
        wc->txWriteIndex = 0;

        /* I2C Transfer initial params */
        wc->i2cTransaction.slaveAddress = 0;
        wc->i2cTransaction.writeBuf = wc->txBuffer;
        wc->i2cTransaction.readBuf = wc->rxBuffer;
        wc->i2cTransaction.readCount = 0;
        wc->i2cTransaction.writeCount = 0;

        Task_setEnv(Task_self(), (void *)wc);
    }

    return (wc);
}

void TwoWire::forceStop(void)
{
    //this has been removed so i can remove the pin map that used to be at the top of this file
}

/*
 * Public Methods
 */

// Initialize as a master
void TwoWire::begin(void)
{
    I2C_Params params;

    /* return if I2C already started */
    if (begun == TRUE) return;

    I2C_Params_init(&params);
    params.transferMode = I2C_MODE_BLOCKING;
    params.bitRate = I2C_400kHz;

    i2c = Board_openI2C(i2cModule, &params);

    if (i2c != NULL) {
        GateMutex_construct(&gate, NULL);
        gateEnterCount = 0;
        begun = TRUE;
    }
}

//Save slave address for use in I2C_Transaction
void TwoWire::begin(uint8_t address)
{
    WireContext *wc = getWireContext();

    wc->i2cTransaction.slaveAddress = address;

    begin();
}

void TwoWire::begin(int address)
{
    begin((uint8_t)address);
}

void TwoWire::end()
{
    begun = false;
    I2C_close(i2c);
}

void TwoWire::beginTransmission(uint8_t address)
{
    WireContext *wc = getWireContext();

    GateMutex_enter(GateMutex_handle(&gate));
    gateEnterCount++;

    if (wc->idle) {
        wc->i2cTransaction.slaveAddress = address;
        wc->idle = false;
    }
}

void TwoWire::beginTransmission(int address)
{
    beginTransmission((uint8_t)address);
}

uint8_t TwoWire::endTransmission(uint8_t sendStop)
{
    bool ret;
    WireContext *wc = getWireContext();

    if (i2c == NULL) {
        return (4); /* 4 = 'other error' */
    }
    
    ret = I2C_transfer(i2c, &(wc->i2cTransaction));

    wc->txWriteIndex = 0;

    wc->i2cTransaction.writeCount = 0;
    wc->rxReadIndex = 0; //i2cTransaction will overwrite buffer staring at 0
    wc->rxWriteIndex = wc->i2cTransaction.readCount;

    wc->i2cTransaction.readCount = 0;

    if (sendStop) {
        wc->idle = true;
    }

    if (gateEnterCount) {
        GateMutex_leave(GateMutex_handle(&gate), --gateEnterCount);
    }

    /* success = 0; 4 = other error */
    return (ret ? 0 : 4);
}

//  This provides backwards compatibility with the original
//  definition, and expected behaviour, of endTransmission
//
uint8_t TwoWire::endTransmission(void)
{
    return (endTransmission(true));
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop)
{
    if (quantity > BUFFER_LENGTH) {
        quantity = BUFFER_LENGTH;
    }
    if (!quantity) {
        return (0);
    }
    WireContext *wc = getWireContext();

    beginTransmission(address);

    wc->i2cTransaction.readCount = quantity;

    /* if != 0; then error occurred */
    return (endTransmission(sendStop) ? 0 : quantity);
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity)
{
    return (requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true));
}
uint8_t TwoWire::requestFrom(int address, int quantity)
{
    return (requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true));
}
uint8_t TwoWire::requestFrom(int address, int quantity, int sendStop)
{
    return (requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)sendStop));
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
size_t TwoWire::write(uint8_t data)
{
    WireContext *wc = getWireContext();

    // if(transmitting){
    // in master transmitter mode
    // don't bother if buffer is full
    if (TX_BUFFER_FULL){
        setWriteError();
        return 0;
    }

    // put byte in tx buffer
    wc->txBuffer[wc->txWriteIndex] = data;
    wc->txWriteIndex = (wc->txWriteIndex + 1) % BUFFER_LENGTH;
    wc->i2cTransaction.writeCount = wc->txWriteIndex;

    return (1);
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
size_t TwoWire::write(const uint8_t *data, size_t quantity)
{
    for (size_t i = 0; i < quantity; i++) {
        write(data[i]);
    }
    return (quantity);
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int TwoWire::available(void)
{
    WireContext *wc = getWireContext();

    return ((wc->rxWriteIndex >= wc->rxReadIndex) ?
        (wc->rxWriteIndex - wc->rxReadIndex) : BUFFER_LENGTH - (wc->rxReadIndex - wc->rxWriteIndex));
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int TwoWire::read(void)
{
    int value;
    WireContext *wc = getWireContext();

    if (RX_BUFFER_EMPTY) {
        return -1;
    }

    value = wc->rxBuffer[wc->rxReadIndex];
    wc->rxReadIndex = (wc->rxReadIndex + 1) % BUFFER_LENGTH;

    return (value);
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int TwoWire::peek(void)
{
    int value = -1;
    WireContext *wc = getWireContext();

    if(!RX_BUFFER_EMPTY){
        value = wc->rxBuffer[wc->rxReadIndex];
    }

    return (value);
}
void TwoWire::flush(void)
{
    WireContext *wc = getWireContext();

    wc->txWriteIndex = 0;
    wc->rxReadIndex = wc->rxWriteIndex;
}

// sets function called on slave write
// we can use a call back for this
void TwoWire::onReceive( void (*function)(int) )
{
    user_onReceive = function;
}

// sets function called on slave read
void TwoWire::onRequest( void (*function)(void) )
{
    user_onRequest = function;
}

void TwoWire::setModule(unsigned long _i2cModule)
{
    WireContext *wc = getWireContext();

    i2cModule = _i2cModule;
    if (wc->i2cTransaction.slaveAddress != 0) {
        begin(wc->i2cTransaction.slaveAddress);
    }
    else {
        begin();
    }
}
