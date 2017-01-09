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
#include <stdio.h>
#include <string.h>

#include <ti/sysbios/family/arm/m3/Hwi.h>

#include "wiring_private.h"
#include "HardwareSerial.h"

#define RX_BUFFER_EMPTY   (rxReadIndex == rxWriteIndex)
#define RX_BUFFER_FULL    (((rxWriteIndex + 1) % SERIAL_RX_BUFFER_SIZE) == rxReadIndex)

#define TX_BUFFER_EMPTY   (txReadIndex == txWriteIndex)
#define TX_BUFFER_FULL    (((txWriteIndex + 1) % SERIAL_TX_BUFFER_SIZE) == txReadIndex)

HardwareSerial::HardwareSerial(void)
{
    init(0, NULL, NULL);
}

HardwareSerial::HardwareSerial(unsigned long module)
{
    init(module, NULL, NULL);
}

HardwareSerial::HardwareSerial(unsigned long module, UART_Callback rCallback, UART_Callback tCallback)
{
    init(module, rCallback, tCallback);
}

/*
 * Private Methods
 */
void HardwareSerial::init(unsigned long module, UART_Callback rCallback, UART_Callback tCallback)
{
    rxCallback = rCallback;
    txCallback = tCallback;

    /* by default, read() will not block */
    blockingModeEnabled = false;

    uartModule = module;
    begun = false;
}

void HardwareSerial::flushAll(void)
{
}

/*
 * Public Methods
 */
void HardwareSerial::begin(unsigned long baud)
{
    UART_Params uartParams;

    if (begun == TRUE) return;

    baudRate = baud;

    UART_Params_init(&uartParams);

    if (blockingModeEnabled == true) {
        uartParams.readMode = UART_MODE_BLOCKING;
        uartParams.writeMode = UART_MODE_BLOCKING;
    }
    else {
        uartParams.readMode = UART_MODE_CALLBACK;
        uartParams.readCallback = rxCallback;
        uartParams.writeMode = UART_MODE_CALLBACK;
        uartParams.writeCallback = txCallback;
        rxWriteIndex = 0;
        rxReadIndex = 0;
        txWriteIndex = 0;
        txReadIndex = 0;
        txActive = false;
    }

    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.baudRate = baud;

    uart = Board_openUART(uartModule, &uartParams);

    if (uart != NULL) {
        GateMutex_construct(&gate, NULL);
        if (blockingModeEnabled == false) {
            /* start the read process */
            UART_read(uart, &rxBuffer[rxWriteIndex], 1);
        }
        begun = TRUE;
    }
}

void HardwareSerial::begin(unsigned long baud, bool blockModeEnabled)
{
    blockingModeEnabled = blockModeEnabled;
    begin(baud);
}

void HardwareSerial::setModule(unsigned long module)
{
    /* Change which pins UART is on */
    /* need to implement */
}

void HardwareSerial::setPins(unsigned long pins)
{
    /* almost same functionality as above */
}

void HardwareSerial::acquire(void)
{
    GateMutex_enter(GateMutex_handle(&gate));
}

void HardwareSerial::release(void)
{
    GateMutex_leave(GateMutex_handle(&gate), 0);
}

void HardwareSerial::end(void)
{
    begun = false;
    UART_close(uart);
    uart = NULL;
}

int HardwareSerial::available(void)
{
    int numChars;

    if (uart == NULL) {
        return (0);
    }

    if (blockingModeEnabled == true) {
        if (UART_control(uart, UART_CMD_GETRXCOUNT, (void *)&numChars)
         == UART_STATUS_SUCCESS) {
            return (numChars);
        }
        else {
            return (0);
        }
    }
    else {
        unsigned int key;

        key = Hwi_disable();

        if (RX_BUFFER_EMPTY) {
            /* kick off another character read operation */
            UART_read(uart, &rxBuffer[rxWriteIndex], 1);
        }

        numChars = (rxWriteIndex >= rxReadIndex) ?
            (rxWriteIndex - rxReadIndex)
            : SERIAL_RX_BUFFER_SIZE - (rxReadIndex - rxWriteIndex);

        Hwi_restore(key);

        return (numChars);
    }
 }

int HardwareSerial::peek(void)
{
    int iChar;

    if (uart == NULL) {
        return (-1);
    }

    if (blockingModeEnabled == true) {
        if (UART_control(uart, UART_CMD_PEEK, (void *)&iChar)
             == UART_STATUS_SUCCESS) {
            return (iChar);
        }
        else {
            return (-1);
        }
    }
    else {
        if (available() == 0) {
            return (-1);
        }

        /* Read a character from the buffer. */
        iChar = (int)rxBuffer[rxReadIndex];

        /* Return the character to the caller. */
        return (iChar);
    }
}

int HardwareSerial::read(void)
{
    int iChar;

    if (uart == NULL) {
        return (0);
    }

    if (blockingModeEnabled == true) {
        UART_read(uart, &iChar, 1);
        return (iChar);
    }
    else {
        unsigned int hwiKey;

        hwiKey = Hwi_disable();

        iChar = peek();

        /* if a character is available, update buffer ptrs */
        if (iChar != -1) {
            rxReadIndex = ((rxReadIndex) + 1) % SERIAL_RX_BUFFER_SIZE;
        }

        Hwi_restore(hwiKey);

        return (iChar);
    }
}

void HardwareSerial::flush()
{
    if (blockingModeEnabled == false) {
        while (txActive) ;
    }
}

HardwareSerial::operator bool()
{
    return true;  // Arduino compatibility (see StringLength example)
}

size_t HardwareSerial::write(uint8_t c)
{
    return (write(&c, 1));
}

size_t HardwareSerial::write(const uint8_t *buffer, size_t size)
{
    if (uart == NULL || size == 0) {
        return (0);
    }

    if (blockingModeEnabled == true) {
        IArg key;
        key = GateMutex_enter(GateMutex_handle(&gate));
        UART_write(uart, (char *)buffer, size);
        GateMutex_leave(GateMutex_handle(&gate), key);
    }
    else {
        unsigned int hwiKey;
        while (size) {
            while (TX_BUFFER_FULL) {
                if (txActive == false) {
                    primeTx();
                }
            }
            hwiKey = Hwi_disable();
            txBuffer[txWriteIndex] = *buffer++;
            txWriteIndex = (txWriteIndex + 1) % SERIAL_TX_BUFFER_SIZE;
            size--;
            Hwi_restore(hwiKey);
        }
        if ((txActive == false) && (!TX_BUFFER_EMPTY)) {
            primeTx();
        }
    }

    return (size);
}

void HardwareSerial::primeTx(void) {
    unsigned int hwiKey;
    size_t size;

    hwiKey = Hwi_disable();

    /*
     * txActive might have become true if an interrupt
     * that calls Serial.print() has occurred just prior
     * to the call to primeTx().
     */
    if (txActive == true) {
        Hwi_restore(hwiKey);
        return;
    }
    
    if (txWriteIndex > txReadIndex) {
        size = txWriteIndex - txReadIndex;
    }
    else {
        size = SERIAL_TX_BUFFER_SIZE - txReadIndex;
    }

    txActive = true;

    Hwi_restore(hwiKey);

    UART_write(uart, &txBuffer[txReadIndex], size);
}

void HardwareSerial::readCallback(UART_Handle uart, void *buf, size_t count)
{
    uint8_t volatile full = RX_BUFFER_FULL;

    rxWriteIndex = ((rxWriteIndex) + 1) % SERIAL_RX_BUFFER_SIZE;

    if (full) {
        rxReadIndex = ((rxReadIndex) + 1) % SERIAL_RX_BUFFER_SIZE;
    }
}

void HardwareSerial::writeCallback(UART_Handle uart, void *buf, size_t txCount)
{
    unsigned int hwiKey;
    size_t size;

    hwiKey = Hwi_disable();

    /*
     * advance txReadIndex by the number of bytes
     * sent by the last call to UART_write().
     */
    txReadIndex = (txReadIndex + txCount) % SERIAL_TX_BUFFER_SIZE;

    if (TX_BUFFER_EMPTY) {
        txActive = false;
        Hwi_restore(hwiKey);
        return;
    }

    if (txWriteIndex > txReadIndex) {
        size = txWriteIndex - txReadIndex;
    }
    else {
        size = SERIAL_TX_BUFFER_SIZE - txReadIndex;
    }

    Hwi_restore(hwiKey);

    UART_write(uart, &txBuffer[txReadIndex], size);
}

void serialEvent() __attribute__((weak));
void serialEvent() { }

void serialEvent1() __attribute__((weak));
void serialEvent1() { }

void serialEventRun(void)
{
    if (Serial.available()) {
        serialEvent();
    }
}

void serialEventRun1(void)
{
    if (Serial1.available()) {
        serialEvent1();
    }
}
