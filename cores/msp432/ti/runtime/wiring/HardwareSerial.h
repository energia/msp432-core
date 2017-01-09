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

#ifndef HardwareSerial_h
#define HardwareSerial_h

#include <inttypes.h>
#include "Stream.h"

#include <ti/drivers/UART.h>

#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/gates/GateMutex.h>

#define SERIAL_RX_BUFFER_SIZE  128
#define SERIAL_TX_BUFFER_SIZE  128

class HardwareSerial : public Stream
{

    private:
        bool begun;
        bool blockingModeEnabled;
        unsigned char rxBuffer[SERIAL_RX_BUFFER_SIZE];
        volatile unsigned long rxWriteIndex;
        volatile unsigned long rxReadIndex;
        unsigned char txBuffer[SERIAL_TX_BUFFER_SIZE];
        volatile unsigned long txWriteIndex;
        volatile unsigned long txReadIndex;
        volatile bool txActive;
        unsigned long baudRate;
        uint8_t uartModule;
        UART_Handle uart;
        UART_Callback rxCallback;
        UART_Callback txCallback;
        GateMutex_Struct gate;
        void init(unsigned long module, UART_Callback rxCallback, UART_Callback txCallback);
        void flushAll(void);
        void primeTx(void);

    public:
        operator bool();// Arduino compatibility (see StringLength example)
        HardwareSerial(void);
        HardwareSerial(unsigned long);
        HardwareSerial(unsigned long, UART_Callback, UART_Callback);
        void begin(unsigned long);
        void begin(unsigned long, bool);
        void setModule(unsigned long);
        void setPins(unsigned long);
        void acquire(void);  /* acquire serial port for this thread */
        void release(void);  /* release serial port */
        void end(void);
        virtual int available(void);
        virtual int peek(void);
        virtual int read(void);
        virtual void flush(void);
        void readCallback(UART_Handle uart, void *buf, size_t count);
        void writeCallback(UART_Handle uart, void *buf, size_t count);
        virtual size_t write(uint8_t c);
        virtual size_t write(const uint8_t *buffer, size_t size);
        using Print::write; // pull in write(str) from Print
};

extern HardwareSerial Serial;
extern HardwareSerial Serial1;

extern void serialEventRun(void) __attribute__((weak));
extern void serialEventRun1(void) __attribute__((weak));

extern void serialEvent() __attribute__((weak));
extern void serialEvent1() __attribute__((weak));

#endif
