/*
 * Copyright (c) 2018-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
 *  ======== spimaster.c ========
 */
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* POSIX Header files */
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/display/Display.h>

/* Example/Board Header files */
#include "Board.h"

#define THREADSTACKSIZE (1024)

#define SPI_MSG_LENGTH  (2)

#define MAX_LOOP        (10)


// ---------- ADDRESSES -----------
#define POWER_CTL       (0x2d)
#define DATA_FORMAT     (0x31)
#define FIFO_CTL        (0x38)

#define DATAX0          (0x32)
#define DATAX1          (0x33)

#define DATAY0          (0x34)
#define DATAY1          (0x35)

#define DATAZ0          (0x36)
#define DATAZ1          (0x37)
// --------------------------------

static Display_Handle display;

unsigned char masterRxBuffer[SPI_MSG_LENGTH];
unsigned char masterTxBuffer[SPI_MSG_LENGTH];

bool breakLoop = false;

void gpioButtonFxn0(uint_least8_t index)
{
    breakLoop = true;
}

void readAddress(SPI_Handle handle, SPI_Transaction *transaction, uint8_t address, uint8_t *p_data, uint8_t bytes){


    bool transferOK;
    uint8_t masterTxBuffer[7];
    memset((void *) masterTxBuffer, 0, 7);

    if(bytes > 1){
        masterTxBuffer[0] = (address | 0x80) | 0x40;
    }
    else{
        masterTxBuffer[0] = address | 0x80;
    }

    transaction->count = bytes+1;
    transaction->txBuf = (void *) masterTxBuffer;
    transaction->rxBuf = (void *) p_data;

    transferOK = SPI_transfer(handle, transaction);
    if (transferOK) {
        //Display_printf(display, 0, 0, "Rx Buffer 1: %x   2: %x   3: %x   4: %x   5: %x   6: %x", p_data[0], p_data[1], p_data[2], p_data[3], p_data[4], p_data[5]);
    }
    else {
        Display_printf(display, 0, 0, "Unsuccessful master SPI transfer");
    }

}

void writeAddress(SPI_Handle handle, SPI_Transaction *transaction, uint8_t address, uint8_t data){

    bool transferOK;
    uint8_t masterTxBuffer[2];
    masterTxBuffer[0] = address & 0x3f;
    masterTxBuffer[1] = data;
    uint8_t masterRxBuffer[2];
    memset((void *) masterRxBuffer, 0, 2);

    transaction->count = 2;
    transaction->txBuf = (void *) masterTxBuffer;
    transaction->rxBuf = (void *) masterRxBuffer;

    transferOK = SPI_transfer(handle, transaction);
    if (transferOK) {
        Display_printf(display, 0, 0, "Master sent: %x to address %x", masterTxBuffer[1], address);
    }
    else {
        Display_printf(display, 0, 0, "Unsuccessful master SPI transfer");
    }
}

void readRawData(SPI_Handle handle, SPI_Transaction *transaction){

    uint8_t rxBuffer[7];
    int16_t rawX = 0x0000;
    int16_t rawY = 0x0000;
    int16_t rawZ = 0x0000;

    memset((void *) rxBuffer, 0, 7);

    readAddress(handle, transaction, DATAX0, rxBuffer, 7);

    rawX = (rxBuffer[2]<<8) | rxBuffer[1];
    rawY = (rxBuffer[4]<<8) | rxBuffer[3];
    rawZ = (rxBuffer[6]<<8) | rxBuffer[5];

    Display_printf(display, 0, 0, "X: %f, Y: %f, Z: %f", (float) 3.9 * rawX, (float) 3.9 *  rawY, (float) 3.9 *  rawZ);

}

/*
 *  ======== masterThread ========
 *  Master SPI sends a message to slave while simultaneously receiving a
 *  message from the slave.
 */
void *masterThread(void *arg0)
{
    SPI_Handle      masterSpi;
    SPI_Params      spiParams;
    SPI_Transaction transaction;

    /* Open SPI as master (default) */
    SPI_Params_init(&spiParams);
    spiParams.frameFormat = SPI_POL1_PHA1;
    spiParams.dataSize = 8;
    spiParams.bitRate = 10000;
    masterSpi = SPI_open(Board_SPI_MASTER, &spiParams);
    if (masterSpi == NULL) {
        Display_printf(display, 0, 0, "Error initializing master SPI\n");
        while (1);
    }
    else {
        Display_printf(display, 0, 0, "Master SPI initialized\n");
    }
    uint8_t deviceID[2];
    readAddress(masterSpi, &transaction, 0x00, deviceID, (uint8_t) 1);
    Display_printf(display, 0, 0, "Device ID: %x", deviceID[1]);


    //------ STANDBY MODE FOR CHANGING SETTINGS ---------
    writeAddress(masterSpi, &transaction, POWER_CTL, 0x00);

    //--------- CHANGE DATA FORMAT ----------------
    writeAddress(masterSpi, &transaction, DATA_FORMAT, 0x00);

    //----------- FIFO SETTING -----------
    // 0x00 - default
    // 0x9f - stream mode, trigger 0, samples 1
    writeAddress(masterSpi, &transaction, FIFO_CTL, 0x9f);

    //----------- TURN ON MEASUREMENT -----------
    writeAddress(masterSpi, &transaction, POWER_CTL, 0x08);

    while(!breakLoop){

        /* Toggle user LED, indicating a SPI transfer is in progress */
        GPIO_toggle(Board_GPIO_LED1);

        readRawData(masterSpi, &transaction);

        /* Sleep for a bit before starting the next SPI transfer  */
        //sleep(1);
    }

    SPI_close(masterSpi);

    Display_printf(display, 0, 0, "\nDone");

    return (NULL);
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    pthread_t           thread0;
    pthread_attr_t      attrs;
    struct sched_param  priParam;
    int                 retc;
    int                 detachState;

    /* Call driver init functions. */
    Display_init();
    GPIO_init();
    SPI_init();

    /* Configure the LED pins */
    GPIO_setConfig(Board_GPIO_LED0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(Board_GPIO_LED1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    GPIO_setConfig(Board_GPIO_BUTTON0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setCallback(Board_GPIO_BUTTON0, gpioButtonFxn0);
    GPIO_enableInt(Board_GPIO_BUTTON0);

    /* Open the display for output */
    display = Display_open(Display_Type_UART, NULL);
    if (display == NULL) {
        /* Failed to open display driver */
        while (1);
    }

    /* Turn on user LED */
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);

    Display_printf(display, 0, 0, "Starting the SPI master example");

    /* Create application threads */
    pthread_attr_init(&attrs);

    detachState = PTHREAD_CREATE_DETACHED;
    /* Set priority and stack size attributes */
    retc = pthread_attr_setdetachstate(&attrs, detachState);
    if (retc != 0) {
        /* pthread_attr_setdetachstate() failed */
        while (1);
    }

    retc |= pthread_attr_setstacksize(&attrs, THREADSTACKSIZE);
    if (retc != 0) {
        /* pthread_attr_setstacksize() failed */
        while (1);
    }

    /* Create master thread */
    priParam.sched_priority = 1;
    pthread_attr_setschedparam(&attrs, &priParam);

    retc = pthread_create(&thread0, &attrs, masterThread, NULL);
    if (retc != 0) {
        /* pthread_create() failed */
        while (1);
    }

    return (NULL);
}



