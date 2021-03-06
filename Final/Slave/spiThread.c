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
//#include <float.h>

#include <xdc/std.h>
#include <xdc/runtime/Error.h>

#include <ti/sysbios/BIOS.h>
//#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>

/* Driver Header files */

#include <ti/drivers/SPI.h>
#include <ti/display/Display.h>

/* Example/Board Header files */
#include "Board.h"

Task_Struct spiTask;
#define THREADSTACKSIZE    500
uint8_t spiTaskStack[THREADSTACKSIZE];

#define SPI_MSG_LENGTH  (10)
#define ACC_DATA_LENGTH (6)
#define BUFFER_LENGTH (3)

// ---------- REGISTERS ---------------

#define DEVID           (0x00) /**< Device ID */
#define THRESH_TAP      (0x1D) /**< Tap threshold */
#define OFSX            (0x1E) /**< X-axis offset */
#define OFSY            (0x1F) /**< Y-axis offset */
#define OFSZ            (0x20) /**< Z-axis offset */
#define DUR             (0x21) /**< Tap duration */
#define LATENT          (0x22) /**< Tap latency */
#define WINDOW          (0x23) /**< Tap window */
#define THRESH_ACT      (0x24) /**< Activity threshold */
#define THRESH_INACT    (0x25) /**< Inactivity threshold */
#define TIME_INACT      (0x26) /**< Inactivity time */
#define ACT_INACT_CTL   (0x27) /**< Axis enable control for activity and inactivity detection */
#define THRESH_FF       (0x28) /**< Free-fall threshold */
#define TIME_FF         (0x29) /**< Free-fall time */
#define TAP_AXES        (0x2A) /**< Axis control for single/double tap */
#define ACT_TAP_STATUS  (0x2B) /**< Source for single/double tap */
#define BW_RATE         (0x2C) /**< Data rate and power mode control */
#define POWER_CTL       (0x2D) /**< Power-saving features control */
#define INT_ENABLE      (0x2E) /**< Interrupt enable control */
#define INT_MAP         (0x2F) /**< Interrupt mapping control */
#define INT_SOURCE      (0x30) /**< Source of interrupts */
#define DATA_FORMAT     (0x31) /**< Data format control */
#define DATAX0          (0x32) /**< X-axis data 0 */
#define DATAX1          (0x33) /**< X-axis data 1 */
#define DATAY0          (0x34) /**< Y-axis data 0 */
#define DATAY1          (0x35) /**< Y-axis data 1 */
#define DATAZ0          (0x36) /**< Z-axis data 0 */
#define DATAZ1          (0x37) /**< Z-axis data 1 */
#define FIFO_CTL        (0x38) /**< FIFO control */
#define FIFO_STATUS     (0x39) /**< FIFO status */

#define XBIAS           10
#define YBIAS           2
#define ZBIAS           250
// ------------------------------------

// ---------- REGISTERS  SETTINGS ---------------

// ----------------------------------------------

// ---------- HANDLES ---------------
extern Display_Handle dispHandle;
//Semaphore_Handle spiSem;
// ----------------------------------


unsigned char masterRxBuffer[SPI_MSG_LENGTH];
unsigned char masterTxBuffer[SPI_MSG_LENGTH];

uint8_t dataToSend[ACC_DATA_LENGTH];

void readAddress(SPI_Handle handle, uint8_t address, uint8_t *p_data, uint8_t bytes){

    bool transferOK;
    SPI_Transaction transaction;

    memset((void *) masterTxBuffer, 0, SPI_MSG_LENGTH);
    memset((void *) masterRxBuffer, 0, SPI_MSG_LENGTH);

    if(bytes > 1){
        masterTxBuffer[0] = (address | 0x80) | 0x40;
    }
    else{
        masterTxBuffer[0] = address | 0x80;
    }

    transaction.count = bytes+1;
    transaction.txBuf = (void *) masterTxBuffer;
    transaction.rxBuf = (void *) masterRxBuffer;

    transferOK = SPI_transfer(handle, &transaction);
    if (transferOK) {
        uint8_t i;
        for (i = 0; i < bytes; i++){
            p_data[i] = masterRxBuffer[i+1];
        }
        //Display_print4(dispHandle, 7, 0, "Master received: %x, %x, %x, %x", masterRxBuffer[1], masterRxBuffer[2], masterRxBuffer[3], masterRxBuffer[4]);
    }
    else {
        Display_print0(dispHandle, 7, 0, "Unsuccessful master SPI transfer");
    }

}

void writeAddress(SPI_Handle handle, uint8_t address, uint8_t data){

    bool transferOK;
    SPI_Transaction transaction;

    masterTxBuffer[0] = address & 0x3f;
    masterTxBuffer[1] = data;

    transaction.count = 2;
    transaction.txBuf = (void *) masterTxBuffer;
    transaction.rxBuf = (void *) masterRxBuffer;

    transferOK = SPI_transfer(handle, &transaction);
    if (transferOK) {
        //Display_print2(dispHandle, 7, 0, "Master sent: %x to address %x", masterTxBuffer[1], address);
    }
    else {
        Display_print0(dispHandle, 7, 0, "Unsuccessful master SPI transfer");
    }
}

void readRawData(SPI_Handle handle, uint8_t *data){

    uint8_t size = 6;
    //uint8_t rxBuffer[size];

    memset((void *) data, 0, size);

    readAddress(handle, DATAX0, data, size);
 /*
    rawX = (rxBuffer[1]<<8) | rxBuffer[0];
    rawY = (rxBuffer[3]<<8) | rxBuffer[2];
    rawZ = (rxBuffer[5]<<8) | rxBuffer[4];
*/
}

void initAccelerometer(SPI_Handle handle){
    //------ STANDBY MODE FOR CHANGING SETTINGS ---------
    writeAddress(handle, POWER_CTL, 0x00);

    //--------- CHANGE DATA FORMAT ----------------
    /*            D1    D0
     *  -------------------
     * |  �2   |  0  |  0  |
     * |  �4   |  0  |  1  |
     * |  �8   |  1  |  0  |
     * |  �16  |  1  |  1  |
     *  -------------------
     */
    writeAddress(handle, DATA_FORMAT, 0x00);

    //----------- FIFO SETTING -----------
    // 0x00 - default
    // 0x9f - stream mode, trigger 0, samples 1
    writeAddress(handle, FIFO_CTL, 0x9f);

    //----------- TURN ON MEASUREMENT -----------
    writeAddress(handle, POWER_CTL, 0x08);
}
/*
void setValueToSend(uint8_t *txData, uint8_t *rawData, uint8_t n){
    uint8_t i;

    for (i = 0; i < ACC_DATA_LENGTH; i++){
        txData[i+(ACC_DATA_LENGTH * n)] = rawData[i];
    }
}
*/
void setValueToSend(uint8_t *txData, uint8_t *rawData){
    uint8_t i;

    for (i = 0; i < ACC_DATA_LENGTH; i++){
        txData[i] = rawData[i];
    }
}

void convertRawData(uint8_t *rawData, int16_t *data){
    data[0] = (rawData[1]<<8) | rawData[0];
    data[1] = (rawData[3]<<8) | rawData[2];
    data[2] = (rawData[5]<<8) | rawData[4];
}

void convertAccData(int16_t *accData, uint8_t *data){
    data[0] = accData[0] & 0x00FF;
    data[1] = (accData[0]>>8) & 0x00FF;

    data[2] = accData[1] & 0x00FF;
    data[3] = (accData[1]>>8) & 0x00FF;

    data[4] = accData[2] & 0x00FF;
    data[5] = (accData[2]>>8) & 0x00FF;
}

void filterData(int16_t *data, int16_t *oldData, uint8_t xbias, uint8_t ybias, uint8_t zbias, uint8_t min){
    uint8_t i;
    int16_t delta[3];

    data[0] -= xbias;
    data[1] -= ybias;
    data[2] -= zbias;
    for (i=0; i<3; i++){
        delta[i] = (data[i] - oldData[i])/2;
        if (delta[i] < min && delta[i] > -min){
            data[i] = oldData[i];
        }
        if (data[i] < min*2 && data[i] > -min*2){
            data[i] = 0;
        }
    }
}

/*
 *  ======== masterThread ========
 *  Master SPI sends a message to slave while simultaneously receiving a
 *  message from the slave.
 */
static void spiThread(UArg a0, UArg a1) {
    //while(1) Task_sleep(1e5);
    SPI_Handle          masterSpi;
    SPI_Params          spiParams;
    uint8_t             rawData[ACC_DATA_LENGTH];
    /*
    uint8_t             tempData[ACC_DATA_LENGTH * BUFFER_LENGTH];
    int16_t             accData[3];
    int16_t             oldAccData[3];
    uint8_t             n=0;
*/
  //Semaphore_Params    semParams;

    SPI_init();

/*
    Semaphore_Params_init(&semParams);
    spiSem = Semaphore_create(0, &semParams, Error_IGNORE);
    if(spiSem == NULL)
    {*/
        /* Semaphore_create() failed *//*
        Display_print0(dispHandle, 6, 0, "spiSem Semaphore creation failed\n");
        while (1);
    }*/

    Display_print0(dispHandle, 10, 0, "Starting the SPI master example");

    /* Open SPI as master (default) */
    SPI_Params_init(&spiParams);
    spiParams.frameFormat = SPI_POL1_PHA1;
    spiParams.dataSize = 8;
    spiParams.bitRate = 10000;
    masterSpi = SPI_open(Board_SPI0, &spiParams);
    if (masterSpi == NULL) {

        while (1) Display_print0(dispHandle, 11, 0, "Error initializing master SPI\n");
    }
    else {
        Display_print0(dispHandle, 11, 0, "Master SPI initialized\n");
    }

    //Semaphore_pend(spiSem, BIOS_WAIT_FOREVER);

    //------ Initialize the accelerometer ---------
    initAccelerometer(masterSpi);

    while(1){

        readRawData(masterSpi, rawData);
/*
        oldAccData[0] = accData[0];
        oldAccData[1] = accData[1];
        oldAccData[2] = accData[2];
        convertRawData(rawData, accData);
        filterData(accData, oldAccData, XBIAS, YBIAS, ZBIAS, 3);
        convertAccData(accData,rawData);
*/
        setValueToSend(dataToSend, rawData);
/*
        if (n == BUFFER_LENGTH - 1){
            n=0;
            memcpy(dataToSend, tempData, sizeof(dataToSend));
        } else{
            n++;
        }
*/
        Task_sleep(1e4);
    }
}

/*
 *  ======== mainThread ========
 */
void spiThread_create(void)
{
    Task_Params taskParams;

    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = spiTaskStack;
    taskParams.stackSize = THREADSTACKSIZE;
    taskParams.priority = 1;

   Task_construct(&spiTask, spiThread, &taskParams, Error_IGNORE);
}



