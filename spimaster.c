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

#define SPI_MSG_LENGTH  (20)

unsigned char masterRxBuffer[SPI_MSG_LENGTH];
unsigned char masterTxBuffer[SPI_MSG_LENGTH];

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

// ------------------------------------

// ---------- REGISTERS  SETTINGS ---------------

// ----------------------------------------------

static Display_Handle display;

bool breakProgramLoop = false;
bool breakMeasureLoop = false;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} RawData_t;

typedef struct {
    float x;
    float y;
    float z;
} AccData_t;

void gpioButtonFxn0(uint_least8_t index)
{
    if (breakMeasureLoop){
        breakMeasureLoop = false;
    }else{
        breakMeasureLoop = true;
    }
}

void gpioButtonFxn1(uint_least8_t index)
{
    breakProgramLoop = true;
}

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
    }
    else {
        Display_printf(display, 0, 0, "Unsuccessful master SPI transfer");
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
        Display_printf(display, 0, 0, "Master sent: %x to address %x", masterTxBuffer[1], address);
    }
    else {
        Display_printf(display, 0, 0, "Unsuccessful master SPI transfer");
    }
}

void readRawData(SPI_Handle handle, RawData_t *data){

    uint8_t rxBuffer[6];
    int16_t rawX = 0x0000;
    int16_t rawY = 0x0000;
    int16_t rawZ = 0x0000;

    memset((void *) rxBuffer, 0, 6);

    readAddress(handle, DATAX0, rxBuffer, 6);

    rawX = (rxBuffer[1]<<8) | rxBuffer[0];
    rawY = (rxBuffer[3]<<8) | rxBuffer[2];
    rawZ = (rxBuffer[5]<<8) | rxBuffer[4];

    data->x = rawX;
    data->y = rawY;
    data->z = rawZ;

}

void readAccData(SPI_Handle handle, AccData_t *data){

    RawData_t rawData;
    readRawData(handle, &rawData);

    data->x = ((float) rawData.x * 3.9 * 9.81) / 1000;
    data->y = ((float) rawData.y * 3.9 * 9.81) / 1000;
    data->z = ((float) rawData.z * 3.9 * 9.81) / 1000;

}

void initAccelerometer(SPI_Handle handle){
    //------ STANDBY MODE FOR CHANGING SETTINGS ---------
    writeAddress(handle, POWER_CTL, 0x00);

    //--------- CHANGE DATA FORMAT ----------------
    /*        D1    D0
     * -----------------
     * ±2   |  0  |  0  |
     * ±4   |  0  |  1  |
     * ±8   |  1  |  0  |
     * ±16  |  1  |  1  |
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
 *  ======== masterThread ========
 *  Master SPI sends a message to slave while simultaneously receiving a
 *  message from the slave.
 */
void *masterThread(void *arg0)
{
    SPI_Handle      masterSpi;
    SPI_Params      spiParams;
    AccData_t       accData;
    RawData_t       rawData;

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


    uint8_t deviceID;
    readAddress(masterSpi, 0x00, &deviceID, (uint8_t) 1);
    Display_printf(display, 0, 0, "Device ID: %x", deviceID);


    //------ Initialize the accelerometer ---------
    initAccelerometer(masterSpi);

    while(!breakProgramLoop){

        while(!breakMeasureLoop){

            /* Toggle user LED, indicating a SPI transfer is in progress */
            GPIO_toggle(Board_GPIO_LED1);

            readRawData(masterSpi, &rawData);
            Display_printf(display, 0, 0, "RAW  X: %d, Y: %d, Z: %d", rawData.x, rawData.y, rawData.z);
            readAccData(masterSpi, &accData);
            Display_printf(display, 0, 0, "ACC  X: %f, Y: %f, Z: %f", accData.x, accData.y, accData.z);

            /* Sleep for a bit before starting the next SPI transfer  */
            //sleep(1);
        }

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

    GPIO_setConfig(Board_GPIO_BUTTON1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setCallback(Board_GPIO_BUTTON1, gpioButtonFxn1);
    GPIO_enableInt(Board_GPIO_BUTTON1);

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