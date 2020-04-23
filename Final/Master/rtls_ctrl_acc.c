/*
 * rtls_ctrl_acc.c
 *
 *  Created on: 19 mars 2020
 *      Author: liper
 */

#include <stdint.h>

#include "rtls_ctrl_acc.h"
#include "rtls_host.h"
#include "rtls_ctrl.h"
#include "rtls_ctrl_api.h"

#define DATA_LENGTH     (10)
#define SEGMENT_LENGTH  (6)

#define Xdir            (1)
#define Ydir            (2)
#define Zdir            (3)

/*********************************************************************
 * @fn      RTLSMaster_processAccDataRaw
 *
 * @brief
 *
 * @param   uint8_t *pMsg
 *
 * @return  none
 */
void RTLSMaster_processAccDataRaw(uint8_t *pData)
{
    rtlsAccResultRaw_t rawData;
    rawData.X0 = pData[0];
    rawData.X1 = pData[1];
    rawData.Y0 = pData[2];
    rawData.Y1 = pData[3];
    rawData.Z0 = pData[4];
    rawData.Z1 = pData[5];

    RTLSHost_sendMsg(RTLS_CMD_ACC_RESULT_RAW, HOST_ASYNC_RSP, (uint8_t *)&rawData, sizeof(rtlsAccResultRaw_t));
}

void RTLSMaster_processAccData(uint8_t *pData)
{
    uint8_t i=0;
    rtlsAccResult_t accData;

    //for(i=0; i < DATA_LENGTH; i+=SEGMENT_LENGTH){
        accData.X = (pData[1+i]<<8) | pData[0+i];
        accData.Y = (pData[3+i]<<8) | pData[2+i];
        accData.Z = (pData[5+i]<<8) | pData[4+i];

        RTLSHost_sendMsg(RTLS_CMD_ACC_RESULT, HOST_ASYNC_RSP, (uint8_t *)&accData, sizeof(rtlsAccResult_t));
    //}
}

void RTLSMaster_slaveRssi(int8_t rssi)
{
    RTLSHost_sendMsg(RTLS_CMD_RSSI_RESULT, HOST_ASYNC_RSP, (uint8_t *)&rssi, sizeof(rssi));
}

void RTLSCtrl_accDebug(void)
{
  rtlsStatus_e status = RTLS_FAIL;
  RTLSHost_sendMsg(RTLS_CMD_ACC_ENABLE, HOST_SYNC_RSP, (uint8_t *)&status, sizeof(rtlsStatus_e));
}
