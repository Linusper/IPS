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
    rtlsAccResult_t accData;
    accData.X = (pData[1]<<8) | pData[0];
    accData.Y = (pData[3]<<8) | pData[2];
    accData.Z = (pData[5]<<8) | pData[4];

    RTLSHost_sendMsg(RTLS_CMD_ACC_RESULT, HOST_ASYNC_RSP, (uint8_t *)&accData, sizeof(rtlsAccResult_t));
}

void RTLSCtrl_accDebug(void)
{
  rtlsStatus_e status = RTLS_FAIL;
  RTLSHost_sendMsg(RTLS_CMD_ACC_ENABLE, HOST_SYNC_RSP, (uint8_t *)&status, sizeof(rtlsStatus_e));
}
