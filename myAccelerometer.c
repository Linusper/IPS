/**********************************************************************************************
 * Filename:       myAccelerometer.c
 *
 * Description:    This file contains the implementation of the service.
 *
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
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
 *
 *************************************************************************************************/


/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <icall.h>

/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "myAccelerometer.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
* GLOBAL VARIABLES
*/

// myAccelerometer Service UUID
CONST uint8_t myAccelerometerUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(MYACCELEROMETER_SERV_UUID), HI_UINT16(MYACCELEROMETER_SERV_UUID)
};

// myAccelerometerChar UUID
CONST uint8_t myAccelerometer_MyAccelerometerCharUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(MYACCELEROMETER_MYACCELEROMETERCHAR_UUID), HI_UINT16(MYACCELEROMETER_MYACCELEROMETERCHAR_UUID)
};

/*********************************************************************
 * LOCAL VARIABLES
 */

static myAccelerometerCBs_t *pAppCBs = NULL;

/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
static CONST gattAttrType_t myAccelerometerDecl = { ATT_BT_UUID_SIZE, myAccelerometerUUID };

// Characteristic "MyAccelerometerChar" Properties (for declaration)
static uint8_t myAccelerometer_MyAccelerometerCharProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "MyAccelerometerChar" Value variable
static uint8_t myAccelerometer_MyAccelerometerCharVal[MYACCELEROMETER_MYACCELEROMETERCHAR_LEN] = {0};

// Characteristic "MyAccelerometerChar" CCCD
static gattCharCfg_t *myAccelerometer_MyAccelerometerCharConfig;

/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t myAccelerometerAttrTbl[] =
{
  // myAccelerometer Service Declaration
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8_t *)&myAccelerometerDecl
  },
    // MyAccelerometerChar Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &myAccelerometer_MyAccelerometerCharProps
    },
      // MyAccelerometerChar Characteristic Value
      {
        { ATT_BT_UUID_SIZE, myAccelerometer_MyAccelerometerCharUUID },
        GATT_PERMIT_READ,
        0,
        myAccelerometer_MyAccelerometerCharVal
      },
      // MyAccelerometerChar CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&myAccelerometer_MyAccelerometerCharConfig
      },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t myAccelerometer_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                           uint16_t maxLen, uint8_t method );
static bStatus_t myAccelerometer_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                            uint8_t *pValue, uint16_t len, uint16_t offset,
                                            uint8_t method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t myAccelerometerCBs =
{
  myAccelerometer_ReadAttrCB,  // Read callback function pointer
  myAccelerometer_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*
 * MyAccelerometer_AddService- Initializes the MyAccelerometer service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t MyAccelerometer_AddService( uint8_t rspTaskId )
{
  uint8_t status;

  // Allocate Client Characteristic Configuration table
  myAccelerometer_MyAccelerometerCharConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( myAccelerometer_MyAccelerometerCharConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, myAccelerometer_MyAccelerometerCharConfig );
  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( myAccelerometerAttrTbl,
                                        GATT_NUM_ATTRS( myAccelerometerAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &myAccelerometerCBs );

  return ( status );
}

/*
 * MyAccelerometer_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t MyAccelerometer_RegisterAppCBs( myAccelerometerCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    pAppCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*
 * MyAccelerometer_SetParameter - Set a MyAccelerometer parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t MyAccelerometer_SetParameter( uint8_t param, uint16_t len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case MYACCELEROMETER_MYACCELEROMETERCHAR_ID:
      if ( len == MYACCELEROMETER_MYACCELEROMETERCHAR_LEN )
      {
        memcpy(myAccelerometer_MyAccelerometerCharVal, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( myAccelerometer_MyAccelerometerCharConfig, (uint8_t *)&myAccelerometer_MyAccelerometerCharVal, FALSE,
                                    myAccelerometerAttrTbl, GATT_NUM_ATTRS( myAccelerometerAttrTbl ),
                                    INVALID_TASK_ID,  myAccelerometer_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*
 * MyAccelerometer_GetParameter - Get a MyAccelerometer parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t MyAccelerometer_GetParameter( uint8_t param, uint16_t *len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*********************************************************************
 * @fn          myAccelerometer_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t myAccelerometer_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                       uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                       uint16_t maxLen, uint8_t method )
{
  bStatus_t status = SUCCESS;

  // See if request is regarding the MyAccelerometerChar Characteristic Value
if ( ! memcmp(pAttr->type.uuid, myAccelerometer_MyAccelerometerCharUUID, pAttr->type.len) )
  {
    if ( offset > MYACCELEROMETER_MYACCELEROMETERCHAR_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, MYACCELEROMETER_MYACCELEROMETERCHAR_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has READ permissions.
    *pLen = 0;
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  return status;
}


/*********************************************************************
 * @fn      myAccelerometer_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t myAccelerometer_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                        uint8_t *pValue, uint16_t len, uint16_t offset,
                                        uint8_t method )
{
  bStatus_t status  = SUCCESS;
  uint8_t   paramID = 0xFF;

  // See if request is regarding a Client Characterisic Configuration
  if ( ! memcmp(pAttr->type.uuid, clientCharCfgUUID, pAttr->type.len) )
  {
    // Allow only notifications.
    status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                             offset, GATT_CLIENT_CFG_NOTIFY);
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has WRITE permissions.
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  // Let the application know something changed (if it did) by using the
  // callback it registered earlier (if it did).
  if (paramID != 0xFF)
    if ( pAppCBs && pAppCBs->pfnChangeCb )
      pAppCBs->pfnChangeCb(connHandle, paramID, len, pValue); // Call app function from stack task context.

  return status;
}
