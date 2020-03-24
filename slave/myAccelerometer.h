/**********************************************************************************************
 * Filename:       myAccelerometer.h
 *
 * Description:    This file contains the myAccelerometer service definitions and
 *                 prototypes.
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


#ifndef _MYACCELEROMETER_H_
#define _MYACCELEROMETER_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
* CONSTANTS
*/
// Service UUID
#define MYACCELEROMETER_SERV_UUID 0xFFF0

//  Characteristic defines
#define MYACCELEROMETER_MYACCELEROMETERCHAR_ID   0
#define MYACCELEROMETER_MYACCELEROMETERCHAR_UUID 0xFFF1
#define MYACCELEROMETER_MYACCELEROMETERCHAR_LEN  6

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*myAccelerometerChange_t)(uint16_t connHandle, uint8_t paramID, uint16_t len, uint8_t *pValue);

typedef struct
{
  myAccelerometerChange_t        pfnChangeCb;  // Called when characteristic value changes
  myAccelerometerChange_t        pfnCfgChangeCb;
} myAccelerometerCBs_t;



/*********************************************************************
 * API FUNCTIONS
 */


/*
 * MyAccelerometer_AddService- Initializes the MyAccelerometer service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t MyAccelerometer_AddService( uint8_t rspTaskId);

/*
 * MyAccelerometer_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t MyAccelerometer_RegisterAppCBs( myAccelerometerCBs_t *appCallbacks );

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
extern bStatus_t MyAccelerometer_SetParameter(uint8_t param, uint16_t len, void *value);

/*
 * MyAccelerometer_GetParameter - Get a MyAccelerometer parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t MyAccelerometer_GetParameter(uint8_t param, uint16_t *len, void *value);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _MYACCELEROMETER_H_ */
