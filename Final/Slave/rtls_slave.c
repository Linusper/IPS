/******************************************************************************

 @file  rtls_slave.c

 @brief This file contains the TOF Responder sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack, based on the 
        Simple Peripheral sample application.

 Group: WCS, BTS
 Target Device: cc2640r2

 ******************************************************************************
 
 Copyright (c) 2018-2020, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/display/Display.h>

#if defined( DEBUG_SW_TRACE )
#include <driverlib/ioc.h>
#endif // DEBUG_SW_TRACE

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "devinfoservice.h"
#include "simple_gatt_profile.h"
#include "ll_common.h"

#include "peripheral.h"

#include "board_key.h"

#include "board.h"

#include "rtls_slave.h"

#include "rtls_ctrl_api.h"
#include "rtls_ble.h"

#include "ble_user_config.h"

#include "myAccelerometer.h"

/*********************************************************************
 * CONSTANTS
 */

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// General discoverable mode: advertise indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) for automatic
// parameter update request
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) for automatic
// parameter update request
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use for automatic parameter update request
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) for automatic parameter
// update request
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// After the connection is formed, the peripheral waits until the central
// device asks for its preferred connection parameters
#define DEFAULT_ENABLE_UPDATE_REQUEST         GAPROLE_LINK_PARAM_UPDATE_WAIT_REMOTE_PARAMS

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// How often to perform periodic event (in msec)
#define RTLS_SLAVE_PERIODIC_EVT_PERIOD        200

// Application specific event ID for HCI Connection Event End Events
#define RTLS_SLAVE_HCI_CONN_EVT_END_EVT       0x0001

// Type of Display to open
#if !defined(Display_DISABLE_ALL)
  #if defined(BOARD_DISPLAY_USE_LCD) && (BOARD_DISPLAY_USE_LCD!=0)
    #define RTLS_SLAVE_DISPLAY_TYPE Display_Type_LCD
  #elif defined (BOARD_DISPLAY_USE_UART) && (BOARD_DISPLAY_USE_UART!=0)
    #define RTLS_SLAVE_DISPLAY_TYPE Display_Type_UART
  #else // !BOARD_DISPLAY_USE_LCD && !BOARD_DISPLAY_USE_UART
    #define RTLS_SLAVE_DISPLAY_TYPE 0 // Option not supported
  #endif // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
#else // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
  #define RTLS_SLAVE_DISPLAY_TYPE 0 // No Display
#endif // !Display_DISABLE_ALL

// Task configuration
#define RTLS_SLAVE_TASK_PRIORITY                     1

#ifndef RTLS_SLAVE_TASK_STACK_SIZE
#define RTLS_SLAVE_TASK_STACK_SIZE                   644
#endif

// Application events
#define RTLS_SLAVE_STATE_CHANGE_EVT                  0x0001
#define RTLS_SLAVE_CHAR_CHANGE_EVT                   0x0002
#define RTLS_SLAVE_PAIRING_STATE_EVT                 0x0004
#define RTLS_SLAVE_PASSCODE_NEEDED_EVT               0x0008
#define RTLS_SLAVE_CONN_EVT                          0x0010
#define RTLS_SLAVE_RTLS_CTRL_EVT                     0x0020

// Internal Events for RTOS application
#define RTLS_SLAVE_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define RTLS_SLAVE_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define RTLS_SLAVE_PERIODIC_EVT                      Event_Id_00

// Bitwise OR of all events to pend on
#define RTLS_SLAVE_ALL_EVENTS                       (RTLS_SLAVE_ICALL_EVT        | \
                                                     RTLS_SLAVE_QUEUE_EVT        | \
                                                     RTLS_SLAVE_PERIODIC_EVT)

#define RTLS_SLAVE_NEXT_TASK_TIME_INFINITE           0xFFFFFFFF

// Set the register cause to the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_SET(RegisterCause) (connectionEventRegisterCauseBitMap |= RegisterCause )
// Remove the register cause from the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_REMOVE(RegisterCause) (connectionEventRegisterCauseBitMap &= (~RegisterCause) )
// Gets whether the current App is registered to the receive connection events
#define CONNECTION_EVENT_IS_REGISTERED (connectionEventRegisterCauseBitMap > 0)
// Gets whether the RegisterCause was registered to recieve connection event
#define CONNECTION_EVENT_REGISTRATION_CAUSE(RegisterCause) (connectionEventRegisterCauseBitMap & RegisterCause )

// Hard coded PSM for passing data between central and peripheral
#define RTLS_PSM      0x0080
#define RTLS_PDU_SIZE MAX_PDU_SIZE

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
  uint8_t *pData;  // event data
} sbpEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Display Interface
Display_Handle dispHandle = NULL;

extern uint8_t iToSend;
extern uint8_t dataToSend[6];       //SIMPLEPROFILE_CHAR1_LEN innan

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct sbpTask;
Char sbpTaskStack[RTLS_SLAVE_TASK_STACK_SIZE];

// Scan response data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  0x0A,                           // Length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE, // Type of this data
  'R',
  'T',
  'L',
  'S',
  'S',
  'l',
  'a',
  'v',
  'e',
};

// Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertising)
static uint8_t advertData[] =
{
 0x0A,                           // Length of this data
 GAP_ADTYPE_LOCAL_NAME_COMPLETE, // Type of this data
 'R',
 'T',
 'L',
 'S',
 'S',
 'l',
 'a',
 'v',
 'e',
  // Flags: this field sets the device to use general discoverable
  // mode (advertises indefinitely) instead of general
  // discoverable mode (advertise for 30 seconds at a time)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16(MYACCELEROMETER_SERV_UUID),
  HI_UINT16(MYACCELEROMETER_SERV_UUID)
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "RTLS Slave";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

// Connection handle of current connection
static uint16_t connHandle = GAP_CONNHANDLE_INIT;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

void updateAccelerometer(void);

static void RTLSSlave_init( void );
static void RTLSSlave_taskFxn(UArg a0, UArg a1);

static uint8_t RTLSSlave_processStackMsg(ICall_Hdr *pMsg);
static uint8_t RTLSSlave_processGATTMsg(gattMsgEvent_t *pMsg);
static void RTLSSlave_processAppMsg(sbpEvt_t *pMsg);
static void RTLSSlave_processStateChangeEvt(gaprole_States_t newState);
static void RTLSSlave_clockHandler(UArg arg);

static void RTLSSlave_sendAttRsp(void);
static void RTLSSlave_freeAttRsp(uint8_t status);

static void RTLSSlave_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                 uint8_t uiInputs, uint8_t uiOutputs,
                                 uint32_t numComparison);
static void RTLSSlave_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status);
static void RTLSSlave_processPairState(uint8_t state, uint8_t status);
static void RTLSSlave_processPasscode(uint8_t uiOutputs);

static void RTLSSlave_stateChangeCB(gaprole_States_t newState);
static uint8_t RTLSSlave_enqueueMsg(uint16_t event, uint8_t state,
                                              uint8_t *pData);
static void RTLSSlave_connEvtCB(Gap_ConnEventRpt_t *pReport);
static void RTLSSlave_processConnEvt(Gap_ConnEventRpt_t *pReport);

static void RTLSSlave_openL2CAPChanCoc(void);
static void RTLSSlave_processL2CAPSignalEvent(l2capSignalEvent_t *pMsg);
static uint8_t RTLSSlave_processL2CAPDataEvent(l2capDataEvent_t *pMsg);
static void RTLSSlave_processRtlsMsg(uint8_t *pMsg);
static void RTLSSlave_enableRtlsSync(uint8_t enable);

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

extern void spiThread_create(void);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Peripheral GAPRole Callbacks
static gapRolesCBs_t RTLSSlave_gapRoleCBs =
{
  RTLSSlave_stateChangeCB     // GAPRole State Change Callbacks
};

// GAP Bond Manager Callbacks
// These are set to NULL since they are not needed. The application
// is set up to only perform justworks pairing.
static gapBondCBs_t RTLSSlave_BondMgrCBs =
{
  RTLSSlave_passcodeCB, // Passcode callback
  RTLSSlave_pairStateCB // Pairing / Bonding state Callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
/*********************************************************************
 * The following typedef and global handle the registration to connection event
 */
typedef enum
{
   NOT_REGISTERED     = 0x0,
   FOR_ATT_RSP        = 0x2,
   FOR_TOF            = 0x4,
} connectionEventRegisterCause_u;

// Handle the registration and un-registration for the connection event, since only one can be registered.
uint32_t  connectionEventRegisterCauseBitMap = NOT_REGISTERED;


/*********************************************************************
 * @fn      RTLSSlave_RegistertToAllConnectionEvent()
 *
 * @brief   register to receive connection events for all the connection
 *
 * @param connectionEventRegister represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */
bStatus_t RTLSSlave_RegistertToAllConnectionEvent(connectionEventRegisterCause_u connectionEventRegisterCause)
{
  bStatus_t status = SUCCESS;

  // In case  there is no registration for the connection event, make the registration
  if (!CONNECTION_EVENT_IS_REGISTERED)
  {
    status = GAP_RegisterConnEventCb(RTLSSlave_connEvtCB, GAP_CB_REGISTER, LINKDB_CONNHANDLE_ALL);
  }

  if(status == SUCCESS)
  {
    // Add the reason bit to the bitamap.
    CONNECTION_EVENT_REGISTER_BIT_SET(connectionEventRegisterCause);
  }

  return(status);
}

/*********************************************************************
 * @fn      RTLSSlave_UnRegistertToAllConnectionEvent()
 *
 * @brief   Un register connection events
 *
 * @param connectionEventRegisterCause represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */
bStatus_t RTLSSlave_UnRegistertToAllConnectionEvent (connectionEventRegisterCause_u connectionEventRegisterCause)
{
  bStatus_t status = SUCCESS;

  CONNECTION_EVENT_REGISTER_BIT_REMOVE(connectionEventRegisterCause);

  // If there is nothing registered to the connection event, request to unregister
  if (!CONNECTION_EVENT_IS_REGISTERED)
  {
    GAP_RegisterConnEventCb(RTLSSlave_connEvtCB, GAP_CB_UNREGISTER, LINKDB_CONNHANDLE_ALL);
  }

  return(status);
}

/*********************************************************************
 * @fn      RTLSSlave_createTask
 *
 * @brief   Task creation function for the TOF Responder.
 *
 * @param   None.
 *
 * @return  None.
 */
void RTLSSlave_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbpTaskStack;
  taskParams.stackSize = RTLS_SLAVE_TASK_STACK_SIZE;
  taskParams.priority = RTLS_SLAVE_TASK_PRIORITY;

  Task_construct(&sbpTask, RTLSSlave_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      RTLSSlave_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
static void RTLSSlave_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

#ifdef USE_RCOSC
  RCOSC_enableCalibration();
#endif // USE_RCOSC

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Create one-shot clocks for internal periodic events.
  Util_constructClock(&periodicClock, RTLSSlave_clockHandler,
                      RTLS_SLAVE_PERIODIC_EVT_PERIOD, 0, false, RTLS_SLAVE_PERIODIC_EVT);

  dispHandle = Display_open(RTLS_SLAVE_DISPLAY_TYPE, NULL);

  // Set GAP Parameters: After a connection was established, delay in seconds
  // before sending when GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE,...)
  // uses GAPROLE_LINK_PARAM_UPDATE_INITIATE_BOTH_PARAMS or
  // GAPROLE_LINK_PARAM_UPDATE_INITIATE_APP_PARAMS
  // For current defaults, this has no effect.
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

  // Setup the Peripheral GAPRole Profile. For more information see the User's
  // Guide:
  // http://software-dl.ti.com/lprf/sdg-latest/html/
  {
    // Device starts advertising upon initialization of GAP
    uint8_t initialAdvertEnable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until re-enabled by the application
    uint16_t advertOffTime = 0;

    uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the Peripheral GAPRole Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime);

    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                         scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout);
  }

  // Set the Device Name characteristic in the GAP GATT Service
  // For more information, see the section in the User's Guide:
  // http://software-dl.ti.com/lprf/sdg-latest/html
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Set GAP Parameters to set the advertising interval
  // For more information, see the GAP section of the User's Guide:
  // http://software-dl.ti.com/lprf/sdg-latest/html
  {
    // Use the same interval for general and limited advertising.
    // Note that only general advertising will occur based on the above configuration
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  // Setup the GAP Bond Manager. For more information see the section in the
  // User's Guide:
  // http://software-dl.ti.com/lprf/sdg-latest/html/
  {
    // Don't send a pairing request after connecting; the peer device must
    // initiate pairing
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    // Use authenticated pairing: require passcode.
    uint8_t mitm = TRUE;
    // This device only has display capabilities. Therefore, it will display the
    // passcode during pairing. However, since the default passcode is being
    // used, there is no need to display anything.
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_YES_NO;
    // Request bonding (storing long-term keys for re-encryption upon subsequent
    // connections without repairing)
    uint8_t bonding = TRUE;

    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP GATT Service
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT Service

  MyAccelerometer_AddService(selfEntity);

  // Initalization of characteristics in myAccelerometer that are readable.
  { //===========�NDRAT KLAMMER ================
  uint8_t myAccelerometer_myAccelerometerChar_initVal[MYACCELEROMETER_MYACCELEROMETERCHAR_LEN] = { 0, 0, 0, 0, 0, 0 };
  MyAccelerometer_SetParameter(MYACCELEROMETER_MYACCELEROMETERCHAR_ID,
                               MYACCELEROMETER_MYACCELEROMETERCHAR_LEN,
                               myAccelerometer_myAccelerometerChar_initVal);
  }// =============== SLUT P� M�SVINGAR ================

  // Start the Device
  VOID GAPRole_StartDevice(&RTLSSlave_gapRoleCBs);

  // Start Bond Manager and register callback
  VOID GAPBondMgr_Register(&RTLSSlave_BondMgrCBs);

  // Register with GAP for HCI/Host messages. This is needed to receive HCI
  // events. For more information, see the section in the User's Guide:
  // http://software-dl.ti.com/lprf/sdg-latest/html
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  //Set default values for Data Length Extension
  {
    //Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
    #define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
    #define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

    //This API is documented in hci.h
    //See the LE Data Length Extension section in the BLE-Stack User's Guide for information on using this command:
    //http://software-dl.ti.com/lprf/sdg-latest/html/cc2640/index.html
    //HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
  }

#if !defined (USE_LL_CONN_PARAM_UPDATE)
  // Get the currently set local supported LE features
  // The HCI will generate an HCI event that will get received in the main
  // loop
  HCI_LE_ReadLocalSupportedFeaturesCmd();
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)

}

/*********************************************************************
 * @fn      RTLSSlave_taskFxn
 *
 * @brief   Application task entry point for the TOF Responder.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void RTLSSlave_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  RTLSSlave_init();

  spiThread_create();
  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, RTLS_SLAVE_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      // Fetch any available messages that might have been sent from the stack
      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

         if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = RTLSSlave_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      if (events & RTLS_SLAVE_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueue))
        {
          sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);
          if (pMsg)
          {
            // Process message.
            RTLSSlave_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }
      if (events & RTLS_SLAVE_PERIODIC_EVT)
      {
          Util_startClock(&periodicClock);
          updateAccelerometer();
          //Display_print1(dispHandle, 5, 0, "d: %d", dataToSend[1]);
      }

    }
  }
}

void updateAccelerometer(void){
    MyAccelerometer_SetParameter(MYACCELEROMETER_MYACCELEROMETERCHAR_ID,
                                 MYACCELEROMETER_MYACCELEROMETERCHAR_LEN,
                                 dataToSend);
}

/*********************************************************************
 * @fn      RTLSSlave_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t RTLSSlave_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case L2CAP_SIGNAL_EVENT:
     RTLSSlave_processL2CAPSignalEvent((l2capSignalEvent_t *)pMsg);
    break;

    case L2CAP_DATA_EVENT:
      safeToDealloc = RTLSSlave_processL2CAPDataEvent((l2capDataEvent_t *)pMsg);
    break;
      
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = RTLSSlave_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
          {
          }
          break;

          case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
            AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
          break;

          default:
            break;
        }
      }
      break;

      default:
        // do nothing
        break;

    }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      RTLSSlave_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t RTLSSlave_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (RTLSSlave_RegistertToAllConnectionEvent(FOR_ATT_RSP) == SUCCESS)
    {
      // First free any pending response
      RTLSSlave_freeAttRsp(FAILURE);

      // Hold on to the response message for retransmission
      pAttRsp = pMsg;

      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
    Display_print1(dispHandle, 5, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    Display_print1(dispHandle, 5, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      RTLSSlave_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void RTLSSlave_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;

    // Increment retransmission count
    rspTxRetry++;

    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      RTLSSlave_UnRegistertToAllConnectionEvent(FOR_ATT_RSP);

      // We're done with the response message
      RTLSSlave_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      Display_print1(dispHandle, 5, 0, "Rsp send retry: %d", rspTxRetry);
    }
  }
}

/*********************************************************************
 * @fn      RTLSSlave_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void RTLSSlave_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      Display_print1(dispHandle, 5, 0, "Rsp sent retry: %d", rspTxRetry);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);

      Display_print1(dispHandle, 5, 0, "Rsp retry failed: %d", rspTxRetry);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
 * @fn      RTLSSlave_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void RTLSSlave_processAppMsg(sbpEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case RTLS_SLAVE_STATE_CHANGE_EVT:
      {
        RTLSSlave_processStateChangeEvt((gaprole_States_t)pMsg->hdr.state);
      }
      break;

    // Pairing event
    case RTLS_SLAVE_PAIRING_STATE_EVT:
      {
        RTLSSlave_processPairState(pMsg->hdr.state, *pMsg->pData);
        ICall_free(pMsg->pData);
        break;
      }

    // Passcode event
    case RTLS_SLAVE_PASSCODE_NEEDED_EVT:
      {
        RTLSSlave_processPasscode(*pMsg->pData);
        ICall_free(pMsg->pData);
        break;
      }

    case RTLS_SLAVE_CONN_EVT:
      {
        RTLSSlave_processConnEvt((Gap_ConnEventRpt_t *)(pMsg->pData));
        ICall_free(pMsg->pData);
        break;
      }

    case RTLS_SLAVE_RTLS_CTRL_EVT:
      {
        RTLSSlave_processRtlsMsg((uint8_t *)pMsg->pData);

        if (pMsg->pData != NULL)
        {
          ICall_free(pMsg->pData);
        }
        break;
      }

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      RTLSSlave_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void RTLSSlave_stateChangeCB(gaprole_States_t newState)
{
  RTLSSlave_enqueueMsg(RTLS_SLAVE_STATE_CHANGE_EVT, newState, 0);
}

/*********************************************************************
 * @fn      RTLSSlave_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void RTLSSlave_processStateChangeEvt(gaprole_States_t newState)
{
#ifdef PLUS_BROADCASTER
  static bool firstConnFlag = false;
#endif // PLUS_BROADCASTER

  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
        
        // Display device address
        Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(ownAddress));
        Display_print0(dispHandle, 2, 0, "Initialized");
      }
      break;

    case GAPROLE_ADVERTISING:
      Display_print0(dispHandle, 2, 0, "Advertising");
      break;

#ifdef PLUS_BROADCASTER
    // After a connection is dropped, a device in PLUS_BROADCASTER will continue
    // sending non-connectable advertisements and shall send this change of
    // state to the application.  These are then disabled here so that sending
    // connectable advertisements can resume.
    case GAPROLE_ADVERTISING_NONCONN:
      {
        uint8_t advertEnabled = FALSE;

        // Disable non-connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                           &advertEnabled);

        advertEnabled = TRUE;

        // Enabled connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &advertEnabled);

        // Reset flag for next connection.
        firstConnFlag = false;

        RTLSSlave_freeAttRsp(bleNotConnected);
      }
      break;
#endif //PLUS_BROADCASTER

    case GAPROLE_CONNECTED:
      {
        linkDBInfo_t linkInfo;
        uint8_t numActive = 0;

        Util_startClock(&periodicClock);

        numActive = linkDB_NumActive();

        // Notify RTLS Control that we are connected
        RTLSSlave_openL2CAPChanCoc();

        RTLSCtrl_connResultEvt(RTLS_SUCCESS);

        // Use numActive to determine the connection handle of the last
        // connection
        if ( linkDB_GetInfo( numActive - 1, &linkInfo ) == SUCCESS )
        {
          GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connHandle);

          Display_print1(dispHandle, 2, 0, "Num Conns: %d", (uint16_t)numActive);
          Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(linkInfo.addr));
        }
        else
        {
          uint8_t peerAddress[B_ADDR_LEN];

          GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

          Display_print0(dispHandle, 2, 0, "Connected");
          Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(peerAddress));
        }

        #ifdef PLUS_BROADCASTER
          // Only turn advertising on for this state when we first connect
          // otherwise, when we go from connected_advertising back to this state
          // we will be turning advertising back on.
          if (firstConnFlag == false)
          {
            uint8_t advertEnabled = FALSE; // Turn on Advertising

            // Disable connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);

            // Set to true for non-connectable advertising.
            advertEnabled = TRUE;

            // Enable non-connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            firstConnFlag = true;
          }
        #endif // PLUS_BROADCASTER

          Display_print0(dispHandle, 5, 0, "");
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      Display_print0(dispHandle, 2, 0, "Connected Advertising");
      break;

    case GAPROLE_WAITING:
      Util_stopClock(&periodicClock);
      RTLSSlave_freeAttRsp(bleNotConnected);
      RTLSCtrl_connResultEvt(RTLS_FAIL);
      Display_print0(dispHandle, 2, 0, "Disconnected");
      RTLSCtrl_connResultEvt(RTLS_LINK_TERMINATED);
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      RTLSSlave_freeAttRsp(bleNotConnected);

      Display_print0(dispHandle, 2, 0, "Timed Out");

      // Clear remaining lines
      Display_clearLines(dispHandle, 3, 5);

      #ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        firstConnFlag = false;
      #endif // PLUS_BROADCASTER

      RTLSCtrl_connResultEvt(RTLS_LINK_TERMINATED);
      break;

    case GAPROLE_ERROR:
      Display_print0(dispHandle, 2, 0, "Error");
      break;

    default:
      Display_clearLine(dispHandle, 2);
      break;
  }

}

/*********************************************************************
 * @fn      RTLSSlave_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void RTLSSlave_pairStateCB(uint16_t connHandle, uint8_t state, uint8_t status)
{
  uint8_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = status;

    // Queue the event.
    RTLSSlave_enqueueMsg(RTLS_SLAVE_PAIRING_STATE_EVT, state, pData);
  }
}

/*********************************************************************
 * @fn      RTLSSlave_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void RTLSSlave_processPairState(uint8_t state, uint8_t status)
{
  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
    Display_print0(dispHandle, 2, 0, "Pairing started");
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 2, 0, "Pairing success");
    }
    else
    {
      Display_print1(dispHandle, 2, 0, "Pairing fail: %d", status);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 2, 0, "Bonding success");
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BOND_SAVED)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 2, 0, "Bond save success");
    }
    else
    {
      Display_print1(dispHandle, 2, 0, "Bond save failed: %d", status);
    }
  }
}

/*********************************************************************
 * @fn      RTLSSlave_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void RTLSSlave_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                 uint8_t uiInputs, uint8_t uiOutputs,
                                 uint32_t numComparison)
{
  uint8_t *pData;

  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = uiOutputs;

    // Enqueue the event.
    RTLSSlave_enqueueMsg(RTLS_SLAVE_PASSCODE_NEEDED_EVT, 0, pData);
  }
}

/*********************************************************************
 * @fn      RTLSSlave_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void RTLSSlave_processPasscode(uint8_t uiOutputs)
{
  // This app uses a default passcode. A real-life scenario would handle all
  // pairing scenarios and likely generate this randomly.
  uint32_t passcode = B_APP_DEFAULT_PASSCODE;

  // Display passcode to user
  if (uiOutputs != 0)
  {
    Display_print1(dispHandle, 4, 0, "Passcode: %d", passcode);
  }

  uint16_t connectionHandle;
  GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connectionHandle);

  // Send passcode response
  GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);
}

/*********************************************************************
 * @fn      RTLSSlave_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void RTLSSlave_clockHandler(UArg arg)
{
  // Wake up the application.
  Event_post(syncEvent, arg);
}

/*********************************************************************
 * @fn      RTLSSlave_connEvtCB
 *
 * @brief   Connection event callback.
 *
 * @param pReport pointer to connection event report
 */
static void RTLSSlave_connEvtCB(Gap_ConnEventRpt_t *pReport)
{
  // Enqueue the event for processing in the app context.
  if (RTLSSlave_enqueueMsg(RTLS_SLAVE_CONN_EVT, 0, (uint8_t *)pReport) == FALSE)
  {
    ICall_free(pReport);
  }
}

/*********************************************************************
 * @fn      RTLSSlave_processConnEvt
 *
 * @brief   Process connection event.
 *
 * @param pReport pointer to connection event report
 */
static void RTLSSlave_processConnEvt(Gap_ConnEventRpt_t *pReport)
{ 
  if (CONNECTION_EVENT_REGISTRATION_CAUSE(FOR_ATT_RSP))
  {
    // The GATT server might have returned a blePending as it was trying
    // to process an ATT Response. Now that we finished with this
    // connection event, let's try sending any remaining ATT Responses
    // on the next connection event.
    RTLSSlave_sendAttRsp();
  }

  // Do a TOF Run, at the end of the active connection period
  if (CONNECTION_EVENT_REGISTRATION_CAUSE(FOR_TOF))
  {
    rtlsStatus_e status;

    // Convert BLE specific status to RTLS Status
    if (pReport->status != GAP_CONN_EVT_STAT_MISSED)
    {
      status = RTLS_SUCCESS;
    }
    else
    {
      status = RTLS_FAIL;
    }

    RTLSCtrl_syncEventNotify(status, pReport->nextTaskTime, pReport->lastRssi, pReport->channel);
  }
}


/*********************************************************************
 * @fn      RTLSSlave_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t RTLSSlave_enqueueMsg(uint16_t event, uint8_t state, uint8_t *pData)
{
  sbpEvt_t *pMsg = ICall_malloc(sizeof(sbpEvt_t));

  // Create dynamic pointer to message.
  if (pMsg)
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
  }

  return FALSE;
}

/*********************************************************************
 * @fn      RTLSSlave_openL2CAPChanCoc
 *
 * @brief   Opens a communication channel between RTLS Master/Slave
 *
 * @param   pairState - Verify that devices are paired
 *
 * @return  none
 */
static void RTLSSlave_openL2CAPChanCoc(void)
{
  l2capPsm_t psm;
  l2capPsmInfo_t psmInfo;

  if (L2CAP_PsmInfo(RTLS_PSM, &psmInfo) == INVALIDPARAMETER)
   {
     // Prepare the PSM parameters
     psm.initPeerCredits = 0xFFFF;
     psm.maxNumChannels = 1;
     psm.mtu = RTLS_PDU_SIZE;
     psm.peerCreditThreshold = 0;
     psm.pfnVerifySecCB = NULL;
     psm.psm = RTLS_PSM;
     psm.taskId = ICall_getLocalMsgEntityId(ICALL_SERVICE_CLASS_BLE_MSG, selfEntity);

     // Register PSM with L2CAP task
     L2CAP_RegisterPsm(&psm);
   }
   else
   {
       Display_print0(dispHandle, 4, 0, "L2CAP Channel is already open");
   }
}

/*********************************************************************
 * @fn      RTLSSlave_processL2CAPSignalEvent
 *
 * @brief   Handle L2CAP signal events
 *
 * @param   pMsg - pointer to the signal that was received
 *
 * @return  none
 */
static void RTLSSlave_processL2CAPSignalEvent(l2capSignalEvent_t *pMsg)
{
  switch (pMsg->opcode)
  {
    case L2CAP_CHANNEL_ESTABLISHED_EVT:
    {
      l2capChannelEstEvt_t *pEstEvt = &(pMsg->cmd.channelEstEvt);

      // Give max credits to the other side
      L2CAP_FlowCtrlCredit(pEstEvt->CID, 0xFFFF);

      Display_print0(dispHandle, 4, 0, "L2CAP Channel Open");
    }
    break;

    case L2CAP_SEND_SDU_DONE_EVT:
    {
      if (pMsg->hdr.status == SUCCESS)
      {
        RTLSCtrl_processDataSent(RTLS_SUCCESS);
      }
      else
      {
        RTLSCtrl_processDataSent(RTLS_FAIL);
      }
    }
    break;
  }
}

/*********************************************************************
 * @fn      RTLSSlave_processL2CAPDataEvent
 *
 * design /ref 159098678
 * @brief   Handles incoming L2CAP data
 *
 * @param   pMsg - pointer to the signal that was received
 *
 * @return  the return value determines whether pMsg can be freed or not
 */
static uint8_t RTLSSlave_processL2CAPDataEvent(l2capDataEvent_t *pMsg)
{
  rtlsPacket_t *pRtlsPkt;
  static uint16_t packetCounter;

  if (!pMsg)
  {
    // Caller needs to figure out by himself that pMsg is NULL
    return TRUE;
  }

  // This application doesn't care about other L2CAP data events other than RTLS
  // It is possible to expand this function to support multiple COC CID's
  pRtlsPkt = (rtlsPacket_t *)ICall_malloc(pMsg->pkt.len);

  // Check for malloc error
  if (!pRtlsPkt)
  {
    // Free the payload (must use BM_free here according to L2CAP documentation)
    BM_free(pMsg->pkt.pPayload);
    return TRUE;
  }

  // Copy the payload
  memcpy(pRtlsPkt, pMsg->pkt.pPayload, pMsg->pkt.len);

  Display_print1(dispHandle, 10, 0, "RTLS Packet Received, cmdId %d", pRtlsPkt->cmdOp);
  Display_print1(dispHandle, 11, 0, "Packet Len: %d", pMsg->pkt.len);
  Display_print1(dispHandle, 12, 0, "Number of packets received: %d", ++packetCounter);

  // Free the payload (must use BM_free here according to L2CAP documentation)
  BM_free(pMsg->pkt.pPayload);

  // RTLS Control will handle the information in the packet
  RTLSCtrl_processRtlsPacket((uint8_t *)pRtlsPkt);

  return TRUE;
}

/*********************************************************************
 * @fn      RTLSSlave_enableRtlsSync
 *
 * @brief   This function is used by RTLS Control to notify the RTLS application
 *          to start sending synchronization events (for BLE this is a connection event)
 *
 * @param   enable - start/stop synchronization
 *
 * @return  none
 */
static void RTLSSlave_enableRtlsSync(uint8_t enable)
{
  bStatus_t status = RTLS_FALSE;

  if (enable == RTLS_TRUE)
  {
    if (!CONNECTION_EVENT_IS_REGISTERED)
    {
      status = GAP_RegisterConnEventCb(RTLSSlave_connEvtCB, GAP_CB_REGISTER, LINKDB_CONNHANDLE_ALL);
    }

    if (status == SUCCESS)
    {
      CONNECTION_EVENT_REGISTER_BIT_SET(FOR_TOF);
    }
  }
  else if (enable == RTLS_FALSE)
  {
    CONNECTION_EVENT_REGISTER_BIT_REMOVE(FOR_TOF);

    // If there is nothing registered to the connection event, request to unregister
    if (!CONNECTION_EVENT_IS_REGISTERED)
    {
      GAP_RegisterConnEventCb(RTLSSlave_connEvtCB, GAP_CB_UNREGISTER, LINKDB_CONNHANDLE_ALL);
    }
  }
}

/*********************************************************************
 * @fn      RTLSSlave_processRtlsMsg
 *
 * @brief   Handle processing messages from RTLS Control
 *
 * @param   msg - a pointer to the message
 *
 * @return  none
 */
static void RTLSSlave_processRtlsMsg(uint8_t *pMsg)
{
  rtlsCtrlReq_t *req = (rtlsCtrlReq_t *)pMsg;

  switch(req->reqOp)
  {
    case RTLS_REQ_ENABLE_SYNC:
    {
      RTLSSlave_enableRtlsSync(*req->pData);
    }
    break;

    default:
      break;
  }

  if (req->pData != NULL)
  {
    ICall_free(req->pData);
  }
}

/*********************************************************************
 * @fn      RTLSMaster_rtlsCtrlMsgCb
 *
 * @brief   Callback given to RTLS Control
 *
 * @param  cmd - the command to be enqueued
 *
 * @return  none
 */
void RTLSSlave_rtlsCtrlMsgCb(uint8_t *cmd)
{
  // Enqueue the message to switch context
  RTLSSlave_enqueueMsg(RTLS_SLAVE_RTLS_CTRL_EVT, SUCCESS, (uint8_t *)cmd);
}

/*********************************************************************
*********************************************************************/
