/*
 * rtls_ctrl_acc.h
 *
 *  Created on: 19 mars 2020
 *      Author: liper
 */

#ifndef EGENKOD_RTLS_CTRL_ACC_H_
#define EGENKOD_RTLS_CTRL_ACC_H_

typedef struct __attribute__((packed))
{
  int16_t X;             //!< Accelerometer Result
  int16_t Y;             //!< Accelerometer Result
  int16_t Z;             //!< Accelerometer Result
} rtlsAccResult_t;

typedef struct __attribute__((packed))
{
  uint8_t X0;             //!< Accelerometer Result
  uint8_t X1;             //!< Accelerometer Result
  uint8_t Y0;             //!< Accelerometer Result
  uint8_t Y1;             //!< Accelerometer Result
  uint8_t Z0;             //!< Accelerometer Result
  uint8_t Z1;             //!< Accelerometer Result
} rtlsAccResultRaw_t;

void RTLSMaster_processAccDataRaw(uint8_t *pMsg);

void RTLSMaster_processAccData(uint8_t *pData);

void RTLSCtrl_accDebug(void);

#endif /* EGENKOD_RTLS_CTRL_ACC_H_ */
