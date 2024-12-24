/**
 ********************************************************************
 * @file    test_flight_control.h
 * @brief   This is the header file for "test_flight_control.c", defining the structure and
 * (exported) function prototypes.
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TEST_FLIGHT_CONTROL_H
#define TEST_FLIGHT_CONTROL_H

/* Includes ------------------------------------------------------------------*/
#include "dji_typedef.h"
#include "dji_flight_controller.h"
#include "dji_fc_subscription.h"
#include "dji_platform.h"
#include "dji_logger.h"
#include <math.h>
#include <widget/test_widget.h>
#include <dji_aircraft_info.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/
typedef enum {
    E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_TAKE_OFF_LANDING,
    E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_TAKE_OFF_POSITION_CTRL_LANDING,
    E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_TAKE_OFF_GO_HOME_FORCE_LANDING,
    E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_TAKE_OFF_VELOCITY_CTRL_LANDING,
    E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_ARREST_FLYING,
    E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_SET_GET_PARAM,
    E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_FTS_TRIGGER,
} E_DjiTestFlightCtrlSampleSelect;

#pragma pack(1)
typedef struct {
    dji_f32_t x;
    dji_f32_t y;
    dji_f32_t z;
} T_DjiTestFlightControlVector3f; // pack(1)
#pragma pack()

/* Exported types ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
T_DjiReturnCode DjiTest_FlightControlRunSample(E_DjiTestFlightCtrlSampleSelect flightCtrlSampleSelect);
void DjiTest_FlightControlVelocityAndYawRateCtrl(const T_DjiTestFlightControlVector3f offsetDesired, float yawRate,
                                                 uint32_t timeMs);

uint8_t DjiTest_FlightControlGetDisplayModeIndex(E_DjiFcSubscriptionDisplayMode displayMode);
T_DjiFcSubscriptionFlightStatus DjiTest_FlightControlGetValueOfFlightStatus(void);
T_DjiFcSubscriptionDisplaymode DjiTest_FlightControlGetValueOfDisplayMode(void);
T_DjiFcSubscriptionHeightFusion DjiTest_FlightControlGetValueOfHeightFusion(void);
T_DjiFcSubscriptionQuaternion DjiTest_FlightControlGetValueOfQuaternion(void);
T_DjiFcSubscriptionPositionFused DjiTest_FlightControlGetValueOfPositionFused(void);
dji_f32_t DjiTest_FlightControlGetValueOfRelativeHeight(void);
bool DjiTest_FlightControlMotorStartedCheck(void);
bool DjiTest_FlightControlTakeOffInAirCheck(void);
bool DjiTest_FlightControlLandFinishedCheck(void);
bool DjiTest_FlightControlMonitoredTakeoff(void);
bool DjiTest_FlightControlCheckActionStarted(E_DjiFcSubscriptionDisplayMode mode);
bool DjiTest_FlightControlMonitoredLanding(void);
bool DjiTest_FlightControlGoHomeAndConfirmLanding(void);
T_DjiTestFlightControlVector3f DjiTest_FlightControlQuaternionToEulerAngle(T_DjiFcSubscriptionQuaternion quat);
T_DjiTestFlightControlVector3f
    DjiTest_FlightControlLocalOffsetFromGpsAndFusedHeightOffset(T_DjiFcSubscriptionPositionFused target,
                                                            T_DjiFcSubscriptionPositionFused origin,
                                                            dji_f32_t targetHeight,
                                                            dji_f32_t originHeight);
T_DjiTestFlightControlVector3f
    DjiTest_FlightControlVector3FSub(T_DjiTestFlightControlVector3f vectorA, T_DjiTestFlightControlVector3f vectorB);
int DjiTest_FlightControlSignOfData(dji_f32_t data);
void DjiTest_FlightControlHorizCommandLimit(dji_f32_t speedFactor, dji_f32_t *commandX, dji_f32_t *commandY);
dji_f32_t DjiTest_FlightControlVectorNorm(T_DjiTestFlightControlVector3f v);
T_DjiReturnCode
    DjiTest_FlightControlJoystickCtrlAuthSwitchEventCallback(T_DjiFlightControllerJoystickCtrlAuthorityEventInfo eventData);
bool DjiTest_FlightControlMoveByPositionOffset(T_DjiTestFlightControlVector3f offsetDesired,
                                                      float yawDesiredInDeg,
                                                      float posThresholdInM,
                                                      float yawThresholdInDeg);
                                                 
T_DjiReturnCode DjiTest_FlightControlInit(void);
T_DjiReturnCode DjiTest_FlightControlDeInit(void);

#ifdef __cplusplus
}
#endif

#endif // TEST_FLIGHT_CONTROL_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
