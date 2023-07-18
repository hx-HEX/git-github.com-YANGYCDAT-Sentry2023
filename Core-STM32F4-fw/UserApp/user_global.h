#pragma once
#include "system_monitor.h"
#include "led.h"
#include "usart_interface.h"
#include "gimbal.h"
#include "djirc.h"
#include "vofa.h"
#include "aim_assist.h"
#include "navigation.h"
#include "referee.h"
#include "uart_protocol.h"
#include "common_math.h"
#include "sentry_robot.h"

// #define CHASSIS_DEBUG
// #define GIMBAL_DEBUG
// #define SHOOT_DEBUG
// #define NAVIGATION_DEBUG
#define AIM_ASSIST_DEBUG
//#define REFREE_DEBUG

enum ControlMode
{
    SAFE = 0,
    RC_GS,
    AUTO_G_RC_S,
    AUTO_GS_RC_C,
    AUTO_CGS,
    RC_C,
    AUTO_G_RC_C
};

extern SystemMonitor G_system_monitor;
extern RefereeMonitor G_referee_monitor;
extern Led G_led;
extern Gimbal G_gimbal;
extern DJIRC G_djirc;
extern Vofa G_vofa;
extern AimAssist G_aim_assist;
extern Navigation G_navigation;
extern Referee G_referee;
extern SentryRobot G_sentry;
extern ControlMode G_control_mode;

void ControlModeUpdate(void);
void RobotStatesUpdate(void);
void RobotTargetsUpdate(void);
void RobotControlExecute(void);
void ChassisTargetsUpdate(void);
void SteerDriveChassisTargetsUpdate(void);
void StandardDriveChassisTargetsUpdate(void);
void DiffDriveChassisTargetsUpdate(void);
void CommonDriveChassisTargetsUpdate(void);
void SteerModeChassisTargetsUpdate(void);
void GimbalTargetsUpdate(void);
void ShootTargetsUpdate(void);

void SendNavigationData(void);
void SendGimbalData(void);
void SendAimAssistData(void);
void SendRefereeData(void);

void VisualizeGimbalData(void);
void VisualizeChassisData(void);
void VisualizeNavigationData(void);
void VisualizeShootData(void);
void VisualizeAimAssistData(void);
void VisualizeRefreeData(void);

void RemoteControlMonitor(void);
void CommunicationMonitor(void);
void MotorMonitor(void);