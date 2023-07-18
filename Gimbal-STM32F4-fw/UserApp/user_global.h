#pragma once
#include "system_monitor.h"
#include "led.h"
#include "usart_interface.h"
#include "gimbal.h"
#include "vofa.h"
#include "uart_protocol.h"
#include "common_math.h"
#include "sentry_robot.h"
#include "gun_shift.h"

extern SystemMonitor G_system_monitor;
extern Led G_led;
extern Vofa G_vofa;
extern Gimbal G_gimbal;
extern SentryRobot G_sentry;
extern GunShift G_gun_shift;

void ControlModeUpdate(void);
void RobotStatesUpdate(void);
void RobotTargetsUpdate(void);

void VisualizeGimbalData(void);
void RobotControlExecute(void);