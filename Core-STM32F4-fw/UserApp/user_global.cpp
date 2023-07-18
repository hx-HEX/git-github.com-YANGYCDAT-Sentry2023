#include "user_global.h"

SystemMonitor G_system_monitor;
RefereeMonitor G_referee_monitor;
Led G_led;
Gimbal G_gimbal;
DJIRC G_djirc;
Vofa G_vofa;
AimAssist G_aim_assist;
Navigation G_navigation;
Referee G_referee;
SentryRobot G_sentry;
ControlMode G_control_mode;


/**
 *@brief Remote control state monitor
 *
 *@param
 */
void RemoteControlMonitor(void)
{
    if (G_system_monitor.UART1_rx_fps < 50)
    {
        G_djirc.channel.Ch0 = RC_CH_VALUE_OFFSET;
        G_djirc.channel.Ch1 = RC_CH_VALUE_OFFSET;
        G_djirc.channel.Ch2 = RC_CH_VALUE_OFFSET;
        G_djirc.channel.Ch3 = RC_CH_VALUE_OFFSET;
        G_djirc.channel.Ch4 = RC_CH_VALUE_OFFSET;
    }
}



/**
 *@brief Motor state monitor
 *
 *@param
 */
void MotorMonitor(void)
{
    static uint32_t times = 0;
    times++;
    if (G_system_monitor.CAN1_rx_fps < 5000 && G_system_monitor.CAN2_rx_fps)
    {
        if (times % 3 == 0)
        {
            G_led.ToggleGreen();
        }
    }
    else if (G_system_monitor.CAN2_rx_fps < 5000)
    {
        if (times % 2 == 0)
        {
            G_led.ToggleGreen();
        }
    }
    else if (G_system_monitor.CAN1_rx_fps < 5000)
    {
        G_led.ToggleGreen();
    }
    else
    {
        G_led.SetGreen(false);
    }
}



/**
 *@brief Communication state monitor
 *
 *@param
 */
void CommunicationMonitor(void)
{
    static uint32_t times = 0;
    times++;
    if (G_system_monitor.UART3_rx_fps < 50)
    {
        if (times % 10 < 3)
        {
            G_led.SetRed(true);
            G_led.SetBlue(false);
            G_led.SetGreen(false);
        }
    }
    if (G_system_monitor.UART6_rx_fps < 50)
    {
        if (times % 10 >= 3 && times % 10 <= 6)
        {
            G_led.SetBlue(true);
            G_led.SetRed(false);
            G_led.SetGreen(false);
        }
    }
    if (G_system_monitor.UART6_rx_fps < 500)
    {
        if (times % 10 > 6)
        {
            G_led.SetGreen(true);
            G_led.SetRed(false);
            G_led.SetBlue(false);
        }
    }
}



/**
 *@brief visualize navigation datas
 *
 *@param
 */
void VisualizeNavigationData(void)
{
    // Chassis states
// #ifdef CHASSIS_STEER_DRIVING_MODE

//     G_vofa.m_data_send_frame.m_data[1] = G_sentry.m_robot_wheel_world_yaw_des_sum;
    
// #endif
//     G_vofa.m_data_send_frame.m_data[0] = G_sentry.m_robot_radar_world_yaw;
//     G_vofa.m_data_send_frame.m_data[1] = G_sentry.m_robot_chassis_world_error;
    
//     G_vofa.m_data_send_frame.m_data[4] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FLA_MOTOR]->m_angle_target;
//     G_vofa.m_data_send_frame.m_data[5] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FLA_MOTOR]->m_angle_current;
//     G_vofa.m_data_send_frame.m_data[6] = G_sentry.m_robot_wheel_world_yaw;
//     G_vofa.m_data_send_frame.m_data[7] = G_sentry.m_robot_chassis_speed;
    
//     G_vofa.m_data_send_frame.m_data[8] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_current_encoder_filter;
//     G_vofa.m_data_send_frame.m_data[9] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_angle_current;
//     G_vofa.m_data_send_frame.m_data[10] = G_navigation.m_data_receive_frame.m_data_f[3];
//     G_vofa.m_data_send_frame.m_data[11] = G_navigation.m_data_receive_frame.m_data_f[4];
    // G_vofa.m_data_send_frame.m_data[12] = G_navigation.m_data_receive_frame.m_data_f[5];
    // G_vofa.m_data_send_frame.m_data[13] = G_navigation.m_data_receive_frame.m_data_f[6];
    // G_vofa.m_data_send_frame.m_data[14] = G_navigation.m_data_receive_frame.m_data_c[0];
    // G_vofa.m_data_send_frame.m_data[12] = G_aim_assist.usart_assist->USART_RT.DMAt_Streamx->M0AR;
    // G_vofa.m_data_send_frame.m_data[13] = (uint32_t)G_aim_assist.usart_assist->USART_RT.pTxbuf;
    // G_vofa.m_data_send_frame.m_data[14] = DMA2_Stream6 ->M0AR;
    // G_vofa.m_data_send_frame.m_data[15] = (uint32_t)&G_aim_assist.m_data_send_frame;

    G_vofa.m_data_send_frame.m_data[0] = can2_context.CANx_TxMsg.Data[0];
    G_vofa.m_data_send_frame.m_data[1] = can2_context.CANx_TxMsg.Data[1];
    
    G_vofa.m_data_send_frame.m_data[2] = (uint8_t)((int16_t)G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_speed_pid->m_output >> 8);
    G_vofa.m_data_send_frame.m_data[3] = (uint8_t)((int16_t)G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_speed_pid->m_output);
    G_vofa.m_data_send_frame.m_data[4] = can2_context.CANx_TxMsg.Data[4];
    // G_vofa.m_data_send_frame.m_data[5] = can1_context.CANx_TxMsg.Data[0];

    // G_vofa.m_data_send_frame.m_data[6] = can1_context.CANx_TxMsg.Data[1];
    // G_vofa.m_data_send_frame.m_data[7] = can1_context.CANx_TxMsg.Data[2];
    // G_vofa.m_data_send_frame.m_data[8] = can1_context.CANx_TxMsg.Data[3];
    // G_vofa.m_data_send_frame.m_data[9] = can1_context.CANx_TxMsg.Data[4];
    // G_vofa.m_data_send_frame.m_data[10] = can1_context.CANx_TxMsg.Data[5];
    G_vofa.m_data_send_frame.m_data[11] = can2_context.CANx_TxMsg.Data[2];
    G_vofa.m_data_send_frame.m_data[12] = can2_context.CANx_TxMsg.Data[3];
    G_vofa.m_data_send_frame.m_data[15] = G_system_monitor.UART2_rx_fps;
    G_vofa.m_data_send_frame.m_data[16] = G_system_monitor.UART1_rx_fps;

    // FPS
    G_vofa.m_data_send_frame.m_data[17] = G_system_monitor.UART5_rx_fps;
    G_vofa.m_data_send_frame.m_data[18] = G_system_monitor.CAN1_rx_fps;
    G_vofa.m_data_send_frame.m_data[19] = G_system_monitor.CAN2_rx_fps;
}



/**
 *@brief visualize chassis datas
 *
 *@param
 */
void VisualizeChassisData(void)
{
    // // Chassis YAW motor angle current
    // G_vofa.m_data_send_frame.m_data[0] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FLA_MOTOR]->m_angle_current;
    // G_vofa.m_data_send_frame.m_data[1] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FRA_MOTOR]->m_angle_current;
    // G_vofa.m_data_send_frame.m_data[2] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BLA_MOTOR]->m_angle_current;
    // G_vofa.m_data_send_frame.m_data[3] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BRA_MOTOR]->m_angle_current; 

    // // Chassis YAW motor angle target
    // G_vofa.m_data_send_frame.m_data[4] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FLA_MOTOR]->m_angle_target;
    // G_vofa.m_data_send_frame.m_data[5] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FRA_MOTOR]->m_angle_target;
    // G_vofa.m_data_send_frame.m_data[6] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BLA_MOTOR]->m_angle_target;
    // G_vofa.m_data_send_frame.m_data[7] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BRA_MOTOR]->m_angle_target;

    // // Chassis YAW motor encoder angle current
    // G_vofa.m_data_send_frame.m_data[8] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FLA_MOTOR]->m_angle_current_encoder;
    // G_vofa.m_data_send_frame.m_data[9] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FRA_MOTOR]->m_angle_current_encoder;
    // G_vofa.m_data_send_frame.m_data[10] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BLA_MOTOR]->m_angle_current_encoder;
    // G_vofa.m_data_send_frame.m_data[11] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BRA_MOTOR]->m_angle_current_encoder;

    // // Chassis YAW motor encoder filter angle current
    // G_vofa.m_data_send_frame.m_data[12] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FLA_MOTOR]->m_angle_current_encoder_filter;
    // G_vofa.m_data_send_frame.m_data[13] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FRA_MOTOR]->m_angle_current_encoder_filter;
    // G_vofa.m_data_send_frame.m_data[14] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BLA_MOTOR]->m_angle_current_encoder_filter;
    // G_vofa.m_data_send_frame.m_data[15] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BRA_MOTOR]->m_angle_current_encoder_filter;

    // Chassis YAW motor external angle current
    // G_vofa.m_data_send_frame.m_data[0] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FLA_MOTOR]->m_angle_current_external;
    // G_vofa.m_data_send_frame.m_data[1] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FRA_MOTOR]->m_angle_current_external;
    // G_vofa.m_data_send_frame.m_data[2] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BLA_MOTOR]->m_angle_current_external;
    // G_vofa.m_data_send_frame.m_data[3] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BRA_MOTOR]->m_angle_current_external;

    // Chassis YAW motor encoder speed target
    // G_vofa.m_data_send_frame.m_data[15] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FLA_MOTOR]->m_speed_target;
    // G_vofa.m_data_send_frame.m_data[1] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FRA_MOTOR]->m_speed_current_encoder;
    // G_vofa.m_data_send_frame.m_data[2] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BLA_MOTOR]->m_speed_current_encoder;
    // G_vofa.m_data_send_frame.m_data[3] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BRA_MOTOR]->m_speed_current_encoder;

    // Chassis YAW motor encoder speed current
    // G_vofa.m_data_send_frame.m_data[16] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FLA_MOTOR]->m_speed_current_encoder;
    // G_vofa.m_data_send_frame.m_data[1] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FRA_MOTOR]->m_speed_current_encoder;
    // G_vofa.m_data_send_frame.m_data[2] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BLA_MOTOR]->m_speed_current_encoder;
    // G_vofa.m_data_send_frame.m_data[3] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BRA_MOTOR]->m_speed_current_encoder;

    // Chassis YAW motor encoder filter speed current
    // G_vofa.m_data_send_frame.m_data[17] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FLA_MOTOR]->m_speed_current_encoder_filter;
    // G_vofa.m_data_send_frame.m_data[5] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FRA_MOTOR]->m_speed_current_encoder_filter;
    // G_vofa.m_data_send_frame.m_data[6] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BLA_MOTOR]->m_speed_current_encoder_filter;
    // G_vofa.m_data_send_frame.m_data[7] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BRA_MOTOR]->m_speed_current_encoder_filter;

    // // Chassis LINE motor speed current
    // G_vofa.m_data_send_frame.m_data[0] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FLL_MOTOR]->m_speed_current;
    // G_vofa.m_data_send_frame.m_data[1] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FRL_MOTOR]->m_speed_current;
    // G_vofa.m_data_send_frame.m_data[2] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BLL_MOTOR]->m_speed_current;
    // G_vofa.m_data_send_frame.m_data[3] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BRL_MOTOR]->m_speed_current;

    // // Chassis LINE motor speed target
    // G_vofa.m_data_send_frame.m_data[4] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FLL_MOTOR]->m_speed_target;
    // G_vofa.m_data_send_frame.m_data[5] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FRL_MOTOR]->m_speed_target;
    // G_vofa.m_data_send_frame.m_data[6] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BLL_MOTOR]->m_speed_target;
    // G_vofa.m_data_send_frame.m_data[7] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BRL_MOTOR]->m_speed_target;

    // // Chassis LINE motor encoder speed current
    // G_vofa.m_data_send_frame.m_data[8] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FLL_MOTOR]->m_speed_current_encoder;
    // G_vofa.m_data_send_frame.m_data[9] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FRL_MOTOR]->m_speed_current_encoder;
    // G_vofa.m_data_send_frame.m_data[10] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BLL_MOTOR]->m_speed_current_encoder;
    // G_vofa.m_data_send_frame.m_data[11] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BRL_MOTOR]->m_speed_current_encoder;

    // // Chassis LINE motor encoder filter speed current
    // G_vofa.m_data_send_frame.m_data[12] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FLL_MOTOR]->m_speed_current_encoder_filter;
    // G_vofa.m_data_send_frame.m_data[13] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FRL_MOTOR]->m_speed_current_encoder_filter;
    // G_vofa.m_data_send_frame.m_data[14] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BLL_MOTOR]->m_speed_current_encoder_filter;
    // G_vofa.m_data_send_frame.m_data[15] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BRL_MOTOR]->m_speed_current_encoder_filter;

    // Chassis LINE motor current output
    // G_vofa.m_data_send_frame.m_data[16] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FLL_MOTOR]->m_speed_pid->m_output;
    // G_vofa.m_data_send_frame.m_data[13] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FRL_MOTOR]->m_speed_pid->m_output;
    // G_vofa.m_data_send_frame.m_data[14] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BLL_MOTOR]->m_speed_pid->m_output;
    // G_vofa.m_data_send_frame.m_data[15] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BRL_MOTOR]->m_speed_pid->m_output;

    // Chassis LINE motor speed error
    // G_vofa.m_data_send_frame.m_data[17] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FLL_MOTOR]->m_speed_pid->m_error;
    // G_vofa.m_data_send_frame.m_data[13] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FRL_MOTOR]->m_speed_pid->m_error;
    // G_vofa.m_data_send_frame.m_data[14] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BLL_MOTOR]->m_speed_pid->m_error;
    // G_vofa.m_data_send_frame.m_data[15] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BRL_MOTOR]->m_speed_pid->m_error;

    // Chassis motor CAN receive update times;
    // G_vofa.m_data_send_frame.m_data[8] = G_sentry.capacitor->m_state_update_times;
    // G_vofa.m_data_send_frame.m_data[9] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_state_update_times; 
    // G_vofa.m_data_send_frame.m_data[10] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FLA_MOTOR]->m_state_update_times;
    // G_vofa.m_data_send_frame.m_data[11] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FRA_MOTOR]->m_state_update_times;
    // G_vofa.m_data_send_frame.m_data[12] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BLA_MOTOR]->m_state_update_times;
    // G_vofa.m_data_send_frame.m_data[13] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BRA_MOTOR]->m_state_update_times;
    // G_vofa.m_data_send_frame.m_data[14] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FLL_MOTOR]->m_state_update_times;
    // G_vofa.m_data_send_frame.m_data[15] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FRL_MOTOR]->m_state_update_times;
    // G_vofa.m_data_send_frame.m_data[16] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BLL_MOTOR]->m_state_update_times;
    // G_vofa.m_data_send_frame.m_data[17] = G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BRL_MOTOR]->m_state_update_times;

    // CAN send context
    // G_vofa.m_data_send_frame.m_data[10] = (int16_t)((uint16_t)can2_context.tx_data[0] << 8 | (uint16_t)can2_context.tx_data[1]);
    // G_vofa.m_data_send_frame.m_data[11] = (int16_t)((uint16_t)can2_context.tx_data[2] << 8 | (uint16_t)can2_context.tx_data[3]);
    // G_vofa.m_data_send_frame.m_data[12] = (int16_t)((uint16_t)can2_context.tx_data[4] << 8 | (uint16_t)can2_context.tx_data[5]);
    // G_vofa.m_data_send_frame.m_data[13] = (int16_t)((uint16_t)can2_context.tx_data[6] << 8 | (uint16_t)can2_context.tx_data[7]);

    // CAN receive fps
        // Chassis YAW motor angle current
    G_vofa.m_data_send_frame.m_data[0] = G_sentry.capacitor->m_cap_vol;
    G_vofa.m_data_send_frame.m_data[1] = G_sentry.capacitor->m_pow_out;
    // G_vofa.m_data_send_frame.m_data[2] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BLA_MOTOR]->m_angle_current;
    // G_vofa.m_data_send_frame.m_data[3] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BRA_MOTOR]->m_angle_current; 

    // // Chassis YAW motor angle target
    // G_vofa.m_data_send_frame.m_data[4] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FLA_MOTOR]->m_angle_target;
    // G_vofa.m_data_send_frame.m_data[5] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FRA_MOTOR]->m_angle_target;
    // G_vofa.m_data_send_frame.m_data[6] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BLA_MOTOR]->m_angle_target;
    // G_vofa.m_data_send_frame.m_data[7] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BRA_MOTOR]->m_angle_target;

    // // Chassis YAW motor encoder angle current
    // G_vofa.m_data_send_frame.m_data[8] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FLA_MOTOR]->m_angle_current_encoder;
    // G_vofa.m_data_send_frame.m_data[9] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FRA_MOTOR]->m_angle_current_encoder;
    // G_vofa.m_data_send_frame.m_data[10] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BLA_MOTOR]->m_angle_current_encoder;
    // G_vofa.m_data_send_frame.m_data[11] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BRA_MOTOR]->m_angle_current_encoder;

    // // Chassis YAW motor encoder filter angle current
    // G_vofa.m_data_send_frame.m_data[12] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FLA_MOTOR]->m_angle_current_encoder_filter;
    // G_vofa.m_data_send_frame.m_data[13] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FRA_MOTOR]->m_angle_current_encoder_filter;
    // G_vofa.m_data_send_frame.m_data[14] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BLA_MOTOR]->m_angle_current_encoder_filter;
    // G_vofa.m_data_send_frame.m_data[15] = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BRA_MOTOR]->m_angle_current_encoder_filter;

    G_vofa.m_data_send_frame.m_data[18] = G_system_monitor.CAN1_rx_fps;
    G_vofa.m_data_send_frame.m_data[19] = G_system_monitor.CAN2_rx_fps;
}



/**
 *@brief visualize chassis datas
 *
 *@param
 */
void VisualizeGimbalData(void)
{
    // Communication receive datas
    // G_vofa.m_data_send_frame.m_data[0] = G_gimbal.m_data_receive_frame.m_fdata[0];
    // G_vofa.m_data_send_frame.m_data[1] = G_gimbal.m_data_receive_frame.m_fdata[1];
    // G_vofa.m_data_send_frame.m_data[2] = G_gimbal.m_data_receive_frame.m_fdata[2];
    // G_vofa.m_data_send_frame.m_data[3] = G_gimbal.m_data_receive_frame.m_fdata[3];
    // G_vofa.m_data_send_frame.m_data[4] = G_gimbal.m_data_receive_frame.m_fdata[4];
    // G_vofa.m_data_send_frame.m_data[0] = G_gimbal.m_data_receive_frame.m_fdata[5];

    // Communication send datas
    // G_vofa.m_data_send_frame.m_data[0] = G_gimbal.m_data_send_frame.m_sdata[0];
    // G_vofa.m_data_send_frame.m_data[1] = G_gimbal.m_data_send_frame.m_sdata[1];
    // G_vofa.m_data_send_frame.m_data[2] = G_gimbal.m_data_send_frame.m_sdata[2];
    // G_vofa.m_data_send_frame.m_data[3] = G_gimbal.m_data_send_frame.m_cdata[0];
    // G_vofa.m_data_send_frame.m_data[4] = G_gimbal.m_data_send_frame.m_cdata[1];
    // G_vofa.m_data_send_frame.m_data[5] = G_gimbal.m_data_send_frame.m_cdata[2];
    // G_vofa.m_data_send_frame.m_data[4] = G_gimbal.m_data_send_frame.m_cdata[3];
    // G_vofa.m_data_send_frame.m_data[5] = G_gimbal.m_data_send_frame.m_cdata[4];

    // Gimbal angle current
    G_vofa.m_data_send_frame.m_data[0] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_current;
    G_vofa.m_data_send_frame.m_data[1] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_angle_current;
    G_vofa.m_data_send_frame.m_data[2] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_current;

    // Gimbal angle target
    G_vofa.m_data_send_frame.m_data[3] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_target;
    G_vofa.m_data_send_frame.m_data[4] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_angle_target;
    G_vofa.m_data_send_frame.m_data[5] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_target;

    // Gimbal angle current encoder
    // G_vofa.m_data_send_frame.m_data[3] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_current_encoder;
    // G_vofa.m_data_send_frame.m_data[5] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_current_encoder;
    // G_vofa.m_data_send_frame.m_data[5] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_angle_current_encoder;

    // Gimbal angle current encoder kalman filter
    // G_vofa.m_data_send_frame.m_data[6] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_current_encoder_filter;
    // G_vofa.m_data_send_frame.m_data[6] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_current_encoder_filter;
    // G_vofa.m_data_send_frame.m_data[3] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_angle_current_encoder_filter;

    // Gimbal angle current external
    // G_vofa.m_data_send_frame.m_data[6] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_current_external;
    // G_vofa.m_data_send_frame.m_data[5] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_current_external;
    // G_vofa.m_data_send_frame.m_data[4] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_angle_current_external;

    // Gimbal speed current
    // G_vofa.m_data_send_frame.m_data[4] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_speed_current;
    // G_vofa.m_data_send_frame.m_data[5] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_speed_current;
    // G_vofa.m_data_send_frame.m_data[11] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_speed_current;
    // G_vofa.m_data_send_frame.m_data[6] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_pid->m_output;
    // G_vofa.m_data_send_frame.m_data[7] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_pid->m_output_i;
    // G_vofa.m_data_send_frame.m_data[8] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_pid->m_output_d;
    G_vofa.m_data_send_frame.m_data[6] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_td->m_x1;
    G_vofa.m_data_send_frame.m_data[7] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_angle_td->m_x1;
    
    // Gimbal speed target
    // G_vofa.m_data_send_frame.m_data[6] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_speed_target;
    // G_vofa.m_data_send_frame.m_data[9] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_speed_target;
    // G_vofa.m_data_send_frame.m_data[8] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_speed_target;

    // Gimbal angle current encoder
    // G_vofa.m_data_send_frame.m_data[9] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_speed_current_encoder;
    // G_vofa.m_data_send_frame.m_data[10] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_speed_current_encoder;
    // G_vofa.m_data_send_frame.m_data[11] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_speed_current_encoder;

    // Gimbal angle current encoder kalman filter
    // G_vofa.m_data_send_frame.m_data[6] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_speed_current_encoder_filter;
    // G_vofa.m_data_send_frame.m_data[11] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_speed_current_encoder_filter;
    // G_vofa.m_data_send_frame.m_data[8] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_speed_current_encoder_filter;

    // Gimbal angle current external
    // G_vofa.m_data_send_frame.m_data[6] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_speed_current_external;
    // G_vofa.m_data_send_frame.m_data[12] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_speed_current_external;
    // G_vofa.m_data_send_frame.m_data[9] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_speed_current_external;

    // fps
    G_vofa.m_data_send_frame.m_data[18] = G_system_monitor.UART3_rx_fps;
    G_vofa.m_data_send_frame.m_data[19] = G_system_monitor.CAN2_rx_fps;
}



/**
 *@brief visualize shoot datas
 *
 *@param
 */
void VisualizeShootData(void)
{
    // Shoot frame work states
    G_vofa.m_data_send_frame.m_data[0] = G_sentry.shoot_mode;
    G_vofa.m_data_send_frame.m_data[1] = G_sentry.shoot_insurance;
    G_vofa.m_data_send_frame.m_data[2] = G_sentry.m_shoot_bullet_cnt_target;
    G_vofa.m_data_send_frame.m_data[3] = G_sentry.shoot_motor[SentryRobot::SHOOT_DRIVE_MOTOR]->m_angle_target;
    G_vofa.m_data_send_frame.m_data[4] = G_sentry.shoot_motor[SentryRobot::SHOOT_DRIVE_MOTOR]->m_angle_current;

    // Shoot Message from refree
    G_vofa.m_data_send_frame.m_data[5] = G_referee.ShootData.shooter_id;
    G_vofa.m_data_send_frame.m_data[6] = G_referee.ShootData.bullet_freq;
    G_vofa.m_data_send_frame.m_data[7] = G_referee.ShootData.bullet_speed;
    G_vofa.m_data_send_frame.m_data[8] = G_referee.BulletRemaining.bullet_remaining_num_17mm;
    G_vofa.m_data_send_frame.m_data[9] = G_referee.PowerHeatData.shooter_id1_17mm_cooling_heat;
    G_vofa.m_data_send_frame.m_data[10] = G_referee.PowerHeatData.shooter_id2_17mm_cooling_heat;
    G_vofa.m_data_send_frame.m_data[11] = G_referee.GameRobotStatus.remain_HP;
    G_vofa.m_data_send_frame.m_data[12] = G_referee.GameRobotStatus.shooter_id2_17mm_cooling_rate;
    G_vofa.m_data_send_frame.m_data[13] = G_referee.GameRobotStatus.shooter_id2_17mm_cooling_limit;

    // Check shoot pid output
    // G_vofa.m_data_send_frame.m_data[14] = (int16_t)((uint16_t)can1_context.tx_data[0] << 8 | (uint16_t)can1_context.tx_data[1]);
    // G_vofa.m_data_send_frame.m_data[15] = (int16_t)((uint16_t)can1_context.tx_data[2] << 8 | (uint16_t)can1_context.tx_data[3]);
    // G_vofa.m_data_send_frame.m_data[16] = (int16_t)((uint16_t)can1_context.tx_data[4] << 8 | (uint16_t)can1_context.tx_data[5]);
    // G_vofa.m_data_send_frame.m_data[17] = (int16_t)((uint16_t)can1_context.tx_data[6] << 8 | (uint16_t)can1_context.tx_data[7]);
    // G_vofa.m_data_send_frame.m_data[18] = G_sentry.shoot_motor[SentryRobot::SHOOT_DRIVE_MOTOR]->m_speed_pid->m_output;
    
    //Real shoot fps
    G_vofa.m_data_send_frame.m_data[14] = G_sentry.m_shoot_bullet_fps;

    G_vofa.m_data_send_frame.m_data[15] = G_referee.GameStatus.game_progress;
    G_vofa.m_data_send_frame.m_data[16] = G_sentry.change_gun_flag;
    G_vofa.m_data_send_frame.m_data[17] = G_sentry.m_shoot_cnt;
    G_vofa.m_data_send_frame.m_data[18] = G_sentry.m_shoot_speed_filter->m_avg;
    // FPS
    G_vofa.m_data_send_frame.m_data[19] = G_system_monitor.CAN1_rx_fps;
}



/**
 *@brief visualize shoot datas
 *
 *@param
 */
void VisualizeAimAssistData(void) {
    // Communication receive datas
    G_vofa.m_data_send_frame.m_data[0] = G_aim_assist.m_data_receive_frame.m_id;
    G_vofa.m_data_send_frame.m_data[1] = G_aim_assist.m_data_receive_frame.m_data_f[0];
    G_vofa.m_data_send_frame.m_data[2] = G_aim_assist.m_data_receive_frame.m_data_f[1];
    G_vofa.m_data_send_frame.m_data[3] = G_aim_assist.m_data_receive_frame.m_data_f[2];
    G_vofa.m_data_send_frame.m_data[4] = G_aim_assist.m_data_receive_frame.m_data_f[3];
    G_vofa.m_data_send_frame.m_data[5] = G_aim_assist.m_data_receive_frame.m_data_f[4];
    G_vofa.m_data_send_frame.m_data[6] = G_aim_assist.m_data_receive_frame.m_data_f[5];
    G_vofa.m_data_send_frame.m_data[7] = G_aim_assist.m_data_receive_frame.m_data_f[6];
    G_vofa.m_data_send_frame.m_data[8] = G_aim_assist.m_data_receive_frame.m_data_f[7];
    G_vofa.m_data_send_frame.m_data[9] = G_aim_assist.m_data_receive_frame.m_data_f[8];

    // G_vofa.m_data_send_frame.m_data[9] = G_sentry.m_robot_radar_world_yaw;
    // G_vofa.m_data_send_frame.m_data[10] = G_sentry.m_robot_chassis_world_error;
    // G_vofa.m_data_send_frame.m_data[12] = G_sentry.m_robot_wheel_world_yaw;
    // G_vofa.m_data_send_frame.m_data[13] = G_system_monitor.UART5_rx_fps;

    // Communication send datas
    // G_vofa.m_data_send_frame.m_data[9] = G_aim_assist.m_data_send_frame.m_id;
    // G_vofa.m_data_send_frame.m_data[9] = G_aim_assist.m_data_send_frame.m_data[0];
    // G_vofa.m_data_send_frame.m_data[10] = G_aim_assist.m_data_send_frame.m_data[1];
    // G_vofa.m_data_send_frame.m_data[11] = G_aim_assist.m_data_send_frame.m_data[2];
    // G_vofa.m_data_send_frame.m_data[12] = G_aim_assist.m_data_send_frame.m_data[3];
    // G_vofa.m_data_send_frame.m_data[13] = G_aim_assist.m_data_send_frame.m_data[4];
    // G_vofa.m_data_send_frame.m_data[14] = G_aim_assist.m_data_send_frame.m_data[5];
    // G_vofa.m_data_send_frame.m_data[15] = G_aim_assist.m_data_send_frame.m_data[6];
    // G_vofa.m_data_send_frame.m_data[16] = G_aim_assist.m_data_send_frame.m_data[7];
    // G_vofa.m_data_send_frame.m_data[17] = G_aim_assist.m_data_send_frame.m_data[8];
    // G_vofa.m_data_send_frame.m_data[18] = G_aim_assist.m_data_send_frame.m_data[9];

    // Aim assist 
//    G_vofa.m_data_send_frame.m_data[0] = G_sentry.enemy_find_flag;
//    G_vofa.m_data_send_frame.m_data[1] = G_aim_assist_pitch_angle_des;
    // G_vofa.m_data_send_frame.m_data[9] = G_aim_assist_yaw_angle_des;
    // G_vofa.m_data_send_frame.m_data[14] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_current;
    // // // G_vofa.m_data_send_frame.m_data[11] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_current;
    // G_vofa.m_data_send_frame.m_data[15] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_target;
    // // // G_vofa.m_data_send_frame.m_data[13] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_target;
	// // 	G_vofa.m_data_send_frame.m_data[9] = G_referee.RobotCommand.commd_keyboard;
    G_vofa.m_data_send_frame.m_data[9] = G_sentry. gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_pid->m_kp;
    G_vofa.m_data_send_frame.m_data[10] = G_system_monitor.UART4_rx_fps;;
    G_vofa.m_data_send_frame.m_data[11] = G_sentry.first_yaw_angle_des;
    G_vofa.m_data_send_frame.m_data[12] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_angle_target;
    G_vofa.m_data_send_frame.m_data[13] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_angle_current;
    G_vofa.m_data_send_frame.m_data[14] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_target;
    G_vofa.m_data_send_frame.m_data[15] =  G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_current;
    G_vofa.m_data_send_frame.m_data[16] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_target;
    G_vofa.m_data_send_frame.m_data[17] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_current;


    //FPS
    G_vofa.m_data_send_frame.m_data[18] = G_sentry.first_yaw_scan_dir;
    G_vofa.m_data_send_frame.m_data[19] = G_system_monitor.UART6_rx_fps;
}



/**
 *@brief visualize refree datas
 *
 *@param
 */
void VisualizeRefreeData(void) {
    // Power heat
    // G_vofa.m_data_send_frame.m_data[0] = G_referee.PowerHeatData.chassis_power;
    // G_vofa.m_data_send_frame.m_data[1] = G_referee.PowerHeatData.chassis_power_buffer;
    // G_vofa.m_data_send_frame.m_data[2] = G_referee.PowerHeatData.chassis_volt;
    // G_vofa.m_data_send_frame.m_data[3] = G_referee.PowerHeatData.shooter_id1_17mm_cooling_heat;
    // G_vofa.m_data_send_frame.m_data[4] = G_referee.PowerHeatData.shooter_id2_17mm_cooling_heat;

    // // Robot command
    // G_vofa.m_data_send_frame.m_data[5] = G_referee.RobotCommand.commd_keyboard;
    // G_vofa.m_data_send_frame.m_data[6] = G_referee.RobotCommand.target_position_x;
    // G_vofa.m_data_send_frame.m_data[7] = G_referee.RobotCommand.target_position_y;
    // G_vofa.m_data_send_frame.m_data[8] = G_referee.RobotCommand.target_position_z;
    // G_vofa.m_data_send_frame.m_data[9] = G_referee.RobotCommand.target_robot_ID;

    // // Game status
    // G_vofa.m_data_send_frame.m_data[10] = G_referee.GameStatus.game_type;
    // G_vofa.m_data_send_frame.m_data[11] = G_referee.GameStatus.game_progress;
    // G_vofa.m_data_send_frame.m_data[12] = G_referee.GameStatus.stage_remain_time;
    // G_vofa.m_data_send_frame.m_data[13] = G_referee.GameStatus.SyncTimeStamp;

    // // FPS
    // G_vofa.m_data_send_frame.m_data[15] = G_referee_monitor.GameStatus_fps;
    // G_vofa.m_data_send_frame.m_data[16] = G_referee_monitor.RobotCommand_fps;
    // G_vofa.m_data_send_frame.m_data[17] = G_referee_monitor.GameRobotStatus_fps;
    // G_vofa.m_data_send_frame.m_data[18] = G_referee_monitor.PowerHeatData_fps;
    // G_vofa.m_data_send_frame.m_data[19] = G_system_monitor.UART2_rx_fps;
		
        uint32_t fps;
        uint32_t fps_can1;
        uint32_t fps_can2;
        uint32_t fps_uart2;
        uint32_t fps_uart4;
        uint32_t fps_uart5;
        uint32_t fps_uart6;

        if(G_system_monitor.CAN1_rx_fps > 5000)
            fps_can1 = 1;
        else
            fps_can1 = 0;
        if(G_system_monitor.CAN2_rx_fps > 6000)
            fps_can2 = 1;
        else
            fps_can2 = 0;
        if(G_system_monitor.UART2_rx_fps > 50)
            fps_uart2 = 1;
        else
            fps_uart2 = 0;
        if(G_system_monitor.UART4_rx_fps > 998)
            fps_uart4 = 1;
        else
            fps_uart4 = 0;
        if(G_system_monitor.UART5_rx_fps > 70)
            fps_uart5 = 1;
        else
            fps_uart5 = 0;
        if(G_system_monitor.UART6_rx_fps > 200)
            fps_uart6 = 1;
        else
            fps_uart6 = 0;

        fps = (fps_can1) | (fps_can2 << 1) | (fps_uart2 << 2) | (fps_uart4 << 3) | (fps_uart5 << 4) | (fps_uart6 << 5);

        G_vofa.m_data_send_frame.m_data[0] = fps;
        G_vofa.m_data_send_frame.m_data[1] = G_referee.GameStatus.game_progress;
        G_vofa.m_data_send_frame.m_data[2] = G_referee.RobotCommand.commd_keyboard;
        G_vofa.m_data_send_frame.m_data[3] = G_referee.GameStatus.stage_remain_time;
        G_vofa.m_data_send_frame.m_data[4] = G_aim_assist.m_data_receive_frame.m_id;
        G_vofa.m_data_send_frame.m_data[5] = G_aim_assist.m_data_receive_frame.m_data_f[0];

        // Robot command
        G_vofa.m_data_send_frame.m_data[6] = G_aim_assist.m_data_receive_frame.m_data_f[1];
        G_vofa.m_data_send_frame.m_data[7] = G_aim_assist.m_data_receive_frame.m_data_f[2];
        G_vofa.m_data_send_frame.m_data[8] = G_aim_assist.m_data_receive_frame.m_data_f[3];
        G_vofa.m_data_send_frame.m_data[9] = G_sentry.capacitor->m_cap_vol;

        // Game status
        G_vofa.m_data_send_frame.m_data[10] = G_referee.GameStatus.game_progress;
        G_vofa.m_data_send_frame.m_data[11] = G_referee.GameStatus.game_progress;
        G_vofa.m_data_send_frame.m_data[12] = G_referee.GameStatus.stage_remain_time;
        G_vofa.m_data_send_frame.m_data[13] = G_referee.GameStatus.SyncTimeStamp;

        // FPS
        G_vofa.m_data_send_frame.m_data[15] = G_referee_monitor.GameStatus_fps;
        G_vofa.m_data_send_frame.m_data[16] = G_referee_monitor.RobotCommand_fps;
        G_vofa.m_data_send_frame.m_data[17] = G_referee_monitor.GameRobotStatus_fps;
        G_vofa.m_data_send_frame.m_data[18] = G_referee_monitor.PowerHeatData_fps;
        G_vofa.m_data_send_frame.m_data[19] = G_system_monitor.UART2_rx_fps;
}



/**
 *@brief send referee datas
 *
 *@param
 */
void SendRefereeData(void)
{
    // TEST
    // G_referee.temp_cnt++;
    // if(G_referee.temp_cnt >= 10){
    //     G_referee.temp_x += 1;
    //     G_referee.temp_y += 1;
    //     G_referee.temp_cnt = 0;
    // }
    // TEST
    G_referee.m_loc_update_cnt++;
    if(G_referee.m_loc_update_cnt >= 10){

#ifdef CHASSIS_STEER_DRIVING_MODE
        if(G_sentry.m_robot_chassis_spin_ready){
            G_referee.MapSentryData.intention = 2; // SPIN
        }else if(!G_sentry.m_robot_chassis_spin_ready && G_sentry.m_robot_chassis_speed > CHASSIS_SPIN_OVER_LIMIT){
            G_referee.MapSentryData.intention = 1; // MOVE
        }else{
            G_referee.MapSentryData.intention = 3; // STOP
        }
#endif
#ifdef CHASSIS_STANDARD_DRIVING_MODE
        G_referee.MapSentryData.intention = 3; // STOP
#endif


        uint16_t current_pos_x = G_navigation.m_data_receive_frame.m_data_f[7]*10;
        uint16_t current_pos_y = G_navigation.m_data_receive_frame.m_data_f[8]*10;
        // uint16_t current_pos_x = G_referee.temp_x; // TEST
        // uint16_t current_pos_y = G_referee.temp_y; // TEST
        if(!G_referee.m_loc_send_init_flag){
            G_referee.m_loc_delta_x_total = 0;
            G_referee.m_loc_delta_y_total = 0;
            G_referee.m_loc_pos_x_pre = current_pos_x;
            G_referee.m_loc_pos_y_pre = current_pos_y;
            for(uint8_t i = 0; i < REFEREE_MAP_SIZE; i++){
                *(G_referee.MapSentryData.delta_x+i) = 0;
                *(G_referee.MapSentryData.delta_y+i) = 0;
            }
            G_referee.m_loc_queue_len = 0;
            G_referee.m_loc_send_init_flag = true;
        }
        int8_t delta_x = current_pos_x - G_referee.m_loc_pos_x_pre;
        int8_t delta_y = current_pos_y - G_referee.m_loc_pos_y_pre;
        if(G_referee.m_loc_queue_len >= REFEREE_MAP_SIZE){
            G_referee.m_loc_delta_x_total -= *(G_referee.MapSentryData.delta_x);
            G_referee.m_loc_delta_y_total -= *(G_referee.MapSentryData.delta_y);
            for(int8_t i = 0; i < REFEREE_MAP_SIZE - 1; i++){
                *(G_referee.MapSentryData.delta_x + i) = *(G_referee.MapSentryData.delta_x + i + 1);
                *(G_referee.MapSentryData.delta_y + i) = *(G_referee.MapSentryData.delta_y + i + 1);
            }
            G_referee.m_loc_queue_len--;
        }
        *(G_referee.MapSentryData.delta_x + G_referee.m_loc_queue_len) = delta_x;
        *(G_referee.MapSentryData.delta_y + G_referee.m_loc_queue_len) = delta_y;
        G_referee.m_loc_delta_x_total += delta_x;
        G_referee.m_loc_delta_y_total += delta_y;
        G_referee.m_loc_queue_len++;
        G_referee.MapSentryData.start_position_x = current_pos_x - G_referee.m_loc_delta_x_total;
        G_referee.MapSentryData.start_position_y = current_pos_y - G_referee.m_loc_delta_y_total;
        G_referee.m_loc_pos_x_pre = current_pos_x;
        G_referee.m_loc_pos_y_pre = current_pos_y;
        G_referee.m_loc_update_cnt = 0;
    }
    G_referee.m_loc_send_cnt++;
    if(G_referee.m_loc_send_cnt >= 100){
        memcpy(&G_referee.m_data_send_buff[7],&G_referee.MapSentryData,sizeof(G_referee.MapSentryData));
        Append_CRC16_Check_Sum(G_referee.m_data_send_buff,REFEREE_SEND_SIZE);
        G_referee.SendData();
        G_referee.m_loc_send_cnt = 0;
    }
}




/**
 *@brief send navigation datas
 *
 *@param
 */
void SendNavigationData(void)
{
    G_navigation.m_data_send_frame.m_data[2] = G_sentry.m_robot_chassis_speed;
    G_navigation.m_data_send_frame.m_data[3] = G_sentry.m_robot_wheel_world_yaw; // warning
    G_navigation.m_data_send_frame.m_data[4] = G_aim_assist.m_data_receive_frame.m_data_f[4];
    G_navigation.m_data_send_frame.m_data[5] = G_aim_assist.m_data_receive_frame.m_data_f[5];
    G_navigation.m_data_send_frame.m_data[6] = G_aim_assist.m_data_receive_frame.m_data_f[6];
    G_navigation.m_data_send_frame.m_data[7] = G_aim_assist.m_data_receive_frame.m_data_f[7];
    G_navigation.m_data_send_frame.m_data[8] = G_aim_assist.m_data_receive_frame.m_data_f[8];

    uint32_t navigation_data_num = sizeof(G_navigation.m_data_send_frame);
    if (G_navigation.navigation_send_buff_cnt + navigation_data_num + NAVIGATION_SEND_HEAD_SIZE < NAVIGATION_SEND_BUFF_SIZE)
    {
        G_navigation.navigation_send_buff[G_navigation.navigation_send_buff_cnt++] = 0xAB;
        G_navigation.navigation_send_buff[G_navigation.navigation_send_buff_cnt++] = 0xA0;
        G_navigation.navigation_send_buff[G_navigation.navigation_send_buff_cnt++] = navigation_data_num;
        memcpy(&G_navigation.navigation_send_buff[G_navigation.navigation_send_buff_cnt], &G_navigation.m_data_send_frame, navigation_data_num);
        G_navigation.navigation_send_buff_cnt += navigation_data_num;
    }
    G_navigation.usart_navigation->USART_RT.txlen = G_navigation.navigation_send_buff_cnt;
    G_navigation.SendData();
    G_navigation.navigation_send_buff_cnt = 0;

    G_navigation.m_data_send_frame.m_id = 0x00;
}



/**
 *@brief send navigation datas
 *
 *@param
 */
void SendAimAssistData(void) {
    if (G_referee.GameRobotStatus.robot_id == ROBOT_ID_SENTRY_RED) {
        // 這個判斷放這裏好像不太對
        G_aim_assist.m_data_send_frame.m_id = 0x09;
        if(G_referee.m_enemy_1_HP_pre == 0 && G_referee.GameRobotHP.blue_1_robot_HP != 0){
            G_aim_assist.m_enemy_1_res_flag = true;
        }
        if(G_referee.m_enemy_2_HP_pre == 0 && G_referee.GameRobotHP.blue_2_robot_HP != 0){
            G_aim_assist.m_enemy_2_res_flag = true;
        }
        if(G_referee.m_enemy_3_HP_pre == 0 && G_referee.GameRobotHP.blue_3_robot_HP != 0){
            G_aim_assist.m_enemy_3_res_flag = true;
        }
        if(G_referee.m_enemy_4_HP_pre == 0 && G_referee.GameRobotHP.blue_4_robot_HP != 0){
            G_aim_assist.m_enemy_4_res_flag = true;
        }
        if(G_referee.m_enemy_5_HP_pre == 0 && G_referee.GameRobotHP.blue_5_robot_HP != 0){
            G_aim_assist.m_enemy_5_res_flag = true;
        }
        if(G_referee.GameRobotHP.blue_outpost_HP == 0){
            G_aim_assist.m_enemy_7_res_flag = false;
        }else{
            G_aim_assist.m_enemy_7_res_flag = true;
        }
        if(G_aim_assist.m_enemy_1_res_flag == true && G_aim_assist.m_enemy_1_res_cnt <= 10000){
            G_aim_assist.m_enemy_1_res_cnt++;
        }else{
            G_aim_assist.m_enemy_1_res_flag = false;
            G_aim_assist.m_enemy_1_res_cnt = 0;
        }
        if(G_aim_assist.m_enemy_2_res_flag == true && G_aim_assist.m_enemy_2_res_cnt <= 10000){
            G_aim_assist.m_enemy_2_res_cnt++;
        }else{
            G_aim_assist.m_enemy_2_res_flag = false;
            G_aim_assist.m_enemy_2_res_cnt = 0;
        }
        if(G_aim_assist.m_enemy_3_res_flag == true && G_aim_assist.m_enemy_3_res_cnt <= 10000){
            G_aim_assist.m_enemy_3_res_cnt++;
        }else{
            G_aim_assist.m_enemy_3_res_flag = false;
            G_aim_assist.m_enemy_3_res_cnt = 0;
        }
        if(G_aim_assist.m_enemy_4_res_flag == true && G_aim_assist.m_enemy_4_res_cnt <= 10000){
            G_aim_assist.m_enemy_4_res_cnt++;
        }else{
            G_aim_assist.m_enemy_4_res_flag = false;
            G_aim_assist.m_enemy_4_res_cnt = 0;
        }
        if(G_aim_assist.m_enemy_5_res_flag == true && G_aim_assist.m_enemy_5_res_cnt <= 10000){
            G_aim_assist.m_enemy_5_res_cnt++;
        }else{
            G_aim_assist.m_enemy_5_res_flag = false;
            G_aim_assist.m_enemy_5_res_cnt = 0;
        }
        G_referee.m_enemy_1_HP_pre = G_referee.GameRobotHP.blue_1_robot_HP;
        G_referee.m_enemy_2_HP_pre = G_referee.GameRobotHP.blue_2_robot_HP;
        G_referee.m_enemy_3_HP_pre = G_referee.GameRobotHP.blue_3_robot_HP;
        G_referee.m_enemy_4_HP_pre = G_referee.GameRobotHP.blue_4_robot_HP;
        G_referee.m_enemy_5_HP_pre = G_referee.GameRobotHP.blue_5_robot_HP;
        if(G_referee.GameStatus.game_progress == 4 && !G_aim_assist.balance_infantry_flag)
        {
            G_aim_assist.balance_infantry_flag = true;
            if(G_referee.GameRobotHP.blue_3_robot_HP == 300)   G_aim_assist.balance_infantry_num += 3;
            if(G_referee.GameRobotHP.blue_4_robot_HP == 300)   G_aim_assist.balance_infantry_num += 4;
            if(G_referee.GameRobotHP.blue_5_robot_HP == 300)   G_aim_assist.balance_infantry_num += 5;
        
        }
        if(G_referee.GameStatus.game_progress != 4)
        {
            G_aim_assist.balance_infantry_flag = false;
            G_aim_assist.balance_infantry_num = 0;
        }
    } else {
        G_aim_assist.m_data_send_frame.m_id = 0x0A;
        if(G_referee.m_enemy_1_HP_pre == 0 && G_referee.GameRobotHP.red_1_robot_HP != 0){
            G_aim_assist.m_enemy_1_res_flag = true;
        }
        if(G_referee.m_enemy_2_HP_pre == 0 && G_referee.GameRobotHP.red_2_robot_HP != 0){
            G_aim_assist.m_enemy_2_res_flag = true;
        }
        if(G_referee.m_enemy_3_HP_pre == 0 && G_referee.GameRobotHP.red_3_robot_HP != 0){
            G_aim_assist.m_enemy_3_res_flag = true;
        }
        if(G_referee.m_enemy_4_HP_pre == 0 && G_referee.GameRobotHP.red_4_robot_HP != 0){
            G_aim_assist.m_enemy_4_res_flag = true;
        }
        if(G_referee.m_enemy_5_HP_pre == 0 && G_referee.GameRobotHP.red_5_robot_HP != 0){
            G_aim_assist.m_enemy_5_res_flag = true;
        }
        if(G_referee.GameRobotHP.red_outpost_HP == 0){
            G_aim_assist.m_enemy_7_res_flag = false;
        }else{
            G_aim_assist.m_enemy_7_res_flag = true;
        }
        if(G_aim_assist.m_enemy_1_res_flag == true && G_aim_assist.m_enemy_1_res_cnt <= 10000){
            G_aim_assist.m_enemy_1_res_cnt++;
        }else{
            G_aim_assist.m_enemy_1_res_flag = false;
            G_aim_assist.m_enemy_1_res_cnt = 0;
        }
        if(G_aim_assist.m_enemy_2_res_flag == true && G_aim_assist.m_enemy_2_res_cnt <= 10000){
            G_aim_assist.m_enemy_2_res_cnt++;
        }else{
            G_aim_assist.m_enemy_2_res_flag = false;
            G_aim_assist.m_enemy_2_res_cnt = 0;
        }
        if(G_aim_assist.m_enemy_3_res_flag == true && G_aim_assist.m_enemy_3_res_cnt <= 10000){
            G_aim_assist.m_enemy_3_res_cnt++;
        }else{
            G_aim_assist.m_enemy_3_res_flag = false;
            G_aim_assist.m_enemy_3_res_cnt = 0;
        }
        if(G_aim_assist.m_enemy_4_res_flag == true && G_aim_assist.m_enemy_4_res_cnt <= 10000){
            G_aim_assist.m_enemy_4_res_cnt++;
        }else{
            G_aim_assist.m_enemy_4_res_flag = false;
            G_aim_assist.m_enemy_4_res_cnt = 0;
        }
        if(G_aim_assist.m_enemy_5_res_flag == true && G_aim_assist.m_enemy_5_res_cnt <= 10000){
            G_aim_assist.m_enemy_5_res_cnt++;
        }else{
            G_aim_assist.m_enemy_5_res_flag = false;
            G_aim_assist.m_enemy_5_res_cnt = 0;
        }
        G_referee.m_enemy_1_HP_pre = G_referee.GameRobotHP.red_1_robot_HP;
        G_referee.m_enemy_2_HP_pre = G_referee.GameRobotHP.red_2_robot_HP;
        G_referee.m_enemy_3_HP_pre = G_referee.GameRobotHP.red_3_robot_HP;
        G_referee.m_enemy_4_HP_pre = G_referee.GameRobotHP.red_4_robot_HP;
        G_referee.m_enemy_5_HP_pre = G_referee.GameRobotHP.red_5_robot_HP;
        if(G_referee.GameStatus.game_progress == 4 && !G_aim_assist.balance_infantry_flag)
        {
            G_aim_assist.balance_infantry_flag = true;
            if(G_referee.GameRobotHP.red_3_robot_HP == 300)   G_aim_assist.balance_infantry_num += 3;
            if(G_referee.GameRobotHP.red_4_robot_HP == 300)   G_aim_assist.balance_infantry_num += 4;
            if(G_referee.GameRobotHP.red_5_robot_HP == 300)   G_aim_assist.balance_infantry_num += 5;
        }
        if(G_referee.GameStatus.game_progress != 4)
        {
            G_aim_assist.balance_infantry_flag = false;
            G_aim_assist.balance_infantry_num = 0;
        }
    }
    
    G_aim_assist.m_data_send_frame.m_data[0] = 
    G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_current;
    G_aim_assist.m_data_send_frame.m_data[1] = 
    G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_current;
    G_aim_assist.m_data_send_frame.m_data[2] = G_sentry.m_shoot_speed_filter->m_avg;
    G_aim_assist.m_data_send_frame.m_data[3] = G_aim_assist.balance_infantry_num;

    if (G_navigation.m_data_receive_frame.m_data_c[0] == 9 || 
    G_system_monitor.UART5_rx_fps < 5) {
        G_aim_assist.m_data_send_frame.m_data[4] = 0;
        G_aim_assist.m_data_send_frame.m_data[5] = 0;
        G_aim_assist.m_data_send_frame.m_data[6] = 0;
        G_aim_assist.m_data_send_frame.m_data[7] = -100;
        G_aim_assist.m_data_send_frame.m_data[8] = 0;

    } else {
        G_aim_assist.m_data_send_frame.m_data[4] = G_navigation.m_data_receive_frame.m_data_f[3];
        G_aim_assist.m_data_send_frame.m_data[5] = G_navigation.m_data_receive_frame.m_data_f[4];
        G_aim_assist.m_data_send_frame.m_data[6] = G_navigation.m_data_receive_frame.m_data_f[5];
        G_aim_assist.m_data_send_frame.m_data[7] = G_navigation.m_data_receive_frame.m_data_f[6];
        G_aim_assist.m_data_send_frame.m_data[8] = G_navigation.m_data_receive_frame.m_data_c[0];
    }
    G_aim_assist.m_data_send_frame.m_data[9] =  G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_angle_current;
    G_aim_assist.m_data_send_frame.m_data[10] = (G_gimbal.m_data_receive_frame.m_fdata[6] > 0)?0:(G_gimbal.m_data_receive_frame.m_fdata[6] < 0)?1:0;

    Append_CRC16_Check_Sum((uint8_t*)&G_aim_assist.m_data_send_frame,AIM_ASSIST_DATA_SEND_SIZE);
    G_aim_assist.SendData();
}

/**
 *@brief send gimabl datas
 *
 *@param
 */
void SendGimbalData(void)
{
    float pitch_current = 0;
    if (G_sentry.gimbal_mode != SentryRobot::GIMBAL_SAFE) {
        pitch_current = -G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_speed_pid->m_output + 
        G_COMPENSATION * cos((G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_current) / RADIAN2DEGREE_VALUE);
        if (pitch_current > G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_speed_pid->m_output_max)
        {
            pitch_current = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_speed_pid->m_output_max;
        }
        else if (pitch_current < -G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_speed_pid->m_output_max)
        {
            pitch_current = -G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_speed_pid->m_output_max;
        }
    } else {
        pitch_current = 0;
        G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_speed_pid->m_output = 0;
    }
    G_gimbal.m_data_send_frame.m_sdata[0] = pitch_current;
    G_gimbal.m_data_send_frame.m_sdata[1] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_speed_pid->m_output;
    G_gimbal.m_data_send_frame.m_sdata[2] = FRIC_WHEEL_SPEED;
    G_gimbal.m_data_send_frame.m_cdata[0] = (uint8_t)G_sentry.shoot_insurance;
    if(G_sentry.m_flag_cnt > 200)
    // G_gimbal.m_data_send_frame.m_cdata[1] = (uint8_t)G_sentry.change_gun_flag;
    G_gimbal.m_data_send_frame.m_cdata[1] = (uint8_t)0;
    else 
    G_gimbal.m_data_send_frame.m_cdata[1] = 0;
    G_gimbal.m_data_send_frame.m_cdata[2] = 1;
    G_gimbal.m_data_send_frame.m_cdata[3] = 0;
    G_gimbal.m_data_send_frame.m_cdata[4] = 0;
    G_gimbal.m_data_send_frame.m_tail[0] = 0x0A;
    G_gimbal.m_data_send_frame.m_tail[1] = 0x0D;

    Append_CRC16_Check_Sum((uint8_t*)&G_gimbal.m_data_send_frame,GIMBAL_DATA_SEND_SIZE);
    G_gimbal.SendData();
}



/**
 *@brief update system control mode
 *
 *@param
 */
void ControlModeUpdate(void)
{
    // Control mode update
    if(G_system_monitor.UART1_rx_fps < 5)
    {
        if(G_referee.GameStatus.game_progress == 4)
        { 
            G_control_mode = AUTO_CGS;
            if(G_sentry.vision_flag) G_control_mode = AUTO_GS_RC_C;
            else    G_control_mode = AUTO_CGS;
            if(G_sentry.safe_flag) G_control_mode = SAFE;
            else    G_control_mode = AUTO_CGS;

            G_sentry.shoot_insurance = false;
            
            if(G_sentry.m_shoot_cnt < 2500)
                G_sentry.m_scan_flag = false;
            else G_sentry.m_scan_flag = true;

            if(G_sentry.m_shoot_cnt < 3000)
                G_sentry.m_shoot_cnt ++;
            else
                G_sentry.m_shoot_cnt = 3000;
        }
        else{
            G_sentry.m_shoot_cnt = 0;
            G_control_mode = SAFE;
            G_sentry.m_scan_flag = true;
        }
    }
    else
    {
        if (G_djirc.channel.SW_L == RC_SW_DOWN && G_djirc.channel.SW_R == RC_SW_DOWN)
        {
            G_control_mode = SAFE;
        }
        else if (G_djirc.channel.SW_L == RC_SW_DOWN && G_djirc.channel.SW_R == RC_SW_MID)
        {
            G_control_mode = RC_GS;
        }
        else if (G_djirc.channel.SW_L == RC_SW_DOWN && G_djirc.channel.SW_R == RC_SW_UP)
        {
            G_control_mode = AUTO_G_RC_S;
        }
        else if (G_djirc.channel.SW_L == RC_SW_MID && G_djirc.channel.SW_R == RC_SW_UP)
        {
            G_control_mode = AUTO_GS_RC_C;
        }
        else if (G_djirc.channel.SW_L == RC_SW_UP && G_djirc.channel.SW_R == RC_SW_UP)
        {
            G_control_mode = AUTO_CGS;
        }
        else if (G_djirc.channel.SW_L == RC_SW_MID && G_djirc.channel.SW_R == RC_SW_MID)
        {
            G_control_mode = RC_C;
        }
        else if (G_djirc.channel.SW_L == RC_SW_MID && G_djirc.channel.SW_R == RC_SW_DOWN)
        {
            G_control_mode = SAFE;
        }
        else if (G_djirc.channel.SW_L == RC_SW_UP && G_djirc.channel.SW_R == RC_SW_DOWN)
        {
            G_control_mode = AUTO_G_RC_C;
        }
        else
        {
            G_control_mode = SAFE;
        }
    }

    // Set sentry robot gimbal mode
    if (G_control_mode == AUTO_G_RC_S || G_control_mode == AUTO_CGS || G_control_mode == AUTO_GS_RC_C || G_control_mode == AUTO_G_RC_C)
    {
        G_sentry.SetGimbalMode(SentryRobot::GIMBAL_AUTO);
    }
    else if (G_control_mode == RC_GS)
    {
        G_sentry.SetGimbalMode(SentryRobot::GIMBAL_MANNAL);
    }
    else
    {
        G_sentry.SetGimbalMode(SentryRobot::GIMBAL_SAFE);
    }

    // // Set sentry robot shoot mode
    // if ((G_control_mode == G_AUTO_GS_RC_C || G_control_mode == G_AUTO_CGS) &&
    // G_system_monitor.UART6_rx_fps > 100)
    
    // Set sentry robot shoot mode
    if ((G_control_mode == AUTO_GS_RC_C || G_control_mode == AUTO_CGS))
    {
        G_sentry.SetShootMode(SentryRobot::SHOOT_AUTO);
    }
    else if (G_control_mode == RC_GS || G_control_mode == AUTO_G_RC_S)
    {
        G_sentry.SetShootMode(SentryRobot::SHOOT_MANNAL);
    }
    else
    {
        G_sentry.SetShootMode(SentryRobot::SHOOT_SAFE);
    }

    // Set sentry robot chassis mode
    if (G_control_mode == AUTO_CGS)
    {
        G_sentry.SetChassisMode(SentryRobot::CHASSIS_AUTO);
    }
    else if (G_control_mode == AUTO_GS_RC_C || G_control_mode == RC_C || G_control_mode == AUTO_G_RC_C)
    {
        G_sentry.SetChassisMode(SentryRobot::CHASSIS_MANNAL);
    }
    else
    {
        G_sentry.SetChassisMode(SentryRobot::CHASSIS_SAFE);
    }
}



/**
 *@brief update the state of the sentry robot
 *
 *@param
 */
void RobotStatesUpdate(void)
{
    // Update sentry robot current gimbal states
    float second_gimbal_yaw_angle = G_gimbal.m_data_receive_frame.m_fdata[0];
    float second_gimbal_yaw_speed = G_gimbal.m_data_receive_frame.m_fdata[1];
    float first_gimbal_pitch_angle = G_gimbal.m_data_receive_frame.m_fdata[2];
    float first_gimbal_pitch_speed = G_gimbal.m_data_receive_frame.m_fdata[3];
    G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->AngleUpdate(2, second_gimbal_yaw_angle);
    G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->SpeedUpdate(2, second_gimbal_yaw_speed);
    G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->AngleUpdate(2, first_gimbal_pitch_angle);
    G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->SpeedUpdate(2, first_gimbal_pitch_speed);
#ifdef FIRST_GIMBAL_YAW_IMU_FEED_BACK
    float first_gimbal_yaw_angle = G_gimbal.m_data_receive_frame.m_fdata[4];
    float first_gimbal_yaw_speed = G_gimbal.m_data_receive_frame.m_fdata[5];
    static float first_gimbal_yaw_angle_pre = first_gimbal_yaw_angle;
    static float first_gimbal_yaw_angle_sum = first_gimbal_yaw_angle;
    float first_gimbal_yaw_angle_step = first_gimbal_yaw_angle - first_gimbal_yaw_angle_pre;
    if(first_gimbal_yaw_angle_step > 360.0f) 
    {
        first_gimbal_yaw_angle_step = 0;
        first_gimbal_yaw_angle = first_gimbal_yaw_angle_pre;
    }
    if (first_gimbal_yaw_angle_step > 90) {
        first_gimbal_yaw_angle_step -= 360;
    } else if (first_gimbal_yaw_angle_step < -90) {
        first_gimbal_yaw_angle_step += 360;
    }
    first_gimbal_yaw_angle_sum += first_gimbal_yaw_angle_step;
    first_gimbal_yaw_angle_pre = first_gimbal_yaw_angle;

    G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->AngleUpdate(2, first_gimbal_yaw_angle_sum);
    G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->SpeedUpdate(2, first_gimbal_yaw_speed);
#elif defined FIRST_GIMBAL_YAW_ENCODER_FEED_BACK
    G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->AngleUpdate(0, 1);
    G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->SpeedUpdate(1, 1);
#endif

    // update sentry robot current chassis states
    G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FLL_MOTOR]->SpeedUpdate(1, 1);
    G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FRL_MOTOR]->SpeedUpdate(1, 1);
    G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BLL_MOTOR]->SpeedUpdate(1, 1);
    G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BRL_MOTOR]->SpeedUpdate(1, 1);
    G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FLA_MOTOR]->SpeedUpdate(1, -1);
    G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FRA_MOTOR]->SpeedUpdate(1, -1);
    G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BLA_MOTOR]->SpeedUpdate(1, -1);
    G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BRA_MOTOR]->SpeedUpdate(1, -1);
    G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FLA_MOTOR]->AngleUpdate(0, -1);
    G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FRA_MOTOR]->AngleUpdate(0, -1);
    G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BLA_MOTOR]->AngleUpdate(0, -1);
    G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BRA_MOTOR]->AngleUpdate(0, -1);


    // Update sentry robot current shoot states
    G_sentry.shoot_motor[SentryRobot::SHOOT_DRIVE_MOTOR]->SpeedUpdate(0, 1);
    G_sentry.shoot_motor[SentryRobot::SHOOT_DRIVE_MOTOR]->AngleUpdate(0, 1);

    // calc transform angle
    G_sentry.m_robot_chassis_world_error = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->
    m_angle_current_encoder_filter + G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->
    m_angle_current + RIDAR_FIXED_ANGLE_ERROR - G_sentry.m_robot_radar_world_yaw;

    // Update sentry robot speed
#ifdef CHASSIS_STEER_DRIVING_MODE
    float line_speed = 0;
    line_speed += G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FLL_MOTOR]->m_speed_current_encoder_filter;
    line_speed += G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BLL_MOTOR]->m_speed_current_encoder_filter;
    line_speed -= G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FRL_MOTOR]->m_speed_current_encoder_filter;
    line_speed -= G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BRL_MOTOR]->m_speed_current_encoder_filter;
    line_speed = line_speed / 4 / RADIAN2DEGREE_VALUE * CHASSIS_WHEEL_RADIUS;
    G_sentry.m_robot_chassis_speed = line_speed;
    // Update sentry robot wheel chassis yaw
    float wheel_chassis_angle = 0;
    for (int i = 0; i < CHASSIS_STEER_MOTOR_NUM; i++)
    {
        wheel_chassis_angle += G_sentry.chassis_steer_motor[i]->m_angle_current;
    }
    wheel_chassis_angle = wheel_chassis_angle / 4;
    G_sentry.m_robot_wheel_world_yaw = (wheel_chassis_angle - G_sentry.m_robot_chassis_world_error);
#elif defined CHASSIS_STANDARD_DRIVING_MODE
    float chassis_line_speed_x = 0;
    float chassis_line_speed_y = 0;
    chassis_line_speed_x += G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FLL_MOTOR]->
                            m_speed_current_encoder_filter / 
                            RADIAN2DEGREE_VALUE * 
                            CHASSIS_WHEEL_RADIUS * 
                            cos(G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FLA_MOTOR]->m_angle_current / RADIAN2DEGREE_VALUE);
    chassis_line_speed_x += -G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FRL_MOTOR]->
                            m_speed_current_encoder_filter / 
                            RADIAN2DEGREE_VALUE * 
                            CHASSIS_WHEEL_RADIUS * 
                            cos(G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FRA_MOTOR]->m_angle_current / RADIAN2DEGREE_VALUE);
    chassis_line_speed_x += -G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BRL_MOTOR]->
                            m_speed_current_encoder_filter / 
                            RADIAN2DEGREE_VALUE * 
                            CHASSIS_WHEEL_RADIUS * 
                            cos(G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BRA_MOTOR]->m_angle_current / RADIAN2DEGREE_VALUE);
    chassis_line_speed_x += G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BLL_MOTOR]->
                            m_speed_current_encoder_filter / 
                            RADIAN2DEGREE_VALUE * 
                            CHASSIS_WHEEL_RADIUS * 
                            cos(G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BLA_MOTOR]->m_angle_current / RADIAN2DEGREE_VALUE);
    chassis_line_speed_y += G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FLL_MOTOR]->
                            m_speed_current_encoder_filter / 
                            RADIAN2DEGREE_VALUE * 
                            CHASSIS_WHEEL_RADIUS * 
                            sin(G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FLA_MOTOR]->m_angle_current / RADIAN2DEGREE_VALUE);
    chassis_line_speed_y += -G_sentry.chassis_line_motor[SentryRobot::CHASSIS_FRL_MOTOR]->
                            m_speed_current_encoder_filter / 
                            RADIAN2DEGREE_VALUE * 
                            CHASSIS_WHEEL_RADIUS * 
                            sin(G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FRA_MOTOR]->m_angle_current / RADIAN2DEGREE_VALUE);
    chassis_line_speed_y += -G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BRL_MOTOR]->
                            m_speed_current_encoder_filter / 
                            RADIAN2DEGREE_VALUE * 
                            CHASSIS_WHEEL_RADIUS * 
                            sin(G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BRA_MOTOR]->m_angle_current / RADIAN2DEGREE_VALUE);
    chassis_line_speed_y += G_sentry.chassis_line_motor[SentryRobot::CHASSIS_BLL_MOTOR]->
                            m_speed_current_encoder_filter / 
                            RADIAN2DEGREE_VALUE * 
                            CHASSIS_WHEEL_RADIUS * 
                            sin(G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BLA_MOTOR]->m_angle_current / RADIAN2DEGREE_VALUE);
    chassis_line_speed_x /= 4.0;
    chassis_line_speed_y /= 4.0;
    float wheel_chassis_angle = safeAtan2(chassis_line_speed_y,chassis_line_speed_x) * RADIAN2DEGREE_VALUE;
    static float wheel_chassis_angle_pre = wheel_chassis_angle;
    static float wheel_chassis_angle_des = wheel_chassis_angle;
    float wheel_chassis_angle_step = wheel_chassis_angle - wheel_chassis_angle_pre;
    if(wheel_chassis_angle_step > 180.0){
        wheel_chassis_angle_step -= 360.0;
    }else if(wheel_chassis_angle_step < -180.0){
        wheel_chassis_angle_step += 360.0;
    }
    wheel_chassis_angle_des += wheel_chassis_angle_step;
    wheel_chassis_angle_pre = wheel_chassis_angle;
    G_sentry.m_robot_wheel_world_yaw = (wheel_chassis_angle_des - G_sentry.m_robot_chassis_world_error);
    G_sentry.m_robot_chassis_speed = sqrt(chassis_line_speed_x*chassis_line_speed_x+chassis_line_speed_y*chassis_line_speed_y);
#endif
    
    
    static float radar_world_yaw_pre = 0;
    if (G_system_monitor.UART5_rx_fps > 80) {
#ifdef CHASSIS_STEER_DRIVING_MODE
        // update sentry robot world yaw des
        G_sentry.m_robot_wheel_world_yaw_des_sum = G_navigation.m_data_receive_frame.m_data_f[0] * RADIAN2DEGREE_VALUE;
        // update sentry robot world speed des
        G_sentry.m_robot_wheel_world_speed_des = G_navigation.m_data_receive_frame.m_data_f[2];
        // speed max limit
        if (G_sentry.m_robot_wheel_world_speed_des > CHASSIS_LINE_SPEED_MAX)
        {
            G_sentry.m_robot_wheel_world_speed_des = CHASSIS_LINE_SPEED_MAX;
        }
        if (G_sentry.m_robot_wheel_world_speed_des < -CHASSIS_LINE_SPEED_MAX)
        {
            G_sentry.m_robot_wheel_world_speed_des = -CHASSIS_LINE_SPEED_MAX;
        }
#endif
        
        // Update sentry robot radar world_yaw
        float radar_world_yaw = G_navigation.m_data_receive_frame.m_data_f[1] * RADIAN2DEGREE_VALUE;
        
        float radar_world_yaw_step = radar_world_yaw - radar_world_yaw_pre;
        if (radar_world_yaw_step > 180) {
            radar_world_yaw_step -= 360;
        } else if (radar_world_yaw_step < -180) {
            radar_world_yaw_step += 360;
        }
        radar_world_yaw_pre = radar_world_yaw;
        G_sentry.m_robot_radar_world_yaw += radar_world_yaw_step;

        if(G_navigation.m_data_receive_frame.m_data_c[2] == 1 && G_sentry.ctrl_spinning_flag == true){
            G_sentry.spinning_flag = true;
        }else if(G_navigation.m_data_receive_frame.m_data_c[2] == 0){
            G_sentry.spinning_flag = false;
            G_sentry.ctrl_spinning_flag = true;
        }
    }else{
        G_navigation.m_data_receive_frame.m_data_c[1] = 1; // attack robot number 2
    }


    // Update the capacitor power charging value
    if (G_system_monitor.UART2_rx_fps > 50)
    {
        // cap update
        float power_now = G_referee.PowerHeatData.chassis_power;
        float power_limit = G_referee.GameRobotStatus.chassis_power_limit;
        float power_buffer_now = G_referee.PowerHeatData.chassis_power_buffer;

        G_sentry.capacitor->UpdateChargingPower(power_now, power_buffer_now, power_limit);
        G_sentry.capacitor->PowerJudgeByVol();
        G_sentry.capacitor->PowerDesUpdate(power_limit);

        // key command from referee
        if(G_referee.RobotCommand.commd_keyboard == PRESS_F){
             G_sentry.spinning_flag = true;
             G_sentry.ctrl_spinning_flag = true;
        }
        if(G_referee.RobotCommand.commd_keyboard == PRESS_G){
            G_sentry.spinning_flag = false;
            G_sentry.ctrl_spinning_flag = false;
        }
        if(G_referee.RobotCommand.commd_keyboard == PRESS_Z)  G_sentry.vision_flag = true;
        if(G_referee.RobotCommand.commd_keyboard == PRESS_X)  G_sentry.vision_flag = false;
        if(G_referee.RobotCommand.commd_keyboard == PRESS_C)  G_sentry.safe_flag = true;
        if(G_referee.RobotCommand.commd_keyboard == PRESS_V)  G_sentry.safe_flag = false;

        // gun shift strategy
       if((G_referee.PowerHeatData.shooter_id1_17mm_cooling_heat > G_referee.GameRobotStatus.shooter_id1_17mm_cooling_limit - 40) || (G_referee.PowerHeatData.shooter_id2_17mm_cooling_heat > G_referee.GameRobotStatus.shooter_id2_17mm_cooling_limit - 40)
       && G_referee.GameRobotStatus.shooter_id1_17mm_cooling_limit != 0 && G_referee.GameRobotStatus.shooter_id2_17mm_cooling_limit != 0)
       {
            G_sentry.change_gun_flag = true;
       }
       if(G_sentry.change_gun_flag == true)  G_sentry.m_flag_cnt ++;
       if(G_sentry.m_flag_cnt > 2500) // 临时不切枪，200热量冷却完要2.5秒，2500
       {
            G_sentry.change_gun_flag = false;   
            G_sentry.m_flag_cnt = 0;
       }
       
       // shoot speed filter update
       if(G_referee.ShootData.bullet_speed != G_sentry.m_shoot_speed_pre && G_referee.ShootData.bullet_speed != 0){
            G_sentry.m_shoot_speed_filter->Update(G_referee.ShootData.bullet_speed);
       }
       G_sentry.m_shoot_speed_pre = G_referee.ShootData.bullet_speed;
    }

    // aim assist update
    G_sentry.enemy_find_flag = false;
    // Update aim assist results 
    if (G_system_monitor.UART6_rx_fps > 50) {
        // enemy detect state
        if (G_aim_assist.m_data_receive_frame.m_id == 0) {
            G_sentry.enemy_find_flag = false;
        } else {
            G_sentry.enemy_find_flag = true;
        }
    }
}



/**
 *@brief update the robot targets
 *
 *@param
 */
void RobotTargetsUpdate(void)
{
    ChassisTargetsUpdate();
    GimbalTargetsUpdate();
    ShootTargetsUpdate();
}



/**
 *@brief update the robot chassis targets
 *
 *@param
 */
void ChassisTargetsUpdate(void)
{
#ifdef CHASSIS_STEER_DRIVING_MODE
    SteerDriveChassisTargetsUpdate();

#elif defined CHASSIS_STANDARD_DRIVING_MODE
    StandardDriveChassisTargetsUpdate();

#elif defined CHASSIS_DIFFERENTIAL_DRIVING_MODE
    DiffDriveChassisTargetsUpdate();

#elif defined CHASSIS_COMMOM_DRIVING_MODE
    CommonDriveChassisTargetsUpdate();

#endif
}



/**
 *@brief update the robot steer drive chassis targets
 *
 *@param
 */
void SteerDriveChassisTargetsUpdate(void)
{
    #ifdef CHASSIS_STEER_DRIVING_MODE
    float wheel_speed_des = 0;
    float wheel_angle_des = 0;

    if (G_sentry.chassis_mode == SentryRobot::CHASSIS_MANNAL)
    {
        if(G_sentry.chassis_mode_pre != SentryRobot::CHASSIS_MANNAL){
            G_sentry.m_robot_chassis_spin_ready = false;
        }
        if (fabs((float)G_djirc.channel.Ch3 - RC_CH_VALUE_OFFSET) > 8 * RC_CH_VALUE_DEAD && fabs((float)G_djirc.channel.Ch0 - RC_CH_VALUE_OFFSET) > RC_CH_VALUE_DEAD)
        {
            G_sentry.SetChassisSpinAngleTarget();
            if(fabs(G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FLA_MOTOR]->m_angle_target - G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FLA_MOTOR]->m_angle_current)<=5 &&
                fabs(G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FRA_MOTOR]->m_angle_target - G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FRA_MOTOR]->m_angle_current)<=5 &&
                fabs(G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BRA_MOTOR]->m_angle_target - G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BRA_MOTOR]->m_angle_current)<=5 &&
                fabs(G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BLA_MOTOR]->m_angle_target - G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BLA_MOTOR]->m_angle_current)<=5){
                wheel_speed_des = ((float)(G_djirc.channel.Ch0 - RC_CH_VALUE_OFFSET) / RC_CH_VALUE_RANGE * CHASSIS_SPIN_SPEED_MAX / CHASSIS_WHEEL_RADIUS * RADIAN2DEGREE_VALUE);
            }
            G_sentry.SetChassisSpeedTarget(wheel_speed_des, wheel_speed_des, wheel_speed_des, wheel_speed_des);
        }
        else
        {
            float chassis_steer_des = 0;
            float steer_angle_step = 0;
            if ((float)G_djirc.channel.Ch1 - RC_CH_VALUE_OFFSET > RC_CH_VALUE_DEAD)
            {
                wheel_speed_des = ((float)(G_djirc.channel.Ch1 - RC_CH_VALUE_OFFSET - RC_CH_VALUE_DEAD) / RC_CH_VALUE_RANGE * CHASSIS_LINE_SPEED_MAX / CHASSIS_WHEEL_RADIUS * RADIAN2DEGREE_VALUE);
            }
            else if ((float)G_djirc.channel.Ch1 - RC_CH_VALUE_OFFSET < -RC_CH_VALUE_DEAD)
            {
                wheel_speed_des = ((float)(G_djirc.channel.Ch1 - RC_CH_VALUE_OFFSET + RC_CH_VALUE_DEAD) / RC_CH_VALUE_RANGE * CHASSIS_LINE_SPEED_MAX / CHASSIS_WHEEL_RADIUS * RADIAN2DEGREE_VALUE);
            }

            if ((float)G_djirc.channel.Ch2 - RC_CH_VALUE_OFFSET > RC_CH_VALUE_DEAD)
            {
                steer_angle_step = -((float)(G_djirc.channel.Ch2 - RC_CH_VALUE_OFFSET - RC_CH_VALUE_DEAD) / RC_CH_VALUE_RANGE * CHASSIS_STEER_ANGLE_SENSITIVITY);
            }
            else if ((float)G_djirc.channel.Ch2 - RC_CH_VALUE_OFFSET < -RC_CH_VALUE_DEAD)
            {
                steer_angle_step = -((float)(G_djirc.channel.Ch2 - RC_CH_VALUE_OFFSET + RC_CH_VALUE_DEAD) / RC_CH_VALUE_RANGE * CHASSIS_STEER_ANGLE_SENSITIVITY);
            }
            chassis_steer_des = G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FLA_MOTOR]->m_angle_target + steer_angle_step;
            if((fabs(G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FLL_MOTOR]->m_speed_current)>= CHASSIS_SPIN_OVER_LIMIT / CHASSIS_WHEEL_RADIUS * RADIAN2DEGREE_VALUE ||
                fabs(G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FRL_MOTOR]->m_speed_current)>= CHASSIS_SPIN_OVER_LIMIT / CHASSIS_WHEEL_RADIUS * RADIAN2DEGREE_VALUE ||
                fabs(G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BRL_MOTOR]->m_speed_current)>= CHASSIS_SPIN_OVER_LIMIT / CHASSIS_WHEEL_RADIUS * RADIAN2DEGREE_VALUE ||
                fabs(G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BLL_MOTOR]->m_speed_current)>= CHASSIS_SPIN_OVER_LIMIT / CHASSIS_WHEEL_RADIUS * RADIAN2DEGREE_VALUE) && 
                G_sentry.m_robot_chassis_spin_ready){
                    wheel_speed_des = 0.0;
            }else{
                G_sentry.m_robot_chassis_spin_ready = false;
                G_sentry.SetChassisAngleTarget(chassis_steer_des, chassis_steer_des, chassis_steer_des, chassis_steer_des);
            }
            G_sentry.SetChassisSpeedTarget(wheel_speed_des, wheel_speed_des, -wheel_speed_des, -wheel_speed_des);
        }
    }
    else if (G_sentry.chassis_mode == SentryRobot::CHASSIS_AUTO)
    {
        if(G_sentry.chassis_mode_pre != SentryRobot::CHASSIS_AUTO){
            G_sentry.m_robot_chassis_spin_ready = false;
        }
        if(G_sentry.spinning_flag)
        {
            G_sentry.SetChassisSpinAngleTarget();
            if(fabs(G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FLA_MOTOR]->m_angle_target - G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FLA_MOTOR]->m_angle_current)<=5 &&
                fabs(G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FRA_MOTOR]->m_angle_target - G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FRA_MOTOR]->m_angle_current)<=5 &&
                fabs(G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BRA_MOTOR]->m_angle_target - G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BRA_MOTOR]->m_angle_current)<=5 &&
                fabs(G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BLA_MOTOR]->m_angle_target - G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BLA_MOTOR]->m_angle_current)<=5){
                    if(G_sentry.enemy_find_flag && G_referee.BulletRemaining.bullet_remaining_num_17mm > 20)
                    {
                        wheel_speed_des = CHASSIS_LINE_SHOOT_SPEED_MAX / CHASSIS_WHEEL_RADIUS * RADIAN2DEGREE_VALUE;
                        // G_sentry.sin_cnt++;
                        // if(G_sentry.sin_cnt > 12345)
                        //     G_sentry.sin_cnt = 0;
                    }
                    else{
                        wheel_speed_des = CHASSIS_SPIN_SPEED_MAX / CHASSIS_WHEEL_RADIUS * RADIAN2DEGREE_VALUE;
                        // wheel_speed_des = fabs(wheel_speed_sin_des*sinf(2*3.14/8765*G_sentry.sin_cnt)) + (1.5f + 0.5/12345*G_sentry.sin_cnt)/ CHASSIS_WHEEL_RADIUS * RADIAN2DEGREE_VALUE;
                        // G_sentry.sin_cnt++;
                        // if(G_sentry.sin_cnt > 12345)
                        //     G_sentry.sin_cnt = 0;
                    }
            }else{
                wheel_speed_des = 0.0;
            }  
            G_sentry.SetChassisSpeedTarget(wheel_speed_des, wheel_speed_des, wheel_speed_des, wheel_speed_des);
        }
        else
        {
            if(G_system_monitor.UART5_rx_fps > 70)
            {
                // transform current yaw angle des sum
                wheel_angle_des = G_sentry.m_robot_wheel_world_yaw_des_sum + G_sentry.m_robot_chassis_world_error;
                // get speed des
                wheel_speed_des = G_sentry.m_robot_wheel_world_speed_des / CHASSIS_WHEEL_RADIUS * RADIAN2DEGREE_VALUE;
            }
            if((fabs(G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FLL_MOTOR]->m_speed_current)>= CHASSIS_SPIN_OVER_LIMIT / CHASSIS_WHEEL_RADIUS * RADIAN2DEGREE_VALUE ||
                fabs(G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_FRL_MOTOR]->m_speed_current)>= CHASSIS_SPIN_OVER_LIMIT / CHASSIS_WHEEL_RADIUS * RADIAN2DEGREE_VALUE ||
                fabs(G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BRL_MOTOR]->m_speed_current)>= CHASSIS_SPIN_OVER_LIMIT / CHASSIS_WHEEL_RADIUS * RADIAN2DEGREE_VALUE ||
                fabs(G_sentry.chassis_steer_motor[SentryRobot::CHASSIS_BLL_MOTOR]->m_speed_current)>= CHASSIS_SPIN_OVER_LIMIT / CHASSIS_WHEEL_RADIUS * RADIAN2DEGREE_VALUE) && 
                G_sentry.m_robot_chassis_spin_ready){
                wheel_speed_des = 0.0;
            }
            else
            {
                G_sentry.m_robot_chassis_spin_ready = false;
                if(G_system_monitor.UART5_rx_fps > 70)
                {
                    G_sentry.SetChassisAngleTarget(wheel_angle_des, wheel_angle_des, wheel_angle_des, wheel_angle_des);
                }
            }
            G_sentry.SetChassisSpeedTarget(wheel_speed_des, wheel_speed_des, -wheel_speed_des, -wheel_speed_des);
            // G_sentry.SetChassisSpeedTarget(0,0,0,0);
        }
    }
    else
    {
        G_sentry.m_robot_chassis_spin_ready = false;
        for (int i = 0; i < CHASSIS_STEER_MOTOR_NUM; i++)
        {
            G_sentry.chassis_steer_motor[i]->m_angle_target = G_sentry.chassis_steer_motor[i]->m_angle_current;
            G_sentry.chassis_steer_motor[i]->m_speed_target = G_sentry.chassis_steer_motor[i]->m_speed_current;
        }
        for (int i = 0; i < CHASSIS_LINE_MOTOR_NUM; i++)
        {
            G_sentry.chassis_line_motor[i]->m_angle_target = G_sentry.chassis_line_motor[i]->m_angle_current;
            G_sentry.chassis_line_motor[i]->m_speed_target = G_sentry.chassis_line_motor[i]->m_speed_current;
        }
        
    }
    #endif
}


/**
 *@brief update the robot steer drive chassis targets
 *
 *@param
 */
void StandardDriveChassisTargetsUpdate(void)
{
    #ifdef CHASSIS_STANDARD_DRIVING_MODE

    float world_line_speed_x_rate = 0.0;
    float world_line_speed_y_rate = 0.0;
    float chassis_spin_line_speed = 0.0;
    float world_line_speed_angle = 0.0;
    float world_line_speed_navigation = 0.0;
    
    if (G_sentry.chassis_mode == SentryRobot::CHASSIS_SAFE){
        for (int i = 0; i < CHASSIS_STEER_MOTOR_NUM; i++)
            {
                G_sentry.chassis_steer_motor[i]->m_angle_target = G_sentry.chassis_steer_motor[i]->m_angle_current;
                G_sentry.chassis_steer_motor[i]->m_speed_target = G_sentry.chassis_steer_motor[i]->m_speed_current;
            }
            for (int i = 0; i < CHASSIS_LINE_MOTOR_NUM; i++)
            {
                G_sentry.chassis_line_motor[i]->m_angle_target = G_sentry.chassis_line_motor[i]->m_angle_current;
                G_sentry.chassis_line_motor[i]->m_speed_target = G_sentry.chassis_line_motor[i]->m_speed_current;
            }
        return;
    }

    if (G_sentry.chassis_mode == SentryRobot::CHASSIS_MANNAL){
        if ((float)G_djirc.channel.Ch1 - RC_CH_VALUE_OFFSET > RC_CH_VALUE_DEAD)
        {
            world_line_speed_x_rate = ((float)(G_djirc.channel.Ch1 - RC_CH_VALUE_OFFSET - RC_CH_VALUE_DEAD) / RC_CH_VALUE_RANGE);
        }
        else if ((float)G_djirc.channel.Ch1 - RC_CH_VALUE_OFFSET < -RC_CH_VALUE_DEAD)
        {
            world_line_speed_x_rate = ((float)(G_djirc.channel.Ch1 - RC_CH_VALUE_OFFSET + RC_CH_VALUE_DEAD) / RC_CH_VALUE_RANGE);
        }
        if ((float)G_djirc.channel.Ch0 - RC_CH_VALUE_OFFSET > RC_CH_VALUE_DEAD)
        {
            world_line_speed_y_rate = -((float)(G_djirc.channel.Ch0 - RC_CH_VALUE_OFFSET - RC_CH_VALUE_DEAD) / RC_CH_VALUE_RANGE);
        }
        else if ((float)G_djirc.channel.Ch0 - RC_CH_VALUE_OFFSET < -RC_CH_VALUE_DEAD)
        {
            world_line_speed_y_rate = -((float)(G_djirc.channel.Ch0 - RC_CH_VALUE_OFFSET + RC_CH_VALUE_DEAD) / RC_CH_VALUE_RANGE);
        }
        if ((float)G_djirc.channel.Ch2 - RC_CH_VALUE_OFFSET > RC_CH_VALUE_DEAD)
        {
            chassis_spin_line_speed = ((float)(G_djirc.channel.Ch2 - RC_CH_VALUE_OFFSET - RC_CH_VALUE_DEAD) / RC_CH_VALUE_RANGE) * CHASSIS_MANNAL_SPIN_SPEED_MAX;
        }
        else if ((float)G_djirc.channel.Ch2 - RC_CH_VALUE_OFFSET < -RC_CH_VALUE_DEAD)
        {
            chassis_spin_line_speed = ((float)(G_djirc.channel.Ch2 - RC_CH_VALUE_OFFSET + RC_CH_VALUE_DEAD) / RC_CH_VALUE_RANGE) * CHASSIS_MANNAL_SPIN_SPEED_MAX;
        }
        // world_line_speed_angle = safeAtan2(world_line_speed_y_rate, world_line_speed_x_rate) * RADIAN2DEGREE_VALUE;
    }else if (G_sentry.chassis_mode == SentryRobot::CHASSIS_AUTO){
        world_line_speed_x_rate = G_navigation.m_data_receive_frame.m_data_f[0];
        world_line_speed_y_rate = G_navigation.m_data_receive_frame.m_data_f[2];
        world_line_speed_navigation = sqrt(world_line_speed_x_rate * world_line_speed_x_rate + world_line_speed_y_rate * world_line_speed_y_rate);
        if(G_sentry.spinning_flag){
            chassis_spin_line_speed = CHASSIS_SPIN_SPEED_MAX;
        }else{
            chassis_spin_line_speed = 0.0;
        }
        
        // world_line_speed_angle = safeAtan2(world_line_speed_y_rate, world_line_speed_x_rate) * RADIAN2DEGREE_VALUE - 
        //                         G_sentry.m_robot_radar_world_yaw + 
        //                         RIDAR_FIXED_ANGLE_ERROR + 
        //                         G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_angle_current + 
        //                         G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_current_external;
    }
    world_line_speed_angle = safeAtan2(world_line_speed_y_rate, world_line_speed_x_rate) * RADIAN2DEGREE_VALUE;

    if(!G_sentry.standard_driving_init_flag){
        G_sentry.world_line_speed_angle_des = world_line_speed_angle;
        G_sentry.world_line_speed_angle_pre = world_line_speed_angle;
    }
    float world_line_speed_angle_step = world_line_speed_angle - G_sentry.world_line_speed_angle_pre;
    if (world_line_speed_angle_step > 180) {
        world_line_speed_angle_step -= 360;
    } else if (world_line_speed_angle_step < -180) {
        world_line_speed_angle_step += 360;
    }
    G_sentry.world_line_speed_angle_pre = world_line_speed_angle;
    G_sentry.world_line_speed_angle_des += world_line_speed_angle_step;

    float chassis_line_speed_angle = G_sentry.world_line_speed_angle_des + G_sentry.m_robot_chassis_world_error;
                                    // G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_current_encoder_filter - 
                                    // G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_current_external;

    float chassis_line_speed_x = 0.0;
    float chassis_line_speed_y = 0.0;

    if (G_sentry.chassis_mode == SentryRobot::CHASSIS_MANNAL){
        float line_speed_rate = sqrt(world_line_speed_x_rate * world_line_speed_x_rate + world_line_speed_y_rate * world_line_speed_y_rate);
        chassis_line_speed_x = line_speed_rate * CHASSIS_MANNAL_LINE_SPEED_MAX * cos(chassis_line_speed_angle / RADIAN2DEGREE_VALUE);
        chassis_line_speed_y = line_speed_rate * CHASSIS_MANNAL_LINE_SPEED_MAX * sin(chassis_line_speed_angle / RADIAN2DEGREE_VALUE);
    }else if(G_sentry.chassis_mode == SentryRobot::CHASSIS_AUTO){
        chassis_line_speed_x = world_line_speed_navigation * cos(chassis_line_speed_angle / RADIAN2DEGREE_VALUE);
        chassis_line_speed_y = world_line_speed_navigation * sin(chassis_line_speed_angle / RADIAN2DEGREE_VALUE);
    }

    float speed_x = 0.0;
    float speed_y = 0.0;
    float angle_raw = 0.0;
    float angle_step = 0.0;

    // fl
    speed_x = chassis_line_speed_x + chassis_spin_line_speed * cos(-45.0 / RADIAN2DEGREE_VALUE);
    speed_y = chassis_line_speed_y + chassis_spin_line_speed * sin(-45.0 / RADIAN2DEGREE_VALUE);
    float fll_line_speed_des = sqrt(speed_x * speed_x + speed_y * speed_y);
    angle_raw = safeAtan2(speed_y, speed_x) * RADIAN2DEGREE_VALUE;
    if(!G_sentry.standard_driving_init_flag){
        G_sentry.fla_angle_des = angle_raw;
        G_sentry.fla_angle_des_pre = angle_raw;
    }
    angle_step = angle_raw - G_sentry.fla_angle_des_pre;
    if (angle_step > 180) {
        angle_step -= 360;
    } else if (angle_step < -180) {
        angle_step += 360;
    }
    if(angle_step > 90){
        angle_step -= 180;
        G_sentry.fll_speed_dir = !G_sentry.fll_speed_dir;
    }else if(angle_step < -90){
        angle_step += 180;
        G_sentry.fll_speed_dir = !G_sentry.fll_speed_dir;
    }
    G_sentry.fla_angle_des_pre = angle_raw;
    G_sentry.fla_angle_des += angle_step;
    // fr
    speed_x = chassis_line_speed_x + -1.0 * chassis_spin_line_speed * cos(45.0 / RADIAN2DEGREE_VALUE);
    speed_y = chassis_line_speed_y + -1.0 * chassis_spin_line_speed * sin(45.0 / RADIAN2DEGREE_VALUE);
    float frl_line_speed_des = sqrt(speed_x * speed_x + speed_y * speed_y);
    angle_raw = safeAtan2(speed_y, speed_x) * RADIAN2DEGREE_VALUE;
    if(!G_sentry.standard_driving_init_flag){
        G_sentry.fra_angle_des = angle_raw;
        G_sentry.fra_angle_des_pre = angle_raw;
    }
    angle_step = angle_raw - G_sentry.fra_angle_des_pre;
    if (angle_step > 180) {
        angle_step -= 360;
    } else if (angle_step < -180) {
        angle_step += 360;
    }
    if(angle_step > 90){
        angle_step -= 180;
        G_sentry.frl_speed_dir = !G_sentry.frl_speed_dir;
    }else if(angle_step < -90){
        angle_step += 180;
        G_sentry.frl_speed_dir = !G_sentry.frl_speed_dir;
    }
    G_sentry.fra_angle_des_pre = angle_raw;
    G_sentry.fra_angle_des += angle_step;
    // br
    speed_x = chassis_line_speed_x + -1.0 * chassis_spin_line_speed * cos(-45.0 / RADIAN2DEGREE_VALUE);
    speed_y = chassis_line_speed_y + -1.0 * chassis_spin_line_speed * sin(-45.0 / RADIAN2DEGREE_VALUE);
    float brl_line_speed_des = sqrt(speed_x * speed_x + speed_y * speed_y);
    angle_raw = safeAtan2(speed_y, speed_x) * RADIAN2DEGREE_VALUE;
    if(!G_sentry.standard_driving_init_flag){
        G_sentry.bra_angle_des = angle_raw;
        G_sentry.bra_angle_des_pre = angle_raw;
    }
    angle_step = angle_raw - G_sentry.bra_angle_des_pre;
    if (angle_step > 180) {
        angle_step -= 360;
    } else if (angle_step < -180) {
        angle_step += 360;
    }
    if(angle_step > 90){
        angle_step -= 180;
        G_sentry.brl_speed_dir = !G_sentry.brl_speed_dir;
    }else if(angle_step < -90){
        angle_step += 180;
        G_sentry.brl_speed_dir = !G_sentry.brl_speed_dir;
    }
    G_sentry.bra_angle_des_pre = angle_raw;
    G_sentry.bra_angle_des += angle_step;
    // bl
    speed_x = chassis_line_speed_x + chassis_spin_line_speed * cos(45.0 / RADIAN2DEGREE_VALUE);
    speed_y = chassis_line_speed_y + chassis_spin_line_speed * sin(45.0 / RADIAN2DEGREE_VALUE);
    float bll_line_speed_des = sqrt(speed_x * speed_x + speed_y * speed_y);
    angle_raw = safeAtan2(speed_y, speed_x) * RADIAN2DEGREE_VALUE;
    if(!G_sentry.standard_driving_init_flag){
        G_sentry.bla_angle_des = angle_raw;
        G_sentry.bla_angle_des_pre = angle_raw;
    }
    angle_step = angle_raw - G_sentry.bla_angle_des_pre;
    if (angle_step > 180) {
        angle_step -= 360;
    } else if (angle_step < -180) {
        angle_step += 360;
    }
    if(angle_step > 90){
        angle_step -= 180;
        G_sentry.bll_speed_dir = !G_sentry.bll_speed_dir;
    }else if(angle_step < -90){
        angle_step += 180;
        G_sentry.bll_speed_dir = !G_sentry.bll_speed_dir;
    }
    G_sentry.bla_angle_des_pre = angle_raw;
    G_sentry.bla_angle_des += angle_step;

    G_sentry.SetChassisSpeedTarget(
        fll_line_speed_des / CHASSIS_WHEEL_RADIUS * RADIAN2DEGREE_VALUE * ((G_sentry.fll_speed_dir)?1.0:-1.0), 
        bll_line_speed_des / CHASSIS_WHEEL_RADIUS * RADIAN2DEGREE_VALUE * ((G_sentry.bll_speed_dir)?1.0:-1.0), 
        -frl_line_speed_des / CHASSIS_WHEEL_RADIUS * RADIAN2DEGREE_VALUE * ((G_sentry.frl_speed_dir)?1.0:-1.0), 
        -brl_line_speed_des / CHASSIS_WHEEL_RADIUS * RADIAN2DEGREE_VALUE * ((G_sentry.brl_speed_dir)?1.0:-1.0));
    G_sentry.SetChassisAngleTarget(G_sentry.fla_angle_des,
                                            G_sentry.bla_angle_des,
                                            G_sentry.fra_angle_des,
                                            G_sentry.bra_angle_des);
    G_sentry.standard_driving_init_flag = true;
    #endif
}


/**
 *@brief update the robot differential drive chassis targets
 *
 *@param
 */
void DiffDriveChassisTargetsUpdate(void)
{
#ifdef CHASSIS_DIFFERENTIAL_DRIVING_MODE
    if (G_sentry.chassis_mode == SentryRobot::CHASSIS_MANNAL)
    {
        // contorlled by the remote controller
        float chassis_line_speed = 0;
        float chassis_angle_speed = 0;
        if (fabs((float)G_djirc.channel.Ch3 - RC_CH_VALUE_OFFSET) > RC_CH_VALUE_DEAD)
        {
            chassis_line_speed = ((float)(G_djirc.channel.Ch3 - RC_CH_VALUE_OFFSET) / RC_CH_VALUE_RANGE * CHASSIS_LINE_SPEED_MAX / CHASSIS_WHEEL_RADIUS);
        }
        if (fabs((float)G_djirc.channel.Ch2 - RC_CH_VALUE_OFFSET) > RC_CH_VALUE_DEAD)
        {
            chassis_angle_speed = -((float)(G_djirc.channel.Ch2 - RC_CH_VALUE_OFFSET) / RC_CH_VALUE_RANGE * CHASSIS_ANGLE_SPEED_MAX);
        }

        float left_speed = chassis_line_speed - CHASSIS_HALF_WHEEL_TREAD * chassis_angle_speed;
        float right_speed = chassis_line_speed + CHASSIS_HALF_WHEEL_TREAD * chassis_angle_speed;

        G_sentry.SetChassisSpeedTarget(left_speed, left_speed, -right_speed, -right_speed);
        G_sentry.SetChassisAngleTarget(0, 0, 0, 0);
    }
    else if (G_sentry.chassis_mode == SentryRobot::CHASSIS_AUTO)
    {
        float left_speed = G_navigation.m_data_receive_frame.m_data[0];
        float right_speed = G_navigation.m_data_receive_frame.m_data[1];
        if (left_speed > CHASSIS_LINE_SPEED_MAX)
        {
            left_speed = CHASSIS_LINE_SPEED_MAX;
        }
        else if (left_speed < -CHASSIS_LINE_SPEED_MAX)
        {
            left_speed = -CHASSIS_LINE_SPEED_MAX;
        }
        if (left_speed > CHASSIS_LINE_SPEED_MAX)
        {
            right_speed = CHASSIS_LINE_SPEED_MAX;
        }
        else if (left_speed < -CHASSIS_LINE_SPEED_MAX)
        {
            right_speed = -CHASSIS_LINE_SPEED_MAX;
        }
        left_speed = left_speed / CHASSIS_WHEEL_RADIUS;
        right_speed = right_speed / CHASSIS_WHEEL_RADIUS;

        G_sentry.SetChassisSpeedTarget(left_speed, left_speed, -right_speed, -right_speed);
        G_sentry.SetChassisAngleTarget(0, 0, 0, 0);
    }
    else if (G_sentry.chassis_mode == SentryRobot::CHASSIS_CALIBRATE)
    {
        for (int i = 0; i < CHASSIS_MOTOR_NUM; i++)
        {
            G_sentry.chassis_motor[i]->m_encoder->m_sum_value = G_sentry.chassis_motor[i]->m_encoder->m_raw_value;

            G_sentry.chassis_motor[i]->m_encoder->m_zero_value = G_sentry.chassis_motor[i]->m_encoder->m_raw_value;
        }
    }
    else
    {
        G_sentry.SetChassisAngleTarget(0, 0, 0, 0);
        G_sentry.SetChassisSpeedTarget(0, 0, 0, 0);
    }
#endif
}



/**
 *@brief update the robot common drive chassis targets
 *
 *@param
 */
void CommonDriveChassisTargetsUpdate(void)
{
#ifdef CHASSIS_COMMON_DRIVING_MODE
    float chassis_angle_speed = 0;
    float chassis_line_steer = 0;

    if (G_sentry.chassis_mode == SentryRobot::CHASSIS_MANNAL)
    {
        if (abs(G_djirc.channel.Ch3 - RC_CH_VALUE_OFFSET) > RC_CH_VALUE_DEAD)
        {
        }
        if (abs(G_djirc.channel.Ch2 - RC_CH_VALUE_OFFSET) > RC_CH_VALUE_DEAD)
        {
        }
    }
    else if (G_sentry.chassis_mode == SentryRobot::CHASSIS_AUTO)
    {
        chassis_angle_speed = G_navigation.m_data_receive_frame.m_data[0];
        chassis_line_steer = G_navigation.m_data_receive_frame.m_data[1];
        // chassis_angle_speed = chassis_angle_speed * RADIAN2DEGREE_VALUE;
        chassis_angle_speed = chassis_angle_speed;

        // speed max limit
        if (chassis_line_steer > CHASSIS_LINE_SPEED_MAX)
        {
            chassis_line_steer = CHASSIS_LINE_SPEED_MAX;
        }
        if (chassis_angle_speed > CHASSIS_ANGLE_SPEED_MAX)
        {
            chassis_angle_speed = CHASSIS_ANGLE_SPEED_MAX;
        }
        chassis_line_steer = chassis_line_steer / CHASSIS_WHEEL_RADIUS;
    }
    else if (G_sentry.chassis_mode == SentryRobot::CHASSIS_CALIBRATE)
    {
        for (int i = 0; i < CHASSIS_MOTOR_NUM; i++)
        {
            G_sentry.chassis_motor[i]->m_encoder->m_sum_value = G_sentry.chassis_motor[i]->m_encoder->m_raw_value;

            G_sentry.chassis_motor[i]->m_encoder->m_zero_value = G_sentry.chassis_motor[i]->m_encoder->m_raw_value;
        }
    }
    else
    {
    }

    G_sentry.SetChassisSpeedTarget(chassis_line_steer, chassis_line_steer, -chassis_line_steer, -chassis_line_steer);
    G_sentry.SetChassisAngleSpeedTarget(chassis_angle_speed, chassis_angle_speed, chassis_angle_speed, chassis_angle_speed);
#endif
}



/**
 *@brief update the robot gimbal targets
 *
 *@param
 */
void GimbalTargetsUpdate(void)
{
    float first_yaw_angle_step = 0;
    float first_pitch_angle_step = 0;
    float first_yaw_des_pre = 0;
    float second_yaw_angle_step = 0;
    
    // Set first gimbal target
    if (G_sentry.gimbal_mode == SentryRobot::GIMBAL_MANNAL)
    {
        if (fabs((float)G_djirc.channel.Ch0 - RC_CH_VALUE_OFFSET) > RC_CH_VALUE_DEAD)
        {
            first_yaw_angle_step = -((float)(G_djirc.channel.Ch0 - RC_CH_VALUE_OFFSET) / 
            RC_CH_VALUE_RANGE * GIMBAL_YAW_ANGLE_MANNAL_CONTROL_SENSITIVITY);
        }

        if (fabs((float)G_djirc.channel.Ch1 - RC_CH_VALUE_OFFSET) > RC_CH_VALUE_DEAD)
        {
            first_pitch_angle_step = -((float)(G_djirc.channel.Ch1 - RC_CH_VALUE_OFFSET) / 
            RC_CH_VALUE_RANGE * GIMBAL_PITCH_ANGLE_MANNAL_CONTROL_SENSITIVITY);
        }

        first_yaw_des_pre = G_sentry.first_yaw_angle_des;
        G_sentry.first_yaw_angle_des += first_yaw_angle_step;
        G_sentry.first_pitch_angle_des += first_pitch_angle_step;

    }
    else if (G_sentry.gimbal_mode == SentryRobot::GIMBAL_AUTO)
    {
        if(!G_sentry.m_scan_flag){
            first_yaw_des_pre = G_sentry.first_yaw_angle_des;
            G_sentry.first_pitch_angle_des = 0;
        }
        else{
            if (G_sentry.enemy_find_flag && (G_aim_assist.m_data_receive_frame.m_data_f[0] > -10.0f) && (G_aim_assist.m_data_receive_frame.m_data_f[0] < 20.0f)
            && !(fpclassify(G_aim_assist.m_data_receive_frame.m_data_f[1])== FP_ZERO) && !(fpclassify(G_aim_assist.m_data_receive_frame.m_data_f[0])== FP_ZERO)) {
                first_yaw_des_pre = G_sentry.first_yaw_angle_des;
                if(fpclassify(G_aim_assist.m_data_receive_frame.m_data_f[1])== FP_NORMAL)
                G_sentry.first_yaw_angle_des  = G_aim_assist.m_data_receive_frame.m_data_f[1];
                if(fpclassify(G_aim_assist.m_data_receive_frame.m_data_f[0])== FP_NORMAL)
                G_sentry.first_pitch_angle_des = G_aim_assist.m_data_receive_frame.m_data_f[0];
                first_yaw_angle_step = G_sentry.first_yaw_angle_des - first_yaw_des_pre;

                // protect 
                if(fabs(first_yaw_angle_step) > 180.0f) 
                {
                    first_yaw_angle_step = 0;
                    G_sentry.first_yaw_angle_des = first_yaw_des_pre;
                }

                if(fabs(G_aim_assist.m_data_receive_frame.m_data_f[1] - G_sentry. gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_current) < 5.0f)
                {
                    if((fabs(G_aim_assist.m_data_receive_frame.m_data_f[1] - G_sentry. gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_current) < 2.0f) || G_sentry.change_yaw_flag)
                    {
                        G_sentry.change_yaw_flag = 1;
                        G_sentry. gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_pid->m_kp = 35; 
                        G_sentry. gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_td->m_r = 20000;  
                        G_sentry. gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_td->m_k = 1;

                        G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_angle_td->m_r = 20000;
                        G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_angle_pid->m_kp = 100;
                        G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_speed_pid->m_kp = 50;
                    }               
                }
                else{
                    G_sentry.change_yaw_flag = 0;
                    G_sentry. gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_pid->m_kp = 15; 
                    G_sentry. gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_td->m_r = 1700;  
                    G_sentry. gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_td->m_k = 0;

                    G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_angle_td->m_r = 1500;
                    G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_angle_pid->m_kp = 40;
                    G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_speed_pid->m_kp = 50;
                }

                if(fabs(G_aim_assist.m_data_receive_frame.m_data_f[0] - G_sentry. gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_current) < 3.0f)
                {
                    if((fabs(G_aim_assist.m_data_receive_frame.m_data_f[0] - G_sentry. gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_current) < 1.5f) || G_sentry.change_pitch_flag)
                    {
                        G_sentry.change_pitch_flag = 1;
                        G_sentry. gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_pid->m_kp = 40; 
                        G_sentry. gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_td->m_r = 7000;  
                        G_sentry. gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_td->m_k = 0.8;
                    }
                }
                else{
                        G_sentry.change_pitch_flag = 0;
                        G_sentry. gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_pid->m_kp = 20; 
                        G_sentry. gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_td->m_r = 2000;  
                        G_sentry. gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_td->m_k = 0;
                }

                G_sentry.m_vision_delay_cnt = 0;
                G_sentry.direction_flag = 1;
            } else {
                    if(G_sentry.m_vision_delay_cnt < 500)
                    {
                        first_yaw_des_pre = G_sentry.first_yaw_angle_des;
                        first_yaw_angle_step = 0;
                        first_pitch_angle_step = 0;
                        G_sentry.m_vision_delay_cnt++;
                    }
                    else{
                        if (G_sentry.first_yaw_scan_dir) {
                            first_yaw_angle_step = GIMBAL_YAW_ANGLE_SCAN_SPEED;
                        } else {
                            first_yaw_angle_step = -GIMBAL_YAW_ANGLE_SCAN_SPEED;
                        }
                        if (G_sentry.first_pitch_scan_dir) {
                            first_pitch_angle_step = GIMBAL_PITCH_ANGLE_SCAN_SPEED;
                        } else {
                            first_pitch_angle_step = -GIMBAL_PITCH_ANGLE_SCAN_SPEED;
                        }
                        first_yaw_des_pre = G_sentry.first_yaw_angle_des;
                        G_sentry.first_yaw_angle_des += first_yaw_angle_step;
                        G_sentry.first_pitch_angle_des += first_pitch_angle_step;

                        if (G_sentry.first_pitch_angle_des > GIMBAL_PITCH_ANGLE_SCAN_ANGLE_MAX) {
                            G_sentry.first_pitch_scan_dir = false;
                        } else if (G_sentry.first_pitch_angle_des < GIMBAL_PITCH_ANGLE_SCAN_ANGLE_MIN) {
                            G_sentry.first_pitch_scan_dir = true;
                        }

                            if (G_aim_assist.m_data_receive_frame.m_data_f[3] < 0.5f && G_sentry.direction_flag) {
                                G_sentry.first_yaw_scan_dir = false;
                            } else if (G_aim_assist.m_data_receive_frame.m_data_f[3] > 0.5f && G_sentry.direction_flag) {
                                G_sentry.first_yaw_scan_dir = true;
                        }
                        G_sentry.m_vision_delay_cnt = 600;
                        G_sentry.direction_flag = 0;
                    }
                   
                    G_sentry. gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_pid->m_kp = 20; 
                    G_sentry. gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_td->m_r = 2000;  
                    G_sentry. gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_td->m_k = 0;

                    G_sentry. gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_pid->m_kp = 15; 
                    G_sentry. gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_td->m_r = 1700;  
                    G_sentry. gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_td->m_k = 0;

                    G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_angle_td->m_r = 500;
                    G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_angle_pid->m_kp = 40;
                    G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_speed_pid->m_kp = 50;
                }
            }

        }
        // else if (G_sentry.chassis_mode == SentryRobot::CHASSIS_MANNAL && 
        // G_sentry.chassis_mode_pre != SentryRobot::CHASSIS_MANNAL)
        // {
        //     // debug
        //     G_sentry.first_yaw_angle_des = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_current;
        //     G_sentry.first_pitch_angle_des = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_current;
        // }
        // else if (G_sentry.chassis_mode == SentryRobot::CHASSIS_MANNAL)
        // {
        //     G_sentry.first_yaw_angle_des = G_sentry.first_yaw_angle_des;
        //     G_sentry.first_pitch_angle_des = G_sentry.first_pitch_angle_des;
        // }
    else
    {
        // debug
        first_yaw_des_pre = G_sentry.first_yaw_angle_des;
        G_sentry.first_yaw_angle_des = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_current;
        G_sentry.first_pitch_angle_des = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_current;

        for (int i = 0; i < GIMBAL_MOTOR_NUM; i++)
        {
            G_sentry.gimbal_motor[i]->m_angle_target = G_sentry.gimbal_motor[i]->m_angle_current;
            G_sentry.gimbal_motor[i]->m_angle_td->m_aim = G_sentry.gimbal_motor[i]->m_angle_target;
            G_sentry.gimbal_motor[i]->m_angle_td->m_x1 = G_sentry.gimbal_motor[i]->m_angle_target;
            G_sentry.gimbal_motor[i]->m_angle_td->m_x2 = 0;
            G_sentry.gimbal_motor[i]->m_speed_pid->m_output = 0;
        }
    }


    // Set second gimbal target
    if (G_sentry.gimbal_mode == SentryRobot::GIMBAL_MANNAL)
    {
        if (fabs((float)G_djirc.channel.Ch2 - RC_CH_VALUE_OFFSET) > RC_CH_VALUE_DEAD)
        {
            // second_yaw_angle_step = -((float)(G_djirc.channel.Ch2 - RC_CH_VALUE_OFFSET) / 
            // RC_CH_VALUE_RANGE * GIMBAL_YAW_ANGLE_MANNAL_CONTROL_SENSITIVITY);
            if(!G_sentry.second_yaw_step_flag)
                second_yaw_angle_step = 30;
            G_sentry.second_yaw_step_flag = 1;
        }
		else
        {
            G_sentry.second_yaw_step_flag = 0;
            second_yaw_angle_step = -(G_sentry.first_yaw_angle_des - first_yaw_des_pre);
        }
		
    }
    else if (G_sentry.gimbal_mode == SentryRobot::GIMBAL_AUTO || 
    G_sentry.chassis_mode == SentryRobot::CHASSIS_AUTO)
    {
        second_yaw_angle_step = -(G_sentry.first_yaw_angle_des - first_yaw_des_pre);
    }
    else
    {
        second_yaw_angle_step = 0;
        for (int i = 0; i < GIMBAL_MOTOR_NUM; i++)
        {
            G_sentry.gimbal_motor[i]->m_angle_target = G_sentry.gimbal_motor[i]->m_angle_current;
            G_sentry.gimbal_motor[i]->m_angle_td->m_aim = G_sentry.gimbal_motor[i]->m_angle_target;
            G_sentry.gimbal_motor[i]->m_angle_td->m_x1 = G_sentry.gimbal_motor[i]->m_angle_target;
            G_sentry.gimbal_motor[i]->m_angle_td->m_x2 = 0;
            G_sentry.gimbal_motor[i]->m_speed_pid->m_output = 0;
        }
    }
    G_sentry.second_yaw_angle_des = G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->
    m_angle_target + second_yaw_angle_step;
    
    G_sentry.first_pitch_angle_des = Clip(G_sentry.first_pitch_angle_des,GIMBAL_PITCH_MIN,GIMBAL_PITCH_MAX);
    G_sentry.SetGimbalAngleTarget(G_sentry.first_yaw_angle_des, G_sentry.first_pitch_angle_des, G_sentry.second_yaw_angle_des);
}



/**
 *@brief update the robot gimbal targets
 *
 *@param
 */
void ShootTargetsUpdate(void)
{
    static uint32_t bullut_cnt_target_pre = 0;
    bullut_cnt_target_pre = G_sentry.m_shoot_bullet_cnt_target;

    // Set the shoot insurance (control the gimbal friction wheels)
    if (G_sentry.shoot_mode == SentryRobot::SHOOT_AUTO)
    {
        G_sentry.shoot_insurance = false;
    }
    else if (G_sentry.shoot_mode == SentryRobot::SHOOT_MANNAL)
    {
        if (G_djirc.channel.Ch4 - RC_CH_VALUE_OFFSET < -RC_CH_VALUE_DEAD)
        {
            G_sentry.shoot_insurance = true;
        }
        else if (G_djirc.channel.Ch4 - RC_CH_VALUE_OFFSET > RC_CH_VALUE_DEAD)
        {
            G_sentry.shoot_insurance = false;
        }
    }
    else
    {
        G_sentry.shoot_insurance = true;
    }

    // Update the robot shoot bullet count target
    if (G_sentry.shoot_mode == SentryRobot::SHOOT_MANNAL && G_sentry.shoot_insurance == false && !G_sentry.change_gun_flag )
    {
        if (G_djirc.channel.Ch3 - RC_CH_VALUE_OFFSET < -RC_CH_VALUE_DEAD)
        {
            G_sentry.m_shoot_bullet_fps = SHOOT_BULLET_FREQUENCY_TEST_1;
            G_sentry.IncreaseShootBulletTarget(G_system_monitor.SysTickTime);
        }
        else if (G_djirc.channel.Ch3 - RC_CH_VALUE_OFFSET > RC_CH_VALUE_DEAD)
        {
            G_sentry.m_shoot_bullet_fps = SHOOT_BULLET_FREQUENCY_TEST_4;
            G_sentry.IncreaseShootBulletTarget(G_system_monitor.SysTickTime);
        }
    }else if(G_sentry.shoot_mode == SentryRobot::SHOOT_MANNAL && G_sentry.shoot_insurance == true)
    {
        if (G_djirc.channel.Ch3 - RC_CH_VALUE_OFFSET < -RC_CH_VALUE_DEAD)
        {
            if(G_sentry.change_delay_cnt == 1000){
                G_sentry.change_gun_flag = true;
            }
            G_sentry.change_delay_cnt++;
        }else{
            G_sentry.change_delay_cnt = 0;
        }
    } 
    else if (G_sentry.shoot_mode == SentryRobot::SHOOT_AUTO && 
    G_sentry.shoot_insurance == false && 
    !G_sentry.change_gun_flag && 
    G_sentry.m_scan_flag && 
    fabs(G_aim_assist.m_data_receive_frame.m_data_f[1] - G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_YAW_MOTOR]->m_angle_current) < Yaw_Vision_Error && 
    fabs(G_aim_assist.m_data_receive_frame.m_data_f[0] - G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_current) < Pitch_Vison_Error && 
    G_aim_assist.m_data_receive_frame.m_data_f[2] > 0.5 &&
    !((G_aim_assist.m_data_receive_frame.m_data_f[3] == 1 && G_aim_assist.m_enemy_1_res_flag)||
        (G_aim_assist.m_data_receive_frame.m_data_f[3] == 2 && (G_aim_assist.m_enemy_2_res_flag || (G_navigation.m_data_receive_frame.m_data_c[1] == 0)))||
        (G_aim_assist.m_data_receive_frame.m_data_f[3] == 3 && G_aim_assist.m_enemy_3_res_flag)||
        (G_aim_assist.m_data_receive_frame.m_data_f[3] == 4 && G_aim_assist.m_enemy_4_res_flag)||
        (G_aim_assist.m_data_receive_frame.m_data_f[3] == 5 && G_aim_assist.m_enemy_5_res_flag)||
        (G_aim_assist.m_data_receive_frame.m_data_f[3] == 7 && G_aim_assist.m_enemy_7_res_flag)))
    {
        G_sentry.m_shoot_bullet_fps = SHOOT_BULLET_FREQUENCY_TEST_4;
        G_sentry.IncreaseShootBulletTarget(G_system_monitor.SysTickTime);
    }
    else if(G_sentry.change_gun_flag)
    {
        // G_sentry.m_shoot_bullet_fps = SHOOT_BULLET_FREQUENCY_TEST_0;
        // G_sentry.IncreaseShootBulletTarget(G_system_monitor.SysTickTime);
    }
    else
    {
        G_sentry.m_shoot_bullet_cnt_target = 0;
        bullut_cnt_target_pre = 0;
        G_sentry.shoot_driver_angle_target =
            G_sentry.shoot_motor[SentryRobot::SHOOT_DRIVE_MOTOR]->m_angle_current;
    }

    if(G_sentry.shoot_motor[SentryRobot::SHOOT_DRIVE_MOTOR]->m_angle_current - G_sentry.shoot_motor[SentryRobot::SHOOT_DRIVE_MOTOR]->m_angle_target > SHOOT_DRIVER_SUPPLY_ANGLE_STEP * 10){
        G_sentry.shoot_driver_angle_target = G_sentry.shoot_motor[SentryRobot::SHOOT_DRIVE_MOTOR]->m_angle_current + SHOOT_DRIVER_SUPPLY_ANGLE_STEP;
    }else if(G_sentry.shoot_motor[SentryRobot::SHOOT_DRIVE_MOTOR]->m_angle_target - G_sentry.shoot_motor[SentryRobot::SHOOT_DRIVE_MOTOR]->m_angle_current > SHOOT_DRIVER_SUPPLY_ANGLE_STEP){
        G_sentry.shoot_driver_angle_target = G_sentry.shoot_motor[SentryRobot::SHOOT_DRIVE_MOTOR]->m_angle_current - SHOOT_DRIVER_SUPPLY_ANGLE_STEP;
    }
    // Update the shoot drive motor position target
    G_sentry.shoot_driver_angle_target -= SHOOT_DRIVER_SUPPLY_ANGLE_STEP *
                                (G_sentry.m_shoot_bullet_cnt_target - bullut_cnt_target_pre);
    G_sentry.SetShootDriverTarget(G_sentry.shoot_driver_angle_target);
}



/**
 *@brief execute the robot control
 *
 *@param
 */
void RobotControlExecute(void)
{
    // Execute the robot chassis control algorithm
    G_sentry.ExecuteChassisAlgorithm();

    // Execute the robot gimbal control algorithm
    G_sentry.ExecuteGimbalAlgorithm();

    // Execute the robot shoot control algorithm
    G_sentry.ExecuteShootAlgorithm();

    // Send the control commands to the actuators
    G_sentry.SendControlCommand();
}





