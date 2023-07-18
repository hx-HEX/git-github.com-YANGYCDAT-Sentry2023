#include "user_global.h"

SystemMonitor G_system_monitor;
Led G_led;
Vofa G_vofa;
Gimbal G_gimbal;
SentryRobot G_sentry;
GunShift G_gun_shift;

/**
 *@brief visualize chassis datas
 * 
 *@param 
*/
void VisualizeGimbalData(void) {
    G_vofa.m_data_send_frame.m_data[0] = G_gimbal.m_data_receive_frame.m_sdata[0];
    G_vofa.m_data_send_frame.m_data[1] = G_gimbal.m_data_receive_frame.m_sdata[1];
    G_vofa.m_data_send_frame.m_data[2] = G_gimbal.m_data_receive_frame.m_sdata[2];
    G_vofa.m_data_send_frame.m_data[3] = G_gimbal.m_data_receive_frame.m_cdata[0];
    G_vofa.m_data_send_frame.m_data[4] = G_gimbal.m_data_receive_frame.m_cdata[1];
    G_vofa.m_data_send_frame.m_data[5] = G_gimbal.m_data_receive_frame.m_cdata[2];
    G_vofa.m_data_send_frame.m_data[6] = G_gimbal.m_data_receive_frame.m_cdata[3];
    G_vofa.m_data_send_frame.m_data[7] = G_gimbal.m_data_receive_frame.m_cdata[4];
    G_vofa.m_data_send_frame.m_data[8] = G_gun_shift.change_flag;
    G_vofa.m_data_send_frame.m_data[9] = G_gun_shift.change_gun_flag;
    G_vofa.m_data_send_frame.m_data[10] = G_gun_shift.m_target_angle;

    // G_vofa.m_data_send_frame.m_data[0] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_encoder->m_raw_value;
    // G_vofa.m_data_send_frame.m_data[5] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_kalman_filter_angle->GetFilterOutput();
    // G_vofa.m_data_send_frame.m_data[6] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_angle_current;
    // G_vofa.m_data_send_frame.m_data[7] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_kalman_filter_angle->GetFilterOutput();
    // G_vofa.m_data_send_frame.m_data[4] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_speed_pid->m_output;
    // G_vofa.m_data_send_frame.m_data[5] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_speed_pid->m_output;
    // G_vofa.m_data_send_frame.m_data[1] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_angle_current;
    // G_vofa.m_data_send_frame.m_data[2] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_speed_current;
    // G_vofa.m_data_send_frame.m_data[8] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_kalman_filter_speed->GetFilterOutput();
    // G_vofa.m_data_send_frame.m_data[9] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_current;
    // G_vofa.m_data_send_frame.m_data[9] = G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_speed_current;
    // G_vofa.m_data_send_frame.m_data[0] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_gyro_data.x;
    // G_vofa.m_data_send_frame.m_data[1] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_gyro_data.y;
    // G_vofa.m_data_send_frame.m_data[2] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_gyro_data.z;
    // G_vofa.m_data_send_frame.m_data[4] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_gyro_offset_x;
    // G_vofa.m_data_send_frame.m_data[5] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_gyro_offset_y;
    // G_vofa.m_data_send_frame.m_data[6] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_gyro_offset_z;
    // G_vofa.m_data_send_frame.m_data[3] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_acc_data.x;
    // G_vofa.m_data_send_frame.m_data[4] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_acc_data.y;
    // G_vofa.m_data_send_frame.m_data[5] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_acc_data.z;
    // G_vofa.m_data_send_frame.m_data[6] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_mahony_filter->m_eular_angle.yaw;
    // G_vofa.m_data_send_frame.m_data[7] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_mahony_filter->m_eular_angle.pitch;
    // G_vofa.m_data_send_frame.m_data[8] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_mahony_filter->m_eular_angle.roll;
    // G_vofa.m_data_send_frame.m_data[0] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_gyro_data.x;
    // G_vofa.m_data_send_frame.m_data[1] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_gyro_data.y;
    // G_vofa.m_data_send_frame.m_data[2] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_gyro_data.z;
    // G_vofa.m_data_send_frame.m_data[3] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_gyro_offset_x;
    // G_vofa.m_data_send_frame.m_data[4] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_gyro_offset_y;
    // G_vofa.m_data_send_frame.m_data[5] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_gyro_offset_z;
    // G_vofa.m_data_send_frame.m_data[6] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_acc_data.x;
    // G_vofa.m_data_send_frame.m_data[7] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_acc_data.y;
    // G_vofa.m_data_send_frame.m_data[8]= G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_acc_data.z;
    // G_vofa.m_data_send_frame.m_data[9] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_mahony_filter->m_eular_angle.yaw;
    // G_vofa.m_data_send_frame.m_data[10] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_mahony_filter->m_eular_angle.pitch;
    // G_vofa.m_data_send_frame.m_data[11] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_mahony_filter->m_eular_angle.roll;
    // G_vofa.m_data_send_frame.m_data[12] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_gyro_data.z;
    // G_vofa.m_data_send_frame.m_data[13] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_kalman_filter_gyro_z->GetFilterOutput() * RADIAN2DEGREE_VALUE;
    // G_vofa.m_data_send_frame.m_data[14] = G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->m_mahony_filter->m_com_gyro.z;
    // G_vofa.m_data_send_frame.m_data[4] = G_sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_smc->m_TD->m_aim;
    // G_vofa.m_data_send_frame.m_data[5] = G_sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_smc->m_TD->m_x1;
    // G_vofa.m_data_send_frame.m_data[6] = G_sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_smc->m_TD->m_x2;
    // G_vofa.m_data_send_frame.m_data[7] = G_sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_smc->m_fpE;
    // G_vofa.m_data_send_frame.m_data[8] = G_sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_smc->m_fpU;
    // G_vofa.m_data_send_frame.m_data[9] = G_sentry.shoot_motor[SentryRobot::RIGHT_FRIC_MOTOR]->m_smc->m_fpU;
    // G_vofa.m_data_send_frame.m_data[10] = G_sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_smc->m_fpDes;
    // G_vofa.m_data_send_frame.m_data[11] = G_sentry.shoot_motor[SentryRobot::RIGHT_FRIC_MOTOR]->m_smc->m_fpDes;
    // G_vofa.m_data_send_frame.m_data[12] = G_sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_smc->m_fpFB;
    // G_vofa.m_data_send_frame.m_data[13] = G_sentry.shoot_motor[SentryRobot::RIGHT_FRIC_MOTOR]->m_smc->m_fpFB;

    G_vofa.m_data_send_frame.m_data[13] = G_system_monitor.UART6_rx_fps;
    G_vofa.m_data_send_frame.m_data[14] = G_system_monitor.CAN2_rx_fps;
}


/**
 *@brief update system control mode
 * 
 *@param 
*/
void ControlModeUpdate(void) {
};



/**
 *@brief update the state of the sentry robot
 * 
 *@param 
*/
void RobotStatesUpdate(void)
{
    if (G_system_monitor.UART6_rx_fps < 900) {
        G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_speed_pid->m_output
        = 0;
        G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_speed_pid->m_output
        = 0;
    } else {
        G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_speed_pid->m_output
        = G_gimbal.m_data_receive_frame.m_sdata[0];

        G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_speed_pid->m_output
        = G_gimbal.m_data_receive_frame.m_sdata[1];

        G_gun_shift.change_gun_flag = G_gimbal.m_data_receive_frame.m_cdata[1];
    }

    G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->UpdataAccData();
    G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->UpdataGyroData();
    G_sentry.gimbal_imu[SentryRobot::GIMBAL_FIRST_IMU]->UpdataAngleData();

    G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_kalman_filter_angle
    ->UpdateFilter(G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_angle_current);
    G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_kalman_filter_angle
    ->UpdateFilter(G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_angle_current);
    
    G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_kalman_filter_speed
    ->UpdateFilter(G_sentry.gimbal_motor[SentryRobot::GIMBAL_SECOND_YAW_MOTOR]->m_speed_current);
    G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_kalman_filter_speed
    ->UpdateFilter(G_sentry.gimbal_motor[SentryRobot::GIMBAL_FIRST_PITCH_MOTOR]->m_speed_current);
}



/**
 *@brief update the robot targets 
 * 
 *@param 
*/
void RobotTargetsUpdate(void) {
    if (G_system_monitor.UART6_rx_fps > 100 && 
    G_gimbal.m_data_receive_frame.m_cdata[0] == 0) {
        G_sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_smc->m_fpDes = -FRIC_WHEEL_SPEED;
        G_sentry.shoot_motor[SentryRobot::RIGHT_FRIC_MOTOR]->m_smc->m_fpDes = FRIC_WHEEL_SPEED;
    } else {
        G_sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_smc->m_fpDes = 0;
        G_sentry.shoot_motor[SentryRobot::RIGHT_FRIC_MOTOR]->m_smc->m_fpDes = 0;
    }
}



/**
 *@brief execute the robot control 
 * 
 *@param 
*/
void RobotControlExecute(void)
{
    // Execute the robot shoot control algorithm
    if (G_system_monitor.UART6_rx_fps > 100 && 
    G_gimbal.m_data_receive_frame.m_cdata[0] == 0) {
        G_sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_smc->CalSMC();
        G_sentry.shoot_motor[SentryRobot::RIGHT_FRIC_MOTOR]->m_smc->CalSMC();

    } else {
        G_sentry.shoot_motor[SentryRobot::LEFT_FRIC_MOTOR]->m_smc->m_fpU = 0;
        G_sentry.shoot_motor[SentryRobot::RIGHT_FRIC_MOTOR]->m_smc->m_fpU = 0;
    }

    if(G_gun_shift.change_flag == 1 && G_gun_shift.change_gun_flag == 1)
    {
        if(G_gun_shift.m_target_angle == GUNSHIFT_ANGLE_1)
            G_gun_shift.m_target_angle = GUNSHIFT_ANGLE_2;
        else
            G_gun_shift.m_target_angle = GUNSHIFT_ANGLE_1;
        G_gun_shift.change_flag = 0;
    }
    if(G_gun_shift.change_gun_flag == 0) G_gun_shift.change_flag = 1;

    // Send the control commands to the actuators
    G_sentry.SendControlCommand();
}