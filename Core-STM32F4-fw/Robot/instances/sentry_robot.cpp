#include "user_global.h"
#include "sentry_robot.h"
#include "can.h"
#include "m3508.h"
#include "gm6020.h"
#include "m2006.h"
#include "pid.h"
#include "adrc.h"
#include "encoder.h"
#include "common_math.h"




/**
*@brief construct sentry robot
* 
*@param 
*/
SentryRobot::SentryRobot(void)
{
    chassis_mode = CHASSIS_SAFE;
    chassis_mode_pre = CHASSIS_SAFE;

    gimbal_mode = GIMBAL_SAFE;
    gimbal_mode_pre = GIMBAL_SAFE;

    shoot_mode = SHOOT_SAFE;
    shoot_mode_pre = SHOOT_SAFE;
    shoot_insurance = true;
    m_shoot_bullet_fps = SHOOT_BULLET_FREQUENCY_STABLE;
    m_shoot_bullet_cnt_target = 0;
    m_shoot_bullet_last_increase_time = 0;
    shoot_driver_angle_target = 0;

    m_robot_chassis_world_error = 0.0f;
    m_robot_wheel_world_yaw = 0.0f;
    m_robot_chassis_speed = 0.0f;

#ifdef CHASSIS_STEER_DRIVING_MODE
    m_robot_wheel_world_yaw_des_sum = 0.0f;
    m_robot_wheel_world_speed_des = 0.0f;
    m_robot_chassis_spin_ready = false;

#endif

#ifdef CHASSIS_STANDARD_DRIVING_MODE
    fll_speed_dir = true;
    frl_speed_dir = true; 
    brl_speed_dir = true;
    bll_speed_dir = true;
    standard_driving_init_flag = false;
    world_line_speed_angle_des = 0.0;
    world_line_speed_angle_pre = 0.0;
    fla_angle_des = 0.0;
    fla_angle_des_pre = 0.0;
    fra_angle_des = 0.0;
    fra_angle_des_pre = 0.0;
    bra_angle_des = 0.0;
    bra_angle_des_pre = 0.0;
    bla_angle_des = 0.0;
    bla_angle_des_pre = 0.0;
    
#endif
    m_robot_radar_world_yaw = 0.0f;

    m_flag_cnt = 0.0f;
    m_shoot_cnt = 0.0f;
    m_vision_delay_cnt = 0.0f;

    ctrl_spinning_flag = true;
    spinning_flag = false;
    change_gun_flag = false;
    safe_flag = false;
    vision_flag = false;
    m_shoot_speed_filter = new EWMA(SHOOT_SPEED_FILTER_BETA);
    m_shoot_speed_pre = 0.0;
    enemy_find_flag = false;
    first_yaw_scan_dir = true;
    first_pitch_scan_dir = true;
    first_yaw_angle_des = 0.0;
    first_pitch_angle_des = 0.0;
    second_yaw_angle_des = 0.0;
    change_yaw_flag = 0;
    change_pitch_flag = 0;
    change_cnt = 0;
    change_delay_cnt = 0;
    direction_flag = 0;
    sin_cnt = 0;
    second_yaw_step_flag = 0;
}



/**
*@brief Initial the whole Sentry robot
* 
*@param 
*/
void SentryRobot::
Init(void)
{
    // Initial all actuators
    InitAllActuators();

    // Initial all Sensors
    InitAllSensors();
}



/**
*@brief Initial all actuators of the Sentry robot
* 
*@param 
*/
void SentryRobot::InitAllActuators(void)
{
    chassis_steer_motor[CHASSIS_FLA_MOTOR] = new GM6020(CAN2, CHASSIS_FLA_MOTOR_ID,CHASSIS_STEER_MOTOR_REDUCTION_RATIO);
    chassis_steer_motor[CHASSIS_FLA_MOTOR]->m_angle_td = new Adrc_TD(8000, 0.001, 0.001,0);
    chassis_steer_motor[CHASSIS_FLA_MOTOR]->m_angle_pid = new Pid(120, 0.1, 0, 10, 2000, 2000, 2000, 2000);
    chassis_steer_motor[CHASSIS_FLA_MOTOR]->m_speed_pid = new Pid(25, 0.00, 0, 10, 10000, 10000, 5000, 2000);
    chassis_steer_motor[CHASSIS_FLA_MOTOR]->m_encoder = new AbsEncoder(CHASSIS_FLA_ENCODER_ZERO_VALUE, ENCODER_RESOLUTION);  
    chassis_steer_motor[CHASSIS_FLA_MOTOR]->m_kalman_filter_angle = new Kalman(1, 0.001f, 0.0001f,0.8f, 0.01f);
    chassis_steer_motor[CHASSIS_FLA_MOTOR]->m_kalman_filter_speed = new Kalman(1, 0.001f, 0.0001f,0.05f, 0.005f);

    
    chassis_steer_motor[CHASSIS_FRA_MOTOR] = new GM6020(CAN1, CHASSIS_FRA_MOTOR_ID,CHASSIS_STEER_MOTOR_REDUCTION_RATIO);
    chassis_steer_motor[CHASSIS_FRA_MOTOR]->m_angle_td = new Adrc_TD(8000, 0.001, 0.001,0);
    chassis_steer_motor[CHASSIS_FRA_MOTOR]->m_angle_pid = new Pid(120, 0.1, 0, 10, 2000, 2000, 5000, 2000);
    chassis_steer_motor[CHASSIS_FRA_MOTOR]->m_speed_pid = new Pid(25, 0.00, 0, 10, 10000, 10000, 5000, 2000);
    chassis_steer_motor[CHASSIS_FRA_MOTOR]->m_encoder = new AbsEncoder(CHASSIS_FRA_ENCODER_ZERO_VALUE,ENCODER_RESOLUTION);
    chassis_steer_motor[CHASSIS_FRA_MOTOR]->m_kalman_filter_angle = new Kalman(1, 0.001f, 0.0001f,0.8f, 0.01f);
    chassis_steer_motor[CHASSIS_FRA_MOTOR]->m_kalman_filter_speed = new Kalman(1, 0.001f, 0.0001f,0.05f, 0.005f);

    
    chassis_steer_motor[CHASSIS_BLA_MOTOR] = new GM6020(CAN2, CHASSIS_BLA_MOTOR_ID, CHASSIS_STEER_MOTOR_REDUCTION_RATIO);
    chassis_steer_motor[CHASSIS_BLA_MOTOR]->m_angle_td = new Adrc_TD(8000, 0.001, 0.001,0);
    chassis_steer_motor[CHASSIS_BLA_MOTOR]->m_angle_pid = new Pid(120, 0.1, 0, 10, 2000, 2000, 5000, 2000);
    chassis_steer_motor[CHASSIS_BLA_MOTOR]->m_speed_pid = new Pid(25, 0.00, 0, 10, 10000, 10000, 5000, 2000);
    chassis_steer_motor[CHASSIS_BLA_MOTOR]->m_encoder = new AbsEncoder(CHASSIS_BLA_ENCODER_ZERO_VALUE, ENCODER_RESOLUTION);
    chassis_steer_motor[CHASSIS_BLA_MOTOR]->m_kalman_filter_angle = new Kalman(1, 0.001f, 0.0001f,0.8f, 0.01f);
    chassis_steer_motor[CHASSIS_BLA_MOTOR]->m_kalman_filter_speed = new Kalman(1, 0.001f, 0.0001f,0.05f, 0.005f);

        
    chassis_steer_motor[CHASSIS_BRA_MOTOR] = new GM6020(CAN1, CHASSIS_BRA_MOTOR_ID, CHASSIS_STEER_MOTOR_REDUCTION_RATIO);
    chassis_steer_motor[CHASSIS_BRA_MOTOR]->m_angle_td = new Adrc_TD(8000, 0.001, 0.001,0);
    chassis_steer_motor[CHASSIS_BRA_MOTOR]->m_angle_pid = new Pid(120, 0.1, 0, 10, 2000, 2000, 5000, 2000);
    chassis_steer_motor[CHASSIS_BRA_MOTOR]->m_speed_pid = new Pid(25, 0.00, 0, 10, 10000, 10000, 5000, 2000);
    chassis_steer_motor[CHASSIS_BRA_MOTOR]->m_encoder = new AbsEncoder(CHASSIS_BRA_ENCODER_ZERO_VALUE, ENCODER_RESOLUTION);
    chassis_steer_motor[CHASSIS_BRA_MOTOR]->m_kalman_filter_angle = new Kalman(1, 0.001f, 0.0001f,0.8f, 0.01f);
    chassis_steer_motor[CHASSIS_BRA_MOTOR]->m_kalman_filter_speed = new Kalman(1, 0.001f, 0.0001f,0.05f, 0.005f);

    
    chassis_line_motor[CHASSIS_FRL_MOTOR] = new M3508(CAN1, CHASSIS_FRL_MOTOR_ID, CHASSIS_SPEED_MOTOR_REDUCTION_RATIO);
    chassis_line_motor[CHASSIS_FRL_MOTOR]->m_speed_td = new Adrc_TD(7000, 0.001, 0.001,0);
    chassis_line_motor[CHASSIS_FRL_MOTOR]->m_angle_pid = new Pid(0, 0.00, 0, 10, 2000, 2000, 5000, 2000);
    chassis_line_motor[CHASSIS_FRL_MOTOR]->m_speed_pid = new Pid(40, 0.00, 0, 10, 10000, 10000, 5000, 2000);
    chassis_line_motor[CHASSIS_FRL_MOTOR]->m_encoder = new AbsEncoder(CHASSIS_FRL_ENCODER_ZERO_VALUE, ENCODER_RESOLUTION);
    chassis_line_motor[CHASSIS_FRL_MOTOR]->m_kalman_filter_angle = new Kalman(1, 0.001f, 0.0001f,0.8f, 0.01f);
    chassis_line_motor[CHASSIS_FRL_MOTOR]->m_kalman_filter_speed = new Kalman(1, 0.001f, 0.0001f,0.05f, 0.005f);

    
    chassis_line_motor[CHASSIS_FLL_MOTOR] = new M3508(CAN2, CHASSIS_FLL_MOTOR_ID, CHASSIS_SPEED_MOTOR_REDUCTION_RATIO);
    chassis_line_motor[CHASSIS_FLL_MOTOR]->m_speed_td = new Adrc_TD(7000, 0.001, 0.001,0);
    chassis_line_motor[CHASSIS_FLL_MOTOR]->m_angle_pid = new Pid(0, 0.00, 0, 10, 2000, 2000, 5000, 2000);
    chassis_line_motor[CHASSIS_FLL_MOTOR]->m_speed_pid = new Pid(40, 0.00, 0, 10, 10000, 10000, 5000, 2000);
    chassis_line_motor[CHASSIS_FLL_MOTOR]->m_encoder = new AbsEncoder(CHASSIS_FLL_ENCODER_ZERO_VALUE, ENCODER_RESOLUTION); 
    chassis_line_motor[CHASSIS_FLL_MOTOR]->m_kalman_filter_angle = new Kalman(1, 0.001f, 0.0001f,0.8f, 0.01f);
    chassis_line_motor[CHASSIS_FLL_MOTOR]->m_kalman_filter_speed = new Kalman(1, 0.001f, 0.0001f,0.001f, 0.5f);

    
    chassis_line_motor[CHASSIS_BLL_MOTOR] = new M3508(CAN2, CHASSIS_BLL_MOTOR_ID, CHASSIS_SPEED_MOTOR_REDUCTION_RATIO);
    chassis_line_motor[CHASSIS_BLL_MOTOR]->m_speed_td = new Adrc_TD(7000, 0.001, 0.001,0);
    chassis_line_motor[CHASSIS_BLL_MOTOR]->m_angle_pid = new Pid(0, 0.00, 0, 10, 2000, 2000, 5000, 2000);
    chassis_line_motor[CHASSIS_BLL_MOTOR]->m_speed_pid = new Pid(40, 0.00, 0, 10, 10000, 10000, 5000, 2000);
    chassis_line_motor[CHASSIS_BLL_MOTOR]->m_encoder = new AbsEncoder(CHASSIS_BLL_ENCODER_ZERO_VALUE, ENCODER_RESOLUTION);
    chassis_line_motor[CHASSIS_BLL_MOTOR]->m_kalman_filter_angle = new Kalman(1, 0.001f, 0.0001f,0.8f, 0.01f);
    chassis_line_motor[CHASSIS_BLL_MOTOR]->m_kalman_filter_speed = new Kalman(1, 0.001f, 0.0001f,0.003f, 0.5f);

    
    chassis_line_motor[CHASSIS_BRL_MOTOR] = new M3508(CAN1, CHASSIS_BRL_MOTOR_ID, CHASSIS_SPEED_MOTOR_REDUCTION_RATIO);
    chassis_line_motor[CHASSIS_BRL_MOTOR]->m_speed_td = new Adrc_TD(7000, 0.001, 0.001,0);
    chassis_line_motor[CHASSIS_BRL_MOTOR]->m_angle_pid = new Pid(0, 0.00, 0, 10, 2000, 2000, 5000, 2000);
    chassis_line_motor[CHASSIS_BRL_MOTOR]->m_speed_pid = new Pid(40, 0.00, 0, 10, 10000, 10000, 5000, 2000);
    chassis_line_motor[CHASSIS_BRL_MOTOR]->m_encoder = new AbsEncoder(CHASSIS_BRL_ENCODER_ZERO_VALUE, ENCODER_RESOLUTION); 
    chassis_line_motor[CHASSIS_BRL_MOTOR]->m_kalman_filter_angle = new Kalman(1, 0.001f, 0.0001f,0.8f, 0.01f);
    chassis_line_motor[CHASSIS_BRL_MOTOR]->m_kalman_filter_speed = new Kalman(1, 0.001f, 0.0001f,0.003f, 0.5f);

    
    shoot_motor[SHOOT_DRIVE_MOTOR] = new M2006(CAN1, SHOOT_DRIVE_MOTOR_ID, SHOOT_MOTOR_REDUCTION_RATIO);
    shoot_motor[SHOOT_DRIVE_MOTOR]->m_angle_td = new Adrc_TD(10000, 0.001, 0.001,0);
    shoot_motor[SHOOT_DRIVE_MOTOR]->m_angle_pid = new Pid(0.010, 0.00, 0, 10, 10000, 10000, 0, 0);
    shoot_motor[SHOOT_DRIVE_MOTOR]->m_speed_pid = new Pid(700, 0.00, 0, 10, 10000, 10000, 0, 0);
    shoot_motor[SHOOT_DRIVE_MOTOR]->m_encoder = new AbsEncoder(0, ENCODER_RESOLUTION);   
    shoot_motor[SHOOT_DRIVE_MOTOR]->m_kalman_filter_angle = new Kalman(1, 0.001f, 0.0001f,0.8f, 0.01f);
    shoot_motor[SHOOT_DRIVE_MOTOR]->m_kalman_filter_speed = new Kalman(1, 0.001f, 0.0001f,0.003f, 0.5f);


    gimbal_motor[GIMBAL_FIRST_YAW_MOTOR] = new GM6020(CAN2, FIRST_GIMBAL_YAW_MOTOR_ID, GIMBAL_MOTOR_REDUCTION_RATIO);
    gimbal_motor[GIMBAL_FIRST_YAW_MOTOR]->m_angle_td = new Adrc_TD(20000, 0.001f, 0.001f,0.8);
    gimbal_motor[GIMBAL_FIRST_YAW_MOTOR]->m_angle_pid = new Pid(30, 0.05, 0, 1, 20000, 20000, 10000, 1500);  
    gimbal_motor[GIMBAL_FIRST_YAW_MOTOR]->m_speed_pid = new Pid(200, 0.00, 0, 15, 28000, 28000, 20000, 20000);
    gimbal_motor[GIMBAL_FIRST_YAW_MOTOR]->m_encoder = new AbsEncoder(FIRST_GIMBAL_YAW_ENCODER_ZERO_VALUE, ENCODER_RESOLUTION);
    gimbal_motor[GIMBAL_FIRST_YAW_MOTOR]->m_kalman_filter_angle = new Kalman(1, 0.001f, 0.0001f,0.1f, 0.01f);
    gimbal_motor[GIMBAL_FIRST_YAW_MOTOR]->m_kalman_filter_speed = new Kalman(1, 0.001f, 0.0001f,0.003f, 0.5f);


    gimbal_motor[GIMBAL_FIRST_PITCH_MOTOR] = new GM6020(CAN2, FIRST_GIMBAL_PITCH_MOTOR_ID, GIMBAL_MOTOR_REDUCTION_RATIO);
    gimbal_motor[GIMBAL_FIRST_PITCH_MOTOR]->m_angle_td = new Adrc_TD(7000, 0.001, 0.001,0.8);
    gimbal_motor[GIMBAL_FIRST_PITCH_MOTOR]->m_angle_pid = new Pid(40, 0.2, 0, 1, 3000, 3000, 3000, 1500);
    gimbal_motor[GIMBAL_FIRST_PITCH_MOTOR]->m_speed_pid = new Pid(200, 0.00, 0, 12, 28000, 28000, 20000, 20000);
    gimbal_motor[GIMBAL_FIRST_PITCH_MOTOR]->m_encoder = new AbsEncoder(FIRST_GIMBAL_PITCH_ENCODER_ZERO_VALUE, ENCODER_RESOLUTION);
    gimbal_motor[GIMBAL_FIRST_PITCH_MOTOR]->m_kalman_filter_angle = new Kalman(1, 0.001f, 0.0001f,0.1f, 0.01f);
    gimbal_motor[GIMBAL_FIRST_PITCH_MOTOR]->m_kalman_filter_speed = new Kalman(1, 0.001f, 0.0001f,0.003f, 0.5f);


    gimbal_motor[GIMBAL_SECOND_YAW_MOTOR] = new GM6020(CAN2, SECOND_GIMBAL_YAW_MOTOR_ID, GIMBAL_MOTOR_REDUCTION_RATIO);
    gimbal_motor[GIMBAL_SECOND_YAW_MOTOR]->m_angle_td = new Adrc_TD(20000, 0.001, 0.001,0);
    gimbal_motor[GIMBAL_SECOND_YAW_MOTOR]->m_angle_pid = new Pid(60, 0.025, 0, 10, 3000, 3500, 1000, 1500);
    gimbal_motor[GIMBAL_SECOND_YAW_MOTOR]->m_speed_pid = new Pid(50, 0.00, 0, 10, 28000, 28000, 20000, 20000);
    gimbal_motor[GIMBAL_SECOND_YAW_MOTOR]->m_encoder = new AbsEncoder(SECOND_GIMBAL_YAW_ENCODER_ZERO_VALUE, ENCODER_RESOLUTION);
    gimbal_motor[GIMBAL_SECOND_YAW_MOTOR]->m_kalman_filter_angle = new Kalman(1, 0.001f, 0.0001f,0.1f, 0.01f);
    gimbal_motor[GIMBAL_SECOND_YAW_MOTOR]->m_kalman_filter_speed = new Kalman(1, 0.001f, 0.0001f,0.003f, 0.5f);
}


/**
 *@brief Initial all sensors of the Sentry robot
 * 
 *@param 
*/
void SentryRobot::InitAllSensors(void)
{
    capacitor = new Capacitor();
    capacitor->m_power_control_pid = new Pid(1,0.02,0,10,10,10,10,0);
}



/**
 *@brief execute the sentry robot chassis control algorithm
 * 
 *@param 
*/
void SentryRobot::ExecuteChassisAlgorithm(void)
{
    if (chassis_mode != CHASSIS_SAFE && chassis_mode != CHASSIS_CALIBRATE) {
        #ifndef CHASSIS_COMMOM_DRIVING_MODE
        for (int i = 0; i < CHASSIS_STEER_MOTOR_NUM; i++) {
            chassis_steer_motor[i]->AngleControl();
        }
        #else
        for (int i = 0; i < CHASSIS_LINE_MOTOR_NUM; i++) {
            chassis_motor[i]->SpeedControl();
        }
        #endif

        #ifdef CHASSIS_STEER_DRIVING_MODE
        if(m_robot_chassis_spin_ready){
            for (int i = 0; i < CHASSIS_LINE_MOTOR_NUM; i++) {
                chassis_line_motor[i]->SpeedTDControl();
            }
        }else{
            for (int i = 0; i < CHASSIS_LINE_MOTOR_NUM; i++) {
                chassis_line_motor[i]->SpeedControl();
                chassis_line_motor[i]->m_speed_td->m_x1 = chassis_line_motor[i]->m_speed_current;
                chassis_line_motor[i]->m_speed_td->m_x2 = 0.0;
            }
        }
        #endif
        #ifdef CHASSIS_STANDARD_DRIVING_MODE
        for (int i = 0; i < CHASSIS_LINE_MOTOR_NUM; i++) {
                chassis_line_motor[i]->SpeedControl();
        }
        #endif
        Power_Control();
    }
}



/**
 *@brief execute the sentry robot gimbal control algorithm
 * 
 *@param 
*/
void SentryRobot::ExecuteGimbalAlgorithm(void)
{
    if (gimbal_mode != GIMBAL_SAFE || chassis_mode == CHASSIS_MANNAL) {
        for (int i = 0; i < GIMBAL_MOTOR_NUM; i++) {
            gimbal_motor[i]->AngleControl();
        }
    }
}



/**
 *@brief execute the sentry robot shoot control algorithm
 * 
 *@param 
*/
void SentryRobot::ExecuteShootAlgorithm(void)
{
    if (shoot_mode != SHOOT_SAFE) {
        shoot_motor[SentryRobot::SHOOT_DRIVE_MOTOR]->AngleControl();
    } else {
        shoot_motor[SentryRobot::SHOOT_DRIVE_MOTOR]->m_speed_pid->m_output = 0;
    }
}



/**
 *@brief Set the sentry robot chassis line wheel speed to the specified speed
 * 
 *@param 
*/
void SentryRobot::SetChassisSpeedTarget(float fll_motor, float bll_motor, float frl_motor, float brl_motor)
{
    chassis_line_motor[CHASSIS_FLL_MOTOR]->m_speed_target = fll_motor;
    chassis_line_motor[CHASSIS_BLL_MOTOR]->m_speed_target = bll_motor;
    chassis_line_motor[CHASSIS_FRL_MOTOR]->m_speed_target = frl_motor;
    chassis_line_motor[CHASSIS_BRL_MOTOR]->m_speed_target = brl_motor;
}



/**
 *@brief Set the sentry robot chassis angle speed to the specified speed
 * 
 *@param 
*/
void SentryRobot::SetChassisAngleSpeedTarget(float fla_motor, float bla_motor, float fra_motor, float bra_motor)
{
    chassis_steer_motor[CHASSIS_FLA_MOTOR]->m_speed_target = fla_motor;
    chassis_steer_motor[CHASSIS_BLA_MOTOR]->m_speed_target = bla_motor;
    chassis_steer_motor[CHASSIS_FRA_MOTOR]->m_speed_target = fra_motor;
    chassis_steer_motor[CHASSIS_BRA_MOTOR]->m_speed_target = bra_motor;
}



/**
 *@brief Set the sentry robot chassis steer angle to the specified angle
 * 
 *@param 
*/
void SentryRobot::SetChassisAngleTarget(float fla_steer, float bla_steer, float fra_steer, float bra_steer)
{
    chassis_steer_motor[CHASSIS_FLA_MOTOR]->m_angle_target = fla_steer;
    chassis_steer_motor[CHASSIS_BLA_MOTOR]->m_angle_target = bla_steer;
    chassis_steer_motor[CHASSIS_FRA_MOTOR]->m_angle_target = fra_steer;
    chassis_steer_motor[CHASSIS_BRA_MOTOR]->m_angle_target = bra_steer;
}


/**
 *@brief Set the sentry robot chassis steer angle to the nearest spin angle
 * 
 *@param 
*/
void SentryRobot::SetChassisSpinAngleTarget()
{
#ifdef CHASSIS_STEER_DRIVING_MODE
    if(!m_robot_chassis_spin_ready){
        float spin_step;
        spin_step = (((int)(chassis_steer_motor[CHASSIS_FLA_MOTOR]->m_angle_current) / 360) * 360.0 - SPIN_ANGLE) - chassis_steer_motor[CHASSIS_FLA_MOTOR]->m_angle_target;
        spin_step = (spin_step > 180.0)?(spin_step-360.0):(spin_step < -180.0)?(spin_step+360.0):(spin_step);
        chassis_steer_motor[CHASSIS_FLA_MOTOR]->m_angle_target += spin_step;
        spin_step = (((int)(chassis_steer_motor[CHASSIS_FRA_MOTOR]->m_angle_current) / 360) * 360.0 + SPIN_ANGLE) - chassis_steer_motor[CHASSIS_FRA_MOTOR]->m_angle_target;
        spin_step = (spin_step > 180.0)?(spin_step-360.0):(spin_step < -180.0)?(spin_step+360.0):(spin_step);
        chassis_steer_motor[CHASSIS_FRA_MOTOR]->m_angle_target += spin_step;
        spin_step = (((int)(chassis_steer_motor[CHASSIS_BRA_MOTOR]->m_angle_current) / 360) * 360.0 - SPIN_ANGLE) - chassis_steer_motor[CHASSIS_BRA_MOTOR]->m_angle_target;
        spin_step = (spin_step > 180.0)?(spin_step-360.0):(spin_step < -180.0)?(spin_step+360.0):(spin_step);
        chassis_steer_motor[CHASSIS_BRA_MOTOR]->m_angle_target += spin_step;
        spin_step = (((int)(chassis_steer_motor[CHASSIS_BLA_MOTOR]->m_angle_current) / 360) * 360.0 + SPIN_ANGLE) - chassis_steer_motor[CHASSIS_BLA_MOTOR]->m_angle_target;
        spin_step = (spin_step > 180.0)?(spin_step-360.0):(spin_step < -180.0)?(spin_step+360.0):(spin_step);
        chassis_steer_motor[CHASSIS_BLA_MOTOR]->m_angle_target += spin_step;
        m_robot_chassis_spin_ready = true;
    }
#endif
}

/**
 *@brief Set the sentry robot gimbal angle to the specified angle
 * 
 *@param 
*/
void SentryRobot::SetGimbalAngleTarget(float f_yaw, float f_pitch, float s_yaw)
{
    gimbal_motor[GIMBAL_FIRST_YAW_MOTOR]->m_angle_target = f_yaw;
    gimbal_motor[GIMBAL_FIRST_PITCH_MOTOR]->m_angle_target = f_pitch;
    gimbal_motor[GIMBAL_SECOND_YAW_MOTOR]->m_angle_target = s_yaw;
}



/**
 *@brief Send control command to all actuators
 * 
 *@param 
*/
void SentryRobot::SendControlCommand(void)
{
    static uint16_t trans_cnt = 0;//待上电后电平稳定再执行发送函数

    // CAN1 ID 0x200    CAN2 ID 0x200
    can1_context.CANx_TxMsg.StdId = 0x200;
    can2_context.CANx_TxMsg.StdId = 0x200;
    for (int i = 0; i < 8; i++) {
        can1_context.CANx_TxMsg.Data[i] = 0;
        can2_context.CANx_TxMsg.Data[i] = 0;
    }
    if (chassis_mode != CHASSIS_SAFE && chassis_mode != CHASSIS_CALIBRATE) {
        for (int i = 0; i < CHASSIS_LINE_MOTOR_NUM; i++) {
            uint32_t id = chassis_line_motor[i]->m_id;
            int16_t cmd = (int16_t)chassis_line_motor[i]->m_speed_pid->m_output;
            if ( cmd >= WHEEL_CURRENT_DEAD_AREA ) {
                cmd += FORWARD_DAMP_FACTOR;
            } else if ( cmd < -WHEEL_CURRENT_DEAD_AREA ){
                cmd -= FORWARD_DAMP_FACTOR;
            }
            if (id >= 1 && id <= 4) {
                if (chassis_line_motor[i]->m_CANx == CAN1) {
                    can1_context.CANx_TxMsg.Data[(id - 1) * 2] = (uint8_t)(cmd >> 8);
                    can1_context.CANx_TxMsg.Data[(id - 1) * 2 + 1] = (uint8_t)(cmd);
                } else if (chassis_line_motor[i]->m_CANx == CAN2) {
                    can2_context.CANx_TxMsg.Data[(id - 1) * 2] = (uint8_t)(cmd >> 8);
                    can2_context.CANx_TxMsg.Data[(id - 1) * 2 + 1] = (uint8_t)(cmd);
                }
            }
        }

        // Forward wheel damp test
        // for (int i = 4; i < CHASSIS_LINE_MOTOR_NUM; i++) {
        //     uint32_t id = chassis_motor[i]->m_id;
        //     int16_t cmd;
        //     if (i == 4 || i == 7) {
        //         cmd = 800;
        //     } else {
        //         cmd = -800;
        //     }
        //     if (id >= 1 && id <= 4) {
        //         if (chassis_line_motor[i]->m_hcan == &hcan1) {
        //             can1_context.tx_data[(id - 1) * 2] = (uint8_t)(cmd >> 8);
        //             can1_context.tx_data[(id - 1) * 2 + 1] = (uint8_t)(cmd);
        //         } else if (chassis_line_motor[i]->m_hcan == &hcan2) {
        //             can2_context.tx_data[(id - 1) * 2] = (uint8_t)(cmd >> 8);
        //             can2_context.tx_data[(id - 1) * 2 + 1] = (uint8_t)(cmd);
        //         }
        //     }
        // }
    } 
    
    for (int i = 0; i < SHOOT_MOTOR_NUM; i++) {
        uint32_t id = shoot_motor[i]->m_id;
        int16_t cmd = (int16_t)shoot_motor[i]->m_speed_pid->m_output;
        if (id >= 1 && id <= 4) {
            if (shoot_motor[i]->m_CANx == CAN1) {
                can1_context.CANx_TxMsg.Data[(id - 1) * 2] = (uint8_t)(cmd >> 8);
                can1_context.CANx_TxMsg.Data[(id - 1) * 2 + 1] = (uint8_t)(cmd);
            } else if (shoot_motor[i]->m_CANx == CAN2) {
                can2_context.CANx_TxMsg.Data[(id - 1) * 2] = (uint8_t)(cmd >> 8);
                can2_context.CANx_TxMsg.Data[(id - 1) * 2 + 1] = (uint8_t)(cmd);
            }
        }
    }
    if(trans_cnt > 2000){
        can1_context.CanSendMessage();
        can2_context.CanSendMessage();
    }

    // CAN1 ID 0x1ff    CAN2 ID 0x1ff
    can1_context.CANx_TxMsg.StdId = 0x1ff;
    can2_context.CANx_TxMsg.StdId = 0x1ff;
    for (int i = 0; i < 8; i++) {
        can1_context.CANx_TxMsg.Data[i] = 0;
        can2_context.CANx_TxMsg.Data[i] = 0;
    }

    if (gimbal_mode != GIMBAL_SAFE ) {
        uint32_t id = gimbal_motor[GIMBAL_FIRST_YAW_MOTOR]->m_id;
        int16_t cmd = (int16_t)gimbal_motor[GIMBAL_FIRST_YAW_MOTOR]->m_speed_pid->m_output;
        if (id >= 1 && id <= 4) {
            if (gimbal_motor[GIMBAL_FIRST_YAW_MOTOR]->m_CANx == CAN1) {
                can1_context.CANx_TxMsg.Data[(id - 1) * 2] = (uint8_t)(cmd >> 8);
                can1_context.CANx_TxMsg.Data[(id - 1) * 2 + 1] = (uint8_t)(cmd);
            } else if (gimbal_motor[GIMBAL_FIRST_YAW_MOTOR]->m_CANx == CAN2) {
                can2_context.CANx_TxMsg.Data[(id - 1) * 2] = (uint8_t)(cmd >> 8);
                can2_context.CANx_TxMsg.Data[(id - 1) * 2 + 1] = (uint8_t)(cmd);
            }
        }
    }


    if (!shoot_insurance) {
        uint32_t id = shoot_motor[SHOOT_DRIVE_MOTOR]->m_id;
        int16_t cmd = (int16_t)shoot_motor[SHOOT_DRIVE_MOTOR]->m_speed_pid->m_output;
        if (id >= 1 && id <= 4) {
            if (shoot_motor[SHOOT_DRIVE_MOTOR]->m_CANx == CAN1) {
                can1_context.CANx_TxMsg.Data[(id - 1) * 2] = (uint8_t)(cmd >> 8);
                can1_context.CANx_TxMsg.Data[(id - 1) * 2 + 1] = (uint8_t)(cmd);
            } else if (shoot_motor[SHOOT_DRIVE_MOTOR]->m_CANx == CAN2) {
                can2_context.CANx_TxMsg.Data[(id - 1) * 2] = (uint8_t)(cmd >> 8);
                can2_context.CANx_TxMsg.Data[(id - 1) * 2 + 1] = (uint8_t)(cmd);
            }
        } else if (id >= 5 && id <= 8){
            if (shoot_motor[SHOOT_DRIVE_MOTOR]->m_CANx == CAN1) {
                can1_context.CANx_TxMsg.Data[(id - 5) * 2] = (uint8_t)(cmd >> 8);
                can1_context.CANx_TxMsg.Data[(id - 5) * 2 + 1] = (uint8_t)(cmd);
            } else if (shoot_motor[SHOOT_DRIVE_MOTOR]->m_CANx == CAN2) {
                can2_context.CANx_TxMsg.Data[(id - 5) * 2] = (uint8_t)(cmd >> 8);
                can2_context.CANx_TxMsg.Data[(id - 5) * 2 + 1] = (uint8_t)(cmd);
            }
        }
    }

    if (chassis_mode != CHASSIS_SAFE && chassis_mode != CHASSIS_CALIBRATE) {
        for (int i = 0; i < CHASSIS_STEER_MOTOR_NUM; i++) {
            uint32_t id = chassis_steer_motor[i]->m_id;
            int16_t cmd = -(int16_t)chassis_steer_motor[i]->m_speed_pid->m_output;
            if (id >= 1 && id <= 4) {
                if (chassis_steer_motor[i]->m_CANx == CAN1) {
                    can1_context.CANx_TxMsg.Data[(id - 1) * 2] = (uint8_t)(cmd >> 8);
                    can1_context.CANx_TxMsg.Data[(id - 1) * 2 + 1] = (uint8_t)(cmd);
                } else if (chassis_steer_motor[i]->m_CANx == CAN2) {
                    can2_context.CANx_TxMsg.Data[(id - 1) * 2] = (uint8_t)(cmd >> 8);
                    can2_context.CANx_TxMsg.Data[(id - 1) * 2 + 1] = (uint8_t)(cmd);
                }
            }
        }
    }

    // Capacitor power charging target
    
    if(G_control_mode == SAFE) capacitor->CapVolLimit();
    int16_t cmd = (int16_t)capacitor->m_power_charging * 100;
    can2_context.CANx_TxMsg.Data[2] = (uint8_t)( cmd >> 8 );
    can2_context.CANx_TxMsg.Data[3] = (uint8_t)(cmd);
    
    if(trans_cnt > 2000){
        can1_context.CanSendMessage();
        can2_context.CanSendMessage();
    }

    trans_cnt++;
    if(trans_cnt > 2500) trans_cnt=2500;
}



/**
 *@brief update the first gimbal pitch motor current angle and current speed
 * 
 *@param 
*/
void SentryRobot::UpdateGimbalFirstPitchState(float angle, float speed)
{
    gimbal_motor[GIMBAL_FIRST_PITCH_MOTOR]->m_angle_current = angle;
    gimbal_motor[GIMBAL_FIRST_PITCH_MOTOR]->m_speed_current = speed;
}



/**
 *@brief update the second gimbal yaw motor current angle and current speed
 * 
 *@param 
*/
void SentryRobot::UpdateGimbalSecondYawState(float angle, float speed)
{
    gimbal_motor[GIMBAL_SECOND_YAW_MOTOR]->m_angle_current = angle;
    gimbal_motor[GIMBAL_SECOND_YAW_MOTOR]->m_speed_current = speed;
}



/**
 *@brief update the first gimbal yaw motor current angle and current speed
 * 
 *@param 
*/
void SentryRobot::UpdateGimbalFirstYawState(float angle, float speed)
{
    gimbal_motor[GIMBAL_FIRST_YAW_MOTOR]->m_angle_current = angle;
    gimbal_motor[GIMBAL_FIRST_YAW_MOTOR]->m_speed_current = speed;
}



/**
 *@brief Increase shoot bullet count target
 * 
 *@param 
*/
void SentryRobot::IncreaseShootBulletTarget(uint32_t time_now) {
    // if(m_shoot_bullet_fps == SHOOT_BULLET_FREQUENCY_TEST_0)
    // {
    //     if (time_now - m_shoot_bullet_last_increase_time > 1100) {
    //         Global::sentry->shoot_driver_angle_target = Global::sentry->shoot_motor[SentryRobot::SHOOT_DRIVE_MOTOR]->m_angle_current;
    //         m_shoot_bullet_cnt_target--;
    //         m_shoot_bullet_last_increase_time = time_now;
    //     }
    // }
    // else
    // {
        if (time_now - m_shoot_bullet_last_increase_time > (1000 / m_shoot_bullet_fps)) {
            m_shoot_bullet_cnt_target++;
            m_shoot_bullet_last_increase_time = time_now;
        }
//     }

}



/**
 *@brief Set the shoot drive motor position target
 * 
 *@param 
*/
void SentryRobot::SetShootDriverTarget(float angle) {
    shoot_motor[SHOOT_DRIVE_MOTOR]->m_angle_target = angle;
}


void SentryRobot::Power_Control(void)
{
   if(capacitor->CAP_STATE == 0)
   {
       capacitor->m_power_control_pid->m_error_sum=0;
       capacitor->m_power_control_pid->m_error_pre=0;
   }

    if((capacitor->Pre_CAP_STATE == 1) && (capacitor->CAP_STATE==0))
    {
        for(uint8_t i = 0; i < CHASSIS_LINE_MOTOR_NUM; i++)
        {
            chassis_line_motor[i]->m_speed_pid->m_error_sum = 0;
        }
    }

    if(capacitor->CAP_STATE == 1)
    {
        if(capacitor->CAP_POW_USE > 0.5)
        {
            if(capacitor->CAP_POW_USE > 0.9 || capacitor->FLAG_CAP_Pow == 1)
            {
                capacitor->m_power_control_pid->CalWeakenPID(-log(capacitor->CAP_POW_USE)/log(2.7183));
                capacitor->CAP_Cur_Coe = pow(2.7183,(double)capacitor->m_power_control_pid->m_output);
                capacitor->FLAG_CAP_Pow = 1;
                for(uint8_t i = 0; i < CHASSIS_LINE_MOTOR_NUM; i++)
                {
                    chassis_line_motor[i]->m_speed_pid->m_output *= capacitor->CAP_Cur_Coe;
                }
            }
        }
        else
        {
            capacitor->FLAG_CAP_Pow = 0;
            capacitor->m_power_control_pid->m_error_pre=0;
            capacitor->m_power_control_pid->m_error_sum=0;
        }
    }

    if(capacitor->CAP_STATE == 2)
    {
        capacitor->m_power_control_pid->m_error_pre=0;
        capacitor->m_power_control_pid->m_error_sum=0;

         for(uint8_t i = 0; i < CHASSIS_LINE_MOTOR_NUM; i++)
        {
            chassis_line_motor[i]->m_speed_pid->m_output = 0;
        }
    }

    capacitor->Pre_CAP_STATE = capacitor->CAP_STATE;
}


