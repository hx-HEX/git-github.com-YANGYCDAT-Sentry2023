#pragma once
#include "m3508.h"
#include "gm6020.h"
#include "m2006.h"
#include "RoboMaster.h"
#include "capacitor.h"
#include "kalman.h"
#include "ewma.h"

// The encoder value corresponding to mechanical zero of the gimbal motor
#define FIRST_GIMBAL_YAW_ENCODER_ZERO_VALUE ( (int32_t) 8185 )
#define FIRST_GIMBAL_PITCH_ENCODER_ZERO_VALUE ( (int32_t) 234 )
#define SECOND_GIMBAL_YAW_ENCODER_ZERO_VALUE ( (int32_t) 2016 )

// The encoder value corresponding to mechanical zero of the chassis motor
#define CHASSIS_FRA_ENCODER_ZERO_VALUE ( (int32_t) 399)
#define CHASSIS_FLA_ENCODER_ZERO_VALUE ( (int32_t) 6416)
#define CHASSIS_BLA_ENCODER_ZERO_VALUE ( (int32_t) 1732 )
#define CHASSIS_BRA_ENCODER_ZERO_VALUE ( (int32_t) 7853)
#define CHASSIS_FLL_ENCODER_ZERO_VALUE ( (int32_t) 789 )
#define CHASSIS_FRL_ENCODER_ZERO_VALUE ( (int32_t) 890 )
#define CHASSIS_BLL_ENCODER_ZERO_VALUE ( (int32_t) 901 )
#define CHASSIS_BRL_ENCODER_ZERO_VALUE ( (int32_t) 1234 )

// The resolution of encoder
#define ENCODER_RESOLUTION ( (uint32_t) 8192)

// The reduction ratio of chassis motor
#define CHASSIS_STEER_MOTOR_REDUCTION_RATIO ( (uint32_t) 1 )
#define CHASSIS_SPEED_MOTOR_REDUCTION_RATIO ( (uint32_t) 19 )

// The reduction ratio of gimbal motor
#define GIMBAL_MOTOR_REDUCTION_RATIO ( (uint32_t) 1 )

// The reduction ratio of shoot motor
#define SHOOT_MOTOR_REDUCTION_RATIO ( (uint32_t) 36 )

// Motor ID(same type motors should have different ID)
#define SHOOT_DRIVE_MOTOR_ID ( (uint8_t) 7 )
#define FIRST_GIMBAL_YAW_MOTOR_ID ( (uint8_t) 1 )
#define FIRST_GIMBAL_PITCH_MOTOR_ID ( (uint8_t) 2 )
#define SECOND_GIMBAL_YAW_MOTOR_ID ( (uint8_t) 1 )
#define CHASSIS_FRA_MOTOR_ID ( (uint8_t) 1 )
#define CHASSIS_FRL_MOTOR_ID ( (uint8_t) 1)
#define CHASSIS_BRL_MOTOR_ID ( (uint8_t) 2 )
#define CHASSIS_BRA_MOTOR_ID ( (uint8_t) 2 )
#define CHASSIS_BLA_MOTOR_ID ( (uint8_t) 3 )
#define CHASSIS_BLL_MOTOR_ID ( (uint8_t) 3 )
#define CHASSIS_FLL_MOTOR_ID ( (uint8_t) 4 )
#define CHASSIS_FLA_MOTOR_ID ( (uint8_t) 4 )

// Motor number
#define CHASSIS_STEER_MOTOR_NUM ( (uint8_t) 4)
#define CHASSIS_LINE_MOTOR_NUM ( (uint8_t) 4 )
#define GIMBAL_MOTOR_NUM ( (uint8_t) 3 )
#define SHOOT_MOTOR_NUM ( (uint8_t) 1 )

// Robot Initial State
#define CHASSIS_INIT_SPEED ( (float) 0.0f )
#define CHASSIS_INIT_ANGLE ( (float) 0.0f )
#define CHASSIS_CALIBRATE_ENCODE_RANGE ( (uint32_t) 200 )
#define GIMBAL_YAW_INIT_ANGLE ( (float) 20.0f )

// Robot mannal control sensitivity
#define CHASSIS_SPEED_MANNAL_CONTROL_SENSITIVITY            ( (float) 100.0f )
#define CHASSIS_ANGLE_MANNAL_CONTROL_SENSITIVITY            ( (float) 100.0f )
#define GIMBAL_PITCH_ANGLE_MANNAL_CONTROL_SENSITIVITY       ( (float) 0.05f )
#define GIMBAL_YAW_ANGLE_MANNAL_CONTROL_SENSITIVITY         ( (float) 0.05f )
#define GIMBAL_PITCH_ANGLE_SCAN_SPEED                       ( (float) 0.04f )
#define GIMBAL_YAW_ANGLE_SCAN_SPEED                         ( (float) 0.1f )
#define GIMBAL_PITCH_ANGLE_SCAN_ANGLE_MIN                   ( (float) -10.0f )
#define GIMBAL_PITCH_ANGLE_SCAN_ANGLE_MAX                   ( (float) 8.0f )
#define GIMBAL_YAW_ANGLE_SCAN_ANGLE_MIN                     ( (float) -180.0f )
#define GIMBAL_YAW_ANGLE_SCAN_ANGLE_MAX                     ( (float) 180.0f )
#define GIMBAL_PITCH_MAX                                    ( (float) 20.0f )
#define GIMBAL_PITCH_MIN                                    ( (float) -10.0f )


// Chassis driving mode
// #define CHASSIS_STEER_DRIVING_MODE
#define CHASSIS_STANDARD_DRIVING_MODE
// #define CHASSIS_DIFFERENTIAL_DRIVING_MODE            
// #define CHASSIS_COMMOM_DRIVING_MODE                                    

#ifdef CHASSIS_STEER_DRIVING_MODE
#define CHASSIS_LINE_SPEED_MAX                              ( (float)(2.8) )
#define CHASSIS_SPIN_SPEED_MAX                              ( (float)(2.25) )
#define CHASSIS_SPIN_OVER_LIMIT                             ( (float)(0.05) )
#define CHASSIS_STEER_ANGLE_SENSITIVITY                     ( (float)(0.3) )
#define CHASSIS_LINE_SHOOT_SPEED_MAX                        ( (float) (1.0) )
#endif

#ifdef CHASSIS_STANDARD_DRIVING_MODE
#define CHASSIS_MANNAL_LINE_SPEED_MAX                              ( (float)(2.0) )
#define CHASSIS_MANNAL_SPIN_SPEED_MAX                                ( (float)(1.0) )
#define CHASSIS_SPIN_OVER_LIMIT                             ( (float)(0.05) )
#define CHASSIS_LINE_SPEED_MAX                              ( (float)(2.8) )
#define CHASSIS_SPIN_SPEED_MAX                              ( (float)(1.0) )
#endif

#ifdef CHASSIS_DIFFERENTIAL_DRIVING_MODE
#define CHASSIS_ANGLE_SPEED_MAX                              ( (float)(300) )
#define CHASSIS_LINE_SPEED_MAX                               ( (float)(2) )
#endif

#ifdef CHASSIS_COMMOM_DRIVING_MODE
#define CHASSIS_LINE_SPEED_MAX                              ( (float)(2) )
#define CHASSIS_ANGLE_SPEED_MAX                              ( (float)(300) )

#endif 

#define CHASSIS_HALF_WHEEL_TREAD                            ( (float)(0.4f) )

#define CHASSIS_WHEEL_RADIUS                                (float)(0.07f)
    
#define RIDAR_FIXED_ANGLE_ERROR                            (float)(0.0f)

#define SPIN_ANGLE                                      (float)(45.0f)

#define SEC_GIMBAL_DES_WORLD_ANGLE                               (float)(90.0f)

#define SHOOT_BULLET_FREQUENCY_TEST_0   0
#define SHOOT_BULLET_FREQUENCY_TEST_1   1
#define SHOOT_BULLET_FREQUENCY_TEST_2   8
#define SHOOT_BULLET_FREQUENCY_TEST_3   13
#define SHOOT_BULLET_FREQUENCY_TEST_4   17

#define SHOOT_DRIVER_SUPPLY_ANGLE_STEP  ((float)(29491 * 5 / 3))

#define SHOOT_SPEED_FILTER_BETA ((float)0.9)

#define FIRST_GIMBAL_YAW_IMU_FEED_BACK                             
// #define FIRST_GIMBAL_YAW_ENCODER_FEED_BACK                         

#define G_COMPENSATION ((float)3000.0f)
#define FRIC_WHEEL_SPEED                ((uint16_t)14500)


#define FORWARD_DAMP_FACTOR                                     ((float)(800))
#define WHEEL_CURRENT_DEAD_AREA                                   ((float)(1000))

#define Pitch_Vison_Error                          ((float) 1.0f)
#define Yaw_Vision_Error                           ((float) 2.0f)

#define PRESS_C                                       67
#define PRESS_F                                       70
#define PRESS_G                                       71
#define PRESS_V                                       86                                      
#define PRESS_X                                       88
#define PRESS_Z                                       90




class SentryRobot {
public:
    enum ChassisMode {
        CHASSIS_SAFE = 0,
        CHASSIS_CALIBRATE,
        CHASSIS_MANNAL,
        CHASSIS_AUTO
    };
    ChassisMode chassis_mode;
    ChassisMode chassis_mode_pre;

    enum GimbalMode {
        GIMBAL_SAFE = 0,
        GIMBAL_MANNAL,
        GIMBAL_AUTO
    };
    GimbalMode gimbal_mode;
    GimbalMode gimbal_mode_pre;

    enum ShootMode {
        SHOOT_SAFE = 0,
        SHOOT_MANNAL,
        SHOOT_AUTO
    };
    ShootMode shoot_mode;
    ShootMode shoot_mode_pre;

    enum ChassisAngleMotor {
        CHASSIS_FRA_MOTOR = 0,
        CHASSIS_FLA_MOTOR,
        CHASSIS_BLA_MOTOR,
        CHASSIS_BRA_MOTOR
    };
    enum ChassisLineMotor {
        CHASSIS_FRL_MOTOR = 0,
        CHASSIS_FLL_MOTOR,
        CHASSIS_BLL_MOTOR,
        CHASSIS_BRL_MOTOR
    };
    M3508* chassis_line_motor[CHASSIS_LINE_MOTOR_NUM];
    GM6020* chassis_steer_motor[CHASSIS_STEER_MOTOR_NUM];

    enum GimbalMotor {
        GIMBAL_FIRST_YAW_MOTOR = 0,
        GIMBAL_FIRST_PITCH_MOTOR,
        GIMBAL_SECOND_YAW_MOTOR
    };
    GM6020* gimbal_motor[GIMBAL_MOTOR_NUM];

    enum ShootMotor {
        SHOOT_DRIVE_MOTOR = 0
    };
    M2006* shoot_motor[SHOOT_MOTOR_NUM];

    Capacitor* capacitor;

   float m_robot_radar_world_yaw;
    
    bool ctrl_spinning_flag;
    bool spinning_flag;
    bool vision_flag;
    bool safe_flag;

    float m_robot_chassis_world_error;
    float m_robot_wheel_world_yaw;
    float m_robot_chassis_speed;

// steer drive value
#ifdef CHASSIS_STEER_DRIVING_MODE

    float m_robot_wheel_world_yaw_des_sum;
    float m_robot_wheel_world_speed_des;

    bool m_robot_chassis_spin_ready;

#endif
// steer drive value

// standard drive value
#ifdef CHASSIS_STANDARD_DRIVING_MODE
    
    // change speed direction if delta angle > 90 degree
    bool fll_speed_dir;
    bool frl_speed_dir; 
    bool brl_speed_dir;
    bool bll_speed_dir;
    bool standard_driving_init_flag;
    float world_line_speed_angle_des;
    float world_line_speed_angle_pre;
    float fla_angle_des;
    float fla_angle_des_pre;
    float fra_angle_des;
    float fra_angle_des_pre;
    float bra_angle_des;
    float bra_angle_des_pre;
    float bla_angle_des;
    float bla_angle_des_pre;

#endif
// standard drive value
    
    // gimbal and shoot value
    bool shoot_insurance;
    uint32_t m_shoot_bullet_cnt_target;
    uint8_t m_shoot_bullet_fps;
    uint32_t m_shoot_bullet_last_increase_time;
    float shoot_driver_angle_target;

    bool change_gun_flag;
    bool enemy_find_flag;

    uint16_t m_flag_cnt;
    uint16_t m_shoot_cnt;
    EWMA* m_shoot_speed_filter;
    float m_shoot_speed_pre; // ???
    
    bool m_scan_flag;
    bool first_yaw_scan_dir;
    bool first_pitch_scan_dir;
    uint8_t change_yaw_flag;
    uint8_t change_pitch_flag;
    uint8_t change_cnt;
    uint16_t change_delay_cnt;
    float first_yaw_angle_des;
    float first_pitch_angle_des;
    float second_yaw_angle_des;
    // gimbal and shoot value

    uint16_t m_vision_delay_cnt;
    uint8_t direction_flag;

    uint16_t sin_cnt;
    uint8_t second_yaw_step_flag;

    void Init(void);

    void ExecuteGimbalAlgorithm(void);
    void ExecuteChassisAlgorithm(void);
    void ExecuteShootAlgorithm(void);
    void SetChassisSpeedTarget(float fll_motor, float frl_motor, float bll_motor, float brl_motor);
    void SetChassisAngleSpeedTarget(float fla_motor, float bla_motor, float fra_motor, float bra_motor);
    void SetChassisAngleTarget(float fla_steer, float fra_steer, float bla_steer, float bra_steer);
    void SetChassisSpinAngleTarget(void);
    void SetGimbalAngleTarget(float f_yaw, float f_pitch, float s_pitch);
    void SetShootDriverTarget(float angle);
    void UpdateGimbalFirstPitchState(float angle, float speed);
    void UpdateGimbalSecondYawState(float angle, float speed);
    void UpdateGimbalFirstYawState(float angle, float speed);
    void SendControlCommand(void);
    void IncreaseShootBulletTarget(uint32_t time_now);
    inline void SetChassisMode(ChassisMode mode) {chassis_mode_pre = chassis_mode; chassis_mode = mode;};
    inline void SetGimbalMode(GimbalMode mode) {gimbal_mode_pre = gimbal_mode; gimbal_mode = mode;};
    inline void SetShootMode(ShootMode mode) {shoot_mode_pre = shoot_mode; shoot_mode = mode;};
    void Power_Control(void);
    
    SentryRobot(void);
    ~SentryRobot(){};

private:
    void InitAllActuators(void);
    void InitAllSensors(void);
};
