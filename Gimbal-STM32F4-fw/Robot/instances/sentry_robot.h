#pragma once
#include "m3508.h"
#include "gm6020.h"
#include "m2006.h"
#include "bmi088.h"

// The encoder value corresponding to mechanical zero of the gimbal motor
#define FIRST_GIMBAL_PITCH_ENCODER_ZERO_VALUE    ( (uint32_t) 4783 )
#define SECOND_GIMBAL_YAW_ENCODER_ZERO_VALUE    ( (uint32_t) 2016 )

// The encoder value corresponding to mechanical zero of the fric motor
#define LEFT_FRIC_ENCODER_ZERO_VALUE            ( (uint32_t) 123 )
#define RIGHT_FRIC_ENCODER_ZERO_VALUE           ( (uint32_t) 234 )

// The resolution of encoder
#define ENCODER_RESOLUTION ( (uint32_t) 8192)

// The reduction ratio of fric motor
#define FRIC_MOTOR_REDUCTION_RATIO ( (uint32_t) 1 )

// The reduction ratio of gimbal motor
#define GIMBAL_MOTOR_REDUCTION_RATIO ( (uint32_t) 1 )

// Motor ID(same type motors should have different ID)
#define GIMBAL_FIRST_PITCH_MOTOR_ID             ( (uint8_t) 2 )
#define GIMBAL_SECOND_YAW_MOTOR_ID              ( (uint8_t) 4 )
#define LEFT_FRIC_MOTOR_ID                      ( (uint8_t) 2 )
#define RIGHT_FRIC_MOTOR_ID                     ( (uint8_t) 1 )

// Motor number
#define SHOOT_MOTOR_NUM ( (uint8_t) 2 )
#define GIMBAL_MOTOR_NUM ( (uint8_t) 2 )

// Robot Initial State
#define CHASSIS_INIT_SPEED ( (float) 0.0f )
#define CHASSIS_INIT_ANGLE ( (float) 0.0f )
#define CHASSIS_CALIBRATE_ENCODE_RANGE ( (uint32_t) 200 )
#define GIMBAL_YAW_INIT_ANGLE ( (float) 20.0f )

// Robot mannal control sensitivity
#define CHASSIS_SPEED_MANNAL_CONTROL_SENSITIVITY            ( (float) 100.0f )
#define CHASSIS_ANGLE_MANNAL_CONTROL_SENSITIVITY            ( (float) 100.0f )
#define GIMBAL_PITCH_ANGLE_MANNAL_CONTROL_SENSITIVITY       ( (float) 100.0f )
#define GIMBAL_YAW_ANGLE_MANNAL_CONTROL_SENSITIVITY         ( (float) 100.0f )

// Chassis driving mode
#define CHASSIS_STEER_DRIVING_MODE
// #define CHASSIS_DIFFERENTIAL_DRIVING_MODE            
// #define CHASSIS_COMMOM_DRIVING_MODE                                    

#ifdef CHASSIS_STEER_DRIVING_MODE
#define CHASSIS_STEER_ANGLE_MAX                             ( (float)(90) )
#define CHASSIS_LINE_SPEED_MAX                              ( (float)(10) )
#define CHASSIS_STEER_ANGLE_SENSITIVITY                     ( (float)(0.1) )
#endif

#ifdef CHASSIS_DIFFERENTIAL_DRIVING_MODE
#define CHASSIS_ANGLE_SPEED_MAX                              ( (float)(300) )
#define CHASSIS_LINE_SPEED_MAX                               ( (float)(20) )
#endif

#ifdef CHASSIS_COMMOM_DRIVING_MODE
#define CHASSIS_LINE_SPEED_MAX                              ( (float)(10) )
#define CHASSIS_ANGLE_SPEED_MAX                              ( (float)(300) )

#endif 

#define CHASSIS_HALF_WHEEL_TREAD                            ( (float)(0.4f) )

#define CHASSIS_WHEEL_RADIUS                                (float)(0.06f)


// IMU number
#define BMI088_IMU_NUM ( (uint8_t) 1 )

#define IMU_CALIBRATE                                       ((bool)false)

#define FRIC_WHEEL_SPEED                ((float)6800.0f)
#define FRIC_WHEEL_SPEED_SAFE           ((float)6500.0f)

#define FIRST_FITCH_SPEED_IMU_FEEDBACK
// #define FIRST_FITCH_SPEED_ENCODER_FEEDBACK
// #define FIRST_FITCH_ANGLE_IMU_FEEDBACK
#define FIRST_FITCH_ANGLE_ENCODER_FEEDBACK



class SentryRobot {
public:
    enum GimbalMode {
        GIMBAL_SAFE = 0,
        GIMBAL_MANNAL,
        GIMBAL_AUTO
    };
    GimbalMode gimbal_mode;

    enum ShootMode {
        SHOOT_SAFE = 0,
        SHOOT_MANNAL,
        SHOOT_AUTO
    };
    ShootMode shoot_mode;

    enum ShootMotor {
        LEFT_FRIC_MOTOR = 0,
        RIGHT_FRIC_MOTOR,
    };
    M3508* shoot_motor[SHOOT_MOTOR_NUM];

    enum GimbalMotor {
        GIMBAL_FIRST_PITCH_MOTOR = 0,
        GIMBAL_SECOND_YAW_MOTOR
    };
    GM6020* gimbal_motor[GIMBAL_MOTOR_NUM];

    enum BMI088IMU {
        GIMBAL_FIRST_IMU = 0,
    };
    BMI088* gimbal_imu[BMI088_IMU_NUM];


    void Init(void);

    void MoveShoot(void);
    void MoveGimbal(void);
    void SetFirstGimbalPitchAngleTarget(float target);
    void SetSecondGimbalYawAngleTarget(float target);
    void SetFricWheelSpeedTarget(float l_target, float r_target);
    inline void SetGimbalMode(GimbalMode mode) {gimbal_mode = mode;};
    inline void SetShootMode(ShootMode mode) {shoot_mode = mode;};

    void SendControlCommand(void);

    SentryRobot(){};
    ~SentryRobot(){};

private:
    void InitAllActuators(void);
    void InitAllSensors(void);
};