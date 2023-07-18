#include "gm6020.h"
#include "can_interface.h"
#include "common_math.h"


#ifndef PI
#define PI 3.14159265358979f
#endif

/**
 * @brief Update the motor speed
 *
 * @param float speed: the input speed of the motor reducer (degree/s)
 */
void GM6020::SpeedUpdate(float speed)
{
    m_speed_current = speed / m_reduction_ratio / 60 * 2 * PI * RADIAN2DEGREE_VALUE;
}



/**
 * @brief Update the motor angle
 *
 * @param uint32_t speed: the feedback encoder value
 */
void GM6020::AngleUpdate(uint32_t value)
{
    m_encoder->EncodeValueUpdate(value);
    m_angle_current = (m_encoder->m_sum_value - m_encoder->m_zero_value) / 
                  m_encoder->m_resolution * 360.0f / m_reduction_ratio;
}



/**
 * @brief Update the motor angle
 *
 * @param float speed: the target motor speed
 */
void GM6020::SpeedControl(void)
{
    m_speed_pid->CalWeakenPID(m_speed_target - m_speed_current);
}



/**
 * @brief Update the motor angle
 *
 */
void GM6020::AngleControl(void)
{
    m_angle_td->CalAdrcTD(m_angle_target);
    m_angle_pid->CalWeakenPID(m_angle_td->m_x1 - m_angle_current);
    m_speed_target = m_angle_pid->m_output;
    m_speed_pid->CalWeakenPID(m_speed_target - m_speed_current);
}