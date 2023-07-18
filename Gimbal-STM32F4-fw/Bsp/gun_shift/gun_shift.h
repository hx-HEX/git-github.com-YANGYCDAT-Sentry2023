#pragma once
#include "tim.h"
#include "stm32f4xx_tim.h"

#define GUNSHIFT_ANGLE_1 41
#define GUNSHIFT_ANGLE_2 118

class GunShift{
public:
    float m_target_angle = GUNSHIFT_ANGLE_1;
    unsigned char change_flag = 1;
    unsigned char change_gun_flag = 0;

    GunShift(){}

    void Shift(float angle);
private:
};