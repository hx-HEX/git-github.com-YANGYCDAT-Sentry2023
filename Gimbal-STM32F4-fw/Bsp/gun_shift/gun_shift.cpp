#include "gun_shift.h"

void GunShift::Shift(float angle)
{
    uint32_t servo_temp;
    servo_temp = angle*200/180+50;
	TIM_SetCompare4(TIM4,servo_temp);
}