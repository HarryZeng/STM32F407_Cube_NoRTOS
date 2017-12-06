#ifndef __CONTACT_H
#define __CONTACT_H

#include "stm32f4xx_hal.h"
#include "main.h"

#include "PID.h"
#include "encoder.h"

#include "math.h"
#include <stdio.h>
#include "cstring"


void LeftMovingSpeedW(unsigned int val);//左轮方向和速度控制函数
void RightMovingSpeedW(unsigned int val2);//右轮方向和速度控制函数

void car_control(float rightspeed,float leftspeed);//小车速度转化和控制函数

//void Contact_Init(void);//左右轮方向和速度初始化

#endif  
