/*
 ******************************************************************************
 *@file    :Chassis.h
 *@brief   :
 *@author  :Cloude Remnant
 *@date    :2019-04-16
 ******************************************************************************
 *@description:
 *
 ******************************************************************************
 */

#ifndef Chassis_H
#define Chassis_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/
//编码器引脚定义
#define ENCODER_L 2 
#define DIRECTION_L 4
#define ENCODER_R 3
#define DIRECTION_R 13
//TB6612FNG驱动模块控制信号 共6个
#define IN1 6   
#define IN2 7
#define IN3 8
#define IN4 9
#define PWMA 5
#define PWMB 10
/* Variable ------------------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/
void Chassis_Init(void);
void Get_velocity(void);
void Set_pwm(int L_pwm,int R_pwm);

#ifdef __cplusplus
}
#endif

#endif // Chassis_H