/*
 ******************************************************************************
 *@file    :Chassis.c
 *@brief   :二轮底盘
 *@author  :Cloude Remnant
 *@date    :2019-04-16
 ******************************************************************************
 *@description:
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include "Chassis.h"
/* Macro ---------------------------------------------------------------------*/
#define MOTOR_PWM_MAX   (200)
#define MOTOR_PWM_DEATH (13)
/* Variable ------------------------------------------------------------------*/
int L_cnt = 0;//编码器计数
int R_cnt = 0;
int L_velocity = 0;//轮子速度
int R_velocity = 0;
/* Function ------------------------------------------------------------------*/
/*
 *@description:编码器中断处理函数//TODO正负，加减，高低电平需要实测
 *@input:无
 *@output:无
*/
void L_encoder()
{
    if (digitalRead(ENCODER_L) == HIGH)
    {
        if (digitalRead(DIRECTION_L) == LOW)
            L_cnt++;
        else
            L_cnt--;
    }
    else
    {
        if (digitalRead(DIRECTION_L) == HIGH)
            L_cnt++;
        else
            L_cnt--;
    }
}

void R_encoder()
{
    if (digitalRead(ENCODER_R) == HIGH)
    {
        if (digitalRead(DIRECTION_R) == LOW)
            R_cnt--;
        else
            R_cnt++;
    }
    else
    {
        if (digitalRead(DIRECTION_R) == HIGH)
            R_cnt--;
        else
            R_cnt++;
    }
}

/*
 *@description:底盘初始化
 *@input:无
 *@output:无
*/
void Chassis_Init(void)
{
    //编码器引脚初始化
    pinMode(ENCODER_L, INPUT);
    pinMode(DIRECTION_L, INPUT);
    pinMode(ENCODER_R, INPUT);
    pinMode(DIRECTION_R, INPUT);

    //电机引脚初始化
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(PWMB, OUTPUT);

    //编码器中断初始化
    attachInterrupt(0, L_encoder, CHANGE);
    attachInterrupt(1, R_encoder, CHANGE);
}

/*
 *@description:设定电机的pwm
 *@input:左轮PWM，右轮PWM，范围为-255到+255
 *@output:无
*/
void Set_pwm(int L_pwm, int R_pwm)
{
    //限幅
    if (L_pwm < -MOTOR_PWM_MAX)
        L_pwm = -MOTOR_PWM_MAX;
    if (L_pwm > MOTOR_PWM_MAX)
        L_pwm = MOTOR_PWM_MAX;
    if (R_pwm < -MOTOR_PWM_MAX)
        R_pwm = -MOTOR_PWM_MAX;
    if (R_pwm > MOTOR_PWM_MAX)
        R_pwm = MOTOR_PWM_MAX;

    //去掉死区，死区大小需要实测
    if (L_pwm < 10 && L_pwm >= 0)
        L_pwm += MOTOR_PWM_DEATH;
    if (L_pwm >-10 && L_pwm < 0)
        L_pwm -= MOTOR_PWM_DEATH;
    if (R_pwm < 10 && L_pwm >= 0)
        R_pwm += MOTOR_PWM_DEATH;
    if (R_pwm >-10 && R_pwm < 0)
        R_pwm -= MOTOR_PWM_DEATH;

    if (L_pwm >= 0)
    {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(PWMA, L_pwm);
    }
    else
    {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(PWMA, -L_pwm);
    }
    if (R_pwm >= 0)
    {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(PWMB, R_pwm);
    }
    else
    {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(PWMB, -R_pwm);
    }
}

/*
 *@description:计算轮子的速度（无量纲）
 *@input:无
 *@output:无
*/
void Get_velocity(void)
{
    static int L_lastcnt = 0;
    static int R_lastcnt = 0;

    L_velocity = L_cnt - L_lastcnt;
    R_velocity = R_cnt - R_lastcnt;

    L_lastcnt = L_cnt;
    R_lastcnt = R_cnt;
}
