/*
 ******************************************************************************
 *@file    :Control.c
 *@brief   :控制函数
 *@author  :Cloude Remnant
 *@date    :2019-04-16
 ******************************************************************************
 *@description:
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <MsTimer2.h>
#include "Control.h"
#include "Chassis.h"
#include "HC06.h"
#include "IMU.h"
/* Macro ---------------------------------------------------------------------*/
//Arduino 宏定义为0或1没有区别，所以测试时只能通过注释来实现
 #define TEST_IMU    1
// #define TEST_HC06   1
// #define TEST_CHASSIS    1
// #define TEST_PID    1
#define DIRECTION_PWM   (20)
#define AIM_VELOCITY    (20)
/* Variable ------------------------------------------------------------------*/
int Balance_pwm = 0;
int Direction_pwm = 0;
int Velocity_pwm = 0;
int L_pwm = 0;
int R_pwm = 0;
int Aim_velocity = 0;
// int Aim_derction = 0; //TODO
// int Encoder_Least;
// float Aim_angle;

/*HC06*/
extern SoftwareSerial BTserial;

/*Chassis*/
extern int L_cnt;
extern int R_cnt;
extern int L_velocity;
extern int R_velocity;

/*IMU*/
extern float accgyo[3]; //x轴加速度、z轴加速度、y轴角速度
extern float Angle;     //小车倾角
extern float Gyo;       //小车角速度


#ifdef TEST_HC06
char test[10];
#endif
/* Function ------------------------------------------------------------------*/

/*
 *@description:小车平衡控制，第一级为位置环，第二级为角速度环
 *@input:无
 *@output:无
*/
void BalancePWM_Set(void)
{
    static float I1    = 1.5;   /*为计算第一级积分*/
    static float I2    = 1.4;   /*为计算第二级积分*/
    static float last1 = 1.5;   /*为计算第一级微分*/
    static float last2 = 1.5;   /*为计算第二级微分*/
    static float out1  = 0;     /*第一级的输出*/
    static float angle_last = 0;
    float  Kp1         = 10;
    float  Ki1         = 0;
    float  Kd1         = 0;     /*第一级的PID系数*/
    float  Kp2         = 0;
    float  Ki2         = 0;
    float  Kd2         = 0;     /*第二级的PID系数*/
    float  angle_error = 0;     /*误差*/

    /*位置环*/
    angle_error = 0 - Angle;
    I1 += angle_error;

    //积分限幅
    if(I1>100)
        I1=100;
    if(I1<-100)
        I1=-100;

    //过零点消除积分
    if(angle_last*Angle<=0)
        I1=0;

    angle_last=angle_error;

    out1 = (angle_error * Kp1 + I1 * Ki1 + (angle_error - last1) * Kd1);
    last1 = angle_error;

    /*角速度环*/
    I2 += (Gyo - out1);

    Balance_pwm = (Gyo - out1) * Kp2 + I2 * Ki2 + ((Gyo - out1) - last2) * Kd2;
    last2 = (Gyo - out1);

    Balance_pwm = out1;//测试位置环

#ifdef TEST_PID
    Serial.print("Balance_pwm:");
    Serial.print(Balance_pwm);
    Serial.print("  ");
    Serial.print("I1:");
    Serial.print(I1);
    Serial.print("  ");
    Serial.print("angle_error:");
    Serial.print(angle_error);
    Serial.print("  ");
    // Serial.print("out1:");
    // Serial.println(out1);
#endif
}

/*
 *@description:小车方向控制，目标参数设置
 *@input:无
 *@output:无
*/
void Direction_Control(void)
{  
    static char direction = 'p'; /*小车运动方向，默认停止*/

    if(BTserial.available() > 0)/*如果有指令，就读取最后一个*/
    {
        while(BTserial.available() > 0)
            direction = BTserial.read();
        BTserial.println("OK");
        BTserial.println(direction);
    }

    switch (direction)
    {
        case 'w': //前进
        {
            Aim_velocity = AIM_VELOCITY;
            Direction_pwm = 0;
            break;
        }
        case 'a': //左转
        {
            Aim_velocity = AIM_VELOCITY;
            Direction_pwm = DIRECTION_PWM;
            break;
        }
        case 'd': //右转
        {
            Aim_velocity = AIM_VELOCITY;
            Direction_pwm = -DIRECTION_PWM;
            break;
        }
        case 's': //后退
        {
            Aim_velocity = -AIM_VELOCITY;
            Direction_pwm = 0;
            break;
        }
        case 'p': //停止
        {
            Aim_velocity = 0;
            Direction_pwm = 0;
            break;
        }
        default://停止
        {
            Aim_velocity = 0;
            Direction_pwm = 0;
        }
    }
}

void Output_Set(void)
{
    L_pwm = Balance_pwm + Velocity_pwm - Direction_pwm;
    R_pwm = Balance_pwm + Velocity_pwm + Direction_pwm;
}

/*
 *@description:小车主控制，5ms循环
 *@input:无
 *@output:无
*/
void Kernel(void)
{
    sei();//开启全局中断

    Get_velocity();    //获取轮子转速
    Angle_Calculate(); //获取并计算MPU数据
    Direction_Control();//小车移动方向获取，目标参数设置

    BalancePWM_Set(); //计算小车平衡PWM
    VelocityPWM_Set();//计算小车速度PWM

    Output_Set();//计算小车PWM输出
    Set_pwm(L_pwm, R_pwm);//输出PWM

#ifdef TEST_PID
    Serial.print("L_pwm:");
    Serial.print(L_pwm);
    Serial.print("  ");
    Serial.print("R_pwm:");
    Serial.println(R_pwm);
#endif
    
#ifdef TEST_HC06
    if (BTserial.available() > 0)
    {
        test[0] = BTserial.read();
        BTserial.print(test[0]);
        Serial.println(test[0]);
    }
#endif

#ifdef TEST_CHASSIS
    Serial.print("L_cnt:");
    Serial.print(L_cnt);
    Serial.print("  ");
    Serial.print("R_cnt:");
    Serial.println(R_cnt);
#endif

#ifdef TEST_IMU
    Serial.print("Angle:");
    Serial.print(Angle);
    Serial.print("  ");
    Serial.print("Gyo:");
    Serial.println(Gyo);
#endif
}

/*
 *@description:任务初始化
 *@input:无
 *@output:无
*/
void Kernel_Init(void)
{
    MsTimer2::set(5, Kernel); //定时中断dt ms, Kernel()主要控制函数
    MsTimer2::start();
}

/*
 *@description:小车速度环控制
 *@input:无
 *@output:无
*/
void VelocityPWM_Set(void)
{
    static int I = 0;
    float p = 0.5;
    float v_I = 0;
    int error;
    
    error = L_velocity + R_velocity - Aim_velocity;
    I += error;
    Velocity_pwm = error * p + I * v_I;
}
