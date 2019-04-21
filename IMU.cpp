/*
 ******************************************************************************
 *@file    :IMU.c
 *@brief   :Arduino版本 MPU6050
 *@author  :Cloude Remnant
 *@date    :2019-04-03
 ******************************************************************************
 *@description:
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include "IMU.h"
#include "Filter.h"
#include <math.h>
/* Macro ---------------------------------------------------------------------*/
#define ANGLE_OFFSET    (-0.01)//TODO：需要测试，小车静止时测试角度
#define ANG_OFFSET      (-2.85)//TODO：需要测试，小车静止时测试角速度
/* Variable ------------------------------------------------------------------*/
float accgyo[3]    = {0};  //x轴加速度、z轴加速度、y轴角速度
float Angle        = 0;    //小车倾角
float Gyo          = 0;    //小车角速度
float gyo_buff[10] = {0};  //角速度滤波数组
/* Function ------------------------------------------------------------------*/
void Mpu6050_Init(void)
{
    Wire.beginTransmission(0b1101000); //Mpu6050硬件地址
    Wire.write(0x6b);                  //Mpu6050电源管理寄存器
    Wire.write(0);                     //写零开启Mpu6050
    Wire.endTransmission();

    //可以根据数据手册设置量程
    Wire.beginTransmission(0b1101000); //Mpu6050硬件地址
    Wire.write(0x1b);                  //角速度设置寄存器
    Wire.write(8);                     //设置角速度量程为500°/s（加速度默认为2g）
    Wire.endTransmission();
}

void MpuData_Get(void)
{
    short data[6]; //储存原始数据，两字节
    Wire.beginTransmission(0b1101000);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0b1101000, 14);

    while (Wire.available() < 14);

    data[0] = Wire.read() << 8 | Wire.read(); //x轴加速度
    data[1] = Wire.read() << 8 | Wire.read(); //y轴加速度
    data[2] = Wire.read() << 8 | Wire.read(); //z轴加速度
    Wire.read();Wire.read();                  //温度计
    data[3] = Wire.read() << 8 | Wire.read(); //x轴陀螺仪
    data[4] = Wire.read() << 8 | Wire.read(); //y轴陀螺仪
    data[5] = Wire.read() << 8 | Wire.read(); //z轴陀螺仪

    accgyo[0] = -data[1] * 0.000598154;
    accgyo[1] =  data[2] * 0.000598154;
    accgyo[2] = -data[3] * 0.015259;
}

//角速度和加速度融合计算倾角，尽量简单实用
void Angle_Calculate(void)
{
//    static float gyo_last = 0;
//
//    MpuData_Get();
//
//    if (accgyo[1] == 0)//防止出现0导致atan不能计算
//        accgyo[1] = 0.001;
//
//    accgyo[2] += ANG_OFFSET; //角速度归零校准
//
//    Gyo = Filter_Junzhilvbo(accgyo[2], gyo_buff, 10);                    //均值滤波
//    Angle = 0.05 * (57.296 * atan(accgyo[0] / accgyo[1]) + ANGLE_OFFSET) /*加速度计部分*/
//            + 0.95 * (Angle + (accgyo[2]) * 0.005);                      /*陀螺仪部分*/

  static float gyo_last=0;

  MpuData_Get();
  
  Gyo=0.5*(accgyo[2]+gyo_last);   //角速度一阶滤波
  gyo_last=accgyo[2];

  float acc_angle = 57.296*atan(accgyo[0]/accgyo[1]);  //根据加速度计算的倾角
  float gyo_angle = Angle+(accgyo[2])*0.005;           //角速度积分计算的倾角
  
  Angle=0.005*acc_angle+0.995*gyo_angle;    //加权融合
}
