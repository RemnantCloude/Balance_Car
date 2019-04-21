/*
 ******************************************************************************
 *@file    :IMU.h
 *@brief   :
 *@author  :Cloude Remnant
 *@date    :2019-04-03
 ******************************************************************************
 *@description:
 *
 ******************************************************************************
 */

#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variable ------------------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/
void Mpu6050_Init(void);
void Angle_Calculate(void);

#ifdef __cplusplus
}
#endif

#endif // IMU_H
