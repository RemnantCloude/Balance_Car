/*
 ******************************************************************************
 *@file    :Control.h
 *@brief   :
 *@author  :Cloude Remnant
 *@date    :2019-04-16
 ******************************************************************************
 *@description:
 *
 ******************************************************************************
 */

#ifndef CONTROL_H
#define CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variable ------------------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/
void Kernel_Init(void);
void Direction_Control(void);
void Velocity(int L_velocity ,int R_velocity);
void Velocity_control();
void Balance_control(void);
void Direction(void);
void stand(void);
void back(void);
void left(void);
void front(void);
void right(void);

#ifdef __cplusplus
}
#endif

#endif // CONTROL_H
