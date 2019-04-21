/*
 ******************************************************************************
 *@file    :Filter.h
 *@brief   :
 *@author  :Cloude Remnant
 *@date    :2019-04-01
 ******************************************************************************
 *@description:
 *
 ******************************************************************************
 */

#ifndef FILTER_H
#define FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variable ------------------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/
double Filter_Kalman(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
float Filter_huadongjiaquan(float *p_buff,float value, int n_sample);
float Filter_Junzhilvbo(float data, float* p_buff, int length);

#ifdef __cplusplus
}
#endif

#endif // FILTER_H

