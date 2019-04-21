/*
 ******************************************************************************
 *@file    :Filter.c
 *@brief   :各种滤波算法
 *@author  :Cloude Remnant
 *@date    :2019-04-01
 ******************************************************************************
 *@description:
 * + 均值滤波
 * + 滑动加权平均滤波
 * + 卡尔曼滤波，待测试
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "Filter.h"
/* Macro ---------------------------------------------------------------------*/

/* Variable ------------------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

/**
  * @brief  将32位的主机数据流转换为网络流
  * @details  
        Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
        R:测量噪声，R增大，动态响应变慢，收敛稳定性变好 
  * @param  ResrcData  需要过滤的数据
  * @param  
  * @return 过滤之后的数据
 */
double Filter_Kalman(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
    double R = MeasureNoise_R;
    double Q = ProcessNiose_Q;

    static double x_last;
    double x_mid = x_last;
    double x_now;

    static double p_last;
    double p_mid ;
    double p_now;

    double kg;

    x_mid=x_last;                       //x_last=x(k-1|k-1),x_mid=x(k|k-1)
    p_mid=p_last+Q;                     //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声

    /*
     *  卡尔曼滤波的五个重要公式
     */
    kg=p_mid/(p_mid+R);                 //kg为kalman filter，R 为噪声
    x_now=x_mid+kg*(ResrcData-x_mid);   //估计出的最优值
    p_now=(1-kg)*p_mid;                 //最优值对应的covariance
    p_last = p_now;                     //更新covariance 值
    x_last = x_now;                     //更新系统状态值

    return x_now;
}

/**
  * @brief 滑动加权滤波算法
  * @details 
  * @gram p_buff 采样缓存队列
  * @gram  value 采样值
  * @gram  n_sample 采样次数 设定好不能改动
  * @retval 无
 */
float Filter_Huadongjiaquan(float *p_buff,float value, int n_sample)
{
    float temp;
    float sum=0;
    for(int i=1;i < n_sample; i++)
    {
        p_buff[i-1] = p_buff[i];
        sum += p_buff[i] * i;
    }
    p_buff[n_sample-1] = value;
    sum += value*n_sample;
     
    temp = sum /(((n_sample+1)/2 * n_sample)-1);
    return temp;
}

/*
 *@description:均值滤波算法
 *@input:新数据，滤波数组，数组原始数据存储长度
 *@output:
*/
float Filter_Junzhilvbo(float data, float* p_buff, int length)
{
    int i = 0;
    float sum = 0;

    //[0]-[length-1]为最近length次数据 [length]为length次数据的平均值
    for(i = 1; i < length; i++)
        p_buff[i-1] = p_buff[i];

    p_buff[length-1] = data;

    for(i = 0; i < length; i++)//求当前数组的合，再取平均值
		sum += p_buff[i];

	return sum / 10;
}

