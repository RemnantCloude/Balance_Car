/*
 ******************************************************************************
 *@file    :Balance.ino
 *@brief   :
 *@author  :Cloude Remnant
 *@date    :2019-04-16
 ******************************************************************************
 *@description:
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "HC06.h"
#include "Chassis.h"
#include "Control.h"
#include "IMU.h"
/* Macro ---------------------------------------------------------------------*/

/* Variable ------------------------------------------------------------------*/
/*HC06*/
extern SoftwareSerial BTserial;

/*Chassis*/
extern int L_cnt;
extern int R_cnt;
extern int L_velocity;
extern int R_velocity;
/* Function ------------------------------------------------------------------*/
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(2000000);

    HC06_Init();
    Mpu6050_Init();
    Chassis_Init();
    Kernel_Init();

    Serial.println("Initial OK");
    BTserial.println("OK");
}

void loop()
{
    // put your main code here, to run repeatedly:

}
