/*
 * pid.h
 *
 *  Created on: 4 de dic. de 2018
 *      Author: usuario
 */

#ifndef PROJECTS_TEST_PID_MPU_INC_PID_H_
#define PROJECTS_TEST_PID_MPU_INC_PID_H_

/*=========================[includes]========================================*/
#include "../../test_pid-mpu/inc/mpu6050.h"

/*==================[macros and definitions]=================================*/


typedef struct
{
	float Kp;
	float Ki;
	float Kd;
}VALUES_K;

/*typedef struct
{
	float p;
	float d;
	float i;
}VALUES_ERROR;*/

/*typedef enum
{
	KP,
	KI,
	KD
}KTES;*/

/*void PID_Init( VALUES_K* var, float k1, float k2, float k3);
void PID_SetK( VALUES_K* var, float value,KTES k);
float PID_GetK(VALUES_K* const var,KTES k);*/
angulos_mpu PID_Compute(angulos_mpu angMPU,  angulos_mpu angMotor, angulos_mpu SetPoint, VALUES_K Kx, VALUES_K Ky, VALUES_K Kz,float period, angulos_mpu limites_Min, angulos_mpu limites_Max);


#endif /* PROJECTS_TEST_PID_MPU_INC_PID_H_ */
