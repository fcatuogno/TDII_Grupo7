/*
 * pid.h
 *
 *  Created on: 4 de dic. de 2018
 *      Author: usuario
 */

#ifndef PROJECTS_TEST_PID_MPU_INC_PID_H_
#define PROJECTS_TEST_PID_MPU_INC_PID_H_

/*==================[macros and definitions]=================================*/


#define OUTPUT_MAX_X	90
#define OUTPUT_MIN_X	-90
#define OUTPUT_MAX_Y	90
#define OUTPUT_MIN_Y	-90
#define OUTPUT_MAX_Z	90
#define OUTPUT_MIN_Z	-90

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
}VALUES_K;

typedef struct
{
	float p;
	float d;
	float i;
}VALUES_ERROR;

typedef enum
{
	KP,
	KI,
	KD
}KTES;

void PID_Init( VALUES_K* var, float k1, float k2, float k3);
void PID_SetK( VALUES_K* var, float value,KTES k);
float PID_GetK(VALUES_K* const var,KTES k);



#endif /* PROJECTS_TEST_PID_MPU_INC_PID_H_ */
