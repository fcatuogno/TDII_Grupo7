/*
 * pid.c
 *
 *  Created on: 4 de dic. de 2018
 *      Author: Daiana
 *      Deatils: Funciones para el control PID
 */

#include "chip.h"
#include "math.h"
#include "pid.h"
/*	Inicializacion de las constante del control PID	*/
void PID_Init( VALUES_K* var, float kp, float ki, float kd)
{
	if( kp <= 0 || ki < 0 || kd < 0)
	{ return; }
	var->Kp=kp;
	var->Ki=ki;
	var->Kd=kd;
}
/* Cambio los valores de las constantes del control PID*/
void PID_SetK(VALUES_K* var, float value, KTES k)
{
	if (value>=0)
	{
		switch(k)
		{
			case KP:
				var->Kp=value;
			break;
			case KI:
				var->Ki=value;
			break;
			case KD:
				var->Kd=value;
			break;
			default:
				break;
		}
	}
}

float PID_GetK(VALUES_K* const var,KTES k)
{
	float aux=0;
	switch(k)
		{
			case KP:
				aux=var->Kp;
			break;
			case KI:
				aux=var->Ki;
			break;
			case KD:
				aux=var->Kd;
			break;
			default:
				break;
		}
	return aux;
}
