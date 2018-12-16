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
/*
void PID_Set()
{
	//--> Error proporcional
	error[0].p=angulos.yaw;
	error[1].p=angulos.roll;
	error[2].p=angulos.pitch;

	//--> Error integral
	error[0].i=error[0].i + error[0].p * iTime ;
	if(error[0].i>OUTPUT_MAX_X)
		error[0].i=OUTPUT_MAX_X;
	else if(error[0].i<OUTPUT_MIN_X)
		error[0].i=OUTPUT_MIN_X;

	error[1].i=error[1].i + error[1].p * iTime ;
	if(error[0].i>OUTPUT_MAX_Y)
		error[0].i=OUTPUT_MAX_Y;
	else if(error[0].i<OUTPUT_MIN_Y)
		error[0].i=OUTPUT_MIN_Y;

	error[2].i=error[2].i + error[2].p * iTime ;
	if(error[0].i>OUTPUT_MAX_Z)
		error[0].i=OUTPUT_MAX_Z;
	else if(error[0].i<OUTPUT_MIN_Z)
		error[0].i=OUTPUT_MIN_Z;

	//Error derivativo
	//No habra derivative kick ya que el setpoint no cambia
	if(FIRST_TIME){
	error[0].d= 0 ;
	error[1].d= 0 ;
	error[2].d= 0 ;
	FIRST_TIME=FALSE;
	}
	else{
	error[0].d= ( error[0].p - error_ant[0] )/ (iTime) ;
	error[1].d= ( error[1].p- error_ant[1] )/ (iTime) ;
	error[2].d= ( error[2].p - error_ant[2] )/ (iTime) ;

}
}*/
