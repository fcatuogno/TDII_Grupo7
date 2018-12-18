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

angulos_mpu PID_Compute(angulos_mpu angMPU, angulos_mpu angMotor, angulos_mpu SetPoint, VALUES_K Kx, VALUES_K Ky, VALUES_K Kz,float period, angulos_mpu limites_Min, angulos_mpu limites_Max)
{
	angulos_mpu error;
	static angulos_mpu ult_angulo;
	static angulos_mpu AngIntErr = {.pitch=0, .roll=0, .yaw=0};
	angulos_mpu AngDerErr;
	static int first_time = TRUE;

	if(first_time){
		ult_angulo = angMPU;
		first_time = FALSE;
	}

	//calculo error en base a setpoint
	error.yaw = SetPoint.yaw - angMPU.yaw;
	error.roll = SetPoint.roll - angMPU.roll;
	error.pitch = SetPoint.pitch - angMPU.pitch;

	//--> Error Integral   --------TIENE EN CUENTA TODOS LOS ERRORES ANTERIORES---------
	//Ante cambios de CTEs on the fly debe afectar solo a muestra actuales, no pasadas:
	AngIntErr.yaw += (Kx.Ki*error.yaw);
	AngIntErr.yaw *= period;
	AngIntErr.roll += (Ky.Ki*error.roll);
	AngIntErr.roll *= period;
	AngIntErr.pitch += (Kz.Ki*error.pitch);
	AngIntErr.pitch *= period;

	//--> Error Derivativo
	//Se trabaja con posicion y no con error para evitar "derivatice kick" en cambio de setpoint
	AngDerErr.yaw = (ult_angulo.yaw - angMPU.yaw)/(period);
	AngDerErr.roll = (ult_angulo.roll - angMPU.roll)/(period);
	AngDerErr.pitch = (ult_angulo.pitch - angMPU.pitch)/(period);

	//Se guarda el valor anterior
	ult_angulo.yaw = angMPU.yaw;
	ult_angulo.roll = angMPU.roll;
	ult_angulo.pitch = angMPU.pitch;

	angMPU.yaw = /*angMotor.yaw*/ + (Kx.Kp*error.yaw + AngIntErr.yaw + Kx.Kd*AngDerErr.yaw);
	angMPU.roll = /*angMotor.roll +*/ (Ky.Kp*error.roll + AngIntErr.roll + Ky.Kd*AngDerErr.roll);
	angMPU.pitch = /*angMotor.pitch +*/ (Kz.Kp*error.pitch + AngIntErr.pitch + Kz.Kd*AngDerErr.pitch);

	/*---------------VERIFICAR LIMITEs-----------------------*/
	if(angMPU.yaw>limites_Max.yaw){
		angMPU.yaw=limites_Max.yaw;
	}else if(angMPU.yaw<limites_Min.yaw){
		angMPU.yaw=limites_Min.yaw;
	}

	if(angMPU.roll>limites_Max.roll){
		angMPU.roll=limites_Max.roll;
	}else if(angMPU.roll<limites_Min.roll){
		angMPU.roll=limites_Min.roll;
	}

	if(angMPU.pitch>limites_Max.pitch){
		angMPU.pitch=limites_Max.pitch;
	}else if(angMPU.pitch<limites_Min.pitch){
		angMPU.pitch=limites_Min.pitch;
	}


	return angMPU;
}

