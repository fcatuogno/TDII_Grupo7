/*
 * @brief motor.h Drivers para el servomotor
 * @note
 *  Created on: 8 de set. de 2018
 *      Author: Daiana
 *
 * @details Funciones para el manejo de los servomotores
 */
#include "chip.h"
#include "motor.h"

/*	Inicilizacion de los pines y configuracion del pwm */
void Motor_Init(uint8_t channel, uint32_t frecuency)
{
	   /******************* PWM *******************/
	    /* Configuramos los pines con la funcion PWM */
		PWM1_SelectChannel(LPC_PWM1,CH1);
		PWM1_SelectChannel(LPC_PWM1,CH2);
		PWM1_SelectChannel(LPC_PWM1,CH3);
		PWM1_SelectChannel(LPC_PWM1,CH4);
		/* Habilitamos el clock y su prescale para el modulo */
		PWM1_enableCLK(LPC_PWM1);
		PWM1_preescaleCLK(LPC_PWM1,0);

		/* Match 0 (period) */
		PWM1_SetMatch(LPC_PWM1,CH0,frecuency);//
		/* Match 1 (duty) */
		PWM1_SetMatch(LPC_PWM1,CH1,ANGLE_0);
		/* Match 2 (duty) */
		PWM1_SetMatch(LPC_PWM1,CH2,ANGLE_0);
		/* Match 3 (duty) */
		PWM1_SetMatch(LPC_PWM1,CH3,ANGLE_0);
		/* Match 4 (duty) */
		PWM1_SetMatch(LPC_PWM1,CH4,ANGLE_0);

		/* Definimos que el MR0 resetee el TC */
		PWM1_ConfigMatch(LPC_PWM1,CH0,0,0,0);

		/* Activamos el modo PWM y habilitamos el conteo de TC y PC */
		PWM1_EnableCounters(LPC_PWM1);

		/* Habilito los cambios en los MATCH's */
		PWM1_EnableMatchValue(LPC_PWM1,CH0);
		PWM1_EnableMatchValue(LPC_PWM1,CH1);
		PWM1_EnableMatchValue(LPC_PWM1,CH2);
		PWM1_EnableMatchValue(LPC_PWM1,CH3);
		PWM1_EnableMatchValue(LPC_PWM1,CH4);

		/* Habilito las salidas del canal */
		PWM1_ControlChannel(LPC_PWM1,CH1,0,1);
		PWM1_ControlChannel(LPC_PWM1,CH2,0,1);
		PWM1_ControlChannel(LPC_PWM1,CH3,0,1);
		PWM1_ControlChannel(LPC_PWM1,CH4,0,1);

		/*	Reseteo todos los contadores */
		PWM1_ResetCounters(LPC_PWM1);

}

/*	Seteo del tiempo de duty para un angulo	determinado */
void Motor_Set(uint8_t channel,int32_t angle)
{
	uint32_t duty=0;
	if(angle>=0){
		duty = ANGLE_0 + (angle*(ANGLE_DELTA1));
	}
	else{
		duty = (ANGLE_N95) + ( (angle + 90) *(ANGLE_DELTA1));
	}

	if( duty >= ANGLE_P95){
		duty=ANGLE_P95; 	}

	if( duty <= ANGLE_N95){
		duty=ANGLE_N95;	}

    PWM1_SetMatch(LPC_PWM1,channel,duty);
    /*PWM1_SetMatch(LPC_PWM1,CH2,duty);
    PWM1_SetMatch(LPC_PWM1,CH3,duty);
    PWM1_SetMatch(LPC_PWM1,CH4,duty);*/
	 // Habilito los cambios en los MATCH's
    PWM1_EnableMatchValue(LPC_PWM1,channel);
/*    PWM1_EnableMatchValue(LPC_PWM1,CH1);
    PWM1_EnableMatchValue(LPC_PWM1,CH2);
    PWM1_EnableMatchValue(LPC_PWM1,CH3);
    PWM1_EnableMatchValue(LPC_PWM1,CH4);*/
	/*	Reseteo todos los contadores */
	PWM1_ResetCounters(LPC_PWM1);
}
/* Devuelvo el valor del angulo */
int32_t Motor_Get(uint8_t channel)
{
	uint32_t aux;
	float aux_angle;
	PWM1_GetMatch(LPC_PWM1 , channel ,&aux );
	if(aux>=ANGLE_0){aux_angle=(aux-ANGLE_0)/(ANGLE_DELTA1);}
	else{ aux_angle= ( ( aux - (ANGLE_N95) )/(ANGLE_DELTA1))-90;}
	return aux_angle;
}
