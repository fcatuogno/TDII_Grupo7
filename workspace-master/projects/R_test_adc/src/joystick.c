/*
 * joystick.c
 *
 *  Created on: 15 de dic. de 2018
 *      Author: Daiana
 */


#include "joystick.h"


void JOYSTICK_AngleLimit(int32_t * valor)
{
	if(*valor< -90){
		*valor=-90;	}
	else if (*valor >90){
		*valor=90;	}
}

