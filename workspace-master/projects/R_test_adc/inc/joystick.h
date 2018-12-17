/*
 * joystick.h
 *
 *  Created on: 15 de dic. de 2018
 *      Author: Daiana
 */

#ifndef JOYSTICK_H_
#define JOYSTICK_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "chip.h"
#include "motor.h"
/* Rango de valores para el ADC
 *
 * Valor de cuenta: 3V3/ (2(12bits) -1) =
 * maximo : MAX : 4095
 * medio  :
 * 			maximo_intermedio : MAX_I :2100
 * 			minimo_intermedio : MIN_I :1994
 * minimo : MIN : 0
 */

#define DATO_MAX		4095
#define DATO_MIN		0
#define DATO_MAX_I		4000
#define DATO_MIN_I		1000

#define DEBOUNCE 10
/* Valor del salto de angulo para el SERVO
 *
 * Valor:
 * normal : ANG : 10
 * minimo : ANG_MIN : 5
 * maximo : ANG_MAX : 15
 */
#define DELTA_ANG 10
#define DELTA_ANG_MAX 15
#define DELTA_ANG_MIN 5

/*Canales para el ADC
 *
 * */
#define CANAL_0	0,23
#define CANAL_1 0,24
#define CANAL_2	0,25

/* LED DEL BOARD*/
#define LED 0,22

typedef struct {
	uint16_t datos_x;
	uint16_t datos_y;
} JOYSTICK_AXIS;

void JOYSTICK_AngleLimit(int32_t * valor);
bool JOYSTICK_Debounce(int32_t * cuenta );

#endif /* PROJECTS_R_TEST_ADC_INC_JOYSTICK_H_ */
