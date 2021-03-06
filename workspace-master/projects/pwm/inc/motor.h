/*
 * @brief motor.h Drivers para el servomotor
 * @note
 *  Created on: 8 de set. de 2018
 *      Author: Daiana
 *
 * @details Funciones para el manejo de los servomotores
 */

#ifndef MOTOR_H_
#define MOTOR_H_


#ifdef __cplusplus
extern "C" {
#endif
/** @defgroup MOTOR API:  LPC17xx/40xx A/D conversion driver
 * @ingroup SRC
 * @{
 */
/* @brief cantidad de ticks para llegar a los 20mseg. */
#define FREQ_50HZ	475600
/* @brief Todos los canales habilitados. */
#define CHANNELs    0x0F
/*
#define ANGLE_N90	475600/40	// a los 0.5mseg
#define ANGLE_N45	475600/20	// a los 1mseg
#define ANGLE_0		475600*0.075	// a los 1.5mseg
#define ANGLE_P45	475600/10	// a los 2mseg
#define ANGLE_P90	475600*0.11	// a los 2.2mseg //
#define ANGLE_PX 	475600*0.12 // a los 2.4mseg
*/

#define ANGLE_N95	475600/40		/* @brief a los 0.5mseg */
#define ANGLE_N90	475600*0.03		/* @brief a los 0.6mseg */
#define ANGLE_N45	475600*0.05125	/* @brief a los 1.025mseg */
#define ANGLE_DELTA1 475600*0.0094*0.5  /* @brief salto de 1 grado */
#define ANGLE_0 	475600*0.0725	/* @brief a los 1.45mseg */
#define ANGLE_P45	475600*0.09375  /* @brief a los 1.875mseg */
#define ANGLE_P90 	475600*0.115	/* @brief a los 2.3mseg */
#define ANGLE_P95 	475600*0.12 	/* @brief a los 2.4mseg */

/**
 * @brief	Inicializacion de los pines y la configuracion del PWM
 * @param	channel		: canales que se habilitan
 * @param	frecuency	: frecuencia de la señal pwm.
 * @return	void
 *
 * @details Los 8 bits del canal pertenecen a los canales correspondientes al numero de bit.
 * 			Esto se realiza a nivel de bit para eliminar variables/parametros demás.
 */
void Motor_Init(uint8_t channels, uint32_t frecuency);
/**
 * @brief	Seteo del duty de un cierto canal.
 * @param	channel		: canales que se habilitan
 * @param	angle		: angulo de desplazamiento del servo.
 * @return	void
 */
void Motor_Set(uint8_t channel,uint32_t angle);

/**
 * @brief	Seteo del duty de un cierto canal.
 * @param	channel		: canales que se habilitan
 * @return	value 		: valor del angulo que
 */
uint32_t Motor_Get(uint8_t channel);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H_ */
