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
//#define FREQ_50HZ	475600
#define FREQ_50HZ	480000
/* @brief Todos los canales habilitados. */
#define CHANNELs    0x0F

#define ANGLE_N95	FREQ_50HZ/40		/* @brief a los 0.5mseg */ 				//11890
#define ANGLE_N90	FREQ_50HZ*0.03		/* @brief a los 0.6mseg */				//14268
#define ANGLE_N45	FREQ_50HZ*0.05125	/* @brief a los 1.025mseg */			//24374.5
//#define ANGLE_DELTA1 FREQ_50HZ*0.0094*0.5  /* @brief salto de 1 grado */
#define ANGLE_DELTA1 (ANGLE_P95-ANGLE_0)/95									//
#define ANGLE_0 	FREQ_50HZ*0.0725	/* @brief a los 1.45mseg */				//34481
#define ANGLE_P45	FREQ_50HZ*0.09375  /* @brief a los 1.875mseg */			//44587.5
#define ANGLE_P90 	FREQ_50HZ*0.115	/* @brief a los 2.3mseg */				//54694
#define ANGLE_P95 	FREQ_50HZ*0.12 	/* @brief a los 2.4mseg */				//57072


typedef enum{
	MOTOR_X=2,
	MOTOR_Y,
	MOTOR_Z

}EJE;
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
void Motor_Set(uint8_t channel,int32_t angle);

/**
 * @brief	Retorna el valor del angulo en el que esta seteado el motor
 * @param	channel		: canal
 * @return	value 		: valor del angulo
 */
int32_t Motor_Get(uint8_t channel);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H_ */
