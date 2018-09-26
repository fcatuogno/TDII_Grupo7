/*
 * pwm.h
 *
 *  Created on: 8 de set. de 2018
 *      Author: usuario
 */

#ifndef MOTOR_H_
#define MOTOR_H_
#include "board.h"


#ifdef __cplusplus
extern "C" {
#endif
/** @defgroup MOTOR API:  LPC17xx/40xx A/D conversion driver
 * @ingroup SRC
 * @{
 */

void Motor_Init(uint8_t, uint8_t);
void Motor_Set(uint8_t,uint32_t);
void Motor_Get();

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H_ */
