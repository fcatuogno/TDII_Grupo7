/*
 * main.c
 *
 *  Created on: 31 de jul. de 2018
 *      Author: Daiana Valeria Casas
 */

/** @brief I2C para el MPU. Desarrollo de las cosas probadas
 */

 /** \addtogroup I2C example
 ** @{ */
#include "board.h"
#include "main.h"
#include "mpu6050.h"
#include "math.h"

/** @brief hardware initialization function
 *	@return none
 */

/*==================[internal data definition]===============================*/

static void initHardware(void)
{
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock/1000);
    Board_Init();
    Board_LED_Set(0, false);
    I2C_Init(I2C1);
    Chip_I2C_SetClockRate(I2C1,CLOCK_RATE_MPU);
	Chip_I2C_SetMasterEventHandler(I2C1, Chip_I2C_EventHandlerPolling);
}

int main(void)
{
	/*==================[Inicializacion]==========================*/										/*	buffer para escribir en el MPU	*/
	uint8_t rbuf[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};	/*	buffer para recibir en el MPU	*/
	uint16_t samples[7] = {0,0,0,0,0,0,0}; 	//	Cada posicion es de 16 bits, necesario para guardar	la parte low y high de las muestras de los datos
	initHardware();
	MPU6050_Init(I2C1);
	while(1)
	{
		Chip_I2C_MasterCmdRead(I2C1,MPU6050_DEVICE_ADDRESS,MPU6050_RA_ACCEL_XOUT_H, rbuf, 14);
		MPU6050_GetData(&samples, &rbuf);
	}
	return 0;
}
/** @} doxygen end group definition */

/*==================[end of file]============================================*/
