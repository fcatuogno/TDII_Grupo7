/*
 * main.c
 *  Created on: 31 de jul. de 2018
 *      Author: Daiana Valeria Casas
 */
/** @brief I2C para pruebas.
 */

 /** \addtogroup I2C
 ** @{ */

#include "chip.h"
#include "main.h"
#include "math.h"

static uint32_t pausems_count;

/** @brief Inicializacion del I2S del microcontrolador LPC1769
 *  @param	id		: Etiqueta del numero de la interfaz de I2S a utilizar
 *	@return nada
 */
void Init_I2C(I2C_ID_T id)
{
	switch(id)
	{
	case I2C0:
		Chip_IOCON_PinMux(LPC_IOCON, I2C_SCL0, IOCON_MODE_INACT, IOCON_FUNC1);
		Chip_IOCON_PinMux(LPC_IOCON, I2C_SDA0, IOCON_MODE_INACT, IOCON_FUNC1);
		Chip_IOCON_SetI2CPad(LPC_IOCON, I2CPADCFG_STD_MODE);
		break;

	case I2C1:
		Chip_IOCON_PinMux(LPC_IOCON, I2C_SCL1, IOCON_MODE_PULLUP, IOCON_FUNC3);
		Chip_IOCON_PinMux(LPC_IOCON, I2C_SDA1, IOCON_MODE_PULLUP, IOCON_FUNC3);
		Chip_IOCON_EnableOD(LPC_IOCON, I2C_SCL1);
		Chip_IOCON_EnableOD(LPC_IOCON, I2C_SDA1);
		break;

	case I2C2:
		Chip_IOCON_PinMux(LPC_IOCON, I2C_SCL2, IOCON_MODE_PULLUP, IOCON_FUNC2);
		Chip_IOCON_PinMux(LPC_IOCON, I2C_SDA2, IOCON_MODE_PULLUP, IOCON_FUNC2);
		Chip_IOCON_EnableOD(LPC_IOCON, I2C_SCL2);
		Chip_IOCON_EnableOD(LPC_IOCON, I2C_SDA2);
		break;
	default:
		break;
	}
}

/** @brief Inicializacion del modulo MPU6050
 *  @param	id	: Etiqueta del numero de la interfaz de I2S a utilizar
 *	@return nada
 *	@details
 *  La configuracion en la que quedan seteados es la por defecto:
 *  accelerometer (±2g) , gyroscope (±250°/sec).
 *
 **/

void MPU6050_Init(I2C_ID_T id)
{
		uint8_t wbuf[2] = {MPU6050_RA_PWR_MGMT_1, 0};
		Chip_I2C_MasterSend(id,MPU6050_DEVICE_ADDRESS,wbuf,2);
		wbuf[0]=MPU6050_RA_CONFIG;
		wbuf[1]=0x00;
		Chip_I2C_MasterSend(id,MPU6050_DEVICE_ADDRESS,wbuf,2);
		wbuf[0]=MPU6050_RA_ACCEL_CONFIG;
		wbuf[1]=0x00;
		Chip_I2C_MasterSend(id,MPU6050_DEVICE_ADDRESS,wbuf,2);
		wbuf[0]=MPU6050_RA_PWR_MGMT_2;
		wbuf[1]=0x00;
		Chip_I2C_MasterSend(id,MPU6050_DEVICE_ADDRESS,wbuf,2);
}
/** @brief Inicializacion de hardware
 *	@return nada
 */
static void initHardware(void)
{
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock/1000);

	Chip_GPIO_Init(LPC_GPIO);
	Chip_IOCON_Init(LPC_IOCON);

    Init_I2C(I2C1);
    Chip_I2C_SetClockRate(I2C1, CLOCK_RATE_MPU);
	Chip_I2C_SetMasterEventHandler(I2C1, Chip_I2C_EventHandlerPolling);
}

/** @brief Tiempo de reposo del microprocesador
 *	@return nada
 */
static void pausems(uint32_t t)
{
   pausems_count = t;
   while(pausems_count != 0) {
      __WFI();
   }
}
/** @brief Handler para tiempos
 *	@return nada
 */
void SysTick_Handler(void)
{
   if (pausems_count != 0) {
      pausems_count--;
   }
}

/** @brief Main del programa
 *	@return bool
 */
int main(void)
{
 	uint8_t rbuf[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};	/*	buffer para recibir en el MPU	*/
	uint16_t samples[7] = {0,0,0,0,0,0,0};	//	Cada posicion es de 16 bits, necesario para guardar	la parte low y high de las muestras de los datos
	initHardware();
	MPU6050_Init(I2C1);
	double aux_x=0,aux_y=0,aux_z=0;
	while(1)
	{
		 	 pausems(1000);
		 	 Chip_I2C_MasterCmdRead(I2C1,MPU6050_DEVICE_ADDRESS,MPU6050_RA_ACCEL_XOUT_H, rbuf, 14);
		 	 MPU6050_GetData(samples,rbuf);
		 	 MPU6050_GetAngle(samples,&aux_x,&aux_y,&aux_z);
	}

	return 0;
}

/** @brief 	Devuelve los datos del modulo MPU6050
 *  @param	samples : vector donde se encuentran los datos ya concatenados.
 *  @param  rbuf    : vector de la comunicacion i2s donde se encuentran las partes de los datos.
 *	@return nada
 *	@details
 *  Desplazamos la parte alta a los 8 ultimos bits y le hacemos un OR para tener en los 8 primeros la parte baja
 */
void MPU6050_GetData(uint16_t * samples, uint8_t * rbuf)
{
	samples[0]=(rbuf[0] << 8) | rbuf[1];  	// aceleracion angulo x
	samples[1]=(rbuf[2] << 8) | rbuf[3];  	// aceleracion angulo y
	samples[2]=(rbuf[4] << 8) | rbuf[5];  	// aceleracion angulo z
	samples[3]=(rbuf[6] << 8) | rbuf[7];  	// temperatura
	samples[4]=(rbuf[8] << 8) | rbuf[9];  	// giroscopio angulo x
	samples[5]=(rbuf[10] << 8)| rbuf[11];  	// giroscopio angulo y
	samples[6]=(rbuf[12] << 8)| rbuf[13];  	// giroscopio angulo z
}

/** @brief 	Devuelve los angulos del modulo MPU6050
 *  @param	samples : vector donde se encuentran los datos ya concatenados.
 *  @param  angle_x : vector de la comunicacion i2s donde se encuentran las partes de los datos.
 *  @param  angle_y : vector de la comunicacion i2s donde se encuentran las partes de los datos.
 *  @param  angle_z : vector de la comunicacion i2s donde se encuentran las partes de los datos.
 *  *	@return nada
 *	@details
 *  Se reaiza el calculo de los angulos a partir del valor extaido del mpu.
 * Tita: angulo de x Acc[1]
 * zeta : angulo de y Acc[0]
 * phi : angulo de z Acc[2]
 *
**/

void MPU6050_GetAngle(uint16_t* samples,double* angle_x,double* angle_y,double* angle_z)
{
	*angle_x = atan(-1*((double)samples[0]/A_R)/sqrt(pow(((double)samples[1]/A_R),2) + pow(((double)samples[2]/A_R),2)))* RAD_TO_DEG;
	*angle_y = atan(((double)samples[1]/A_R)/sqrt(pow(((double)samples[0]/A_R),2) + pow(((double)samples[2]/A_R),2)))*RAD_TO_DEG;
	*angle_z= atan(sqrt(pow(((double)samples[0]/A_R),2) + pow(((double)samples[1]/A_R),2))/ ((double)samples[2]/A_R) ) *RAD_TO_DEG;
}

/** @} doxygen end group definition */
/*==================[end of file]============================================*/
