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
#include "../../test_MPU6050/inc/FreeRTOSConfig.h"
#include "freertos.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "main.h"
#include "math.h"

/* Colas para los datos del MPU*/
/* aceleracion+giroscopo */
xQueueHandle 		ColaMPU;

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

		uint8_t wbuf[2]={0,0};
		/*Sleep & cycle disabled, 00
		 *temp_sensor: 0, enabled but no used,
		 *clock source: 0 : MPU internal - 8MHz
		*/
		wbuf[0]=MPU6050_RA_PWR_MGMT_1;
		wbuf[1]=0x01;
		Chip_I2C_MasterSend(id,MPU6050_DEVICE_ADDRESS,wbuf,2);

		/*
		*FSYNC: input disabled
		*DLPF: 0
		*	LPF Acc: 260Hz
		*	LPF gyr: 265Hz
		*/
		wbuf[0]=MPU6050_RA_CONFIG;
		wbuf[1]=0x00;
		Chip_I2C_MasterSend(id,MPU6050_DEVICE_ADDRESS,wbuf,2);

		/*
		 * gyro full scale: -250º/seg : 250º/seg
		 */
		wbuf[0]=MPU6050_RA_GYRO_CONFIG;
		wbuf[1]=0x00;
		Chip_I2C_MasterSend(id,MPU6050_DEVICE_ADDRESS,wbuf,2);

		/*
		 * acc full scale: (-2G : 2G)
		 */
		wbuf[0]=MPU6050_RA_ACCEL_CONFIG;
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

/** @brief Main del programa
 *	@return bool
 */

void LecturaDeDatos( void * param)
{
	while(1)
	{

	}
}
void ProcesamientoDeDatos( void * param)
{
	while(1)
	{

	}
}
int main(void)
{
	initHardware();
	ColaMPU= xQueueCreate(4, sizeof(datos_acelerometro));
//	xTaskCreate(LecturaDeDatos, (const char *)"Datos", configMINIMAL_STACK_SIZE*2,0, tskIDLE_PRIORITY+1, 0);
//	xTaskCreate(ProcesamientoDeDatos, (const char *)"Datos", configMINIMAL_STACK_SIZE*2,0, tskIDLE_PRIORITY+1, 0);
	vTaskStartScheduler();
	for(;;);

/* 	uint8_t rbuf_accel[6] = {0,0,0,0,0,0};		buffer para recibir ACCEl del MPU
	uint8_t  rbuf_gyro[6] = {0,0,0,0,0,0};		buffer para recibir GYRO del MPU
	int16_t samples[6] ={ 0,0,0,0,0,0 };	//	Cada posicion es de 16 bits, necesario para guardar	la parte low y high de las muestras de los datos
	initHardware();
	MPU6050_Init(I2C1);
	double aux_x=0,aux_y=0,aux_z=0;
	while(1)
	{
		 	Chip_I2C_MasterCmdRead(I2C1,MPU6050_DEVICE_ADDRESS,MPU6050_RA_ACCEL_XOUT_H, rbuf_accel, 6);
		 	Chip_I2C_MasterCmdRead(I2C1,MPU6050_DEVICE_ADDRESS,MPU6050_RA_GYRO_XOUT_H, rbuf_gyro, 6);
		 	MPU6050_GetData(samples,rbuf_accel,rbuf_gyro);
		 	MPU6050_GetAngle(samples,&aux_x,&aux_y,&aux_z);
	}*/
	return 0;
}

/** @brief 	Devuelve los datos del modulo MPU6050
 *  @param	samples : vector donde se encuentran los datos ya concatenados.
 *  @param  rbuf    : vector de la comunicacion i2s donde se encuentran las partes de los datos.
 *	@return nada
 *	@details
 *  Desplazamos la parte alta a los 8 ultimos bits y le hacemos un OR para tener en los 8 primeros la parte baja
 */

void MPU6050_GetData(int16_t * samples, uint8_t * rbuf_a,uint8_t * rbuf_g)
{
	samples[0]=( ((int16_t) rbuf_a[0] << 8) | rbuf_a[1] );  	// aceleracion angulo x
	samples[1]=(  ((int16_t) rbuf_a[2] << 8) | rbuf_a[3] );  	// aceleracion angulo y
	samples[2]=(  ((int16_t) rbuf_a[4] << 8) | rbuf_a[5] );  	// aceleracion angulo z

	samples[3]=(  ((int16_t) rbuf_g[0] << 8) | rbuf_g[1] );  	// giroscopio angulo x
	samples[4]=(  ((int16_t) rbuf_g[2] << 8)| rbuf_g[3] );  	// giroscopio angulo y
	samples[5]=(  ((int16_t) rbuf_g[4] << 8)| rbuf_g[5] );  	// giroscopio angulo z
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

void MPU6050_GetAngle(int16_t* samples,double* angle_x,double* angle_y,double* angle_z)
{
	double altX=0,altY=0,altZ=0;
	static double subX=0,subY=0,subZ=0;

	altX = atan2((double)(samples[0]),sqrt(pow( samples[1],2) + pow( samples[2],2))) * RAD_TO_DEG;
	altY = atan2((double)(samples[1]),sqrt(pow( samples[0],2) + pow(samples[2],2)))*RAD_TO_DEG;
	altZ = atan2((double)sqrt(pow(samples[0],2) + pow( samples[1],2) ),samples[2] ) *RAD_TO_DEG;

	*angle_x=altX*(1 - MPU6050_BETA)+ MPU6050_BETA*(( samples[4] / MPU6050_GS)*MPU6050_DT + subX);
	*angle_y=altY*(1 - MPU6050_BETA)+ MPU6050_BETA*(( samples[5] / MPU6050_GS)*MPU6050_DT + subY);
	*angle_z=altZ*(1 - MPU6050_BETA)+ MPU6050_BETA*(( samples[6] / MPU6050_GS)*MPU6050_DT + subZ);


	subX=*angle_x;
	subY=*angle_y;
	subZ=*angle_z;
}
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
