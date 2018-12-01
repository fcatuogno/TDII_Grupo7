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
#include "../../test_MPU6050/inc/main.h"
#include "../../test_MPU6050/inc/FreeRTOSConfig.h"
#include "../../test_MPU6050/inc/mpu6050.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
/* Colas para los datos del MPU*/
/* aceleracion+giroscopo */
xQueueHandle 		ColaMPU;

#define TIME_PERIOD_TASK	1/portTICK_RATE_MS // cada milisegundos
#define deMS_aS 0.001

/** @brief Inicializacion de hardware
 *	@return nada
 */
static void initHardware(void)
{
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock/1000);

	Chip_GPIO_Init(LPC_GPIO);
	Chip_IOCON_Init(LPC_IOCON);

    Chip_I2C_InitPins(I2C1);
    Chip_I2C_SetClockRate(I2C1, CLOCK_RATE_MPU);
	Chip_I2C_SetMasterEventHandler(I2C1, Chip_I2C_EventHandlerPolling);

	MPU6050_Init(I2C1);
}

/** @brief LecturaDeDatos
 *  @details
 *  Tarea que se encarga de pedir datos al MPU
 *	@return bool
 */
void LecturaDeDatos (void * param)
{
	portTickType xLastWakeTime;
	const portTickType xPeriod=TIME_PERIOD_TASK;
	static values_mpu datos;

	// Inicializamos el xLastWakeTime con el tiempo actual al ingresar a la tarea
	xLastWakeTime=xTaskGetTickCount();
	while(1){

		// Espera al proximo ciclo;
		vTaskDelayUntil( &xLastWakeTime , xPeriod );

		// Realizamos el pedido de datos:
	 	Chip_I2C_MasterCmdRead(I2C1,MPU6050_DEVICE_ADDRESS,MPU6050_RA_ACCEL_XOUT_H, datos.read_buffer_accel, 6);
	 	Chip_I2C_MasterCmdRead(I2C1,MPU6050_DEVICE_ADDRESS,MPU6050_RA_GYRO_XOUT_H, datos.read_buffer_giro, 6);
	 	MPU6050_GetData( datos.buffer_gral , datos.read_buffer_accel , datos.read_buffer_giro );
	 	xQueueSend( ColaMPU, & datos , portMAX_DELAY);
	}

}
/** @brief ProcesamientoDeDatos
 *  @details
 *  Tarea que se encarga de convertir datos al MPU a angulos
 *	@return bool
 */
void ProcesamientoDeDatos (void * param)
{
	static values_mpu medicion;
	static angulos_mpu angulos;
	while(1){
		xQueueReceive( ColaMPU , & medicion , portMAX_DELAY );
		MPU6050_GetAngle( & medicion ,& angulos ,  ((float)TIME_PERIOD_TASK * deMS_aS));
	}
}

int main(void)
{
	//Inicializamos el hardware
	initHardware();
	// Creamos la cola que nos permite acceder a los datos del acelerometro
	ColaMPU= xQueueCreate(5, sizeof(values_mpu));
	// Tareas para manipular los datos y calcular los angulos:

	xTaskCreate(LecturaDeDatos,(const char*) "ObtenerDatos", configMINIMAL_STACK_SIZE*2,0, tskIDLE_PRIORITY+1, NULL);
	xTaskCreate(ProcesamientoDeDatos,(const char*) "DatosParaProcesar", configMINIMAL_STACK_SIZE*2,0, tskIDLE_PRIORITY, NULL);

	// Iniciamos el Scheduler
	vTaskStartScheduler();

	for(;;);
	return 0;
}


/** @} doxygen end group definition */
/*==================[end of file]============================================*/
