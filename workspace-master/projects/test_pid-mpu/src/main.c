/*
 * main.c
 *  Created on: 31 de jul. de 2018
 *      Author: Daiana Valeria Casas
 */
/** @brief I2C para pruebas, se agrega el control PID
 */

 /** \addtogroup I2C
 ** @{ */

#include "chip.h"
#include "../../test_pid-mpu/inc/main.h"
#include "../../test_pid-mpu/inc/FreeRTOSConfig.h"
#include "../../test_pid-mpu/inc/mpu6050.h"
#include "../../test_pid-mpu/inc/pid.h"
#include "../../test_pid-mpu/inc/motor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
/* Colas para los datos del MPU*/
/* aceleracion+giroscopo */
xQueueHandle 		ColaMPU;

#define TIME_PERIOD_TASK	1/portTICK_RATE_MS // cada milisegundos
#define deMS_aS 0.001

/* Datos para el control PID */
VALUES_K PID;
VALUES_ERROR error[3];

#define P 0
#define I 0
#define D 0

/*	Datos para el motor */


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
	PID_Init(&PID, P , I , D);
	Motor_Init(CHANNELs,FREQ_50HZ);
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

	static float x_ant=0,y_ant=0,z_ant=0;
	static float x_out=0,y_out=0,z_out=0;

	static float error_ant[3]={0,0,0};
	static float iTime=0;

	while(1){

		/* Se espera que haya algun dato en la cola para procesar */
		xQueueReceive( ColaMPU , & medicion , portMAX_DELAY );

		/* Se toman los valores de los angulos */
		MPU6050_GetAngle( & medicion ,& angulos ,  ((float)TIME_PERIOD_TASK * deMS_aS));

		/* Error proporcional */
		error[0].p=angulos.yaw - x_ant;
		error[1].p=angulos.roll - y_ant;
		error[2].p=angulos.pitch - z_ant;

		/* Error integral */
		error[0].i=error[0].i + error[0].p * iTime ;
		error[1].i=error[1].i + error[1].p * iTime ;
		error[2].i=error[2].i + error[2].p * iTime ;

		/* Error derivativo */
		error[0].d= ( error[0].i - error_ant[0] )/ (iTime) ;
		error[1].d= ( error[1].i - error_ant[1] )/ (iTime) ;
		error[2].d= ( error[2].i - error_ant[2] )/ (iTime) ;

		/* Se determinan las salidas por la ecuacion tipica del pid */
		x_out= PID.Kp * error[0].p + PID.Ki * error[0].i + PID.Kd * error[0].d ;
		y_out= PID.Kp * error[1].p + PID.Ki * error[1].i + PID.Kd * error[1].d ;
		z_out= PID.Kp * error[2].p + PID.Ki * error[2].i + PID.Kd * error[2].d ;

		/* Se considera la posicion del motor como el punto anterior */
		x_ant=(float)Motor_Get(MOTOR_X);
		y_ant=(float)Motor_Get(MOTOR_Y);
		z_ant=(float)Motor_Get(MOTOR_Z);

		/* Se guarda el error del punto anterior */
		error_ant[0]= error[0].p;
		error_ant[1]= error[1].p;
		error_ant[2]= error[2].p;
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

	PID_SetK(&PID, 1, KP);

	while(1){
	Motor_Set(1,0);
	Motor_Set(1,30);
	Motor_Set(1,45);
	Motor_Set(1,60);
	Motor_Set(1,45);
	Motor_Set(1,90);
    PWM1_ValueMatch(LPC_PWM1,CH4,ANGLE_P95);
	 // Habilito los cambios en los MATCH's
    PWM1_EnableMatchValue(LPC_PWM1,CH4);
	/*	Reseteo todos los contadores */
	PWM1_ResetCounters(LPC_PWM1);

	}
	// Iniciamos el Scheduler
	//vTaskStartScheduler();

	for(;;);
	return 0;
}


/** @} doxygen end group definition */
/*==================[end of file]============================================*/
