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

//--> Tiempo en que levanto cada muestra del MPU
#define TIME_PERIOD_TASK_inTICKS	1/portTICK_RATE_MS 	// cada milisegundos

/*el tiempo cada cuanto nuestra tarea se ejecute sera el tiempo entre datos del MPU*/
#define TIME_inS  1/TIME_PERIOD_TASK_inTICKS*0.001				// conversion de tick a segundos, se define en base al tiempo en TICKs

/*original:*/
//#define TIME_inS  1*portTICK_RATE_MS*0.001				// conversion de tick a segundos:

/* Datos para el control PID */
//VALUES_K PID;
//VALUES_ERROR error[3];
#define TAM_QUEUE 50
#define TAM 30


/*Constantes definidas para Debug, seran recibidas por queue*/
#define Px 1 // 1.3
#define Ix 0
#define Dx 0

#define Py 1 //1
#define Iy 0 //0.8
#define Dy 0

#define Pz 1
#define Iz 0
#define Dz 0

/******************************************************************************/

#define OUTPUT_MAX_X	90
#define OUTPUT_MIN_X	-90
#define OUTPUT_MAX_Y	60
#define OUTPUT_MIN_Y	-60
#define OUTPUT_MAX_Z	90
#define OUTPUT_MIN_Z	-90

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

	Motor_Init(CHANNELs,FREQ_50HZ);
}

void LecturaDeDatos(void * param)
{
	portTickType xLastWakeTime;
	const portTickType xPeriod=TIME_PERIOD_TASK_inTICKS;

	static values_mpu datos;
	static angulos_mpu angulos;

	// Inicializamos el xLastWakeTime con el tiempo actual al ingresar a la tarea
	xLastWakeTime=xTaskGetTickCount();
	while(1){

				// Espera al proximo ciclo;
				vTaskDelayUntil( &xLastWakeTime , xPeriod );

				// Realizamos el pedido de datos:
			 	Chip_I2C_MasterCmdRead(I2C1,MPU6050_DEVICE_ADDRESS,MPU6050_RA_ACCEL_XOUT_H, datos.read_buffer_accel, 6);
			 	Chip_I2C_MasterCmdRead(I2C1,MPU6050_DEVICE_ADDRESS,MPU6050_RA_GYRO_XOUT_H, datos.read_buffer_giro, 6);
			 	MPU6050_GetData( datos.buffer_gral , datos.read_buffer_accel , datos.read_buffer_giro );

			 	MPU6050_GetAngle( & datos ,& angulos ,  ((float)TIME_inS));

			 	xQueueSend(ColaMPU,& angulos, 2*TIME_PERIOD_TASK_inTICKS);
	}
}

void Estabilizador (void * param)
{
	//portTickType xLastWakeTime;
	//const portTickType xPeriod=TIME_PERIOD_TASK_inTICKS;
	//static values_mpu datos;
	static angulos_mpu angulos;
	static angulos_mpu angulosMPU = {.pitch=0, .roll=0, .yaw=0};														 
	static angulos_mpu angMotor;


	/*variables PID*/
	static angulos_mpu SetPoints = {.pitch=0, .roll=0, .yaw=0};
	static VALUES_K constantes_X, constantes_Y, constantes_Z;
	static float period = TIME_inS*TAM;
	static angulos_mpu	limites_Max = {.pitch=OUTPUT_MAX_Z, .roll=OUTPUT_MAX_Y, .yaw=OUTPUT_MAX_X};
	static angulos_mpu	limites_Min = {.pitch=OUTPUT_MIN_Z, .roll=OUTPUT_MIN_Y, .yaw=OUTPUT_MIN_X};

	uint8_t cont = TAM;

	// Inicializamos el xLastWakeTime con el tiempo actual al ingresar a la tarea

	//LastWakeTime=xTaskGetTickCount();
	while(1){
		/*
		// Espera al proximo ciclo;
		vTaskDelayUntil( &xLastWakeTime , xPeriod );

		// Realizamos el pedido de datos:
	 	Chip_I2C_MasterCmdRead(I2C1,MPU6050_DEVICE_ADDRESS,MPU6050_RA_ACCEL_XOUT_H, datos.read_buffer_accel, 6);
	 	Chip_I2C_MasterCmdRead(I2C1,MPU6050_DEVICE_ADDRESS,MPU6050_RA_GYRO_XOUT_H, datos.read_buffer_giro, 6);
	 	MPU6050_GetData( datos.buffer_gral , datos.read_buffer_accel , datos.read_buffer_giro );

	 	MPU6050_GetAngle( & datos ,& angulos ,  ((float)TIME_inS));*/

		xQueueReceive(ColaMPU, & angulos,portMAX_DELAY);

	 	//Recibir contantes por Queue:
	 	constantes_X.Kp=Px;
	 	constantes_X.Ki=Ix;
	 	constantes_X.Kd=Dx;
		constantes_Y.Kp=Py;
		constantes_Y.Ki=Iy;
		constantes_Y.Kd=Dy;
		constantes_Z.Kp=Pz;
		constantes_Z.Ki=Iz;
		constantes_Z.Kd=Dz;

		//Recibir setpoints por Queue
		SetPoints.yaw = 0;
	 	SetPoints.roll = 0;
	 	SetPoints.pitch = 0;

	 	/*promediador de TAM muestras*/
	 	angulosMPU.yaw += angulos.yaw;
	 	angulosMPU.roll += angulos.roll;
	 	angulosMPU.pitch += angulos.pitch;
	 	cont--;

	 	if(cont == 0)
		{
	 		// Obtenemos el valor promediado actual
	 		angulosMPU.yaw /=TAM;
	 		angulosMPU.roll /=TAM;
	 		angulosMPU.pitch /=TAM;
	 		//Se reinicia el contador del promediador
	 		cont=TAM;

			//Veo posicion actual de los motores:
	 		angMotor.yaw = Motor_Get(MOTOR_X);
	 		angMotor.roll = Motor_Get(MOTOR_Y);
	 		angMotor.pitch = Motor_Get(MOTOR_Z);						 
	 		//corrige discrepancia entre Motor y MPU:
	 		angulosMPU.yaw = -angulosMPU.yaw;

	 		//Se computa PID
	 		//Setpoints sin joystick inicializados en 0
			angulos=PID_Compute(angulosMPU, angMotor, SetPoints, constantes_X, constantes_Y, constantes_Z, period, limites_Min, limites_Max);

	 		//ojo con los signos!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
/*			Motor_Set(MOTOR_X, -1*(angMotor.yaw));
			Motor_Set(MOTOR_Y, (angMotor.roll));
			Motor_Set(MOTOR_Z, -1*(angMotor.pitch));*/

			Motor_Set(MOTOR_X, -1*(angulos.yaw));
			Motor_Set(MOTOR_Y, -1*(angulos.roll));
			Motor_Set(MOTOR_Z, -1*(angulos.pitch));


			/*reseteo promediado de valores del mpu*/
			angulosMPU.yaw = 0;
			angulosMPU.roll = 0;
			angulosMPU.pitch = 0;
		}
	}

}

int main(void)
{
	//Inicializamos el hardware
	initHardware();
	// Creamos la cola que nos permite acceder a los datos del acelerometro
	ColaMPU= xQueueCreate(TAM_QUEUE, sizeof(angulos_mpu));
	// Tareas para manipular los datos y calcular los angulos:
	xTaskCreate(LecturaDeDatos,(const char*) "ObtenerDatos", configMINIMAL_STACK_SIZE*2,0, tskIDLE_PRIORITY+2, NULL);
	//xTaskCreate(ProcesamientoDeDatos,(const char*) "DatosParaProcesar", configMINIMAL_STACK_SIZE*2,0, tskIDLE_PRIORITY+1, NULL);

	xTaskCreate(Estabilizador,(const char*) "Estabilizador", configMINIMAL_STACK_SIZE*2,0, tskIDLE_PRIORITY+2, NULL);

	// Iniciamos el Scheduler
	vTaskStartScheduler();

	for(;;);
	return 0;
}


/** @} doxygen end group definition */
/*==================[end of file]============================================*/
