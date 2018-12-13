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
#include "../../test_pid-mpu_save/inc/main.h"
#include "../../test_pid-mpu_save/inc/FreeRTOSConfig.h"
#include "../../test_pid-mpu_save/inc/mpu6050.h"
#include "../../test_pid-mpu_save/inc/pid.h"
#include "../../test_pid-mpu_save/inc/motor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
/* Colas para los datos del MPU*/
/* aceleracion+giroscopo */
xQueueHandle 		ColaMPU;

#define TIME_PERIOD_TASK_inTICKS	1/portTICK_RATE_MS 	// cada milisegundos

#define TIME_inS  1*portTICK_RATE_MS*0.001				// conversion de tick a segundos:

/* Datos para el control PID */
VALUES_K PID;
VALUES_ERROR error[3];
#define TAM_QUEUE 1
#define TAM 30

#define P 20
#define I 0
#define D 0

#define OUTPUT_MAX_X	90
#define OUTPUT_MIN_X	-90
#define OUTPUT_MAX_Y	90
#define OUTPUT_MIN_Y	-90
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
	PID_Init(&PID, P , I , D);
	Motor_Init(CHANNELs,FREQ_50HZ);
}

void Estabilizador (void * param)
{
	portTickType xLastWakeTime;
	const portTickType xPeriod=TIME_PERIOD_TASK_inTICKS;
	static values_mpu datos;
	static angulos_mpu angulos;
	static angulos_mpu angMotor;

	uint8_t cont = TAM;

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

	 	angMotor.yaw = (angulos.yaw + angMotor.yaw);
	 	angMotor.roll = (angulos.roll + angMotor.roll);
	 	angMotor.pitch = (angulos.pitch + angMotor.pitch);
	 	cont--;

	 	if(cont == 0)
		{
			cont=TAM;
			Motor_Set(MOTOR_X, -angMotor.yaw/TAM);
			Motor_Set(MOTOR_Y, -angMotor.roll/TAM);
			Motor_Set(MOTOR_Z, -angMotor.yaw/TAM);

			angMotor.yaw = 0;
			angMotor.roll = 0;
			angMotor.pitch = 0;
		}
	}

}



/** @brief LecturaDeDatos
 *  @details
 *  Tarea que se encarga de pedir datos al MPU
 *	@return bool
 */
/*void LecturaDeDatos (void * param)
{
	portTickType xLastWakeTime;
	const portTickType xPeriod=TIME_PERIOD_TASK_inTICKS;
	static values_mpu datos;

	static angulos_mpu angulos;

	values_mpu auxiliar;
	//static angulos_mpu angulos;
	// Inicializamos el xLastWakeTime con el tiempo actual al ingresar a la tarea
	xLastWakeTime=xTaskGetTickCount();
	while(1){

		// Espera al proximo ciclo;
		vTaskDelayUntil( &xLastWakeTime , xPeriod );

		// Realizamos el pedido de datos:
	 	Chip_I2C_MasterCmdRead(I2C1,MPU6050_DEVICE_ADDRESS,MPU6050_RA_ACCEL_XOUT_H, datos.read_buffer_accel, 6);
	 	Chip_I2C_MasterCmdRead(I2C1,MPU6050_DEVICE_ADDRESS,MPU6050_RA_GYRO_XOUT_H, datos.read_buffer_giro, 6);
	 	MPU6050_GetData( datos.buffer_gral , datos.read_buffer_accel , datos.read_buffer_giro );

	 	xQueueReceive( ColaMPU, &auxiliar, 0 );
	 	xQueueSend( ColaMPU, & datos , 0);

	}

}
*/
/** @brief ProcesamientoDeDatos
 *  @details
 *  Tarea que se encarga de convertir datos al MPU a angulos
 *	@return bool
 */


/*void ProcesamientoDeDatos (void * param)
{
	static values_mpu medicion;
	static angulos_mpu angulos;

	static float x_ant=0,y_ant=0,z_ant=0;
	static float x_out=0,y_out=0,z_out=0;

	static float error_ant[3]={0,0,0};
	static float iTime=TIME_inS;

	static bool FIRST_TIME=TRUE;

	while(1){

		 Se espera que haya algun dato en la cola para procesar
		xQueueReceive( ColaMPU , & medicion , portMAX_DELAY );

		 Se toman los valores de los angulos
		MPU6050_GetAngle( & medicion ,& angulos ,  ((float)TIME_inS));

		 Error proporcional
		error[0].p=angulos.yaw; //
		error[1].p=angulos.roll;//
		error[2].p=angulos.pitch;//

		 Error integral
		error[0].i=error[0].i + error[0].p * iTime ;
		if(error[0].i>OUTPUT_MAX_X)
			error[0].i=OUTPUT_MAX_X;
		else if(error[0].i<OUTPUT_MIN_X)
			error[0].i=OUTPUT_MIN_X;

		error[1].i=error[1].i + error[1].p * iTime ;
		if(error[0].i>OUTPUT_MAX_Y)
			error[0].i=OUTPUT_MAX_Y;
		else if(error[0].i<OUTPUT_MIN_Y)
			error[0].i=OUTPUT_MIN_Y;

		error[2].i=error[2].i + error[2].p * iTime ;
		if(error[0].i>OUTPUT_MAX_Z)
			error[0].i=OUTPUT_MAX_Z;
		else if(error[0].i<OUTPUT_MIN_Z)
			error[0].i=OUTPUT_MIN_Z;

		 Error derivativo
		No habra derivative kick ya que el setpoint no cambia
		if(FIRST_TIME){
		error[0].d= 0 ;
		error[1].d= 0 ;
		error[2].d= 0 ;
		FIRST_TIME=FALSE;
		}
		else{
		error[0].d= ( error[0].p - error_ant[0] )/ (iTime) ;
		error[1].d= ( error[1].p- error_ant[1] )/ (iTime) ;
		error[2].d= ( error[2].p - error_ant[2] )/ (iTime) ;

		}
		x_out=0;
		y_out=0;
		z_out=0;
		 Se determinan las salidas por la ecuacion tipica del pid
		x_out= PID.Kp * error[0].p + PID.Ki * error[0].i + PID.Kd * error[0].d ;
		y_out= PID.Kp * error[1].p + PID.Ki * error[1].i + PID.Kd * error[1].d ;
		z_out= PID.Kp * error[2].p + PID.Ki * error[2].i + PID.Kd * error[2].d ;

		 Se considera la posicion del motor como el punto anterior
		x_ant=(float)Motor_Get(MOTOR_X);
		y_ant=(float)Motor_Get(MOTOR_Y);
		z_ant=(float)Motor_Get(MOTOR_Z);

		//x_out += x_ant;
		//y_out += y_ant;
		//z_out += z_ant;

		Verifico no pasarme de los limites
		if(x_out>OUTPUT_MAX_X)
			x_out=OUTPUT_MAX_X;
		else if(x_out<OUTPUT_MIN_X)
			x_out=OUTPUT_MIN_X;

		if(y_out>OUTPUT_MAX_X)
			y_out=OUTPUT_MAX_X;
		else if(y_out<OUTPUT_MIN_X)
			y_out=OUTPUT_MIN_X;

		if(z_out>OUTPUT_MAX_X)
			z_out=OUTPUT_MAX_X;
		else if(z_out<OUTPUT_MIN_X)
			z_out=OUTPUT_MIN_X;*/

		/*Motor_Set(MOTOR_X, x_out);
		Motor_Set(MOTOR_Y, y_out);
		Motor_Set(MOTOR_Z, z_out);

		Pruebo con solo corregir error proporcional
		Motor_Set(MOTOR_X, angulos.yaw);
		Motor_Set(MOTOR_Y, angulos.roll);
		Motor_Set(MOTOR_Z, angulos.pitch);



		 Se guarda el error del punto anterior
		error_ant[0]= error[0].p;
		error_ant[1]= error[1].p;
		error_ant[2]= error[2].p;

		// Espera al proximo ciclo;
		//vTaskDelayUntil( &xLastWakeTime , xPeriod );
	}
}*/
int main(void)
{
	//Inicializamos el hardware
	initHardware();
	// Creamos la cola que nos permite acceder a los datos del acelerometro
	//ColaMPU= xQueueCreate(TAM_QUEUE, sizeof(values_mpu));
	// Tareas para manipular los datos y calcular los angulos:
	//xTaskCreate(LecturaDeDatos,(const char*) "ObtenerDatos", configMINIMAL_STACK_SIZE*2,0, tskIDLE_PRIORITY+2, NULL);
	//xTaskCreate(ProcesamientoDeDatos,(const char*) "DatosParaProcesar", configMINIMAL_STACK_SIZE*2,0, tskIDLE_PRIORITY+1, NULL);
	xTaskCreate(Estabilizador,(const char*) "Estabilizador", configMINIMAL_STACK_SIZE*2,0, tskIDLE_PRIORITY+2, NULL);


	/*while(1){
		Motor_Set(2, 90);
		Motor_Set(2, -90);
		Motor_Set(2, 0);
		Motor_Set(3, 90);
		Motor_Set(3, -90);
		Motor_Set(3, 0);
		Motor_Set(4, 90);
		Motor_Set(4, -90);
		Motor_Set(4, 0);

	}*/


	// Iniciamos el Scheduler
	vTaskStartScheduler();

	for(;;);
	return 0;
}


/** @} doxygen end group definition */
/*==================[end of file]============================================*/
