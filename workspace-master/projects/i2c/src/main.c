/*
 * main.c
 *
 *  Created on: 31 de jul. de 2018
 *      Author: Daiana Valeria Casas
 */

/** @brief I2C para pruebas.
 */

 /** \addtogroup I2C example
 ** @{ */

#include "chip.h"
#include "main.h"
#include "math.h"
/** @brief hardware initialization function
 *	@return none
 */
static uint32_t pausems_count;

static void initHardware(void)
{
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock/1000);

	/* Initializes GPIO */
	Chip_GPIO_Init(LPC_GPIO);
	Chip_IOCON_Init(LPC_IOCON);

    Init_I2C(I2C1);
    Chip_I2C_SetClockRate(I2C1, CLOCK_RATE_MPU);
	Chip_I2C_SetMasterEventHandler(I2C1, Chip_I2C_EventHandlerPolling);
}

static void pausems(uint32_t t)
{
   pausems_count = t;
   while(pausems_count != 0) {
      __WFI();
   }
}
void SysTick_Handler(void)
{
   if (pausems_count != 0) {
      pausems_count--;
   }
}

int main(void)
{
// 	uint8_t rbuf[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};	/*	buffer para recibir en el MPU	*/
 	uint8_t rbuf[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};	/*	buffer para recibir en el MPU	*/
	uint16_t samples[7] = {0,0,0,0,0,0,0};	//	Cada posicion es de 16 bits, necesario para guardar	la parte low y high de las muestras de los datos
	initHardware();
	MPU6050_Init(I2C1);
	uint32_t aux_x=0,aux_y=0,aux_z=0;
	while(1)
	{
		 	 pausems(10000);
		 	 Chip_I2C_MasterCmdRead(I2C1,MPU6050_DEVICE_ADDRESS,MPU6050_RA_ACCEL_XOUT_H, rbuf, 14);
		 	 Fill_Samples(samples,rbuf);
		 	 Converter_angle(samples,&aux_x,&aux_y,&aux_z);
	}

	return 0;
}

/*
 * Configura la estructura XFER para realizar la comunicacion i2c.
 * * xfer	  : Puntero a la estructura del tipo I2C_XFER_T necesaria para utilizar la funcion Chip_I2C_MasterTransfer.
 * 				Chip_I2C_MasterTransfer : Funcion que resuelve la interaccion i2c en funcion de lo especificado en la estructura I2C_XFER_T
 *	 rbuf 	  : Puntero al buffer de lectura donde se volcaran los bytes leidos
 *	 rxSz 	  : Cantidad de bytes que se leeran y volcaran en rbuf
 *	 slaveAddr: Direccion estatica del slave con el que se desea comunicar
 *	 status   : Estado de la comunicacion, (estado inicial 0)
 *	 wbuf	  : Buffer de escritura donde se colocara tanto el registro que se desea escribir como el dato que desea ser escrito
 *	 			Ej de uso: wbuf[] = {reg_inicial, dato} solo escribe el byte dato en reg_inicial
 *	 					   wbuf[] = {reg_inicial, dato1, dato2} escribe el byte dato1 en reg_incial y dato2 en reg_inicial+1 (el registro siguiente)
 *	 txSz	  : La cantidad de bytes que se desean enviar, osea empezando a leer wbuf desde 0 inclusive, cuantos bytes manda de ese buffer
 *	 			Ej : wbuf[] = {reg_inicial, dato1, dato2}, entonces txSz deberia ser = 3
 *	 				 wbuf[] = {reg_inicial}, (caso tipico de solo lectura de ese registro), entonces txSz deberia ser = 1
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

/*
 * Se encarga de inicializar los registros de PWR_MGMT necesarios para habilitar los
 *  sensores (acelerometro y giroscopo) en cada eje, para sus lecturas.
 *  La configuracion en la que quedan seteados es la por defecto:
 *  accelerometer (±2g) , gyroscope (±250°/sec).
 *
 *  * xfer : Puntero a la estructura del tipo I2C_XFER_T necesaria para la utilizacion de Chip_I2C_MasterTransfer.
 *  		 Chip_I2C_MasterTransfer : Funcion que resuelve la interaccion i2c en funcion de lo especificado en la estructura I2C_XFER_T
 */
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

/* Llena el vector de muestras samples con la data de los registros de ACCEL, GYRO y TEMP del MPU
 * Al estar la informacion en 16 bits y ser levantada por registros de 8, en rbuf esta la parte low
 * 	y high de cada componente, por lo que se debe recomponer desplazando la parte high y concatenando la low]
 * 	para obtener el valor que se midio. Proceso que se realiza en esta funcion para pasar a samples.
 *
 * 	 rbuf : Tiene la data leida por el MPU con la data dividida en high y low
 * 	 samples : Tendra la data agrupada que representa al valor medido por los sensores en cada eje
 */
void Fill_Samples(uint16_t * samples, uint8_t * rbuf)
{
	//De momento leer rbuf es lo mismo que leer xfer.rxBuff porque apuntan a la misma direccion
	//Desplazamos la parte alta a los 8 ultimos bits y le hacemos un OR para tener en los 8 primeros la parte baja
	samples[0]=(rbuf[0] << 8) | rbuf[1];  	// acel x
	samples[1]=(rbuf[2] << 8) | rbuf[3];  	// acel y
	samples[2]=(rbuf[4] << 8) | rbuf[5];  	// acel z
	samples[3]=(rbuf[6] << 8) | rbuf[7];  	// temp
	samples[4]=(rbuf[8] << 8) | rbuf[9];  	// giros x
	samples[5]=(rbuf[10] << 8) | rbuf[11];  	// giros y
	samples[6]=(rbuf[12] << 8) | rbuf[13];  	// giros z

}


void Converter_angle(uint16_t* samples,uint32_t* angle_x,uint32_t* angle_y,uint32_t* angle_z)
{
//A partir de los valores del acelerometro, se calculan los angulos Y, X
//respectivamente, con la formula de la tangente:
// Tita: angulo de x Acc[1]
// zeta : angulo de y Acc[0]
// phi : angulo de z Acc[2]
	/*angle_x = atan(-1*((double)samples[0]/A_R)/sqrt(pow(((double)samples[1]/A_R),2) + pow(((double)samples[2]/A_R),2)))* RAD_TO_DEG;
	angle_y = atan(((double)samples[1]/A_R)/sqrt(pow(((double)samples[0]/A_R),2) + pow(((double)samples[2]/A_R),2)))*RAD_TO_DEG;
	angle_z= atan(sqrt(pow(((double)samples[0]/A_R),2) + pow(((double)samples[1]/A_R),2))/ ((double)samples[2]/A_R) ) *RAD_TO_DEG;*/


}

/** @} doxygen end group definition */
/*==================[end of file]============================================*/
