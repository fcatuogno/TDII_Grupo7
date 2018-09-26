/*
 * mpu6050.c
 *
 *  Created on: 19 de set. de 2018
 *      Author: usuario
 */

#include "mpu6050.h"

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
void MPU6050_wakeup(I2C_XFER_T * xfer)
{
		uint8_t wbuf[3] = {MPU6050_RA_PWR_MGMT_1, 0, 0};
		I2C_XFER_config(xfer, xfer->rxBuff, 0, MPU6050_I2C_SLAVE_ADDRESS, 0, wbuf, 3);
}

/* Llena el vector de muestras samples con la data de los registros de ACCEL, GYRO y TEMP del MPU
 * Al estar la informacion en 16 bits y ser levantada por registros de 8, en rbuf esta la parte low
 * 	y high de cada componente, por lo que se debe recomponer desplazando la parte high y concatenando la low]
 * 	para obtener el valor que se midio. Proceso que se realiza en esta funcion para pasar a samples.
 *
 * 	 rbuf : Tiene la data leida por el MPU con la data dividida en high y low
 * 	 samples : Tendra la data agrupada que representa al valor medido por los sensores en cada eje
 */
void MPU6050_GetData(uint16_t * samples, uint8_t * rbuf)
{
	samples[0]=(rbuf[0] << 8) | rbuf[1];
	samples[1]=(rbuf[2] << 8) | rbuf[3];
	samples[2]=(rbuf[4] << 8) | rbuf[5];
	samples[3]=(rbuf[6] << 8) | rbuf[7];
	samples[4]=(rbuf[8] << 8) | rbuf[9];
	samples[5]=(rbuf[10] << 8) | rbuf[11];
	samples[6]=(rbuf[12] << 8) | rbuf[13];
}

/*
 Se realiza la conversion de la aceleracion al angulo correspondiente.
 Se pasa la refencia de la variable a llenar y un BYTE de ejes.
 El BYTE  de ejes pertenece a un mapa de bits correspondiente a que eje
 se desea pedir.

 axis= ANGLE_X(...)|ANGLE_Y(...)|ANGLE_Z(...);

 donde: ... : ON-> 1 , OFF-> 0
 * */
void MPU6050_GetAngle(uint16_t*data_in, uint8_t axis)
{
	swit
}
