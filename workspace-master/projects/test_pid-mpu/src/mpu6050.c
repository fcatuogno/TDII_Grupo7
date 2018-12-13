/*
 * mpu6050.c
 *
 *  Created on: 30 de nov. de 2018
 *      Author: Daiana
 *     Deatils: Funciones para la obtencion de datos del mpu
 */

#include "chip.h"
#include "math.h"
#include "mpu6050.h"
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

/** @brief 	Devuelve los datos del modulo MPU6050
 *  @param	samples : vector donde se encuentran los datos ya concatenados.
 *  @param  rbuf_a    : vector de la comunicacion i2s donde se encuentran las partes de los datos de aceleracion.
 *  @param  rbuf_g    : vector de la comunicacion i2s donde se encuentran las partes de los datos del giroscopo.
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

void MPU6050_GetAngle(values_mpu* samples,angulos_mpu* angle, float time)
{
	float accel_X=0,accel_Y=0,accel_Z=0;
	static float ant_angleX=0,ant_angleY=0,ant_angleZ=0;

	accel_X = atan2((double)(samples->buffer_gral[0]),sqrt(pow( samples->buffer_gral[1],2) + pow( samples->buffer_gral[2],2))) * RAD_TO_DEG;
	accel_Y = atan2((double)(samples->buffer_gral[1]),sqrt(pow( samples->buffer_gral[0],2) + pow(samples->buffer_gral[2],2)))*RAD_TO_DEG;
	accel_Z = 0;//atan2((double)sqrt(pow(samples->buffer_gral[0],2) + pow( samples->buffer_gral[1],2) ),samples->buffer_gral[2] ) *RAD_TO_DEG;

	(angle->yaw)=accel_Y*(1 - MPU6050_BETA)+ MPU6050_BETA*(( samples->buffer_gral[4] / MPU6050_GS)* time + ant_angleX);//angle_y
	(angle->roll)=accel_X*(1 - MPU6050_BETA)+ MPU6050_BETA*(( samples->buffer_gral[3] / MPU6050_GS)* time + ant_angleY); //angle_x
	(angle->pitch)=/*accel_Z*(1 - MPU6050_BETA)+ MPU6050_BETA**/(( samples->buffer_gral[5] / MPU6050_GS)* time + ant_angleZ);//angle_z

	ant_angleY=(angle->roll);
	ant_angleX=(angle->yaw);
	ant_angleZ=(angle->pitch);
}

