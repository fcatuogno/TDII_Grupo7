/*
 * main.c
 *
 *  Created on: 31 de jul. de 2018
 *      Author: usuario
 */


#ifndef _MAIN_H_
#define _MAIN_H_

#ifdef __cplusplus
extern "C" {
#endif

/** \addtogroup I2C example
 ** @{ */

/*==================[macros and definitions]=================================*/
#define MPU6050_DEVICE_ADDRESS  	 0x68
/*	Configuration	*/
#define MPU6050_RA_SMPLRT_DIV		 0x19
#define MPU6050_RA_CONFIG			 0x1A
#define MPU6050_RA_GYRO_CONFIG		 0x1B
#define MPU6050_RA_ACCEL_CONFIG	 	 0x1C

/*	data registers: accelerometer	*/
/*	x	*/
#define MPU6050_RA_ACCEL_XOUT_H  	 0x3B
#define MPU6050_RA_ACCEL_XOUT_L		 0x3C
/*	y	*/
#define MPU6050_RA_ACCEL_YOUT_H  	 0x3D
#define MPU6050_RA_ACCEL_YOUT_L		 0x3E
/*	z	*/
#define MPU6050_RA_ACCEL_ZOUT_H  	 0x3F
#define MPU6050_RA_ACCEL_ZOUT_L		 0x40
/*	data registers: gyroscope	*/
/*	x	*/
#define MPU6050_RA_GYRO_XOUT_H  	 0x43
#define MPU6050_RA_GYRO_XOUT_L		 0x44
/*	y	*/
#define MPU6050_RA_GYRO_YOUT_H  	 0x45
#define MPU6050_RA_GYRO_YOUT_L		 0x46
/*	z	*/
#define MPU6050_RA_GYRO_ZOUT_H  	 0x47
#define MPU6050_RA_GYRO_ZOUT_L		 0x48

#define MPU6050_RA_PWR_MGMT_1    	 0x6B
#define MPU6050_RA_PWR_MGMT_2    	 0x6C

#define MPU6050_PWR1_SLEEP_BIT   	 6

#define CLOCK_RATE_MPU 				400000
/**
 * I2C 0
 */
#define I2C_SCL0		0, 28
#define I2C_SDA0		0, 27

/**
 * I2C 1
 */
#define I2C_SCL1		0, 20
#define I2C_SDA1		0, 19

/**
 * I2C 2
 */
#define I2C_SCL2		0, 11
#define I2C_SDA2		0, 10


//Conversion de radianes a grados 180/PI
#define RAD_TO_DEG 57.295779

/*************************************** Constante para el filtro complementario *******************************************/
/*
 * Sensibilidad: Factor de escala de sensibilidad del giroscopio es de 131 LSB / [º/seg]
 * */

#define MPU6050_GS 131.0


/*
 * Sensibilidad: Factor de escalal de sensibilidad del acelerometro LSB/g
 * */
#define MPU6050_AS 16384.0



/*
 * Tiempo entre cada muestra: se realiza por medio del Systick Handler, cada 1 seg
 * */
#define MPU6050_DT		0.010
/*
 * BETA : constante con valor de 0.95 , esto determina que l giroscopo tenga más influencia en
 * el valor del angulo.
 */
#define MPU6050_BETA 	0.98
/*
 * w0 :Velocidad angular nivel cero. 500º/seg / 2^16 -1
 * */
#define MPU6050_W0		0.0076



typedef struct
{
	unsigned short gx;
	unsigned short gy;
	unsigned short gz;
	unsigned short pitch;
	unsigned short roll;
	unsigned short yaw;
} datos_acelerometro;

static void initHardware(void);
int main(void);

/*==================[inclusions]=============================================*/


/*==================[ functions ]=============================================*/
void Init_I2C(I2C_ID_T);
void MPU6050_Init(I2C_ID_T id);
void MPU6050_GetData(int16_t *, uint8_t *, uint8_t * );
void MPU6050_GetAngle(int16_t*,double*,double*,double*);
/*==================[cplusplus]==============================================*/


#ifdef __cplusplus
}
#endif

#endif /* _MAIN_H_ */
