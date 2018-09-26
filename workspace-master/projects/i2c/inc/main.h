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
#define EJEMPLO 1
/*==================[macros and definitions]=================================*/
#define MPU6050_DEVICE_ADDRESS  	 0x68
/*	Configuration	*/
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

#ifdef EJEMPLO
// Según ejemplo de LC:

#define CLOCK_RATE_MPU 				400000

#elif
#define CLOCK_RATE_MPU 				100000
/*==================[inclusions]=============================================*/
#endif



//Ratios de conversion
#define A_R 16384.0
#define G_R 131.0

//Conversion de radianes a grados 180/PI
#define RAD_TO_DEG 57.295779



static void initHardware(void);
static void pausems(uint32_t t);

int main(void);
/*==================[ functions ]=============================================*/
void Init_I2C(I2C_ID_T);
void MPU6050_Init(I2C_ID_T id);
void Fill_Samples(uint16_t * samples, uint8_t * rbuf);
void Converter_angle(uint16_t*,uint32_t*,uint32_t*,uint32_t*);
/*==================[cplusplus]==============================================*/


#ifdef __cplusplus
}
#endif

#endif /* _MAIN_H_ */
