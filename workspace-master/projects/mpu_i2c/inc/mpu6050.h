/*
 * mpu6050.h
 *
 *  Created on: 8 de ago. de 2018
 *      Author: usuario
 */

#ifndef PROJECTS_MPU_I2C_INC_MPU6050_H_
#define PROJECTS_MPU_I2C_INC_MPU6050_H_

/** @brief I2C para el MPU6050
 */

 /** \addtogroup I2C example with MPU6050
 ** @{ */

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

#define CLOCK_RATE_MPU 				400000



#define ANGLE_Z(x) x<<2
#define ANGLE_Y(x) x<<1
#define ANGLE_X(x) x<<0

#define ON 1
#define OFF 0

/*==================[ functions ]=============================================*/
void MPU6050_Init(I2C_XFER_T * xfer);
void I2C_Init(I2C_ID_T);
void MPU6050_GetData(uint16_t * samples, uint8_t * rbuf);
void MPU6050_GetAngle(uint16_t* data_in, uint8_t axis);

#endif /* PROJECTS_MPU_I2C_INC_MPU6050_H_ */
