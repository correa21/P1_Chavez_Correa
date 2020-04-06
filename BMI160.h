/*
 * BMI160.h
 *
 *  Created on: Apr 6, 2020
 *      Author: armando
 */

#ifndef BMI160_H_
#define BMI160_H_


#include <stdint.h>
#include "Bits.h"

#define BMI160_MODE_t 			uint8_t
#define BMI160_BASE_ADDRESS_t 	uint8_t
#define BMI160_SUB_ADDRESS_t 	uint8_t

#define BMI160_SLAVE_ADDRESS 	0x68//BMI160 PHYSICAL ADDRESS
#define BMI160_CMD_REGISTER 	0x7E //REGISTER TO COMUNICATE WITH PMU REGISTER
#define BMI160_ACC_NORMAL_MODE 	0x11 //CMD_REG = ACCELEROMETER NORMAL MODE VALUE
#define BMI160_GYRO_NORMAL_MODE 0x15 //CMD_REG = GYROSCOPE NORMAL MODE VALUE
#define BMI160_GYRO_X_LSB_REG 	0x0C
#define BMI160_GYRO_X_MSB_REG 	0x0D
#define BMI160_GYRO_Y_LSB_REG 	0x0E
#define BMI160_GYRO_Y_MSB_REG 	0x0F
#define BMI160_GYRO_Z_LSB_REG 	0x10
#define BMI160_GYRO_Z_MSB_REG 	0x11
#define BMI160_ACC_X_LSB_REG 	0x12
#define BMI160_ACC_X_MSB_REG 	0x13
#define BMI160_ACC_Y_LSB_REG 	0x14
#define BMI160_ACC_Y_MSB_REG 	0x15
#define BMI160_ACC_Z_LSB_REG 	0x16
#define BMI160_ACC_Z_MSB_REG 	0x17

typedef struct{

	BMI160_BASE_ADDRESS_t 	base_address;
	BMI160_SUB_ADDRESS_t	sub_address;
	BMI160_MODE_t 			acc_mode;
	BMI160_MODE_t 			gyro_mode;
	rtos_i2c_config_t 		i2c_port_config;

}BMI160_config_t;

typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t z;
}BMI160_data_t;

BooleanType BMI160_Init();

#endif /* BMI160_H_ */
