/*
 * BMI160.c
 *
 *  Created on: Apr 6, 2020
 *      Author: armando
 */

#include "BMI160.h"

#define DATA_LENGTH		1U
#define SUBSIZE			1

typedef struct{
	rtos_i2c_number_t port;
	BMI160_BASE_ADDRESS_t bmi_addres;
}port_data_t;

port_data_t g_port;

BooleanType BMI160_Init(BMI160_config_t configuration)
{
	rtos_i2c_flag_t i2c_status = rtos_i2c_fail;
	i2c_status = rtos_i2c_init(configuration.i2c_port_config);
	if (rtos_i2c_fail == i2c_status)
	{
		return FALSE;
	}
	g_port.port = configuration.i2c_port_config.port;
	g_port.bmi_addres = configuration.base_address;
	/*accelerometer initialization*/
	i2c_status = rtos_i2c_transfer(configuration.i2c_port_config.i2c_number,
								   &configuration.acc_mode,
								   DATA_LENGTH,
								   configuration.base_address,
								   configuration.sub_address,
								   SUBSIZE);

	if (rtos_i2c_fail == i2c_status)
	{
		return FALSE;
	}

	/*accelerometer initialization*/
	i2c_status = rtos_i2c_transfer(configuration.i2c_port_config.i2c_number,
								   &configuration.gyro_mode,
								   DATA_LENGTH,
								   configuration.base_address,
								   configuration.sub_address,
								   SUBSIZE);

	if (rtos_i2c_fail == i2c_status)
	{
		return FALSE;
	}
	return TRUE;
}

BMI160_data_t BMI160_get_acc(void)
{
BMI160_data_t acc_data;
	uint8_t MSB = 0;
	uint8_t LSB = 0;

	//Receiving MSB and  LSB of Accelerometer X
	rtos_i2c_receive(g_port.port, &MSB, DATA_LENGTH, g_port.bmi_addres,BMI160_ACC_X_MSB_REG, SUBSIZE);
	rtos_i2c_receive(g_port.port, &LSB, DATA_LENGTH, BMI160_SLAVE_ADDRESS,BMI160_ACC_X_LSB_REG, SUBSIZE);
	acc_data.x = (MSB << 8) + LSB;

	//Receiving MSB and  LSB of Accelerometer Y
	rtos_i2c_receive(g_port.port, &MSB, DATA_LENGTH, g_port.bmi_addres,BMI160_ACC_Y_MSB_REG, SUBSIZE);
	rtos_i2c_receive(g_port.port, &LSB, DATA_LENGTH, g_port.bmi_addres,BMI160_ACC_Y_LSB_REG, SUBSIZE);
	acc_data.y = (MSB << 8) + LSB;

	//Receiving MSB and  LSB of Accelerometer Z
	rtos_i2c_receive(g_port.port, &MSB, DATA_LENGTH, g_port.bmi_addres,BMI160_ACC_Z_MSB_REG, SUBSIZE);
	rtos_i2c_receive(g_port.port, &LSB, DATA_LENGTH, g_port.bmi_addres,BMI160_ACC_Z_LSB_REG, SUBSIZE);
	acc_data.z = (MSB << 8) + LSB;

	return acc_data;
};
BMI160_data_t BMI160_get_gyro(void)
{
BMI160_data_t gyro_data;
	uint8_t MSB = 0;
	uint8_t LSB = 0;

	//Receiving MSB and  LSB of Accelerometer X
	rtos_i2c_receive(g_port.port, &MSB, DATA_LENGTH, g_port.bmi_addres,BMI160_GYRO_X_MSB_REG, SUBSIZE);
	rtos_i2c_receive(g_port.port, &LSB, DATA_LENGTH, BMI160_SLAVE_ADDRESS,BMI160_GYRO_X_LSB_REG, SUBSIZE);
	gyro_data.x = (MSB << 8) + LSB;

	//Receiving MSB and  LSB of Accelerometer Y
	rtos_i2c_receive(g_port.port, &MSB, DATA_LENGTH, g_port.bmi_addres,BMI160_GYRO_Y_MSB_REG, SUBSIZE);
	rtos_i2c_receive(g_port.port, &LSB, DATA_LENGTH, g_port.bmi_addres,BMI160_GYRO_Y_LSB_REG, SUBSIZE);
	gyro_data.y = (MSB << 8) + LSB;

	//Receiving MSB and  LSB of Accelerometer Z
	rtos_i2c_receive(g_port.port, &MSB, DATA_LENGTH, g_port.bmi_addres,BMI160_GYRO_Z_MSB_REG, SUBSIZE);
	rtos_i2c_receive(g_port.port, &LSB, DATA_LENGTH, g_port.bmi_addres,BMI160_GYRO_Z_LSB_REG, SUBSIZE);
	gyro_data.z = (MSB << 8) + LSB;

	return gyro_data;
};
