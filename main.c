/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    p1_chavez_correa.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "FreeRTOSConfig.h"
#include "task.h"

/* TODO: insert other include files here. */
#include "mahony.h"
#include "BMI160.h"
#include "rtos_uart.h"
#include "Bits.h"
/* TODO: insert other definitions and declarations here. */

/**UART definitions for future configuration */
#define UART_BR			115200U
#define	UART			rtos_uart0
#define UART_PORT		rtos_uart_portB
#define UART_RX_PIN		16U
#define UART_TX_PIN		17U
#define UART_PIN_MUX	3U

/**I2C definitions for future configuration */
#define I2C_BR			115200U
#define	I2C				rtos_i2c_0
#define I2C_PORT		rtos_i2c_portB
#define I2C_SCL_PIN		2U
#define I2C_SDA_PIN		3U
#define I2C_PIN_MUX		2U




typedef struct{
	uint32_t header;
	float x;
	float y;
	float z;
}comm_msg_t;

void mainTask(void* parameters);
void setupTask(void* parameters);

/*
 * @brief   Application entry point.
 */
int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

    xTaskCreate(setupTask, "setupTask", 200, NULL, configMAX_PRIORITIES, NULL);

    vTaskStartScheduler();

    while(1) {

    }
    return 0 ;
}
void setupTask(void* parameters)
{
	rtos_uart_config_t uart_config;
//	rtos_i2c_config_t i2c_config;
	BMI160_config_t bmi160_config;

	uart_config.baudrate = UART_BR;
	uart_config.uart_number = UART;
	uart_config.port = UART_PORT;
	uart_config.rx_pin = UART_RX_PIN;
	uart_config.tx_pin = UART_TX_PIN;
	uart_config.pin_mux = UART_PIN_MUX;

	/**This i2c configuration will be use as the i2c port configuration for the bmi160 sensor*/
//	i2c_config.i2c_number = I2C;
//	i2c_config.baudrate = I2C_BR;
//	i2c_config.port = I2C_PORT;
//	i2c_config.SCL_pin = I2C_SCL_PIN;
//	i2c_config.SDA_pin = I2C_SDA_PIN;
//	i2c_config.pin_mux = I2C_PIN_MUX;

	bmi160_config.base_address = BMI160_SLAVE_ADDRESS;
	bmi160_config.sub_address = BMI160_CMD_REGISTER;
	bmi160_config.acc_mode = BMI160_ACC_NORMAL_MODE;
	bmi160_config.gyro_mode = BMI160_GYRO_NORMAL_MODE;
	//bmi160_config.i2c_port_config = i2c_config;
	bmi160_config.i2c_port_config.i2c_number = I2C;
	bmi160_config.i2c_port_config.baudrate = I2C_BR;
	bmi160_config.i2c_port_config.port = I2C_PORT;
	bmi160_config.i2c_port_config.SCL_pin = I2C_SCL_PIN;
	bmi160_config.i2c_port_config.SDA_pin = I2C_SDA_PIN;
	bmi160_config.i2c_port_config.pin_mux = I2C_PIN_MUX;

//	while(rtos_uart_init(uart_config))
//	{
//		//do nothing till the the UART port is correctly initialized
//	};
//
//	while(!BMI160_Init(bmi160_config))
//	{
//		//do nothing till the the i2c and BMI160 are correctly initialized
//	};
	rtos_uart_flag_t flag = 0;
	BooleanType      flag2 = 0;

	flag = rtos_uart_init(uart_config);
	flag2 = BMI160_Init(bmi160_config);



	xTaskCreate(mainTask, "mainTask", 200, NULL, configMAX_PRIORITIES, NULL);

	/**This task have no other purpose, so we suspend it */
	vTaskSuspend(NULL);

}


void mainTask(void* parameters)
{
	  while(1) {

	    }
}
