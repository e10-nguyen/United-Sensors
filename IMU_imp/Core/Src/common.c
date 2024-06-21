//#include <stdint.h>
//#include <stdlib.h>
//#include <stdio.h>
//
//#include "common.h"
//
///******************************************************************************/
///*!                       Macro definitions                                   */
//
//#define BMI08X_READ_WRITE_LEN  UINT8_C(46)
//
///*! BMI085 shuttle id */
//#define BMI085_SHUTTLE_ID      UINT16_C(0x46)
//
///*! BMI088 shuttle id */
//#define BMI088_SHUTTLE_ID      UINT16_C(0x66)
//
///******************************************************************************/
///*!                Static variable definition                                 */
//
///*! Variable that holds the I2C device address or SPI chip selection for accel */
//uint16_t acc_dev_add;
//
///*! Variable that holds the I2C device address or SPI chip selection for gyro */
//uint16_t gyro_dev_add;
//
//bmi085_handle_t bmi085_handle;
//
//void init_handle(bmi085_handle_t *handle){
//
//	bmi085_handle.spi_handle = handle->spi_handle;
//	bmi085_handle.nssa_port =  handle->nssa_port;
//	bmi085_handle.nssg_port =  handle->nssg_port;
//	bmi085_handle.nssa_pin =  handle->nssa_pin;
//	bmi085_handle.nssg_pin =  handle->nssg_pin;
//	bmi085_handle.ps_pin = handle->ps_pin;
//	bmi085_handle.ps_port =  handle->ps_port;
//	bmi085_handle.timer_ptr = handle->timer_ptr;
//}
//
////BMI08X_INTF_RET_TYPE bmi08x_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
////
//////	uint16_t dev_addr = BMI085_NSSa_Pin;
////	GPIO_TypeDef *port = bmi085_handle.nssa_port;
//////
//////	if(intf_ptr == &gyro_dev_add){
//////		port = bmi085_handle.nssg_port;
//////		dev_addr = BMI085_NSSg_Pin;
//////	}
////
////	uint16_t dev_addr =  *(uint16_t*)intf_ptr;
////
////	HAL_GPIO_WritePin(port, dev_addr, GPIO_PIN_RESET);
////
//////	uint8_t transmit_buffer = (uint8_t)reg & 0x7fu;
////
//////	if (HAL_SPI_Transmit(bmi085_handle.spi_handle, &dev_addr, 1, 50) != HAL_OK) {
//////		return false;
//////	}
////
////	if (HAL_SPI_Transmit(bmi085_handle.spi_handle, &reg_addr, 1, 50)
////			!= HAL_OK) {
////		return 1;
////	}
////
////	if (HAL_SPI_Receive(bmi085_handle.spi_handle, reg_data, len, 50)
////			!= HAL_OK) {
////		return 1;
////	}
////
////	HAL_GPIO_WritePin(port, dev_addr, GPIO_PIN_SET);
////
////	return 0;
////}
//
//
////BMI08X_INTF_RET_TYPE bmi08x_spi_write(uint8_t reg_addr, const uint8_t *reg_data,
////		uint32_t len, void *intf_ptr) {
////
//////	uint16_t dev_addr = BMI085_NSSa_Pin;
////	GPIO_TypeDef *port = GPIOC;
//////
//////	if(intf_ptr == &gyro_dev_add){
//////		port = bmi085_handle.nssg_port;
//////		dev_addr = BMI085_NSSg_Pin;
//////	}
////	uint16_t dev_addr = *(uint16_t*)intf_ptr;
////
////
////	HAL_GPIO_WritePin(port, dev_addr, GPIO_PIN_RESET);
////
////	if (HAL_SPI_Transmit(bmi085_handle.spi_handle, &reg_addr, 1, 50)
////			!= HAL_OK) {
////		return 1;
////	}
////
////	if (HAL_SPI_Transmit(bmi085_handle.spi_handle, reg_data, len, 5)
////			!= HAL_OK) {
////		return 1;
////	}
////
////	HAL_GPIO_WritePin(port, dev_addr, GPIO_PIN_SET);
////	return 0;
////}
///*!
// * Set up with a timer
// */
//void bmi08x_delay_us(uint32_t period, void *intf_ptr) {
////    coines_delay_usec(period);
//	__HAL_TIM_SET_COUNTER(bmi085_handle.timer_ptr, 0);  // set the counter value a 0
//	while (__HAL_TIM_GET_COUNTER(bmi085_handle.timer_ptr) < period)
//		;  // wait for the counter to reach the us input in the parameter
//}
//
///*!
// *  @brief Function to select the interface between SPI and I2C.
// *  Also to initialize coines platform
// */
//int8_t bmi08x_interface_init(struct bmi08x_dev *bmi08x, uint8_t intf,
//		uint8_t variant) {
//	int8_t rslt = BMI08X_OK;
////    struct coines_board_info board_info;
//
//	if (bmi08x != NULL) {
//
//
//#if defined(PC)
//        setbuf(stdout, NULL);
//#endif
//
//		HAL_Delay(10);
//
//		/* Bus configuration : I2C */
//		if (intf == BMI08X_I2C_INTF) {
////            printf("I2C Interface \n");
////
////            /* To initialize the user I2C function */
////            acc_dev_add = BMI08X_ACCEL_I2C_ADDR_PRIMARY;
////            gyro_dev_add = BMI08X_GYRO_I2C_ADDR_PRIMARY;
////            bmi08x->intf = BMI08X_I2C_INTF;
////            bmi08x->read = bmi08x_i2c_read;
////            bmi08x->write = bmi08x_i2c_write;
////
////            /* SDO pin is made low for selecting I2C address 0x76*/
////            coines_set_pin_config(COINES_SHUTTLE_PIN_8, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
////
////            coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);
////            coines_delay_msec(10);
////
////            /* PS pin is made high for selecting I2C protocol (gyroscope)*/
////            coines_set_pin_config(COINES_SHUTTLE_PIN_9, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
//		}
//		/* Bus configuration : SPI */
//		// BMI085a_NSS_Pin|BMI085g_NSS_Pin|BMI085_PS_Pin
//		else if (intf == BMI08X_SPI_INTF) {
//
//			bmi08x->intf = BMI08X_SPI_INTF;
//			bmi08x->read = bmi08x_spi_read;
//			bmi08x->write = bmi08x_spi_write;
//
//			/* SPI chip select pin for Accel (CSB1_A) */
//			acc_dev_add = BMI085_NSSa_Pin;
//
//			/* SPI chip select pin for Gyro (CSB2_G) */
//			gyro_dev_add = BMI085_NSSg_Pin;
//
//			/* CSB1 pin is made high for selecting SPI protocol (accelerometer)*/
//			// Done through STM32CubeIDE
//			HAL_GPIO_WritePin(GPIOC, BMI085_NSSa_Pin, GPIO_PIN_SET);
//			HAL_Delay(10);
////            coines_set_pin_config(COINES_SHUTTLE_PIN_8, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
//			/* CS pin is made high for selecting SPI protocol*/
//			HAL_GPIO_WritePin(GPIOC, BMI085_NSSg_Pin, GPIO_PIN_SET);
//			HAL_Delay(10);
//			// Done through STM32CubeIDE
////            coines_set_pin_config(COINES_SHUTTLE_PIN_14, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
//			/* PS pin is made low for selecting SPI protocol*/
//
////            coines_set_pin_config(COINES_SHUTTLE_PIN_9, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
////			HAL_GPIO_WritePin(GPIOC, BMI085_PS_Pin, GPIO_PIN_RESET);
////			HAL_Delay(10);
////            coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_5_MHZ, COINES_SPI_MODE3);
//		}
//
//		/* Selection of bmi085 or bmi088 sensor variant */
//		bmi08x->variant = variant;
//
//		/* Assign accel device address to accel interface pointer */
//		bmi08x->intf_ptr_accel = &acc_dev_add;
//
//		/* Assign gyro device address to gyro interface pointer */
//		bmi08x->intf_ptr_gyro = &gyro_dev_add;
//
//		/* Configure delay in microseconds */
//		bmi08x->delay_us = bmi08x_delay_us;
//
//		/* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
//		bmi08x->read_write_len = BMI08X_READ_WRITE_LEN;
//
////        coines_delay_usec(10000);
//		HAL_Delay(10);
//
////        coines_set_shuttleboard_vdd_vddio_config(3300, 3300);
//
////        coines_delay_usec(10000);
//		HAL_Delay(10);
//	} else {
//		rslt = BMI08X_E_NULL_PTR;
//	}
//
//	return rslt;
//
//}
//
///*!
// *  @brief Prints the execution status of the APIs.
// */
//void bmi08x_error_codes_print_result(const char api_name[], int8_t rslt) {
//	if (rslt != BMI08X_OK) {
//		printf("%s\t", api_name);
//		if (rslt == BMI08X_E_NULL_PTR) {
//			printf("Error [%d] : Null pointer\r\n", rslt);
//		} else if (rslt == BMI08X_E_COM_FAIL) {
//			printf("Error [%d] : Communication failure\r\n", rslt);
//		} else if (rslt == BMI08X_E_DEV_NOT_FOUND) {
//			printf("Error [%d] : Device not found\r\n", rslt);
//		} else if (rslt == BMI08X_E_OUT_OF_RANGE) {
//			printf("Error [%d] : Out of Range\r\n", rslt);
//		} else if (rslt == BMI08X_E_INVALID_INPUT) {
//			printf("Error [%d] : Invalid input\r\n", rslt);
//		} else if (rslt == BMI08X_E_CONFIG_STREAM_ERROR) {
//			printf("Error [%d] : Config stream error\r\n", rslt);
//		} else if (rslt == BMI08X_E_RD_WR_LENGTH_INVALID) {
//			printf("Error [%d] : Invalid Read write length\r\n", rslt);
//		} else if (rslt == BMI08X_E_INVALID_CONFIG) {
//			printf("Error [%d] : Invalid config\r\n", rslt);
//		} else if (rslt == BMI08X_E_FEATURE_NOT_SUPPORTED) {
//			printf("Error [%d] : Feature not supported\r\n", rslt);
//		} else if (rslt == BMI08X_W_FIFO_EMPTY) {
//			printf("Warning [%d] : FIFO empty\r\n", rslt);
//		} else {
//			printf("Error [%d] : Unknown error code\r\n", rslt);
//		}
//	}
//
//}
