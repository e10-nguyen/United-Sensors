/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "bmi08x.h"
#include "bmi08x_defs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define GRAVITY_EARTH  (9.80665f)

/******************************************************************************/
/*!                       Macro definitions                                   */

#define BMI08X_READ_WRITE_LEN  UINT8_C(46)

/*! BMI085 shuttle id */
#define BMI085_SHUTTLE_ID      UINT16_C(0x46)

/*! BMI088 shuttle id */
#define BMI088_SHUTTLE_ID      UINT16_C(0x66)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

USART_HandleTypeDef husart2;

/* USER CODE BEGIN PV */
/*********************************************************************/
/* global variables */
/*********************************************************************/
unsigned char data_sync_int = false;
unsigned char accel_data_ready = false;
unsigned char gyro_data_ready = false;

/*! @brief This structure containing relevant bmi08x info */
struct bmi08x_dev bmi08xdev;

/*! bmi08x int config */
struct bmi08x_int_cfg int_config;

/*Data Sync configuration object*/
struct bmi08x_data_sync_cfg sync_cfg;

/*! bmi08x accel int config */
struct bmi08x_accel_int_channel_cfg accel_int_config;

/*! bmi08x gyro int config */
struct bmi08x_gyro_int_channel_cfg gyro_int_config;

/*! @brief variable to hold the bmi08x accel data */
struct bmi08x_sensor_data bmi08x_accel;

/*! @brief variable to hold the bmi08x gyro data */
struct bmi08x_sensor_data bmi08x_gyro;

///*! Variable that holds the I2C device address or SPI chip selection for accel */
uint16_t acc_dev_add;
//
///*! Variable that holds the I2C device address or SPI chip selection for gyro */
uint16_t gyro_dev_add;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_Init(void);
static void MX_USB_OTG_FS_USB_Init(void);
/* USER CODE BEGIN PFP */
/*********************************************************************/
/* function declarations */
/*********************************************************************/
BMI08X_INTF_RET_TYPE bmi08x_spi_read(uint8_t reg_addr, uint8_t *reg_data,
		uint32_t len, void *intf_ptr);

/*!
 *  @brief Function for writing the sensor's registers through SPI bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[in] reg_data     : Pointer to the data buffer whose data has to be written.
 *  @param[in] length       : No of bytes to write.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BMI08X_INTF_RET_SUCCESS -> Success
 *  @retval != BMI08X_INTF_RET_SUCCESS  -> Failure Info
 *
 */
BMI08X_INTF_RET_TYPE bmi08x_spi_write(uint8_t reg_addr, const uint8_t *reg_data,
		uint32_t len, void *intf_ptr);

/*!
 * @brief This function provides the delay for required time (Microsecond) as per the input provided in some of the
 * APIs.
 *
 *  @param[in] period_us    : The required wait time in microsecond.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return void.
 *
 */
void bmi08x_delay_us(uint32_t period, void *intf_ptr);

/*!
 *  @brief Function to select the interface between SPI and I2C.
 *
 *  @param[in] bma      : Structure instance of bmi08x_dev
 *  @param[in] intf     : Interface selection parameter
 *                          For I2C : BMI08X_I2C_INTF
 *                          For SPI : BMI08X_SPI_INTF
 *  @param[in] variant  : Sensor variant parameter
 *                          For BMI085 : BMI085_VARIANT
 *                          For BMI088 : BMI088_VARIANT
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
int8_t bmi08x_interface_init(struct bmi08x_dev *bma, uint8_t intf,
		uint8_t variant);

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : Name of the API whose execution status has to be printed.
 *  @param[in] rslt     : Error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void bmi08x_error_codes_print_result(const char api_name[], int8_t rslt);

/*!
 *  @brief This internal function converts lsb to meter per second squared for 16 bit accelerometer for
 *  range 2G, 4G, 8G or 16G.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] g_range   : Gravity range.
 *  @param[in] bit_width : Resolution for accel.
 *
 *  @return Gravity.
 */
static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width);

/*!
 *  @brief This function converts lsb to degree per second for 16 bit gyro at
 *  range 125, 250, 500, 1000 or 2000dps.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] dps       : Degree per second.
 *  @param[in] bit_width : Resolution for gyro.
 *
 *  @return Degree per second.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);

/*!
 * @brief    This internal API is used to initialize the bmi08x sensor
 */
static int8_t init_bmi08x_sync(void);
static int8_t init_bmi08x_get_data(void);

/*!
 * @brief    BMI08x data sync. interrupt callback
 */
void bmi08x_data_sync_int();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
	HAL_USART_Transmit(&husart2, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
}
/*********************************************************************/
/* functions */
/*********************************************************************/

/*!
 *  @brief This internal API is used to initializes the bmi08x sensor
 *  settings like power mode and OSRS settings.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static int8_t init_bmi08x_get_data(void) {
	int8_t rslt;

	rslt = bmi08a_init(&bmi08xdev);
	bmi08x_error_codes_print_result("bmi08a_init", rslt);
	printf("Accel init\n\r");

//	if (rslt == BMI08X_OK) {

		rslt = bmi08g_init(&bmi08xdev);
		bmi08x_error_codes_print_result("bmi08g_init", rslt);
		printf("Gyro init\n\r");
//	}
	bmi08x_delay_us(5000,0);
//	printf("help\n");
//    if (rslt == BMI08X_OK)
//    {
//        printf("Uploading config file !\n");
//        rslt = bmi08a_load_config_file(&bmi08xdev);
//        bmi08x_error_codes_print_result("bmi08a_load_config_file", rslt);
//    }

	if (rslt == BMI08X_OK) {
		bmi08xdev.accel_cfg.odr = BMI08X_ACCEL_ODR_1600_HZ;

		if (bmi08xdev.variant == BMI085_VARIANT) {
			bmi08xdev.accel_cfg.range = BMI085_ACCEL_RANGE_16G;
		} else if (bmi08xdev.variant == BMI088_VARIANT) {
			bmi08xdev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
		}

		bmi08xdev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE; /*user_accel_power_modes[user_bmi088_accel_low_power]; */
		bmi08xdev.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL; /* Bandwidth and OSR are same */

		rslt = bmi08a_set_power_mode(&bmi08xdev);
		bmi08x_error_codes_print_result("bmi08a_set_power_mode", rslt);

		rslt = bmi08a_set_meas_conf(&bmi08xdev);
		bmi08x_error_codes_print_result("bmi08a_set_meas_conf", rslt);

		bmi08xdev.gyro_cfg.odr = BMI08X_GYRO_BW_230_ODR_2000_HZ;
		bmi08xdev.gyro_cfg.range = BMI08X_GYRO_RANGE_250_DPS;
		bmi08xdev.gyro_cfg.bw = BMI08X_GYRO_BW_230_ODR_2000_HZ;
		bmi08xdev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;

		rslt = bmi08g_set_power_mode(&bmi08xdev);
		bmi08x_error_codes_print_result("bmi08g_set_power_mode", rslt);

		rslt = bmi08g_set_meas_conf(&bmi08xdev);
		bmi08x_error_codes_print_result("bmi08g_set_meas_conf", rslt);
	}

	return rslt;

}

/*!
 *  @brief This internal API is used to initializes the bmi08x sensor with default.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static int8_t init_bmi08x_sync(void) {
	int8_t rslt;

	/* Initialize bmi08a */
	rslt = bmi08a_init(&bmi08xdev);
	printf("%d",&bmi08xdev);
	bmi08x_error_codes_print_result("bmi08a_init", rslt);

	/* Initialize bmi08g */
	rslt = bmi08g_init(&bmi08xdev);
	bmi08x_error_codes_print_result("bmi08g_init", rslt);

	if (rslt == BMI08X_OK) {
		printf("BMI08x initialization success!\n");
		printf("Accel chip ID - 0x%x\n", bmi08xdev.accel_chip_id);
		printf("Gyro chip ID - 0x%x\n", bmi08xdev.gyro_chip_id);

		/* Reset the accelerometer */
		rslt = bmi08a_soft_reset(&bmi08xdev);
		bmi08x_error_codes_print_result("bmi08a_soft_reset", rslt);

		/* Read/write length */
		bmi08xdev.read_write_len = 32;

		printf("Uploading BMI08X data synchronization feature config !\n");
		rslt = bmi08a_load_config_file(&bmi08xdev);
		bmi08x_error_codes_print_result("bmi08a_load_config_file", rslt);

		/* Set accel power mode */
		bmi08xdev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
		rslt = bmi08a_set_power_mode(&bmi08xdev);
		bmi08x_error_codes_print_result("bmi08a_set_power_mode", rslt);

		if (rslt == BMI08X_OK) {
			bmi08xdev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
			rslt = bmi08g_set_power_mode(&bmi08xdev);
			bmi08x_error_codes_print_result("bmi08g_set_power_mode", rslt);
		}

		if ((bmi08xdev.accel_cfg.power == BMI08X_ACCEL_PM_ACTIVE)
				&& (bmi08xdev.gyro_cfg.power == BMI08X_GYRO_PM_NORMAL)) {
			/* API uploads the bmi08x config file onto the device */
			if (rslt == BMI08X_OK) {
				/* Assign accel range setting */
				if (bmi08xdev.variant == BMI085_VARIANT) {
					bmi08xdev.accel_cfg.range = BMI085_ACCEL_RANGE_16G;
				} else if (bmi08xdev.variant == BMI088_VARIANT) {
					bmi08xdev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
				}

				/* Assign gyro range setting */
				bmi08xdev.gyro_cfg.range = BMI08X_GYRO_RANGE_2000_DPS;

				/* Mode (0 = off, 1 = 400Hz, 2 = 1kHz, 3 = 2kHz) */
				sync_cfg.mode = BMI08X_ACCEL_DATA_SYNC_MODE_400HZ;

				rslt = bmi08a_configure_data_synchronization(sync_cfg,
						&bmi08xdev);
				bmi08x_error_codes_print_result(
						"bmi08a_configure_data_synchronization", rslt);
			}

			if (rslt == BMI08X_OK) {
				printf("BMI08x data synchronization feature configured !\n\n");
			} else {
				printf(
						"BMI08x data synchronization feature configuration failure!\n\n");
			}
		}
	}

	return rslt;
}
/*!
 *  @brief This API is used to enable bmi08x interrupt
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static int8_t enable_bmi08x_interrupt() {
	int8_t rslt;
	uint8_t data = 0;

	/* Set accel interrupt pin configuration */
	accel_int_config.int_channel = BMI08X_INT_CHANNEL_1;
	accel_int_config.int_type = BMI08X_ACCEL_INT_DATA_RDY;
	accel_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
	accel_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
	accel_int_config.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

	/* Enable accel data ready interrupt channel */
	rslt = bmi08a_set_int_config(
			(const struct bmi08x_accel_int_channel_cfg*) &accel_int_config,
			&bmi08xdev);
	bmi08x_error_codes_print_result("bmi08a_set_int_config", rslt);

	if (rslt == BMI08X_OK) {
		/* Set gyro interrupt pin configuration */
		gyro_int_config.int_channel = BMI08X_INT_CHANNEL_3;
		gyro_int_config.int_type = BMI08X_GYRO_INT_DATA_RDY;
		gyro_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
		gyro_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
		gyro_int_config.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

		/* Enable gyro data ready interrupt channel */
		rslt = bmi08g_set_int_config(
				(const struct bmi08x_gyro_int_channel_cfg*) &gyro_int_config,
				&bmi08xdev);
		bmi08x_error_codes_print_result("bmi08g_set_int_config", rslt);

		rslt = bmi08g_get_regs(BMI08X_REG_GYRO_INT3_INT4_IO_MAP, &data, 1,
				&bmi08xdev);
		bmi08x_error_codes_print_result("bmi08g_get_regs", rslt);
	}

	return rslt;
}

/*!
 *  @brief This API is used to disable bmi08x interrupt
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static int8_t disable_bmi08x_interrupt() {
	int8_t rslt;

	/* Set accel interrupt pin configuration */
	accel_int_config.int_channel = BMI08X_INT_CHANNEL_1;
	accel_int_config.int_type = BMI08X_ACCEL_INT_DATA_RDY;
	accel_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
	accel_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
	accel_int_config.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

	/* Disable accel data ready interrupt channel */
	rslt = bmi08a_set_int_config(
			(const struct bmi08x_accel_int_channel_cfg*) &accel_int_config,
			&bmi08xdev);
	bmi08x_error_codes_print_result("bmi08a_set_int_config", rslt);

	if (rslt == BMI08X_OK) {
		/* Set gyro interrupt pin configuration */
		gyro_int_config.int_channel = BMI08X_INT_CHANNEL_3;
		gyro_int_config.int_type = BMI08X_GYRO_INT_DATA_RDY;
		gyro_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
		gyro_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
		gyro_int_config.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

		/* Disable gyro data ready interrupt channel */
		rslt = bmi08g_set_int_config(
				(const struct bmi08x_gyro_int_channel_cfg*) &gyro_int_config,
				&bmi08xdev);
		bmi08x_error_codes_print_result("bmi08g_set_int_config", rslt);
	}

	return rslt;
}

/*!
 *  @brief This internal API is used to enable data synchronization interrupt.
 *
 *  @return int8_t
 *
 */
static int8_t enable_bmi08x_data_synchronization_interrupt() {
	int8_t rslt = BMI08X_OK;

	/* Set accel interrupt pin configuration */
	/* Configure host data ready interrupt */
	int_config.accel_int_config_1.int_channel = BMI08X_INT_CHANNEL_2;
	int_config.accel_int_config_1.int_type = BMI08X_ACCEL_SYNC_INPUT;
	int_config.accel_int_config_1.int_pin_cfg.output_mode =
	BMI08X_INT_MODE_PUSH_PULL;
	int_config.accel_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
	int_config.accel_int_config_1.int_pin_cfg.enable_int_pin =
	BMI08X_ENABLE;

	/* Configure Accel syncronization input interrupt pin */
	int_config.accel_int_config_2.int_channel = BMI08X_INT_CHANNEL_1;
	int_config.accel_int_config_2.int_type = BMI08X_ACCEL_INT_SYNC_DATA_RDY;
	int_config.accel_int_config_2.int_pin_cfg.output_mode =
	BMI08X_INT_MODE_PUSH_PULL;
	int_config.accel_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
	int_config.accel_int_config_2.int_pin_cfg.enable_int_pin =
	BMI08X_ENABLE;

	/* Set gyro interrupt pin configuration */
	int_config.gyro_int_config_1.int_channel = BMI08X_INT_CHANNEL_4;
	int_config.gyro_int_config_1.int_type = BMI08X_GYRO_INT_DATA_RDY;
	int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;
	int_config.gyro_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
	int_config.gyro_int_config_1.int_pin_cfg.output_mode =
	BMI08X_INT_MODE_PUSH_PULL;

	int_config.gyro_int_config_2.int_channel = BMI08X_INT_CHANNEL_3;
	int_config.gyro_int_config_2.int_type = BMI08X_GYRO_INT_DATA_RDY;
	int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin =
	BMI08X_DISABLE;
	int_config.gyro_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
	int_config.gyro_int_config_2.int_pin_cfg.output_mode =
	BMI08X_INT_MODE_PUSH_PULL;

	/* Enable synchronization interrupt pin */
	rslt = bmi08a_set_data_sync_int_config(&int_config, &bmi08xdev);
	bmi08x_error_codes_print_result("bmi08a_set_data_sync_int_config", rslt);

	return rslt;
}

/*!
 *  @brief This internal API is used to disable data synchronization interrupt.
 *
 *  @param[in] void
 *
 *  @return int8_t
 *
 */
static int8_t disable_bmi08x_data_synchronization_interrupt() {
	int8_t rslt;

	/*turn off the sync feature*/
	sync_cfg.mode = BMI08X_ACCEL_DATA_SYNC_MODE_OFF;

	rslt = bmi08a_configure_data_synchronization(sync_cfg, &bmi08xdev);
	bmi08x_error_codes_print_result("bmi08a_configure_data_synchronization",
			rslt);

	/* Wait for 150ms to enable the data synchronization --delay taken care inside the function */
	/* configure synchronization interrupt pins */
	if (rslt == BMI08X_OK) {
		/* Set accel interrupt pin configuration */
		/* Configure host data ready interrupt */
#if defined(MCU_APP20)
	        int_config.accel_int_config_1.int_channel = BMI08X_INT_CHANNEL_1;
	    #elif defined(MCU_APP30)
	        int_config.accel_int_config_1.int_channel = BMI08X_INT_CHANNEL_2;
	    #endif
		int_config.accel_int_config_1.int_type = BMI08X_ACCEL_SYNC_INPUT;
		int_config.accel_int_config_1.int_pin_cfg.output_mode =
		BMI08X_INT_MODE_PUSH_PULL;
		int_config.accel_int_config_1.int_pin_cfg.lvl =
		BMI08X_INT_ACTIVE_HIGH;
		int_config.accel_int_config_1.int_pin_cfg.enable_int_pin =
		BMI08X_DISABLE;

		/* Configure Accel synchronization input interrupt pin */
#if defined(MCU_APP20)
	        int_config.accel_int_config_2.int_channel = BMI08X_INT_CHANNEL_2;
	    #elif defined(MCU_APP30)
	        int_config.accel_int_config_2.int_channel = BMI08X_INT_CHANNEL_1;
	    #endif
		int_config.accel_int_config_2.int_type = BMI08X_ACCEL_INT_SYNC_DATA_RDY;
		int_config.accel_int_config_2.int_pin_cfg.output_mode =
		BMI08X_INT_MODE_PUSH_PULL;
		int_config.accel_int_config_2.int_pin_cfg.lvl =
		BMI08X_INT_ACTIVE_HIGH;
		int_config.accel_int_config_2.int_pin_cfg.enable_int_pin =
		BMI08X_DISABLE;

		/* Set gyro interrupt pin configuration */
#if defined(MCU_APP20)
	        int_config.gyro_int_config_1.int_channel = BMI08X_INT_CHANNEL_3;
	    #elif defined(MCU_APP30)
	        int_config.gyro_int_config_1.int_channel = BMI08X_INT_CHANNEL_4;
	    #endif
		int_config.gyro_int_config_1.int_type = BMI08X_GYRO_INT_DATA_RDY;
		int_config.gyro_int_config_1.int_pin_cfg.lvl =
		BMI08X_INT_ACTIVE_HIGH;
		int_config.gyro_int_config_1.int_pin_cfg.output_mode =
		BMI08X_INT_MODE_PUSH_PULL;
		int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin =
		BMI08X_DISABLE;

#if defined(MCU_APP20)
	        int_config.gyro_int_config_2.int_channel = BMI08X_INT_CHANNEL_4;
	    #elif defined(MCU_APP30)
	        int_config.gyro_int_config_2.int_channel = BMI08X_INT_CHANNEL_3;
	    #endif
		int_config.gyro_int_config_2.int_type = BMI08X_GYRO_INT_DATA_RDY;
		int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin =
		BMI08X_DISABLE;
		int_config.gyro_int_config_2.int_pin_cfg.lvl =
		BMI08X_INT_ACTIVE_HIGH;
		int_config.gyro_int_config_2.int_pin_cfg.output_mode =
		BMI08X_INT_MODE_PUSH_PULL;

		/* Disable synchronization interrupt pin */
		rslt = bmi08a_set_data_sync_int_config(&int_config, &bmi08xdev);
		bmi08x_error_codes_print_result("bmi08a_set_data_sync_int_config",
				rslt);
	}

	return rslt;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */

  MX_USART2_Init();

  /* USER CODE BEGIN 2 */
	printf("Hello\n\r");

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  husart2.Instance = USART2;
  husart2.Init.BaudRate = 115200;
  husart2.Init.WordLength = USART_WORDLENGTH_8B;
  husart2.Init.StopBits = USART_STOPBITS_1;
  husart2.Init.Parity = USART_PARITY_NONE;
  husart2.Init.Mode = USART_MODE_TX_RX;
  husart2.Init.CLKPolarity = USART_POLARITY_LOW;
  husart2.Init.CLKPhase = USART_PHASE_1EDGE;
  husart2.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  husart2.Init.ClockPrescaler = USART_PRESCALER_DIV1;
  husart2.SlaveMode = USART_SLAVEMODE_ENABLE;
  if (HAL_USART_Init(&husart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_SetTxFifoThreshold(&husart2, USART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_SetRxFifoThreshold(&husart2, USART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_DisableFifoMode(&husart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_EnableSlaveMode(&husart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA11 PA12 */
//  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
//  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}

// These functions are the only ones that really need to be implemented
BMI08X_INTF_RET_TYPE bmi08x_spi_read(uint8_t reg_addr, uint8_t *reg_data,
		uint32_t len, void *intf_ptr) {

	uint16_t dev_addr = *(uint16_t*) intf_ptr;

	HAL_GPIO_WritePin(GPIOC, dev_addr, GPIO_PIN_RESET);

	if (HAL_SPI_Transmit(&hspi1, &reg_addr, 1, 50) != HAL_OK) {
		return 1;
	}
	if (HAL_SPI_Receive(&hspi1, reg_data, len, 50) != HAL_OK) {
		return 1;
	}

	HAL_GPIO_WritePin(GPIOC, dev_addr, GPIO_PIN_SET);
	bmi08x_delay_us(100, 0);
	return 0;
}

BMI08X_INTF_RET_TYPE bmi08x_spi_write(uint8_t reg_addr, const uint8_t *reg_data,
		uint32_t len, void *intf_ptr) {

	uint16_t dev_addr = *(uint16_t*) intf_ptr;

	HAL_GPIO_WritePin(GPIOC, dev_addr, GPIO_PIN_RESET);

	if (HAL_SPI_Transmit(&hspi1, &reg_addr, 1, 50) != HAL_OK) {
		return 1;
	}
	if (HAL_SPI_Transmit(&hspi1, reg_data, 1, 5) != HAL_OK) {
		return 1;
	}

	HAL_GPIO_WritePin(GPIOC, dev_addr, GPIO_PIN_SET);

	bmi08x_delay_us(100, 0);
	return 0;
}

void bmi08x_delay_us(uint32_t period, void *intf_ptr) {
//    coines_delay_usec(period);
	__HAL_TIM_SET_COUNTER(&htim2, 0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim2) < period)
		;  // wait for the counter to reach the us input in the parameter
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 *  Also to initialize coines platform
 */
int8_t bmi08x_interface_init(struct bmi08x_dev *bmi08x, uint8_t intf,
		uint8_t variant) {
	int8_t rslt = BMI08X_OK;
//    struct coines_board_info board_info;

	if (bmi08x != NULL) {

#if defined(PC)
        setbuf(stdout, NULL);
#endif

		bmi08x_delay_us(1000, 0);

		/* Bus configuration : I2C */
		if (intf == BMI08X_I2C_INTF) {
//            printf("I2C Interface \n");
//
//            /* To initialize the user I2C function */
//            acc_dev_add = BMI08X_ACCEL_I2C_ADDR_PRIMARY;
//            gyro_dev_add = BMI08X_GYRO_I2C_ADDR_PRIMARY;
//            bmi08x->intf = BMI08X_I2C_INTF;
//            bmi08x->read = bmi08x_i2c_read;
//            bmi08x->write = bmi08x_i2c_write;
//
//            /* SDO pin is made low for selecting I2C address 0x76*/
//            coines_set_pin_config(COINES_SHUTTLE_PIN_8, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
//
//            coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);
//            coines_delay_msec(10);
//
//            /* PS pin is made high for selecting I2C protocol (gyroscope)*/
//            coines_set_pin_config(COINES_SHUTTLE_PIN_9, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
		}
		/* Bus configuration : SPI */
		// BMI085a_NSS_Pin|BMI085g_NSS_Pin|BMI085_PS_Pin
		else if (intf == BMI08X_SPI_INTF) {

			bmi08x->intf = BMI08X_SPI_INTF;
			bmi08x->read = bmi08x_spi_read;
			bmi08x->write = bmi08x_spi_write;

			/* SPI chip select pin for Accel (CSB1_A) */
			acc_dev_add = BMI085_NSSa_Pin;

			/* SPI chip select pin for Gyro (CSB2_G) */
			gyro_dev_add = BMI085_NSSg_Pin;

			/* CSB1 pin is made high for selecting SPI protocol (accelerometer)*/
			// Done through STM32CubeIDE
			//HAL_GPIO_WritePin(GPIOF, BMI085_NSSa_Pin, GPIO_PIN_SET);
			bmi08x_delay_us(1000, 00);
//            coines_set_pin_config(COINES_SHUTTLE_PIN_8, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
			/* CS pin is made high for selecting SPI protocol*/
//			HAL_GPIO_WritePin(GPIOF, BMI085_NSSg_Pin, GPIO_PIN_SET);
//			HAL_Delay(10);
			// Done through STM32CubeIDE
//            coines_set_pin_config(COINES_SHUTTLE_PIN_14, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
			/* PS pin is made low for selecting SPI protocol*/

//            coines_set_pin_config(COINES_SHUTTLE_PIN_9, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
//			HAL_GPIO_WritePin(GPIOF, BMI085_PS_Pin, GPIO_PIN_RESET);
//			HAL_Delay(10);
//            coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_5_MHZ, COINES_SPI_MODE3);
		}

		/* Selection of bmi085 or bmi088 sensor variant */
		bmi08x->variant = variant;

		/* Assign accel device address to accel interface pointer */
		bmi08x->intf_ptr_accel = &acc_dev_add;

		/* Assign gyro device address to gyro interface pointer */
		bmi08x->intf_ptr_gyro = &gyro_dev_add;

		/* Configure delay in microseconds */
		bmi08x->delay_us = bmi08x_delay_us;

		/* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
		bmi08x->read_write_len = BMI08X_READ_WRITE_LEN;

//        coines_delay_usec(10000);
		bmi08x_delay_us(1000, 0);

//        coines_set_shuttleboard_vdd_vddio_config(3300, 3300);

//        coines_delay_usec(10000);
		bmi08x_delay_us(1000, 0);
	} else {
		rslt = BMI08X_E_NULL_PTR;
	}

	return rslt;

}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bmi08x_error_codes_print_result(const char api_name[], int8_t rslt) {
	if (rslt != BMI08X_OK) {
		printf("%s\t", api_name);
		if (rslt == BMI08X_E_NULL_PTR) {
			printf("Error [%d] : Null pointer\r\n", rslt);
		} else if (rslt == BMI08X_E_COM_FAIL) {
			printf("Error [%d] : Communication failure\r\n", rslt);
		} else if (rslt == BMI08X_E_DEV_NOT_FOUND) {
			printf("Error [%d] : Device not found\r\n", rslt);
		} else if (rslt == BMI08X_E_OUT_OF_RANGE) {
			printf("Error [%d] : Out of Range\r\n", rslt);
		} else if (rslt == BMI08X_E_INVALID_INPUT) {
			printf("Error [%d] : Invalid input\r\n", rslt);
		} else if (rslt == BMI08X_E_CONFIG_STREAM_ERROR) {
			printf("Error [%d] : Config stream error\r\n", rslt);
		} else if (rslt == BMI08X_E_RD_WR_LENGTH_INVALID) {
			printf("Error [%d] : Invalid Read write length\r\n", rslt);
		} else if (rslt == BMI08X_E_INVALID_CONFIG) {
			printf("Error [%d] : Invalid config\r\n", rslt);
		} else if (rslt == BMI08X_E_FEATURE_NOT_SUPPORTED) {
			printf("Error [%d] : Feature not supported\r\n", rslt);
		} else if (rslt == BMI08X_W_FIFO_EMPTY) {
			printf("Warning [%d] : FIFO empty\r\n", rslt);
		} else {
			printf("Error [%d] : Unknown error code\r\n", rslt);
		}
	}

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */