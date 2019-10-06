/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : I2C Application for BMP180 based on HAL Library 
	* @credits				: Adapted from sparkfun arduino BMP180 Library
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "bmp180.h"
#include "math.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
HAL_StatusTypeDef i2c_status, i2c_device_stat;
uint8_t buffer[2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static int bmp_init(void);
static int bmp_readInt(uint16_t reg, int16_t *val);
static int bmp_readUInt(uint16_t reg, uint16_t *value);
static int bmp_readBytes(uint8_t *values, uint16_t numberOfBytes);
static int bmp_writeBytes(uint8_t *val, uint8_t length);
static int bmp_start_temperature_reading(void);
static int bmp_get_temperature_reading(double *T);
static int bmp_start_pressure_reading(uint8_t oversampling);
static int bmp_get_pressure_reading(double *P, double *T);
static int bmp_compute_sealevel(double P, double A);
static int bmp_compute_altitude(double P, double P0);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/**
	*	@brief API initialise BMP180 and read calibration data from its E2PROM Memory 
	* @param none 
	* @return 0 on successful execution, -1 otherwise
	*/
static int bmp_init(void)
{
	/* Read calibration constants from BMP180 E2PROM */
	if (bmp_readInt(0xAA,&AC1) &&
				bmp_readInt(0xAC,&AC2) && 
				bmp_readInt(0xAE,&AC3) && 
				bmp_readUInt(0xB0,&AC4) && 
				bmp_readUInt(0xB2,&AC5) && 
				bmp_readUInt(0xB4,&AC6) &&
				bmp_readInt(0xB6,&VB1) &&
				bmp_readInt(0xB8,&VB2) && 
				bmp_readInt(0xBA,&MB) && 
				bmp_readInt(0xBC,&MC) &&
				bmp_readInt(0xBE,&MD)) {
					
					/* On successful reading of calibration data */	
					/* compute values of floating point polynomials */
					c3 = 160.0 * pow(2,-15) * AC3;
					c4 = pow(10,-3) * pow(2,-15) * AC4;
					b1 = pow(160,2) * pow(2,-30) * VB1;
					c5 = (pow(2,-15) / 160) * AC5;
					c6 = AC6;
					mc = (pow(2,11) / pow(160,2)) * MC;
					md = MD / 160.0;
					x0 = AC1;
					x1 = 160.0 * pow(2,-13) * AC2;
					x2 = pow(160,2) * pow(2,-25) * VB2;
					y0 = c4 * pow(2,15);
					y1 = c4 * c3;
					y2 = c4 * b1;
					p0 = (3791.0 - 8.0) / 1600.0;
					p1 = 1.0 - 7357.0 * pow(2,-20);
					p2 = 3038.0 * 100.0 * pow(2,-36);
		
					return 0;
	} else {
	
		return -1;
	}
				
}

/**
	*	@brief API to read an unsigned integer from BMP180 
	* @param 2 parameters: register address(including subsequent address) to read from 
	* @return 0 on successful execution, -1 otherwise
	*/
static int bmp_readUInt(uint16_t reg_address, uint16_t *val)
{
	int ret;

	data[0] = reg_address;
	ret = bmp_readBytes(data,2);
	if (ret == 0) {
		*val = (((uint16_t)data[0]<<8)|(uint16_t)data[1]);
		return 0; 
	} else {
		return -1;
	}
}

/**
	*	@brief API to read a signed integer from BMP180 
	* @param 2 parameters: register address(including subsequent address) to read from 
	* @return 0 on successful execution, -1 otherwise
	*/
static int bmp_readInt(uint16_t reg_address, int16_t *value)
{
	int ret;

	data[0] = reg_address;
	ret = bmp_readBytes(data,2);
	if (ret == 0) {
		*value = (int16_t)((data[0]<<8)|data[1]);
		return 0; 
	} else {
		return -1;
	}
}

/**
	* @brief API to read an array of bytes from bmp180 device
	* @param 2 parameters: register address to read from and the number of bytes to be read
	* @return 0 on success, -ve values otherwise
	*/
static int bmp_readBytes(uint8_t *values, uint16_t numberOfBytes)
{
	i2c_device_stat = HAL_I2C_IsDeviceReady(&hi2c1, BMP180_ADDR, 3, 10);
	if (i2c_device_stat == HAL_OK) {
		i2c_status = HAL_I2C_Master_Receive(&hi2c1, BMP180_ADDR, values, numberOfBytes, 10 );
		if (i2c_status == HAL_OK) {
			return 0;
		} else {
			return -1;
		}
	} else {
		return -2;
	}
}

/**
	* @brief API to write an array of bytes to bmp180 IC
	* @param 2 parameters: register address to write to and the length in bytes of data to be written
	* @return 0 on success, -ve values otherwise
	*/
static int bmp_writeBytes(uint8_t *val, uint8_t length)
{
	i2c_device_stat = HAL_I2C_IsDeviceReady(&hi2c1, BMP180_ADDR, 3, 10);
	if (i2c_device_stat == HAL_OK) {
		i2c_status = HAL_I2C_Master_Transmit(&hi2c1, BMP180_ADDR, val , length, 10);
//	i2c_status = HAL_I2C_Mem_Write(&hi2c1, BMP180_ADDR, reg, 1, val, 1, 10);
		if (i2c_status == HAL_OK) {
			return 0;
		} else {
			return -1;
		}
	} else {
		return -2;
	}
	
}

/**
	* @brief API to initiate temperature reading in bmp180 IC
	* @param none
	* @return duration of delay i.e 5ms on success, 0 otherwise
	*/
static int bmp_start_temperature_reading(void)
{
	int result;
	uint8_t data[2];
	
	data[0] = BMP180_REG_CONTROL;
	data[1] = BMP180_COMMAND_TEMPERATURE;
	result = bmp_writeBytes(data, 2);
	if (result) 
		return 5; /* return the delay in ms (rounded up) to wait before retrieving data */
	else
		return 0; /* or return 0 if there was a problem communicating with the BMP */
}

/**
	* @brief API to get temperature reading from bmp180 IC, must be called after bmp_start_temperature_reading, after appropriate delay and bmp_init API's
	* @param External variable temperature
	* @return 0 on success, -ve otherwise
	*/
static int bmp_get_temperature_reading(double *T)
{
	uint8_t data[2];
	uint8_t result;
	double tu, a;
	
	data[0] = BMP180_REG_RESULT;

	result = bmp_readBytes(data, 2);
	if (result) {
		tu = (data[0] * 256.0) + data[1];
		a = c5 * (tu - c6);
		
		if((a+md) == 0) {
			/* return divide by zero exception */
				return -3;
		}
		*T = a + (mc / (a + md));
		
		return 0;
	} else {
	
			return -1;
	}
}
/**
	* @brief API to initiate pressure reading in bmp180 IC
	* @param 1 parameter: Oversampling: 0 to 3, higher numbers are slower, higher-res outputs.
	* @return duration of delay a postive integer on success, 0 otherwise
	*/
static int bmp_start_pressure_reading(uint8_t oversampling)
{
	uint8_t data[2], result;
	int delay;
	
	data[0] = BMP180_REG_CONTROL;
	switch (oversampling)
	{
		case 0:
			data[1] = BMP180_COMMAND_PRESSURE0;
			delay = 5;
		break;
		case 1:
			data[1] = BMP180_COMMAND_PRESSURE1;
			delay = 8;
		break;
		case 2:
			data[1] = BMP180_COMMAND_PRESSURE2;
			delay = 14;
		break;
		case 3:
			data[1] = BMP180_COMMAND_PRESSURE3;
			delay = 26;
		break;
		default:
			data[1] = BMP180_COMMAND_PRESSURE0;
			delay = 5;
		break;
	}
	result = bmp_writeBytes(data, 2);
	if (result) 
		return delay; /* return the delay in ms (rounded up) to wait before retrieving data */
	else
		return 0; /* or return 0 if there was a problem communicating with the BMP */
}

/**
	* @brief API to get pressure reading from bmp180 IC, must be called after bmp_start_temperature_reading, after appropriate delay and bmp_init API's
	* @param 2 parameters: P:External variable pressure, T:External variable temperature
	* @return Pressure in mbars on success, -1 otherwise
	*/
static int bmp_get_pressure_reading(double *P, double *T)
{
	unsigned char data[3];
	int result;
	double pu,s,x,y,z;
	
	data[0] = BMP180_REG_RESULT;
	result = bmp_readBytes(data, 3);
	if (result) {
		pu = (data[0] * 256.0) + data[1] + (data[2]/256.0);
		s = *T - 25.0;
		x = (x2 * pow(s,2)) + (x1 * s) + x0;
		y = (y2 * pow(s,2)) + (y1 * s) + y0;
		z = (pu - x) / y;
		*P = (p2 * pow(z,2)) + (p1 * z) + p0;
		
		return result;
	} else {
	
		return -1;
	}
}

/**
	*	@brief API to compute equivalent sea level pressure, for a given pressure measured at a specific altitude
	* @param 2 parameters P: pressure(mbars) at a specific altitude A: altitude at which this pressure is measured
	* @return equivalent pressure at sea level
	*/
static int bmp_compute_sealevel(double P, double A)
{
	return(P/pow(1-(A/44330.0),5.255));
}

/**
	*	@brief API to compute altitude (meters) above baseline, for a given pressure measurement and the pressure at baseline
	* @param 2 parameters P: pressure(mbars) P0: baseline pressure
	* @return altitude above baseline
	*/
static int bmp_compute_altitude(double P, double P0)
{
	return(44330.0*(1-pow(P/P0,1/5.255)));
}


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
		int ret;
		int delay;
	 double temperature;
		double pressure;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	/* Initialise BMP180 Sensor */
	ret = bmp_init();
	if (ret < 0) {
		printf("%s", "Error Initialising BMP Sensor \n");
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		/* USER CODE END WHILE */
		/*NOTE: Pressure measurement must always be preceded by temperature measurement */
		delay = bmp_start_temperature_reading();
		if (delay == 0) {
			printf("%s", "Error in starting temperature reading \n");
		}
		/* wait until specified delay is elapsed */
		HAL_Delay((delay * 1000));
		ret = bmp_get_temperature_reading(&temperature);
		if (ret < 0) {
					printf("%s", "Error in getting temperature reading \n");
		}
		printf("temperature = %lf\n", temperature);
		
		/* oversampling = 1 */
		delay = bmp_start_pressure_reading(1);
		if (delay == 0) {
			printf("%s", "Error in starting pressure reading \n");
		}
		/* Wait for specified delay to elapse */
		HAL_Delay((delay * 1000));
		ret = bmp_get_pressure_reading(&pressure, &temperature);
		if (ret < 0) {
					printf("%s", "Error in getting pressure reading \n");
		}
		printf("pressure = %lf\n", pressure);
		/* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Pinout Configuration
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
