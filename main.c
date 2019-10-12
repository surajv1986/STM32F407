/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* USER CODE BEGIN Includes */
#define MSBFIRST 1
#define LSBFIRST 0

uint16_t decode_mode_code[4] = {0x900, 0x901, 0x90F, 0x9FF};
uint16_t intensity_mode_code[16] = {0xA00, 0xA01, 0xA02, 0xA03, 0xA04, 0xA05, 0xA06, 0xA07, 0xA08, 0xA09, 0xA0A, 0xA0B, 0xA0C, 0xA0D, 0xA0E, 0xA0F};
uint16_t display_code[10] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};
uint16_t display_e = 0x0B;
uint16_t display_h = 0x0C;
uint16_t display_l = 0x0D;
uint16_t display_p = 0x0E;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void shiftOut(uint16_t, uint16_t, uint8_t, uint16_t);
void set_decode_mode(uint16_t);
void set_intensity(uint16_t);
void display_zero(void);
void display_one(void);
void display_two(void);
void display_three(void);
void display_four(void);
void display_five(void);
void display_six(void);
void display_seven(void);
void display_eight(void);
void display_nine(void);
void display_E(void);
void display_H(void);
void display_L(void);
void display_P(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/**
	* @brief API to set decode Mode of Display Driver
	* @param 1 parameter: mode_code: A 16 bit hex code specifying the decode mode
	* @return none
	*/
void set_decode_mode(uint16_t mode_code)
{
		/* Enable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	shiftOut(GPIO_PIN_11, GPIO_PIN_9, MSBFIRST, mode_code);
	/* Disable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}
/**
	* @brief API to set intensity level of Display Driver
	* @param 1 parameter: intensity_code: A 16 bit hex code specifying the intensity level
	* @return none
	*/
void set_intensity(uint16_t intensity_code)
{
		/* Enable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	shiftOut(GPIO_PIN_11, GPIO_PIN_9, MSBFIRST, intensity_code);
	/* Disable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

}
/**
	* @brief API to display zero in b decode mode
	* @param  none
	* @return none
	*/
void display_zero(void)
{
	/* Enable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	shiftOut(GPIO_PIN_11, GPIO_PIN_9, MSBFIRST, display_code[0]);
	/* Disable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

/**
	* @brief API to display 1 in b decode mode
	* @param  none
	* @return none
	*/
void display_one(void)
{
	/* Enable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	shiftOut(GPIO_PIN_11, GPIO_PIN_9, MSBFIRST, display_code[1]);
	/* Disable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

/**
	* @brief API to display 2 in b decode mode
	* @param  none
	* @return none
	*/
void display_two(void)
{
	/* Enable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	shiftOut(GPIO_PIN_11, GPIO_PIN_9, MSBFIRST, display_code[2]);
	/* Disable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

/**
	* @brief API to display 3 in b decode mode
	* @param  none
	* @return none
	*/
void display_three(void)
{
	/* Enable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	shiftOut(GPIO_PIN_11, GPIO_PIN_9, MSBFIRST, display_code[3]);
	/* Disable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

/**
	* @brief API to display 4 in b decode mode
	* @param  none
	* @return none
	*/
void display_four(void)
{
	/* Enable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	shiftOut(GPIO_PIN_11, GPIO_PIN_9, MSBFIRST, display_code[4]);
	/* Disable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

/**
	* @brief API to display 5 in b decode mode
	* @param  none
	* @return none
	*/
void display_five(void)
{
	/* Enable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	shiftOut(GPIO_PIN_11, GPIO_PIN_9, MSBFIRST, display_code[5]);
	/* Disable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

/**
	* @brief API to display 6 in b decode mode
	* @param  none
	* @return none
	*/
void display_six(void)
{
	/* Enable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	shiftOut(GPIO_PIN_11, GPIO_PIN_9, MSBFIRST, display_code[6]);
	/* Disable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

/**
	* @brief API to display 7 in b decode mode
	* @param  none
	* @return none
	*/
void display_seven(void)
{
	/* Enable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	shiftOut(GPIO_PIN_11, GPIO_PIN_9, MSBFIRST, display_code[7]);
	/* Disable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

/**
	* @brief API to display zero in b decode mode
	* @param  none
	* @return none
	*/
void display_eight(void)
{
	/* Enable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	shiftOut(GPIO_PIN_11, GPIO_PIN_9, MSBFIRST, display_code[8]);
	/* Disable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

/**
	* @brief API to display 9 in b decode mode
	* @param  none
	* @return none
	*/
void display_nine(void)
{
	/* Enable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	shiftOut(GPIO_PIN_11, GPIO_PIN_9, MSBFIRST, display_code[9]);
	/* Disable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

/**
	* @brief API to display E in b decode mode
	* @param  none
	* @return none
	*/
void display_E(void)
{
	/* Enable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	shiftOut(GPIO_PIN_11, GPIO_PIN_9, MSBFIRST, display_e);
	/* Disable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

/**
	* @brief API to display H in b decode mode
	* @param  none
	* @return none
	*/
void display_H(void)
{
	/* Enable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	shiftOut(GPIO_PIN_11, GPIO_PIN_9, MSBFIRST, display_h);
	/* Disable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

/**
	* @brief API to display L in b decode mode
	* @param  none
	* @return none
	*/
void display_L(void)
{
	/* Enable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	shiftOut(GPIO_PIN_11, GPIO_PIN_9, MSBFIRST, display_l);
	/* Disable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

/**
	* @brief API to display P in b decode mode
	* @param  none
	* @return none
	*/
void display_P(void)
{
	/* Enable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	shiftOut(GPIO_PIN_11, GPIO_PIN_9, MSBFIRST, display_p);
	/* Disable Latch */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}


/**
	*@brief A function to push 16 bit data serially using a clock to synchronise
	*@param 4 parameters: dataPin:GPIO for shifting data out, clockPin:GPIO for controlling clock, bitorder: specify bitorder, val:serial data to sent
	*@return none	
	*/
void shiftOut(uint16_t dataPin, uint16_t clockPin, uint8_t bitOrder, uint16_t val)
{
		uint16_t i;
		uint16_t value_lsb;
		uint16_t value_msb;
		
    for (i = 0; i < 8; i++)  {
			
				/* Extract each bit starting from LSB */
				value_lsb = !!(val & (1 << i));
				/* Extract each bit starting from MSB */
				value_msb = !!(val & (1 << (15 - i)));
				/* If bitorder is LSB First */
        if (bitOrder == 0) {
					
					if (value_lsb == 0)
						HAL_GPIO_WritePin(GPIOA, dataPin, GPIO_PIN_RESET);
					else
						HAL_GPIO_WritePin(GPIOA, dataPin, GPIO_PIN_SET);
				}
				/* If bit order is MSB first */
        else {
								if (value_msb == 0)
									HAL_GPIO_WritePin(GPIOA, dataPin, GPIO_PIN_RESET);
								else
									HAL_GPIO_WritePin(GPIOA, dataPin, GPIO_PIN_SET);
				}
            
				/* Toggle Clock Pin */
				HAL_GPIO_WritePin(GPIOA, clockPin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, clockPin, GPIO_PIN_RESET);
                    
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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

  /* USER CODE BEGIN 2 */
	/* Set display driver to code b decode mode */
	set_decode_mode(decode_mode_code[3]);
	/* Set Intensity Levels of display to 9/16 of Max7221 or 17/32 of MAX7219's display */
	set_intensity(intensity_mode_code[9]);
	/* Display numbers 0-9 */
	display_one();
	HAL_Delay(500);
	display_two();
	HAL_Delay(500);
	display_three();
	HAL_Delay(500);
	display_four();
	HAL_Delay(500);
	display_five();
	HAL_Delay(500);
	display_six();
	HAL_Delay(500);
	display_seven();
	HAL_Delay(500);
	display_eight();
	HAL_Delay(500);
	display_nine();
	HAL_Delay(500);
	
	/* Display alphabets e, h, l ,p */
	display_E();
	HAL_Delay(500);
	display_H();
	HAL_Delay(500);
	display_L();
	HAL_Delay(500);
	display_P();
	HAL_Delay(500);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

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

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_9|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA9 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_9|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
