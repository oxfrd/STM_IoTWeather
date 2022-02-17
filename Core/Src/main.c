/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmpxx80.h"
#include "BH1750.h"
#include "stdio.h"
#include "DHT11.h"
#include "dwt_stm32_delay.h"
#include "NRF24L01.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t tx_buffer[100];

uint8_t TxAddress[] = {0xEE,0xDD,0xCC,0xBB,0xAA};
uint8_t TxData[] = "Hello World\n";

uint8_t data[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


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
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  BH1750_Init(&hi2c1);
  BH1750_SetMode(CONTINUOUS_HIGH_RES_MODE);

  BMP280_Init(&hi2c1, BMP280_TEMPERATURE_16BIT, BMP280_STANDARD, BMP280_FORCEDMODE);

  DWT_Delay_Init();

  NRF24_Init();
  NRF24_TxMode(TxAddress, 10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t BH1750_lx_value;
  uint32_t BMP280_presure;
  uint32_t BMP280_presure_hPa;
  uint32_t BMP280_presure_afterComa;
  //uint32_t BMP280_temperature;
  uint32_t DHT11_temperature;
  uint32_t DHT11_humidity;
  uint32_t FC37_status;
  int size;

  while (1)
  {
		DHT11_TempAndHumidity(&DHT11_temperature, &DHT11_humidity);
		/*size = sprintf((char*) tx_buffer, "DHT11:  %d st. C\r\n",
				(int) DHT11_temperature);
		HAL_UART_Transmit(&huart2, tx_buffer, size, 1000);
		size = sprintf((char*) tx_buffer, "DHT11:  %d %% \r\n",
				(int) DHT11_humidity);
		HAL_UART_Transmit(&huart2, tx_buffer, size, 1000);

		BMP280_temperature = BMP280_ReadTemperature();
		size = sprintf((char*) tx_buffer, "BMP280: %d st. C\r\n",
				(int) BMP280_temperature);
		HAL_UART_Transmit(&huart2, tx_buffer, size, 1000);*/

		BMP280_presure = BMP280_ReadPressure();
		BMP280_presure_hPa = BMP280_presure/100;
		BMP280_presure_afterComa = BMP280_presure - (BMP280_presure_hPa*100);

		/*size = sprintf((char*) tx_buffer, "BMP280: %d.%d hPa\r\n",
				(int) BMP280_presure_hPa, (int) BMP280_presure_afterComa);
		HAL_UART_Transmit(&huart2, tx_buffer, size, 1000);*/

		BH1750_ReadLight(&BH1750_lx_value);

		/*size = sprintf((char*) tx_buffer, "BH1750: %d lx\r\n",
				(int) BH1750_lx_value);
		HAL_UART_Transmit(&huart2, tx_buffer, size, 1000);*/

		if (HAL_GPIO_ReadPin(FC37_Pin_GPIO_Port, FC37_Pin_Pin)) {
			FC37_status = 1;
		// size = sprintf((char*) tx_buffer, "FC37:   It is raining!\r\n");
		} else {
			FC37_status = 0;
		// size = sprintf((char*) tx_buffer, "FC37:   It is not raining!\r\n");
		}
		/*HAL_UART_Transmit(&huart2, tx_buffer, size, 1000);*/

		size = sprintf((char*) tx_buffer, "{\"temp\":%d, \"hum\":%d, \"press\":%d.%d,"
				"\"light\":%d, \"batt\":4.58, \"rain\":%d}",
						(int) DHT11_temperature, 	(int) DHT11_humidity,
						(int) BMP280_presure_hPa, 	(int) BMP280_presure_afterComa,
						(int) BH1750_lx_value, 		(int) FC37_status);
		HAL_UART_Transmit(&huart2, tx_buffer, size, 1000);
/*
		if (NRF24_Transmit(TxData) == 1)
			{
			HAL_UART_Transmit(&huart2, "hey", 6, 1000);
			}
		else
			{
			HAL_UART_Transmit(&huart2, "kur", 6, 1000);
			}
*/
		DWT_Delay_us(5000000);

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
