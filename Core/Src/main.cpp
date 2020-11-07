/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "json.h"
#include "programI2C.h"
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float sum_internalAbsoluteHumidity = 0;
float sum_externalAbsoluteHumidity = 0;
float averageInternalAbsoluteHumidity = 0;
float averageExternalAbsoluteHumidity = 0;

uint8_t averageCounter = 0;

I2C internalI2C = I2C(INTERNAL_SCL_Pin, INTERNAL_SDA_Pin, INTERNAL_SDA_GPIO_Port);
I2C externalI2C = I2C(EXTERNAL_SCL_Pin, EXTERNAL_SDA_Pin, EXTERNAL_SDA_GPIO_Port);
Si7021 internalSensor = Si7021(&internalI2C);
Si7021 externalSensor = Si7021(&externalI2C);
uint8_t message[100] = {};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void reset_sum(void);
float calculationAbsH(float, float);
void CheckON(void);
void initLcd(void);
uint8_t numberToString(float number, uint8_t *strArray, uint8_t fractionalNumber);
uint8_t createJsonString(float iH, float iT, float eH, float eT, uint8_t motor, uint8_t *strArray);
void sendBluetoothMessage(Si7021 internalSensor, Si7021 externalSensor);
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
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

#ifdef DEBUG
		HAL_UART_Transmit(&huart2, (uint8_t*)"1) ***** start *****\r\n", (sizeof "1) ***** start *****\r\n") - 1, 100);
#endif

		internalSensor.measureTemperatureAndHumidity();
		externalSensor.measureTemperatureAndHumidity();

#ifdef DEBUG
		if (!externalSensor.measuredSuccessful) {
			HAL_UART_Transmit(&huart2, (uint8_t*)"external sensor FAIL\r\n", (sizeof "external sensor FAIL\r\n") - 1, 100);
		}
		if (!internalSensor.measuredSuccessful) {
			HAL_UART_Transmit(&huart2, (uint8_t*)"internal sensor FAIL\r\n", (sizeof "internal sensor FAIL\r\n") - 1, 100);
		}
#endif

		if (
				internalSensor.measuredSuccessful
				&&
				externalSensor.measuredSuccessful
		) {
			// all measuring are successful

#ifdef DEBUG
			HAL_UART_Transmit(&huart2, (uint8_t*)"2) sensors have been read\r\n", (sizeof "2) sensors have been read\r\n") - 1, 100);
#endif

			if (averageCounter < NUM_SAMPLES) {

#ifdef DEBUG
				HAL_UART_Transmit(&huart2, (uint8_t*)"3) summation to calculate averages\r\n", (sizeof "3) summation to calculate averages\r\n") - 1, 100);
#endif

				// add the measured values for averaging
				sum_internalAbsoluteHumidity += internalSensor.getAbsoluteHumidity();
				sum_externalAbsoluteHumidity += externalSensor.getAbsoluteHumidity();
				averageCounter++;
			}

			if (averageCounter >= NUM_SAMPLES) {

#ifdef DEBUG
				HAL_UART_Transmit(&huart2, (uint8_t*)"4) calculating average\r\n", (sizeof "4) calculating average\r\n") - 1, 100);
#endif
				// calculate the average values
				averageInternalAbsoluteHumidity = sum_internalAbsoluteHumidity / (float)NUM_SAMPLES;
				averageExternalAbsoluteHumidity = sum_externalAbsoluteHumidity / (float)NUM_SAMPLES;

				CheckON(); // Проверка статуса вентилятора
				reset_sum();
			}
		} else {
			// some mistake happens during measuring
#ifdef DEBUG
			HAL_UART_Transmit(&huart2, (uint8_t*)"ERROR\r\n", (sizeof "ERROR\r\n") - 1, 100);
#endif
			motorStop();
			reset_sum();
		}
		sendBluetoothMessage(internalSensor, externalSensor);
		HAL_Delay(TIME_SCAN_SENSOR);
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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, EXTERNAL_SCL_Pin|EXTERNAL_SDA_Pin|INTERNAL_SCL_Pin|INTERNAL_SDA_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED1_Pin */
	GPIO_InitStruct.Pin = LED1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : EXTERNAL_SCL_Pin EXTERNAL_SDA_Pin INTERNAL_SCL_Pin INTERNAL_SDA_Pin */
	GPIO_InitStruct.Pin = EXTERNAL_SCL_Pin|EXTERNAL_SDA_Pin|INTERNAL_SCL_Pin|INTERNAL_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : MOTOR_Pin */
	GPIO_InitStruct.Pin = MOTOR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(MOTOR_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void reset_sum(void)  // Сброс счетчиков накоплений
{
	averageCounter = 0;
	sum_internalAbsoluteHumidity = 0;
	sum_externalAbsoluteHumidity = 0;
}

float calculationAbsH(float t, float h) {
	float temp;
	temp = pow(2.718281828, (17.67 * t) / (t + 243.5));
	return (6.112 * temp * h * 2.1674) / (273.15 + t);
}

void CheckON(void) {
	if (HAL_GPIO_ReadPin(MOTOR_GPIO_Port, MOTOR_Pin) == GPIO_PIN_RESET) {
		// Вентилятор выключен
		if ((averageInternalAbsoluteHumidity - d_HUMIDITY)
				> averageExternalAbsoluteHumidity) {
			motorStart();

		}
	} else {
		// Вентилятор включен
		if (averageInternalAbsoluteHumidity < averageExternalAbsoluteHumidity) {
			motorStop();
		}
	}
}
void sendBluetoothMessage(Si7021 internalSensor, Si7021 externalSensor) {
	uint8_t motor = 0;
	if(HAL_GPIO_ReadPin(MOTOR_GPIO_Port, MOTOR_Pin) == GPIO_PIN_SET) {
		motor = 1;
	}
	uint8_t messageLength = JSON::createJsonString(internalSensor, externalSensor, motor, &message[0]);

	HAL_UART_Transmit(&huart2, (uint8_t*)message, messageLength, 100);
	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
