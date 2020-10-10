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
#include "Si7021.h"
#include "Adafruit_G77E0_TEA6320.h"
#include "math.h"
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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
typedef struct {
	uint8_t num; // сколько отсчетов уже сложили не болле NUM_SAMPLES
	float sum_internalTemperature; // Сумма для усреднения Текущие температуры в сотых градуса !!! место экономим
	float sum_externalTemperature; // Сумма для усреднения Текущие температуры в сотых градуса !!! место экономим
	float sum_internalHumidity; // Сумма для усреднения Относительные влажности сотых процента !!! место экономим
	float sum_externalHumidity; // Сумма для усреднения Относительные влажности сотых процента !!! место экономим
} type_sensors; // структура для усреднения

typedef struct {
	float internalTemperature;
	float externalTemperature;
	float internalHumidity;
	float externalHumidity;
	float abs_internalHumidity; // Абсолютные влажности в граммах на м*3
	float abs_externalHumidity; // Абсолютные влажности в граммах на м*3
} type_result;

Si7021 internalSensor = Si7021(&hi2c1);
Si7021 externalSensor = Si7021(&hi2c2);
type_sensors sensors;
type_result results;
Adafruit_G77E0_TEA6320 display = Adafruit_G77E0_TEA6320();
uint8_t message[100] = {};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void reset_sum(void);
float calculationAbsH(float, float);
void CheckON(void);
void initLcd(void);
uint8_t numberToString(float number, uint8_t *strArray, uint8_t fractionalNumber);
uint8_t createJsonString(float iH, float iT, float eH, float eT, uint8_t motor, uint8_t *strArray);
void showSuccesMeasuringOnLcd(type_result results);
void sendSuccesBluetoothMessage(type_result results);
void showErrorOnLcd(Si7021 externalSensor, Si7021 internalSensor);
void sendErrorBluetoothMessage(Si7021 externalSensor, Si7021 internalSensor);
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
	__HAL_RCC_I2C1_CLK_ENABLE();
	__HAL_RCC_I2C2_CLK_ENABLE();
	HAL_Delay(100);
	__HAL_RCC_I2C1_FORCE_RESET();
	__HAL_RCC_I2C2_FORCE_RESET();
	HAL_Delay(100);
	__HAL_RCC_I2C1_RELEASE_RESET();
	__HAL_RCC_I2C2_RELEASE_RESET();
	HAL_Delay(100);
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C2_Init();
	MX_I2C1_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	initLcd();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

#ifdef DEBUG
		HAL_UART_Transmit(&huart2, (uint8_t*)"1111", 3, 100);
		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);
#endif

		externalSensor.measureTemperatureAndHumidity();
		internalSensor.measureTemperatureAndHumidity();

		if (externalSensor.measuredSuccessful
				&& internalSensor.measuredSuccessful) {
			// all measuring are successful

#ifdef DEBUG
			HAL_UART_Transmit(&huart2, (uint8_t*)"3333", 3, 100);
			HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);
#endif

			if (sensors.num < NUM_SAMPLES) {

#ifdef DEBUG
				HAL_UART_Transmit(&huart2, (uint8_t*)"4.111", 3, 100);
				HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);
#endif

				// add the measured values for averaging
				sensors.sum_externalTemperature += externalSensor.getTemperature();
				sensors.sum_internalTemperature += internalSensor.getTemperature();
				sensors.sum_externalHumidity += externalSensor.getHumidity();
				sensors.sum_internalHumidity += internalSensor.getHumidity();
				sensors.num++;
			}

			if (sensors.num >= NUM_SAMPLES) {

#ifdef DEBUG
				HAL_UART_Transmit(&huart2, (uint8_t*)"4.222", 3, 100);
				HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);
#endif
				// calculate the average values
				results.externalTemperature = sensors.sum_externalTemperature / (float)NUM_SAMPLES;
				results.internalTemperature = sensors.sum_internalTemperature / (float)NUM_SAMPLES;
				results.externalHumidity = sensors.sum_externalHumidity / (float)NUM_SAMPLES;
				results.internalHumidity = sensors.sum_internalHumidity / (float)NUM_SAMPLES;
				results.abs_externalHumidity = calculationAbsH(
						results.externalTemperature, results.externalHumidity);
				results.abs_internalHumidity = calculationAbsH(
						results.internalTemperature, results.internalHumidity);

				CheckON(); // Проверка статуса вентилятора
				reset_sum();
			}
			showSuccesMeasuringOnLcd(results);
			sendSuccesBluetoothMessage(results);
		} else {

#ifdef DEBUG
			HAL_UART_Transmit(&huart2, (uint8_t*)"5555", 3, 100);
			HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);
#endif

			// some mistake happens during measuring
			showErrorOnLcd(externalSensor, internalSensor);
			sendErrorBluetoothMessage(externalSensor, internalSensor);
			reset_sum();
		}

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
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
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
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 50000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

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
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : LED1_Pin */
	GPIO_InitStruct.Pin = LED1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : MOTOR_Pin LCD_RESET_Pin */
	GPIO_InitStruct.Pin = MOTOR_Pin|LCD_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void reset_sum(void)  // Сброс счетчиков накоплений
{
	sensors.num = 0;
	sensors.sum_internalTemperature = 0;
	sensors.sum_externalTemperature = 0;
	sensors.sum_internalHumidity = 0;
	sensors.sum_externalHumidity = 0;
}

float calculationAbsH(float t, float h) {
	float temp;
	temp = pow(2.718281828, (17.67 * t) / (t + 243.5));
	return (6.112 * temp * h * 2.1674) / (273.15 + t);
}

void CheckON(void) {
	if (HAL_GPIO_ReadPin(MOTOR_GPIO_Port, MOTOR_Pin) == GPIO_PIN_RESET) {
		// Вентилятор выключен
		if ((results.abs_internalHumidity - d_HUMIDITY)
				> results.abs_externalHumidity) {
			HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, GPIO_PIN_SET);
			display.fillRect(0, ROW_1, 128, 8, WHITE);
			display.setCursor(17, ROW_1);
			display.println("Вытяжка включена");

		}
	} else {
		// Вентилятор включен
		if (results.abs_internalHumidity < results.abs_externalHumidity) {
			HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, GPIO_PIN_RESET);
			display.fillRect(0, ROW_1, 128, 8, WHITE);
			display.setCursor(17, ROW_1);
			display.println("Вытяжка отключена");
		}
	}
}

void initLcd(void) {
	display.initDisplay();
	display.setTextColor(BLACK);
	display.setTextSize(1);
	display.setCursor(4, 0);
	display.println("Контpоллеp влажности");
	display.writeFastHLine(0, 8, 128, BLACK);

	display.setCursor(17, ROW_1);
	display.println("Вытяжка выключена");

	display.writeFastHLine(0, TABLE_ROW_0 - 3, 128, BLACK);

	display.setCursor(27, TABLE_ROW_0);
	display.println("Датчик");

	display.setCursor(TABLE_COLUMN_2 + 7, TABLE_ROW_0);
	display.println("H,%");

	display.setCursor(TABLE_COLUMN_3 + 7, TABLE_ROW_0);
	display.println("t,");
	display.drawChar(TABLE_COLUMN_3 + 7 + 10, TABLE_ROW_0 - 2, 9, BLACK, WHITE,
			1); // symbol "°"
	display.setCursor(TABLE_COLUMN_3 + 7 + 15, TABLE_ROW_0);
	display.println("С");

	display.writeFastHLine(0, TABLE_ROW_1 - 3, 128, BLACK);
	display.setCursor(21, TABLE_ROW_1);
	display.println("Внешний");
	display.writeFastHLine(0, TABLE_ROW_2 - 3, 128, BLACK);
	display.setCursor(3, TABLE_ROW_2);
	display.println("Внутpенний");
	display.writeFastHLine(0, 63, 128, BLACK);

	display.writeFastVLine(0, TABLE_ROW_0 - 3, 63 - 25, BLACK);
	display.writeFastVLine(TABLE_COLUMN_2, TABLE_ROW_0 - 3, 63 - 25, BLACK);
	display.writeFastVLine(TABLE_COLUMN_3, TABLE_ROW_0 - 3, 63 - 25, BLACK);
	display.writeFastVLine(127, TABLE_ROW_0 - 3, 63 - 25, BLACK);

	CLEAR_CELL_1_1();
	CLEAR_CELL_1_2();
	CLEAR_CELL_2_1();
	CLEAR_CELL_2_2();

	display.display();
}

uint8_t numberToString(float number, uint8_t *strArray, uint8_t fractionalNumber) {
	uint8_t numbers[10] = {
			(uint8_t)'0',
			(uint8_t)'1',
			(uint8_t)'2',
			(uint8_t)'3',
			(uint8_t)'4',
			(uint8_t)'5',
			(uint8_t)'6',
			(uint8_t)'7',
			(uint8_t)'8',
			(uint8_t)'9'
	};
	uint8_t isNegativeSign = number < 0 ? 1 : 0;
	if(isNegativeSign) {
		number *= -1;
		strArray[0] = (uint8_t)'-';
	}
	uint32_t previousNumber = number;
	uint32_t temp = 0;
	uint8_t symbolCount = 1; // количества разрядов
	uint32_t factor = 1;

	// расчет количества разрядов (symbolCount)
	previousNumber = (uint32_t)(previousNumber / 10);
	while(previousNumber > 0) {
		previousNumber = previousNumber / 10;
		symbolCount++;
	}

	// расчет начального делителя
	for (uint8_t factorIndex = 0; factorIndex < (symbolCount - 1); factorIndex++) {
		factor *= 10;
	}

	previousNumber = number;
	for (uint8_t index = 0; index < (symbolCount); index++) {
		temp = (uint32_t)(previousNumber / factor);
		strArray[index + isNegativeSign] = numbers[temp];
		previousNumber = previousNumber - (temp * factor);
		factor /= 10;
	}

	if(fractionalNumber == 0) {
		return isNegativeSign ? symbolCount + 1 : symbolCount;
	}

	strArray[symbolCount + isNegativeSign] = (uint8_t)'.';
	symbolCount += 1;
	factor = 1;

	for (uint8_t factorIndex = 0; factorIndex < fractionalNumber; factorIndex++) {
		factor *= 10;
		uint8_t a = (uint32_t)(number * factor) % 10;
		strArray[symbolCount + isNegativeSign] = numbers[a];
		symbolCount += 1;
	}

	return isNegativeSign ? symbolCount + 1 : symbolCount;
}

uint8_t createJsonString(float iH, float iT, float eH, float eT, uint8_t motor, uint8_t *strArray) {
	uint8_t sub0[6] = { (uint8_t)'{', (uint8_t)'"', (uint8_t)'i', (uint8_t)'H', (uint8_t)'"', (uint8_t)':' };
	uint8_t sub1[6] = { (uint8_t)',', (uint8_t)'"', (uint8_t)'i', (uint8_t)'T', (uint8_t)'"', (uint8_t)':' };
	uint8_t sub2[6] = { (uint8_t)',', (uint8_t)'"', (uint8_t)'e', (uint8_t)'H', (uint8_t)'"', (uint8_t)':' };
	uint8_t sub3[6] = { (uint8_t)',', (uint8_t)'"', (uint8_t)'e', (uint8_t)'T', (uint8_t)'"', (uint8_t)':' };
	uint8_t sub4[5] = { (uint8_t)',', (uint8_t)'"', (uint8_t)'m', (uint8_t)'"', (uint8_t)':' };
	uint8_t sub5[1] = { (uint8_t)'}' };
	uint8_t lengts[6] = { 6, 6, 6, 6, 5, 1 };
	uint8_t numberArray[10] = {};
	uint8_t messageLength = 0;
	uint8_t numberLength = 0;
	//---------------------------------------------------------------------------------------------------------
	// "{'iH':"
	for(uint8_t index = 0; index < lengts[0]; index++) {
		message[messageLength] = sub0[index];
		messageLength++;
	}
	numberLength = numberToString(iH, &numberArray[0], 2);
	for(uint8_t numberIndex = 0; numberIndex < numberLength; numberIndex++) {
		message[messageLength] = numberArray[numberIndex];
		messageLength++;
	}
	//---------------------------------------------------------------------------------------------------------
	// "{'iT':"
	for(uint8_t index = 0; index < lengts[1]; index++) {
		message[messageLength] = sub1[index];
		messageLength++;
	}
	numberLength = numberToString(iT, &numberArray[0], 2);
	for(uint8_t numberIndex = 0; numberIndex < numberLength; numberIndex++) {
		message[messageLength] = numberArray[numberIndex];
		messageLength++;
	}
	//---------------------------------------------------------------------------------------------------------
	// ",{'eH':"
	for(uint8_t index = 0; index < lengts[2]; index++) {
		message[messageLength] = sub2[index];
		messageLength++;
	}
	numberLength = numberToString(eH, &numberArray[0], 2);
	for(uint8_t numberIndex = 0; numberIndex < numberLength; numberIndex++) {
		message[messageLength] = numberArray[numberIndex];
		messageLength++;
	}
	//---------------------------------------------------------------------------------------------------------
	// ",{'eH':"
	for(uint8_t index = 0; index < lengts[3]; index++) {
		message[messageLength] = sub3[index];
		messageLength++;
	}
	numberLength = numberToString(eT, &numberArray[0], 2);
	for(uint8_t numberIndex = 0; numberIndex < numberLength; numberIndex++) {
		message[messageLength] = numberArray[numberIndex];
		messageLength++;
	}
	//---------------------------------------------------------------------------------------------------------
	// ",{'m':"
	for(uint8_t index = 0; index < lengts[4]; index++) {
		message[messageLength] = sub4[index];
		messageLength++;
	}
	numberLength = numberToString((float)(motor), &numberArray[0], 0);
	for(uint8_t numberIndex = 0; numberIndex < numberLength; numberIndex++) {
		message[messageLength] = numberArray[numberIndex];
		messageLength++;
	}
	//---------------------------------------------------------------------------------------------------------
	// "}"
	for(uint8_t index = 0; index < lengts[5]; index++) {
		message[messageLength] = sub5[index];
		messageLength++;
	}
	//---------------------------------------------------------------------------------------------------------
	return messageLength;
}

void showSuccesMeasuringOnLcd(type_result results) {
	CLEAR_CELL_1_1();
	CLEAR_CELL_1_2();
	display.setCursor(TABLE_COLUMN_2 + 2, TABLE_ROW_1);
	display.drawFloat(results.abs_externalHumidity);
	display.setCursor(TABLE_COLUMN_3 + 2, TABLE_ROW_1);
	display.drawFloat(results.externalTemperature);

	CLEAR_CELL_2_1();
	CLEAR_CELL_2_2();
	display.setCursor(TABLE_COLUMN_2 + 2, TABLE_ROW_2);
	display.drawFloat(results.abs_internalHumidity);
	display.setCursor(TABLE_COLUMN_3 + 2, TABLE_ROW_2);
	display.drawFloat(results.internalTemperature);
	display.display();
}

void showErrorOnLcd(Si7021 externalSensor, Si7021 internalSensor) {
	if (!externalSensor.measuredSuccessful) {
		CLEAR_CELL_1_1();
		CLEAR_CELL_1_2();
		display.setCursor(TABLE_COLUMN_2 + 10, TABLE_ROW_1);
		display.println("--");
		display.setCursor(TABLE_COLUMN_3 + 10, TABLE_ROW_1);
		display.println("--");
	}

	if (!internalSensor.measuredSuccessful) {
		CLEAR_CELL_2_1();
		CLEAR_CELL_2_2();
		display.setCursor(TABLE_COLUMN_2 + 10, TABLE_ROW_2);
		display.println("--");
		display.setCursor(TABLE_COLUMN_3 + 10, TABLE_ROW_2);
		display.println("--");
	}
	display.display();
}

void sendSuccesBluetoothMessage(type_result results) {
	uint8_t motor = 0;
	if(HAL_GPIO_ReadPin(MOTOR_GPIO_Port, MOTOR_Pin) == GPIO_PIN_SET) {
		motor = 1;
	}
	uint8_t messageLength = createJsonString(
			results.abs_internalHumidity,
			results.internalTemperature,
			results.abs_externalHumidity,
			results.externalTemperature,
			motor,
			&message[0]
	);

	HAL_UART_Transmit(&huart2, (uint8_t*)message, messageLength, 100);
	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);
}

void sendErrorBluetoothMessage(Si7021 externalSensor, Si7021 internalSensor) {
	// TODO: add implementation
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
