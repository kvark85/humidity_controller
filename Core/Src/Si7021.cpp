#include "Si7021.h"

extern UART_HandleTypeDef huart2;

Si7021::Si7021(I2C_HandleTypeDef *hi2c) {
	this->hi2c = hi2c;
}

si7021 Si7021::measureHumidity(void) {
	uint8_t RX_Data[3] = {0};
	uint16_t rawData;
	HAL_StatusTypeDef i2cStatus;
	si7021 humidity;
	humidity.value = 0;
	humidity.measuredSuccessful = false;

	RX_Data[0] = 0;
	RX_Data[1] = 0;
	RX_Data[2] = 0;

#ifdef DEBUG
	HAL_UART_Transmit(&huart2, (uint8_t*)"h.1 ", (sizeof "h.1 ") - 1, 100);
#endif

	i2cStatus = HAL_I2C_IsDeviceReady(hi2c, HTU21D_Address, 3, 100);
	if (i2cStatus == HAL_OK) {

#ifdef DEBUG
		HAL_UART_Transmit(&huart2, (uint8_t*)"HAL_OK_T ", (sizeof "HAL_OK_T ") - 1, 100);
#endif
		/* Humidity ---->; */
		HAL_I2C_Mem_Read(hi2c, HTU21D_Address, HUMIDITY_CMD,
				I2C_MEMADD_SIZE_8BIT, (uint8_t*) RX_Data, 3, 1000);
		rawData = ((uint16_t) (RX_Data[0] << 8) | (RX_Data[1]));
		humidity.value = (float) (rawData * 125.0 / 65536.0) - 6.0;

		if (checkCRC8(rawData) == RX_Data[2] && RX_Data[2] != 0) {
#ifdef DEBUG
			HAL_UART_Transmit(&huart2, (uint8_t*)"h.2\r\n", (sizeof "h.2\r\n") - 1, 100);
#endif
			humidity.measuredSuccessful = true;
		}
	} else {
		switch(i2cStatus) {
		case HAL_ERROR:
			HAL_UART_Transmit(&huart2, (uint8_t*)"HAL_ERROR_H\r\n", (sizeof "HAL_ERROR_H\r\n") - 1, 100);
			break;
		case HAL_BUSY:
			HAL_UART_Transmit(&huart2, (uint8_t*)"HAL_BUSY_H\r\n", (sizeof "HAL_BUSY_H\r\n") - 1, 100);
			break;
		case HAL_TIMEOUT:
			HAL_UART_Transmit(&huart2, (uint8_t*)"HAL_TIMEOUT_H\r\n", (sizeof "HAL_TIMEOUT_H\r\n") - 1, 100);
			break;
		}
	}
	return humidity;
}

si7021 Si7021::measureTemperature(void) {
	uint8_t RX_Data[3] = {0};
	uint16_t rawData;
	HAL_StatusTypeDef i2cStatus;
	si7021 temperature;
	temperature.value = 0;
	temperature.measuredSuccessful = false;

	RX_Data[0] = 0;
	RX_Data[1] = 0;
	RX_Data[2] = 0;

#ifdef DEBUG
	HAL_UART_Transmit(&huart2, (uint8_t*)"t.1 ", (sizeof "t.1 ") - 1, 100);
#endif

	i2cStatus = HAL_I2C_IsDeviceReady(hi2c, HTU21D_Address, 3, 100);
	if (i2cStatus == HAL_OK) {

#ifdef DEBUG
		HAL_UART_Transmit(&huart2, (uint8_t*)"HAL_OK_T ", (sizeof "HAL_OK_T ") - 1, 100);
#endif
		/* Temperature ---->; */
		HAL_I2C_Mem_Read(hi2c, HTU21D_Address, TEMPERATURE_CMD,
				I2C_MEMADD_SIZE_8BIT, (uint8_t*) RX_Data, 3, 1000);
		rawData = ((uint16_t) (RX_Data[0] << 8) | (RX_Data[1]));
		temperature.value = (float) (rawData * 175.72 / 65536.00) - 46.85;

		if (checkCRC8(rawData) == RX_Data[2] && RX_Data[2] != 0) {
#ifdef DEBUG
			HAL_UART_Transmit(&huart2, (uint8_t*)"t.2\r\n", (sizeof "t.2\r\n") - 1, 100);
#endif
			temperature.measuredSuccessful = true;
		}
	} else {
		switch(i2cStatus) {
		case HAL_ERROR:
			HAL_UART_Transmit(&huart2, (uint8_t*)"HAL_ERROR_T\r\n", (sizeof "HAL_ERROR_T\r\n") - 1, 100);
			break;
		case HAL_BUSY:
			HAL_UART_Transmit(&huart2, (uint8_t*)"HAL_BUSY_T\r\n", (sizeof "HAL_BUSY_T\r\n") - 1, 100);
			break;
		case HAL_TIMEOUT:
			HAL_UART_Transmit(&huart2, (uint8_t*)"HAL_TIMEOUT_T\r\n", (sizeof "HAL_TIMEOUT_T\r\n") - 1, 100);
			break;
		}
	}
	return temperature;
}

void Si7021::measureTemperatureAndHumidity(void) {
	si7021 humidityResult = measureHumidity();
	HAL_Delay(100);
	si7021 temperatureResult = measureTemperature();
	HAL_Delay(100);

	if(humidityResult.measuredSuccessful && temperatureResult.measuredSuccessful) {
		measuredSuccessful = true;
		humidity = humidityResult.value;
		temperature = temperatureResult.value;
	} else {
		measuredSuccessful = false;
		humidity = 0;
		temperature = 0;
	}
}

uint8_t Si7021::checkCRC8(uint16_t data) {
	for (uint8_t bit = 0; bit < 16; bit++) {
		if (data & 0x8000)
			data = (data << 1) ^ HTU21D_CRC8_POLYNOMINAL;
		else
			data <<= 1;
	}
	return data >>= 8;
}

float Si7021::getTemperature(void) {
	return temperature;
}

float Si7021::getHumidity(void) {
	return humidity;
}
