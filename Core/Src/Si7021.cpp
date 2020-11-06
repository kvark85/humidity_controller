#include "Si7021.h"
#include "programI2C.h"

extern UART_HandleTypeDef huart2;

Si7021::Si7021(I2C *i2c) {
	this->i2c = i2c;
}

si7021 Si7021::measureHumidity(void) {
	uint8_t RX_Data[3] = {0};
	uint16_t rawData;
	bool isReadSuccessful = false;
	si7021 humidity;
	humidity.value = 0;
	humidity.measuredSuccessful = false;

	isReadSuccessful = i2c->readInHoldMode(HTU21D_Address, HUMIDITY_CMD, RX_Data, 3, 100);

	if(isReadSuccessful == false) {
		return humidity;
	}

	rawData = ((uint16_t) (RX_Data[0] << 8) | (RX_Data[1]));

	humidity.measuredSuccessful = checkCRC8(rawData) == RX_Data[2] && RX_Data[2] != 0;

	if(humidity.measuredSuccessful == false) {
		return humidity;
	}

	humidity.value = (float) (rawData * 125.0 / 65536.0) - 6.0;

	return humidity;
}

si7021 Si7021::measureTemperature(void) {
	uint8_t RX_Data[3] = {0};
	uint16_t rawData;
	bool isReadSuccessful = false;
	si7021 temperature;
	temperature.value = 0;
	temperature.measuredSuccessful = false;

	isReadSuccessful = i2c->readInHoldMode(HTU21D_Address, TEMPERATURE_CMD, RX_Data, 3, 100);

	if(isReadSuccessful == false) {
		return temperature;
	}

	rawData = ((uint16_t) (RX_Data[0] << 8) | (RX_Data[1]));

	temperature.measuredSuccessful = checkCRC8(rawData) == RX_Data[2] && RX_Data[2] != 0;

	if(temperature.measuredSuccessful == false) {
		return temperature;
	}

	temperature.value = (float) (rawData * 175.72 / 65536.00) - 46.85;

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
