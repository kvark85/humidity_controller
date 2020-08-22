#include "Si7021.h"

Si7021::Si7021(I2C_HandleTypeDef *hi2c) {
	this->hi2c = hi2c;
}

void Si7021::measureTemperatureAndHumidity(void) {
	measuredSuccessful = false;

	/* Temperature ---->; */
	HAL_I2C_Mem_Read(hi2c, HTU21D_Adress, temperature_Cmd,
	I2C_MEMADD_SIZE_8BIT, (uint8_t*) RX_Data, 3, 1000);
	ADC_Raw = ((uint16_t) (RX_Data[0] << 8) | (RX_Data[1]));
	temperature = (float) (ADC_Raw * 175.72 / 65536.00) - 46.85;

	if (checkCRC8(ADC_Raw) == RX_Data[2] && RX_Data[2] != 0) {
		HAL_Delay(100);

		/* Humidity ---->; */
		HAL_I2C_Mem_Read(hi2c, HTU21D_Adress, humidity_Cmd,
		I2C_MEMADD_SIZE_8BIT, (uint8_t*) RX_Data, 3, 1000);
		ADC_Raw = ((uint16_t) (RX_Data[0] << 8) | (RX_Data[1]));
		humidity = (float) (ADC_Raw * 125.0 / 65536.0) - 6.0;

		if (checkCRC8(ADC_Raw) == RX_Data[2] && RX_Data[2] != 0) {
			measuredSuccessful = true;
		}
	}
	HAL_Delay(100);
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
