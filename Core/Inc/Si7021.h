#ifndef _SI7021_H
#define _SI7021_H

#include "main.h"

#define HTU21D_Adress (0x40 << 1)
#define HTU21D_CRC8_POLYNOMINAL 0x13100   //crc8 polynomial for 16bit value, CRC8 -> x^8 + x^5 + x^4 + 1

class Si7021 {
private:
	I2C_HandleTypeDef *hi2c;
	uint8_t RX_Data[3];
	float temperature;
	float humidity;
	uint16_t ADC_Raw;
	uint8_t temperature_Cmd = 0xE3;
	uint8_t humidity_Cmd = 0xE5;

	uint8_t checkCRC8(uint16_t data);
public:
	bool measuredSuccessful = false;

	Si7021(I2C_HandleTypeDef *hi2c);
	void measureTemperatureAndHumidity(void);
	float getTemperature(void);
	float getHumidity(void);
};
#endif
