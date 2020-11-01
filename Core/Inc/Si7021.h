#ifndef _SI7021_H
#define _SI7021_H

#include "main.h"

/**
 * @brief value from sensor
 */
typedef struct
{
	float value;
	uint8_t measuredSuccessful;
}si7021;

#define HTU21D_Address (0x40 << 1)
#define HTU21D_CRC8_POLYNOMINAL 0x13100   //crc8 polynomial for 16bit value, CRC8 -> x^8 + x^5 + x^4 + 1
#define TEMPERATURE_CMD 0xE3
#define HUMIDITY_CMD 0xE5

class Si7021 {
private:
	I2C_HandleTypeDef *hi2c;
	float temperature;
	float humidity;
	si7021 measureHumidity(void);
	si7021 measureTemperature(void);
	uint8_t checkCRC8(uint16_t data);
public:
	bool measuredSuccessful = false;
	Si7021(I2C_HandleTypeDef *hi2c);
	void measureTemperatureAndHumidity(void);
	float getTemperature(void);
	float getHumidity(void);
};
#endif
