/**************************************************************************/
/*!
  @file I2C.h

  This is a library for program I2C
 */
/**************************************************************************/

#ifndef __I2C_H_H__
#define __I2C_H_H__

#include "main.h"

//---подключение шины к пинам-----------------------------------------------------
#define DELAY_US 10
#define I2C_ACK 0
#define I2C_NACK 1

//--------------------------------------------------------------------------------
void Delay_us (uint32_t __IO us);
//--------------------------------------------------------------------------------
class I2C {
private:
	uint32_t sclPin;
	uint32_t sdaPin;
	GPIO_TypeDef *i2cPort;

	void setAck(void);
	void setNAck(void);
	void start (void);
	void stop (void);
	uint8_t wait_ACK (void);
	void write_byte (uint8_t data);
	uint8_t read_byte (uint8_t ack);
public:
	I2C(uint32_t sclPin, uint32_t sdaPin, GPIO_TypeDef  *i2cPort);
	bool readInHoldMode(uint8_t address, uint8_t comand, uint8_t *dataArray, uint8_t length, uint16_t timeout);
};

#endif /* __I2C_H_H__ */
