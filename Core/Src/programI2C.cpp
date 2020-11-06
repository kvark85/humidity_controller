/**************************************************************************/
/*!
 @file I2C.cpp

 This is a library for program I2C
 */
/**************************************************************************/

#include "main.h"
#include "programI2C.h"

volatile uint8_t i2c_frame_error=0;

//-----------------------------------------------------------
void Delay_us (uint32_t __IO us) { // delay function in us
	us *=(SystemCoreClock/1000000)/15;
	while(us--);
}
//----------------------------------------------------
void I2C::setAck(void) {
	HAL_GPIO_WritePin(i2cPort, sclPin, GPIO_PIN_RESET); // SCL_LOW;
	Delay_us(DELAY_US);
	HAL_GPIO_WritePin(i2cPort, sdaPin, GPIO_PIN_RESET); // SDA_lOW;
	Delay_us(DELAY_US);
	HAL_GPIO_WritePin(i2cPort, sclPin, GPIO_PIN_SET); // SCL_HIGH;
	Delay_us(DELAY_US);
	HAL_GPIO_WritePin(i2cPort, sclPin, GPIO_PIN_RESET); // SCL_LOW;
	Delay_us(DELAY_US);
	HAL_GPIO_WritePin(i2cPort, sdaPin, GPIO_PIN_SET); // SDA_HIGH;
}
//----------------------------------------------------
void I2C::setNAck(void) {
	HAL_GPIO_WritePin(i2cPort, sclPin, GPIO_PIN_RESET); // SCL_LOW;
	Delay_us(DELAY_US);
	HAL_GPIO_WritePin(i2cPort, sdaPin, GPIO_PIN_SET); // SDA_HIGH;
	Delay_us(DELAY_US);
	HAL_GPIO_WritePin(i2cPort, sclPin, GPIO_PIN_SET); // SCL_HIGH;
	Delay_us(DELAY_US);
	HAL_GPIO_WritePin(i2cPort, sclPin, GPIO_PIN_RESET); // SCL_LOW;
	Delay_us(DELAY_US);
}
//----------------------------------------------------
uint8_t I2C::wait_ACK (void) {
	uint8_t ack;

	HAL_GPIO_WritePin(i2cPort, sdaPin, GPIO_PIN_SET); // SDA_HIGH;
	Delay_us(DELAY_US);
	HAL_GPIO_WritePin(i2cPort, sclPin, GPIO_PIN_SET); // SCL_HIGH;
	Delay_us(DELAY_US);

	if(HAL_GPIO_ReadPin(i2cPort, sdaPin) == 0)
		ack = I2C_ACK;
	else
		ack = I2C_NACK;

	//	HAL_GPIO_WritePin(i2cPort, sdaPin, GPIO_PIN_RESET); // SDA_lOW;
	HAL_GPIO_WritePin(i2cPort, sclPin, GPIO_PIN_RESET); // SCL_LOW;
	Delay_us(DELAY_US);

	return ack;
}
//----------------------------------------------------
I2C::I2C(uint32_t sclPin, uint32_t sdaPin, GPIO_TypeDef  *i2cPort) {
	this->sclPin = sclPin;
	this->sdaPin = sdaPin;
	this->i2cPort = i2cPort;
}

//----------------------------------------------------
void I2C::start (void) {
	HAL_GPIO_WritePin(i2cPort, sclPin, GPIO_PIN_SET); // SCL_HIGH;
	Delay_us(DELAY_US);
	HAL_GPIO_WritePin(i2cPort, sdaPin, GPIO_PIN_SET); // SDA_HIGH;
	Delay_us(DELAY_US);
	HAL_GPIO_WritePin(i2cPort, sdaPin, GPIO_PIN_RESET); // SDA_LOW;
	Delay_us(DELAY_US);
	HAL_GPIO_WritePin(i2cPort, sclPin, GPIO_PIN_RESET); // SCL_LOW;
	Delay_us(DELAY_US);
}
//----------------------------------------------------
void I2C::stop (void) {
	HAL_GPIO_WritePin(i2cPort, sdaPin, GPIO_PIN_RESET); // SDA_LOW;
	Delay_us(DELAY_US);
	HAL_GPIO_WritePin(i2cPort, sclPin, GPIO_PIN_SET); // SCL_HIGH;
	Delay_us(DELAY_US);
	HAL_GPIO_WritePin(i2cPort, sdaPin, GPIO_PIN_SET); // SDA_HIGH;
	Delay_us(DELAY_US);
}
//----------------------------------------------------
void I2C::write_byte (uint8_t data) {
	uint8_t count;

	for(count = 0; count < 8; count++) {
		HAL_GPIO_WritePin(i2cPort, sclPin, GPIO_PIN_RESET); // SCL_LOW;
		Delay_us(DELAY_US);

		if(data & 0x80) {
			HAL_GPIO_WritePin(i2cPort, sdaPin, GPIO_PIN_SET); // SDA_HIGH;
		} else {
			HAL_GPIO_WritePin(i2cPort, sdaPin, GPIO_PIN_RESET); // SDA_LOW;
		}
		data <<= 1;
		Delay_us(DELAY_US);

		HAL_GPIO_WritePin(i2cPort, sclPin, GPIO_PIN_SET); // SCL_HIGH;
		Delay_us(DELAY_US);
	}

	HAL_GPIO_WritePin(i2cPort, sclPin, GPIO_PIN_RESET); // SCL_LOW;
	Delay_us(DELAY_US);
}
//----------------------------------------------------
uint8_t I2C::read_byte (uint8_t ack)
{
	uint8_t cnt, dat;

	Delay_us(DELAY_US);

	HAL_GPIO_WritePin(i2cPort, sdaPin, GPIO_PIN_SET); // SDA_HIGH;

	for(cnt = 0; cnt < 8; cnt++)
	{
		HAL_GPIO_WritePin(i2cPort, sclPin, GPIO_PIN_SET); // SCL_HIGH;
		Delay_us(DELAY_US);

		dat <<= 1;

		if(HAL_GPIO_ReadPin(i2cPort, sdaPin)) {
			dat |= 0x01;
		} else {
			dat &= 0xfe;
		}
		HAL_GPIO_WritePin(i2cPort, sclPin, GPIO_PIN_RESET); // SCL_LOW;
		Delay_us(DELAY_US);
	}
	if(ack == I2C_NACK) {
		this->setNAck();
	} else {
		this->setAck();
	}

	return dat;
}
//----------------------------------------------------
bool I2C::readInHoldMode(uint8_t address, uint8_t comand, uint8_t *dataArray, uint8_t length, uint16_t timeout) {

	if(!HAL_GPIO_ReadPin(i2cPort, sclPin) || !HAL_GPIO_ReadPin(i2cPort, sdaPin)) {
		this->start();
		HAL_Delay(1);
		this->stop();
		HAL_Delay(1);

		return false;
	}

	// FIRST START
	this->start();
	// ADDRESS
	this->write_byte(address << 1);
	if(this->wait_ACK() == I2C_NACK) {
		return false;
	}

	// COMMAND
	this->write_byte(comand);
	if(this->wait_ACK() == I2C_NACK) {
		return false;
	}


	// DUBBLED START
	this->start();
	//ADDRESS
	this->write_byte((address << 1) | 0b00000001);
	if(this->wait_ACK() == I2C_NACK) return false;

	HAL_GPIO_WritePin(this->i2cPort, this->sclPin, GPIO_PIN_SET); // SCL_HIGH;

	while (HAL_GPIO_ReadPin(this->i2cPort, this->sclPin) == 0) {
		Delay_us(DELAY_US);
	};

	for(uint8_t count = 0; count < length; count++) {
		if (count == (length -1)) {
			dataArray[count] = this->read_byte(I2C_NACK);
		} else {
			dataArray[count] = this->read_byte(I2C_ACK);;
		}
	}

	// STOP
	this->stop();

	return true;
};


