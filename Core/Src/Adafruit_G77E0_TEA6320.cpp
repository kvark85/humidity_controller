/**************************************************************************/
/*!
 @file Adafruit_G77E0_TEA6320.cpp

 @mainpage Adafruit G77E0 (TEA6320) LCD Library

 @section intro Introduction

 This is a library for our Monochrome G77E0 (TEA6320) LCD Displays

 These displays use I2C
 */
/**************************************************************************/

#include "Adafruit_G77E0_TEA6320.h"

#ifndef _BV
#define _BV(bit) (128>>(bit))
#endif

/** the memory buffer for the LCD */
uint8_t pcd8544_buffer[ALL_VISIBLE_LCD_PIXELS] = { };
uint16_t previousAnalysis[FFT_LENGTH / 2] = { 0 };
char *numbers[10] = { "0", "1", "2", "3", "4", "5", "6", "7", "8", "9" };

/*!
 @brief Constructor for hardware I2C
 */
Adafruit_G77E0_TEA6320::Adafruit_G77E0_TEA6320() :
				Adafruit_GFX(LCD_WIDTH, LCD_HEIGHT) {
}

/*!
 @brief The most basic function, set a single pixel
 @param x     x coord
 @param y     y coord
 @param color pixel color (BLACK or WHITE)
 */
void Adafruit_G77E0_TEA6320::drawPixel(int16_t x, int16_t y, uint16_t color) {
	if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height))
		return;

	int16_t t;
	switch (rotation) {
	case 1:
		t = x;
		x = y;
		y = LCD_HEIGHT - 1 - t;
		break;
	case 2:
		x = LCD_WIDTH - 1 - x;
		y = LCD_HEIGHT - 1 - y;
		break;
	case 3:
		t = x;
		x = LCD_WIDTH - 1 - y;
		y = t;
		break;
	}

	if ((x < 0) || (x >= LCD_WIDTH) || (y < 0) || (y >= LCD_HEIGHT))
		return;

	// x is which column
	if (color)
		pcd8544_buffer[x + (y / 8) * LCD_WIDTH] |= _BV(y % 8);
	else
		pcd8544_buffer[x + (y / 8) * LCD_WIDTH] &= ~_BV(y % 8);
}

/*!
 @brief Initialize the display. Set bias and contrast, enter normal mode.
 */
void Adafruit_G77E0_TEA6320::initDisplay() {
	uint8_t initArray[15] = { 0b00000000,		// control byte, Co = 0, D/C = 0
			0b00000001,	// H[2:0] independent command; select function and RAM command page H[1:0] = 111
			0b00010000,	// function and RAM command page; PD = 0 (chip is active), V = 0 (horizontal addressing)
			0b00001110,	// function and RAM command page; select display setting command page H[1:0] = 110
			0b00010000 | 3,	// BIAS = 1/9
			0b00000110,		// display mode D=1, E = 0 - normal mode
			0b10000100,		// MUX = 1/65
			0b00001010,		// mirror (MX = 0, MY = 1)
			0b00100100,		// IB = 1
			0b00000001,	// H[2:0] independent command; select function and RAM command page H[1:0] = 111
			0b00001101,	// function and RAM command page; select HV-gen command page H[2:0] = 101
			0b00001001,		// S[1:0] = 01, mul factor = 3 (3 * V dd2)
			0b00010010,	// HV-gen command page; select temperature coefficient 2 TC[2:0] = 010
			0b10101000, 0b00000111, };

	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1,
			LCD_SLAVE_ADDRESS, initArray, (uint16_t) 15, (uint32_t) I2C_DELAY);
	//	while(status != HAL_OK);
	clearDisplay();
}

/*!
 @brief Update the display
 */
void Adafruit_G77E0_TEA6320::display(void) {
	uint8_t arrayToWrite[1 + ALL_LCD_PIXELS] = { 0b01000000 };
	uint16_t count = 0;

	for (int y = 0; y < (LCD_HEIGHT / 8); y++) {
		for (int x = 0; x < LCD_WIDTH; x++) {
			count++;
			arrayToWrite[count] = pcd8544_buffer[(LCD_WIDTH * y) + x];
		}
		count += 5;
	}

	setLcdCursorPosition(0, 0);
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1,
			LCD_SLAVE_ADDRESS, arrayToWrite, 1 + ALL_LCD_PIXELS, I2C_DELAY);
	//	while(status != HAL_OK);
}

/*!
 @brief Clear the entire display
 */
void Adafruit_G77E0_TEA6320::clearDisplay(void) {
	uint8_t arrayToClear[1 + ALL_LCD_PIXELS] = { 0b01000000 };
	for (int i = 1; i < ALL_LCD_PIXELS; i++)
		arrayToClear[i] = 0b00000000;
	setLcdCursorPosition(0, 0);

	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1,
			LCD_SLAVE_ADDRESS, arrayToClear, 1 + ALL_LCD_PIXELS, I2C_DELAY);
	//	while(status != HAL_OK);
}

/*!
 @brief Set the cursor position on LCD display
 */
void Adafruit_G77E0_TEA6320::setLcdCursorPosition(uint8_t x, uint8_t y) {
	uint8_t setPositionArray[5] = { 0b00000000,		// control byte C0=0,D/C=0;
			0b00000001,		// LCD_COMMAND_DEF_PAGE
			0b00100000,		// set ram page 0
			0b01000000 | y,	// set y pos
			0b10000000 | x,	// set x pos
	};
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1,
			LCD_SLAVE_ADDRESS, setPositionArray, 5, I2C_DELAY);
	//	while(status != HAL_OK);
}

/*!
 @brief Print string to LCD memory
 */
void Adafruit_G77E0_TEA6320::println(const char *string) {
	uint8_t stringLength = strlen(string);

	for (int i = 0; i < stringLength; i++) {
		uint8_t symbol = (uint8_t) string[i];

		if (symbol >= 0xC0) {
			i++;
			switch (symbol) {
			case 0xD0: {
				symbol = (uint8_t) string[i] - 1;
				if (symbol == 0x81) {
					symbol = 0xA8;
					break;
				}
				if (symbol >= 0x90 && symbol <= 0xBF)
					symbol = symbol + 0x30;
				break;
			}
			case 0xD1: {
				symbol = (uint8_t) string[i] - 1;
				if (symbol == 0x91) {
					symbol = 0xB8;
					break;
				}
				if (symbol >= 0x80 && symbol <= 0x8F)
					symbol = symbol + 0x70;
				break;
			}
			}
		}

		write(symbol);
	}
}

/*!
 @brief Print string to LCD memory
 */

void Adafruit_G77E0_TEA6320::drawSpectrum(uint16_t *inputArray) {
	uint16_t temp;
	uint16_t i;

	inputArray[0] = 0;
	fillRect(0, 0, 128, 64, WHITE);
	for (i = 0; i < FFT_LENGTH / 2; i++) {
		temp = (uint16_t) (17 * log10f((float) (inputArray[i] + 1))); // also works "log10" and "log10l" functions

		if (temp > previousAnalysis[i]) {
			previousAnalysis[i] = temp;
		} else if (previousAnalysis[i] >= 3) {
			previousAnalysis[i] = previousAnalysis[i] - 3;
		} else {
			previousAnalysis[i] = temp;
		}

		fillRect(i * 1, 64 - previousAnalysis[i], 1, previousAnalysis[i],
				BLACK);
	}

	display();
}

void Adafruit_G77E0_TEA6320::printNumber(uint8_t number) {

}

void Adafruit_G77E0_TEA6320::drawFloat(float number) {
	number *= 100;
	uint8_t temp = 0;
	uint8_t index = 0;
	uint8_t isNegative = false;
	char digits[7] = { 0 };

	if (number < 0) {
		isNegative = true;
		number *= -1;
	}

	if (number >= 10000) {
		if (isNegative) {
			println("-");
			println(numbers[1]);
			println(numbers[0]);
			println(numbers[0]);
		} else {
			println(numbers[1]);
			println(numbers[0]);
			println(numbers[0]);
			println(".");
			println(numbers[0]);
		}
	} else {
		temp = (uint8_t) (number / 1000000);
		if (temp > 0 || digits[0] != 0) {
			digits[index] = temp;
			index++;
			number = number - (temp * 1000000);
		};
		temp = (uint8_t) (number / 100000);
		if (temp > 0 || digits[0] != 0) {
			digits[index] = temp;
			index++;
			number = number - (temp * 100000);
		};
		temp = (uint8_t) (number / 10000);
		if (temp > 0 || digits[0] != 0) {
			digits[index] = temp;
			index++;
			number = number - (temp * 10000);
		};
		temp = (uint8_t) (number / 1000);
		digits[index] = temp;
		index++;
		number = number - (temp * 1000);
		temp = (uint8_t) (number / 100);
		digits[index] = temp;
		index++;
		number = number - (temp * 100);

		temp = (uint8_t) (number / 10);
		digits[index] = temp;
		index++;
		number = number - (temp * 10);

		temp = (uint8_t) ((uint16_t) number % 10);
		digits[index] = temp;
		index++;

		if (isNegative) {
			println("-");
			for (uint8_t outputIndex = 0; outputIndex < (index - 2);
					outputIndex++) {
				println(numbers[digits[outputIndex]]);
			}

			println(".");

			println(numbers[digits[index - 2]]);
		} else {
			for (uint8_t outputIndex = 0; outputIndex < (index - 2);
					outputIndex++) {
				println(numbers[digits[outputIndex]]);
			}

			println(".");

			for (uint8_t outputIndex = index - 2; outputIndex < index;
					outputIndex++) {
				println(numbers[digits[outputIndex]]);
			}
		}

	}

}

void Adafruit_G77E0_TEA6320::drawInt(float number) {
	uint8_t temp = 0;
	uint8_t index = 0;
	char digits[5] = { 0 };

	temp = (uint8_t) (number / 10000);
	if (temp > 0 || digits[0] != 0) {
		digits[index] = temp;
		index++;
		number = number - (temp * 10000);
	};
	temp = (uint8_t) (number / 1000);
	if (temp > 0 || digits[0] != 0) {
		digits[index] = temp;
		index++;
		number = number - (temp * 1000);
	};
	temp = (uint8_t) (number / 100);
	if (temp > 0 || digits[0] != 0) {
		digits[index] = temp;
		index++;
		number = number - (temp * 100);
	};
	temp = (uint8_t) (number / 10);
	if (temp > 0 || digits[0] != 0) {
		digits[index] = temp;
		index++;
		number = number - (temp * 10);
	};
	temp = (uint8_t) number;
	digits[index] = temp;
	index++;

	for (uint8_t outputIndex = 0; outputIndex < (index); outputIndex++) {
		println(numbers[digits[outputIndex]]);
	}
}

/*
// this doesnt touch the buffer, just clears the display RAM - might be handy
void Adafruit_G77E0_TEA6320::clearDisplay(void) {

  uint8_t p, c;

  for(p = 0; p < 8; p++) {

    st7565_command(CMD_SET_PAGE | p);
    for(c = 0; c < 129; c++) {
      //uart_putw_dec(c);
      //uart_putchar(' ');
      st7565_command(CMD_SET_COLUMN_LOWER | (c & 0xf));
      st7565_command(CMD_SET_COLUMN_UPPER | ((c >> 4) & 0xf));
      st7565_data(0x0);
    }
    }

}
 */
