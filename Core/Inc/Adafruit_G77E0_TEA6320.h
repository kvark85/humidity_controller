/**************************************************************************/
/*!
  @file Adafruit_G77E0_TEA6320.h

  This is a library for our Monochrome G77E0 (TEA6320) LCD Displays

  These displays use I2C
 */
/**************************************************************************/
#ifndef _ADAFRUIT_G77E0_TEA6320_H
#define _ADAFRUIT_G77E0_TEA6320_H

#include "main.h"
#include <Adafruit_GFX.h>

#define LCD_SLAVE_ADDRESS 0b01111000
#define I2C_DELAY 400

#define BLACK 1 ///< Black pixel
#define WHITE 0 ///< White pixel

#define LCD_WIDTH 128	///< LCD is 128 pixels wide
#define LCD_HEIGHT 64	///< 64 pixels high

#define ALL_LCD_PIXELS (LCD_WIDTH * 8) + (5 * 8) ///< Amount of all LCD pixels (including invisible)
#define ALL_VISIBLE_LCD_PIXELS LCD_WIDTH * 8 ///< Amount of all LCD pixels (including invisible)

extern I2C_HandleTypeDef hi2c1;

#define FFT_INVERSE_FLAG        ((uint8_t)0)
#define FFT_Normal_OUTPUT_FLAG  ((uint8_t)1)
#define FFT_LENGTH  			256

/**************************************************************************/
/*!
    @brief The PCD8544 LCD class
 */
class Adafruit_G77E0_TEA6320 : public Adafruit_GFX {
private:
	void printNumber(uint8_t number);

public:
	Adafruit_G77E0_TEA6320(void);

	void drawPixel(int16_t x, int16_t y, uint16_t color);
	void initDisplay(void);
	void display(void);
	void clearDisplay(void);
	void setLcdCursorPosition(uint8_t x, uint8_t y);
	void println(const char* string);
	void drawSpectrum(uint16_t * inputArray);
	void drawFloat(float number);
	void drawInt(float number);
};

#endif
