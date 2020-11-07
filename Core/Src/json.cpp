/*
 * json.cpp
 *
 *  Created on: 15 окт. 2020 г.
 *      Author: kvark85
 */

#include "json.h"

uint8_t* subStrings[] = {
		(uint8_t*)"{\"i\":",
		(uint8_t*)",\"e\":",
		(uint8_t*)",\"m\":",
		(uint8_t*)"}\r\n",
};

uint8_t JSON::numberToString(float number, uint8_t *strArray, uint8_t fractionalNumber) {
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

uint8_t JSON::sideInfo(Si7021 sensor, uint8_t *strArray) {
	// "error"
	// {"rH":1.23,"aH":1.23,"T":33.33}
	uint8_t* sideSubStrings[] = {
			(uint8_t*)"{\"rH\":",
			(uint8_t*)",\"aH\":",
			(uint8_t*)",\"T\":",
			(uint8_t*)"}"
	};
	uint8_t substringIndex = 0;
	uint8_t numberArray[10] = {};
	uint8_t messageLength = 0;
	uint8_t numberLength = 0;

	if(!sensor.measuredSuccessful) {
		uint8_t* errorString = (uint8_t*)"\"error\"";

		while(errorString[substringIndex] != '\0') {
			strArray[substringIndex] = errorString[substringIndex];
			substringIndex++;
		}
		return substringIndex;
	}

	//---------------------------------------------------------------------------------------------------------
	// "{'rH':"
	while(sideSubStrings[0][substringIndex] != '\0') {
		strArray[messageLength] = sideSubStrings[0][substringIndex];
		messageLength++;
		substringIndex++;
	}
	numberLength = numberToString(sensor.getRelativeHumidity(), &numberArray[0], 2);
	for(uint8_t numberIndex = 0; numberIndex < numberLength; numberIndex++) {
		strArray[messageLength] = numberArray[numberIndex];
		messageLength++;
	}
	//---------------------------------------------------------------------------------------------------------
	// ",'aH\':"
	substringIndex = 0;
	while(sideSubStrings[1][substringIndex] != '\0') {
		strArray[messageLength] = sideSubStrings[1][substringIndex];
		messageLength++;
		substringIndex++;
	}
	numberLength = numberToString(sensor.getAbsoluteHumidity(), &numberArray[0], 2);
	for(uint8_t numberIndex = 0; numberIndex < numberLength; numberIndex++) {
		strArray[messageLength] = numberArray[numberIndex];
		messageLength++;
	}
	//---------------------------------------------------------------------------------------------------------
	// ",'T\':"
	substringIndex = 0;
	while(sideSubStrings[2][substringIndex] != '\0') {
		strArray[messageLength] = sideSubStrings[2][substringIndex];
		messageLength++;
		substringIndex++;
	}
	numberLength = numberToString(sensor.getTemperature(), &numberArray[0], 2);
	for(uint8_t numberIndex = 0; numberIndex < numberLength; numberIndex++) {
		strArray[messageLength] = numberArray[numberIndex];
		messageLength++;
	}
	//---------------------------------------------------------------------------------------------------------
	// "}"
	strArray[messageLength] = sideSubStrings[3][0];
	messageLength++;

	//---------------------------------------------------------------------------------------------------------
	return messageLength;
}


uint8_t JSON::createJsonString(Si7021 internalSensor, Si7021 externalSensor, uint8_t motor, uint8_t *strArray) {
	// {"internal":"error","external":{"H":1.23,"T":33.33},"motor":1}

	uint8_t substringIndex = 0;
	uint8_t numberArray[100] = {};
	uint8_t messageLength = 0;
	uint8_t numberLength = 0;
	//---------------------------------------------------------------------------------------------------------
	// "{\"internal\":"
	while(subStrings[0][substringIndex] != '\0') {
		strArray[messageLength] = subStrings[0][substringIndex];
		messageLength++;
		substringIndex++;
	}

	numberLength = sideInfo(internalSensor, &numberArray[0]);
	for(uint8_t numberIndex = 0; numberIndex < numberLength; numberIndex++) {
		strArray[messageLength] = numberArray[numberIndex];
		messageLength++;
	}
	//---------------------------------------------------------------------------------------------------------
	// ",'external':"
	substringIndex = 0;
	while(subStrings[1][substringIndex] != '\0') {
		strArray[messageLength] = subStrings[1][substringIndex];
		messageLength++;
		substringIndex++;
	}
	numberLength = sideInfo(externalSensor, &numberArray[0]);
	for(uint8_t numberIndex = 0; numberIndex < numberLength; numberIndex++) {
		strArray[messageLength] = numberArray[numberIndex];
		messageLength++;
	}
	//---------------------------------------------------------------------------------------------------------
	// ",'motor':"
	substringIndex = 0;
	while(subStrings[2][substringIndex] != '\0') {
		strArray[messageLength] = subStrings[2][substringIndex];
		messageLength++;
		substringIndex++;
	}
	numberLength = numberToString((float)(motor), &numberArray[0], 0);
	for(uint8_t numberIndex = 0; numberIndex < numberLength; numberIndex++) {
		strArray[messageLength] = numberArray[numberIndex];
		messageLength++;
	}
	//---------------------------------------------------------------------------------------------------------
	// "}\r\n"
	substringIndex = 0;
	while(subStrings[3][substringIndex] != '\0') {
		strArray[messageLength] = subStrings[3][substringIndex];
		messageLength++;
		substringIndex++;
	}
	//---------------------------------------------------------------------------------------------------------
	return messageLength;
}


