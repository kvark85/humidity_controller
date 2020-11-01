/*
 * json.h
 *
 *  Created on: 15 окт. 2020 г.
 *      Author: kvark85
 */

#ifndef INC_JSON_H_
#define INC_JSON_H_

#include "main.h"
#include "Si7021.h"

class JSON {
public:
	static uint8_t numberToString(float number, uint8_t *strArray, uint8_t fractionalNumber);
	static uint8_t sideInfo(Si7021 sensor, uint8_t *strArray);
	static uint8_t createJsonString(Si7021 internalSensor, Si7021 externalSensor, uint8_t motor, uint8_t *strArray);
};

#endif /* INC_JSON_H_ */
