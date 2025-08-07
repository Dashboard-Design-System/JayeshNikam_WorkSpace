/*
 * lm35.h
 *
 *  Created on: Jun 5, 2025
 *      Author: rahul
 */

#ifndef Temperature_H_
#define Temperature_H_

#include "stm32f4xx.h"


void Temp_Init(void);
float ReadTemperature(void);

#endif /* LM35_H_ */
