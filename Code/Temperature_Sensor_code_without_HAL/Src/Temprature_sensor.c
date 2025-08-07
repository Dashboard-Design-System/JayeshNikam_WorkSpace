/*
 * lm35.c
 *
 *  Created on: Jun 5, 2025
 *      Author: rahul
 */

#include "Temperature_sensor.h"
#include "stm32f4xx.h"

//Pinout Configuration
// GND-GND PA0--Vout  Vcc-5v Stm

void Rd_35_Init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; 	//GPIOA enable for PA0
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;  //for ADC 1 Enaable

	GPIOA->MODER |= BV(0) | BV(1);  //Mode set as analog input 11 for PA0 pin
	ADC1->CR2=0;	//disabled before config
	ADC1->SQR3=0; //Regular Sequence register 3 PA0=ADC1_IN0
	ADC1->SQR1=0; 	// 0 means 1 conversion
	ADC1->SMPR2 |= BV(0) | BV(1) | BV(2);  // Set bits 2:0 = 111 => 480 cycles
	ADC1->CR2 |= ADC_CR2_ADON;   //after all config turn on adc 1

}


float ReadTemperature(void)
{

	ADC1->CR2 |= ADC_CR2_SWSTART;
	while(!(ADC1->SR & ADC_SR_EOC));//wait untill end of conversion flag is set

	uint16_t adc_val= ADC1->DR;	//12 bit register data is read.(0 to 4095)

	 // Convert to voltage (3.3V ref, 12-bit ADC)
	int temperature = (adc_val / 4095.0f) * 100.0f;
	return temperature;



;
}



