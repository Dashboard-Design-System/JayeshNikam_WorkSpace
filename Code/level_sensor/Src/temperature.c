/*
 * temperature.c
 *
 *  Created on: Jul 26, 2025
 *      Author: hp15s-pc
 */


#include "temperature.h"

uint8_t DHT_data[5];  // Global data buffer

void delay_us(uint32_t us) {
    SysTick->LOAD = (us * (SystemCoreClock / 1000000)) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = 5; // Enable counter, no interrupt
    while (!(SysTick->CTRL & (1 << 16))); // Wait until COUNTFLAG is set
    SysTick->CTRL = 0; // Stop timer
}

void DHT_set_output(void) {
    DHT_PORT->MODER &= ~(3 << (DHT_PIN * 2));     // Clear mode bits
    DHT_PORT->MODER |=  (1 << (DHT_PIN * 2));     // Set as output
    DHT_PORT->OTYPER &= ~(1 << DHT_PIN);          // Push-pull
    DHT_PORT->OSPEEDR |= (3 << (DHT_PIN * 2));    // High speed
}

void DHT_set_input(void) {
    DHT_PORT->MODER &= ~(3 << (DHT_PIN * 2));     // Set as input
}

void DHT_write(uint8_t value) {
    if (value)
        DHT_PORT->BSRR = (1 << DHT_PIN);
    else
        DHT_PORT->BSRR = (1 << (DHT_PIN + 16));
}

uint8_t DHT_read(void) {
    return (DHT_PORT->IDR & (1 << DHT_PIN)) != 0;
}

uint8_t DHT_read_sensor(uint8_t *temperature, uint8_t *humidity) {
    uint8_t i, j;

    // Start signal to DHT11
    DHT_set_output();
    DHT_write(0);
    delay_us(18000); // ≥18ms LOW
    DHT_write(1);
    delay_us(30);
    DHT_set_input();

    // Wait for DHT11 response
    delay_us(40);
    if (DHT_read()) return 1;
    delay_us(80);
    if (!DHT_read()) return 2;
    delay_us(80);

    // Read 5 bytes
    for (j = 0; j < 5; j++) {
        DHT_data[j] = 0;
        for (i = 0; i < 8; i++) {
            while (!DHT_read()); // Wait for HIGH
            delay_us(30);
            if (DHT_read()) {
                DHT_data[j] |= (1 << (7 - i));
                while (DHT_read()); // Wait for LOW
            }
        }
    }

    // Checksum validation
    uint8_t checksum = DHT_data[0] + DHT_data[1] + DHT_data[2] + DHT_data[3];
    if (checksum != DHT_data[4]) return 3; // checksum failed

    // Only assign if checksum is valid
    *humidity = DHT_data[0];
    *temperature = DHT_data[2];

    return 0; // success
}
