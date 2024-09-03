#include "stm32f7xx_hal.h"

// Define the GPIO pin and port used for One-Wire communication
#define DS18B20_PIN GPIO_PIN_6
#define DS18B20_PORT GPIOB

// Function to set the GPIO pin state
void DS18B20_WritePin(GPIO_PinState state) {
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, state);
}

// Function to read the GPIO pin state
GPIO_PinState DS18B20_ReadPin(void) {
    return HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN);
}

// Delay function using TIM7 (with 1MHz clock, so 1 tick = 1 µs)
extern TIM_HandleTypeDef htim7;

void Delay_us(uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim7, 0);  // Set the timer counter to 0
    while (__HAL_TIM_GET_COUNTER(&htim7) < us);  // Wait until the counter reaches the specified value
}

// 1-Wire reset pulse function
uint8_t OneWire_Reset(void) {
    uint8_t response;

    DS18B20_WritePin(GPIO_PIN_RESET);  // Pull the One-Wire line low
    Delay_us(480);                      // Wait for 480 µs
    DS18B20_WritePin(GPIO_PIN_SET);    // Release the line
    Delay_us(80);                       // Wait for 80 µs

    response = DS18B20_ReadPin();       // Check if DS18B20 pulls the line low
    Delay_us(400);                      // Wait for the rest of the timeslot

    return response == GPIO_PIN_RESET ? 1 : 0;  // 1 = presence detected, 0 = no device
}

// Function to write one bit to the DS18B20
void OneWire_WriteBit(uint8_t bit) {
    if (bit) {
        DS18B20_WritePin(GPIO_PIN_RESET); // Pull the line low
        Delay_us(1);                      // Wait for 1 µs
        DS18B20_WritePin(GPIO_PIN_SET);  // Release the line
        Delay_us(60);                     // Wait for the rest of the timeslot (60 µs)
    } else {
        DS18B20_WritePin(GPIO_PIN_RESET); // Pull the line low
        Delay_us(60);                     // Keep the line low for 60 µs
        DS18B20_WritePin(GPIO_PIN_SET);  // Release the line
    }
}

// Function to read one bit from the DS18B20
uint8_t OneWire_ReadBit(void) {
    uint8_t bit;

    DS18B20_WritePin(GPIO_PIN_RESET); // Pull the line low
    Delay_us(1);                      // Wait for 1 µs
    DS18B20_WritePin(GPIO_PIN_SET);  // Release the line
    Delay_us(14);                     // Wait for the DS18B20 to pull the line low
    bit = DS18B20_ReadPin();          // Read the bit value
    Delay_us(45);                     // Wait for the rest of the timeslot (45 µs)

    return bit;
}

// Function to write one byte to the DS18B20
void OneWire_WriteByte(uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        OneWire_WriteBit(byte & 0x01);  // Write LSB first
        byte >>= 1;                     // Shift right for the next bit
    }
}

// Function to read one byte from the DS18B20
uint8_t OneWire_ReadByte(void) {
    uint8_t byte = 0;
    for (uint8_t i = 0; i < 8; i++) {
        byte >>= 1;                     // Shift right to make room for the next bit
        if (OneWire_ReadBit()) {
            byte |= 0x80;               // If the bit is '1', set the MSB
        }
    }
    return byte;
}
