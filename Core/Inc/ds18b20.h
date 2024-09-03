#ifndef __ONEWIRE_H
#define __ONEWIRE_H

#include "stm32f7xx_hal.h"

// GPIO settings for One-Wire communication
#define DS18B20_PIN GPIO_PIN_7
#define DS18B20_PORT GPIOF

// Function prototypes
void DS18B20_WritePin(GPIO_PinState state);
GPIO_PinState DS18B20_ReadPin(void);
void Delay_us(uint16_t us);

uint8_t OneWire_Reset(void);
void OneWire_WriteBit(uint8_t bit);
uint8_t OneWire_ReadBit(void);
void OneWire_WriteByte(uint8_t byte);
uint8_t OneWire_ReadByte(void);

#endif /* __ONEWIRE_H */
