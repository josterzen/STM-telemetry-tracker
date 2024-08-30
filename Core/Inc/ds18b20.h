#ifndef DS18B20_H
#define DS18B20_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_tim.h"

typedef struct {
    TIM_HandleTypeDef *tim;
    GPIO_TypeDef *port;
    uint16_t pin;
} DS18B20;

void DS18B20_init(DS18B20 *sensor, TIM_HandleTypeDef *tim, GPIO_TypeDef *port, uint16_t pin);
float DS18B20_read_temp_celsius(DS18B20 *sensor);
float DS18B20_read_temp_fahrenheit(DS18B20 *sensor);

void DS18B20_set_data_pin(DS18B20 *sensor, bool on);
void DS18B20_toggle_data_pin(DS18B20 *sensor);
void DS18B20_set_pin_output(DS18B20 *sensor);
void DS18B20_set_pin_input(DS18B20 *sensor);
GPIO_PinState DS18B20_read_data_pin(DS18B20 *sensor);
void DS18B20_start_sensor(DS18B20 *sensor);
void DS18B20_writeData(DS18B20 *sensor, uint8_t data);
uint8_t DS18B20_read_data(DS18B20 *sensor);
void DS18B20_delay(DS18B20 *sensor, uint16_t us);

#endif // DS18B20_H

