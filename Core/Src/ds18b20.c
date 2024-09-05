#include "ds18b20.h"
#include "stm32f7xx_hal.h"
#include "cmsis_os.h"

/*
Constructor for temperature sensor object
@param tim Pointer to hardware timer handle. The timer has to tick every microsecond!
@param port GPIO port of the sensor pin, e.g. GPIOB
@param pin GPIO pin number of the sensor pin
*/

void DS18B20_init(DS18B20 *sensor, TIM_HandleTypeDef *tim, GPIO_TypeDef *port, uint16_t pin) {
    sensor->tim = tim;
    sensor->port = port;
    sensor->pin = pin;
    HAL_TIM_Base_Start(sensor->tim);
}

/*
Block for given time in microseconds by waiting for the htim ticks
*/
void DS18B20_delay(DS18B20 *sensor, uint16_t us) {
	__HAL_TIM_SET_COUNTER(sensor->tim, 0);
	while ((__HAL_TIM_GET_COUNTER(sensor->tim))<us);
}

void DS18B20_set_data_pin(DS18B20 *sensor, bool on) {
    HAL_GPIO_WritePin(sensor->port, sensor->pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void DS18B20_toggle_data_pin(DS18B20 *sensor) {
    HAL_GPIO_TogglePin(sensor->port, sensor->pin);
}

GPIO_PinState DS18B20_read_data_pin(DS18B20 *sensor) {
    return HAL_GPIO_ReadPin(sensor->port, sensor->pin);
}

void DS18B20_set_pin_output(DS18B20 *sensor) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = sensor->pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(sensor->port, &GPIO_InitStruct);
}

void DS18B20_set_pin_input(DS18B20 *sensor) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = sensor->pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(sensor->port, &GPIO_InitStruct);
}

uint8_t DS18B20_Start (DS18B20 *sensor)
{
    //taskENTER_CRITICAL();
	uint8_t response = 0;
	DS18B20_set_pin_output(sensor);   // set the pin as output
	DS18B20_set_data_pin(sensor, false);  // pull the pin low
	DS18B20_delay(sensor, 480);   // delay according to datasheet //600

	DS18B20_set_pin_input(sensor);    // set the pin as input
	DS18B20_delay(sensor, 80);    // delay according to datasheet //100
	if (!(DS18B20_read_data_pin(sensor))) response = 1;    // if the pin is low i.e the presence pulse is detected
	else response = -1;
    //taskEXIT_CRITICAL();

	DS18B20_delay(sensor, 400); // 480 us delay totally.  //600

	return response;
}

void DS18B20_Write (DS18B20 *sensor, uint8_t data)
{
	DS18B20_set_pin_output(sensor);  // set as output

	for (int i=0; i<8; i++)
	{
		//taskENTER_CRITICAL();
		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1
			DS18B20_set_data_pin(sensor, false);  // pull the pin LOW
			DS18B20_delay(sensor, 5);  // wait for 1 us //10
			DS18B20_set_data_pin(sensor, true);
			DS18B20_delay(sensor, 60);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0
			DS18B20_set_data_pin(sensor, false);  // pull the pin LOW
			DS18B20_delay(sensor, 60);  // wait for 60 us
			DS18B20_set_data_pin(sensor, true);
		}
	    //taskEXIT_CRITICAL();
	}
}

uint8_t DS18B20_Read(DS18B20 *sensor)
{
	uint8_t value=0;
	DS18B20_set_pin_input(sensor);


	for (int i=0;i<8;i++)
	{
		//taskENTER_CRITICAL();
		DS18B20_set_pin_output(sensor);   // set as output
		DS18B20_set_data_pin(sensor, 0);  // pull the data pin LOW
		DS18B20_delay(sensor, 5);  // wait for 2 us
		DS18B20_set_data_pin(sensor, 1);
		DS18B20_set_pin_input(sensor);
		DS18B20_delay(sensor, 10); //11
		if (DS18B20_read_data_pin(sensor))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		//taskEXIT_CRITICAL();
		DS18B20_delay(sensor, 45);  // wait for 60 us //48
	}
	return value;
}


/*
Read the current temperature from the sensor.
This functions blocks for around 800ms as it waits for the conversion time!
@return Temperature in degrees Celsius
*/
float DS18B20_read_temp_celsius(DS18B20 *sensor) {
	float temperature = (float)DS18B20_read_temp(sensor)/16.0;
	return temperature;
}

/*
Read the current temperature from the sensor.
This functions blocks for around 800ms as it waits for the conversion time!
@return Temperature
*/
uint16_t DS18B20_read_temp(DS18B20 *sensor) {
	uint8_t presence = DS18B20_Start(sensor);
	if(presence != 1)
	{
		return -1;
	}
	DS18B20_Write(sensor, 0xCC);  // skip ROM
	DS18B20_Write(sensor, 0x44);  // convert t

	presence = DS18B20_Start(sensor);
	if(presence != 1)
	{
		return -1;
	}
	DS18B20_Write(sensor, 0xCC);  // skip ROM
	DS18B20_Write(sensor, 0xBE);  // Read Scratch-pad

	uint8_t temp_byte1 = DS18B20_Read(sensor);
	uint8_t temp_byte2 = DS18B20_Read(sensor);
	uint16_t TEMP = ((temp_byte2<<8))|temp_byte1;
	return TEMP;
}
