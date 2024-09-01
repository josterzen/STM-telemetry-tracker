/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.

#include "dwt.h"  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "dma2d.h"
#include "dsihost.h"
#include "fatfs.h"
#include "i2c.h"
#include "ltdc.h"
#include "rtc.h"
#include "sdmmc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050.h"
#include "ds18b20.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "stm32f769i_discovery_lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int _write(int file, uint8_t* p, int len)
{
	if(HAL_UART_Transmit(&huart1, p, len, len) == HAL_OK )
	{
		return len;
	}
	return 0;
}

void GetTimestampFileString(char* timestampStr);
void UpdateBars(float value1, float value2, float value3);
void DrawBar(float value, uint16_t originX, uint16_t originY);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DISPLAY_WIDTH  800   // Width of the display
#define DISPLAY_HEIGHT 472   // Height of the display
#define BAR_WIDTH      720   // Width of the bar
#define BAR_HEIGHT      20   // Height of the bar
#define BAR_SPACING     40   // Spacing between bars
#define BAR_ORIGIN_Y    50   // Y coordinate of the first bar's origin
#define MARKER_WIDTH     2   // Width of the middle marker

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
DS18B20 temp_sensor;
FRESULT res; /* FatFs function common result code */
uint32_t byteswritten, bytesread; /* File write/read counts */
uint8_t wtext[WTEXT_SIZE]; /* File write buffer */
uint8_t wbuff1[BUFFER_SIZE]; /* File write buffer */
uint8_t wbuff2[BUFFER_SIZE]; /* File write buffer */
uint8_t *active_buffer = wbuff1;
uint8_t *write_buffer = wbuff2;
int activeBufferPos = 0;  // Current position in the active buffer
int writeBufferLen = 0;  // Length of the write buffer
uint8_t rtext[_MAX_SS];/* File read buffer */
char timestamp[24];
bool writeYes = false;
float engineTemperature;
bool readTemp = true;
bool printToWB = false;
uint32_t milliseconds = 0;
uint16_t pulseCount = 0;
uint16_t engineRpm = 0;
RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef sDate = {0};
bool swapBuff = false;
uint8_t pcnt = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SDMMC2_SD_Init();
  MX_TIM6_Init();
  MX_FATFS_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_RTC_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_DSIHOST_DSI_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_Initialization();
  DS18B20_init(&temp_sensor, &htim6, GPIOF, GPIO_PIN_7);
  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
  BSP_LCD_SelectLayer(0);
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  GetTimestampFileString(timestamp);
  memset(wbuff1, 0, BUFFER_SIZE);
  memset(wbuff2, 0, BUFFER_SIZE);
  if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK)
  	{
  		Error_Handler();
  	}
  	else
  	{
  		//Open file for writing (Create)
  		if(f_open(&SDFile, timestamp, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
  		{
  			Error_Handler();
  		}
  		else
  		{
  			//f_close(&SDFile);
  		}

  	}

  	if(readTemp)
  	{
  		engineTemperature = DS18B20_read_temp_celsius(&temp_sensor);
  		readTemp = false;
  	}
    if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
    {
		Error_Handler();
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		if(readTemp)
		{
			engineRpm = pulseCount*60;
			pulseCount = 0;
			engineTemperature = DS18B20_read_temp_celsius(&temp_sensor);
			readTemp = false;
		}
		if(MPU6050_DataReady() == 1)
		{
			  MPU6050_ProcessData(&MPU6050);
		}
		if(printToWB)
		{
			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
		    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

			milliseconds = ((hrtc.Init.SynchPrediv - sTime.SubSeconds) * 1000) / (hrtc.Init.SynchPrediv + 1);
			/*int wtext_len = custom_format(wtext, WTEXT_SIZE,
			                             &sTime.Hours, &sTime.Minutes, &sTime.Seconds, milliseconds,
			                             MPU6050.acc_x, MPU6050.acc_y, MPU6050.acc_z,
			                             MPU6050.temperature, MPU6050.gyro_x, MPU6050.gyro_y, MPU6050.gyro_z,
			                             engineTemperature);*/
			/*int wtext_len = snprintf((char*) &wtext, WTEXT_SIZE, "%02u\t%02u\t%02u\t%03lu\t%.8f\t%.8f\t%.8f\t%.4f\t%.6f\t%.6f\t%.6f\t%.4f\t%u\n",
			 sTime.Hours, sTime.Minutes, sTime.Seconds, milliseconds,
			 MPU6050.acc_x, MPU6050.acc_y, MPU6050.acc_z, MPU6050.temperature, MPU6050.gyro_x, MPU6050.gyro_y, MPU6050.gyro_z,
			 engineTemperature, engineRpm);*/
			int wtext_len = snprintf((char*) &wtext, WTEXT_SIZE, "%02u\t%02u\t%02u\t%03lu\t%d\t%d\t%d\t%.4f\t%d\t%d\t%d\t%.4f\t%u\n",
						 sTime.Hours, sTime.Minutes, sTime.Seconds, milliseconds,
						 MPU6050.acc_x_raw, MPU6050.acc_y_raw, MPU6050.acc_z_raw, MPU6050.temperature_raw, MPU6050.gyro_x_raw, MPU6050.gyro_y_raw, MPU6050.gyro_z_raw,
						 engineTemperature, engineRpm);


			if (wtext_len < 0) {
				// Handle buffer overflow error
				Error_Handler();
			}

			if ((activeBufferPos + wtext_len + WTEXT_SIZE) >= BUFFER_SIZE) {
				//swap active buffers, set wrieYes to true
				/*if (writeYes) {
					Error_Handler();
				} else {

					uint8_t *tmpBuff = active_buffer;
					__disable_irq();
					active_buffer = write_buffer;
					write_buffer = tmpBuff;

					writeBufferLen = activeBufferPos;
					activeBufferPos = 0;
					writeYes = true;
					__enable_irq();
				}*/
				swapBuff = true;
			}
			memcpy(&active_buffer[activeBufferPos], wtext, wtext_len);

			activeBufferPos += wtext_len;
			memset(wtext, 0, WTEXT_SIZE);
			printToWB = false;
		}

		if (writeYes) {
			// Open the file for appending and writing
				// Write the buffer to the file
				res = f_write(&SDFile, write_buffer, writeBufferLen,
						(UINT*) &byteswritten);
				if (res != FR_OK || byteswritten != writeBufferLen) {
					// Handle write error
					Error_Handler();
				}

				// Clear the buffer after writing
				memset(write_buffer, 0, BUFFER_SIZE);

				// Close the file
				f_sync(&SDFile);

				// Reset the writeYes flag
				writeYes = false;
		}
		if (swapBuff) {
			if (writeYes) {
				Error_Handler();
			} else {

				uint8_t *tmpBuff = active_buffer;
				__disable_irq();
				active_buffer = write_buffer;
				write_buffer = tmpBuff;

				writeBufferLen = activeBufferPos;
				activeBufferPos = 0;
				writeYes = true;
				__enable_irq();
			}
		}
		if(pcnt%16 == 0)
		{
			UpdateBars(MPU6050.gyro_y/180, MPU6050.acc_x/2, engineRpm/7000);
			pcnt = pcnt%16;
		}
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
void GetTimestampFileString(char* timestampStr)
{
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    snprintf(timestampStr, 24, "%04d-%02d-%02d %02d%02d%02d.txt",
             2000 + sDate.Year, sDate.Month, sDate.Date,
             sTime.Hours, sTime.Minutes, sTime.Seconds);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_8)
    {
        pulseCount++;
    }
}

void DrawBar(float value, uint16_t originX, uint16_t originY) {
    uint16_t markerX = originX + BAR_WIDTH / 2;
    uint16_t fillLength = (uint16_t)(fabsf(value) * (BAR_WIDTH / 2));
    uint16_t fillStartX;

    // Draw the empty bar outline
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DrawRect(originX, originY, BAR_WIDTH, BAR_HEIGHT);

    // Draw the middle marker
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
    BSP_LCD_FillRect(markerX - MARKER_WIDTH / 2, originY, MARKER_WIDTH, BAR_HEIGHT);

    // Draw the fill based on the sign of the value
    if (value > 0) {
        fillStartX = markerX;
        BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
        BSP_LCD_FillRect(fillStartX, originY, fillLength, BAR_HEIGHT);
    } else if (value < 0) {
        fillStartX = markerX - fillLength;
        BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
        BSP_LCD_FillRect(fillStartX, originY, fillLength, BAR_HEIGHT);
    }
}

void UpdateBars(float value1, float value2, float value3) {
    // Calculate the horizontal origin to center the bars on the display
    uint16_t originX = (DISPLAY_WIDTH - BAR_WIDTH) / 2;

    // Clear the area where bars are drawn
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_FillRect(originX, BAR_ORIGIN_Y, BAR_WIDTH, BAR_HEIGHT);
    BSP_LCD_FillRect(originX, BAR_ORIGIN_Y + BAR_SPACING, BAR_WIDTH, BAR_HEIGHT);
    BSP_LCD_FillRect(originX, BAR_ORIGIN_Y + 2 * BAR_SPACING, BAR_WIDTH, BAR_HEIGHT);

    // Draw the updated bars
    DrawBar(value1, originX, BAR_ORIGIN_Y);
    DrawBar(value2, originX, BAR_ORIGIN_Y + BAR_SPACING);
    DrawBar(value3, originX, BAR_ORIGIN_Y + 2 * BAR_SPACING);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
