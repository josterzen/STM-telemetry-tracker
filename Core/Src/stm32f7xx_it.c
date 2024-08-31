/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050.h"
#include "ds18b20.h"
#include "fatfs.h"
#include "rtc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int custom_format(char *buffer, size_t buffer_size,
                  uint32_t hours, uint32_t minutes, uint32_t seconds, uint32_t milliseconds,
                  float acc_x, float acc_y, float acc_z,
                  float temperature, float gyro_x, float gyro_y, float gyro_z,
                  float engineTemperature);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t tim3cnt_iter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_sdmmc2_rx;
extern DMA_HandleTypeDef hdma_sdmmc2_tx;
extern SD_HandleTypeDef hsd2;
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */
extern DS18B20 temp_sensor;
extern FRESULT res;
extern uint32_t byteswritten, bytesread;
extern uint8_t wtext[];
extern uint8_t rtext[_MAX_SS];
extern bool writeYes;
extern char *active_buffer;
extern char *write_buffer;
extern int activeBufferPos;
extern int writeBufferLen;
extern float engineTemperature;

extern bool readTemp;
extern bool printToWB;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles RCC global interrupt.
  */
void RCC_IRQHandler(void)
{
  /* USER CODE BEGIN RCC_IRQn 0 */

  /* USER CODE END RCC_IRQn 0 */
  /* USER CODE BEGIN RCC_IRQn 1 */

  /* USER CODE END RCC_IRQn 1 */
}

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(B_USER_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
  if(tim3cnt_iter%100 == 0)
  {
	  tim3cnt_iter = tim3cnt_iter%100;
	  readTemp = true;
  }
  printToWB = true;
  tim3cnt_iter +=1;
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_sdmmc2_rx);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream5 global interrupt.
  */
void DMA2_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream5_IRQn 0 */

  /* USER CODE END DMA2_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_sdmmc2_tx);
  /* USER CODE BEGIN DMA2_Stream5_IRQn 1 */

  /* USER CODE END DMA2_Stream5_IRQn 1 */
}

/**
  * @brief This function handles SDMMC2 global interrupt.
  */
void SDMMC2_IRQHandler(void)
{
  /* USER CODE BEGIN SDMMC2_IRQn 0 */

  /* USER CODE END SDMMC2_IRQn 0 */
  HAL_SD_IRQHandler(&hsd2);
  /* USER CODE BEGIN SDMMC2_IRQn 1 */

  /* USER CODE END SDMMC2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
// Helper function to convert an integer to a string
int int_to_str(char *buffer, uint32_t value) {
    int length = 0;
    char temp[10]; // Temporary buffer for reversing

    if (value == 0) {
        buffer[length++] = '0';
    } else {
        while (value != 0) {
            temp[length++] = (value % 10) + '0';
            value /= 10;
        }

        // Reverse the string
        for (int i = 0; i < length; ++i) {
            buffer[i] = temp[length - i - 1];
        }
    }

    buffer[length] = '\0';
    return length;
}

// Helper function to convert a float to a string with specific precision
int float_to_str(char *buffer, float value, int precision) {
    int length = 0;

    if (value < 0) {
        buffer[length++] = '-';
        value = -value;
    }

    // Handle the integer part
    uint32_t int_part = (uint32_t)value;
    length += int_to_str(&buffer[length], int_part);

    buffer[length++] = '.';

    // Handle the fractional part
    float frac_part = value - int_part;
    for (int i = 0; i < precision; ++i) {
        frac_part *= 10;
        buffer[length++] = ((int)frac_part % 10) + '0';
    }

    buffer[length] = '\0';
    return length;
}

// Main function to format data
int custom_format(char *buffer, size_t buffer_size,
                  uint32_t hours, uint32_t minutes, uint32_t seconds, uint32_t milliseconds,
                  float acc_x, float acc_y, float acc_z,
                  float temperature, float gyro_x, float gyro_y, float gyro_z,
                  float engineTemperature) {
    int offset = 0;

    // Format integers (hours, minutes, seconds, milliseconds)
    offset += int_to_str(buffer + offset, hours);
    buffer[offset++] = '\t';
    offset += int_to_str(buffer + offset, minutes);
    buffer[offset++] = '\t';
    offset += int_to_str(buffer + offset, seconds);
    buffer[offset++] = '\t';
    offset += int_to_str(buffer + offset, milliseconds);
    buffer[offset++] = '\t';

    // Format floats (accelerometer, temperature, gyroscope, engine temperature)
    offset += float_to_str(buffer + offset, acc_x, 8);
    buffer[offset++] = '\t';
    offset += float_to_str(buffer + offset, acc_y, 8);
    buffer[offset++] = '\t';
    offset += float_to_str(buffer + offset, acc_z, 8);
    buffer[offset++] = '\t';
    offset += float_to_str(buffer + offset, temperature, 4);
    buffer[offset++] = '\t';
    offset += float_to_str(buffer + offset, gyro_x, 6);
    buffer[offset++] = '\t';
    offset += float_to_str(buffer + offset, gyro_y, 6);
    buffer[offset++] = '\t';
    offset += float_to_str(buffer + offset, gyro_z, 6);
    buffer[offset++] = '\t';
    offset += float_to_str(buffer + offset, engineTemperature, 4);
    buffer[offset++] = '\n';

    if (offset >= buffer_size) {
        return -1;  // Buffer overflow
    }

    return offset;
}
/* USER CODE END 1 */
