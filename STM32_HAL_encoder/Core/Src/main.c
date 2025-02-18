/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "string.h"
#include "MT6701.h"
#include "AS5600.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SAMP 1

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t dma_end_flag = 0;
uint32_t AD_DMA[SAMP]={0};
float adc1_angle[SAMP]={0};
int16_t angle_int1, angle_int2, angle_int3;
float angle1, angle2, angle3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f) {
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		dma_end_flag = 1;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	char txbuf[50];
	int len;
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start_IT(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_Delay(10);
		
		/***********************hello world test****************/
		/*
		memset(txbuf,0,sizeof(txbuf));
		len = sprintf(txbuf,"Hello world!\n");
		if(len > 0)
			HAL_UART_Transmit_IT(&huart1,(uint8_t*)txbuf,len);
		*/
		
		
		/**********************MT6701 Analog********************/
		/*
		HAL_ADC_Start_DMA(&hadc1, AD_DMA, SAMP);
		HAL_Delay(100);
		if(dma_end_flag == 1)
		{
			dma_end_flag = 0;
			for(uint8_t i=0;i<SAMP;i++)
			{
				adc1_angle[i] = (float)AD_DMA[i]* 3.3 / 16384 * 360;
				//printf("adc_angle%d = %.03f\r\n",i,adc1_angle[i]);
				printf("adc_angle=%.03f\n",adc1_angle[i]);
			}
		}
		*/
		
		/**********************AS5600 IIC********************/
		
	  i2c_as5600_get_angle_with_mux(&hi2c1, 2, &angle_int3, &angle3);
		

    /**********************TCA9548A test********************/
    // tca9548a_set_channel(&hi2c1, 0);
    // for (uint8_t addr = 0; addr < 128; addr++) {
    //   if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10) == HAL_OK) {
    //       printf("Device found at 0x%02X\n", addr);
    //   }
    // } 

    // if (HAL_I2C_IsDeviceReady(&hi2c1, TCA9548A_SLAVE_ADDR << 1, 1, 10) == HAL_OK) {
    //     printf("TCA9548A detected at 0x%02X\n", TCA9548A_SLAVE_ADDR);
    // } else {
    //     printf("TCA9548A not detected\n");
    // }
		
		/**********************MT6701 IIC********************/
		
    // 使用 I2C1
    // i2c_mt6701_get_angle(&hi2c1, &angle_int1, &angle1);
    //printf("I2C1 angle=%.03f\n", angle1);

    // 使用 TCA9548A 的 I2C1，选择通道 0
    i2c_mt6701_get_angle_with_mux(&hi2c1, 0, &angle_int1, &angle1);

    // 使用 TCA9548A 的 I2C1，选择通道 1
    i2c_mt6701_get_angle_with_mux(&hi2c1, 1, &angle_int2, &angle2);

    // 使用 I2C2
    // i2c_mt6701_get_angle(&hi2c2, &angle_int2, &angle2);
    //printf("I2C2 angle=%.03f\n", angle2);
		
		// printf("%.3f, %.3f\r\n", angle1, angle2);
    printf("angle1=%.3f, angle2=%.3f, angle3=%.3f\r\n", angle1, angle2, angle3);
		
		HAL_Delay(30);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
