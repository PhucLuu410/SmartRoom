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
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include <LCD20x4.h>
#include <DHT.h>
#include <ShowScreen.h>
#include <FLash.h>
#include "rc522.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ADD_HUMI_DATA 0x0800FC00
#define ADD_TEMP_DATA 0x08010000
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t NhietDo;
uint16_t DoAm;
uint8_t  TempSet = 30, HumiSet = 60;
bool menu = 0 , tang = 0 , giam  = 0 , ok = 0 , i , ex;
char buffer1[2];
char buffer2[2];
DHT_data d;
int a;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t status;
uint8_t str[MAX_LEN]; // Max_LEN = 16
uint8_t sNum[5];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Normal(void) {
	HAL_GPIO_WritePin(BuzzerPort, BuzzerPin, 0);
	HAL_GPIO_WritePin(LedPort, LedPin, 0);
}

void Warning(void) {
	HAL_GPIO_TogglePin(BuzzerPort, BuzzerPin);
	HAL_GPIO_TogglePin(LedPort, LedPin);
}

void PressSound(void) {
	HAL_GPIO_TogglePin(BuzzerPort, BuzzerPin);
	HAL_GPIO_TogglePin(LedPort, LedPin);
	HAL_Delay(20);
	HAL_GPIO_TogglePin(BuzzerPort, BuzzerPin);
	HAL_GPIO_TogglePin(LedPort, LedPin);
}

void Mode(void) {
	if (menu == 0)
	{
		hienthi(NhietDo, DoAm);
		if (HAL_GPIO_ReadPin(GPIOA, Menu) == 0)
		{
			while (HAL_GPIO_ReadPin(GPIOA, Menu) == 0){}
			PressSound();
			LCD_Clear();
			menu = !menu;
		}
	}
	if (menu == 1)
	{
		LCD_SetCursor(0, 0);
		LCD_SendString("Chon gia tri cai dat");
		LCD_SetCursor(1, 4);
		LCD_SendString("Nhiet do");
		LCD_SetCursor(2, 4);
		LCD_SendString("Do am");


		LCD_SetCursor(i + 1, 0);
		LCD_SendString("->");
		if (HAL_GPIO_ReadPin(GPIOA, Menu) == 0)
		{
			while (HAL_GPIO_ReadPin(GPIOA, Menu) == 0) {}
			PressSound();
			LCD_Clear();
			menu = !menu;
		}
		if (HAL_GPIO_ReadPin(GPIOA, Tang) == 0)
		{
			LCD_SetCursor(1, 0);
			LCD_SendString("  ");
			LCD_SetCursor(2, 0);
			LCD_SendString("  ");
			i = !i ;
			LCD_SetCursor(i + 1, 0);
			LCD_SendString("->");
			PressSound();
		}
		if (HAL_GPIO_ReadPin(GPIOA, Giam) == 0)
		{
			LCD_SetCursor(1, 0);
			LCD_SendString("  ");
			LCD_SetCursor(2, 0);
			LCD_SendString("  ");
			i = !i ;
			LCD_SetCursor(i - 1, 0);
			LCD_SendString("->");
			PressSound();
		}
		if( HAL_GPIO_ReadPin(GPIOA, Ok) == 0 && i == 0 && ex == 0)
		{
			while(HAL_GPIO_ReadPin(GPIOA, Ok) == 0){}
			PressSound();
			ex = !ex;
			LCD_Clear();
			while(ex == 1)
			{
				LCD_SetCursor(0, 0);
				LCD_PrintInt(TempSet);
				LCD_SetCursor(1, 0);
				LCD_SendString("Press OK to setup !!");
				if (HAL_GPIO_ReadPin(GPIOA, Ok) == 0)
				{
					PressSound();
					LCD_Clear();
					ex = 0;
					menu = 0;
				}
				if (HAL_GPIO_ReadPin(GPIOA, Tang) == 0)
				{
					LCD_SetCursor(0,0);
					LCD_SendString("   ");
					FlashErase(ADD_TEMP_DATA);
					TempSet += 1;
					if(TempSet >= 80)
					{
						TempSet = 20;
					}
					FlashWrite(TempSet , ADD_TEMP_DATA);
				}
				if (HAL_GPIO_ReadPin(GPIOA, Giam) == 0)
				{
					LCD_SetCursor(0,0);
					LCD_SendString("   ");
					FlashErase(ADD_TEMP_DATA);
					TempSet -= 1;
					if(TempSet <= 20)
					{
						TempSet = 80;
					}
					FlashWrite(TempSet , ADD_TEMP_DATA);
				}
			}
		}
		if (HAL_GPIO_ReadPin(GPIOA, Ok) == 0 && i == 1 && ex == 0)
		{
			while (HAL_GPIO_ReadPin(GPIOA, Ok) == 0) {}
			PressSound();
			ex = !ex;
			LCD_Clear();
			while (ex == 1)
			{
				LCD_SetCursor(0, 0);
				LCD_PrintInt(HumiSet);
				LCD_SetCursor(1, 0);
				LCD_SendString("Press OK to setup !!");
				if (HAL_GPIO_ReadPin(GPIOA, Ok) == 0)
				{
					PressSound();
					LCD_Clear();
					ex = 0;
					menu = 0;
				}
				if (HAL_GPIO_ReadPin(GPIOA, Tang) == 0)
				{
					LCD_SetCursor(0,0);
					LCD_SendString("   ");
					FlashErase(ADD_HUMI_DATA);
					HumiSet += 1;
					if( HumiSet >= 70)
					{
						HumiSet = 20;
					}
					FlashWrite(HumiSet , ADD_HUMI_DATA);
				}
				if (HAL_GPIO_ReadPin(GPIOA, Giam) == 0)
				{
					LCD_SetCursor(0,0);
					LCD_SendString("   ");
					FlashErase(ADD_HUMI_DATA);
					HumiSet -= 1;
					if (HumiSet <= 20)
					{
						HumiSet = 70;
					}
					FlashWrite(HumiSet , ADD_HUMI_DATA);
				}
			}
		}
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
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  MFRC522_Init();
  LCD_Init();
  HAL_TIM_Base_Start(&htim4);
  DHT_sensor MyRoom = { GPIOB, GPIO_PIN_4, DHT22, GPIO_NOPULL };
  HumiSet = FlashRead (ADD_HUMI_DATA) ;
  TempSet = FlashRead (ADD_TEMP_DATA) ;
  __HAL_TIM_SET_COUNTER(&htim4 , 0);
  HAL_TIM_PWM_Start(&htim4 , TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		d = DHT_getData(&MyRoom);

		NhietDo = (uint16_t) d.temp;
		DoAm = (uint16_t) d.hum;

		if (d.temp > TempSet || d.hum > HumiSet) {
			Warning();
		} else {
			Normal();
		}
		Mode();
		sprintf(buffer1 , "%d" ,(uint16_t) d.temp);
		sprintf(buffer2 , "%d" ,(uint16_t) d.hum);
		if(__HAL_TIM_GET_COUNTER(&htim4) > 1000)
		{
			HAL_UART_Transmit(&huart1 , (uint8_t *)buffer1 , 2,1);
			HAL_UART_Transmit(&huart1 , (uint8_t *)buffer2 , 2,1);
			__HAL_TIM_SET_COUNTER(&htim4 , 0);
		}
		status = MFRC522_Request(PICC_REQIDL, str);
		status = MFRC522_Anticoll(str);
		memcpy(sNum, str, 5);
		if((sNum[0]==115) && (sNum[1]==238) && (sNum[2]==40) && (sNum[3]==168) && (sNum[4]==29) )
		{

			   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,1);
			   LCD_SetCursor(2, 0);
			   LCD_SendString("         ");
			   LCD_SetCursor(2, 0);
			   LCD_SendString("CUA MO");
			   HAL_Delay(1000);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,0);
			LCD_SetCursor(2,0);
			LCD_SendString("         ");
			LCD_SetCursor(2, 0);
			LCD_SendString("CUA DONG");
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_12
                          |GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA8 PA9
                           PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB12
                           PB4 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_12
                          |GPIO_PIN_4|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
	while (1) {
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
