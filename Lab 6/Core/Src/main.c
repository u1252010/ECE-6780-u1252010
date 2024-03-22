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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

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
  /* USER CODE BEGIN 2 */

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_ADC1_CLK_ENABLE();
	__HAL_RCC_DAC1_CLK_ENABLE();
	
	/*
	PC6 - RED
	PC7 - BLUE
	PC8 - ORANGE
	PC9 - GREEN
	*/
	
	GPIO_InitTypeDef LEDs = {
		GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
		GPIO_MODE_OUTPUT_PP,
		GPIO_SPEED_FREQ_LOW,
		GPIO_NOPULL
	};
	
	GPIO_InitTypeDef ADC_In = {
		GPIO_PIN_0,
		GPIO_MODE_ANALOG,
		GPIO_SPEED_FREQ_LOW,
		GPIO_NOPULL
	};
	
	GPIO_InitTypeDef DAC_Out = {
		GPIO_PIN_4,
		GPIO_MODE_ANALOG,
		GPIO_SPEED_FREQ_LOW,
		GPIO_NOPULL
	};
	
	HAL_GPIO_Init(GPIOC,&LEDs);
	HAL_GPIO_Init(GPIOC,&ADC_In);
	HAL_GPIO_Init(GPIOA,&DAC_Out);
	
	/* ADC Setup */
	ADC1->CFGR1 |= (1<<13); // Continuous conversion mode
	ADC1->CFGR1 |= (1<<4); // 8-bit resolution
	ADC1->CFGR1 &= ~(0x3<<10); // Disable hardware triggers
	
	ADC1->CHSELR |= (1<<10); // Select channel 10
	
	// Perform ADC self-calibration
	ADC1->CR |= (1<<31);
	
	while (ADC1->CR & (1<<31)) {
		// wait for calibration to finish.
	}
	
	ADC1->CR |= ADC_CR_ADEN; // Enable ADC
	
	while (!(ADC1->ISR & (1<<0))) {
		// wait for ADC to be ready
	}
	
	ADC1->CR |= ADC_CR_ADSTART; // Begin conversion

	unsigned short int data; // ADC Data Variable
	
	/* DAC Setup */
	
	// PA4 - DAC_OUT1
	DAC1->CR |= (0x7<<3); // Set software trigger
	DAC1->CR |= (1<<0); // Enable channel 1
	
	// Sine Wave: 8-bit, 32 samples/cycle
	const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
		232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};
	
	// Triangle Wave: 8-bit, 32 samples/cycle
	const uint8_t triangle_table[32] = {0,15,31,47,63,79,95,111,127,142,158,174,
		190,206,222,238,254,238,222,206,190,174,158,142,127,111,95,79,63,47,31,15};
	
	// Sawtooth Wave: 8-bit, 32 samples/cycle
	const uint8_t sawtooth_table[32] = {0,7,15,23,31,39,47,55,63,71,79,87,95,103,
		111,119,127,134,142,150,158,166,174,182,190,198,206,214,222,230,238,246};
	
	// Square Wave: 8-bit, 32 samples/cycle
	const uint8_t square_table[32] = {254,254,254,254,254,254,254,254,254,254,
		254,254,254,254,254,254,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	
	unsigned short int index = 0;
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		data = ADC1->DR;
		
		if (data > 0) {
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9, GPIO_PIN_RESET);
		}
		if (data > 64) {
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7, GPIO_PIN_RESET);
		}
		if (data > 128) {
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8, GPIO_PIN_RESET);
		}
		if (data > 192) {
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6, GPIO_PIN_RESET);
		}
		
		if (index >= 32) {
			index = 0;
		}
		
		DAC1->DHR8R1 = sawtooth_table[index];
		
		index++;
		
		HAL_Delay(1);
		
    /* USER CODE BEGIN 3 */
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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

