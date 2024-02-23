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
extern volatile char newDataVal;
extern volatile int newDataFlag;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

void transmitChar(char input) {
	while (!(USART3->ISR & (1<<7))) {
		// Waiting
	}
		
	USART3->TDR = input;
}

void transmitStr(char input[]) {
	int i = 0;
	while (1) {
		if (input[i] == 0) {
			break;
		}
		transmitChar(input[i]);
		i += 1;
	}
}

void receive1(void) {
	while (!(USART3->ISR & (1<<5))) {
		// Waiting
	}
	
	/*
	PC6 - RED
	PC7 - BLUE
	PC8 - ORANGE
	PC9 - GREEN
	*/
	
	char input = USART3->RDR;
	
	if (input == 'R' || input == 'r') {
		GPIOC->ODR ^= GPIO_ODR_6;
	} else if (input == 'B' || input == 'b') {
		GPIOC->ODR ^= GPIO_ODR_7;
	} else if (input == 'O' || input == 'o') {
		GPIOC->ODR ^= GPIO_ODR_8;
	} else if (input == 'G' || input == 'g') {
		GPIOC->ODR ^= GPIO_ODR_9;
	} else {
		transmitStr("Wrong LED code, try r,b,o,g\r");
	}
	
}

void switchLED(int pin, int mode) {
	if (pin == 6) {
		if (mode == '0') {
			GPIOC->ODR &= ~(GPIO_ODR_6);
		} else if (mode == '1') {
			GPIOC->ODR |= GPIO_ODR_6;
		} else if (mode == '2') {
			GPIOC->ODR ^= GPIO_ODR_6;
		} else {
			transmitStr("Wrong option code, try 0,1,2\r");
		}
	} else if (pin == 7) {
		if (mode == '0') {
			GPIOC->ODR &= ~(GPIO_ODR_7);
		} else if (mode == '1') {
			GPIOC->ODR |= GPIO_ODR_7;
		} else if (mode == '2'){ 
			GPIOC->ODR ^= GPIO_ODR_7;
		} else {
			transmitStr("Wrong option code, try 0,1,2\r");
		}
	} else if (pin == 8) {
		if (mode == '0') {
			GPIOC->ODR &= ~(GPIO_ODR_8);
		} else if (mode == '1') {
			GPIOC->ODR |= GPIO_ODR_8;
		} else if (mode == '2') {
			GPIOC->ODR ^= GPIO_ODR_8;
		} else {
			transmitStr("Wrong option code, try 0,1,2\r");
		}
	} else if (pin == 9) {
		if (mode == '0') {
			GPIOC->ODR &= ~(GPIO_ODR_9);
		} else if (mode == '1') {
			GPIOC->ODR |= GPIO_ODR_9;
		} else if (mode == '2') {
			GPIOC->ODR ^= GPIO_ODR_9;
		} else {
			transmitStr("Wrong option code, try 0,1,2\r");
		}
	}
}

void receive2(void) {
	transmitStr("CMD?");
	
	while (!newDataFlag) {
		// Waiting
	}

	char command1 = newDataVal;
	transmitStr(&command1);
	newDataFlag = 0;
	
	while (!newDataFlag) {
		// Waiting
	}
	
	char command2 = newDataVal;
	transmitStr(&command2);
	newDataFlag = 0;
	
	/*
	PC6 - RED
	PC7 - BLUE
	PC8 - ORANGE
	PC9 - GREEN
	*/
	
	if (command1 == 'R' || command1 == 'r') {
		switchLED(6,command2);
	} else if (command1 == 'B' || command1 == 'b') {
		switchLED(7,command2);
	} else if (command1 == 'O' || command1 == 'o') {
		switchLED(8,command2);
	} else if (command1 == 'G' || command1 == 'g') {
		switchLED(9,command2);
	} else {
		transmitStr("Wrong LED code, try r,b,o,g\r");
	}
	
}

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
	
	/*
	Using USART3
	USART_TX - PC4
	USART_RX - PC5
	Alternate function mode: AF1
	*/
	
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_USART3_CLK_ENABLE();
	
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
	
	GPIO_InitTypeDef com = {
		GPIO_PIN_4 | GPIO_PIN_5,
			GPIO_MODE_AF_PP,
		GPIO_SPEED_FREQ_LOW,
		GPIO_NOPULL
	};
	
	HAL_GPIO_Init(GPIOC,&com);
	HAL_GPIO_Init(GPIOC,&LEDs);
	
	GPIOC->AFR[0] |= (1<<16);
	GPIOC->AFR[0] |= (1<<20);
	
	int baud = 115200;
	
	int USART_div = HAL_RCC_GetHCLKFreq() / baud;
	
	USART3->BRR = USART_div;
	
	USART3->CR1 |= USART_CR1_TE;
	USART3->CR1 |= USART_CR1_RE;
	USART3->CR1 |= USART_CR1_RXNEIE;
	
	
	USART3->CR1 |= USART_CR1_UE;
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn,1);
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
	//char message[] = "Hello Putty!\r";
	while (1)
  {
    /* USER CODE END WHILE */
		
		// transmitChar('c');
		//transmitStr(message);
		//HAL_Delay(500);
		
		receive2();
		
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

