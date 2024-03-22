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

	void I2CWho(void) {
		// Set up transaction parameters
		I2C2->CR2 &= ~((0x7F << 16) | (0x3FF<<0));
		I2C2->CR2 |= (0x69<<1); // slave address
		I2C2->CR2 |= (1<<16);  // # bytes to transmit
		I2C2->CR2 &= ~(1<<10); // indicate write operation
		I2C2->CR2 |= (1<<13); // set start bit
		
		/*
		PC6 - RED
		PC7 - BLUE
		PC8 - ORANGE
		PC9 - GREEN
		*/
		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
		while (!(I2C2->ISR & I2C_ISR_NACKF) & !(I2C2->ISR & I2C_ISR_TXIS)) {
			// waiting

		}
		
		if (I2C2->ISR & I2C_ISR_NACKF) {
			// NACKF set
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6, GPIO_PIN_SET);
			return;
		}
		if (I2C2->ISR & I2C_ISR_TXIS) {
			// TXIS set
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8, GPIO_PIN_SET);
		}
		
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_7);
		I2C2->TXDR |= (0xF); 
		
		while (!(I2C2->ISR & (1<<6))) {
			// waiting for TC
		}
		
		I2C2->CR2 |= (0x69<<1); // slave address
		I2C2->CR2 |= (1<<16);  // # bytes to transmit
		I2C2->CR2 |= (1<<10); // indicate read operation
		I2C2->CR2 |= (1<<13); // set start bit
		
		while (!((I2C2->ISR & (1<<4)) || (I2C2->ISR & (1<<2)))) {
			// waiting
		}
		
		if (I2C2->ISR & (1<<4)) {
			// NACKF set
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6, GPIO_PIN_SET);
			return;
		}
		
		while (!(I2C2->ISR & (1<<6))) {
			// waiting for TC
		}
		
		if (I2C2->RXDR & 0xD3) {
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9, GPIO_PIN_SET);
		}
		
		I2C2->CR2 |= (1<<14);
		
	}
	
	void I2CInit(char address, char mode, unsigned char bytes) {
		I2C2->CR2 &= ~((0x7F << 16) | (0x3FF<<0));
		I2C2->CR2 |= (address<<1); // slave address
		I2C2->CR2 |= (bytes<<16);  // # bytes to transmit
		if (mode == 'w' || mode == 'W') {
			I2C2->CR2 &= ~(1<<10); // indicate write operation
		} else if (mode == 'r' || mode == 'R') {
			I2C2->CR2 |= (1<<10); // indicate read operation
		}
		
	}
	
	void I2CStart(void) {
		I2C2->CR2 |= (1<<13); // set start bit
	}
	
	void I2CWaitWrite(void) {
		while (!(I2C2->ISR & I2C_ISR_NACKF) & !(I2C2->ISR & I2C_ISR_TXIS)) {
			// waiting
		}
		
		if (I2C2->ISR & I2C_ISR_NACKF) {
			// NACKF set
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6, GPIO_PIN_SET);
			return;
		}
	}
	
	void I2CWaitRead(void) {
		while (!(I2C2->ISR & I2C_ISR_NACKF) & !(I2C2->ISR & I2C_ISR_RXNE)) {
			// waiting
		}
		
		if (I2C2->ISR & I2C_ISR_NACKF) {
			// NACKF set
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6, GPIO_PIN_SET);
			return;
		}
	}
	
	void I2CWrite(char msg) {
		
		I2C2->TXDR = msg;
		
	}
	
	char I2CRead(void) {
		
		char msg = I2C2->RXDR; // read msg
		
		return msg; // return msg
		
	}
	
	void I2CWaitTC(void) {
		while (!(I2C2->ISR & (1<<6))) {
			// waiting for TC
		}
	}
	
	void I2CStop(void) {
		
		I2C2->CR2 |= (1<<14); // set stop bit 
	}
	
	void I2CWriteRegister(char address, char reg, char value) {
		I2CInit(address,'w',2);
		I2CStart();
		I2CWaitWrite();
		I2CWrite(reg);
		I2CWaitWrite();
		I2CWrite(value);
		I2CWaitTC();
		I2CStop();
	}
	
	char I2CReadRegister(char address, char reg) {
		I2CInit(address,'w',1);
		I2CStart();
		I2CWaitWrite();
		I2CWrite(reg);
		I2CWaitTC();
		I2CInit(address,'r',1);
		I2CStart();
		I2CWaitRead();
		char result = I2CRead();
		I2CWaitTC();
		I2CStop();
		
		return result; 
	}
	
	void tilt(void) {
		char xl = I2CReadRegister(0x69,0x28);
		HAL_Delay(10);
		char xh = I2CReadRegister(0x69,0x29);
		HAL_Delay(10);
		char yl = I2CReadRegister(0x69,0x2A);
		HAL_Delay(10);
		char yh = I2CReadRegister(0x69,0x2B);
		
		short int x = (xh<<8) | xl;
		if (x > 600) {
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
		} else if (x < -600) {
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
		} else{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
		}
		
		short int y = (yh<<8) | yl;
		if (y > 600) {
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
		} else if (y < -600) {
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
		}
}
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

	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_I2C2_CLK_ENABLE();
	
	GPIO_InitTypeDef com = {
		GPIO_PIN_11 | GPIO_PIN_13,
		GPIO_MODE_AF_OD,
		GPIO_SPEED_FREQ_LOW,
		GPIO_NOPULL
	};
	
	GPIO_InitTypeDef out1 = {
		GPIO_PIN_14,
		GPIO_MODE_OUTPUT_PP,
		GPIO_SPEED_FREQ_LOW,
		GPIO_NOPULL
	};
	
	GPIO_InitTypeDef out2 = {
		GPIO_PIN_0,
		GPIO_MODE_OUTPUT_PP,
		GPIO_SPEED_FREQ_LOW,
		GPIO_NOPULL
	};
	
	GPIO_InitTypeDef in = {
		GPIO_PIN_15,
		GPIO_MODE_INPUT,
		GPIO_SPEED_FREQ_LOW,
		GPIO_NOPULL
	};
	
	
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
	
	HAL_GPIO_Init(GPIOB,&com);
	HAL_GPIO_Init(GPIOB,&out1);
	HAL_GPIO_Init(GPIOC,&out2);
	HAL_GPIO_Init(GPIOB,&in);
	HAL_GPIO_Init(GPIOC,&LEDs);
	
	GPIOB->AFR[1] |= (1<<12);
	GPIOB->AFR[1] |= (1<<20);
	GPIOB->AFR[1] |= (1<<22);
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0, GPIO_PIN_SET);
	
	// Set up timing
	I2C2->TIMINGR |= (1<<28);
	I2C2->TIMINGR |= (0x13<<0);
	I2C2->TIMINGR |= (0xF<<8);
	I2C2->TIMINGR |= (0x2<<16);
	I2C2->TIMINGR |= (0x4<<20);
	
	// Enable I2C2
	I2C2->CR1 |= (1<<0);
	
	// Set up the gyro
	//I2CWho();
	
	I2CWriteRegister(0x69,0x20,0xB);
	
	HAL_Delay(100);
	
	char result = I2CReadRegister(0x69,0x20);
	
	if (result == 0xB) {
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9, GPIO_PIN_SET);
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // I2CWho();

	while (1)
  {
    /* USER CODE END WHILE */
		HAL_Delay(100);
		tilt();
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

