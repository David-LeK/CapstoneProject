/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <math.h>
#include <stdio.h>
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;

/* USER CODE BEGIN PV */
uint8_t MSG[50] = {'\0'};
uint8_t Received_MSG[512] = {'\0'};
uint8_t line[50];
int encoder_count = 0;
unsigned long prevT = 0;
float eprev = 0;
float eintegral = 0;
float kp, ki, kd = 0;
int dir = 0;
char old_mode[2] = "x1";
char mode[2] = "x1";
int pos_target = 0;
int spd_target = 0;
int rate = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
//Motor driver PIN must be on the same PORT type
void setMotor(int dir, GPIO_TypeDef *GPIO_PORT, uint32_t GPIO_PWM_VAL, TIM_HandleTypeDef *htim, uint16_t GPIO_INT1_PIN, uint16_t GPIO_INT2_PIN)
{
	//htim->Instance->ARR = 65535;
	htim->Instance->CCR1 = GPIO_PWM_VAL;
	if (dir == 1)
	{
		HAL_GPIO_WritePin(GPIO_PORT, GPIO_INT1_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIO_PORT, GPIO_INT2_PIN, GPIO_PIN_RESET);
	}
	else if (dir == -1)
	{
		HAL_GPIO_WritePin(GPIO_PORT, GPIO_INT1_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIO_PORT, GPIO_INT2_PIN, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIO_PORT, GPIO_INT1_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIO_PORT, GPIO_INT2_PIN, GPIO_PIN_RESET);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // Callback function code goes here
	encoder_count = __HAL_TIM_GET_COUNTER(&htim3);
	//sprintf((char*) MSG, "%cpos=%u\r\n%c", 17, encoder_count, 19); //flow control
	sprintf((char*) MSG, "pos=%u\r\n", encoder_count);
	HAL_UART_Transmit_DMA(&huart4, MSG, sizeof(MSG));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// Process received data here
	    int line_pos = 0;
	    for (int i = 0; i < 512; i++)
	    {
	      if (Received_MSG[i] == '\r')
	      {
	        // Carriage return found, check for line feed
	        if (Received_MSG[i+1] == '\n')
	        {
	          // End of line found
	          line[line_pos] = '\0'; // Add null terminator to line buffer
	          break;
	        }
	      }
	      else if (Received_MSG[i] == '\n')
	      {
	        // Line feed found, check for carriage return
	        if (Received_MSG[i-1] == '\r')
	        {
	          // End of line found
	          line[line_pos] = '\0'; // Add null terminator to line buffer
	          break;
	        }
	      }
	      else
	      {
	        line[line_pos] = Received_MSG[i];
	        line_pos++;
	      }
	    }
	    sscanf((char *)line, "kp=%f", &kp);
	    sscanf((char *)line, "ki=%f", &ki);
	    sscanf((char *)line, "kd=%f", &kd);
	    sscanf((char *)line, "dir=%d", &dir);
	    sscanf((char *)line, "mode=%s", mode);
	    sscanf((char *)line, "target_pid=%d", &pos_target);
	    sscanf((char *)line, "target_speed=%d", &spd_target);
	    memset(line, '\0', sizeof(line));
	HAL_UART_Receive_DMA(&huart4, Received_MSG, sizeof(Received_MSG));
}

void setmode(char *mode)
{
	if (strcmp(mode, "x1") == 0)
	{
		htim3.Instance->SMCR &= ~TIM_SMCR_SMS; // Clear EncoderMode field
		htim3.Instance->SMCR |= TIM_ENCODERMODE_TI1 << TIM_SMCR_SMS_Pos; // Set EncoderMode field to TIM_ENCODERMODE_TI1
		htim3.Instance->CCER &= ~TIM_CCER_CC1P; // Clear IC1Polarity field
		htim3.Instance->CCER |= TIM_ICPOLARITY_RISING << TIM_CCER_CC1P_Pos; // Set IC1Polarity field to TIM_ICPOLARITY_RISING
	}
	else if (strcmp(mode, "x2") == 0)
	{
		htim3.Instance->SMCR &= ~TIM_SMCR_SMS; // Clear EncoderMode field
		htim3.Instance->SMCR |= TIM_ENCODERMODE_TI1 << TIM_SMCR_SMS_Pos; // Set EncoderMode field to TIM_ENCODERMODE_TI1
		htim3.Instance->CCER &= ~TIM_CCER_CC1P; // Clear IC1Polarity field
		htim3.Instance->CCER |= TIM_ICPOLARITY_BOTHEDGE << TIM_CCER_CC1P_Pos; // Set IC1Polarity field to TIM_ICPOLARITY_BOTHEDGE
	}
	else if (strcmp(mode, "x4") == 0)
	{
		htim3.Instance->SMCR &= ~TIM_SMCR_SMS; // Clear EncoderMode field
		htim3.Instance->SMCR |= TIM_ENCODERMODE_TI12 << TIM_SMCR_SMS_Pos; // Set EncoderMode field to TIM_ENCODERMODE_TI12
		htim3.Instance->CCER &= ~TIM_CCER_CC1P; // Clear IC1Polarity field
		htim3.Instance->CCER |= TIM_ICPOLARITY_BOTHEDGE << TIM_CCER_CC1P_Pos; // Set IC1Polarity field to TIM_ICPOLARITY_BOTHEDGE
		htim3.Instance->CCER &= ~TIM_CCER_CC2P; // Clear IC2Polarity field
		htim3.Instance->CCER |= TIM_ICPOLARITY_BOTHEDGE << TIM_CCER_CC2P_Pos; // Set IC2Polarity field to TIM_ICPOLARITY_BOTHEDGE
	}
}

void PID(int target, float kp, float ki, float kd, int delay_time)
{
	unsigned long currT = HAL_GetTick() * 1000;

	float deltaT = ((float)(currT - prevT))/1.0e6;
	prevT = currT;

	//error
	int e = target - encoder_count;

	//derivative
	float dedt = (e-eprev)/deltaT;

	//integral
	eintegral += e*deltaT;

	float u = kp*e + kd*dedt + ki*eintegral;

	//motor power
	float pwr = u;
	if (u < 0)
	{
		pwr = -u;
	}
	if (pwr > 10000)
	{
		pwr = 10000;
	}

	//motor direction
	int dir=1;
	if (u < 0)
	{
		dir = -1;
	}

	//signal the motor
	setMotor(dir, GPIOA, pwr, &htim2, GPIO_PIN_8, GPIO_PIN_9);

	//store previous error
	eprev = e;

	HAL_Delay(delay_time);
}

void moveto(int target)
{
	setMotor(1, GPIOA, target, &htim2, GPIO_PIN_8, GPIO_PIN_9);
	if (encoder_count >= target)
	{
		setMotor(2, GPIOA, 0, &htim2, GPIO_PIN_8, GPIO_PIN_9);
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
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Init(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  setmode("x1");
  HAL_UART_Transmit_DMA(&huart4, MSG, sizeof(MSG));
  HAL_UART_Receive_DMA(&huart4, Received_MSG, sizeof(Received_MSG));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (strcmp(mode, old_mode) != 0) {
		setmode(mode);
	    strcpy(old_mode, mode);
	  }
	  if (dir == 1 || dir == -1)
	  {
		  setMotor(dir, GPIOA, spd_target, &htim2, GPIO_PIN_8, GPIO_PIN_9);
	  }
	  else
	  {
		  PID(pos_target, kp, ki, kd, 10);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 900;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
