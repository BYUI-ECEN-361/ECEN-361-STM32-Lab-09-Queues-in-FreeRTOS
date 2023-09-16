/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MultiFunctionShield.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

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
ADC_HandleTypeDef hadc2;

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

const osThreadAttr_t bigStackTask_attributes = {
  .name = "bigStackTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Here are pointers to the tasks so I can suspend/resume */

osThreadId_t ADC_Task_Handle;
osThreadId_t DAC_Task_Handle;
osThreadId_t PWM_Task_Handle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
void D2_Task(void *argument);
void PWM_Brightness_Task(void *argument);
void Sample_Print_POT_ADC_Task();
void Do_SevenSeg_Display();
void DAC_Cycle_Task();
// void Read_and_Transmit_Task();
// void Receive_and_Print_Task();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*** Globals *********/
uint8_t RX_Buffer[BUFFER_SIZE] = {0};
typedef enum {ADC_mode,PWM_mode,DAC_mode} display_modes ;
display_modes display_mode = ADC_mode;
float ADC_voltage;
int  DAC_value = 4095;	// the value written to the DAC  -- set to 12-bit
bool Tasks_Running = true;
int duty_cycle_percent = 0;

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
  // HAL_GPIO_WritePin(PWM_LED_GPIO_Port, PWM_LED_Pin, 1);
  // HAL_GPIO_WritePin(LED_D3_GPIO_Port,LED_D3_Pin,0);



  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM17_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */

  // Start timer
  HAL_TIM_Base_Start_IT(&htim17);							// LED SevenSeg cycle thru them
  MultiFunctionShield_Clear();								// Clear the 7-seg display
  Clear_LEDs();												// Clear the lights
  printf("\033\143");
  printf("Welcome to ECEN-361 Lab-06\n\r\n\r");
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);		// Start the Timer with the PWM Assigned.
/**
 * Note that Timer-1 Channel 1 goes to our MultiBoard D3, and it's Negative True output
 *
 */



  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadNew(D2_Task, "D2_Task", &defaultTask_attributes);
  PWM_Task_Handle = osThreadNew(PWM_Brightness_Task, "PWM_Task", &defaultTask_attributes);
  ADC_Task_Handle = osThreadNew(Sample_Print_POT_ADC_Task, "ADC_Task", &bigStackTask_attributes);
  DAC_Task_Handle = osThreadNew(DAC_Cycle_Task, "DAC_Task", &defaultTask_attributes);
  osThreadNew(Do_SevenSeg_Display, "Display_Task", &defaultTask_attributes);




  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Read_and_Transmit_Task();

	// osDelay(500);
	/*
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, 0);
	osDelay(5); // wait a bit before transmitting so the process can start the read
	HAL_SPI_Transmit(&hspi1, RX_Buffer, BUFFER_SIZE , HAL_MAX_DELAY);
	osDelay(5); // wait a bit before transmitting so the process can start the read
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, 1);
	*/

	// Sample_Print_POT_ADC_Task();

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_6B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 16000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 16000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 800-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 100;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_D2_Pin|SevenSeg_CLK_Pin|SevenSeg_DATA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SevenSeg_LATCH_GPIO_Port, SevenSeg_LATCH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LM35_IN_Pin */
  GPIO_InitStruct.Pin = LM35_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LM35_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_1_Pin */
  GPIO_InitStruct.Pin = Button_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_2_Pin */
  GPIO_InitStruct.Pin = Button_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_D2_Pin */
  GPIO_InitStruct.Pin = LED_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_D2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_3_Pin */
  GPIO_InitStruct.Pin = Button_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SevenSeg_CLK_Pin SevenSeg_DATA_Pin */
  GPIO_InitStruct.Pin = SevenSeg_CLK_Pin|SevenSeg_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SevenSeg_LATCH_Pin */
  GPIO_InitStruct.Pin = SevenSeg_LATCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SevenSeg_LATCH_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */







/************  Task-Creation-Part-B *****************/
void D2_Task(void *argument)
	{ while(true)
		{
		  while(!Tasks_Running)
			  {
			  HAL_GPIO_TogglePin(LED_D2_GPIO_Port,LED_D2_Pin);
			  osDelay(500);
			  }
			  HAL_GPIO_WritePin(LED_D2_GPIO_Port,LED_D2_Pin,1);
			  osDelay(5);
		}
	}

void DAC_Cycle_Task()
	{
	int increment =  100;
	int  up = increment;
	int  down = -1 * increment;
	int  inc = down;  //Start by going down

	/*
	 * This task will just change the output voltage on the DAC in a periodic fashion
	 * Remember the light is negative true output, so a DAC_value of 0: brightest value
	 * */

	while (true)
		{
		while (Tasks_Running)
			{
			DAC_value = DAC_value + inc;
			if (DAC_value <= 0 )
				{
				DAC_value = 0;
				inc = up;
				}
			if (DAC_value >= 4096)
				{
				DAC_value = 4000;
				inc = down;
				}
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (DAC_value & 0xfff));
			// printf("DAC Value is: %i \n\r",DAC_value);
			osDelay(300);
			}
		}
	}

void Sample_Print_POT_ADC_Task()
	{
	uint16_t current_sample = 1000;
	float max_reference_voltage = 5.0;
	// read this from the configuration of the ADC
	int bits_per_sample = (((ADC2->CFGR && 0xc)>>2) * 2) + 6;	// ADC config register [3:2] defines 6,8,10, or 12-bit resolution
	int samples = 1 << bits_per_sample;		//power of 2

	while(true)
		{
		current_sample = Poll_POT_ADC_Value();
		ADC_voltage = ((float)current_sample/(float)samples * max_reference_voltage);
		osDelay(1000);
		}
	}

void PWM_Brightness_Task(void *arguments)
	{
		int percent_per_step = 5;
		// int full_count =  TIM15->ARR;		//Get the top range from what's set in the init.
		int full_count =  TIM4->ARR;		//Get the top range from what's set in the init.
		bool count_up = true;
		uint16_t ccr1 = 0;
		while(true)
		{
			while (Tasks_Running)
				{
				ccr1 = (uint16_t) duty_cycle_percent * full_count / 100;
				//TIM15->CCR1 = ccr1;		// This is the limit that turns on the PWM
				TIM4->CCR1 = ccr1;		// This is the limit that turns on the PWM
				if (count_up)
					{
					duty_cycle_percent += percent_per_step;
					if (duty_cycle_percent >=100) count_up = false;
					} else
					{
					duty_cycle_percent -= percent_per_step;
					if (duty_cycle_percent <=0) count_up = true;
					}
				osDelay(100);
				}
		}
	}



void Read_and_Transmit_Task()
	{
	uint8_t receive_byte;
	// uint8_t receive_buffer[BUFFER_SIZE] = {0};
	// uint8_t *receive_buffer_ptr = receive_buffer;
	uint8_t bytes_in =0;
	uint8_t xmitmsg[] = "\n\rInput Line to Send ->";
	uint8_t sndmsg[] = "\n\rSending -> ";
	uint8_t *xmitmsg_ptr = xmitmsg;
	uint8_t *sndmsg_ptr = sndmsg;

		bytes_in = 0;
		receive_byte = 0;
		HAL_UART_Transmit(&huart2, xmitmsg_ptr, 23, HAL_MAX_DELAY);

		/* This task reads a line from the Serial/USB port and
		 * transmits out thru SPI
		 * Note that this is polling!  One byte at a time.  Very inefficient
		 */
		while (receive_byte != '\r')
		{
			while (HAL_UART_Receive(&huart2, &receive_byte, 1,10) != HAL_OK) osDelay(1);
			/* Now we have a byte, if it's a carriage return, send the string
			 * If not, put it on the buffer
			 */
			RX_Buffer[bytes_in] = receive_byte;
			HAL_UART_Transmit(&huart2, &RX_Buffer[bytes_in++] , 1, HAL_MAX_DELAY);  //echo each one as it's typed
		}

		RX_Buffer[bytes_in++] = '\n'; // Add a line_feed
		// Tell the User what we got and what we're sending
		HAL_UART_Transmit(&huart2, sndmsg_ptr, 13, HAL_MAX_DELAY);
		//HAL_UART_Transmit(&huart2, receive_buffer_ptr, bytes_in, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart2, RX_Buffer, bytes_in, HAL_MAX_DELAY);
		// Now send it from the SPI Master (SPI_1) -> SPI Slave (SPI_2)
		// Turn on the ChipEnable (SPI1_NSS -- active low)
		}




/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

void Do_SevenSeg_Display()
	{ /* Just cycle thru the modes when there's a button push */
	while (true)
		{
		switch (display_mode)
			{
			case ADC_mode:
				disp_adc_on_7seg(ADC_voltage);
				break;
			case PWM_mode:
				// Show the duty cycle percentage
			    MultiFunctionShield_Display_PWM(duty_cycle_percent);
				break;
			case DAC_mode:
			    MultiFunctionShield_Display(DAC_value);
				break;
			}
		osDelay(20);
		}
	}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{
	// All three buttons generate GPIO  interrupts
	switch(GPIO_Pin)
	{
	case Button_1_Pin:
		/* Button_1 is the Display Mode switcher.  See the lab description */
		switch (display_mode)
		{
		case ADC_mode:
			display_mode = PWM_mode;
			break;
		case PWM_mode:
			display_mode = DAC_mode;
			break;
		case DAC_mode:
			display_mode = ADC_mode;
			break;
		default: __NOP();
		}
		break;
	case Button_2_Pin:
		/* Button_2 is the Start/Stop Button. */
			Tasks_Running = !Tasks_Running;
		/* If we just went into stop mode, then display some info */
		if (!Tasks_Running)
			{
			printf("STOPPED  D2 Blinking\n\r   PWM  Value: %%%d\n\r   CCRL Value: %d\n\r",
				duty_cycle_percent,
				((uint16_t) TIM4->ARR / 100 * duty_cycle_percent ));
				// ccr1 = (uint16_t) duty_cycle_percent * full_count / 100;
			printf("   DAC 12-bit :0x%x\n\r",DAC_value);
			} else
			printf("RE-STARTING\n\r\r");
		break;
	case Button_3_Pin:
		break;
	default: __NOP();
	HAL_Delay(70);  //* Time to make sure the switch is debounced
	}
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	  if (htim == &htim17 ) { MultiFunctionShield__ISRFunc(); }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
}

uint16_t Poll_POT_ADC_Value(void)
	{
	uint16_t  ADC_Result;
	// Start ADC Conversion of the 10K potentiometer
	MX_ADC2_Init();
	HAL_ADC_Start(&hadc2);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

	// The conversion should be less than a millisec
	// so it'll be done while we're waiting for the next number to show
	osDelay(10);

   // Poll ADC2 Perihperal & TimeOut = 1mSec
	HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
   // Read The ADC Conversion Result & Map It To PWM DutyCycle
	ADC_Result = HAL_ADC_GetValue(&hadc2);
	return ADC_Result;

  /* USER CODE END Callback 1 */
}

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
