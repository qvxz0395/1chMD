/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#define ARM_MATH_CM0
#include "arm_math.h"
#include "arm_const_structs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//PID value
#define KP 6.1
#define KI 0
#define KD 0
//ADC
#define ADC_CONVERTED_DATA_BUFFER_SIZE 1
#define CURRENT_AVERAGING_NUM 50//[times]
#define LPF_VALUE 0.08//if this value is 1, cut off frequency is infinitiy.
#define CURRENT_SENS 0.0176//[V/A]
#define CURRENT_DRIFT 0.331//[V]
//#define CURRENT_DRIFT_VALUE 406.97//[-]
float CURRENT_DRIFT_VALUE;
#define VOLTAGE_REF 3.3//[V]
#define VOLTAGE_REF_VALUE 4096//[-]

//Current limit
#define CURRENT_LIMIT_LPF_VALUE 0.01 //if this value is 1, cut off frequency is infinitiy.
#define CURRENT_LIMIT_FALLING_GAIN 100
#define CURRENT_LIMIT_RISING_GAIN 1 //

//Duty resolution and PWM frequency
#define PWM_PSC (1-1)
#define PWM_ARR (2000-1)//	48MHz/PWM_PSC*PWM_ARR = PWM frequency
#define PWM_CHANGE_CONTROL_THRESHOLD 85
#define PWM_DUTY_MAX_ABS 30

//Timer interruption1 frequency
#define TIM1_PSC (48-1)//48MHz/Timer_PSC = Timer interruption1(main control) frequency
#define TIM1_ARR (1000-1)
#define TIM14_PSC (48-1)
#define TIM14_ARR (250-1)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

float target_value =1;

//CAN

uint32_t pTxMailbox;
uint32_t mailbox,timestamp;
uint8_t CAN_data[]={0xaa,0x55,0x00,0x00,0xff,0x01,0x80,0x45};
uint8_t CAN_ReceiveData[8]={};


//ADC
uint16_t ADC_Data[ADC_CONVERTED_DATA_BUFFER_SIZE];
float ADC_Slope;
float ADC_Offset;


//PWM
float Duty_Present = 0;//100%=100
float PWM_Duty_Max=10;//[%]

//Current
float Current_Present = 0;//[A]
float current_average_dt=0;
float Current_Limit_Value = 5;//[A] //if Current_Limit_State is ENABLE and over this value become duty down except for transient over.
uint8_t Current_Limit_State= ENABLE;

//ERROR
typedef enum{
	HAL_ERROR_STATE,
	PWM_DUTY_MAX_ABS_OVER,
	PWM_DUTY_MAX_OVER,
	CURRENT_LIMIT_RISING,
	CURRENT_LIMIT_OVER,
	CAN_ERROR
}error;
uint8_t Error_State=0;

float CURRENT_AVERAGE_DT;
float DUTY_AVERAGE_DT;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */
int Get_Enocder_Value(void);
float Get_Current_Value(void);
void Motor_pwm(float pwm);
void Motor_InitSetting(char setting);
float pid(float present, float target);
void CAN_Tx(uint8_t aData[]);
void CAN_Rx(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void TIM1_BRK_UP_TRG_COM_IRQHandler(void)//TIM1 timer interruption
{
	HAL_TIM_IRQHandler(&htim1);
	static float cos_,i=0;

	//Motor_pwm(cos_);
	i=i+0.001;
	target_value =0.5*arm_sin_f32(6.28318530718*i);

	//if(i ==0) Motor_pwm(5);
	//else Motor_pwm(-5);
	//i++;
	//if(i==2) i=0;

	//Duty_Present += pid(Current_Present,target_value);
	//Motor_pwm(Duty_Present);

	//Motor_pwm(0);

	//Motor_pwm(pid(Current_Value,target_value));
}

void TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */

	//CAN_Rx();
	CAN_Tx(CAN_data);
  /* USER CODE END TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM14_IRQn 1 */
  int8_t direction;
  float sum=0;
  static float current_abs_lpf,current_abs_prev;

  static float current_abs_high_lpf,current_abs_prev_high_lpf;

	for(uint16_t i =0; i<CURRENT_AVERAGING_NUM; i++)//get current value by normalization filter
	{
		HAL_ADC_Start_DMA(&hadc,(uint32_t *)ADC_Data,ADC_CONVERTED_DATA_BUFFER_SIZE);//start adc
		if(HAL_GPIO_ReadPin(PHASE_GPIO_Port,PHASE_Pin) == 1) sum += ADC_Data[0];
		else sum -= ADC_Data[0];
	}
	sum /= CURRENT_AVERAGING_NUM;
	if(sum > 0) direction = 1;
	else
	{
		direction = -1;
		sum *= -1;
	}
	current_abs_lpf = (1-LPF_VALUE)*current_abs_prev + LPF_VALUE*((sum-CURRENT_DRIFT_VALUE)*VOLTAGE_REF/(CURRENT_SENS*VOLTAGE_REF_VALUE));
	current_abs_prev = current_abs_lpf;
	Current_Present = direction*current_abs_lpf;

	if(Current_Limit_State == ENABLE)//current limit
	{
		current_abs_high_lpf = (1-CURRENT_LIMIT_LPF_VALUE)*current_abs_prev_high_lpf + CURRENT_LIMIT_LPF_VALUE*((sum-CURRENT_DRIFT_VALUE)*VOLTAGE_REF/(CURRENT_SENS*VOLTAGE_REF_VALUE));
		current_abs_prev_high_lpf = current_abs_high_lpf;
		current_average_dt = direction*current_abs_high_lpf;
		CURRENT_AVERAGE_DT = current_average_dt;
		if(fabs(current_average_dt) > Current_Limit_Value && PWM_Duty_Max > 0)//current shut down
		{
			PWM_Duty_Max+=(Current_Limit_Value-current_average_dt)*CURRENT_LIMIT_FALLING_GAIN;
			Error_State|=(1<<CURRENT_LIMIT_OVER);
		}
		else Error_State&=~(1<<CURRENT_LIMIT_OVER);

		Motor_pwm(Duty_Present);
	}
	HAL_CAN_IRQHandler(&hcan);

	/* USER CODE END TIM14_IRQn 1 */
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
  MX_DMA_Init();
  MX_ADC_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc);//adc calibration
  Motor_InitSetting(1);
  Current_Limit_State=ENABLE;

  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);//encoder start
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//PWM generation
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
  HAL_CAN_Start(&hcan);
  HAL_Delay(10);
  HAL_TIM_Base_Start_IT(&htim14);
  HAL_Delay(500);
  HAL_TIM_Base_Start_IT(&htim1);//timer interruption

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 6;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_14TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  HAL_GPIO_WritePin(CAN_STBY_GPIO_Port,CAN_STBY_Pin,0);
 // CAN_FilterTypeDef sFilterConfig;
 // sFilterConfig.FilterMaskIdHigh = 0;
 // sFilterConfig.FilterMaskIdLow = 0;
 // sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
 // sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
 // sFilterConfig.FilterFIFOAssignment =CAN_FILTER_FIFO0;
//  sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;

 // HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
  /* USER CODE END CAN_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = TIM1_PSC;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = TIM1_ARR;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = PWM_PSC;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = PWM_ARR;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  htim3.Init.Period = 65535;
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
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = TIM14_PSC;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = TIM14_ARR;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SR_Pin|PHASE_Pin|CAN_STBY_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SR_Pin PHASE_Pin CAN_STBY_Pin LED1_Pin */
  GPIO_InitStruct.Pin = SR_Pin|PHASE_Pin|CAN_STBY_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : FF2_Pin */
  GPIO_InitStruct.Pin = FF2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FF2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FF1_Pin */
  GPIO_InitStruct.Pin = FF1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FF1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_X_Pin */
  GPIO_InitStruct.Pin = ENC_X_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENC_X_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIP3_Pin */
  GPIO_InitStruct.Pin = DIP3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DIP3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM16;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

int Get_Enocder_Value(void)
{
	return TIM3->CNT;
}


void Motor_pwm(float duty)
{
//limit PWM
	int8_t direction;
	if(duty > 0) direction = 1;
	else direction = -1;

	if(direction*duty > PWM_DUTY_MAX_ABS)
	{
		duty = direction*PWM_DUTY_MAX_ABS;
		Error_State|=(1<<PWM_DUTY_MAX_ABS_OVER);
	}
	else Error_State &=~(1<<PWM_DUTY_MAX_ABS_OVER);

	if(direction*duty > PWM_Duty_Max)
	{
		duty = direction*PWM_Duty_Max;
		Error_State|=(1<<PWM_DUTY_MAX_OVER);
	}
	else Error_State &=~(1<<PWM_DUTY_MAX_OVER);

	if(PWM_Duty_Max < PWM_DUTY_MAX_ABS && (Error_State & (1<<CURRENT_LIMIT_OVER)) == 0)
	{
		PWM_Duty_Max+=(Current_Limit_Value-current_average_dt)*CURRENT_LIMIT_RISING_GAIN;
		Error_State|=(1<<CURRENT_LIMIT_RISING);
	}
	else Error_State&=~(1<<CURRENT_LIMIT_RISING);
	Duty_Present = duty;

//output PWM
	if(duty < 0)//set direction
	{
		HAL_GPIO_WritePin(PHASE_GPIO_Port,PHASE_Pin,0);
		duty*=-1;
	}
	else HAL_GPIO_WritePin(PHASE_GPIO_Port,PHASE_Pin,1);

	if(duty > PWM_CHANGE_CONTROL_THRESHOLD)//PWM_FET_Lo
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_ARR+1);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty*(PWM_ARR+1)/100);
	}
	else//PWM_FET_Hi
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_ARR+1);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty*(PWM_ARR+1)/100);
	}

}


void Motor_InitSetting(char setting)
{

	if(setting == 0)//This mode is slow decay no dead time, but it has polarity of braking. no recomend except for special purpose.
	{
		HAL_GPIO_WritePin(SR_GPIO_Port,SR_Pin,0);
	}
	else if(setting == 1)//This mode is slow decay and no polarity of braking, but it has dead time(180ns) to prevent short FET of switching.
	{
		HAL_GPIO_WritePin(SR_GPIO_Port,SR_Pin,1);
	}

	for(uint16_t i = 0; i<4096; i++)//get current drift value and derieve adc calibration value.
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_ARR+1);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		HAL_ADC_Start_DMA(&hadc,(uint32_t *)ADC_Data,ADC_CONVERTED_DATA_BUFFER_SIZE);//start adc
		CURRENT_DRIFT_VALUE += ADC_Data[0];
	}
	CURRENT_DRIFT_VALUE /= 4096;
}


float pid(float present, float target)
{
	//pid
	static float diff_[2];
	static float integral_;
	static float p_, i_, d_;
	static float kp_=KP, ki_=KI, kd_=KD;
	static float control_;

	float difference = target-present;
    diff_[0] = diff_[1];
    diff_[1] = difference;

    if(diff_[0] * diff_[1] < 0)
    	integral_ = 0;
    else
    	integral_ += ((diff_[0] + diff_[1]) / 2.0);
    p_ = kp_ * diff_[1];
    i_ = ki_ * integral_;
    d_ = kd_ * (diff_[0] - diff_[1]);
    control_ = p_ + i_ + d_;
    return control_;
}

void CAN_Tx(uint8_t aData[])
{
	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.StdId = 0x8;
	TxHeader.ExtId = 0x0;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = sizeof(aData)/sizeof(uint8_t);
	TxHeader.TransmitGlobalTime = DISABLE;

	  if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, aData, &pTxMailbox) != HAL_OK) Error_State|=(1<<CAN_ERROR);
	  else Error_State&=~(1<<CAN_ERROR);

}

void CAN_Rx(void)
{
	CAN_RxHeaderTypeDef RxHeader;
	RxHeader.StdId = 0x6;
	RxHeader.ExtId = 0x0;
	RxHeader.IDE = CAN_ID_STD;
	RxHeader.RTR = CAN_RTR_DATA;
	RxHeader.DLC = sizeof(CAN_ReceiveData)/sizeof(uint8_t);

	HAL_CAN_GetRxMessage(&hcan,CAN_RX_FIFO0,&RxHeader,CAN_ReceiveData);
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
