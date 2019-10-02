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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//PID value
#define KP 15.8
#define KI 0
#define KD 0
//ADC
#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  1)
#define CURRENT_AVERAGING_NUM 100//[times]
#define CURRENT_SENS 0.0176//[V/A]
#define CURRENT_DRIFT 0.332//[V]
#define CURRENT_DRIFT_VALUE 406.97//[-]
#define VOLTAGE_REF 3.32//[V]
#define VOLTAGE_REF_VALUE 4093.64//[-]

//Current limit
#define CURRENT_LIMIT_DT 40 //x0.1ms
#define CURRENT_LIMIT_FALL_RATIO 70 //percent
#define CURRENT_LIMIT_RISING_RATIO 5 //percent

//Duty resolution and PWM frequency
#define PWM_PSC (1-1)
#define PWM_ARR (2000-1)//	48MHz/PWM_PSC*PWM_ARR = PWM frequency
#define PWM_CHANGE_CONTROL_THRESHOLD 85
#define PWM_DUTY_MAX_ABS 50

//Timer interruption1 frequency
#define TIM1_PSC (10000-1)//48MHz/Timer_PSC = Timer interruption1(main control) frequency
#define TIM1_ARR (4800-1)
#define TIM14_PSC (400-1)
#define TIM14_ARR (10-1)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

float target_value =1;

//ADC
uint16_t ADC_Data[ADC_CONVERTED_DATA_BUFFER_SIZE];
float ADC_Slope = (VOLTAGE_REF - CURRENT_DRIFT)/(VOLTAGE_REF_VALUE - CURRENT_DRIFT_VALUE);
float ADC_Offset = VOLTAGE_REF - (((VOLTAGE_REF - CURRENT_DRIFT)/(VOLTAGE_REF_VALUE - CURRENT_DRIFT_VALUE))*VOLTAGE_REF_VALUE) ;

//PWM
float Duty_Present = 0;//100%=100
uint16_t PWM_Duty_Max=PWM_DUTY_MAX_ABS;//[%]

//Current
float Current_Present = 0;//[A]
float Current_Limit_Value = 20;//[A] //if Current_Limit_State is ENABLE and over this value become duty down except for transient over.
uint8_t Current_Limit_State= ENABLE;

//pid
double diff_[2];
double integral_;
double p_, i_, d_;
double kp_=KP, ki_=KI, kd_=KD;
double control_;

//ERROR
typedef enum{
	HAL_ERROR_STATE,
	PWM_DUTY_MAX_ABS_OVER,
	PWM_DUTY_MAX_OVER,
	CURRENT_LIMIT_RISING,
	CURRENT_LIMIT_OVER
}error;
uint64_t Error_State=0;
//debug value
uint8_t LED1_State=0;
uint8_t LED2_State=0;
uint8_t PHASE_State = 0;
uint8_t SR_State = 0;
uint8_t PWML_State = 0;
uint8_t DIP0_State = 0;
uint8_t DIP1_State = 0;
uint8_t DIP2_State = 0;
uint8_t DIP3_State = 0;
int16_t Enc_Value = 0;

float CURRENT_AVERAGE_DT;
float DUTY_AVERAGE_DT;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM14_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
int Get_Enocder_Value(void);
float Get_Current_Value(void);
void Motor_pwm(float pwm);
void Motor_InitSetting(char setting);
double pid(double present, double target);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void TIM1_BRK_UP_TRG_COM_IRQHandler(void)//TIM1 timer interruption
{

	static int i;
	if(i<1)
	{
		Motor_pwm(1.5);
	}
	if(i>=1)
	{
		Motor_pwm(1.5);
	}
	i++;
	if(i==2) i=0;


	HAL_TIM_IRQHandler(&htim1);

	//Motor_pwm(pid(Current_Value,target_value));
}

void TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */

  /* USER CODE END TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM14_IRQn 1 */
  	float sum=0;

  	for(uint16_t i =0; i<CURRENT_AVERAGING_NUM; i++)//get current value by normalization filter
	{
		HAL_ADC_Start_DMA(&hadc,(uint32_t *)ADC_Data,ADC_CONVERTED_DATA_BUFFER_SIZE);//start adc
		if(HAL_GPIO_ReadPin(PHASE_GPIO_Port,PHASE_Pin) == 1) sum += ADC_Data[0];
		else sum -= ADC_Data[0];
	}
	sum/=CURRENT_AVERAGING_NUM;
	Current_Present = (sum/fabs(sum))*((fabs(sum)*ADC_Slope + ADC_Offset - CURRENT_DRIFT)/CURRENT_SENS);

	if(Current_Limit_State == ENABLE)//current limit
	{
		static float current_buff[CURRENT_LIMIT_DT];
		float current_average_dt=0;
		static float duty_buff[CURRENT_LIMIT_DT];
		float duty_average_dt=0;

		for(uint16_t j =(CURRENT_LIMIT_DT-1); j>0; j--)//ignore transient current overload.
		{
			current_buff[j] = current_buff[j-1];//shift current buffer
			current_average_dt += current_buff[j];//sum current buffer

			duty_buff[j] = duty_buff[j-1];//shift duty buffer
			duty_average_dt += duty_buff[j];//sum duty buffer
		}

		current_buff[0] = Current_Present;//substitution new current
		current_average_dt += Current_Present;
		current_average_dt /= CURRENT_LIMIT_DT;//calc dt current
		CURRENT_AVERAGE_DT = current_average_dt;

		duty_buff[0] = Duty_Present;//substitution new duty
		duty_average_dt += Duty_Present;
		duty_average_dt /= CURRENT_LIMIT_DT;//calc dt duty
		DUTY_AVERAGE_DT = duty_average_dt;

		if(fabs(current_average_dt) > Current_Limit_Value && PWM_Duty_Max > 0)//current shut down
		{
			PWM_Duty_Max = (float)duty_average_dt*CURRENT_LIMIT_FALL_RATIO/100;
			Error_State|=(1<<CURRENT_LIMIT_OVER);
		}
		else if(PWM_Duty_Max < PWM_DUTY_MAX_ABS)
		{
			PWM_Duty_Max+=CURRENT_LIMIT_RISING_RATIO;
			Error_State&=~(1<<CURRENT_LIMIT_OVER);
			Error_State|=(1<<CURRENT_LIMIT_RISING);
		}
		else Error_State&=~(1<<CURRENT_LIMIT_RISING|1<<CURRENT_LIMIT_OVER);

		Motor_pwm(Duty_Present);
	}
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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM14_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  Motor_InitSetting(1);
  Current_Limit_State=ENABLE;
  HAL_ADCEx_Calibration_Start(&hadc);//adc calibration
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);//encoder start
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//PWM generation
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
  HAL_ADC_Start_DMA(&hadc,(uint32_t *)ADC_Data,ADC_CONVERTED_DATA_BUFFER_SIZE);//start adc
  HAL_Delay(1000);
  HAL_TIM_Base_Start_IT(&htim1);//timer interruption
  HAL_TIM_Base_Start_IT(&htim14);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
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
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
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

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SR_GPIO_Port, SR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SR_Pin */
  GPIO_InitStruct.Pin = SR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIP0_Pin DIP1_Pin */
  GPIO_InitStruct.Pin = DIP0_Pin|DIP1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIP2_Pin DIP3_Pin */
  GPIO_InitStruct.Pin = DIP2_Pin|DIP3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

  /*Configure GPIO pin : FF2_Pin */
  GPIO_InitStruct.Pin = FF2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FF2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

int Get_Enocder_Value(void)
{
	return TIM3->CNT;
}


void Motor_pwm(float duty)
{
//limit PWM
	if(fabs(duty) > PWM_DUTY_MAX_ABS)
	{
		duty = (duty/fabs(duty)) *PWM_DUTY_MAX_ABS;
		Error_State|=(1<<PWM_DUTY_MAX_ABS_OVER);
	}
	else Error_State &=~(1<<PWM_DUTY_MAX_ABS_OVER);

	if(fabs(duty) > PWM_Duty_Max)
	{
		duty = (duty/fabs(duty)) *PWM_Duty_Max;// current limit.
		Error_State|=(1<<PWM_DUTY_MAX_OVER);
	}
	else Error_State&=~(1<<PWM_DUTY_MAX_OVER);

	Duty_Present = duty;

//output PWM
	if(duty < 0)//set direction
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
		duty*=-1;
	}
	else __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_ARR+1);

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
}


double pid(double present, double target)
{
	double difference = target-present;
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
