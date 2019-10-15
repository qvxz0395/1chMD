/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIP2_Pin GPIO_PIN_0
#define DIP2_GPIO_Port GPIOF
#define DIP1_Pin GPIO_PIN_1
#define DIP1_GPIO_Port GPIOF
#define CURRENT_SENS_Pin GPIO_PIN_0
#define CURRENT_SENS_GPIO_Port GPIOA
#define FF1_Pin GPIO_PIN_1
#define FF1_GPIO_Port GPIOA
#define ADC_IO_Pin GPIO_PIN_2
#define ADC_IO_GPIO_Port GPIOA
#define DIP0_Pin GPIO_PIN_3
#define DIP0_GPIO_Port GPIOA
#define SR_Pin GPIO_PIN_4
#define SR_GPIO_Port GPIOA
#define PHASE_Pin GPIO_PIN_5
#define PHASE_GPIO_Port GPIOA
#define PWMH_Pin GPIO_PIN_6
#define PWMH_GPIO_Port GPIOA
#define PWML_Pin GPIO_PIN_7
#define PWML_GPIO_Port GPIOA
#define CW_Pin GPIO_PIN_0
#define CW_GPIO_Port GPIOB
#define CCW_Pin GPIO_PIN_1
#define CCW_GPIO_Port GPIOB
#define ENC_A_Pin GPIO_PIN_8
#define ENC_A_GPIO_Port GPIOA
#define ENC_B_Pin GPIO_PIN_9
#define ENC_B_GPIO_Port GPIOA
#define ENC_X_Pin GPIO_PIN_10
#define ENC_X_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_4
#define LED_GPIO_Port GPIOB
#define DIP3_Pin GPIO_PIN_5
#define DIP3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
