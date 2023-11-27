/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define UP_Pin GPIO_PIN_13
#define UP_GPIO_Port GPIOC
#define DOWN_Pin GPIO_PIN_14
#define DOWN_GPIO_Port GPIOC
#define POWER_Pin GPIO_PIN_15
#define POWER_GPIO_Port GPIOC
#define Fan_Temp_Pin GPIO_PIN_0
#define Fan_Temp_GPIO_Port GPIOA
#define Fan_F_Pin GPIO_PIN_2
#define Fan_F_GPIO_Port GPIOA
#define MODE_Pin GPIO_PIN_3
#define MODE_GPIO_Port GPIOA
#define SAVE_Pin GPIO_PIN_4
#define SAVE_GPIO_Port GPIOA
#define Zero_Crossing_Pin GPIO_PIN_5
#define Zero_Crossing_GPIO_Port GPIOA
#define Zero_Crossing_EXTI_IRQn EXTI9_5_IRQn
#define Ctrl_Prd_Triac_Pin GPIO_PIN_1
#define Ctrl_Prd_Triac_GPIO_Port GPIOB
#define DS18B20_Pin GPIO_PIN_15
#define DS18B20_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
