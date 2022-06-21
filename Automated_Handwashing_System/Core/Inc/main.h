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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define soapSensor_Pin GPIO_PIN_0
#define soapSensor_GPIO_Port GPIOA
#define soapValve_Pin GPIO_PIN_4
#define soapValve_GPIO_Port GPIOA
#define soapLED_Pin GPIO_PIN_5
#define soapLED_GPIO_Port GPIOA
#define fanSensor_Pin GPIO_PIN_6
#define fanSensor_GPIO_Port GPIOA
#define waterValve_Pin GPIO_PIN_12
#define waterValve_GPIO_Port GPIOB
#define sensorTrigger_Pin GPIO_PIN_9
#define sensorTrigger_GPIO_Port GPIOA
#define fan_Pin GPIO_PIN_5
#define fan_GPIO_Port GPIOB
#define waterSensor_Pin GPIO_PIN_6
#define waterSensor_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
