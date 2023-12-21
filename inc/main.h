/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

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
#define OUT2_OE_Pin GPIO_PIN_13
#define OUT2_OE_GPIO_Port GPIOC
#define OUT2_PP_Pin GPIO_PIN_14
#define OUT2_PP_GPIO_Port GPIOC
#define LED_L_Pin GPIO_PIN_15
#define LED_L_GPIO_Port GPIOC
#define OUT3_OE_Pin GPIO_PIN_1
#define OUT3_OE_GPIO_Port GPIOF
#define OUT3_PP_Pin GPIO_PIN_0
#define OUT3_PP_GPIO_Port GPIOA
#define OUT4_PP_Pin GPIO_PIN_1
#define OUT4_PP_GPIO_Port GPIOA
#define OUT4_OE_Pin GPIO_PIN_2
#define OUT4_OE_GPIO_Port GPIOA
#define OUT5_OE_Pin GPIO_PIN_3
#define OUT5_OE_GPIO_Port GPIOA
#define OUT5_PP_Pin GPIO_PIN_4
#define OUT5_PP_GPIO_Port GPIOA
#define FAN_L_Pin GPIO_PIN_5
#define FAN_L_GPIO_Port GPIOA
#define FAN_M_Pin GPIO_PIN_6
#define FAN_M_GPIO_Port GPIOA
#define FAN_AUTO_Pin GPIO_PIN_7
#define FAN_AUTO_GPIO_Port GPIOA
#define LED_M_Pin GPIO_PIN_0
#define LED_M_GPIO_Port GPIOB
#define LED_H_Pin GPIO_PIN_1
#define LED_H_GPIO_Port GPIOB
#define LED_AUTO_Pin GPIO_PIN_2
#define LED_AUTO_GPIO_Port GPIOB
#define LED_LIGHTS_Pin GPIO_PIN_11
#define LED_LIGHTS_GPIO_Port GPIOB
#define LED_DELAY_Pin GPIO_PIN_12
#define LED_DELAY_GPIO_Port GPIOB
#define LED_FILTER_Pin GPIO_PIN_13
#define LED_FILTER_GPIO_Port GPIOB
#define BEEP_P_Pin GPIO_PIN_14
#define BEEP_P_GPIO_Port GPIOB
#define BEEP_N_Pin GPIO_PIN_15
#define BEEP_N_GPIO_Port GPIOB
#define FAN_H_Pin GPIO_PIN_8
#define FAN_H_GPIO_Port GPIOA
#define FAN_OFF_Pin GPIO_PIN_9
#define FAN_OFF_GPIO_Port GPIOA
#define FILTER_Pin GPIO_PIN_10
#define FILTER_GPIO_Port GPIOA
#define LIGHTS_Pin GPIO_PIN_11
#define LIGHTS_GPIO_Port GPIOA
#define DELAY_Pin GPIO_PIN_12
#define DELAY_GPIO_Port GPIOA
#define LED_PWM_Pin GPIO_PIN_15
#define LED_PWM_GPIO_Port GPIOA
#define LED_OFF_Pin GPIO_PIN_4
#define LED_OFF_GPIO_Port GPIOB
#define OUT1_PP_Pin GPIO_PIN_5
#define OUT1_PP_GPIO_Port GPIOB
#define OUT1_OE_Pin GPIO_PIN_9
#define OUT1_OE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
