/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define encl1_Pin GPIO_PIN_0
#define encl1_GPIO_Port GPIOA
#define encl2_Pin GPIO_PIN_1
#define encl2_GPIO_Port GPIOA
#define sleepl_Pin GPIO_PIN_4
#define sleepl_GPIO_Port GPIOA
#define dirl_Pin GPIO_PIN_5
#define dirl_GPIO_Port GPIOA
#define pwml_Pin GPIO_PIN_6
#define pwml_GPIO_Port GPIOA
#define pwmr_Pin GPIO_PIN_7
#define pwmr_GPIO_Port GPIOA
#define dirr_Pin GPIO_PIN_4
#define dirr_GPIO_Port GPIOC
#define sleepr_Pin GPIO_PIN_5
#define sleepr_GPIO_Port GPIOC
#define encr1_Pin GPIO_PIN_9
#define encr1_GPIO_Port GPIOE
#define encr2_Pin GPIO_PIN_11
#define encr2_GPIO_Port GPIOE
#define led1_Pin GPIO_PIN_12
#define led1_GPIO_Port GPIOB
#define led2_Pin GPIO_PIN_13
#define led2_GPIO_Port GPIOB
#define led3_Pin GPIO_PIN_14
#define led3_GPIO_Port GPIOB
#define led4_Pin GPIO_PIN_15
#define led4_GPIO_Port GPIOB
#define camtx_Pin GPIO_PIN_8
#define camtx_GPIO_Port GPIOD
#define camrx_Pin GPIO_PIN_9
#define camrx_GPIO_Port GPIOD
#define gryotx_Pin GPIO_PIN_6
#define gryotx_GPIO_Port GPIOC
#define gryorx_Pin GPIO_PIN_7
#define gryorx_GPIO_Port GPIOC
#define bttx_Pin GPIO_PIN_9
#define bttx_GPIO_Port GPIOA
#define btrx_Pin GPIO_PIN_10
#define btrx_GPIO_Port GPIOA
#define buzzer_Pin GPIO_PIN_11
#define buzzer_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
