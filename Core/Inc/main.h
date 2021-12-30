/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#define LORA_DIO2_Pin GPIO_PIN_1
#define LORA_DIO2_GPIO_Port GPIOA
#define LORA_DIO4_Pin GPIO_PIN_2
#define LORA_DIO4_GPIO_Port GPIOA
#define LORA_DIO5_Pin GPIO_PIN_3
#define LORA_DIO5_GPIO_Port GPIOA
#define LORA_NSS_Pin GPIO_PIN_4
#define LORA_NSS_GPIO_Port GPIOA
#define LORA_DIO1_Pin GPIO_PIN_2
#define LORA_DIO1_GPIO_Port GPIOB
#define LORA_DIO1_EXTI_IRQn EXTI2_IRQn
#define LORA_DIO0_Pin GPIO_PIN_10
#define LORA_DIO0_GPIO_Port GPIOB
#define LORA_DIO0_EXTI_IRQn EXTI15_10_IRQn
#define LORA_RESET_Pin GPIO_PIN_11
#define LORA_RESET_GPIO_Port GPIOB
#define LORA_DIO3_Pin GPIO_PIN_12
#define LORA_DIO3_GPIO_Port GPIOB
#define BUTTON_Pin GPIO_PIN_13
#define BUTTON_GPIO_Port GPIOB
#define BUTTON_EXTI_IRQn EXTI15_10_IRQn
#define LED3_PIN_Pin GPIO_PIN_8
#define LED3_PIN_GPIO_Port GPIOA
#define LED2_PIN_Pin GPIO_PIN_9
#define LED2_PIN_GPIO_Port GPIOA
#define LED1_PIN_Pin GPIO_PIN_10
#define LED1_PIN_GPIO_Port GPIOA
#define RELAY4_PIN_Pin GPIO_PIN_3
#define RELAY4_PIN_GPIO_Port GPIOB
#define RELAY3_PIN_Pin GPIO_PIN_4
#define RELAY3_PIN_GPIO_Port GPIOB
#define RELAY2_PIN_Pin GPIO_PIN_5
#define RELAY2_PIN_GPIO_Port GPIOB
#define RELAY1_PIN_Pin GPIO_PIN_6
#define RELAY1_PIN_GPIO_Port GPIOB
#define RELAY5_PIN_Pin GPIO_PIN_8
#define RELAY5_PIN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

//#define USE_STARTING_PWM_SIGNAL 1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
