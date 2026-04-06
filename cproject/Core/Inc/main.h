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
#define LED0_Pin GPIO_PIN_13
#define LED0_GPIO_Port GPIOC
#define KEY_IN_1_Pin GPIO_PIN_0
#define KEY_IN_1_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOA
#define UART2_TX_Pin GPIO_PIN_2
#define UART2_TX_GPIO_Port GPIOA
#define UART2_RX_Pin GPIO_PIN_3
#define UART2_RX_GPIO_Port GPIOA
#define KEY_IN_3_Pin GPIO_PIN_4
#define KEY_IN_3_GPIO_Port GPIOA
#define KEY_IN_4_Pin GPIO_PIN_5
#define KEY_IN_4_GPIO_Port GPIOA
#define PM1_ENCA_Pin GPIO_PIN_6
#define PM1_ENCA_GPIO_Port GPIOA
#define PM1_ENCB_Pin GPIO_PIN_7
#define PM1_ENCB_GPIO_Port GPIOA
#define MI_Pin GPIO_PIN_0
#define MI_GPIO_Port GPIOB
#define GPIO_EXTI1_Pin GPIO_PIN_1
#define GPIO_EXTI1_GPIO_Port GPIOB
#define MPU_SCL_Pin GPIO_PIN_10
#define MPU_SCL_GPIO_Port GPIOB
#define MPU_SDA_Pin GPIO_PIN_11
#define MPU_SDA_GPIO_Port GPIOB
#define L1_Pin GPIO_PIN_12
#define L1_GPIO_Port GPIOB
#define L2_Pin GPIO_PIN_15
#define L2_GPIO_Port GPIOB
#define PM1_AIN1_Pin GPIO_PIN_8
#define PM1_AIN1_GPIO_Port GPIOA
#define PM1_BIN1_Pin GPIO_PIN_9
#define PM1_BIN1_GPIO_Port GPIOA
#define KEY_IN_2_Pin GPIO_PIN_10
#define KEY_IN_2_GPIO_Port GPIOA
#define BEEP_Pin GPIO_PIN_11
#define BEEP_GPIO_Port GPIOA
#define R1_Pin GPIO_PIN_4
#define R1_GPIO_Port GPIOB
#define R2_Pin GPIO_PIN_5
#define R2_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_8
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_9
#define OLED_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
