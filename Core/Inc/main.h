/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Motor_1A_IN1_Pin GPIO_PIN_0
#define Motor_1A_IN1_GPIO_Port GPIOA
#define Motor_1A_IN2_Pin GPIO_PIN_1
#define Motor_1A_IN2_GPIO_Port GPIOA
#define USART2_RX_SBUS_RX_Pin GPIO_PIN_3
#define USART2_RX_SBUS_RX_GPIO_Port GPIOA
#define INT_for_CLK___SNP00128_Pin GPIO_PIN_4
#define INT_for_CLK___SNP00128_GPIO_Port GPIOA
#define INT_for_CLK___SNP00128_EXTI_IRQn EXTI4_IRQn
#define for_DT___SNP00128_Pin GPIO_PIN_5
#define for_DT___SNP00128_GPIO_Port GPIOA
#define INT_for_SW___SNP00128_Pin GPIO_PIN_6
#define INT_for_SW___SNP00128_GPIO_Port GPIOA
#define INT_for_SW___SNP00128_EXTI_IRQn EXTI9_5_IRQn
#define Motor_2B_PWM_Pin GPIO_PIN_7
#define Motor_2B_PWM_GPIO_Port GPIOA
#define Motor_2A_IN1_Pin GPIO_PIN_0
#define Motor_2A_IN1_GPIO_Port GPIOB
#define Motor_2A_IN2_Pin GPIO_PIN_1
#define Motor_2A_IN2_GPIO_Port GPIOB
#define Motor_1B_IN2_Pin GPIO_PIN_10
#define Motor_1B_IN2_GPIO_Port GPIOB
#define TB6612__1_STBY_Pin GPIO_PIN_12
#define TB6612__1_STBY_GPIO_Port GPIOB
#define TB6612__2_STBY_Pin GPIO_PIN_13
#define TB6612__2_STBY_GPIO_Port GPIOB
#define Motor_2B_IN1_Pin GPIO_PIN_14
#define Motor_2B_IN1_GPIO_Port GPIOB
#define Motor_2B_IN2_Pin GPIO_PIN_15
#define Motor_2B_IN2_GPIO_Port GPIOB
#define Motor_1A_PWM_Pin GPIO_PIN_8
#define Motor_1A_PWM_GPIO_Port GPIOA
#define Motor_1B_PWM_Pin GPIO_PIN_9
#define Motor_1B_PWM_GPIO_Port GPIOA
#define Motor_2A_PWM_Pin GPIO_PIN_11
#define Motor_2A_PWM_GPIO_Port GPIOA
#define Motor_1B_IN1_Pin GPIO_PIN_5
#define Motor_1B_IN1_GPIO_Port GPIOB
#define INT_for_BNO085_Pin GPIO_PIN_8
#define INT_for_BNO085_GPIO_Port GPIOB
#define INT_for_BNO085_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */
#define BNO085_ADDR  0x4A << 1   // Or 0x4B << 1 if ADDR = 3.3V
#define APP_VERSION "v1.0.0_SBUS"
#define BNO085_BOOT_DELAY_MS 500
#define OLED_INFO_DELAY_MS 1000

#define ENC_CLK_Pin         GPIO_PIN_4
#define ENC_CLK_GPIO_Port   GPIOA
#define ENC_DT_Pin          GPIO_PIN_5
#define ENC_DT_GPIO_Port    GPIOA
#define ENC_SW_Pin          GPIO_PIN_6
#define ENC_SW_GPIO_Port    GPIOA
#define ENC_CLK_EXTI_IRQn   EXTI4_IRQn
#define ENC_SW_EXTI_IRQn    EXTI9_5_IRQn
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
