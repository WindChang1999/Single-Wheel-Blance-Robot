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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OLED_DC_Pin GPIO_PIN_0
#define OLED_DC_GPIO_Port GPIOC
#define OLED_RST_Pin GPIO_PIN_1
#define OLED_RST_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define EncoderA_InertiaWheel_Pin GPIO_PIN_6
#define EncoderA_InertiaWheel_GPIO_Port GPIOA
#define EncoderB_InertiaWheel_Pin GPIO_PIN_7
#define EncoderB_InertiaWheel_GPIO_Port GPIOA
#define OLED_SCL_Pin GPIO_PIN_10
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_11
#define OLED_SDA_GPIO_Port GPIOB
#define L298N_IN2_Pin GPIO_PIN_8
#define L298N_IN2_GPIO_Port GPIOC
#define L298N_IN1_Pin GPIO_PIN_9
#define L298N_IN1_GPIO_Port GPIOC
#define L298N_ENA_Pin GPIO_PIN_8
#define L298N_ENA_GPIO_Port GPIOA
#define L298N_ENB_Pin GPIO_PIN_9
#define L298N_ENB_GPIO_Port GPIOA
#define L298N_IN3_Pin GPIO_PIN_10
#define L298N_IN3_GPIO_Port GPIOC
#define L298N_IN4_Pin GPIO_PIN_11
#define L298N_IN4_GPIO_Port GPIOC
#define EncoderA_BottomWheel_Pin GPIO_PIN_6
#define EncoderA_BottomWheel_GPIO_Port GPIOB
#define EncoderB_BottomWheel_Pin GPIO_PIN_7
#define EncoderB_BottomWheel_GPIO_Port GPIOB
#define MPU_SCL_Pin GPIO_PIN_8
#define MPU_SCL_GPIO_Port GPIOB
#define MPU_SDA_Pin GPIO_PIN_9
#define MPU_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
