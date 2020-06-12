/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "ssd1306.h"
#include "encoder.h"
#include "pwm.h"
#include "PID.h"
#include <stdlib.h>
#define OLED_DEBUG
// #define OLED_DISPALY_KAlMAN
// #define OLED_DISPLAY_ENCODER
#define OLED_DISPLAY_PID_PARAMS

#define MAX_RX_BUFFER_SIZE 1024
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050_t MPU6050;
extern PID InertiaWheel_PID, BottomWheel_PID;
char RX_Buffer[MAX_RX_BUFFER_SIZE];
char* RX_pointer = RX_Buffer;
char currRXChar;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define dataFont Font_7x10
void Update_display(){
  #ifdef OLED_DISPALY_KAlMAN
  /* Displat MPU6050 */
  ssd1306_WriteString(0, 0, "KX = ", dataFont, White);
  ssd1306_WriteFloat(50, 0, MPU6050.KalmanAngleX, 2);
  ssd1306_WriteString(0, 10, "KY = ", dataFont, White);
  ssd1306_WriteFloat(50, 10, MPU6050.KalmanAngleY, 2);
  #endif
  #ifdef OLED_DISPLAY_ENCODER
  /* Display encoder */
  ssd1306_WriteString(0, 20, "BEC = ", dataFont, White);
  ssd1306_WriteInt(50, 20, Read_Encoder_BottomWheel_Count(), 5);
  ssd1306_WriteString(0, 30, "IEC = ", dataFont, White);
  ssd1306_WriteInt(50, 30, Read_Encoder_InertiaWheel_Count(), 5);
  #endif
  #ifdef OLED_DISPLAY_PID_PARAMS
  ssd1306_WriteString(0, 0, "BWP = ", dataFont, White); ssd1306_WriteFloat(70, 0, BottomWheel_PID.Kp, 2);
  ssd1306_WriteString(0, 10, "BWI = ", dataFont, White); ssd1306_WriteFloat(70, 10, BottomWheel_PID.Ki, 2);
  ssd1306_WriteString(0, 20, "BWD = ", dataFont, White); ssd1306_WriteFloat(70, 20, BottomWheel_PID.Kd, 2);
  ssd1306_WriteString(0, 30, "IWP = ", dataFont, White); ssd1306_WriteFloat(70, 30, InertiaWheel_PID.Kp, 2);
  ssd1306_WriteString(0, 40, "IWI = ", dataFont, White); ssd1306_WriteFloat(70, 40, InertiaWheel_PID.Ki, 2);
  ssd1306_WriteString(0, 50, "IWD = ", dataFont, White); ssd1306_WriteFloat(70, 50, InertiaWheel_PID.Kd, 2);
  #endif
  ssd1306_UpdateScreen();
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  Encoder_Init();
  PWM_Init(80.5);
  ssd1306_Init();
  PID_Init();
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_IT(&huart2, (uint8_t *) &currRXChar, 1);     // 开启接收中断
  while (MPU6050_Init(&hi2c1) == 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    #ifdef OLED_DEBUG

    #endif
    /* USER CODE END WHILE */
    MPU6050_Read_All(&hi2c1, &MPU6050);
    Update_display();
    HAL_Delay(100);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  static WHEEL Setting_Wheel;
  if(currRXChar == 'B'){
    Setting_Wheel = BOTTOM_WHEEL;
  }
  else if(currRXChar == 'I'){
    Setting_Wheel = INERTIA_WHEEL;
  }
  else{
    *(RX_pointer++) = currRXChar;
    if(currRXChar == '*'){
      char Kp[1024], Ki[1024], Kd[1024];
      uint8_t state = 0;
      uint8_t i = 0;
      for(char* p = RX_Buffer + 1; *p != '*'; p++){
        if(*p == ' '){
          i = 0;
          state++;
          continue;
        }
        if(state == 0){
          Kp[i++] = *p;
        }
        if(state == 1){
          Ki[i++] = *p;
        }
        if(state == 2){
          Kd[i++] = *p;
        }
      }
      HAL_UART_Transmit(&huart2, (uint8_t*) "OK", 3, 200);
      if(Setting_Wheel == BOTTOM_WHEEL){
        // sscanf(RX_Buffer, "%f %f %f", &BottomWheel_PID.Kp, &BottomWheel_PID.Ki, &BottomWheel_PID.Kd);
        BottomWheel_PID.Kp = atof(Kp);
        BottomWheel_PID.Ki = atof(Ki);
        BottomWheel_PID.Kd = atof(Kd);
      }
      if(Setting_Wheel == INERTIA_WHEEL){
        // sscanf(RX_Buffer, "%f %f %f", &InertiaWheel_PID.Kp, &InertiaWheel_PID.Ki, &InertiaWheel_PID.Kd);
        InertiaWheel_PID.Kp = atof(Kp);
        InertiaWheel_PID.Ki = atof(Ki);
        InertiaWheel_PID.Kd = atof(Kd);
      }
      RX_pointer = RX_Buffer;
    }
  }
  HAL_UART_Receive_IT(&huart2, (uint8_t *) &currRXChar, 1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  static uint16_t BlinkCount = 0;
  if(BlinkCount == 100){
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    BlinkCount = 0;
  }
  BlinkCount++;
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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
