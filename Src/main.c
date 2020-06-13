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
#include "stdio.h"
#include <stdlib.h>
#include "string.h"
#define OLED_DISPALY_KAlMAN
// #define OLED_DISPLAY_ENCODER
// #define OLED_DISPLAY_PID_PARAMS

#define MAX_RX_BUFFER_SIZE 1024
#define BLINK_PERIOD       128
#define OLED_PERIOD        128
#define SP_PERIOD          128
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
char PWM_OUTPUT_ENABLE = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Print_Params();
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
  #ifdef BW_ANGLE_LOOP_ENABLE
  ssd1306_WriteString(0, 0, "BWP = ", dataFont, White); ssd1306_WriteFloat(70, 0, BottomWheel_Speed_LOOP_PID.Kp, 2);
  ssd1306_WriteString(0, 10, "BWI = ", dataFont, White); ssd1306_WriteFloat(70, 10, BottomWheel_Speed_LOOP_PID.Ki, 2);
  ssd1306_WriteString(0, 20, "BWD = ", dataFont, White); ssd1306_WriteFloat(70, 20, BottomWheel_Speed_LOOP_PID.Kd, 2);
  #endif
  #ifdef IW_ANGLE_LOOP_ENABLE
  ssd1306_WriteString(0, 30, "IWP = ", dataFont, White); ssd1306_WriteFloat(70, 30, InertiaWheel_PID.Kp, 2);
  ssd1306_WriteString(0, 40, "IWI = ", dataFont, White); ssd1306_WriteFloat(70, 40, InertiaWheel_PID.Ki, 2);
  ssd1306_WriteString(0, 50, "IWD = ", dataFont, White); ssd1306_WriteFloat(70, 50, InertiaWheel_PID.Kd, 2);
  #endif
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
  while (MPU6050_Init(&hi2c1) == 1);
  HAL_Delay(500);
  // PWM_Init(0);
  Encoder_Init();
  ssd1306_Init();
  HAL_Delay(500);

  HAL_TIM_Base_Start_IT(&htim2);    // 开启定时器2中断: PID 控制, I2C 读取
  HAL_UART_Receive_IT(&huart2, (uint8_t *) &currRXChar, 1);     // 开启接收中断
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    // uint8_t val = MPU_CNT % MPU_PERIOD;
    // if(val == 0){
    //   MPU6050_Read_All(&hi2c1, &MPU6050);
    // }
    // if(OLED_CNT % 40 == 0){
    //   Update_display();
    // }
    // if(SP_CNT % 40 == 0){
    //   // if(PWM_OUTPUT_ENABLE){
    //     Print_Params();
    //   // }
    // }
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
  static char LOOP;
  if(currRXChar == 'B'){
    Setting_Wheel = BOTTOM_WHEEL;
  }
  else if(currRXChar == 'I'){
    Setting_Wheel = INERTIA_WHEEL;
  }
  else if(currRXChar == 'A'){
    if(Setting_Wheel == BOTTOM_WHEEL){
      LOOP = 'A';   // 设置角度环
    }
  }
  else if(currRXChar == 'S'){
    if(Setting_Wheel == BOTTOM_WHEEL){
      LOOP = 'S';   // 设置角度环
    }
  }
  else if(currRXChar == 'G'){         // GO !
    PWM_OUTPUT_ENABLE = 1;
    PWM_Init(0);
    HAL_UART_Transmit(&huart2, (uint8_t*) "START\n", 6, 200);
  } 
  else if(currRXChar == 'P'){         // PAUSE !
    PWM_OUTPUT_ENABLE = 0;
    PWM_DeInit();
    HAL_UART_Transmit(&huart2, (uint8_t*) "STOP\n", 6, 200);
    PID_TopLevel();   // 清零
  }
  else{
    *(RX_pointer++) = currRXChar;
    if(currRXChar == '*'){
      char Kp[100], Ki[100], Kd[100];
      uint8_t state = 0;
      uint8_t i = 0;
      for(char* p = RX_Buffer + 1; *p != '*'; p++){
        if(*p == ' '){
          i++;
          if(state == 0){
            Kp[i] = '\0';
          }
          if(state == 1){
            Ki[i] = '\0';
          }
          if(state == 2){
            Kd[i] = '\0';
          }
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
      HAL_UART_Transmit(&huart2, (uint8_t*) "OK\n", 4, 200);
      if(Setting_Wheel == BOTTOM_WHEEL){
        Set_BottomWheel_PID_Params(atof(Kp), atof(Ki), atof(Kd), LOOP);
      }
      if(Setting_Wheel == INERTIA_WHEEL){
        Set_InertiaWheel_PID_Params(atof(Kp), atof(Ki), atof(Kd));
      }
      RX_pointer = RX_Buffer;
    }
  }
  HAL_UART_Receive_IT(&huart2, (uint8_t *) &currRXChar, 1);
}

static void SendString(char* str, const char* end){
  HAL_UART_Transmit(&huart2, (uint8_t*) str, strlen(str), 50);
  if(*end){
    HAL_UART_Transmit(&huart2, (uint8_t*) end, 1, 50);
  }
}

static void floatTo2Int(float val, int* ip, int* fp){
	*ip = val;
	val = val - *ip;
	if(val < 0) val = -val;
  val = val*100;
  *fp = val;
}

static void Print_Params(){
  char intStr[100], floatStr[100];
  int ipart, fpart;
  static uint16_t PID_NUM = 1;
  sprintf(intStr, "%d", PID_NUM);
  SendString(intStr, " ");
  sprintf(intStr, "%ld", Read_Encoder_BottomWheel_Count());
  SendString(intStr, " ");
  sprintf(intStr, "%ld", Read_Encoder_InertiaWheel_Count());
  SendString(intStr, " ");

  // sprintf(intStr, "%d", (int) MPU6050.KalmanAngleX);
  // SendString(intStr, " ");
  // sprintf(intStr, "%d", (int) MPU6050.KalmanAngleY);
  // SendString(intStr, "\n");


  floatTo2Int(MPU6050.KalmanAngleX, &ipart, &fpart);
  sprintf(floatStr, "%d.%d", ipart, fpart);
  SendString(floatStr, " ");
  floatTo2Int(MPU6050.KalmanAngleY, &ipart, &fpart);
  sprintf(floatStr, "%d.%d", ipart, fpart);
  SendString(floatStr, "\n");
  PID_NUM++;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)     // TIM2 溢出周期为 1ms
{
  /* Prevent unused argument(s) compilation warning */
  static uint8_t BLINK_CNT = 0;
  static uint8_t BW_ANGLE_PID_CNT = 10;
  // static uint8_t IW_ANGLE_PID_CNT = 2;
  static uint8_t MPU_CNT = 32;
  static uint8_t OLED_CNT = 64;
  static uint8_t SP_CNT = 96;
  UNUSED(htim);
  if(!BLINK_CNT){
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }
  if(!BW_ANGLE_PID_CNT){
    if(PWM_OUTPUT_ENABLE){
      PID_TopLevel();
    }
  }
  if(!MPU_CNT){
    MPU6050_Read_All(&hi2c1, &MPU6050);
  }
  if(!OLED_CNT){
    Update_display();
  }
  if(!SP_CNT){
    // if(PWM_OUTPUT_ENABLE){
      Print_Params();
    // }
  }
  BLINK_CNT = (BLINK_CNT + 1) % BLINK_PERIOD;
  BW_ANGLE_PID_CNT = (BW_ANGLE_PID_CNT + 1) % BW_PID_PERIOD;
  // IW_ANGLE_PID_CNT = (IW_ANGLE_PID_CNT + 1) % IW_PID_PERIOD;
  MPU_CNT = (MPU_CNT + 1) % MPU_PERIOD;
  OLED_CNT = (OLED_CNT + 1) % OLED_PERIOD;
  SP_CNT = (SP_CNT + 1) % SP_PERIOD;
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
// 