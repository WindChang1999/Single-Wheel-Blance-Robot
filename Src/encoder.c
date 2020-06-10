#include "encoder.h"
#include "tim.h"
uint16_t InertiaWheel_EncoderCNT;
uint8_t InertiaWheel_EncoderDIR;
uint16_t BottomWheel_EncoderCNT;
uint8_t BottomWheel_EncoderDIR;

void Encoder_Init(){
  HAL_TIM_Encoder_Start(&INERTIA_WHEEL_TIM_HANDLE, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&BOTTOM_WHEEL_TIM_HANDLE, TIM_CHANNEL_ALL);
}

void Read_Encoder_InertiaWheel(){
  InertiaWheel_EncoderCNT = __HAL_TIM_GET_COUNTER(&INERTIA_WHEEL_TIM_HANDLE);
  InertiaWheel_EncoderDIR = __HAL_TIM_IS_TIM_COUNTING_DOWN(&INERTIA_WHEEL_TIM_HANDLE);
}

void Read_Encoder_BottomWheel(){
  BottomWheel_EncoderCNT = __HAL_TIM_GET_COUNTER(&BOTTOM_WHEEL_TIM_HANDLE);
  BottomWheel_EncoderDIR = __HAL_TIM_IS_TIM_COUNTING_DOWN(&BOTTOM_WHEEL_TIM_HANDLE);
}