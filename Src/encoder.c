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

/* 两次调用这个函数, 可以获得两次调用之间的编码器数量差值 */
int32_t Read_Encoder_InertiaWheel_Count(){
  int32_t res;
  InertiaWheel_EncoderCNT = __HAL_TIM_GET_COUNTER(&INERTIA_WHEEL_TIM_HANDLE);
  InertiaWheel_EncoderDIR = __HAL_TIM_IS_TIM_COUNTING_DOWN(&INERTIA_WHEEL_TIM_HANDLE);
  res = (int32_t) (InertiaWheel_EncoderCNT);
  res = InertiaWheel_EncoderDIR ? res-65536 : res;
  __HAL_TIM_SET_COUNTER(&INERTIA_WHEEL_TIM_HANDLE, 0);
  return res;
}

void Read_Encoder_BottomWheel(){
  BottomWheel_EncoderCNT = __HAL_TIM_GET_COUNTER(&BOTTOM_WHEEL_TIM_HANDLE);
  BottomWheel_EncoderDIR = __HAL_TIM_IS_TIM_COUNTING_DOWN(&BOTTOM_WHEEL_TIM_HANDLE);
}

/* 两次调用这个函数, 可以获得两次调用之间的编码器数量差值 */
int32_t Read_Encoder_BottomWheel_Count(){
  int32_t res;
  BottomWheel_EncoderCNT = __HAL_TIM_GET_COUNTER(&BOTTOM_WHEEL_TIM_HANDLE);
  BottomWheel_EncoderDIR = __HAL_TIM_IS_TIM_COUNTING_DOWN(&BOTTOM_WHEEL_TIM_HANDLE);
  res = (int32_t) (BottomWheel_EncoderCNT);
  res = BottomWheel_EncoderDIR ? res-65536 : res;
  __HAL_TIM_SET_COUNTER(&BOTTOM_WHEEL_TIM_HANDLE, 0);
  return res;
}