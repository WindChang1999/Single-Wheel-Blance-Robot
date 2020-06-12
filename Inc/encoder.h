#ifndef ENCODER_H
#define ENCODER_H
#include "main.h"
#define INERTIA_WHEEL_TIM_HANDLE    htim3
#define BOTTOM_WHEEL_TIM_HANDLE     htim4

extern uint16_t InertiaWheel_EncoderCNT;
extern uint8_t InertiaWheel_EncoderDIR;
extern uint16_t BottomWheel_EncoderCNT;
extern uint8_t BottomWheel_EncoderDIR;
extern uint16_t InertiaWheel_EncoderLastCNT;
extern uint16_t BottomWheel_EncoderLastCNT;

void Encoder_Init();
void Read_Encoder_InertiaWheel();
void Read_Encoder_BottomWheel();
/* 两次调用这个函数, 可以获得两次调用之间的编码器数量差值 */
int32_t Read_Encoder_BottomWheel_Count();
/* 两次调用这个函数, 可以获得两次调用之间的编码器数量差值 */
int32_t Read_Encoder_InertiaWheel_Count();

#endif
