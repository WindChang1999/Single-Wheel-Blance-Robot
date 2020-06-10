#ifndef PWM_H
#define PWM_H
#include "main.h"
// Bottom Wheel : L298N_ENA, IN1, IN2
// Inertia Wheel : L298N_ENB, IN3, IN4

#define PWM_TIMER_HANDLE            htim1
#define PWM_BOTTOM_WHEEL_CHANNEL    TIM_CHANNEL_1
#define PWM_INERTIA_WHEEL_CHANNEL   TIM_CHANNEL_2
#define PWM_MAX_VALUE               100
#define PWM_MIN_VALUE               -100
#define PWM_PERIOD                  12800
#define PWM_CLOCK                   64000000

typedef enum { CW, CCW } DIRECTION;
typedef enum { BOTTOM_WHEEL, INERTIA_WHEEL } WHEEL;

void PWM_Start(WHEEL wheel, int8_t dutyCycle_Init);
void PWM_Stop(WHEEL wheel);
void PWM_Init();
/* duty cycle > 0 : CW, duty cycle < 0 : CCW */
void PWM_SetDutyCycle(WHEEL wheel, int8_t dutyCycle);
#endif