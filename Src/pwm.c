#include "pwm.h"
#include "tim.h"

// Bottom Wheel : L298N_ENA, IN1, IN2
// Inertia Wheel : L298N_ENB, IN3, IN4

void PWM_Start(WHEEL wheel, float dutyCycle_Init){
    if(wheel == BOTTOM_WHEEL){
        HAL_TIM_PWM_Start(&PWM_TIMER_HANDLE, PWM_BOTTOM_WHEEL_CHANNEL);
    }
    else{
        HAL_TIM_PWM_Start(&PWM_TIMER_HANDLE, PWM_INERTIA_WHEEL_CHANNEL);
    }
    PWM_SetDutyCycle(wheel, dutyCycle_Init);
}
void PWM_Stop(WHEEL wheel){
    if(wheel == BOTTOM_WHEEL){
        HAL_TIM_PWM_Stop(&PWM_TIMER_HANDLE, PWM_BOTTOM_WHEEL_CHANNEL);
    }
    else{
        HAL_TIM_PWM_Stop(&PWM_TIMER_HANDLE, PWM_INERTIA_WHEEL_CHANNEL);
    }
}

void PWM_Init(float dutyCycle_Init){
    PWM_Start(BOTTOM_WHEEL, dutyCycle_Init);
    PWM_Start(INERTIA_WHEEL, dutyCycle_Init);
}

static void PWM_SetDirection(WHEEL wheel, DIRECTION dir){
    switch (wheel){
    case BOTTOM_WHEEL:
        switch (dir){
        case CW:
            HAL_GPIO_WritePin(L298N_IN1_GPIO_Port, L298N_IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(L298N_IN2_GPIO_Port, L298N_IN2_Pin, GPIO_PIN_RESET);
            break;
        case CCW:
            HAL_GPIO_WritePin(L298N_IN1_GPIO_Port, L298N_IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(L298N_IN2_GPIO_Port, L298N_IN2_Pin, GPIO_PIN_SET);
            break;
        }
        break;
    case INERTIA_WHEEL:
        switch (dir){
        case CW:
            HAL_GPIO_WritePin(L298N_IN3_GPIO_Port, L298N_IN3_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(L298N_IN4_GPIO_Port, L298N_IN4_Pin, GPIO_PIN_RESET);
            break;
        case CCW:
            HAL_GPIO_WritePin(L298N_IN3_GPIO_Port, L298N_IN3_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(L298N_IN4_GPIO_Port, L298N_IN4_Pin, GPIO_PIN_SET);
            break;
        }
        break;
    }
}

/* duty cycle > 0 : CW, duty cycle < 0 : CCW */
void PWM_SetDutyCycle(WHEEL wheel, float dutyCycle){
    if(dutyCycle > PWM_MAX_VALUE){
        dutyCycle = PWM_MAX_VALUE;
    }
    if(dutyCycle < PWM_MIN_VALUE){      // 防止 -128 情况, 无法取负
        dutyCycle = PWM_MIN_VALUE;
    }
    if(dutyCycle > 0){
        PWM_SetDirection(wheel, CW);
    }
    if(dutyCycle <= 0){
        PWM_SetDirection(wheel, CCW);
        dutyCycle = - dutyCycle;
    }
    int16_t pwmval = dutyCycle * PWM_PERIOD / 100;
    switch (wheel){
    case BOTTOM_WHEEL:
        __HAL_TIM_SET_COMPARE(&PWM_TIMER_HANDLE, PWM_BOTTOM_WHEEL_CHANNEL, pwmval);
        break;
    case INERTIA_WHEEL:
        __HAL_TIM_SET_COMPARE(&PWM_TIMER_HANDLE, PWM_INERTIA_WHEEL_CHANNEL, pwmval);
        break;
    }
}
