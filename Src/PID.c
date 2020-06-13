#include "PID.h"
#include "pwm.h"
#include "encoder.h"
#include "mpu6050.h"

PID InertiaWheel_PID = {10, 0, 0}, BottomWheel_Angle_LOOP_PID = {3.2, 0, 0}, BottomWheel_Speed_LOOP_PID = 
{1, 0. , 0};
extern MPU6050_t MPU6050;
extern char PWM_OUTPUT_ENABLE;
void Set_InertiaWheel_PID_Params(float Kp, float Ki, float Kd){
    InertiaWheel_PID.Kp = Kp;
    InertiaWheel_PID.Ki = Ki;
    InertiaWheel_PID.Kd = Kd;
}

void Set_BottomWheel_PID_Params(float Kp, float Ki, float Kd, char LOOP){
    if(LOOP == 'A'){
        BottomWheel_Angle_LOOP_PID.Kp = Kp;
        BottomWheel_Angle_LOOP_PID.Ki = Ki;
        BottomWheel_Angle_LOOP_PID.Kd = Kd;
    }
    if(LOOP == 'S'){
        BottomWheel_Speed_LOOP_PID.Kp = Kp;
        BottomWheel_Speed_LOOP_PID.Ki = Ki;
        BottomWheel_Speed_LOOP_PID.Kd = Kd;        
    }
}

float PID_InertialWheel_Delta(){
    float P_term, I_term, D_term;
    static double Angle_IW;
    static double error, lerror, llerror;
    Angle_IW = MPU6050.KalmanAngleY;
    llerror = lerror;
    lerror = error;
    error = Angle_IW - IW_ANGLE_SETPOINT;
    P_term = InertiaWheel_PID.Kp * (error - lerror);
    I_term = InertiaWheel_PID.Ki * (error);
    D_term = InertiaWheel_PID.Kd * (error + llerror - lerror - lerror);
    return (P_term + I_term + D_term);
}


float PID_BottomWheel_Delta(){
    float Angle_Loop_Output = PID_BottomWheel_Delta_Angle_LOOP();
#ifdef BW_SPEED_LOOP_ENABLE
    return PID_BottomWheel_Delta_Speed_LOOP(Angle_Loop_Output);
#else
    return Angle_Loop_Output;
#endif
}

float PID_BottomWheel_Delta_Angle_LOOP(){
    float P_term, I_term, D_term;
    static double Angle_BW;
    static double error, lerror, llerror;
    Angle_BW = MPU6050.KalmanAngleX;
    llerror = lerror;
    lerror = error;
    error = BW_ANGLE_SETPOINT - Angle_BW;
    P_term = BottomWheel_Angle_LOOP_PID.Kp * (error - lerror);
    I_term = BottomWheel_Angle_LOOP_PID.Ki * (error);
    D_term = BottomWheel_Angle_LOOP_PID.Kd * (error + llerror - lerror - lerror);
    return (P_term + I_term + D_term);
}

float PID_BottomWheel_Delta_Speed_LOOP(float Angle_Loop_Output){
    float P_term, I_term, D_term;
    static int32_t ENC_CNT;
    static int error, lerror, llerror;
    ENC_CNT = Read_Encoder_BottomWheel_Count();
    llerror = lerror;
    lerror = error;
    error = Angle_Loop_Output - ENC_CNT;
    P_term = BottomWheel_Speed_LOOP_PID.Kp * (error - lerror);
    I_term = BottomWheel_Speed_LOOP_PID.Ki * (error);
    D_term = BottomWheel_Speed_LOOP_PID.Kd * (error + llerror - lerror - lerror);
    return (P_term + I_term + D_term);
}

static float Limit_PWM(float val){
    if(val > PWM_MAX_VALUE){
        return PWM_MAX_VALUE;
    }
    if(val < PWM_MIN_VALUE){
        return PWM_MIN_VALUE;
    }
    return val;
}

void PID_TopLevel(){
    static float BW_PWM_DC = 0, IW_PWM_DC = 0;      // 两轮对应的 PWM 占空比
    if(!PWM_OUTPUT_ENABLE){
        BW_PWM_DC = 0;
        IW_PWM_DC = 0;
        PWM_SetDutyCycle(BOTTOM_WHEEL, BW_PWM_DC);
        PWM_SetDutyCycle(INERTIA_WHEEL, IW_PWM_DC);
        return;
    }
#ifdef BW_ANGLE_LOOP_ENABLE
    BW_PWM_DC += PID_BottomWheel_Delta();
    BW_PWM_DC = Limit_PWM(BW_PWM_DC);
    PWM_SetDutyCycle(BOTTOM_WHEEL, BW_PWM_DC);
#endif
#ifdef IW_ANGLE_LOOP_ENABLE
    IW_PWM_DC += PID_InertialWheel_Delta();
    IW_PWM_DC = Limit_PWM(IW_PWM_DC);
    PWM_SetDutyCycle(INERTIA_WHEEL, IW_PWM_DC);
#endif
}

    