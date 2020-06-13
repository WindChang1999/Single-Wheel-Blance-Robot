#ifndef PID_H
#define PID_H
#include "main.h"


#define BW_PID_PERIOD 10       // unit: ms 执行一次底轮 PID 的周期
#define IW_PID_PERIOD 10       
#define BW_ANGLE_LOOP_ENABLE
// #define BW_SPEED_LOOP_ENABLE
#define IW_ANGLE_LOOP_ENABLE
#define BW_ANGLE_SETPOINT 0
#define IW_ANGLE_SETPOINT 0

typedef struct PID {
    float Kp;
    float Ki;
    float Kd;
} PID;

extern PID InertiaWheel_PID, BottomWheel_Angle_LOOP_PID, BottomWheel_Speed_LOOP_PID;
void Set_InertiaWheel_PID_Params(float Kp, float Ki, float Kd);
void Set_BottomWheel_PID_Params(float Kp, float Ki, float Kd, char LOOP);
void PID_Init();

float PID_InertialWheel_Delta();
void PID_TopLevel();
float PID_BottomWheel_Delta_Speed_LOOP(float Angle_Loop_Output);
float PID_BottomWheel_Delta_Angle_LOOP();
float PID_BottomWheel_Delta();
#endif