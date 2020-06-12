#include "PID.h"

PID InertiaWheel_PID, BottomWheel_PID;

void Set_InertiaWheel_PID(float Kp, float Ki, float Kd){
    InertiaWheel_PID.Kp = Kp;
    InertiaWheel_PID.Ki = Ki;
    InertiaWheel_PID.Kd = Kd;
}

void Set_BottomWheel_PID(float Kp, float Ki, float Kd){
    BottomWheel_PID.Kp = Kp;
    BottomWheel_PID.Ki = Ki;
    BottomWheel_PID.Kd = Kd;
}

void PID_Init(){
    BottomWheel_PID.Kp = 1.555;
    BottomWheel_PID.Ki = 2.333;
    BottomWheel_PID.Kd = 3.124;
    InertiaWheel_PID.Kp = 4.214;
    InertiaWheel_PID.Ki = 5.52414;
    InertiaWheel_PID.Kd = 4.51241;
}