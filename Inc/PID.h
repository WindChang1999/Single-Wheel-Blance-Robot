#ifndef PID_H
#define PID_H

typedef struct PID {
    float Kp;
    float Ki;
    float Kd;
} PID;

extern PID InertiaWheel_PID, BottomWheel_PID;
void Set_InertiaWheel_PID(float Kp, float Ki, float Kd);
void Set_BottomWheel_PID(float Kp, float Ki, float Kd);
void PID_Init();
#endif