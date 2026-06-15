#ifndef __PID_H
#define __PID_H

#include "main.h"

typedef struct 
{
    float Kp;
    float Ki;  
    int Encoder_S;       
    int IntegralLimit;
    int MaxOutput;
} PID_Controller;
extern PID_Controller motor_pid[4];
extern int target_wheel_speed[4];
extern int actual_speed[4];
void PID_Init(PID_Controller *pid, float kp, float ki, int int_limit, int max_out);
int Velocity(PID_Controller *pid, int target, int actual);
int Control();



#endif /* __PID_H */