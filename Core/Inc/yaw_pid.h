#ifndef __YAW_PID_H
#define __YAW_PID_H

#include "main.h"

typedef struct
{
    float kp;
    float ki;
    float kd;

    float integral;
    float last_error;

    float integral_limit;
    float output_limit;
} YawPID_t;

void YawCtrl_Init(float kp, float ki, float kd, float i_limit, float out_limit);
void YawCtrl_SetTarget(float target_yaw);
float YawCtrl_GetTarget(void);
void YawCtrl_Reset(void);
float YawCtrl_Update(float current_yaw);

#endif /* __YAW_PID_H */
