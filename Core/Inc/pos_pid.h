#ifndef __POS_PID_H
#define __POS_PID_H

#include "main.h"

typedef struct
{
    float vx;
    float vy;
} PosCtrl_Output_t;

/* 初始化全局位置环 */
void PosCtrl_Init(float kp_x, float out_limit_x, float kp_y, float out_limit_y, float finish_tol_m);

/* 设置全局目标点 */
void PosCtrl_SetGlobalTarget(float target_x_m, float target_y_m);

/* 根据当前全局位置和当前航向角，输出车体坐标系速度 */
PosCtrl_Output_t PosCtrl_UpdateGlobal(float current_x_m, float current_y_m, float current_yaw_deg);

/* 判断是否到达全局目标点 */
uint8_t PosCtrl_IsGlobalFinished(float current_x_m, float current_y_m);

#endif /* __POS_PID_H */