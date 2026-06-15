#include "pos_pid.h"
#include <math.h>

typedef struct
{
    float kp;           // 比例系数
    float out_limit;    // 输出限幅
} PosAxisCtrl_t;

static PosAxisCtrl_t pos_x_ctrl;
static PosAxisCtrl_t pos_y_ctrl;

static float target_x_m = 0.0f;
static float target_y_m = 0.0f;


static float finish_tol = 0.02f;     // 完成判定阈值


/* 限幅函数 */
static float PosCtrl_Limit(float value, float limit)
{
    if (value > limit)  return limit;
    if (value < -limit) return -limit;
    return value;
}

/* 初始化全局位置环 */
void PosCtrl_Init(float kp_x, float out_limit_x, float kp_y, float out_limit_y, float finish_tol_m)
{
    pos_x_ctrl.kp = kp_x;
    pos_x_ctrl.out_limit = out_limit_x;  

    pos_y_ctrl.kp = kp_y;
    pos_y_ctrl.out_limit = out_limit_y;

    target_x_m = 0.0f;
    target_y_m = 0.0f;

    finish_tol = finish_tol_m;
}



/* 设置全局目标点 */
void PosCtrl_SetGlobalTarget(float target_x_m_in, float target_y_m_in)
{
    target_x_m = target_x_m_in;
    target_y_m = target_y_m_in;
}


/*
 * 全局位置环更新
 * 输入：
 *   current_x_m     当前全局X
 *   current_y_m     当前全局Y
 *   current_yaw_deg 当前航向角（度）
 *
 * 输出：
 *   车体坐标系速度指令 vx / vy
 */
PosCtrl_Output_t PosCtrl_UpdateGlobal(float current_x_m, float current_y_m, float current_yaw_deg)
{
    PosCtrl_Output_t out;
    float err_x_w, err_y_w;
    float vx_w, vy_w;
    float yaw_rad, cos_yaw, sin_yaw;

    //1. 计算世界坐标误差 
    err_x_w = target_x_m - current_x_m;
    err_y_w = target_y_m - current_y_m;

    //2. 在世界坐标系下做P控制，得到世界系速度
    vx_w = pos_x_ctrl.kp * err_x_w;
    vy_w = pos_y_ctrl.kp * err_y_w;

    //3. 限幅
    vx_w = PosCtrl_Limit(vx_w, pos_x_ctrl.out_limit);
    vy_w = PosCtrl_Limit(vy_w, pos_y_ctrl.out_limit);

    //4. 世界系速度 -> 车体系速度
    yaw_rad = current_yaw_deg * 3.1415926f / 180.0f;
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);

    out.vx =  vx_w * cos_yaw + vy_w * sin_yaw;
    out.vy = -vx_w * sin_yaw + vy_w * cos_yaw;

    return out;
}

uint8_t PosCtrl_IsGlobalFinished(float current_x_m, float current_y_m)
{
    float err_x = target_x_m - current_x_m;
    float err_y = target_y_m - current_y_m;

    if ((fabsf(err_x) < finish_tol) && (fabsf(err_y) < finish_tol))
    {
        return 1;
    }

    return 0;
}
