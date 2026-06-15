#include "pos_pid.h"
#include <math.h>

static float traj_v = 0.0f;
static float traj_vmax = 700.0f;
static float traj_acc = 100.0f;   // 每次 Chassis_Update 增加的速度量

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
/*void PosCtrl_SetGlobalTarget(float target_x_m_in, float target_y_m_in)
{
    target_x_m = target_x_m_in;
    target_y_m = target_y_m_in;
}
*/
void PosCtrl_SetGlobalTarget(float target_x_m_in, float target_y_m_in)
{
    target_x_m = target_x_m_in;
    target_y_m = target_y_m_in;
    traj_v = 0.0f;
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
/*PosCtrl_Output_t PosCtrl_UpdateGlobal(float current_x_m, float current_y_m, float current_yaw_deg)
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
*/
PosCtrl_Output_t PosCtrl_UpdateGlobal(float current_x_m, float current_y_m, float current_yaw_deg)
{
    PosCtrl_Output_t out;

    /* 世界坐标系下的位置误差 */
    float err_x_w, err_y_w;

    /* 当前点到目标点的直线距离 */
    float dist;

    /* 从当前位置指向目标点的单位方向向量 */
    float dir_x, dir_y;

    /* v_brake: 根据剩余距离计算出的刹车速度
       v_target: 本周期希望达到的规划速度 */
    float v_brake, v_target;

    /* 世界坐标系下的速度指令 */
    float vx_w, vy_w;

    /* 航向角相关变量，用于世界坐标转车体坐标 */
    float yaw_rad, cos_yaw, sin_yaw;

    /* 1. 计算当前位置到目标点的误差，单位 m */
    err_x_w = target_x_m - current_x_m;
    err_y_w = target_y_m - current_y_m;

    /* 2. 计算当前位置到目标点的直线距离，单位 m */
    dist = sqrtf(err_x_w * err_x_w + err_y_w * err_y_w);

    /* 3. 如果距离已经小于到点阈值，则认为到达目标点，输出速度为 0 */
    if (dist < finish_tol)
    {
        traj_v = 0.0f;
        out.vx = 0.0f;
        out.vy = 0.0f;
        return out;
    }

    /* 4. 计算运动方向。
       dir_x / dir_y 只表示方向，不表示速度大小 */
    dir_x = err_x_w / dist;
    dir_y = err_y_w / dist;

    /* 5. 根据剩余距离计算刹车速度。
       距离越远，允许速度越大；
       距离越近，允许速度越小。
       1500.0f 是比例系数，用来把“米”映射到你当前 SetVelocity 使用的速度量级。 */
    v_brake = sqrtf(2.0f * traj_acc * dist * 1500.0f);
    v_target = v_brake;

    /* 6. 限制最大速度，防止速度规划超过设定上限 */
    if (v_target > traj_vmax)
    {
        v_target = traj_vmax;
    }

    /* 7. 根据目标速度进行加速或减速。
       traj_v 不会突变，而是每个控制周期最多变化 traj_acc。 */
    if (traj_v < v_target)
    {
        traj_v += traj_acc;
        if (traj_v > v_target) traj_v = v_target;
    }
    else
    {
        traj_v -= traj_acc;
        if (traj_v < v_target) traj_v = v_target;
    }

    /* 8. 把规划速度按照目标方向分解到世界坐标 X/Y */
    vx_w = dir_x * traj_v;
    vy_w = dir_y * traj_v;

    /* 9. 当前航向角由角度转弧度 */
    yaw_rad = current_yaw_deg * 3.1415926f / 180.0f;
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);

    /* 10. 世界坐标速度转换到车体坐标速度。
       out.vx 是车体前后方向速度；
       out.vy 是车体左右方向速度。 */
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
