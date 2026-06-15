#include "odometry.h"
#include <math.h>

/*
 * WHEEL_RADIUS_M   : 轮子半径，单位 m
 * ENCODER_PPR      : 电机轴每圈对应的编码器计数
 */
#define WHEEL_RADIUS_M     0.04f
#define ENCODER_PPR        60000.0f

#define PI_F               3.1415926f

Odom_t g_odom = {0};

static float EncoderCount_To_DistanceM(int32_t count)
{
    float rev = (float)count / ENCODER_PPR;
    float distance = rev * 2.0f * PI_F * WHEEL_RADIUS_M;
    return distance;
}

void Odom_Reset(void)
{
    g_odom.x_m = 0.0f;
    g_odom.y_m = 0.0f;
    g_odom.forward_m = 0.0f;
    g_odom.lateral_m = 0.0f;
}

void Odom_UpdateFromEncoderDelta(int32_t e1, int32_t e2, int32_t e3, int32_t e4, float yaw_deg)
{
    float d1, d2, d3, d4;
    float ds_forward, ds_lateral;
    float yaw_rad;
    float cos_yaw, sin_yaw;

    // 1. 先把四个轮子的编码器增量换成轮子位移
    d1 = EncoderCount_To_DistanceM(e1);
    d2 = EncoderCount_To_DistanceM(e2);
    d3 = EncoderCount_To_DistanceM(e3);
    d4 = EncoderCount_To_DistanceM(e4);

    // 2. 按你当前麦轮正运动学的反关系，求车体本周期位移
    //    ds_forward：车体前后方向位移增量
    //    ds_lateral：车体左右方向位移增量
    ds_forward = (d1 + d2 + d3 + d4) * 0.25f;
    ds_lateral = (d1 - d2 - d3 + d4) * 0.25f;

    // 3. 车体坐标系累计位移
    g_odom.forward_m += ds_forward;
    g_odom.lateral_m += ds_lateral;

    // 4. 如果需要全局坐标，再根据当前航向角把位移旋转到全局系
    yaw_rad = yaw_deg * PI_F / 180.0f;
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);

    g_odom.x_m += ds_forward * cos_yaw - ds_lateral * sin_yaw;
    g_odom.y_m += ds_forward * sin_yaw + ds_lateral * cos_yaw;
}