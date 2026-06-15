#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include "main.h"

typedef struct
{
    volatile float x_m;        // 全局坐标X，可用于后续扩展
    volatile float y_m;        // 全局坐标Y，可用于后续扩展

    volatile float forward_m;  // 车体坐标系前后累计位移
    volatile float lateral_m;  // 车体坐标系左右累计位移
} Odom_t;

extern Odom_t g_odom;

void Odom_Reset(void);
void Odom_UpdateFromEncoderDelta(int32_t e1, int32_t e2, int32_t e3, int32_t e4, float yaw_deg);

#endif /* __ODOMETRY_H */