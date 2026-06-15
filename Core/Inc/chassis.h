#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "main.h"
#include "mpu6050.h"

typedef enum
{
    CHASSIS_MODE_REMOTE = 0,   // 遥控模式
    CHASSIS_MODE_POSITION      // 位置模式
} ChassisMode_t;

void Chassis_Update(void);
void Chassis_Init(I2C_HandleTypeDef *hi2c);
void Chassis_OnImuReady(void);
void Chassis_StartMoveWorld(float target_x_m, float target_y_m, float target_yaw);
void Chassis_StopMove(void);
uint8_t Chassis_IsMoveFinished(void);

extern uint32_t chassis_last_tick;
extern MPU6050_t mpu6050;

#endif /* __CHASSIS_H */