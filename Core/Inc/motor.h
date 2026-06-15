#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

void Motor_Init(void);
void Motor_SetPower(uint8_t id, int16_t power);
void SetVelocity(float vx, float vy, float wz);
void Motor_Stop(void);

#endif /* __MOTOR_H */