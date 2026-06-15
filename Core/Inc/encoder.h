#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

void Encoder_Init(void);
int32_t Encoder_GetCount(TIM_HandleTypeDef *htim);



#endif /* __ENCODER_H */