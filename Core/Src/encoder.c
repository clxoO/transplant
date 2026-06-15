#include "encoder.h"
#include "tim.h"

void Encoder_Init(void)
{
  /* 启动定时器，开始计数 */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  /* 上电时，把所有定时器的计数值清零，防止一开机就有个随机乱码速度 */
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  __HAL_TIM_SET_COUNTER(&htim4, 0);
  __HAL_TIM_SET_COUNTER(&htim5, 0);
}


int32_t Encoder_GetCount(TIM_HandleTypeDef *htim)
{
  int32_t count;
  /* 1. 直接读取定时器的计数寄存器 */
  count = (int16_t)__HAL_TIM_GET_COUNTER(htim);
  /* 2. 读完就清零，下一次再读就是增量值 */
  __HAL_TIM_SET_COUNTER(htim, 0); 

  return count;
}
