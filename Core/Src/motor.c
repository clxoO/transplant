#include "motor.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"
#include "pid.h"

#define TARGET_PULSE_MAX 70

typedef struct
{
    TIM_HandleTypeDef *in1_timer;
    uint32_t in1_channel;
    TIM_HandleTypeDef *in2_timer;
    uint32_t in2_channel;
} MotorConfig_t;

static const MotorConfig_t motor_config[4] =
{
  {&htim10, TIM_CHANNEL_1, &htim11, TIM_CHANNEL_1},//1
  {&htim9,  TIM_CHANNEL_2, &htim9,  TIM_CHANNEL_1},//2
  {&htim1,  TIM_CHANNEL_1, &htim1,  TIM_CHANNEL_2},//3
  {&htim1,  TIM_CHANNEL_4, &htim1,  TIM_CHANNEL_3} //4
};

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

    Motor_Stop();
}

static int16_t Limit(int16_t power)
{
  if (power > 8400)
  {
    return 8400;
  }

  if (power < -8400)
  {
    return -8400;
  }

  return power;
}

static uint32_t PowerToCompare(int16_t power)
{
  return (uint32_t)((power >= 0) ? power : -power);
}

void Motor_SetPower(uint8_t id, int16_t power)
{
  const MotorConfig_t *motor;
  uint32_t compare;

  power = Limit(power);
  motor = &motor_config[id - 1];
  compare = PowerToCompare(power);

  if (power > 0)
  {
    __HAL_TIM_SET_COMPARE(motor->in1_timer, motor->in1_channel, compare);
    __HAL_TIM_SET_COMPARE(motor->in2_timer, motor->in2_channel, 0);
  }
  else if (power < 0)
  {
    __HAL_TIM_SET_COMPARE(motor->in1_timer, motor->in1_channel, 0);
    __HAL_TIM_SET_COMPARE(motor->in2_timer, motor->in2_channel, compare);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(motor->in1_timer, motor->in1_channel, 0);
    __HAL_TIM_SET_COMPARE(motor->in2_timer, motor->in2_channel, 0);
  }
}

void SetVelocity(float vx, float vy, float wz)
{
  float wheel_speed[4];
  uint8_t i;

  wheel_speed[0] = vx + vy + wz;
  wheel_speed[1] = vx - vy - wz;
  wheel_speed[2] = vx - vy + wz;
  wheel_speed[3] = vx + vy - wz;

  for (i = 0; i < 4; i++)
  {
    target_wheel_speed[i] = (int)(wheel_speed[i]);
  }
}

void Motor_Stop(void)
{
  uint8_t i;

  for (i = 0; i < 4; i++)
  {
    Motor_SetPower(i + 1, 0);
  }
}
