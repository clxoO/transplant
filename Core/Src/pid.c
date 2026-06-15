#include "pid.h"
#include "encoder.h"
#include "tim.h"
#include "motor.h"
#include "odometry.h"
#include "mpu6050.h"
#include "chassis.h"



PID_Controller motor_pid[4];
TIM_HandleTypeDef* encoder_timers[4] = {&htim2, &htim3, &htim4, &htim5};

int actual_speed[4] = {0,0,0,0};
int target_wheel_speed[4] = {0,0,0,0};


void PID_Init(PID_Controller *pid, float kp, float ki, int int_limit, int max_out)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->IntegralLimit = int_limit; // 在这里定义它的大小！
    pid->MaxOutput = max_out;       // 输出限幅也在这里定义
    
    // 初始化时，把历史数据清零，防止开机乱跳
    pid->Encoder_S = 0;
}

int Velocity(PID_Controller *pid, int target, int actual)
{
     int Err, Out;
     // 1. 计算当前偏差值
     Err = target - actual;  
     // 2. 积分累加
     pid->Encoder_S += Err;
     // 3. 积分限幅
    if(pid->Encoder_S > pid->IntegralLimit)
    {
        pid->Encoder_S = pid->IntegralLimit;
    }
    else if(pid->Encoder_S < -pid->IntegralLimit)
    {
        pid->Encoder_S = -pid->IntegralLimit;
    }
     // 4. 速度环PI计算
    Out = (pid->Kp * Err) + (pid->Ki * pid->Encoder_S);
     // 5. 输出限幅
    if(Out > pid->MaxOutput)
    {
        Out = pid->MaxOutput;
    }
    else if(Out < -pid->MaxOutput)
    {
        Out = -pid->MaxOutput;
    }

    return Out;

}

int Control()
{
    static const int encoder_sign[4] = {1, -1, 1, -1};
    int Pwm_Out = 0;
    int32_t delta[4];
    int i;

    for (i = 0; i < 4; i++)
    {
        delta[i] = encoder_sign[i] * Encoder_GetCount(encoder_timers[i]);
        actual_speed[i] = delta[i]; 
        Pwm_Out = Velocity(&motor_pid[i], target_wheel_speed[i], actual_speed[i]);
        Motor_SetPower(i + 1, Pwm_Out);
    }

    Odom_UpdateFromEncoderDelta(delta[0], delta[1], delta[2], delta[3], mpu6050.KalmanAngleZ);

    return 0;
}
