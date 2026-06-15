#include "yaw_pid.h"

static YawPID_t yaw_pid;
static float yaw_target = 0.0f;

static float YawCtrl_Limit(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}

static float YawCtrl_GetAngleError(float target_yaw, float current_yaw)
{
    float err = current_yaw - target_yaw;

    while (err > 180.0f)
    {
        err -= 360.0f;
    }

    while (err < -180.0f)
    {
        err += 360.0f;
    }

    return err;
}

void YawCtrl_Init(float kp, float ki, float kd, float i_limit, float out_limit)
{
    yaw_pid.kp = kp;
    yaw_pid.ki = ki;
    yaw_pid.kd = kd;

    yaw_pid.integral = 0.0f;
    yaw_pid.last_error = 0.0f;

    yaw_pid.integral_limit = i_limit;
    yaw_pid.output_limit = out_limit;

    yaw_target = 0.0f;
}

void YawCtrl_SetTarget(float target_yaw)
{
    yaw_target = target_yaw;
}

float YawCtrl_GetTarget(void)
{
    return yaw_target;
}

void YawCtrl_Reset(void)
{
    yaw_pid.integral = 0.0f;
    yaw_pid.last_error = 0.0f;
}

float YawCtrl_Update(float current_yaw)
{
    float error;
    float derivative;
    float output;

    error = YawCtrl_GetAngleError(yaw_target, current_yaw);

    if ((error > -0.2f) && (error < 0.2f))
    {
        error = 0.0f;
        yaw_pid.integral = 0.0f;
        yaw_pid.last_error = 0.0f;
        return 0.0f;
    }

    yaw_pid.integral += error;
    yaw_pid.integral = YawCtrl_Limit(yaw_pid.integral, yaw_pid.integral_limit);

    derivative = error - yaw_pid.last_error;

    output = yaw_pid.kp * error
           + yaw_pid.ki * yaw_pid.integral
           + yaw_pid.kd * derivative;

    output = YawCtrl_Limit(output, yaw_pid.output_limit);
    yaw_pid.last_error = error;

    return output;
}
