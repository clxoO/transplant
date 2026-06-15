#include "chassis.h"
#include "motor.h"
#include "mpu6050.h"
#include "yaw_pid.h"
#include "math.h"
#include "pos_pid.h"
#include "odometry.h"


MPU6050_t mpu6050;
static volatile uint8_t mpu_data_ready = 0;
static I2C_HandleTypeDef *chassis_i2c = NULL;

uint32_t chassis_last_tick = 0;

extern uint8_t sbus_buffer[25];
extern uint8_t sbus_new_cmd;
extern uint16_t rc_channel[16];
extern void SBUS_Parse(uint8_t *buff);

static volatile ChassisMode_t g_chassis_mode = CHASSIS_MODE_REMOTE;
static volatile uint8_t g_move_finished = 1;

void Chassis_Init(I2C_HandleTypeDef *hi2c)
{
    chassis_i2c = hi2c;

    while (MPU6050_Init(chassis_i2c) != 0)
    {
        HAL_Delay(100);
    }

    MPU6050_Read_All(chassis_i2c, &mpu6050);
}

void Chassis_OnImuReady(void)
{
    mpu_data_ready = 1;
}

/* 角度误差归一化到 [-180, 180] */
static float Chassis_AngleError(float current_yaw, float target_yaw)
{
    float err = current_yaw - target_yaw;

    while (err > 180.0f)  err -= 360.0f;
    while (err < -180.0f) err += 360.0f;

    return err;
}

/* 全局点位运动 */
void Chassis_StartMoveWorld(float target_x_m, float target_y_m, float target_yaw)
{
    PosCtrl_SetGlobalTarget(target_x_m, target_y_m);

    YawCtrl_SetTarget(target_yaw);
    YawCtrl_Reset();

    g_move_finished = 0;
    g_chassis_mode = CHASSIS_MODE_POSITION;
}

/* 停止运动 */
void Chassis_StopMove(void)
{
    g_chassis_mode = CHASSIS_MODE_REMOTE;
    g_move_finished = 1;
    SetVelocity(0.0f, 0.0f, 0.0f);
}

/* 查询是否走完 */
uint8_t Chassis_IsMoveFinished(void)
{
    return g_move_finished;
}



void Chassis_Update(void)
{
    if ((mpu_data_ready != 0U) && (chassis_i2c != NULL))
    {
        mpu_data_ready = 0;
        MPU6050_Read_All(chassis_i2c, &mpu6050);
    }
    
    float target_vx = 0.0f;
    float target_vy = 0.0f;
    float target_wz = 0.0f;

    int ch1_offset = 0;
    int ch3_offset = 0;
    int ch4_offset = 0;

    /* 1. 有新的遥控器数据就解析 */
    if (sbus_new_cmd == 1)
    {
        sbus_new_cmd = 0;
        SBUS_Parse(sbus_buffer);
    }

     /* 2. 定位模式 */
    if (g_chassis_mode == CHASSIS_MODE_POSITION)
    {
        PosCtrl_Output_t pos_out;
        float err_yaw;

        /* 位置环输出车体系 vx / vy */
        pos_out = PosCtrl_UpdateGlobal(g_odom.x_m, g_odom.y_m, mpu6050.KalmanAngleZ);
        target_vx = pos_out.vx;
        target_vy = pos_out.vy;

        /* 航向锁头 */
        target_wz = YawCtrl_Update(mpu6050.KalmanAngleZ);

        /* 到点判断 */
        err_yaw = Chassis_AngleError(mpu6050.KalmanAngleZ, YawCtrl_GetTarget());

        if (PosCtrl_IsGlobalFinished(g_odom.x_m, g_odom.y_m) && (fabsf(err_yaw) < 0.3f))
        {
            target_vx = 0.0f;
            target_vy = 0.0f;
            target_wz = 0.0f;

            g_move_finished = 1;
            g_chassis_mode = CHASSIS_MODE_REMOTE;
        }

        SetVelocity(target_vx, target_vy, target_wz);
        return;
    }

    /* 3. 遥控模式 */
    if (rc_channel[0] == 0)
    {
        ch1_offset = 0;
        ch3_offset = 0;
        ch4_offset = 0;

        YawCtrl_SetTarget(mpu6050.KalmanAngleZ);
        YawCtrl_Reset();
    }
    else
    {
        ch3_offset = rc_channel[2] - 992;   // 前后
        ch4_offset = rc_channel[3] - 992;   // 左右
        ch1_offset = rc_channel[0] - 992;   // 自转
    }

    /* 4. 死区 */
    if (ch3_offset > -10 && ch3_offset < 10) ch3_offset = 0;
    if (ch4_offset > -10 && ch4_offset < 10) ch4_offset = 0;
    if (ch1_offset > -10 && ch1_offset < 10) ch1_offset = 0;

    /* 5. 平移控制 */
    target_vx = (ch3_offset / 800.0f) * 700.0f;
    target_vy = (ch4_offset / 800.0f) * 700.0f;

    /* 6. 自转 / 锁头 */
    if (ch1_offset != 0)
    {
        target_wz = (ch1_offset / 800.0f) * 700.0f;

        /* 松杆后从当前角度锁头 */
        YawCtrl_SetTarget(mpu6050.KalmanAngleZ);
        YawCtrl_Reset();
    }
    else
    {
        target_wz = YawCtrl_Update(mpu6050.KalmanAngleZ);
    }

    SetVelocity(target_vx, target_vy, target_wz);

}
