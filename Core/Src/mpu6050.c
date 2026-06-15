#include "mpu6050.h"

#include <math.h>

#define RAD_TO_DEG              57.2957795130823208768
#define WHO_AM_I_REG            0x75
#define PWR_MGMT_1_REG          0x6B
#define SMPLRT_DIV_REG          0x19
#define ACCEL_CONFIG_REG        0x1C
#define ACCEL_XOUT_H_REG        0x3B
#define TEMP_OUT_H_REG          0x41
#define GYRO_CONFIG_REG         0x1B
#define GYRO_XOUT_H_REG         0x43
#define INT_PIN_CFG_REG         0x37
#define INT_ENABLE_REG          0x38
#define MPU6050_ADDR            (0x68 << 1)
#define MPU6050_I2C_TIMEOUT     100U
#define ACCEL_Z_CORRECTOR       14418.0
#define MPU6050_GZ_BIAS         -1.38

static uint32_t mpu6050_timer = 0;
static double yaw_angle = 0.0;

static Kalman_t KalmanX = {
    .Q_angle = 0.001,
    .Q_bias = 0.003,
    .R_measure = 0.03
};

static Kalman_t KalmanY = {
    .Q_angle = 0.001,
    .Q_bias = 0.003,
    .R_measure = 0.03
};

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check = 0;
    uint8_t data = 0;

    if (HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, I2C_MEMADD_SIZE_8BIT,
                         &check, 1, MPU6050_I2C_TIMEOUT) != HAL_OK) {
        return 1;
    }

    if (check != 0x68) {
        return 1;
    }

    if (HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, I2C_MEMADD_SIZE_8BIT,
                          &data, 1, MPU6050_I2C_TIMEOUT) != HAL_OK) {
        return 1;
    }

    data = 0x07;
    if (HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, I2C_MEMADD_SIZE_8BIT,
                          &data, 1, MPU6050_I2C_TIMEOUT) != HAL_OK) {
        return 1;
    }

    data = 0x00;
    if (HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, I2C_MEMADD_SIZE_8BIT,
                          &data, 1, MPU6050_I2C_TIMEOUT) != HAL_OK) {
        return 1;
    }

    if (HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, I2C_MEMADD_SIZE_8BIT,
                          &data, 1, MPU6050_I2C_TIMEOUT) != HAL_OK) {
        return 1;
    }

    /* Active-high pulse on INT pin, suitable for EXTI rising-edge trigger. */
    data = 0x00;
    if (HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, INT_PIN_CFG_REG, I2C_MEMADD_SIZE_8BIT,
                          &data, 1, MPU6050_I2C_TIMEOUT) != HAL_OK) {
        return 1;
    }

    /* Enable Data Ready interrupt. */
    data = 0x01;
    if (HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, INT_ENABLE_REG, I2C_MEMADD_SIZE_8BIT,
                          &data, 1, MPU6050_I2C_TIMEOUT) != HAL_OK) {
        return 1;
    }

    mpu6050_timer = HAL_GetTick();
    yaw_angle = 0.0;
    KalmanX.angle = 0.0;
    KalmanX.bias = 0.0;
    KalmanX.P[0][0] = 0.0;
    KalmanX.P[0][1] = 0.0;
    KalmanX.P[1][0] = 0.0;
    KalmanX.P[1][1] = 0.0;
    KalmanY.angle = 0.0;
    KalmanY.bias = 0.0;
    KalmanY.P[0][0] = 0.0;
    KalmanY.P[0][1] = 0.0;
    KalmanY.P[1][0] = 0.0;
    KalmanY.P[1][1] = 0.0;

    return 0;
}

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t rec_data[6];

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, I2C_MEMADD_SIZE_8BIT,
                     rec_data, 6, MPU6050_I2C_TIMEOUT);

    DataStruct->Accel_X_RAW = (int16_t)(rec_data[0] << 8 | rec_data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(rec_data[2] << 8 | rec_data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(rec_data[4] << 8 | rec_data[5]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / ACCEL_Z_CORRECTOR;
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t rec_data[6];

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, I2C_MEMADD_SIZE_8BIT,
                     rec_data, 6, MPU6050_I2C_TIMEOUT);

    DataStruct->Gyro_X_RAW = (int16_t)(rec_data[0] << 8 | rec_data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(rec_data[2] << 8 | rec_data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(rec_data[4] << 8 | rec_data[5]);

    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t rec_data[2];
    int16_t temp = 0;

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, I2C_MEMADD_SIZE_8BIT,
                     rec_data, 2, MPU6050_I2C_TIMEOUT);

    temp = (int16_t)(rec_data[0] << 8 | rec_data[1]);
    DataStruct->Temperature = ((float)temp / 340.0f) + 36.53f;
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate;
    double s;
    double k[2];
    double gap;
    double p00_temp;
    double p01_temp;

    rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    s = Kalman->P[0][0] + Kalman->R_measure;
    k[0] = Kalman->P[0][0] / s;
    k[1] = Kalman->P[1][0] / s;

    gap = newAngle - Kalman->angle;
    Kalman->angle += k[0] * gap;
    Kalman->bias += k[1] * gap;

    p00_temp = Kalman->P[0][0];
    p01_temp = Kalman->P[0][1];
    Kalman->P[0][0] -= k[0] * p00_temp;
    Kalman->P[0][1] -= k[0] * p01_temp;
    Kalman->P[1][0] -= k[1] * p00_temp;
    Kalman->P[1][1] -= k[1] * p01_temp;

    return Kalman->angle;
}

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t rec_data[14];
    int16_t temp;
    uint32_t now;
    double dt;
    double roll;
    double roll_sqrt;
    double pitch;

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, I2C_MEMADD_SIZE_8BIT,
                     rec_data, 14, MPU6050_I2C_TIMEOUT);

    DataStruct->Accel_X_RAW = (int16_t)(rec_data[0] << 8 | rec_data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(rec_data[2] << 8 | rec_data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(rec_data[4] << 8 | rec_data[5]);
    temp = (int16_t)(rec_data[6] << 8 | rec_data[7]);
    DataStruct->Gyro_X_RAW = (int16_t)(rec_data[8] << 8 | rec_data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(rec_data[10] << 8 | rec_data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(rec_data[12] << 8 | rec_data[13]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / ACCEL_Z_CORRECTOR;
    DataStruct->Temperature = ((float)temp / 340.0f) + 36.53f;
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = (DataStruct->Gyro_Z_RAW / 131.0) - MPU6050_GZ_BIAS;

    if ((DataStruct->Gz > -1.0) && (DataStruct->Gz < 1.0)) {
        DataStruct->Gz = 0.0;
    }

    now = HAL_GetTick();
    dt = (double)(now - mpu6050_timer) / 1000.0;
    mpu6050_timer = now;
    if (dt <= 0.0) {
        dt = 0.001;
    }

    roll_sqrt = sqrt((double)DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW +
                     (double)DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0) {
        roll = atan((double)DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    } else {
        roll = 0.0;
    }

    pitch = atan2(-(double)DataStruct->Accel_X_RAW, (double)DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    if (((pitch < -90.0) && (DataStruct->KalmanAngleY > 90.0)) ||
        ((pitch > 90.0) && (DataStruct->KalmanAngleY < -90.0))) {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    } else {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }

    if (fabs(DataStruct->KalmanAngleY) > 90.0) {
        DataStruct->Gx = -DataStruct->Gx;
    }

    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);
    yaw_angle += DataStruct->Gz * dt;

    while (yaw_angle > 180.0) {
        yaw_angle -= 360.0;
    }

    while (yaw_angle < -180.0) {
        yaw_angle += 360.0;
    }

    DataStruct->KalmanAngleZ = yaw_angle;
}
