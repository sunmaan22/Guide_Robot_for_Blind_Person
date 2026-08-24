#include "mpu6050.h"
#include "main.h"
#include <math.h>

#define MPU6050_ADDR        (0x68 << 1)
#define REG_PWR_MGMT_1      0x6B
#define REG_ACCEL_XOUT_H    0x3B
#define REG_GYRO_XOUT_H     0x43

#define ACCEL_SENS_2G   16384.0f  /* LSB/g */
#define GYRO_SENS_250DPS 131.0f   /* LSB/(deg/s) */

static float s_pitch = 0.0f;
static float s_roll  = 0.0f;

void MPU6050_Init(void)
{
    uint8_t wake = 0x00;
    /* PWR_MGMT_1 레지스터를 0으로 써서 슬립모드 해제 */
    HAL_I2C_Mem_Write(&hi2c4, MPU6050_ADDR, REG_PWR_MGMT_1, 1, &wake, 1, 100);
}

static int16_t combine_i16(uint8_t hi, uint8_t lo)
{
    return (int16_t)((hi << 8) | lo);
}

void Read_Accel(MPU6050_Data *data)
{
    uint8_t buf[6];
    if (HAL_I2C_Mem_Read(&hi2c4, MPU6050_ADDR, REG_ACCEL_XOUT_H, 1, buf, 6, 100) != HAL_OK) {
        return;
    }
    data->ax = combine_i16(buf[0], buf[1]) / ACCEL_SENS_2G;
    data->ay = combine_i16(buf[2], buf[3]) / ACCEL_SENS_2G;
    data->az = combine_i16(buf[4], buf[5]) / ACCEL_SENS_2G;
}

void Read_Gyro(MPU6050_Data *data)
{
    uint8_t buf[6];
    if (HAL_I2C_Mem_Read(&hi2c4, MPU6050_ADDR, REG_GYRO_XOUT_H, 1, buf, 6, 100) != HAL_OK) {
        return;
    }
    data->gx = combine_i16(buf[0], buf[1]) / GYRO_SENS_250DPS;
    data->gy = combine_i16(buf[2], buf[3]) / GYRO_SENS_250DPS;
    data->gz = combine_i16(buf[4], buf[5]) / GYRO_SENS_250DPS;
}

void Calculate_CompAngle(const MPU6050_Data *data, float dt_s, float *pitch_deg, float *roll_deg)
{
    /* 가속도로부터 절대 각도 계산 (중력 벡터 기준) */
    float accel_pitch = atan2f(-data->ax, sqrtf(data->ay * data->ay + data->az * data->az)) * (180.0f / (float)M_PI);
    float accel_roll  = atan2f(data->ay, data->az) * (180.0f / (float)M_PI);

    /* 자이로 적분값과 가속도 각도를 상보필터로 융합: 자이로 98%, 가속도 2% */
    s_pitch = 0.98f * (s_pitch + data->gy * dt_s) + 0.02f * accel_pitch;
    s_roll  = 0.98f * (s_roll  + data->gx * dt_s) + 0.02f * accel_roll;

    *pitch_deg = s_pitch;
    *roll_deg  = s_roll;
}
