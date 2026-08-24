#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

typedef struct {
    float ax, ay, az; /* g */
    float gx, gy, gz; /* deg/s */
} MPU6050_Data;

void MPU6050_Init(void);

/* I2C4로 가속도/자이로 원시값을 읽어 물리 단위로 변환 */
void Read_Accel(MPU6050_Data *data);
void Read_Gyro(MPU6050_Data *data);

/* 가속도+자이로를 상보필터(자이로 98% / 가속도 2%)로 융합해 pitch/roll 계산.
 * dt_s: 이전 호출 이후 경과 시간(초) */
void Calculate_CompAngle(const MPU6050_Data *data, float dt_s, float *pitch_deg, float *roll_deg);

#endif /* MPU6050_H */
