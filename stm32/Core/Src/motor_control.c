#include "motor_control.h"
#include "main.h"

/* PWM 펄스폭 상수 (단위: us) */
#define SERVO_PWM_MIN   1000
#define SERVO_PWM_MAX   2000
#define SERVO_ANGLE_MAX 180.0f

#define ESC_PWM_MIN     1000
#define ESC_PWM_MAX     2000
#define ESC_PWM_NEUTRAL 1500

#define ESC_FWD_PWM_MIN 1300  /* speed = -1000 */
#define ESC_FWD_PWM_MAX 1700  /* speed = +1000 */

#define MOTOR_RAMP_STEP 20    /* 호출당 최대 PWM 변화폭(us), 급변 방지 */

static uint16_t last_pwm = ESC_PWM_NEUTRAL;

static uint16_t clamp_u16(int32_t value, uint16_t lo, uint16_t hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return (uint16_t)value;
}

void Servo_SetAngle(float deg)
{
    if (deg < 0.0f) deg = 0.0f;
    if (deg > SERVO_ANGLE_MAX) deg = SERVO_ANGLE_MAX;

    uint16_t pulse_us = (uint16_t)(SERVO_PWM_MIN +
        (deg / SERVO_ANGLE_MAX) * (SERVO_PWM_MAX - SERVO_PWM_MIN));

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse_us);
}

void ESC_SetPWM(uint16_t us)
{
    uint16_t clamped = clamp_u16(us, ESC_PWM_MIN, ESC_PWM_MAX);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, clamped);
    last_pwm = clamped;
}

uint16_t map_bidir(int16_t speed)
{
    if (speed < -1000) speed = -1000;
    if (speed > 1000) speed = 1000;

    /* -1000..1000 -> 1300..1700 선형 변환 */
    int32_t pwm = ESC_FWD_PWM_MIN +
        ((int32_t)(speed + 1000) * (ESC_FWD_PWM_MAX - ESC_FWD_PWM_MIN)) / 2000;

    return clamp_u16(pwm, ESC_FWD_PWM_MIN, ESC_FWD_PWM_MAX);
}

void Motor_Forward(int16_t speed)
{
    if (speed < 0) speed = 0;

    uint16_t target_pwm = map_bidir(speed);

    /* last_pwm에서 target_pwm으로 스텝 제한을 두고 부드럽게 램핑 */
    if (target_pwm > last_pwm) {
        uint16_t next = last_pwm + MOTOR_RAMP_STEP;
        ESC_SetPWM(next > target_pwm ? target_pwm : next);
    } else if (target_pwm < last_pwm) {
        uint16_t next = (last_pwm > MOTOR_RAMP_STEP) ? last_pwm - MOTOR_RAMP_STEP : ESC_PWM_MIN;
        ESC_SetPWM(next < target_pwm ? target_pwm : next);
    }
    /* target_pwm == last_pwm 이면 변경 없음 */
}

void Motor_Backward(void)
{
    /* 일부 ESC는 중립 -> 브레이크 -> 후진 펄스 시퀀스를 거쳐야 리버스로 전환된다 */
    ESC_SetPWM(ESC_PWM_NEUTRAL);
    HAL_Delay(50);
    ESC_SetPWM(ESC_FWD_PWM_MIN - 50); /* 짧은 브레이크 펄스 */
    HAL_Delay(50);
    ESC_SetPWM(ESC_PWM_NEUTRAL);
    HAL_Delay(50);
    ESC_SetPWM(ESC_FWD_PWM_MIN); /* 후진 진입 */
}

void Motor_Stop(void)
{
    ESC_SetPWM(ESC_PWM_NEUTRAL);
}
