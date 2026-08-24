#include "joystick.h"
#include "main.h"

/* 조이스틱 Y축(조향) 데드존 및 실사용 범위 (16비트 오버샘플링 ADC 기준) */
#define JOY_DEADZONE_LO 31000
#define JOY_DEADZONE_HI 37000
#define JOY_RANGE_MIN   19000
#define JOY_RANGE_MAX   65500
#define JOY_CENTER      ((JOY_DEADZONE_LO + JOY_DEADZONE_HI) / 2)

static volatile uint16_t s_adc_buf[2]; /* [0]=X, [1]=Y */

void Joystick_Init(void)
{
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)s_adc_buf, 2);
}

void Joystick_TriggerRead(void)
{
    /* ADC2가 연속모드가 아니라면 소프트웨어 트리거로 변환 시작 */
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)s_adc_buf, 2);
}

uint16_t Joystick_GetRawX(void)
{
    return s_adc_buf[0];
}

uint16_t Joystick_GetRawY(void)
{
    return s_adc_buf[1];
}

float Joystick_GetSteeringAngleDeg(void)
{
    uint16_t raw = s_adc_buf[1];

    /* 중앙 데드존 안이면 정중앙(90deg)으로 고정 */
    if (raw >= JOY_DEADZONE_LO && raw <= JOY_DEADZONE_HI) {
        return 90.0f;
    }

    if (raw < JOY_RANGE_MIN) raw = JOY_RANGE_MIN;
    if (raw > JOY_RANGE_MAX) raw = JOY_RANGE_MAX;

    float ratio = (float)(raw - JOY_RANGE_MIN) / (float)(JOY_RANGE_MAX - JOY_RANGE_MIN);
    return ratio * 180.0f;
}
