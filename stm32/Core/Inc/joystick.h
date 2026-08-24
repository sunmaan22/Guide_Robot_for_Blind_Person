#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <stdint.h>

/* ADC2 + DMA로 2채널(X/Y)을 스캔 변환하기 위한 초기화. 50ms 주기로
 * Joystick_TriggerRead()를 호출해 변환을 트리거한다. */
void Joystick_Init(void);
void Joystick_TriggerRead(void);

/* 가장 최근 DMA로 채워진 원시값 (오버샘플링 적용, 0~65500 범위) */
uint16_t Joystick_GetRawX(void);
uint16_t Joystick_GetRawY(void);

/* joy_y 원시값을 데드존(31000~37000)과 실사용 범위(19000~65500)를 반영해
 * 서보 각도(0~180deg)로 변환 */
float Joystick_GetSteeringAngleDeg(void);

#endif /* JOYSTICK_H */
