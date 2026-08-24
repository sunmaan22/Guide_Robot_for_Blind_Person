#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

/* 조향 서보: deg(0~180) -> 1000~2000us PWM (TIM1 CH4) */
void Servo_SetAngle(float deg);

/* ESC: 원시 PWM 펄스폭(us)을 1000~2000us로 제한해 출력 (TIM4 CH2) */
void ESC_SetPWM(uint16_t us);

/* 양방향 속도(-1000~1000)를 ESC 중립 기준 PWM(1300~1700us)으로 선형 변환 */
uint16_t map_bidir(int16_t speed);

/* 목표 속도로 부드럽게 램핑하며 전진 구동 */
void Motor_Forward(int16_t speed);

/* 중립 -> 브레이크 -> 후진 순서로 ESC 아밍 시퀀스를 수행 */
void Motor_Backward(void);

/* 중립(1500us) 출력으로 즉시 정지 */
void Motor_Stop(void);

#endif /* MOTOR_CONTROL_H */
