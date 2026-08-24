#ifndef MAIN_H
#define MAIN_H

#include "stm32h7xx_hal.h"

/* 주변장치 핸들 (CubeMX가 생성하는 것과 동일한 이름 규약을 따름) */
extern ADC_HandleTypeDef  hadc2;
extern DMA_HandleTypeDef  hdma_adc2;
extern I2C_HandleTypeDef  hi2c4;
extern TIM_HandleTypeDef  htim1;  /* CH4: 조향 서보 PWM */
extern TIM_HandleTypeDef  htim4;  /* CH2: ESC PWM */
extern UART_HandleTypeDef huart1; /* rosserial */
extern UART_HandleTypeDef huart3; /* 디버그 로그 */
extern DMA_HandleTypeDef  hdma_usart1_rx;

/* 터치 안전 스위치 핀 */
#define TOUCH_SW_GPIO_Port GPIOC
#define TOUCH_SW_Pin       GPIO_PIN_13

#endif /* MAIN_H */
