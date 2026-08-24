#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include <stdint.h>

/* USART3로 "pitch, joy_x, joy_y, touch" 등 현재 상태를 사람이 읽을 수 있게 출력 */
void debug_uart(const char *msg);
void Debug_PrintStatus(float pitch, uint16_t joy_x, uint16_t joy_y, int touch_active);

#endif /* DEBUG_UART_H */
