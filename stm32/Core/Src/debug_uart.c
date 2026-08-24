#include "debug_uart.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

void debug_uart(const char *msg)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)msg, (uint16_t)strlen(msg), 50);
}

void Debug_PrintStatus(float pitch, uint16_t joy_x, uint16_t joy_y, int touch_active)
{
    char line[96];
    int len = snprintf(line, sizeof(line),
        "pitch=%.1f joy_x=%u joy_y=%u touch=%d\r\n",
        pitch, joy_x, joy_y, touch_active);
    if (len > 0) {
        HAL_UART_Transmit(&huart3, (uint8_t *)line, (uint16_t)len, 50);
    }
}
