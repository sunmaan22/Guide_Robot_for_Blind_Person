#ifndef ROSSERIAL_HARDWARE_H
#define ROSSERIAL_HARDWARE_H

#include "main.h"

/*
 * rosserial의 기본 Hardware 클래스(ArduinoHardware 등)를 대체하는 STM32용 구현.
 * USART1을 DMA로 수신하고, IDLE 라인 인터럽트로 "새 데이터가 들어왔다"는
 * 타이밍을 감지한다. 수신 버퍼는 원형(circular) DMA로 계속 채워지며,
 * read()는 tail 포인터를 기준으로 1바이트씩 순차적으로 꺼내온다.
 *
 * STM32H7은 D-Cache가 있어 DMA가 채운 영역을 CPU가 그대로 읽으면 캐시된
 * 옛 값을 볼 수 있으므로, read() 시점에 해당 영역을 무효화(invalidate)한다.
 */
class STM32Hardware
{
public:
    STM32Hardware() : rx_tail(0) {}

    void init();
    int read();
    void write(uint8_t *data, int length);
    unsigned long time();

private:
    static const uint16_t RX_BUF_SIZE = 256;
    uint8_t dma_rx_buf[RX_BUF_SIZE];
    uint16_t rx_tail;
};

#endif /* ROSSERIAL_HARDWARE_H */
