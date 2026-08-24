#include "rosserial_hardware.h"

void STM32Hardware::init()
{
    /* 원형 DMA 수신 시작: DMA가 계속 dma_rx_buf를 순환하며 채운다 */
    HAL_UART_Receive_DMA(&huart1, dma_rx_buf, RX_BUF_SIZE);
    /* 라인이 일정 시간 비면(=한 rosserial 패킷 경계로 간주) IDLE 인터럽트 발생 */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    rx_tail = 0;
}

int STM32Hardware::read()
{
    /* DMA가 현재까지 채운 위치 = 버퍼 크기 - 남은 카운터 */
    uint16_t dma_write_pos = RX_BUF_SIZE - (uint16_t)__HAL_DMA_GET_COUNTER(huart1.hdmarx);

    if (dma_write_pos == rx_tail) {
        return -1; /* 새 바이트 없음 */
    }

    /* D-Cache에 남아있는 이전 값이 아니라 실제 DMA가 쓴 값을 읽기 위해 무효화 */
    SCB_InvalidateDCache_by_Addr((uint32_t *)&dma_rx_buf[rx_tail], 32);

    uint8_t byte = dma_rx_buf[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1) % RX_BUF_SIZE);
    return byte;
}

void STM32Hardware::write(uint8_t *data, int length)
{
    HAL_UART_Transmit(&huart1, data, (uint16_t)length, 100);
}

unsigned long STM32Hardware::time()
{
    return HAL_GetTick();
}

/*
 * USART1 IDLE 라인 인터럽트 핸들러.
 * 실제 STM32CubeIDE 프로젝트에서는 stm32h7xx_it.c에 위치하지만,
 * rosserial 연동과의 관련성을 보여주기 위해 여기에 함께 두었다.
 * IDLE 플래그만 클리어하면 되고, 실제 바이트 소비는 spinOnce()가
 * 주기적으로 호출하는 read()에서 이루어진다.
 */
extern "C" void USART1_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    }
    HAL_UART_IRQHandler(&huart1);
}
