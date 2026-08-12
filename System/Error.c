#include "Config.h"

void Reset_USART(UART_HandleTypeDef *huart)
{
    if(huart == &huart2)
    {
        HAL_DMA_DeInit(&hdma_usart2_rx);
        HAL_UART_DeInit(&huart2);
        MX_USART2_UART_Init();
    }
    else if(huart == &huart3)
    {
        HAL_DMA_DeInit(&hdma_usart3_rx);
        HAL_UART_DeInit(&huart3);
        MX_USART3_UART_Init();
    }
}