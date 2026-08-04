#include "Error.h"
#include "usart.h"

error_t  Error_Code  = {0};
giveup_t Giveup_Code = {0};

void Reset_USART(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
        HAL_UART_DeInit(&huart1);
        MX_USART1_UART_Init();
    }
    else if(huart == &huart2)
    {
        HAL_UART_DeInit(&huart2);
        MX_USART2_UART_Init();
    }
    else if(huart == &huart3)
    {
        HAL_UART_DeInit(&huart3);
        MX_USART3_UART_Init();
    }
}