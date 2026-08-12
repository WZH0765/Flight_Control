#include "Config.h"

/**
*   电机停止：置位停机标志供控制任务读取，并立即输出最小脉宽
*   由控制任务根据 MotorStopCmd 标志统一决定是否保持停止，避免寄存器写竞争
**/
void MOTOR_Stop(void)
{
    MotorStopCmd = 1;    //通知控制任务停止输出

    //立即将PWM输出至最小脉宽，防止电机误动
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,PWM_MIN);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,PWM_MIN);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,PWM_MIN);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,PWM_MIN);
}

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