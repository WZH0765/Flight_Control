#ifndef _ERROR_H_
#define _ERROR_H_

#include "usart.h"
#include <stdint.h>

void Reset_USART(UART_HandleTypeDef *huart);

//电机紧急停机标志（由控制任务读取，避免直接写寄存器产生竞争）
extern volatile uint8_t MotorStopCmd;
//电调校准标志（校准期间禁止控制任务干预PWM）
extern volatile uint8_t EscCalibrating;

//电机停止：置位停机标志并立即输出最小脉宽
void MOTOR_Stop(void);

#endif