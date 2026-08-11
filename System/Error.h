#ifndef _ERROR_H_
#define _ERROR_H_

#include "usart.h"
#include <stdint.h>

/*
*   1 有错误
*   0 无错误
*/
typedef struct
{
    //RC
    uint8_t RC_Timeout_Error;   //超时错误

    //MAG
    uint8_t MAG_Error;

    //BAR
    uint8_t BAR_Error;

    //GPS
    uint8_t GPS_Error;

    //LOG
    uint8_t LOG_Open_Error;     //打开错误
    uint8_t LOG_Mount_Error;    //挂载错误
    uint8_t LOG_Write_Error;    //写入错误

} error_t;

/*
*   1 放弃尝试
*   0 继续尝试
*/
typedef struct
{
    uint8_t LOG_Open_Giveup;
    uint8_t LOG_Mount_Giveup;
    uint8_t LOG_Write_Giveup;

} giveup_t;

extern error_t Error_Code;
extern giveup_t Giveup_Code;

void Reset_USART(UART_HandleTypeDef *huart);

#endif