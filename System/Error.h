#ifndef _ERROR_H_
#define _ERROR_H_

#include <stdint.h>

typedef struct
{
    uint8_t IMU_ReadID_Error;   //读ID错误
    uint8_t IMU_Config_Error;   //配置错误
    uint8_t IMU_Timeout_Error;  //超时错误

    uint8_t LOG_Open_Error;     //打开错误
    uint8_t LOG_Mount_Error;    //挂载错误
    uint8_t LOG_Write_Error;    //写入错误

    uint8_t RC_Config_Error;

} Error_t;

extern Error_t Error_Code;

#endif