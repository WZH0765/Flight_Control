#ifndef _ERROR_H_
#define _ERROR_H_

#include <stdint.h>

typedef struct
{
    uint8_t IMU_ReadID_Error;
    uint8_t IMU_Config_Error;
    uint8_t IMU_Timeout_Error;

    uint8_t RC_Config_Error;

} Error_t;

extern Error_t Error_Code;

#endif