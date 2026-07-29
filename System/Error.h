#ifndef _ERROR_H_
#define _ERROR_H_

#include <stdint.h>

#define PWM_MIN   900
#define PWM_MAX   2000
#define PID_NORM  1.0f/50.0f
#define PWM_RANGE PWM_MAX - PWM_MIN

typedef struct
{
    uint8_t IMU_ReadID_Error;
    uint8_t IMU_Config_Error;
    uint8_t IMU_Timeout_Error;

    uint8_t RC_Config_Error;

} Error_t;

extern Error_t Error_Code;

#endif