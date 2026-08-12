#ifndef _HW_STATE_H_
#define _HW_STATE_H_

#include "stdbool.h"

//硬件ID
typedef enum
{
    HW_IMU = 0,
    HW_MAG = 1,
    HW_BAR = 2,
    HW_GPS = 3

} hw_id_t;

//硬件是否就绪
typedef struct
{
    hw_id_t ID;
    char *Name;
    uint8_t IsReady;
    TickType_t TimeStamp;

} hw_ready_t;

void HW_SetReady(hw_id_t ID);
void HW_SetUnready(hw_id_t ID);
bool HW_AllReady(void);

#endif