#ifndef __BAR_H__
#define __BAR_H__

#include "FreeRTOS.h"
#include "semphr.h"
#include <stdint.h>

typedef struct
{
    float Temp;
    float Press;

} bar_raw_t;

typedef struct
{
    float Alt;
    uint32_t TimeStamp;

} bar_data_t;

extern QueueHandle_t xBAR_DataQ;

void BAR_Init(void);
int  BAR_Read(void);

#endif