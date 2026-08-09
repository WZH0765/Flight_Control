#ifndef __MAG_H__
#define __MAG_H__

#include <stdint.h>

typedef struct
{
    int16_t Mag[3];         // X/Y/Z 原始LSB

} mag_raw_t;

typedef struct
{
    float Mx;
    float My;
    float Mz;
    
} mag_data_t;

extern QueueHandle_t xMAG_DataQ;

void MAG_Init(void);
int MAG_Parse(void);

#endif