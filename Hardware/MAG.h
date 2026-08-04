#ifndef __MAG_H__
#define __MAG_H__

#include <stdint.h>

typedef struct
{
    int16_t Mag[3];         // X/Y/Z 原始LSB

} mag_raw_t;

typedef struct
{
    float Mag[3];           // X/Y/Z 磁场强度(uT)
    
} mag_data_t;

extern mag_raw_t  MAG_RAW;
extern mag_data_t MAG_DATA;

int MAG_Init(void);
int MAG_Read(void);

#endif