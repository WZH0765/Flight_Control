#ifndef __MAG_H__
#define __MAG_H__

#include <stdint.h>

#define MAG_OK 0

typedef struct
{
    int16_t Mag[3];         // X/Y/Z 原始LSB

} mag_raw_t;

typedef struct
{
    float Mag[3];           // X/Y/Z 磁场强度(uT)
    
} mag_data_t;

extern mag_raw_t MagRaw;

void MAG_Init(void);
void MAG_Read(void);

#endif