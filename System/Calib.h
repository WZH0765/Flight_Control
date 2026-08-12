#ifndef CALIB_H
#define CALIB_H

#include <stdbool.h>

typedef struct
{
    float HomeAlt;
    float GyroBias[3];
    float AccBias[3];

} calib_data_t;

bool Calibrate_All(void);
bool Calibrate_ESC(void);
extern calib_data_t Result;
extern LPS22HH_Object_t BAR;

#endif