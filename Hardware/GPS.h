#ifndef __GPS_H__
#define __GPS_H__

#include <stdint.h>
#include "Config.h"

typedef struct
{
	uint8_t Fix;            //定位状态
	uint8_t SatNum;         //可见卫星数

	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;

	float  Altitude;       //海拔(米)
	double Latitude;       //纬度(度)
	double Longitude;      //经度(度)

	float  GroundSpeed;    //对地速度(m/s)
	float  GroundCourse;   //对地航向(度,0-359.9)

	uint32_t TimeStamp;    //数据更新时间(ms,由外部写入)

} gps_data_t;

extern gps_data_t GPS_DATA;

void GPS_Init(void);
void GPS_Parse(uint8_t *buffer, uint16_t len);

#endif