#ifndef __IMU_H__
#define __IMU_H__

#include "inv_imu_driver.h"
#include "math.h"

void IMU_Init(void);

/*DATA PROCESSED BEGIN*/

//经过处理的数据
typedef struct
{
	float ACC_X;
	float ACC_Y;
	float ACC_Z;
	
	float GYRO_X;
	float GYRO_Y;
	float GYRO_Z;

} imu_data_t;

//未经处理的数据
typedef struct
{
    int16_t ACC[3];
    int16_t GYRO[3];

} imu_raw_t;

extern imu_raw_t  IMU_RAW;
extern imu_data_t IMU_DATA;

extern inv_imu_device_t IMU;
extern inv_imu_fifo_data_t FIFO_Data;
/*DATA PROCESSED END*/

static inline void Get_Angle(float ax,float ay,float az,float* roll,float* pitch)
{
	*roll = atan2f(ay,az);
	const float den = sqrtf(ay*ay + az*az);
	*pitch = atan2f(-ax,den);
}

#endif
