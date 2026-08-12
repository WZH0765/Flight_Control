#include "Config.h"
#include <stdbool.h>

calib_data_t Result  = {0};

bool Calibrate_BAR(void)
{
    float Pressure = 0.0f;
    float Sum = 0.0f;
    int   j   = 0;

    for(int i = 0;i < 50;i ++)
    {
        if(LPS22HH_PRESS_GetPressure(&BAR,&Pressure) == LPS22HH_OK)
        {
            float Height = (1.0f - powf(Pressure/1013.25f,0.190263f))*44330.8f;
            Sum += Height;
            j ++;
        }
        HAL_Delay(2);  // 等待传感器更新
    }
    Result.HomeAlt = Sum/j;

    if(Result.HomeAlt != 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Calibrate_GYR(void)
{
    float Sum[3] = {0};
    int j = 0;

    for(int i = 0;i < 100;i ++)
    {
        inv_imu_sensor_data_t data;
        if(inv_imu_get_register_data(&IMU,&data) == INV_IMU_OK)
        {
            Sum[0] += data.gyro_data[0]*GYRO_SCALE;
            Sum[1] += data.gyro_data[1]*GYRO_SCALE;
            Sum[2] += data.gyro_data[2]*GYRO_SCALE;
            j ++;
        }
        HAL_Delay(3);
    }
    if(j > 0)
    {
        Result.GyroBias[0] = Sum[0]/j;
        Result.GyroBias[1] = Sum[1]/j;
        Result.GyroBias[2] = Sum[2]/j;
    }

    if(Result.GyroBias[0] != 0 && Result.GyroBias[1] != 0 && Result.GyroBias[2] != 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Calibrate_ACC(void)
{
    float Sum[3] = {0};
    int j = 0;

    for (int i = 0; i < 100; i++)
    {
        inv_imu_sensor_data_t data;
        if(inv_imu_get_register_data(&IMU,&data) == INV_IMU_OK)
        {
            Sum[0] += data.accel_data[0]*ACC_SCALE;
            Sum[1] += data.accel_data[1]*ACC_SCALE;
            Sum[2] += data.accel_data[2]*ACC_SCALE;
            j ++;
        }
        HAL_Delay(3);
    }
    if(j > 0)
    {
        Result.AccBias[0] =  Sum[0]/j;
        Result.AccBias[1] =  Sum[1]/j;
        Result.AccBias[2] = (Sum[2]/j) - 9.8f;
    }

    if(Result.AccBias[0] != 0 && Result.AccBias[1] != 0 && Result.AccBias[2] != 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Calibrate_All(void)
{
    bool status = true;
    static bool already_cali = false;
    if(already_cali == true) return true;

    status &= Calibrate_BAR();
    status &= Calibrate_GYR();
    status &= Calibrate_ACC();
    if(status == true)
    {
        already_cali = true;
        EvtBus_Publish(&(evt_publish_t){.ID = EVT_CALIB_DONE});
        return true;
    }
    else
    {
        return false;
    }
}