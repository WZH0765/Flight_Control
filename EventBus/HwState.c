#include "Config.h"

//硬件表
hw_ready_t HW_Table[4] =
{
    {HW_IMU,"ICM45686" ,0,0},
    {HW_MAG,"IST8310"  ,0,0},
    {HW_BAR,"LPS22HH"  ,0,0},
    {HW_GPS,"SR25M10DI",0,0},
};

//设置硬件就绪
void HW_SetReady(hw_id_t ID)
{
    HW_Table[ID].IsReady = 1;
    HW_Table[ID].TimeStamp = xTaskGetTickCount();
}

//设置硬件未就绪
void HW_SetUnready(hw_id_t ID)
{
    HW_Table[ID].IsReady = 0;
    HW_Table[ID].TimeStamp = xTaskGetTickCount();
}

//检查所有硬件就绪情况
bool HW_AllReady(void)
{
    for(int i = 0;i < 4;i ++)
    {
        if(!HW_Table[i].IsReady) return false;
    }
    return true;
}