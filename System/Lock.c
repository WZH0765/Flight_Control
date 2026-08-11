#include "Receiver.h"
#include "Lock.h"
#include <stdio.h>
#include "Config.h"
#include "tim.h"

hw_lock_t  HW_LockState  = {0};
sys_lock_t Sys_LockState = {0};

void Lock_Init(void)
{
    Sys_LockState.BeepCnt   = 0;
    Sys_LockState.LockCnt   = 0;
    Sys_LockState.Locking   = 0;
    Sys_LockState.UnlockCnt = 0;
    Sys_LockState.LockState = 1;

    HW_LockState.SD_Unlock  = 0;
    HW_LockState.IMU_Unlock = 0;
    HW_LockState.GPS_Unlock = 0;
    HW_LockState.BAR_Unlock = 0;
}

void Disable(void)
{
    Sys_LockState.LockState = 1;        //禁止电机运行
    Sys_LockState.Locking = 1;          //开启蜂鸣
    Sys_LockState.BeepCnt = 0;
}

void Enable(void)
{
    Sys_LockState.LockState = 0;        //允许电机运行
    Sys_LockState.Locking = 1;          //开启蜂鸣
    Sys_LockState.BeepCnt = 0;

    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,PWM_MIN);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,PWM_MIN);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,PWM_MIN);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,PWM_MIN);
}

/*蜂鸣总时间150ms，单次蜂鸣时间50ms*/
void Beep(void)
{
    uint16_t Cnt = Sys_LockState.BeepCnt;

    if(Cnt < 150)       //蜂鸣未结束
    {
        HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin,(Cnt%50 < 35) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        Sys_LockState.BeepCnt ++;

        if(Cnt == 150 - 1)
        {
            Sys_LockState.Locking = 0;
            HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_RESET);
        }
    }
}

void Lock_Update(void)
{
    if(Sys_LockState.Locking)
    {
        Beep();
    }
}

uint8_t HW_Unlock(void)
{
    TickType_t xCurrentTime = xTaskGetTickCount();

    if(HW_LockState.IMU_Unlock == 0) return 0;
    if(HW_LockState.MAG_Unlock == 0) return 0;
    if(HW_LockState.BAR_Unlock == 0) return 0;
    if(HW_LockState.GPS_Unlock == 0) return 0;
    //添加

    if((xCurrentTime - RcTime.TimeStamp) < pdMS_TO_TICKS(500))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}