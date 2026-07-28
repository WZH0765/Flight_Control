#include "Receiver.h"
#include "Lock.h"
#include <stdio.h>
#include "tim.h"

HW_Lock_t  HW_LockState  = {0};
Sys_Lock_t Sys_LockState = {0};

#define LOCK_RC_THRESHOLD   80.0f
#define LOCK_RC_HOLDTIME    1000

#define PWM_MIN 900
#define BEEP_PWM        2000
#define BEEP_COUNT      5
#define BEEP_DURATION   15
#define BEEP_PAUSE      35
#define BEEP_CYCLE      (BEEP_DURATION + BEEP_PAUSE)

void Lock_Init(void)
{
    Sys_LockState.LockCnt = 0;
    Sys_LockState.UnlockCnt = 0;
    Sys_LockState.LockState = 0;
    Sys_LockState.Locking = 0;
    Sys_LockState.Arm_Beep_Phase = 0;

    HW_LockState.SD_Unlock       = 0;
    HW_LockState.IMU_Unlock      = 0;
    HW_LockState.GPS_Unlock      = 0;
    HW_LockState.Baro_Unlock     = 0;
    HW_LockState.Receiver_Unlock = 0;
}

void DisArm(void)
{
    Sys_LockState.LockState = 1;
    Sys_LockState.Locking = 1;
    Sys_LockState.Arm_Beep_Phase = 0;
}

void Arm(void)
{
    Sys_LockState.LockState = 0;
    Sys_LockState.Locking = 0;
    Sys_LockState.Arm_Beep_Phase = 0;

    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,PWM_MIN);
    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,PWM_MIN);
    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,PWM_MIN);
    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,PWM_MIN);
}

void Lock_Detect(Detect_Lock_t gesture)
{
    /* 外八→解锁 */
    if(gesture.Left_X > LOCK_RC_THRESHOLD && gesture.Right_X < -LOCK_RC_THRESHOLD)
    {
        if(Sys_LockState.LockState == 0)
        {
            Sys_LockState.LockCnt++;
            Sys_LockState.UnlockCnt = 0;
            if(Sys_LockState.LockCnt >= LOCK_RC_HOLDTIME && HW_Unlock())
            {
                DisArm();
            }
        }
    }
    /* 内八→上锁 */
    else if(gesture.Left_X < -LOCK_RC_THRESHOLD && gesture.Right_X > LOCK_RC_THRESHOLD)
    {
        if(Sys_LockState.LockState == 1)
        {
            Sys_LockState.UnlockCnt++;
            Sys_LockState.LockCnt = 0;
            if(Sys_LockState.UnlockCnt >= LOCK_RC_HOLDTIME && HW_Unlock())
            {
                Arm();
            }
        }
    }
    else
    {
        Sys_LockState.LockCnt = 0;
        Sys_LockState.UnlockCnt = 0;
    }
}

void Beep(void)
{
    uint16_t phase = (uint16_t)Sys_LockState.Arm_Beep_Phase;

    if(phase < BEEP_COUNT*BEEP_CYCLE)
    {
        uint16_t cycle_pos = phase%BEEP_CYCLE;

        if(cycle_pos < BEEP_DURATION)
        {
            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,BEEP_PWM);
            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,BEEP_PWM);
            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,BEEP_PWM);
            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,BEEP_PWM);
        }
        else
        {
            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,900);
            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,900);
            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,900);
            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,900);
        }

        Sys_LockState.Arm_Beep_Phase += 1.0f;

        if(phase >= BEEP_COUNT*BEEP_CYCLE - 1)
        {
            Sys_LockState.Locking = 0;
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
    if(HW_LockState.IMU_Unlock && HW_LockState.Receiver_Unlock)
    {
        return 1;
    }
    return 0;
}