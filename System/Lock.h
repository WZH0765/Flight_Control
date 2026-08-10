#ifndef _LOCK_H_
#define _LOCK_H_

#include <stdint.h>

/*硬件锁状态*/
typedef struct
{
    uint8_t SD_Unlock;
    uint8_t GPS_Unlock;
    uint8_t IMU_Unlock;
    uint8_t MAG_Unlock;
    uint8_t BAR_Unlock;

} hw_lock_t;

/*系统锁状态*/
typedef struct
{
    uint16_t BeepCnt;       //蜂鸣相位

    uint8_t Locking;        //1 蜂鸣进行中/正在上锁
    uint8_t LockState;      //0 允许电机运行/1 禁止电机运行

    uint16_t LockCnt;       //上锁手势计数
    uint16_t UnlockCnt;     //解锁手势计数

} sys_lock_t;

/*手势锁状态*/
typedef struct
{
    float Left_X;
    float Right_X;
    float Throttle;

} ges_lock_t;

void Lock_Init(void);
void Lock_Update(void);
uint8_t HW_Unlock(void);
void Lock_Detect(ges_lock_t gesture);

extern hw_lock_t  HW_LockState;
extern sys_lock_t Sys_LockState;

#endif