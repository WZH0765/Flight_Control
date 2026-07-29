#ifndef _LOCK_H_
#define _LOCK_H_

#include <stdint.h>

/*硬件锁状态*/
typedef struct
{
    uint8_t SD_Unlock       : 1;
    uint8_t GPS_Unlock      : 1;
    uint8_t IMU_Unlock      : 1;
    uint8_t Baro_Unlock     : 1;
    uint8_t Receiver_Unlock : 1;
} HW_Lock_t;

/*系统锁状态*/
typedef struct
{
    uint16_t BeepCnt;       // 蜂鸣相位
    uint8_t Locking;        //1 蜂鸣进行中/正在上锁
    uint8_t LockState;      //0 允许电机运行/1 禁止电机运行

    uint16_t LockCnt;       // 解锁手势计数
    uint16_t UnlockCnt;     // 上锁手势计数

} Sys_Lock_t;

/*手势输入*/
typedef struct
{
    float Left_X;
    float Left_Y;
    float Right_X;
    float Right_Y;
} Detect_Lock_t;

void Lock_Init(void);
void Lock_Update(void);
uint8_t HW_Unlock(void);
void Lock_Detect(Detect_Lock_t gesture);

extern HW_Lock_t  HW_LockState;
extern Sys_Lock_t Sys_LockState;

#endif