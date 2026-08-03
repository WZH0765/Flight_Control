#ifndef __LOG_H__
#define __LOG_H__

#include <stdint.h>

/*日志文件最大尺寸(字节)：1MB*/
#define LOG_FILE_MAX_SIZE (1024 * 1024)

/*一帧飞控数据*/
typedef struct
{
    float Ax;
    float Ay;
    float Az;
    float Gx;
    float Gy;
    float Gz;

    float Yaw;
    float Roll;
    float Pitch;

    /*内环目标值*/
    float RateYawTarget;
    float RateRollTarget;
    float RatePitchTarget;

    /*外环目标值*/
    float AngleYawTarget;
    float AngleRollTarget;
    float AnglePitchTarget;

    /*PID输出*/
    float OutYaw;
    float OutRoll;
    float OutPitch;

    /*PWM输出*/
    uint16_t Pwm1;
    uint16_t Pwm2;
    uint16_t Pwm3;
    uint16_t Pwm4;

    /*油门*/
    float Throttle;

    /*时间戳*/
    uint32_t TimeStamp;

} log_data_t;

/*日志系统状态*/
typedef struct
{
    uint8_t  Ready;         //挂载成功
    uint8_t  FileOpen;      //日志文件
    uint32_t FrameCount;    //已记录帧数
    uint32_t FileSize;      //当前文件大小

} log_status_t;

void Log_Init(void);
void Log_Sync(void);
void Log_Close(void);
uint8_t Log_Open(void);
void Log_Save(const log_data_t *d);
void Log_Record(const log_data_t *d);

extern log_status_t Log_Status;

#endif