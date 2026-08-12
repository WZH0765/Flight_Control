#include "FreeRTOS.h"
#include "Config.h"
#include <string.h>
#include <stdio.h>
#include "queue.h"
#include "fatfs.h"
#include "Error.h"
#include "Log.h"

static FIL  Log_File;
static char Log_Line[512];
static char Log_FileName[32];

log_status_t Log_Status = {0};
extern QueueHandle_t xLOG_DataQ;

void Log_Init(void)
{
    FRESULT res;

    res = f_mount(&SDFatFS,SDPath,0);     //尝试挂载SD驱动器
    if(res != FR_OK)
    {
        return ;
    }
    Log_Status.Ready = 1;       //挂载成功
}

void Log_Record(const log_data_t *data)
{
    if(xLOG_DataQ == NULL || data == NULL || Log_Status.Ready == 0) return;
    xQueueSend(xLOG_DataQ,data,0);
}

void Log_Save(const log_data_t *d)
{
    UINT bw;
    int n;

    if(Log_Status.Ready == 0)
    {

        return;
    }
    if(Log_Status.FileOpen == 0)
    {
        return;
    }

    //文件是否超过规定大小
    if(Log_Status.FileSize >= LOG_FILE_MAX_SIZE)
    {
        Log_Close();
        //尝试打开新文件失败
        if(Log_Open() == 0)
        {
            Log_Status.FileOpen = 0;
            return;
        }
        else
        {

        }
    }

    n = snprintf(Log_Line,sizeof(Log_Line),
    "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
    "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
    "%.3f,%.3f,%.3f,%u,%u,%u,%u,%.3f\r\n",
    (unsigned long)d->TimeStamp,d->Ax,d->Ay,d->Az,d->Gx,d->Gy,d->Gz,d->Yaw,d->Roll,d->Pitch,
    d->RateYawTarget,d->RateRollTarget,d->RatePitchTarget,d->AngleYawTarget,d->AngleRollTarget,d->AnglePitchTarget,
    d->OutYaw,d->OutRoll,d->OutPitch,d->Pwm1,d->Pwm2,d->Pwm3,d->Pwm4,d->Throttle);

    if(n <= 0 || n >= (int)sizeof(Log_Line)) return;

    if(f_write(&Log_File,Log_Line,(UINT)n,&bw) != FR_OK || bw != (UINT)n)
    {
        Log_Close();
        
        return;
    }

    Log_Status.FileSize += bw;
    Log_Status.FrameCount ++;
}

uint8_t Log_Open(void)
{
    FRESULT res;
    UINT bw;
    uint16_t Index = 0;
    uint16_t retry = 0;    //创建文件的重试计数器

    if(!Log_Status.Ready) return 0;

    do
    {
        //文件系统异常时避免无限循环
        if(Index > 9999 || (++ retry) > 10000) return 0;
        snprintf(Log_FileName,sizeof(Log_FileName),"LOG_%04u.CSV",Index ++);
        res = f_open(&Log_File,Log_FileName,FA_CREATE_NEW | FA_WRITE);

    } while(res == FR_EXIST);

    if(res != FR_OK) return 0;

    snprintf(Log_Line,sizeof(Log_Line),
    "Time,Ax,Ay,Az,Gx,Gy,Gz,Yaw,Roll,Pitch,"
    "RtYaw,RtRoll,RtPitch,AtYaw,AtRoll,AtPitch,"
    "OutYaw,OutRoll,OutPitch,Pwm1,Pwm2,Pwm3,Pwm4,Thr\r\n");

    if(f_write(&Log_File,Log_Line,strlen(Log_Line),&bw) != FR_OK || bw != strlen(Log_Line))
    {
        f_close(&Log_File);
        f_unlink(Log_FileName);

        return 0;
    }

    Log_Status.FileOpen = 1;
    Log_Status.FileSize = 0;
    Log_Status.FrameCount = 0;
    return 1;
}

void Log_Close(void)
{
    if(Log_Status.FileOpen == 1)
    {
        f_sync(&Log_File);
        f_close(&Log_File);
        Log_Status.FileOpen = 0;
    }
}

void Log_Sync(void)
{
    if(Log_Status.FileOpen == 1)
    {
        f_sync(&Log_File);
    }
}