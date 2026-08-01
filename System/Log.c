#include "FreeRTOS.h"
#include "Config.h"
#include "queue.h"
#include "fatfs.h"
#include "Error.h"
#include "Log.h"

#include <stdio.h>
#include <string.h>

static FIL  Log_File;
static char Log_Line[512];
static char Log_FileName[32];

log_status_t Log_Status = {0};
extern QueueHandle_t xLOG_DataQ;

void Log_Init(void)
{
    FRESULT res;

    res = f_mount(&SDFatFS, SDPath, 0);
    if(res != FR_OK)
    {
        Error_Code.LOG_Mount_Error = 1;
        return ;
    }
    Log_Status.Ready = 1;
}

void Log_Record(const log_data_t *data)
{
    if(xLOG_DataQ == NULL || data == NULL || !Log_Status.Ready) return ;
    xQueueSend(xLOG_DataQ,data,0);
}

void Log_Save(const log_data_t *d)
{
    UINT bw;
    int n;

    if(!Log_Status.FileOpen) return ;

    if(Log_Status.FileSize >= LOG_FILE_MAX_SIZE)
    {
        Log_Close();
        if(!Log_Open())
        {
            Log_Status.FileOpen = 0;
            return;
        }
    }

    n = snprintf(Log_Line, sizeof(Log_Line),
        "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
        "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
        "%.3f,%.3f,%.3f,%u,%u,%u,%u,%.3f\r\n",
        (unsigned long)d->TimeStamp,
        d->Ax, d->Ay, d->Az, d->Gx, d->Gy, d->Gz,
        d->Yaw, d->Roll, d->Pitch,
        d->RateYawTarget, d->RateRollTarget, d->RatePitchTarget,
        d->AngleYawTarget, d->AngleRollTarget, d->AnglePitchTarget,
        d->OutYaw, d->OutRoll, d->OutPitch,
        d->Pwm1, d->Pwm2, d->Pwm3, d->Pwm4,
        d->Throttle);

    if(n <= 0 || n > (int)sizeof(Log_Line)) return;

    if(f_write(&Log_File, Log_Line, (UINT)n, &bw) != FR_OK || bw != (UINT)n)
    {
        Error_Code.LOG_Write_Error = 1;
        Log_Close();
        return;
    }

    Log_Status.FileSize += bw;
    Log_Status.FrameCount++;
}

uint8_t Log_Open(void)
{
    FRESULT res;
    UINT bw;
    uint16_t idx = 0;

    if(!Log_Status.Ready) return 0;

    do
    {
        if(idx > 9999) return 0;
        snprintf(Log_FileName, sizeof(Log_FileName), "LOG_%04u.CSV", idx++);
        res = f_open(&Log_File,Log_FileName,FA_CREATE_NEW | FA_WRITE);
    } while(res == FR_EXIST);

    if(res != FR_OK) return 0;

    snprintf(Log_Line, sizeof(Log_Line),
        "Time,Ax,Ay,Az,Gx,Gy,Gz,Yaw,Roll,Pitch,"
        "RtYaw,RtRoll,RtPitch,AtYaw,AtRoll,AtPitch,"
        "OutYaw,OutRoll,OutPitch,Pwm1,Pwm2,Pwm3,Pwm4,Thr\r\n");

    if(f_write(&Log_File,Log_Line,strlen(Log_Line),&bw) != FR_OK || bw != strlen(Log_Line))
    {
        f_close(&Log_File);
        return 0;
    }

    Log_Status.FileOpen = 1;
    Log_Status.FileSize = 0;
    Log_Status.FrameCount = 0;
    return 1;
}

void Log_Close(void)
{
    if(Log_Status.FileOpen)
    {
        f_sync(&Log_File);
        f_close(&Log_File);
        Log_Status.FileOpen = 0;
    }
}

void Log_Sync(void)
{
    if(Log_Status.FileOpen)
    {
        f_sync(&Log_File);
    }
}