#include "FreeRTOS.h"
#include <stdbool.h>
#include <string.h>
#include "minmea.h"
#include "queue.h"
#include "usart.h"
#include "task.h"
#include "GPS.h"

gps_data_t GpsData = {0};

void GPS_Init(void)
{
    GpsData.Fix          = 0;
    GpsData.SatNum       = 0;
    GpsData.TimeStamp    = 0;

    GpsData.Hour         = 0;
    GpsData.Minute       = 0;
    GpsData.Second       = 0;

    GpsData.Altitude     = 0;
    GpsData.Latitude  	 = 0;
    GpsData.Longitude 	 = 0;

    GpsData.GroundSpeed  = 0;
    GpsData.GroundCourse = 0;

    HAL_UARTEx_ReceiveToIdle_DMA(&huart3,GPS_RxBuffer,sizeof(GPS_RxBuffer));
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);
}

/*
*   解析单行NMEA数据
*/
static void GPS_ParseLine(const char *line)
{
    struct minmea_sentence_rmc rmc;
    struct minmea_sentence_gga gga;

	//检查语句满足NMEA标准
    if(minmea_check(line, true) == false) return;

    switch(minmea_sentence_id(line,false))
    {
    case MINMEA_SENTENCE_RMC:
        if(minmea_parse_rmc(&rmc,line))
        {
			/*授时*/
            GpsData.Hour   = rmc.time.hours;
            GpsData.Minute = rmc.time.minutes;
            GpsData.Second = rmc.time.seconds;

            if(rmc.valid == true)
            {
                GpsData.Latitude  	  = minmea_tocoord(&rmc.latitude);
                GpsData.Longitude 	  = minmea_tocoord(&rmc.longitude);
                GpsData.GroundSpeed  = minmea_tofloat(&rmc.speed)*0.514f;   //节 -> m/s
                GpsData.GroundCourse = minmea_tofloat(&rmc.course);
            }
            else
            {
                GpsData.Latitude     = 0.0;
                GpsData.Longitude    = 0.0;
                GpsData.GroundSpeed  = 0.0f;
                GpsData.GroundCourse = 0.0f;
            }
            GpsData.TimeStamp = xTaskGetTickCount();
        }
        break;

    case MINMEA_SENTENCE_GGA:
        if(minmea_parse_gga(&gga,line))
        {
            if(gga.fix_quality > 0)
            {
                GpsData.Fix = (gga.fix_quality >= 2) ? GPS_FIX_3D : GPS_FIX_2D;
                GpsData.SatNum    = gga.satellites_tracked;
                GpsData.Altitude  = minmea_tofloat(&gga.altitude);
                GpsData.Latitude  = minmea_tocoord(&gga.latitude);
                GpsData.Longitude = minmea_tocoord(&gga.longitude);
            }
            else
            {
                GpsData.Fix = GPS_FIX_INVALID;
                GpsData.Altitude  = 0.0f;
                GpsData.Latitude  = 0.0;
                GpsData.Longitude = 0.0;
            }
            GpsData.TimeStamp = xTaskGetTickCount();
        //直接写入队列
        xQueueOverwrite(xGPS_DataQ,&GpsData);
        }
        break;

    default:
        break;
    }
}

/*
*   解析所有NMEA语句
*/
void GPS_Parse(uint8_t *buffer,uint16_t len)
{
    if(len == 0 || len >= GPS_LINE_MAX_LEN) return;

    buffer[len] = '\0';

    char *p = (char*)buffer;
    char *next;
    while(p && *p)
    {
        next = strchr(p,'\n');
        if(next)
        {
            *next = '\0';
        }

        size_t line_len = strlen(p);

        if(line_len > 0 && p[line_len - 1] == '\r')
        {
            p[line_len - 1] = '\0';
        }

        GPS_ParseLine(p);

        if(next)
        {
            p = next + 1;
        }
        else
        {
            break;
        }
    }
}
