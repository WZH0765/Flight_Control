#include "FreeRTOS.h"
#include <stdbool.h>
#include <string.h>
#include "minmea.h"
#include "queue.h"
#include "task.h"
#include "GPS.h"

uint8_t Data_Flag   = 0;
gps_data_t GPS_DATA = {0};

extern QueueHandle_t xGPS_DataQ;

void GPS_Init(void)
{
    GPS_DATA.Fix          = 0;
    GPS_DATA.SatNum       = 0;
    GPS_DATA.TimeStamp    = 0;

    GPS_DATA.Hour         = 0;
    GPS_DATA.Minute       = 0;
    GPS_DATA.Second       = 0;

    GPS_DATA.Altitude     = 0;
    GPS_DATA.Latitude  	  = 0;
    GPS_DATA.Longitude 	  = 0;

    GPS_DATA.GroundSpeed  = 0;
    GPS_DATA.GroundCourse = 0;
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
            GPS_DATA.Hour   = rmc.time.hours;
            GPS_DATA.Minute = rmc.time.minutes;
            GPS_DATA.Second = rmc.time.seconds;

            if(rmc.valid == true)
            {
                GPS_DATA.Fix = GPS_FIX_2D;
                GPS_DATA.Latitude  	  = minmea_tocoord(&rmc.latitude);
                GPS_DATA.Longitude 	  = minmea_tocoord(&rmc.longitude);
                GPS_DATA.GroundSpeed  = minmea_tofloat(&rmc.speed)*0.514444f;   //节 -> m/s
                GPS_DATA.GroundCourse = minmea_tofloat(&rmc.course);
            }
            else
            {
                GPS_DATA.Fix = GPS_FIX_INVALID;
                GPS_DATA.Latitude     = 0.0;
                GPS_DATA.Longitude    = 0.0;
                GPS_DATA.GroundSpeed  = 0.0f;
                GPS_DATA.GroundCourse = 0.0f;
            }
            GPS_DATA.TimeStamp = xTaskGetTickCount();
            Data_Flag = 1;
        }
        break;

    case MINMEA_SENTENCE_GGA:
        if(minmea_parse_gga(&gga,line))
        {
            if(gga.fix_quality > 0)
            {
                GPS_DATA.Fix = (gga.fix_quality >= 2) ? GPS_FIX_3D : GPS_FIX_2D;
                GPS_DATA.SatNum    = gga.satellites_tracked;
                GPS_DATA.Altitude  = minmea_tofloat(&gga.altitude);
                GPS_DATA.Latitude  = minmea_tocoord(&gga.latitude);
                GPS_DATA.Longitude = minmea_tocoord(&gga.longitude);
            }
            else
            {
                GPS_DATA.Fix = GPS_FIX_INVALID;
                GPS_DATA.Altitude  = 0.0f;
                GPS_DATA.Latitude  = 0.0;
                GPS_DATA.Longitude = 0.0;
            }
            GPS_DATA.TimeStamp = xTaskGetTickCount();
            Data_Flag = 1;
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

    if(Data_Flag == 1)
    {
        Data_Flag = 0;
        xQueueOverwrite(xGPS_DataQ,&GPS_DATA);
    }
}
