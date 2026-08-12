#ifndef _EVTBUS_H
#define _EVTBUS_H

#include "FreeRTOS.h"
#include "stdbool.h"
#include "queue.h"

//事件总线上所有事件
typedef enum
{
//事件 -系统事件
    EVT_LANDED,
    EVT_TAKEOFF,
    EVT_INIT_DONE,
    EVT_CALIB_DONE,
    EVT_ARM_GESTURE,
    EVT_DISARM_GESTURE,

//事件 -传感器ready
    EVT_IMU_READY,
    EVT_MAG_READY,
    EVT_BAR_READY,
    EVT_GPS_READY,
    EVT_SENSORS_READY,

//事件 -传感器error
    EVT_IMU_ERROR,
    EVT_MAG_ERROR,
    EVT_BAR_ERROR,
    EVT_GPS_ERROR,

//事件 -遥控lost&found
    EVT_RC_LOST,
    EVT_RC_RECOVER,

//事件 -数据ready
    EVT_MAG_DATA_READY,
    EVT_BAR_DATA_READY,
    EVT_GPS_DATA_READY

} evt_id_t;

//发送事件
typedef struct
{
    evt_id_t ID;
    TickType_t TimeStamp;

} evt_publish_t;

//根据发布event指向函数callback
typedef void (*evt_callback_t)(evt_publish_t *event);

//订阅事件
typedef struct
{
    evt_id_t ID;
    evt_callback_t Callback;

} evt_subscribe_t;

void EvtBus_Init(void);
bool EvtBus_Publish(evt_publish_t *event);
bool EvtBus_Subscribe(evt_id_t ID,evt_callback_t Callback);

void EvtBus_SensorReady_Callback(evt_publish_t *ev);
void EvtBus_SensorError_Callback(evt_publish_t *ev);

extern QueueHandle_t xEvent_Q;

#endif