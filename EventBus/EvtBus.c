#include "Config.h"

void EvtBus_Init(void)
{
//订阅传感器ready事件 -回调函数：EvtBus_SensorReady_Callback
    EvtBus_Subscribe(EVT_IMU_READY,EvtBus_SensorReady_Callback);
    EvtBus_Subscribe(EVT_MAG_READY,EvtBus_SensorReady_Callback);
    EvtBus_Subscribe(EVT_BAR_READY,EvtBus_SensorReady_Callback);
    EvtBus_Subscribe(EVT_GPS_READY,EvtBus_SensorReady_Callback);
//订阅传感器error事件 -回调函数：EvtBus_SensorError_Callback
    EvtBus_Subscribe(EVT_IMU_ERROR,EvtBus_SensorError_Callback);
    EvtBus_Subscribe(EVT_MAG_ERROR,EvtBus_SensorError_Callback);
    EvtBus_Subscribe(EVT_BAR_ERROR,EvtBus_SensorError_Callback);
    EvtBus_Subscribe(EVT_GPS_ERROR,EvtBus_SensorError_Callback);
//订阅系统级事件      -回调函数：EvtBus_StateMachine_CallBack
    EvtBus_Subscribe(EVT_LANDED        ,EvtBus_StateMachine_CallBack);
    EvtBus_Subscribe(EVT_TAKEOFF       ,EvtBus_StateMachine_CallBack);
    EvtBus_Subscribe(EVT_INIT_DONE     ,EvtBus_StateMachine_CallBack);
    EvtBus_Subscribe(EVT_CALIB_DONE    ,EvtBus_StateMachine_CallBack);
    EvtBus_Subscribe(EVT_ARM_GESTURE   ,EvtBus_StateMachine_CallBack);
    EvtBus_Subscribe(EVT_DISARM_GESTURE,EvtBus_StateMachine_CallBack);
}

/**
*   发布一件事到事件总线
*   发布成功 -true/发布失败 -false
**/
bool EvtBus_Publish(evt_publish_t *event)
{
    event->TimeStamp = xTaskGetTickCount();
    return xQueueSend(xEvent_Q,event,0) == pdTRUE;
}

/**
*   从事件总线订阅一件事
*   接收成功 -true/接收失败 -false
**/
uint16_t cnt = 0;                   //记录总订阅数
evt_subscribe_t Subscriber[32];     //订阅存放数组

bool EvtBus_Subscribe(evt_id_t ID,evt_callback_t Callback)
{
    Subscriber[cnt].ID = ID;
    Subscriber[cnt].Callback = Callback;
    cnt ++;
    
    return true;
}

/**
*   传感器就绪回调函数
**/
void EvtBus_SensorReady_Callback(evt_publish_t *event)
{
    (void)event;

    if(HW_AllReady() == true)
    {
        EvtBus_Publish(&(evt_publish_t){.ID = EVT_SENSORS_READY});
    }
}

void EvtBus_SensorError_Callback(evt_publish_t *event)
{
    (void)event;
}