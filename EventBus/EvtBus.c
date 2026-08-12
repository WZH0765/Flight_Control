#include "Config.h"

uint16_t cnt = 0;                   //记录总订阅数
evt_subscribe_t Subscriber[32];     //订阅存放数组

//传感器连续错误阈值：达到后触发更高层级故障
#define SENSOR_RETRY_THRESHOLD  3

//传感器连续错误计数
static uint16_t Mag_ErrorCnt = 0;
static uint16_t Bar_ErrorCnt = 0;
static uint16_t Gps_ErrorCnt = 0;

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
//订阅传感器error事件 -回调函数：EvtBus_StateMachine_CallBack（IMU故障进入紧急状态）
    EvtBus_Subscribe(EVT_IMU_ERROR,EvtBus_StateMachine_CallBack);
    EvtBus_Subscribe(EVT_MAG_ERROR,EvtBus_StateMachine_CallBack);
    EvtBus_Subscribe(EVT_BAR_ERROR,EvtBus_StateMachine_CallBack);
    EvtBus_Subscribe(EVT_GPS_ERROR,EvtBus_StateMachine_CallBack);
//订阅系统级事件      -回调函数：EvtBus_StateMachine_CallBack
    EvtBus_Subscribe(EVT_LANDED        ,EvtBus_StateMachine_CallBack);
    EvtBus_Subscribe(EVT_TAKEOFF       ,EvtBus_StateMachine_CallBack);
    EvtBus_Subscribe(EVT_INIT_DONE     ,EvtBus_StateMachine_CallBack);
    EvtBus_Subscribe(EVT_CALIB_DONE    ,EvtBus_StateMachine_CallBack);
    EvtBus_Subscribe(EVT_ARM_GESTURE   ,EvtBus_StateMachine_CallBack);
    EvtBus_Subscribe(EVT_DISARM_GESTURE,EvtBus_StateMachine_CallBack);
    EvtBus_Subscribe(EVT_SENSORS_ERROR ,EvtBus_StateMachine_CallBack);
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
bool EvtBus_Subscribe(evt_id_t ID,evt_callback_t Callback)
{
    //订阅表已满则拒绝
    if(cnt >= (sizeof(Subscriber)/sizeof(Subscriber[0]))) return false;

    Subscriber[cnt].ID = ID;
    Subscriber[cnt].Callback = Callback;
    cnt ++;
    
    return true;
}

/**
*   事件分发：根据事件ID遍历订阅表并执行对应回调
*   由 Task_EvtBus_Handler 任务调用
*   @param event: 待分发的事件
**/
void EvtBus_Dispatch(evt_publish_t *event)
{
    if(event == NULL) return;

    for(uint16_t i = 0;i < cnt;i ++)
    {
        if(Subscriber[i].ID == event->ID)
        {
            Subscriber[i].Callback(event);
        }
    }
}

/**
*   传感器就绪回调函数
*   同步硬件表就绪状态，单次就绪即清除对应错误计数
**/
void EvtBus_SensorReady_Callback(evt_publish_t *event)
{
    if(event->ID == EVT_IMU_READY)
    {
        HW_SetReady(HW_IMU);
    }
    else if(event->ID == EVT_MAG_READY)
    {
        HW_SetReady(HW_MAG);
        Mag_ErrorCnt = 0;
    }
    else if(event->ID == EVT_BAR_READY)
    {
        HW_SetReady(HW_BAR);
        Bar_ErrorCnt = 0;
    }
    else if(event->ID == EVT_GPS_READY)
    {
        HW_SetReady(HW_GPS);
        Gps_ErrorCnt = 0;
    }

    if(HW_AllReady() == true)
    {
        EvtBus_Publish(&(evt_publish_t){.ID = EVT_SENSORS_READY});
    }
}

/**
*   传感器错误回调函数（分级处理）
*   IMU故障        → 标记未就绪，交由状态机进入紧急状态
*   MAG/BAR/GPS故障 → 标记未就绪并尝试重新初始化，连续失败触发更高层级故障
**/
void EvtBus_SensorError_Callback(evt_publish_t *event)
{
    switch(event->ID)
    {
    case EVT_IMU_ERROR:
        //IMU为飞行安全关键传感器，标记未就绪，由状态机进入紧急状态
        HW_SetUnready(HW_IMU);
        break;

    case EVT_MAG_ERROR:
        HW_SetUnready(HW_MAG);
        if((++ Mag_ErrorCnt) >= SENSOR_RETRY_THRESHOLD)
        {
            //连续失败，上报更高层级故障
            EvtBus_Publish(&(evt_publish_t){.ID = EVT_SENSORS_ERROR});
            Mag_ErrorCnt = 0;
        }
        else
        {
            //尝试重新初始化
            MAG_Init();
        }
        break;

    case EVT_BAR_ERROR:
        HW_SetUnready(HW_BAR);
        if((++ Bar_ErrorCnt) >= SENSOR_RETRY_THRESHOLD)
        {
            EvtBus_Publish(&(evt_publish_t){.ID = EVT_SENSORS_ERROR});
            Bar_ErrorCnt = 0;
        }
        else
        {
            BAR_Init();
        }
        break;

    case EVT_GPS_ERROR:
        HW_SetUnready(HW_GPS);
        if((++ Gps_ErrorCnt) >= SENSOR_RETRY_THRESHOLD)
        {
            EvtBus_Publish(&(evt_publish_t){.ID = EVT_SENSORS_ERROR});
            Gps_ErrorCnt = 0;
        }
        else
        {
            GPS_Init();
        }
        break;

    default:
        break;
    }
}