#ifndef FLIGHT_STATE_H
#define FLIGHT_STATE_H

#include <stdint.h>

//飞控状态
typedef enum
{
    STATE_UNINIT = 0,       //刚上电，未初始化
    STATE_CALIB,            //校准中
    STATE_DISARMED,         //已就绪，等待解锁
    STATE_ARMED,            //已解锁，待起飞
    STATE_FLYING,           //飞行中
    STATE_EMERGENCY,        //紧急状态

} fc_state_t;

//飞控事件
typedef enum
{
    EVENT_INIT_DONE,        //所有传感器初始化完成
    EVENT_CALIB_DONE,       //校准完成
    EVENT_ARM_GESTURE,      //外八解锁手势
    EVENT_DISARM_GESTURE,   //内八上锁手势
    EVENT_TAKEOFF,          //起飞
    EVENT_LANDED,           //已落地
    EVENT_IMU_ERROR,      //IMU错误
    EVENT_RC_LOST,          //RC 信号丢失
    EVENT_RC_RECOVER,       //RC 信号恢复

} fc_event_t;

void FC_InitState(void);
fc_state_t FC_GetState(void);
void FC_HandleEvent(fc_event_t Event);

#endif