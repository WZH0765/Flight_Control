#ifndef STATE_H
#define STATE_H

#include "EvtBus.h"

typedef enum
{
    STATE_UNINIT,    //上电
    STATE_CALIB,     //校准
    STATE_DISARMED,  //待解锁
    STATE_ARMED,     //已解锁
    STATE_FLYING,    //飞行中
    STATE_EMERGENCY  //紧急

} state_t;

void StateMachine_Init(void);
state_t Get_CurrentState(void);
void EvtBus_StateMachine_CallBack(evt_publish_t *event);

#endif