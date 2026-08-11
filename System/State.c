#include "state.h"
#include "Lock.h"
#include "Error.h"
#include "Filter.h"
#include "IMU.h"
#include "BAR.h"
#include "MAG.h"
#include "GPS.h"
#include "tim.h"
#include <stdio.h>

//设置初始状态 - UnInit
fc_state_t CurrentState = STATE_UNINIT;

//设置新状态
static void FC_EnterState(fc_state_t NewState)
{
    if(CurrentState != NewState)
    {
        switch(NewState)
        {
            case STATE_EMERGENCY:
                Sys_LockState.LockState = 1;
                __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,PWM_MIN);
                __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,PWM_MIN);
                __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,PWM_MIN);
                __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,PWM_MIN);
                break;
            case STATE_DISARMED:
                Sys_LockState.LockState = 1;
                break;
            case STATE_FLYING:

                break;
            default:
                break;
        }
        CurrentState = NewState;
    }
    else
    {
        return ;
    }
}

//事件处理
void FC_HandleEvent(fc_event_t Event)
{
    switch(CurrentState)
    {
        case STATE_UNINIT:
            if(Event == EVENT_INIT_DONE)
            {
                FC_EnterState(STATE_CALIB);
            }
            break;

        case STATE_CALIB:
            if (Event == EVENT_CALIB_DONE)
            {
                FC_EnterState(STATE_DISARMED);
            }
            break;

        case STATE_DISARMED:
            if(Event == EVENT_ARM_GESTURE)
            {
                if(HW_Unlock())
                {
                    FC_EnterState(STATE_ARMED);
                }
            }
            break;

        case STATE_ARMED:
            if(Event == EVENT_DISARM_GESTURE)
            {
                FC_EnterState(STATE_DISARMED);
            }
            if(Event == EVENT_TAKEOFF)
            {
                FC_EnterState(STATE_FLYING);
            }
            break;

        case STATE_FLYING:
            if(Event == EVENT_DISARM_GESTURE)
            {
                FC_EnterState(STATE_EMERGENCY);
            }
            if(Event == EVENT_LANDED)
            {
                FC_EnterState(STATE_ARMED);
            }
            if(Event == EVENT_IMU_ERROR)
            {
                FC_EnterState(STATE_EMERGENCY);
            }
            break;

        case STATE_EMERGENCY:
            if(Event == EVENT_RC_RECOVER)
            {

            }
            break;

        default:
            break;
    }
}

void FC_InitState(void)
{
    CurrentState = STATE_UNINIT;
}

fc_state_t FC_GetState(void)
{
    return CurrentState;
}
