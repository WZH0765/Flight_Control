#include "Config.h"

state_t CurrentState = STATE_UNINIT;

void EnterState(state_t NewState)
{
    if(CurrentState == NewState) return;

    switch(NewState)
    {
    case STATE_CALIB:
        Calibrate_All();
        break;

    case STATE_EMERGENCY:
        MOTOR_Stop();
        break;

    case STATE_DISARMED:
        MOTOR_Stop();
        //进入待解锁前进行电调校准（仅首次）
        Calibrate_ESC();
        MotorStopCmd = 0;    //校准完成后清除停机标志，允许后续解锁
        break;

    case STATE_ARMED:
        MotorStopCmd = 0;    //已解锁，清除停机标志
        break;

    case STATE_FLYING:
        MotorStopCmd = 0;    //进入飞行，允许控制任务输出
        break;

    default:
        break;
    }
    CurrentState = NewState;
}

/**
*   状态机回调函数
**/
void EvtBus_StateMachine_CallBack(evt_publish_t *event)
{
    switch(CurrentState)
    {
    case STATE_UNINIT:
        if(event->ID == EVT_SENSORS_READY)
        {
            EnterState(STATE_CALIB);
        }
        if(event->ID == EVT_SENSORS_ERROR)
        {
            //传感器长时间未就绪，进入紧急状态
            EnterState(STATE_EMERGENCY);
        }
        break;

    case STATE_CALIB:
        if(event->ID == EVT_CALIB_DONE)
        {
            EnterState(STATE_DISARMED);
        }
        if(event->ID == EVT_IMU_ERROR)
        {
            EnterState(STATE_EMERGENCY);
        }
        break;

    case STATE_DISARMED:
        if(event->ID == EVT_ARM_GESTURE)
        {
            if(HW_AllReady())
            {
                EnterState(STATE_ARMED);
            }
        }
        break;

    case STATE_ARMED:
        if(event->ID == EVT_DISARM_GESTURE)
        {
            EnterState(STATE_DISARMED);
        }
        if(event->ID == EVT_TAKEOFF)
        {
            EnterState(STATE_FLYING);
        }
        if(event->ID == EVT_IMU_ERROR)
        {
            EnterState(STATE_EMERGENCY);
        }
        break;

    case STATE_FLYING:
        if(event->ID == EVT_DISARM_GESTURE)
        {
            EnterState(STATE_EMERGENCY);
        }
        if(event->ID == EVT_LANDED)
        {
            EnterState(STATE_ARMED);
        }
        if(event->ID == EVT_IMU_ERROR)
        {
            EnterState(STATE_EMERGENCY);
        }
        if(event->ID == EVT_MAG_ERROR || event->ID == EVT_BAR_ERROR || event->ID == EVT_GPS_ERROR)
        {
            //记录日志，通知地面站
        }
        break;

    case STATE_EMERGENCY:
        
        break;

    default:
        break;
    }
}

void StateMachine_Init(void)
{
    CurrentState = STATE_UNINIT;
}

state_t Get_CurrentState(void)
{
    return CurrentState;
}