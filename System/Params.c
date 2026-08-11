#include "Params.h"
#include "Config.h"

// 全局参数
params_t PARAMS = {0};

void Params_Init(void)
{
    /* -------- 姿态环-外环 -------- */
    PARAMS.Angle_Roll.Kp       = 4.0f;
    PARAMS.Angle_Roll.Ki       = 0.0f;
    PARAMS.Angle_Roll.Kd       = 0.0f;
    PARAMS.Angle_Roll.IntLimit = 80.0f;
    PARAMS.Angle_Roll.OutLimit = 30.0f;

    PARAMS.Angle_Pitch.Kp       = 4.0f;
    PARAMS.Angle_Pitch.Ki       = 0.0f;
    PARAMS.Angle_Pitch.Kd       = 0.0f;
    PARAMS.Angle_Pitch.IntLimit = 80.0f;
    PARAMS.Angle_Pitch.OutLimit = 30.0f;

    PARAMS.Angle_Yaw.Kp       = 4.0f;
    PARAMS.Angle_Yaw.Ki       = 0.0f;
    PARAMS.Angle_Yaw.Kd       = 0.0f;
    PARAMS.Angle_Yaw.IntLimit = 80.0f;
    PARAMS.Angle_Yaw.OutLimit = 30.0f;

    /* -------- 姿态环-内环 -------- */
    PARAMS.Rate_Roll.Kp       = 0.8f;
    PARAMS.Rate_Roll.Ki       = 0.02f;
    PARAMS.Rate_Roll.Kd       = 0.05f;
    PARAMS.Rate_Roll.IntLimit = 50.0f;
    PARAMS.Rate_Roll.OutLimit = 50.0f;

    PARAMS.Rate_Pitch.Kp       = 0.8f;
    PARAMS.Rate_Pitch.Ki       = 0.02f;
    PARAMS.Rate_Pitch.Kd       = 0.05f;
    PARAMS.Rate_Pitch.IntLimit = 50.0f;
    PARAMS.Rate_Pitch.OutLimit = 50.0f;

    PARAMS.Rate_Yaw.Kp       = 0.8f;
    PARAMS.Rate_Yaw.Ki       = 0.02f;
    PARAMS.Rate_Yaw.Kd       = 0.05f;
    PARAMS.Rate_Yaw.IntLimit = 50.0f;
    PARAMS.Rate_Yaw.OutLimit = 50.0f;

    /* -------- 高度环-外环 -------- */
    PARAMS.Height.Kp       = 1.5f;
    PARAMS.Height.Ki       = 0.0f;
    PARAMS.Height.Kd       = 0.0f;
    PARAMS.Height.IntLimit = 10.0f;
    PARAMS.Height.OutLimit = 5.0f;

    /* -------- 高度环-内环 -------- */
    PARAMS.Velocity.Kp       = 0.8f;
    PARAMS.Velocity.Ki       = 0.0f;
    PARAMS.Velocity.Kd       = 0.0f;
    PARAMS.Velocity.IntLimit = 5.0f;
    PARAMS.Velocity.OutLimit = 0.25f;

    /* -------- 高度控制参数 -------- */
    PARAMS.Height_Control.Deadzone_High = 0.55f;
    PARAMS.Height_Control.Deadzone_Low  = 0.45f;
    PARAMS.Height_Control.Target_Rate   = 0.3f;
    PARAMS.Height_Control.Max_Height    = 50.0f;
    PARAMS.Height_Control.Min_Height    = 0.0f;

    /* -------- 姿态控制参数 -------- */
    PARAMS.Attitude_Control.Yaw_Scale   = 45.0f/100.0f;
    PARAMS.Attitude_Control.Roll_Scale  = 45.0f/100.0f;
    PARAMS.Attitude_Control.Pitch_Scale = 45.0f/100.0f;

    /* -------- 锁定手势 -------- */
    PARAMS.Lock.HoldTime        = 1000;
    PARAMS.Lock.X_Threshold     = 80.0f;
    PARAMS.Lock.Thro_Threshold  = 0.05f;

    /* -------- 滤波器 -------- */
    PARAMS.Filter.MAG_Weight           = 0.3f;
    PARAMS.Filter.Acc_Norm_Threshold   = 0.001f;

    /* -------- 气压计 -------- */
    PARAMS.Baro.Calib_Threshold    = 1.0f;
}