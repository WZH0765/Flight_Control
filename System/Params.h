#ifndef __PARAMS_H
#define __PARAMS_H

#include <stdint.h>

typedef struct
{
    /* -------- 姿态环 -------- */
    struct
    {
        float Kp;
        float Ki;
        float Kd;
        float IntLimit;
        float OutLimit;
    } Angle_Roll, Angle_Pitch, Angle_Yaw;

    struct
    {
        float Kp;
        float Ki;
        float Kd;
        float IntLimit;
        float OutLimit;
    } Rate_Roll, Rate_Pitch, Rate_Yaw;

    /* -------- 高度环 -------- */
    struct
    {
        float Kp;
        float Ki;
        float Kd;
        float IntLimit;
        float OutLimit;
    } Height;

    struct
    {
        float Kp;
        float Ki;
        float Kd;
        float IntLimit;
        float OutLimit;
    } Velocity;

    /* -------- 高度控制 -------- */
    struct
    {
        float Deadzone_High;    // 油门死区上限
        float Deadzone_Low;     // 油门死区下限
        float Target_Rate;      // 油门位移 → 高度变化率（m/s per 单位油门）
        float Max_Height;       // 最大目标高度（米）
        float Min_Height;       // 最小目标高度（米）
    } Height_Control;

    /* -------- 姿态控制 -------- */
    struct
    {
        float Yaw_Scale;
        float Roll_Scale;
        float Pitch_Scale;
    } Attitude_Control;

    /* -------- 锁定手势 -------- */
    struct
    {
        uint16_t HoldTime;
        float X_Thr;
        float Throttle_Thr;
    } Lock;

    /* -------- 滤波器 -------- */
    struct
    {
        float MAG_Weight;
        float Acc_Norm_Threshold;
    } Filter;

    /* -------- 气压计 -------- */
    struct
    {
        float Cal_Threshold;
    } Baro;

} params_t;

extern params_t PARAMS;

void Params_Init(void);

#endif