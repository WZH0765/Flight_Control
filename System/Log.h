#ifndef __LOG_H__
#define __LOG_H__

typedef struct
{
    float Ax;
    float Ay;
    float Az;
    float Gx;
    float Gy;
    float Gz;

    float Yaw;
    float Roll;
    float Pitch;

    /*内环目标值*/
    float RateYawTarget;
    float RateRollTarget;
    float RatePitchTarget;

    /*外环目标值*/
    float AngleYawTarget;
    float AngleRollTarget;
    float AnglePitchTarget;

    /*PID输出*/
    float OutYaw;
    float OutRoll;
    float OutPitch;

    /*PWM输出*/
    uint16_t Pwm1;
    uint16_t Pwm2;
    uint16_t Pwm3;
    uint16_t Pwm4;

    /*油门*/
    float Throttle;

    /*时间戳*/
    uint32_t TimeStamp;

} log_data_t;

#endif
