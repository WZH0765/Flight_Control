#include "stm32h7xx.h"                  // Device header
#include "PID.h"
#include "math.h"
#include "Params.h"
#include <stdint.h>

/*PID(内环)-角速度*/
PID_Controller PID_Rate_Pitch;
PID_Controller PID_Rate_Roll;
PID_Controller PID_Rate_Yaw;

/*PID(外环)-角度*/
PID_Controller PID_Angle_Pitch;
PID_Controller PID_Angle_Roll;
PID_Controller PID_Angle_Yaw;

/*PID（内环）-速度*/
PID_Controller PID_Velocity;

/*PID（外环）-高度*/
PID_Controller PID_Altitude;

void PID_Init(void)
{
    PID_UpdateParams();
}

/**
*   运行时更新PID参数：从PARAMS重新读取所有系数并下发
*   供遥控器/地面站修改参数后实时生效，无需重新上电
**/
void PID_UpdateParams(void)
{
	PID_Set_Paramaters(&PID_Rate_Roll,
        				PARAMS.Rate_Roll.Kp,
        				PARAMS.Rate_Roll.Ki,
       					PARAMS.Rate_Roll.Kd,
        				PARAMS.Rate_Roll.IntLimit,
        				PARAMS.Rate_Roll.OutLimit);

    PID_Set_Paramaters(&PID_Rate_Pitch,
        				PARAMS.Rate_Pitch.Kp,
        				PARAMS.Rate_Pitch.Ki,
						PARAMS.Rate_Pitch.Kd,
						PARAMS.Rate_Pitch.IntLimit,
						PARAMS.Rate_Pitch.OutLimit);

    PID_Set_Paramaters(&PID_Rate_Yaw,
						PARAMS.Rate_Yaw.Kp,
						PARAMS.Rate_Yaw.Ki,
						PARAMS.Rate_Yaw.Kd,
						PARAMS.Rate_Yaw.IntLimit,
						PARAMS.Rate_Yaw.OutLimit);

    PID_Set_Paramaters(&PID_Angle_Roll,
						PARAMS.Angle_Roll.Kp,
						PARAMS.Angle_Roll.Ki,
						PARAMS.Angle_Roll.Kd,
						PARAMS.Angle_Roll.IntLimit,
						PARAMS.Angle_Roll.OutLimit);
	
	PID_Set_Paramaters(&PID_Angle_Pitch,
						PARAMS.Angle_Pitch.Kp,
						PARAMS.Angle_Pitch.Ki,
						PARAMS.Angle_Pitch.Kd,
						PARAMS.Angle_Pitch.IntLimit,
						PARAMS.Angle_Pitch.OutLimit);

	PID_Set_Paramaters(&PID_Angle_Yaw,
						PARAMS.Angle_Yaw.Kp,
						PARAMS.Angle_Yaw.Ki,
						PARAMS.Angle_Yaw.Kd,
						PARAMS.Angle_Yaw.IntLimit,
						PARAMS.Angle_Yaw.OutLimit);

    PID_Set_Paramaters(&PID_Altitude,
						PARAMS.Height.Kp,
						PARAMS.Height.Ki,
						PARAMS.Height.Kd,
						PARAMS.Height.IntLimit,
						PARAMS.Height.OutLimit);

    PID_Set_Paramaters(&PID_Velocity,
						PARAMS.Velocity.Kp,
						PARAMS.Velocity.Ki,
						PARAMS.Velocity.Kd,
						PARAMS.Velocity.IntLimit,
						PARAMS.Velocity.OutLimit);
}

/*设置PID参数值*/
void PID_Set_Paramaters(PID_Controller *PID,float Kp,float Ki,float Kd,float Int_Limit,float Out_Limit)
{
	PID -> Kp = Kp;
	PID -> Ki = Ki;
	PID -> Kd = Kd;
	PID -> Int_Limit = Int_Limit;
	PID -> Out_Limit = Out_Limit;
	
	/*RESET OTHER VUALS*/
	PID ->ErrorInt = 0;
	PID ->Error0   = 0;
    PID ->Error1   = 0;
    PID ->Output   = 0;
}

/*
	PID计算
	内外环PID控制
	传入时间 ms
*/
float PID_Calculate(PID_Controller *PID,uint8_t dt)
{
	PID ->Error1 = PID ->Error0;					//更新上一次误差
	PID ->Error0 = PID ->Target - PID ->Actual;		//写入当前误差
	if(PID ->Ki != 0.0f)
	{
		PID ->ErrorInt += PID ->Error0*dt/1000;
		/*积分限幅*/
		if(PID ->ErrorInt > PID ->Int_Limit)
		{
			PID ->ErrorInt = PID ->Int_Limit;
		}
		else if(PID ->ErrorInt < -PID ->Int_Limit)
		{
			PID ->ErrorInt = -PID ->Int_Limit;
		}
	}
	else
	{
		PID ->ErrorInt = 0;
	}
	
	PID ->Output = PID ->Kp*PID ->Error0 + PID ->Ki*PID ->ErrorInt + PID ->Kd*(PID ->Error0 - PID ->Error1)/dt/1000;
	
	if(PID ->Output >= PID ->Out_Limit)
	{
		PID ->Output = PID ->Out_Limit;
	}
	else if(PID ->Output <= -PID ->Out_Limit)
	{
		PID ->Output = -PID ->Out_Limit;
	}
	return PID ->Output;
}
