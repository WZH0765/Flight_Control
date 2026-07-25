#include "stm32h7xx.h"                  // Device header
#include "PID.h"
#include "math.h"

/*PID(内环)-角速度*/
PID_Controller PID_Rate_Pitch;
PID_Controller PID_Rate_Roll;
PID_Controller PID_Rate_Yaw;

/*PID(外环)-角度*/
PID_Controller PID_Angle_Pitch;
PID_Controller PID_Angle_Roll;
PID_Controller PID_Angle_Yaw;

void PID_Init(void)
{
	/*INNERLOOP INIT BEGIN*/
	PID_Set_Paramaters(&PID_Rate_Pitch,0.8f,0.02f,0.05f,50.0f,50.0f);
	PID_Set_Paramaters(&PID_Rate_Roll,0.8f,0.02f,0.05f,50.0f,50.0f);
	PID_Set_Paramaters(&PID_Rate_Yaw,0.8f,0.02f,0.05f,50.0f,50.0f);
	/*INNERLOOP INIT END*/
	
	/*OUTTERLOOP INIT BEGIN*/
	PID_Set_Paramaters(&PID_Angle_Pitch,4.0f,0.0f,0.0f,80.0f,200.0f);
	PID_Set_Paramaters(&PID_Angle_Roll,4.0f,0.0f,0.0f,80.0f,200.0f);
	PID_Set_Paramaters(&PID_Angle_Yaw,4.0f,0.0f,0.0f,80.0f,200.0f);
	/*OUTTERLOOP INIT END*/
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
*/
float PID_Calculate(PID_Controller *PID,float dt)
{
	PID ->Error1 = PID ->Error0;					//更新上一次误差
	PID ->Error0 = PID ->Target - PID ->Actual;		//写入当前误差
	if(PID ->Ki != 0.0f)
	{
		PID ->ErrorInt += PID ->Error0*dt;
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
	
	PID ->Output = PID ->Kp*PID ->Error0 + PID ->Ki*PID ->ErrorInt + PID ->Kd*(PID ->Error0 - PID ->Error1)/dt;
	
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
