#ifndef _PID__H__
#define _PID__H__

typedef struct
{
	float Target;
    float Actual;
    float Error0;
	float Error1;
	float ErrorInt;
    float Kp,Ki,Kd;
    float Output;
	
	float Int_Limit;
	float Out_Limit;
	
}PID_Controller;

void PID_Init(void);
float PID_Calculate(PID_Controller *PID,float dt);
void PID_Set_Paramaters(PID_Controller *PID,float Kp,float Ki,float Kd,float Int_Limit,float Out_Limit);

extern PID_Controller PID_Rate_Pitch;
extern PID_Controller PID_Rate_Roll;
extern PID_Controller PID_Rate_Yaw;

extern PID_Controller PID_Angle_Pitch;
extern PID_Controller PID_Angle_Roll;
extern PID_Controller PID_Angle_Yaw;

#endif
