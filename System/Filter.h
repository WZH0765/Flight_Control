#ifndef _FILTER_H_
#define _FILTER_H_

//姿态估计结构体
typedef struct
{
    float Yaw;
    float Roll;
    float Pitch;

    float Kp;
    float Ki;

    float Q0,Q1,Q2,Q3;          //四元数
    float Int_X,Int_Y,Int_Z;    //积分误差

} Attitude_t;

extern Attitude_t Att;

void Filter_Init(float P,float I);
void Filter_Update(float Ax,float Ay,float Az,float Gx,float Gy,float Gz,float dt);

#endif