#include "math.h"
#include "Filter.h"

Attitude_t Att;

void Filter_Init(float P,float I)
{
    /*归零四元数*/
    Att.Q0 = 1.0f;
    Att.Q1 = 0.0f;
    Att.Q2 = 0.0f;
    Att.Q3 = 0.0f;

    /*归零积分项*/
    Att.Int_X = 0.0f;
    Att.Int_Y = 0.0f;
    Att.Int_Z = 0.0f;

    Att.Kp = P;       //比例增益，越大越信任加速度计
    Att.Ki = I;       //积分增益，消除稳态误差
}

void Filter_Update(float Ax,float Ay,float Az,float Gx,float Gy,float Gz,float dt)
{
    //加速度归一化
    float norm = sqrtf(Ax*Ax + Ay*Ay + Az*Az);
    if(norm < 0.001f) return;
    Ax /= norm; Ay /= norm; Az /= norm;

    //提取重力向量（四元数转旋转矩阵第三列）
    float V_x = 2.0f*(Att.Q1*Att.Q3 - Att.Q0*Att.Q2);
    float V_y = 2.0f*(Att.Q0*Att.Q1 + Att.Q2*Att.Q3);
    float V_z = Att.Q0*Att.Q0 - Att.Q1*Att.Q1 - Att.Q2*Att.Q2 + Att.Q3*Att.Q3;

    //叉乘计算误差
    float Error_X = Ay*V_z - Az*V_y;
    float Error_Y = Az*V_x - Ax*V_z;
    float Error_Z = Ax*V_y - Ay*V_x;

    //积分误差
    Att.Int_X += Error_X*Att.Ki*dt;
    Att.Int_Y += Error_Y*Att.Ki*dt;
    Att.Int_Z += Error_Z*Att.Ki*dt;

    //陀螺仪补偿
    Gx += Att.Kp * Error_X + Att.Int_X;
    Gy += Att.Kp * Error_Y + Att.Int_Y;
    Gz += Att.Kp * Error_Z + Att.Int_Z;

    //四元数更新
    Att.Q0 += (-Att.Q1*Gx - Att.Q2*Gy - Att.Q3*Gz)*0.5f*dt;
    Att.Q1 += ( Att.Q0*Gx + Att.Q3*Gy - Att.Q2*Gz)*0.5f*dt;
    Att.Q2 += (-Att.Q3*Gx + Att.Q0*Gy + Att.Q1*Gz)*0.5f*dt;
    Att.Q3 += ( Att.Q2*Gx - Att.Q1*Gy + Att.Q0*Gz)*0.5f*dt;

    //四元数归一化
    norm = sqrtf(Att.Q0*Att.Q0 + Att.Q1*Att.Q1 + Att.Q2*Att.Q2 + Att.Q3*Att.Q3);
    Att.Q0 /= norm; Att.Q1 /= norm; Att.Q2 /= norm; Att.Q3 /= norm;

    //转欧拉角
    Att.Yaw   = atan2f(2.0f*(Att.Q1*Att.Q2 + Att.Q0*Att.Q3),1.0f - 2.0f*(Att.Q2*Att.Q2 + Att.Q3*Att.Q3));
    Att.Roll  = atan2f(2.0f*(Att.Q2*Att.Q3 + Att.Q0*Att.Q1),1.0f - 2.0f*(Att.Q1*Att.Q1 + Att.Q2*Att.Q2));
    Att.Pitch = asinf(2.0f*(Att.Q0*Att.Q2 - Att.Q1*Att.Q3));
}
