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

void Filter_Update(float Ax,float Ay,float Az,
                   float Gx,float Gy,float Gz,
                   float Mx,float My,float Mz,
                   float dt)
{
    float Vx,Vy,Vz;
    float Hx,Hy,Hz;
    float Wx,Wy,Wz;
    float Bx,   Bz;

    //加速度归一化
    float norm = sqrtf(Ax*Ax + Ay*Ay + Az*Az);
    float diff = fabsf(norm - 9.8f);
    if(norm < 0.001f) return;
    Ax /= norm; Ay /= norm; Az /= norm;

    //磁力计权重
    float mag_weight = 0.3f;
    if(diff > 0.5f)
    {
        mag_weight = 0.3f*(1.0f - (diff - 0.5f)/4.5f);  // 0.5~5.0 线性递减
        if(mag_weight < 0.0f) mag_weight = 0.0f;
    }

    //提取重力向量
    Vx = 2.0f*(Att.Q1*Att.Q3 - Att.Q0*Att.Q2);
    Vy = 2.0f*(Att.Q0*Att.Q1 + Att.Q2*Att.Q3);
    Vz = Att.Q0*Att.Q0 - Att.Q1*Att.Q1 - Att.Q2*Att.Q2 + Att.Q3*Att.Q3;

    //叉乘计算误差
    float Error_X = Ay*Vz - Az*Vy;
    float Error_Y = Az*Vx - Ax*Vz;
    float Error_Z = Ax*Vy - Ay*Vx;

    Hx = 2.0f*(Mx*(0.5f - Att.Q2*Att.Q2 - Att.Q3*Att.Q3) + My*(Att.Q1*Att.Q2 - Att.Q0*Att.Q3) + Mz*(Att.Q1*Att.Q3 + Att.Q0*Att.Q2));
    Hy = 2.0f*(Mx*(Att.Q1*Att.Q2 + Att.Q0*Att.Q3) + My*(0.5f - Att.Q1*Att.Q1 - Att.Q3*Att.Q3) + Mz*(Att.Q2*Att.Q3 - Att.Q0*Att.Q1));
    Hz = 2.0f*(Mx*(Att.Q1*Att.Q3 - Att.Q0*Att.Q2) + My*(Att.Q2*Att.Q3 + Att.Q0*Att.Q1) + Mz*(0.5f - Att.Q1*Att.Q1 - Att.Q2*Att.Q2));

    Bx = sqrtf(Hx*Hx + Hy*Hy);
    Bz = Hz;

    //将地磁参考向量旋转回机体坐标系
    Wx = 2.0f*(Bx*(0.5f - Att.Q2*Att.Q2 - Att.Q3*Att.Q3) + Bz*(Att.Q1*Att.Q3 - Att.Q0*Att.Q2));
    Wy = 2.0f*(Bx*(Att.Q1*Att.Q2 - Att.Q0*Att.Q3) + Bz*(Att.Q0*Att.Q1 + Att.Q2*Att.Q3));
    Wz = 2.0f*(Bx*(Att.Q0*Att.Q2 + Att.Q1*Att.Q3) + Bz*(0.5f - Att.Q1*Att.Q1 - Att.Q2*Att.Q2));

    //累计总误差
    Error_X += (My*Wz - Mz*Wy)*mag_weight;
    Error_Y += (Mz*Wx - Mx*Wz)*mag_weight;
    Error_Z += (Mx*Wy - My*Wx)*mag_weight;

    //积分误差
    Att.Int_X += Error_X*Att.Ki*dt;
    Att.Int_Y += Error_Y*Att.Ki*dt;
    Att.Int_Z += Error_Z*Att.Ki*dt;

    //陀螺仪补偿
    Gx += Att.Kp*Error_X + Att.Int_X;
    Gy += Att.Kp*Error_Y + Att.Int_Y;
    Gz += Att.Kp*Error_Z + Att.Int_Z;

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
