/*
  X型四轴电�?:M1前左,M2前右,M3后左,M4后右
*/

/****IMU define BEGIN****/
#define RAD         0.0174533f

#define ACC_SCALE   9.80f/8192.0f     //m/s^2
#define GYRO_SCALE  0.0174533f/32.8f

#define YAW_SCALE   180.0f/100.0f
#define ROLL_SCALE  45.00f/100.0f
#define PITCH_SCALE 45.00f/100.0f
/*****IMU define END*****/

/*CONTROL_TASK define BEGIN*/
#define PWM_MIN     900
#define PWM_MAX     2000

#define PID_NORM    1.0f/50.0f
#define ATT_CTRL_DT 0.001f

#define PWM_RANGE   PWM_MAX - PWM_MIN
/**CONTROL_TASK define END**/

/*LOCK_TASK define BEGIN*/
#define LOCK_HOLDTIME  1000
#define LOCK_X_THRESHOLD 80.0f
#define LOCK_THRO_THRESHOLD  0.05f
/*LOCK_TASK define END*/

#define CLAMP(value,low,high) ((value)<(low)?(low):((value)>(high)?(high):(value)))