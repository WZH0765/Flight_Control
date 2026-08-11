/*
  X型四轴电机:M1前左,M2前右,M3后左,M4后右
*/

/****IMU define BEGIN****/
#define RAD         0.0174533f

#define ACC_SCALE   9.80f/8192.0f     //m/s^2
#define GYRO_SCALE  0.0174533f/32.8f
/*****IMU define END*****/

/****MAG define BEGIN****/
#define MAG_ID         0x10

#define MAG_ID_REG     0x00
#define MAG_STAT_REG   0x02
#define MAG_CTRL_REG   0x0A

#define MAG_DATA_L     0x03
#define MAG_CTRL_SRST  0x04
#define MAG_CTRL_MODE  0x10

#define MAG_SCALE      0.3f
/*****MAG define END*****/

/*CONTROL_TASK define BEGIN*/
#define PWM_MIN     900
#define PWM_MAX     2000

#define PID_NORM    1.0f/50.0f


#define PWM_RANGE   PWM_MAX - PWM_MIN
/**CONTROL_TASK define END**/

/*LOCK_TASK define BEGIN*/
#define LOCK_HOLDTIME        1000
#define LOCK_X_THRESHOLD     80.0f
#define LOCK_THRO_THRESHOLD  0.05f
/*LOCK_TASK define END*/

#define LOG_BUF_SIZE      128

/*GPS_TASK define BEGIN*/
#define GPS_LINE_MAX_LEN   100     //NMEA语句最大长度(含$和校验)

//定位状态
#define GPS_FIX_2D         1       //2D定位
#define GPS_FIX_3D         2       //3D定位
#define GPS_FIX_INVALID    0       //无定位
/*GPS_TASK define END*/

#define CLAMP(value,low,high) ((value)<(low)?(low):((value)>(high)?(high):(value)))

//对于I2C轮询模块
#define MAG_OK      0
#define MAG_BUSY    1
#define MAG_ERROR  -1

//任务周期 DT
#define SEN_READ_DT 10
#define MAG_READ_DT 5
#define BAR_READ_DT 2
#define POS_ESTI_DT 20
#define ATT_CTRL_DT 1
#define RC_PARSE_DT 500
#define LOG_WRITE_DT 100

//传感器超时阈值
#define RC_TIMEOUT_THRESHOLD  5
#define IMU_TIMEOUT_THRESHOLD 500
#define GPS_TIMEOUT_THRESHOLD 500
#define MAG_TIMEOUT_THRESHOLD 20 
#define BAR_TIMEOUT_THRESHOLD 50