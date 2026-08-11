/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * Author          : 王子恒
  * Description     : 飞控控制逻辑
  ******************************************************************************
**/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "projdefs.h"
#include "state.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "inv_imu_driver.h"
#include "Receiver.h"
#include "lps22hh.h"
#include <stdint.h>
#include <string.h>
#include "semphr.h"
#include "Filter.h"
#include "Config.h"
#include "Params.h"
#include "usart.h"
#include "Error.h"
#include "fatfs.h"
#include "State.h"
#include "iwdg.h"
#include "Lock.h"
#include "IMU.h"
#include "BAR.h"
#include "MAG.h"
#include "GPS.h"
#include "PID.h"
#include "tim.h"
#include "Log.h"
#include "ff.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//数据就绪信号量
SemaphoreHandle_t xGPS_DataReady;
SemaphoreHandle_t xIMU_DataReady;
SemaphoreHandle_t xRC_DataReady;

//数据队列
QueueHandle_t     xLOG_DataQ;
QueueHandle_t     xGPS_DataQ;
QueueHandle_t     xIMU_DataQ;
QueueHandle_t     xMAG_DataQ;
QueueHandle_t     xBAR_DataQ;
QueueHandle_t     xRC_DataQ;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define MOTOR_STOP() do\
{\
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,PWM_MIN); \
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,PWM_MIN); \
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,PWM_MIN); \
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,PWM_MIN); \
} while(0)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

rc_raw_t RcRaw = {0};

float HeightCorr = 0;

//超时计数器
static uint16_t Rc_Timeout  = 0;
static uint16_t Gps_Timeout = 0;
static uint16_t Imu_Timeout = 0;
static uint16_t Mag_Timeout = 0;
static uint16_t Bar_Timeout = 0;

/* USER CODE END Variables */
/* Definitions for Task_Imu_Rd */
osThreadId_t Task_Imu_RdHandle;
const osThreadAttr_t Task_Imu_Rd_attributes = {
  .name = "Task_Imu_Rd",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Task_Sen_Rd */
osThreadId_t Task_Sen_RdHandle;
const osThreadAttr_t Task_Sen_Rd_attributes = {
  .name = "Task_Sen_Rd",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_Pos_Est */
osThreadId_t Task_Pos_EstHandle;
const osThreadAttr_t Task_Pos_Est_attributes = {
  .name = "Task_Pos_Est",
  .stack_size = 1536 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Task_Att_Ctrl */
osThreadId_t Task_Att_CtrlHandle;
const osThreadAttr_t Task_Att_Ctrl_attributes = {
  .name = "Task_Att_Ctrl",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Task_Rc_Parse */
osThreadId_t Task_Rc_ParseHandle;
const osThreadAttr_t Task_Rc_Parse_attributes = {
  .name = "Task_Rc_Parse",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_Log_Wrt */
osThreadId_t Task_Log_WrtHandle;
const osThreadAttr_t Task_Log_Wrt_attributes = {
  .name = "Task_Log_Wrt",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Task_Sys_Obs */
osThreadId_t Task_Sys_ObsHandle;
const osThreadAttr_t Task_Sys_Obs_attributes = {
  .name = "Task_Sys_Obs",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Imu_Read(void *argument);
void Sen_Read(void *argument);
void Pos_Estimate(void *argument);
void Att_Control(void *argument);
void Rc_Parse(void *argument);
void Log_Write(void *argument);
void Sys_Observe(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */

  //创建信号量
  xRC_DataReady  = xSemaphoreCreateBinary();
  xIMU_DataReady = xSemaphoreCreateBinary();
  xGPS_DataReady = xSemaphoreCreateBinary();

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */

  //创建队列
  xRC_DataQ  = xQueueCreate(1,sizeof(rc_data_t));
  xIMU_DataQ = xQueueCreate(1,sizeof(imu_raw_t));
  xMAG_DataQ = xQueueCreate(1,sizeof(mag_data_t));
  xGPS_DataQ = xQueueCreate(1,sizeof(gps_data_t));
  xBAR_DataQ = xQueueCreate(1,sizeof(bar_data_t));
  xLOG_DataQ = xQueueCreate(256,sizeof(log_data_t));

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task_Imu_Rd */
  Task_Imu_RdHandle = osThreadNew(Imu_Read, NULL, &Task_Imu_Rd_attributes);

  /* creation of Task_Sen_Rd */
  Task_Sen_RdHandle = osThreadNew(Sen_Read, NULL, &Task_Sen_Rd_attributes);

  /* creation of Task_Pos_Est */
  Task_Pos_EstHandle = osThreadNew(Pos_Estimate, NULL, &Task_Pos_Est_attributes);

  /* creation of Task_Att_Ctrl */
  Task_Att_CtrlHandle = osThreadNew(Att_Control, NULL, &Task_Att_Ctrl_attributes);

  /* creation of Task_Rc_Parse */
  Task_Rc_ParseHandle = osThreadNew(Rc_Parse, NULL, &Task_Rc_Parse_attributes);

  /* creation of Task_Log_Wrt */
  Task_Log_WrtHandle = osThreadNew(Log_Write, NULL, &Task_Log_Wrt_attributes);

  /* creation of Task_Sys_Obs */
  Task_Sys_ObsHandle = osThreadNew(Sys_Observe, NULL, &Task_Sys_Obs_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Imu_Read */
/**
  * @brief  IMU数据处理函数
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Imu_Read */
void Imu_Read(void *argument)
{
  /* USER CODE BEGIN Imu_Read */
  (void)argument;

  uint16_t  Cnt = 0;
  imu_raw_t ImuRaw = {0};

  for(;;)
  {
    int32_t AccSum[3]  = {0};
    int32_t GyroSum[3] = {0};

    xSemaphoreTake(xIMU_DataReady,portMAX_DELAY);

    //获取字节
    inv_imu_get_frame_count(&IMU,&Cnt);

    //转换为帧
    Cnt = Cnt/16;
    if(Cnt < 3 || Cnt > 10)
    {
      inv_imu_flush_fifo(&IMU);
      continue;
    }

    for(int i = 0;i < Cnt;i ++)
    {
      inv_imu_get_fifo_frame(&IMU,&FIFO_Data);

      AccSum[0] += FIFO_Data.byte_16.accel_data[0];
      AccSum[1] += FIFO_Data.byte_16.accel_data[1];
      AccSum[2] += FIFO_Data.byte_16.accel_data[2];

      GyroSum[0] += FIFO_Data.byte_16.gyro_data[0];
      GyroSum[1] += FIFO_Data.byte_16.gyro_data[1];
      GyroSum[2] += FIFO_Data.byte_16.gyro_data[2];
    }

    //更新原数
    ImuRaw.Acc[0] = AccSum[0]/Cnt;
    ImuRaw.Acc[1] = AccSum[1]/Cnt;
    ImuRaw.Acc[2] = AccSum[2]/Cnt;

    ImuRaw.Gyro[0] = GyroSum[0]/Cnt;
    ImuRaw.Gyro[1] = GyroSum[1]/Cnt;
    ImuRaw.Gyro[2] = GyroSum[2]/Cnt;

    xQueueOverwrite(xIMU_DataQ,&ImuRaw);
  }
  /* USER CODE END Imu_Read */
}

/* USER CODE BEGIN Header_Sen_Read */
/**
* @brief  sensor数据处理函数
* @param  argument: Not used
* @retval None
*/
/* USER CODE END Header_Sen_Read */
void Sen_Read(void *argument)
{
  /* USER CODE BEGIN Sen_Read */
  (void)argument;

  uint8_t Magcnt = 0;
  uint8_t Barcnt = 0;

  TickType_t xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {
    vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(SEN_READ_DT));

//读取GPS数据并入队
    if(xSemaphoreTake(xGPS_DataReady,0) == pdTRUE)
    {
      Gps_Timeout = 0;
      GPS_Parse(GPS_RxBuffer,GPS_RxLength);
      GPS_Init();
    }
    else
    {
      Gps_Timeout ++;
      if(Gps_Timeout > GPS_TIMEOUT_THRESHOLD)
      {
        Error_Code.GPS_Error = 1;
      }
    }

//读取磁力计数据并入队
    if((++ Magcnt) >= MAG_READ_DIV)
    {
      int result = MAG_Parse();
      if(result == MAG_OK)
      {
        Mag_Timeout = 0;
        Error_Code.MAG_Error = 0;
      }
      else if(result == MAG_BUSY)
      {
        if((++ Mag_Timeout) >= MAG_TIMEOUT_THRESHOLD)
        {
          Error_Code.MAG_Error = 1;
        }
      }
      else
      {
        Error_Code.MAG_Error = 1;
      }
      Magcnt = 0;
    }

//读取气压计数据并入队
    if((++ Barcnt) >= BAR_READ_DIV)
    {
      if(BAR_Read() == LPS22HH_OK)
      {
        Bar_Timeout = 0;
        Error_Code.BAR_Error = 0;
      }
      else
      {
        if((++ Bar_Timeout) >= BAR_TIMEOUT_THRESHOLD)
        {
          Error_Code.BAR_Error = 1;
        }
      }
      Barcnt = 0;
    }

//添加其它传感器

  }
  /* USER CODE END Sen_Read */
}

/* USER CODE BEGIN Header_Pos_Estimate */
/**
* @brief  系统位置数据处理函数
* @param  argument: Not used
* @retval None
*/
/* USER CODE END Header_Pos_Estimate */
void Pos_Estimate(void *argument)
{
  /* USER CODE BEGIN Pos_Estimate */
  (void)argument;

  rc_data_t  RcData;
  bar_data_t BarData;
  fc_state_t CurrentState;

  float HomeAlt    = 0.0f;    //基准高度（绝对）
  float TargetAlt  = 0.0f;    //目标高度（相对）
  float CurrentAlt = 0.0f;    //当前高度（相对）

  uint8_t HomeSet  = 0;       //是否完成校准

  TickType_t xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {
    vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(POS_ESTI_DT));

    if(xQueuePeek(xBAR_DataQ,&BarData,0) != pdTRUE) continue;

    CurrentState = FC_GetState();

    if(HomeSet == 0 && CurrentState == STATE_CALIB)
    {
      float Sum = 0.0f;
      uint8_t j = 0;
      
      for(int i = 0; i < 50; i++)
      {
        if(xQueuePeek(xBAR_DataQ, &BarData, 0) == pdTRUE)
        {
          Sum += BarData.Height;
          j++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      if(j > 0)
      {
        HomeAlt = Sum / j;
        HomeSet = 1;
        FC_HandleEvent(EVENT_CALIB_DONE);
      }
      else
      {
        HomeSet = 0;
      }
    }
    if(HomeSet == 1)
    {
      if (xQueuePeek(xBAR_DataQ, &BarData, 0) == pdTRUE)
      {
        CurrentAlt = BarData.Height - HomeAlt;
      }
    }

    if(CurrentState == STATE_FLYING)
    {
      if(xQueuePeek(xRC_DataQ,&RcData,0) == pdTRUE)
      {
        float Throttle = RcData.Throttle;
        //上升
        if(Throttle > PARAMS.Height_Control.Deadzone_High && TargetAlt < PARAMS.Height_Control.Max_Height)
        {
          TargetAlt += (Throttle - 0.5f) * PARAMS.Height_Control.Target_Rate;
        }
        //下降
        else if(Throttle < PARAMS.Height_Control.Deadzone_Low && TargetAlt > PARAMS.Height_Control.Min_Height)
        {
          TargetAlt += (Throttle - 0.5f) * PARAMS.Height_Control.Target_Rate;
        }
        //保持
        else
        {
          TargetAlt = CurrentAlt;
        }
        TargetAlt = CLAMP(TargetAlt,PARAMS.Height_Control.Min_Height,PARAMS.Height_Control.Max_Height);
      }

      PID_Altitude.Target = TargetAlt;
      PID_Altitude.Actual = CurrentAlt;
      float TargetSpeed = PID_Calculate(&PID_Altitude,POS_ESTI_DT);

      PID_Velocity.Target = TargetSpeed;
      PID_Velocity.Actual = 0.0f;
      HeightCorr = PID_Calculate(&PID_Velocity,POS_ESTI_DT);

      HeightCorr = CLAMP(HeightCorr,-PARAMS.Velocity.OutLimit,PARAMS.Velocity.OutLimit);
    }
    else
    {
      HeightCorr = 0.0f;
    }
  }
  /* USER CODE END Pos_Estimate */
}

/* USER CODE BEGIN Header_Att_Control */
/**
  * @brief  系统总体数据处理函数（状态机驱动）
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Att_Control */
void Att_Control(void *argument)
{
  /* USER CODE BEGIN Att_Control */
  (void)argument;

  rc_data_t  RcData  = {0};     //RC 缩放数据
  imu_raw_t  ImuRaw  = {0};     //IMU原始数据
  imu_data_t ImuData = {0};     //IMU缩放数据
  mag_data_t MagData = {0};     //MAG缩放数据
  fc_state_t CurrentState ;

  TickType_t xCurrentTime;
  TickType_t xLastWakeTime = xTaskGetTickCount();     //上次任务唤醒时刻

  for(;;)
  {
    //固定1KHz调度
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    xCurrentTime = xTaskGetTickCount();

    //获取当前飞控状态
    CurrentState = FC_GetState();

    switch(CurrentState)
    {
      case STATE_FLYING:      //正常飞行中
      {
        if(xQueueReceive(xIMU_DataQ,&ImuRaw,0) == pdTRUE)       //获取IMU数据
        {
          Imu_Timeout = 0;

          //原始数据缩放为物理量
          ImuData.Ax = ImuRaw.Acc[0]*ACC_SCALE;
          ImuData.Ay = ImuRaw.Acc[1]*ACC_SCALE;
          ImuData.Az = ImuRaw.Acc[2]*ACC_SCALE;

          ImuData.Gx = ImuRaw.Gyro[0]*GYRO_SCALE;
          ImuData.Gy = ImuRaw.Gyro[1]*GYRO_SCALE;
          ImuData.Gz = ImuRaw.Gyro[2]*GYRO_SCALE;

          if(xQueuePeek(xMAG_DataQ,&MagData,0) != pdTRUE)      //获取MAG数据
          {
            MagData.Mx = 0.0f;
            MagData.My = 0.0f;
            MagData.Mz = 0.0f;
          }

          //姿态解算
          Filter_Update(ImuData.Ax, ImuData.Ay, ImuData.Az,
                        ImuData.Gx, ImuData.Gy, ImuData.Gz,
                        MagData.Mx, MagData.My, MagData.Mz,
                        ATT_CTRL_DT);

          /** 获取RC数据并校验时效 **/
          if(xQueuePeek(xRC_DataQ,&RcData,0) == pdTRUE && (xCurrentTime - RcData.TimeStamp) < pdMS_TO_TICKS(200))
          {
            /*** 外环：角度PID ***/
            PID_Angle_Yaw.Target   = RcData.Yaw_Target;
            PID_Angle_Yaw.Actual   = Att.Yaw;
            float Rate_Yaw_Target  = PID_Calculate(&PID_Angle_Yaw, ATT_CTRL_DT);

            PID_Angle_Roll.Target  = RcData.Roll_Target;
            PID_Angle_Roll.Actual  = Att.Roll;
            float Rate_Roll_Target = PID_Calculate(&PID_Angle_Roll, ATT_CTRL_DT);

            PID_Angle_Pitch.Target = RcData.Pitch_Target;
            PID_Angle_Pitch.Actual = Att.Pitch;
            float Rate_Pitch_Target = PID_Calculate(&PID_Angle_Pitch, ATT_CTRL_DT);

            /*** 内环：角速度PID ***/
            PID_Rate_Yaw.Target    = Rate_Yaw_Target;
            PID_Rate_Yaw.Actual    = ImuData.Gz;
            float Out_Yaw          = PID_Calculate(&PID_Rate_Yaw, ATT_CTRL_DT);

            PID_Rate_Roll.Target   = Rate_Roll_Target;
            PID_Rate_Roll.Actual   = ImuData.Gx;
            float Out_Roll         = PID_Calculate(&PID_Rate_Roll, ATT_CTRL_DT);

            PID_Rate_Pitch.Target  = Rate_Pitch_Target;
            PID_Rate_Pitch.Actual  = ImuData.Gy;
            float Out_Pitch        = PID_Calculate(&PID_Rate_Pitch, ATT_CTRL_DT);

            //应用高度修正，得到最终油门
            float Throttle = CLAMP(RcData.Throttle + HeightCorr,0.0f,1.0f);
            float BasePwm  = PWM_MIN + Throttle * PWM_RANGE;
            float BaseCorr = 0.5f * Throttle * PWM_RANGE;

            //X型四轴混控
            float M1_Corr = ( Out_Roll + Out_Pitch - Out_Yaw)*BaseCorr*PID_NORM;
            float M2_Corr = (-Out_Roll + Out_Pitch + Out_Yaw)*BaseCorr*PID_NORM;
            float M3_Corr = ( Out_Roll - Out_Pitch - Out_Yaw)*BaseCorr*PID_NORM;
            float M4_Corr = (-Out_Roll - Out_Pitch + Out_Yaw)*BaseCorr*PID_NORM;

            uint16_t Pwm1 = (uint16_t)(BasePwm + M1_Corr);
            uint16_t Pwm2 = (uint16_t)(BasePwm + M2_Corr);
            uint16_t Pwm3 = (uint16_t)(BasePwm + M3_Corr);
            uint16_t Pwm4 = (uint16_t)(BasePwm + M4_Corr);

            //钳位输出
            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,CLAMP(Pwm1,PWM_MIN,PWM_MAX));
            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,CLAMP(Pwm2,PWM_MIN,PWM_MAX));
            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,CLAMP(Pwm3,PWM_MIN,PWM_MAX));
            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,CLAMP(Pwm4,PWM_MIN,PWM_MAX));
          }
          else
          {
            //RC 数据无效
            MOTOR_STOP();

            PID_Rate_Yaw.ErrorInt   = 0.0f;
            PID_Rate_Roll.ErrorInt  = 0.0f;
            PID_Rate_Pitch.ErrorInt = 0.0f;

            PID_Angle_Roll.ErrorInt = 0.0f;
            PID_Angle_Pitch.ErrorInt= 0.0f;
          }
        }
        else
        {
          //IMU数据超时
          if((++ Imu_Timeout) > IMU_TIMEOUT_THRESHOLD)
          {
            FC_HandleEvent(EVENT_IMU_ERROR);
          }
        }
        break;
      }
      case STATE_DISARMED:
        MOTOR_STOP();
        break;
      case STATE_ARMED:
        MOTOR_STOP();
        break;
      case STATE_EMERGENCY:
        MOTOR_STOP();
        vTaskSuspend(NULL);
        break;
      default:
        MOTOR_STOP();
        break;
    }
    //喂狗
    HAL_IWDG_Refresh(&hiwdg1);
  }
  /* USER CODE END Att_Control */
}

/* USER CODE BEGIN Header_Rc_Parse */
/**
* @brief  系统遥控数据处理函数
* @param  argument: Not used
* @retval None
*/
/* USER CODE END Header_Rc_Parse */
void Rc_Parse(void *argument)
{
  /* USER CODE BEGIN Rc_Parse */
  (void)argument;

  rc_data_t RcData = {0};

  for(;;)
  {
    if(xSemaphoreTake(xRC_DataReady,pdMS_TO_TICKS(RC_PARSE_DT)) == pdTRUE)
    {
      Rc_Timeout = 0;
      Error_Code.RC_Timeout_Error = 0;

      RC_Parse(RC_RxBuffer,RC_RxLength,&RcRaw);

      RcData.Throttle     = RcRaw.Left_Y/100.0f;
      RcData.TimeStamp    = xTaskGetTickCount();

      RcData.Yaw_Target   = RcRaw.Left_X *PARAMS.Attitude_Control.Yaw_Scale  *RAD;
      RcData.Roll_Target  = RcRaw.Right_X*PARAMS.Attitude_Control.Roll_Scale *RAD;
      RcData.Pitch_Target = RcRaw.Right_Y*PARAMS.Attitude_Control.Pitch_Scale*RAD;

      //外八
      if(RcRaw.Left_X > LOCK_X_THRESHOLD && RcRaw.Right_X < -LOCK_X_THRESHOLD && RcRaw.Left_Y/100.0f < LOCK_THRO_THRESHOLD)
      {
        FC_HandleEvent(EVENT_ARM_GESTURE);
      }
      //内八
      if(RcRaw.Left_X < -LOCK_X_THRESHOLD && RcRaw.Right_X > LOCK_X_THRESHOLD && RcRaw.Left_Y/100.0f < LOCK_THRO_THRESHOLD)
      {
        FC_HandleEvent(EVENT_DISARM_GESTURE);
      }
      Lock_Update();

      xQueueOverwrite(xRC_DataQ,&RcData);
      RC_Init();
    }
    else
    {
      if((++ Rc_Timeout) > RC_TIMEOUT_THRESHOLD)
      {
        //上报超时错误
        Error_Code.RC_Timeout_Error = 1;
      }
    }
  }
  /* USER CODE END Rc_Parse */
}

/* USER CODE BEGIN Header_Log_Write */
/**
* @brief  系统日志处理函数
* @param  argument: Not used
* @retval None
*/
/* USER CODE END Header_Log_Write */
void Log_Write(void *argument)
{
  /* USER CODE BEGIN Log_Write */
  (void)argument;

  log_data_t LogData;

  TickType_t xCurrentTime;
  TickType_t xLastSyncTime = xTaskGetTickCount();

  for(;;)
  {
    //从日志队列取数据
    if(xQueueReceive(xLOG_DataQ,&LogData,pdMS_TO_TICKS(LOG_WRITE_DT)) == pdTRUE)
    {
      Log_Save(&LogData);
    }

    xCurrentTime = xTaskGetTickCount();
    //自动同步
    if(xCurrentTime - xLastSyncTime >= pdMS_TO_TICKS(5000))
    {
      Log_Sync();
      xLastSyncTime = xCurrentTime;
    }
  }
  /* USER CODE END Log_Write */
}

/* USER CODE BEGIN Header_Sys_Observe */
/**
* @brief  系统错误处理函数
* @param  argument: Not used
* @retval None
*/
/* USER CODE END Header_Sys_Observe */
void Sys_Observe(void *argument)
{
  /* USER CODE BEGIN Sys_Observe */
  (void)argument;
  
  for(;;)
  {
//GPS超时错误 恢复机制
    if(Error_Code.GPS_Error == 1)
    {
      static uint16_t cnt = 0;

      if((++ cnt)%1000 == 0)
      {
        Reset_USART(&huart3);    //尝试重启
        GPS_Init();
        
        HW_LockState.GPS_Unlock = 0;
        if(HW_LockState.GPS_Unlock == 1)
        {
          cnt = 0;
          Error_Code.GPS_Error = 0;
        }
      }
    }

//MAG超时错误 恢复机制
    if(Error_Code.MAG_Error == 1)
    {
      static uint16_t cnt = 0;
      if((++cnt)%50 == 0)
      {
        MAG_Init();
        if(HW_LockState.MAG_Unlock == 1)
        {
          cnt = 0;
          Error_Code.MAG_Error = 0;
        }
      }
    }

//BAR超时错误 恢复机制
    if (Error_Code.BAR_Error == 1)
    {
      static uint16_t cnt = 0;
      if ((++cnt) % 50 == 0)
      {
        BAR_Init();
        if (HW_LockState.BAR_Unlock == 1)
        {
          cnt = 0;
          Error_Code.BAR_Error = 0;
        }
      }
    }

    if(Error_Code.RC_Timeout_Error == 1)
    {
      static uint16_t cnt = 0;

      if((++cnt)%500 == 0)
      {
        Reset_USART(&huart2);
        RC_Init();
        
        if((cnt/500) > 100)
        {
          //警报
        }
      }
    }

    /*SD卡挂载错误,尝试重启*/
    if(Error_Code.LOG_Mount_Error == 1 && Giveup_Code.LOG_Mount_Giveup == 0)
    {
      static uint16_t cnt = 0;

      if((++ cnt)%500 == 0)
      {
        FRESULT res = f_mount(&SDFatFS,SDPath,1);
        if(res == FR_OK)
        {
          cnt = 0;
          Log_Status.Ready = 1;
          Error_Code.LOG_Mount_Error = 0;

          //尝试打开日志文件
          if(Log_Open() == 0)
          {
            Error_Code.LOG_Open_Error = 1;
          }
        }
        //多次失败
        else if((cnt/500) > 20)
        {
          Giveup_Code.LOG_Mount_Giveup = 1;
        }
      }
    }
    /*日志打开错误,尝试重启*/
    if(Error_Code.LOG_Open_Error == 1 && Giveup_Code.LOG_Open_Giveup == 0)
    {
      static uint16_t cnt = 0;

      if((++ cnt)%50 == 0)
      {
        if(Log_Status.Ready && Log_Open())
        {
          cnt = 0;
          Error_Code.LOG_Open_Error = 0;
          Giveup_Code.LOG_Open_Giveup = 0;
        }
        //多次失败
        else if(cnt/50 > 150)
        {
          Giveup_Code.LOG_Open_Giveup = 1;
        }
      }
    }
    /*日志写入错误,尝试重启*/
    if(Error_Code.LOG_Write_Error == 1 && Giveup_Code.LOG_Write_Giveup == 0)
    {
      static uint16_t cnt = 0;
      if((++ cnt)%50 == 0)
      {
        if(Log_Status.Ready && Log_Open() == 1)
        {
          cnt = 0;
          Error_Code.LOG_Write_Error = 0;
          Giveup_Code.LOG_Write_Giveup = 0;
        }
        //多次失败
        else if(cnt/50 > 100)
        {
          Giveup_Code.LOG_Write_Giveup = 1;
        }
      }
    }
    
    osDelay(10);
  }
  /* USER CODE END Sys_Observe */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

