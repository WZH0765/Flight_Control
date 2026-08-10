/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "projdefs.h"
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
#include "usart.h"
#include "Error.h"
#include "fatfs.h"
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

SemaphoreHandle_t xGPS_DataReady;     //GPS数据就绪信号
SemaphoreHandle_t xIMU_DataReady;     //IMU数据就绪信号
SemaphoreHandle_t xRC_DataReady;      //RC 数据就绪信号

QueueHandle_t     xLOG_DataQ;         //LOG数据队列
QueueHandle_t     xGPS_DataQ;         //GPS数据队列
QueueHandle_t     xIMU_DataQ;         //IMU数据队列
QueueHandle_t     xMAG_DataQ;         //MAG数据队列
QueueHandle_t     xBAR_DataQ;         //BAR数据队列
QueueHandle_t     xRC_DataQ;          //RC 数据队列

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

rc_raw_t RcRaw = {0};

float HeightCorr = 0;

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

  xRC_DataReady  = xSemaphoreCreateBinary();
  xIMU_DataReady = xSemaphoreCreateBinary();
  xGPS_DataReady = xSemaphoreCreateBinary();

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */

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
  * @brief  系统IMU数据处理函数
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

    //等待信号
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
* @brief  系统sensor数据处理函数
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
    vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(10));    //10ms周期
/***********GPS Data Read BEGIN***********/
    if(xSemaphoreTake(xGPS_DataReady,0) == pdTRUE)
    {
      Gps_Timeout = 0;
      GPS_Parse(GPS_RxBuffer,GPS_RxLength);
      GPS_Init();
    }
    else
    {
      Gps_Timeout ++;
      if(Gps_Timeout > 500)                 //5秒无数据
      {
        Error_Code.GPS_Timeout_Error = 1;
      }
    }
/************GPS Data Read END************/

/***********MAG Data Read BEGIN***********/
    if((++ Magcnt) >= 5)
    {
      int result = MAG_Parse();
      if(result == MAG_OK)
      {
        Mag_Timeout = 0;
        Error_Code.MAG_Timeout_Error = 0;
      }
      else if(result == MAG_BUSY)
      {
        if((++ Mag_Timeout) >= 20)
        {
          Error_Code.MAG_Timeout_Error = 1;
        }
      }
      else
      {
        Error_Code.MAG_Timeout_Error = 1;
      }
      Magcnt = 0;
    }
/************MAG Data Read END************/

/***********BAR Data Read BEGIN***********/
    if((++ Barcnt) >= 2)
    {
      if(BAR_Read() == LPS22HH_OK)
      {
        Bar_Timeout = 0;
        Error_Code.BAR_Timeout_Error = 0;
      }
      else
      {
        if((++ Bar_Timeout) >= 50)
        {
          Error_Code.BAR_Timeout_Error = 1;
        }
      }
      Barcnt = 0;
    }
/************BAR Data Read END************/
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
  gps_data_t GpsData;
  bar_data_t BarData;

  float Base_Height   = 0.0f;    //基准高度（绝对）
  float Target_Height = 0.0f;    //目标高度（相对）
  float Actual_Height = 0.0f;    //当前高度（相对）

  uint8_t Set_Ground  = 0;       //是否完成校准

  TickType_t xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {
    vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(20));  // 50Hz

    if(xQueuePeek(xBAR_DataQ,&BarData,0) != pdTRUE) continue;
    if(Set_Ground == 0 && Sys_LockState.LockState == 1 && BarData.Height > 1.0f)
    {
      Target_Height = 0.0f;
      Base_Height = BarData.Height;
      Set_Ground = 1;
    }
    if(Set_Ground == 0) continue;

    Actual_Height = BarData.Height - Base_Height;
    if(xQueuePeek(xRC_DataQ,&RcData,0) == pdTRUE)
    {
      //上升
      if(RcData.Throttle > 0.55f && Target_Height < 50.0f)
      {
        Target_Height += (RcData.Throttle - 0.5f)*0.3f;
      }
      //下降
      else if(RcData.Throttle < 0.45f && Target_Height > 0.0f)
      {
        Target_Height += (RcData.Throttle - 0.5f)*0.3f;
      }
      //保持
      else
      {
        Target_Height = Actual_Height;
      }
      Target_Height = CLAMP(Target_Height,0,50);
    }

    PID_Altitude.Target = Target_Height;
    PID_Altitude.Actual = Actual_Height;
    float Target_Speed = PID_Calculate(&PID_Altitude,0.02f);

    PID_Velocity.Target = Target_Speed;
    PID_Velocity.Actual = 0.0f;
    HeightCorr = PID_Calculate(&PID_Velocity,0.02f);

    HeightCorr = CLAMP(HeightCorr,-0.25,0.25);
  }
  /* USER CODE END Pos_Estimate */
}

/* USER CODE BEGIN Header_Att_Control */
/**
* @brief  系统总体数据处理函数
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

  TickType_t xCurrentTime;
  TickType_t xLastWakeTime = xTaskGetTickCount();     //上次任务唤醒时刻

  for(;;)
  {
    //固定1KHz调度
    vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(1));
    xCurrentTime = xTaskGetTickCount();                   //当前时间

    /**获取IMU数据**/
    if(xQueueReceive(xIMU_DataQ,&ImuRaw,0) == pdTRUE)
    {
      //接收成功
      Imu_Timeout = 0;

      //原始数据缩放为物理量
      ImuData.Ax = ImuRaw.Acc[0]*ACC_SCALE;
      ImuData.Ay = ImuRaw.Acc[1]*ACC_SCALE;
      ImuData.Az = ImuRaw.Acc[2]*ACC_SCALE;

      ImuData.Gx = ImuRaw.Gyro[0]*GYRO_SCALE;
      ImuData.Gy = ImuRaw.Gyro[1]*GYRO_SCALE;
      ImuData.Gz = ImuRaw.Gyro[2]*GYRO_SCALE;

      if(xQueuePeek(xMAG_DataQ,&MagData,0) != pdTRUE)
      {
        MagData.Mx = 0.0f;
        MagData.My = 0.0f;
        MagData.Mz = 0.0f;
      }
      Filter_Update(ImuData.Ax,ImuData.Ay,ImuData.Az,
                    ImuData.Gx,ImuData.Gy,ImuData.Gz,
                    MagData.Mx,MagData.My,MagData.Mz,
                    ATT_CTRL_DT);

      /**获取RC数据**/
      if(xQueuePeek(xRC_DataQ,&RcData,0) == pdTRUE && (xCurrentTime - RcData.TimeStamp) < pdMS_TO_TICKS(200))
      {
        if(Sys_LockState.LockState == 1)    //电机失能
        {
          __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,PWM_MIN);
          __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,PWM_MIN);
          __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,PWM_MIN);
          __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,PWM_MIN);
        }
        else if(Sys_LockState.Locking == 1)   //蜂鸣进行
        {

        }
        else    //电机使能且未蜂鸣
        {
          /***外环:角度PID***/
          PID_Angle_Yaw.Target    = RcData.Yaw_Target;
          PID_Angle_Yaw.Actual    = Att.Yaw;
          float Rate_Yaw_Target   = PID_Calculate(&PID_Angle_Yaw  ,ATT_CTRL_DT);

          PID_Angle_Roll.Target   = RcData.Roll_Target;
          PID_Angle_Roll.Actual   = Att.Roll;
          float Rate_Roll_Target  = PID_Calculate(&PID_Angle_Roll ,ATT_CTRL_DT);

          PID_Angle_Pitch.Target  = RcData.Pitch_Target;
          PID_Angle_Pitch.Actual  = Att.Pitch;
          float Rate_Pitch_Target = PID_Calculate(&PID_Angle_Pitch,ATT_CTRL_DT);

          /***内环:角速度PID***/
          PID_Rate_Yaw.Target     = Rate_Yaw_Target;
          PID_Rate_Yaw.Actual     = ImuData.Gz;          //Z=偏航速度(rad/s)
          float Out_Yaw           = PID_Calculate(&PID_Rate_Yaw,ATT_CTRL_DT);

          PID_Rate_Roll.Target    = Rate_Roll_Target;
          PID_Rate_Roll.Actual    = ImuData.Gx;          //X=滚转速度(rad/s)
          float Out_Roll          = PID_Calculate(&PID_Rate_Roll,ATT_CTRL_DT);

          PID_Rate_Pitch.Target   = Rate_Pitch_Target;
          PID_Rate_Pitch.Actual   = ImuData.Gy;          //Y=俯仰速度(rad/s)
          float Out_Pitch         = PID_Calculate(&PID_Rate_Pitch,ATT_CTRL_DT);

          //基准PWM与修正量
          float Throttle = CLAMP(RcData.Throttle + HeightCorr,0.0f,1.0f);
          float BasePwm  = PWM_MIN + Throttle*PWM_RANGE;
          float BaseCorr = 0.5f*Throttle*PWM_RANGE;

          //四电机修正量
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
      }
      else /*RC数据异常处理*/
      {
        //无有效RC数据则电机锁
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,PWM_MIN);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,PWM_MIN);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,PWM_MIN);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,PWM_MIN);

        //防止积分饱和
        PID_Rate_Yaw.ErrorInt    = 0.0f;
        PID_Rate_Roll.ErrorInt   = 0.0f;
        PID_Rate_Pitch.ErrorInt  = 0.0f;

        PID_Angle_Roll.ErrorInt  = 0.0f;
        PID_Angle_Pitch.ErrorInt = 0.0f;
      }
    }
    else /*IMU数据异常处理*/
    {
      if((++ Imu_Timeout) > 500)
      {
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,PWM_MIN);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,PWM_MIN);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,PWM_MIN);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,PWM_MIN);

        //上报超时错误
        Error_Code.IMU_Timeout_Error = 1;
        vTaskSuspend(NULL);
      }
    }
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

  rc_data_t  RcData  = {0};
  ges_lock_t Gesture = {0};

  for(;;)
  {
    if(xSemaphoreTake(xRC_DataReady,pdMS_TO_TICKS(500)) == pdTRUE)
    {
      Rc_Timeout = 0;
      Error_Code.RC_Timeout_Error = 0;

      RC_Parse(RC_RxBuffer,RC_RxLength,&RcRaw);

      RcData.Throttle     = RcRaw.Left_Y/100.0f;
      RcData.TimeStamp    = xTaskGetTickCount();
      RcData.Yaw_Target   = RcRaw.Left_X *YAW_SCALE  *RAD;
      RcData.Roll_Target  = RcRaw.Right_X*ROLL_SCALE *RAD;
      RcData.Pitch_Target = RcRaw.Right_Y*PITCH_SCALE*RAD;

      Gesture.Left_X      = RcRaw.Left_X;
      Gesture.Right_X     = RcRaw.Right_X;
      Gesture.Throttle    = RcRaw.Left_Y/100.0f;

      Lock_Detect(Gesture);
      Lock_Update();

      //覆盖写入队列
      xQueueOverwrite(xRC_DataQ,&RcData);
      RC_Init();
    }
    else
    {
      if((++ Rc_Timeout) > 5)
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

  /* Infinite loop */
  for(;;)
  {
    //从日志队列取数据
    if(xQueueReceive(xLOG_DataQ,&LogData,pdMS_TO_TICKS(100)) == pdTRUE)
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
    /*IMU读取ID错误,直接停机*/
    if(Error_Code.IMU_ReadID_Error == 1)
    {
      Error_Handler();
    }
    /*IMU配置错误,直接停机*/
    if(Error_Code.IMU_Config_Error == 1)
    {
      Error_Handler();
    }
    /*IMU超时错误,尝试重启*/
    if(Error_Code.IMU_Timeout_Error == 1)
    {
      static uint16_t cnt = 0;

      if((++ cnt)%500 == 0)
      {
        IMU_Init();   //尝试重启

        if(HW_LockState.IMU_Unlock == 1) 
        {
          cnt = 0;
          Imu_Timeout = 0;
          Error_Code.IMU_Timeout_Error = 0;
          vTaskResume(Task_Att_CtrlHandle);
        }
        //多次失败
        else if((cnt/500) > 3)
        { 
          Error_Handler();
        }
      }
    }

    /*GPS超时错误,尝试重启*/
    if(Error_Code.GPS_Timeout_Error == 1 && Giveup_Code.GPS_Giveup == 0)
    {
      static uint16_t cnt = 0;

      if((++ cnt)%1000 == 0)
      {
        Reset_USART(&huart3);    //尝试重启
        GPS_Init();
        
        if(HW_LockState.GPS_Unlock == 1)
        {
          cnt = 0;
          Giveup_Code.GPS_Giveup = 0;
          Error_Code.GPS_Timeout_Error = 0;
        }
        //多次失败
        else if((cnt/1000) > 3)
        {
          Giveup_Code.GPS_Giveup = 1;
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

