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
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "inv_imu_driver.h"
#include "Receiver.h"
#include <stdint.h>
#include <string.h>
#include "semphr.h"
#include "Filter.h"
#include "Config.h"
#include "Error.h"
#include "fatfs.h"
#include "iwdg.h"
#include "Lock.h"
#include "IMU.h"
#include "PID.h"
#include "tim.h"
#include "Log.h"
#include "ff.h"
#include "GPS.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

SemaphoreHandle_t xGPS_DataReady;     //GPS数据就绪信号量
SemaphoreHandle_t xIMU_DataReady;     //IMU数据就绪信号量
SemaphoreHandle_t xRC_DataReady;      //RC 数据就绪信号量

QueueHandle_t     xLOG_DataQ;         //LOG数据队列
QueueHandle_t     xGPS_DataQ;         //GPS数据队列
QueueHandle_t     xIMU_DataQ;         //IMU数据队列
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

static uint16_t Imu_Timeout = 0;

/* USER CODE END Variables */
/* Definitions for Task_IMU_Rd */
osThreadId_t Task_IMU_RdHandle;
const osThreadAttr_t Task_IMU_Rd_attributes = {
  .name = "Task_IMU_Rd",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Task_Att_Ctrl */
osThreadId_t Task_Att_CtrlHandle;
const osThreadAttr_t Task_Att_Ctrl_attributes = {
  .name = "Task_Att_Ctrl",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Task_RC_Prs */
osThreadId_t Task_RC_PrsHandle;
const osThreadAttr_t Task_RC_Prs_attributes = {
  .name = "Task_RC_Prs",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Task_Pos_Est */
osThreadId_t Task_Pos_EstHandle;
const osThreadAttr_t Task_Pos_Est_attributes = {
  .name = "Task_Pos_Est",
  .stack_size = 1536 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_Tlm_Snd */
osThreadId_t Task_Tlm_SndHandle;
const osThreadAttr_t Task_Tlm_Snd_attributes = {
  .name = "Task_Tlm_Snd",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
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

void IMU_Read(void *argument);
void Att_Control(void *argument);
void RC_Parse(void *argument);
void Pos_Estimate(void *argument);
void Tlm_Send(void *argument);
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

  xRC_DataReady  = xSemaphoreCreateBinary();                   //状态信号量
  xIMU_DataReady = xSemaphoreCreateBinary();                   //状态信号量
  xGPS_DataReady = xSemaphoreCreateBinary();                   //GPS就绪信号量

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */

  xRC_DataQ  = xQueueCreate(1,sizeof(RC_DATA));
  xIMU_DataQ = xQueueCreate(1,sizeof(IMU_RAW));
  xLOG_DataQ = xQueueCreate(LOG_QUEUE_LEN,sizeof(log_data_t));
  xGPS_DataQ = xQueueCreate(GPS_QUEUE_LEN,sizeof(gps_data_t));

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task_IMU_Rd */
  Task_IMU_RdHandle = osThreadNew(IMU_Read, NULL, &Task_IMU_Rd_attributes);

  /* creation of Task_Att_Ctrl */
  Task_Att_CtrlHandle = osThreadNew(Att_Control, NULL, &Task_Att_Ctrl_attributes);

  /* creation of Task_RC_Prs */
  Task_RC_PrsHandle = osThreadNew(RC_Parse, NULL, &Task_RC_Prs_attributes);

  /* creation of Task_Pos_Est */
  Task_Pos_EstHandle = osThreadNew(Pos_Estimate, NULL, &Task_Pos_Est_attributes);

  /* creation of Task_Tlm_Snd */
  Task_Tlm_SndHandle = osThreadNew(Tlm_Send, NULL, &Task_Tlm_Snd_attributes);

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

/* USER CODE BEGIN Header_IMU_Read */
/**
  * @brief  Function implementing the Task_IMU_Rd thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_IMU_Read */
void IMU_Read(void *argument)
{
  /* USER CODE BEGIN IMU_Read */

  (void)argument;

  uint16_t Cnt = 0;

  /* Infinite loop */
  for(;;)
  {
    int32_t AccSum[3]  = {0};
    int32_t GyroSum[3] = {0};

    //等待FIFO水位中断信号
    xSemaphoreTake(xIMU_DataReady,portMAX_DELAY);

    //获取FIFO字节数
    inv_imu_get_frame_count(&IMU,&Cnt);

    //字节数换算为帧数(48/16=3帧)
    Cnt = Cnt/16;

    //帧数异常则清空FIFO
    if(Cnt < 3 || Cnt > 10)
    {
      inv_imu_flush_fifo(&IMU);
      continue;
    }

    //逐帧累加取均值
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

    //取平均后存入原始数据
    IMU_RAW.Acc[0] = AccSum[0]/Cnt;
    IMU_RAW.Acc[1] = AccSum[1]/Cnt;
    IMU_RAW.Acc[2] = AccSum[2]/Cnt;

    IMU_RAW.Gyro[0] = GyroSum[0]/Cnt;
    IMU_RAW.Gyro[1] = GyroSum[1]/Cnt;
    IMU_RAW.Gyro[2] = GyroSum[2]/Cnt;

    //覆盖写入队列
    xQueueOverwrite(xIMU_DataQ,&IMU_RAW);
  }
  /* USER CODE END IMU_Read */
}

/* USER CODE BEGIN Header_Att_Control */
/**
* @brief Function implementing the Task_Att_Ctrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Att_Control */
void Att_Control(void *argument)
{
  /* USER CODE BEGIN Att_Control */

  (void)argument;

  rc_data_t  RcData  = {0};     //RC数据
  imu_raw_t  ImuRaw  = {0};     //IMU原始数据
  imu_data_t ImuData = {0};     //IMU缩放数据

  TickType_t xCurrentTime;
  TickType_t xLastWakeTime = xTaskGetTickCount();     //上次任务唤醒时刻

  /* Infinite loop */
  for(;;)
  {
    //固定1KHz调度
    vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(1));
    xCurrentTime = xTaskGetTickCount();                   //当前时间

    /**获取IMU数据**/
    if(xQueueReceive(xIMU_DataQ,&ImuRaw,0) == pdTRUE)
    {
      //接收成功,清超时计数
      Imu_Timeout = 0;

      //原始数据缩放为物理量
      ImuData.Ax = ImuRaw.Acc[0]*ACC_SCALE;
      ImuData.Ay = ImuRaw.Acc[1]*ACC_SCALE;
      ImuData.Az = ImuRaw.Acc[2]*ACC_SCALE;

      ImuData.Gx = ImuRaw.Gyro[0]*GYRO_SCALE;
      ImuData.Gy = ImuRaw.Gyro[1]*GYRO_SCALE;
      ImuData.Gz = ImuRaw.Gyro[2]*GYRO_SCALE;

      //姿态解算
      Filter_Update(ImuData.Ax,ImuData.Ay,ImuData.Az,ImuData.Gx,ImuData.Gy,ImuData.Gz,ATT_CTRL_DT);

      /*获取RC数据并校验时效性(<200ms)*/
      if(xQueuePeek(xRC_DataQ,&RcData,0) == pdTRUE && (xCurrentTime - RcData.TimeStamp) < pdMS_TO_TICKS(200))
      {
        //组装手势输入
        Detect_Lock_t gesture =
        {
          .Left_X   = RcData.Left_X,
          .Right_X  = RcData.Right_X,
          .Throttle = RcData.Left_Y/100.0f
        };
        //解锁手势检测
        Lock_Detect(gesture);

        //更新锁定/蜂鸣状态
        Lock_Update();

        if(Sys_LockState.LockState == 1)    //电机失能
        {
          __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,PWM_MIN);
          __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,PWM_MIN);
          __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,PWM_MIN);
          __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,PWM_MIN);
        }
        else if(Sys_LockState.Locking == 1)   //蜂鸣进行中
        {
          //蜂鸣期间由Lock_Update内部处理
        }
        else    //电机使能且未蜂鸣
        {
          /***解锁状态: 执行PID+混控输出***/
          //RC摇杆映射为角度/角速度目标
          float Yaw_Target   = RcData.Left_X *YAW_SCALE  *RAD;
          float Roll_Target  = RcData.Right_X*ROLL_SCALE *RAD;
          float Pitch_Target = RcData.Right_Y*PITCH_SCALE*RAD;

          /***外环:角度PID,输出角速度目标***/
          PID_Angle_Roll.Target   = Roll_Target;
          PID_Angle_Roll.Actual   = Att.Roll;
          float Rate_Roll_Target  = PID_Calculate(&PID_Angle_Roll ,ATT_CTRL_DT);

          PID_Angle_Pitch.Target  = Pitch_Target;
          PID_Angle_Pitch.Actual  = Att.Pitch;
          float Rate_Pitch_Target = PID_Calculate(&PID_Angle_Pitch,ATT_CTRL_DT);

          /***内环:角速度PID,输出混控指令***/
          PID_Rate_Roll.Target    = Rate_Roll_Target;
          PID_Rate_Roll.Actual    = ImuData.Gx;          //陀螺仪X=滚转速度(rad/s)
          float Out_Roll          = PID_Calculate(&PID_Rate_Roll,ATT_CTRL_DT);

          PID_Rate_Pitch.Target   = Rate_Pitch_Target;
          PID_Rate_Pitch.Actual   = ImuData.Gy;          //陀螺仪Y=俯仰速度(rad/s)
          float Out_Pitch         = PID_Calculate(&PID_Rate_Pitch,ATT_CTRL_DT);

          PID_Rate_Yaw.Target     = Yaw_Target;
          PID_Rate_Yaw.Actual     = ImuData.Gz;          //陀螺仪Z=偏航速度(rad/s)
          float Out_Yaw           = PID_Calculate(&PID_Rate_Yaw,ATT_CTRL_DT);

          //油门归一化到0~1
          float Throttle = RcData.Left_Y / 100.0f;
          if(Throttle < 0.0f) Throttle = 0.0f;
          if(Throttle > 1.0f) Throttle = 1.0f;

          /**X型四轴混控**/
          //基准PWM与修正系数随油门变化
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

          //限幅后输出到TIM1
          __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,CLAMP(Pwm1,PWM_MIN,PWM_MAX));
          __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,CLAMP(Pwm2,PWM_MIN,PWM_MAX));
          __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,CLAMP(Pwm3,PWM_MIN,PWM_MAX));
          __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,CLAMP(Pwm4,PWM_MIN,PWM_MAX));
        }
      }
      else /*RC数据异常处理*/
      {
        //无有效RC数据则电机锁定最低脉宽
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,PWM_MIN);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,PWM_MIN);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,PWM_MIN);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,PWM_MIN);

        //清零内外环积分,防止积分饱和
        PID_Rate_Yaw.ErrorInt    = 0.0f;
        PID_Rate_Roll.ErrorInt   = 0.0f;
        PID_Rate_Pitch.ErrorInt  = 0.0f;

        PID_Angle_Roll.ErrorInt  = 0.0f;
        PID_Angle_Pitch.ErrorInt = 0.0f;
      }
    }
    else /*IMU数据异常处理*/
    {
      Imu_Timeout ++;

      //超时500ms则停机挂起
      if(Imu_Timeout > 500)
      {
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,PWM_MIN);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,PWM_MIN);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,PWM_MIN);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,PWM_MIN);

        //上报超时错误并挂起,由Sys_Observe恢复
        Error_Code.IMU_Timeout_Error = 1;

        vTaskSuspend(NULL);
      }
    }
    HAL_IWDG_Refresh(&hiwdg1);
  }
  /* USER CODE END Att_Control */
}

/* USER CODE BEGIN Header_RC_Parse */
/**
* @brief Function implementing the Task_RC_Prs thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RC_Parse */
void RC_Parse(void *argument)
{
  /* USER CODE BEGIN RC_Parse */

  (void)argument;
  Receiver_Init();

  uint8_t RcCopy[36] = {0};

  static uint8_t  First_Receive = 1;     //首次收到数据标志

  /* Infinite loop */
  for(;;)
  {
    if(xSemaphoreTake(xRC_DataReady,portMAX_DELAY) == pdTRUE)
    {
      //临界区拷贝DMA缓冲
      taskENTER_CRITICAL();
      memcpy(RcCopy,Rx_Buffer,MAX_FRAME_SIZE);
      taskEXIT_CRITICAL();

      //解析CRSF数据
      Process_CRSF_Data(RcCopy,MAX_FRAME_SIZE,&RC_DATA);

      //记录时间戳供时效校验
      RC_DATA.TimeStamp = xTaskGetTickCount();
      //覆盖写入队列
      xQueueOverwrite(xRC_DataQ,&RC_DATA);

      //重启DMA接收
      Receiver_Init();

      //首次收到有效数据,标记接收机就绪
      if(First_Receive != 0)
      {
        First_Receive = 0;
        HW_LockState.Receiver_Unlock = 1;
      }
    }
  }
  /* USER CODE END RC_Parse */
}

/* USER CODE BEGIN Header_Pos_Estimate */
/**
* @brief Function implementing the Task_Pos_Est thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Pos_Estimate */
void Pos_Estimate(void *argument)
{
  /* USER CODE BEGIN Pos_Estimate */

  (void)argument;

  gps_data_t GpsData = {0};     //GPS数据

  GPS_Init();

  /* Infinite loop */
  for(;;)
  {
    //等待GPS数据就绪信号量
    if(xSemaphoreTake(xGPS_DataReady,pdMS_TO_TICKS(500)) == pdTRUE)
    {
      GPS_Process();
      
      if(xQueueReceive(xGPS_DataQ,&GpsData,0) == pdTRUE)
      {
        //TODO: 根据GpsData做位置估计(GPS+IMU融合)
      }
    }
  }
  /* USER CODE END Pos_Estimate */
}

/* USER CODE BEGIN Header_Tlm_Send */
/**
* @brief Function implementing the Task_Tlm_Snd thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Tlm_Send */
void Tlm_Send(void *argument)
{
  /* USER CODE BEGIN Tlm_Send */

  (void)argument;

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Tlm_Send */
}

/* USER CODE BEGIN Header_Log_Write */
/**
* @brief Function implementing the Task_Log_Wrt thread.
* @param argument: Not used
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
    //从日志队列取数据写盘,超时则继续
    if(xQueueReceive(xLOG_DataQ,&LogData,pdMS_TO_TICKS(100)) == pdTRUE)
    {
      Log_Save(&LogData);
    }

    xCurrentTime = xTaskGetTickCount();

    //每5秒同步一次文件,防断电丢数据
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
* @brief Function implementing the Task_Sys_Obs thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Sys_Observe */
void Sys_Observe(void *argument)
{
  /* USER CODE BEGIN Sys_Observe */

  (void)argument;

  /* Infinite loop */
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
      IMU_Init();   //尝试重启

      if(HW_LockState.IMU_Unlock == 1) 
      {
        Error_Code.IMU_Timeout_Error = 0;
        vTaskResume(Task_Att_CtrlHandle);
        Imu_Timeout = 0;
      }
      else
      {
        //多次失败则停机
        static uint8_t cnt = 0;
        if((++ cnt) > 3) 
        {
          Error_Handler();
        }
      }
    }

    /*SD挂载错误:每500ms重试一次挂载*/
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
        else
        {
          //重试超出20次则放弃
          if((cnt/500) > 20)
          {
            Giveup_Code.LOG_Mount_Giveup = 1;
          }
        }
      }
    }
    /*日志打开错误:每50ms重试一次打开*/
    if(Error_Code.LOG_Open_Error == 1 && Giveup_Code.LOG_Open_Giveup == 0)
    {
      static uint16_t cnt = 0;

      if((++ cnt)%50 == 0)           //50ms间隔
      {
        if(Log_Status.Ready && Log_Open())
        {
          cnt = 0;
          Error_Code.LOG_Open_Error = 0;
          Giveup_Code.LOG_Open_Giveup = 0;
        }
        else if(cnt/50 > 150)   //尝试150次
        {
          Giveup_Code.LOG_Open_Giveup = 1;
        }
      }
    }
    /*日志写入错误:每50ms重试重开文件*/
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
        else if(cnt/50 > 100)
        {
          Giveup_Code.LOG_Write_Giveup = 1;
        }
      }
    }
    
    osDelay(1);
  }
  /* USER CODE END Sys_Observe */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */