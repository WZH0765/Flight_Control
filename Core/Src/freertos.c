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
#include "stm32h7xx.h"
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
#include "Error.h"
#include "iwdg.h"
#include "IMU.h"
#include "PID.h"
#include "tim.h"
#include "Lock.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

SemaphoreHandle_t xIMU_DataReady;     //IMU数据就绪
SemaphoreHandle_t xRC_DataReady;      //RC数据就绪

QueueHandle_t     xIMU_DataQ;         //IMU数据队列
QueueHandle_t     xRC_DataQ;          //RC数据队列

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
  X型四轴电�?:M1前左,M2前右,M3后左,M4后右
*/

//电调�?�?/�?小脉�? us
#define ACC_SCALE   9.80f/8192.0f
#define GYRO_SCALE  0.0174533f/32.8f

#define RAD         0.0174533f
#define YAW_SCALE   180.0f/100.0f   /* �?大偏航角180° */
#define ROLL_SCALE  45.00f/100.0f   /* �?大�?�斜�?45°  */
#define PITCH_SCALE 45.00f/100.0f   /* �?大�?�斜�?45°  */
#define ATT_CTRL_DT 0.001f          /* 姿�?�控制周�?   */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define CLAMP(value,low,high) ((value)<(low)?(low):((value)>(high)?(high):(value)))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

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
  .stack_size = 512 * 4,
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
  .stack_size = 256 * 4,
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

  xRC_DataReady  = xSemaphoreCreateBinary();
  xIMU_DataReady = xSemaphoreCreateBinary();    //创建信号

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */

  xRC_DataQ  = xQueueCreate(1,sizeof(RC_DATA));
  xIMU_DataQ = xQueueCreate(1,sizeof(IMU_DATA));
  
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

    xSemaphoreTake(xIMU_DataReady,portMAX_DELAY);     //等待数据信号

    inv_imu_get_frame_count(&IMU,&Cnt);    //帧数

    if(Cnt < 3 || Cnt > 10)
    {
      inv_imu_flush_fifo(&IMU);
      continue;
    }

    for(int i = 0;i < Cnt;i++)
    {
      inv_imu_get_fifo_frame(&IMU,&FIFO_Data);

      AccSum[0] += FIFO_Data.byte_16.accel_data[0];
      AccSum[1] += FIFO_Data.byte_16.accel_data[1];
      AccSum[2] += FIFO_Data.byte_16.accel_data[2];
      
      GyroSum[0] += FIFO_Data.byte_16.gyro_data[0];
      GyroSum[1] += FIFO_Data.byte_16.gyro_data[1];
      GyroSum[2] += FIFO_Data.byte_16.gyro_data[2];
    }

    IMU_DATA.ACC_X = (float)AccSum[0]/Cnt;
    IMU_DATA.ACC_Y = (float)AccSum[1]/Cnt;
    IMU_DATA.ACC_Z = (float)AccSum[2]/Cnt;

    IMU_DATA.GYRO_X = (float)GyroSum[0]/Cnt;
    IMU_DATA.GYRO_Y = (float)GyroSum[1]/Cnt;
    IMU_DATA.GYRO_Z = (float)GyroSum[2]/Cnt;
    
    xQueueOverwrite(xIMU_DataQ,&IMU_DATA);
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

  static uint16_t Imu_Timeout = 0;

  rc_data_t  RcData  = {0};     //RC 数据
  imu_data_t ImuData = {0};     //IMU数据

  TickType_t xCurrentTime;
  TickType_t xLastWakeTime = xTaskGetTickCount();     //获取上一次任务唤醒时�?

  /* Infinite loop */
  for(;;)
  {
    vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(1));     //固定1KHz
    xCurrentTime = xTaskGetTickCount();                   //当前时间

    /**获取到IMU数据**/
    if(xQueueReceive(xIMU_DataQ,&ImuData,0) == pdTRUE)
    {
      Imu_Timeout = 0;

      //数据缩放
      float Ax = ImuData.ACC_X*ACC_SCALE;
      float Ay = ImuData.ACC_Y*ACC_SCALE;
      float Az = ImuData.ACC_Z*ACC_SCALE;

      float Gx = ImuData.GYRO_X*GYRO_SCALE;
      float Gy = ImuData.GYRO_Y*GYRO_SCALE;
      float Gz = ImuData.GYRO_Z*GYRO_SCALE;

      //姿�?�解�?
      Filter_Update(Ax,Ay,Az,Gx,Gy,Gz,ATT_CTRL_DT);

      /**获取到RC数据 RC数据有效性**/
      if(xQueuePeek(xRC_DataQ,&RcData,0) == pdTRUE && (xCurrentTime - RcData.TimeStamp) < pdMS_TO_TICKS(200))
      {
        Detect_Lock_t gesture =
        {
          .Left_X  = RcData.Left_X,
          .Left_Y  = RcData.Left_Y,
          .Right_X = RcData.Right_X,
          .Right_Y = RcData.Right_Y
        };
        Lock_Detect(gesture);

        /*系统更新*/
        Lock_Update();

        if(Sys_LockState.LockState == 1)    //电机失能状�??
        {
          __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,PWM_MIN);
          __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,PWM_MIN);
          __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,PWM_MIN);
          __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,PWM_MIN);
        }
        else if(Sys_LockState.Locking == 1)   //蜂鸣进行�?
        {

        }
        else    //电机使能且未蜂鸣
        {
          /***正常解锁状�??: 执行PID + 混控输出***/
          float Yaw_Target   = RcData.Left_X *YAW_SCALE  *RAD;
          float Roll_Target  = RcData.Right_X*ROLL_SCALE *RAD;
          float Pitch_Target = RcData.Right_Y*PITCH_SCALE*RAD;

          /***外环:角度PID 输出:角�?�度目标�?***/
          PID_Angle_Roll.Target   = Roll_Target;
          PID_Angle_Roll.Actual   = Att.Roll;
          float Rate_Roll_Target  = PID_Calculate(&PID_Angle_Roll ,ATT_CTRL_DT);

          PID_Angle_Pitch.Target  = Pitch_Target;
          PID_Angle_Pitch.Actual  = Att.Pitch;
          float Rate_Pitch_Target = PID_Calculate(&PID_Angle_Pitch,ATT_CTRL_DT);

          /***内环:角�?�度PID 输出:混控指令***/
          PID_Rate_Roll.Target    = Rate_Roll_Target;
          PID_Rate_Roll.Actual    = Gx;          //�?螺仪X=滚转速度(rad/s)
          float Out_Roll          = PID_Calculate(&PID_Rate_Roll,ATT_CTRL_DT);

          PID_Rate_Pitch.Target   = Rate_Pitch_Target;
          PID_Rate_Pitch.Actual   = Gy;          //�?螺仪Y=俯仰速度(rad/s)
          float Out_Pitch         = PID_Calculate(&PID_Rate_Pitch,ATT_CTRL_DT);

          PID_Rate_Yaw.Target     = Yaw_Target;
          PID_Rate_Yaw.Actual     = Gz;          //�?螺仪Z=偏航速度(rad/s)
          float Out_Yaw           = PID_Calculate(&PID_Rate_Yaw,ATT_CTRL_DT);

          //油门
          float Throttle = RcData.Left_Y / 100.0f;
          if(Throttle < 0.0f) Throttle = 0.0f;
          if(Throttle > 1.0f) Throttle = 1.0f;

          /**X型四轴混�?**/
          float BasePwm  = PWM_MIN + Throttle*PWM_RANGE;
          float BaseCorr = 0.5f*Throttle*PWM_RANGE;

          float M1_Corr = ( Out_Roll + Out_Pitch - Out_Yaw) * BaseCorr * PID_NORM;
          float M2_Corr = (-Out_Roll + Out_Pitch + Out_Yaw) * BaseCorr * PID_NORM;
          float M3_Corr = ( Out_Roll - Out_Pitch - Out_Yaw) * BaseCorr * PID_NORM;
          float M4_Corr = (-Out_Roll - Out_Pitch + Out_Yaw) * BaseCorr * PID_NORM;

          uint16_t Pwm1 = (uint16_t)(BasePwm + M1_Corr);
          uint16_t Pwm2 = (uint16_t)(BasePwm + M2_Corr);
          uint16_t Pwm3 = (uint16_t)(BasePwm + M3_Corr);
          uint16_t Pwm4 = (uint16_t)(BasePwm + M4_Corr);

          __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,CLAMP(Pwm1,PWM_MIN,PWM_MAX));
          __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,CLAMP(Pwm2,PWM_MIN,PWM_MAX));
          __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,CLAMP(Pwm3,PWM_MIN,PWM_MAX));
          __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,CLAMP(Pwm4,PWM_MIN,PWM_MAX));
        }
      }
      else /*RC数据异常处理*/
      {
        __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,PWM_MIN);
        __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,PWM_MIN);
        __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,PWM_MIN);
        __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,PWM_MIN);

        //清零内/外环积分，防止积分饱和
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

      if(Imu_Timeout > 500)
      {
        __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,PWM_MIN);
        __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,PWM_MIN);
        __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,PWM_MIN);
        __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,PWM_MIN);

        Error_Handler();
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

  static uint8_t  First_Receive = 1;     //第一次接收到数据

  /* Infinite loop */
  for(;;)
  {
    if(xSemaphoreTake(xRC_DataReady,portMAX_DELAY) == pdTRUE)
    {
      //临界区拷贝数�?
      taskENTER_CRITICAL();
      memcpy(RcCopy,Rx_Buffer,MAX_FRAME_SIZE);
      taskEXIT_CRITICAL();

      //解析CRSF数据
      Process_CRSF_Data(RcCopy,MAX_FRAME_SIZE,&RC_DATA);

      RC_DATA.TimeStamp = xTaskGetTickCount();
      //发数据到队列
      xQueueOverwrite(xRC_DataQ,&RC_DATA);

      //重启 DMA 接收
      Receiver_Init();

      //首次收到有效数据,标记接收机就�?
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

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
    /*IMU读取ID错误*/
    if(Error_Code.IMU_ReadID_Error == 1)
    {

    }
    /*IMU配置错误*/
    else if(Error_Code.IMU_Config_Error == 1)
    {

    }
    /*IMU超时错误*/
    else if(Error_Code.IMU_Timeout_Error == 1)
    {
      osDelay(100);
      
      IMU_Init();
      if(HW_LockState.IMU_Unlock == 1)
      {
        break;
      }
    }
    osDelay(1);
  }
  /* USER CODE END Sys_Observe */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

