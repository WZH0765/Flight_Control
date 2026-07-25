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
#include "FreeRTOS.h"
#include "Receiver.h"
#include <stdint.h>
#include <string.h>
#include "semphr.h"
#include "Filter.h"
#include "iwdg.h"
#include "IMU.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ACC_SCALE  9.80f/8192.0f
#define GYRO_SCALE 0.0174f/32.8f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

SemaphoreHandle_t xIMU_DataReady;     //IMU数据就绪
SemaphoreHandle_t xRC_DataReady;      //RC数据就绪

QueueHandle_t     xIMU_DataQ;         //IMU数据队列
QueueHandle_t     xRC_DataQ;          //RC数据队列

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
  xIMU_DataReady = xSemaphoreCreateBinary();    //创建信号量

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

  uint16_t FIFO_CNT = 0;
  int32_t Acc_Sum[3]  = {0};
  int32_t Gyro_Sum[3] = {0};
  /* Infinite loop */
  for(;;)
  {
    xSemaphoreTake(xIMU_DataReady,portMAX_DELAY);     //等待数据信号

    inv_imu_get_frame_count(&IMU,&FIFO_CNT);    //帧数

    if(FIFO_CNT < 3 || FIFO_CNT > 10)
    {
      inv_imu_flush_fifo(&IMU);
      continue;
    }

    for(int i = 0;i < FIFO_CNT;i++)
    {
      inv_imu_get_fifo_frame(&IMU,&FIFO_Data);

      Acc_Sum[0] += FIFO_Data.byte_16.accel_data[0];
      Acc_Sum[1] += FIFO_Data.byte_16.accel_data[1];
      Acc_Sum[2] += FIFO_Data.byte_16.accel_data[2];
      
      Gyro_Sum[0] += FIFO_Data.byte_16.gyro_data[0];
      Gyro_Sum[1] += FIFO_Data.byte_16.gyro_data[1];
      Gyro_Sum[2] += FIFO_Data.byte_16.gyro_data[2];
    }

    IMU_DATA.ACC_X = (float)Acc_Sum[0]/FIFO_CNT;
    IMU_DATA.ACC_Y = (float)Acc_Sum[1]/FIFO_CNT;
    IMU_DATA.ACC_Z = (float)Acc_Sum[2]/FIFO_CNT;

    IMU_DATA.GYRO_X = (float)Gyro_Sum[0]/FIFO_CNT;
    IMU_DATA.GYRO_Y = (float)Gyro_Sum[1]/FIFO_CNT;
    IMU_DATA.GYRO_Z = (float)Gyro_Sum[2]/FIFO_CNT;
    
    memset(&Acc_Sum, 0,sizeof(Acc_Sum));      //清空SUM
    memset(&Gyro_Sum,0,sizeof(Gyro_Sum));     //清空SUM

    xQueueOverwrite(xIMU_DataQ,&IMU_DATA);

    inv_imu_flush_fifo(&IMU);
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

  imu_data_t IMU = {0};
  TickType_t xLastWakeTime = xTaskGetTickCount();     //获取上一次任务唤醒时刻

  Filter_Init(0.5f,0.01f);

  /* Infinite loop */
  for(;;)
  {
    vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(1));     //固定1KHz

    if(xQueueReceive(xIMU_DataQ,&IMU,0) == pdTRUE)
    {
      //如果有数据

			/*数据缩放 BEGIN*/
			float Ax = IMU.ACC_X*ACC_SCALE;
      float Ay = IMU.ACC_Y*ACC_SCALE;
      float Az = IMU.ACC_Z*ACC_SCALE;

      float Gx = IMU.GYRO_X*GYRO_SCALE;
      float Gy = IMU.GYRO_Y*GYRO_SCALE;
      float Gz = IMU.GYRO_Z*GYRO_SCALE;
			/**数据缩放 END**/

			/*姿态解算 BEGIN*/
      Filter_Update(Ax,Ay,Az,Gx,Gy,Gz,0.001f);
			/**姿态解算 END**/

      /*PID控制 BEGIN*/

			/**PID控制 END**/
    }
    HAL_IWDG_Refresh(&hiwdg1);    //无条件喂狗
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

  uint8_t RC_Copy[36] = {0};
  /* Infinite loop */
  for(;;)
  {
    if(xSemaphoreTake(xRC_DataReady,portMAX_DELAY) == pdTRUE)
    {
      //临界区拷贝数据
      taskENTER_CRITICAL();
      memcpy(RC_Copy,Rx_Buffer,MAX_FRAME_SIZE);
      taskEXIT_CRITICAL();

      //解析CRSF数据
      Process_CRSF_Data(RC_Copy,MAX_FRAME_SIZE,&RC_DATA);

      //发送到队列
      xQueueOverwrite(xRC_DataQ,&RC_DATA);

      //重启 DMA 接收
      Receiver_Init();
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
    osDelay(1);
  }
  /* USER CODE END Sys_Observe */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

