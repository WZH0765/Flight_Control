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
#include "EvtBus.h"
#include "State.h"
#include "projdefs.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "Config.h"

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

//事件队列
QueueHandle_t     xEvent_Q;

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

//电机紧急停机标志（定义，Error.h中extern声明，由控制任务读取）
volatile uint8_t MotorStopCmd = 0;
//电调校准标志（校准期间禁止控制任务干预PWM）
volatile uint8_t EscCalibrating = 0;
//控制任务过载标志（供Sys_Observe监控复位）
volatile uint8_t ExecOverrun = 0;

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
/* Definitions for Task_EvtBus_Handler */
osThreadId_t Task_EvtBus_HandlerHandle;
const osThreadAttr_t Task_EvtBus_Handler_attributes = {
  .name = "Task_EvtBus_Handler",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
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
void EvtBus_Handler(void *argument);

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
  //xEvent_Q 已在 main.c 传感器初始化前创建（此处不再重复创建）

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

  /* creation of Task_EvtBus_Handler */
  Task_EvtBus_HandlerHandle = osThreadNew(EvtBus_Handler, NULL, &Task_EvtBus_Handler_attributes);

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
    inv_imu_get_frame_count(&IMU,&Cnt);

    //转换为帧
    Cnt = Cnt/16;
    if(Cnt == 0)
    {
      inv_imu_flush_fifo(&IMU);
      continue;
    }
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
      //Gps数据ready 发布至事件总线
      EvtBus_Publish(&(evt_publish_t){.ID = EVT_GPS_DATA_READY});
    }
    else
    {
      if((++ Gps_Timeout) > GPS_TIMEOUT_THRESHOLD)
      {
        EvtBus_Publish(&(evt_publish_t){.ID = EVT_GPS_ERROR});
      }
    }

//读取磁力计数据并入队
    if((++ Magcnt) >= MAG_READ_DIV)
    {
      int result = MAG_Parse();
      if(result == RET_OK)
      {
        Mag_Timeout = 0;
        //Mag数据ready 发布至事件总线
        EvtBus_Publish(&(evt_publish_t){.ID = EVT_MAG_DATA_READY});
      }
      else if(result == MAG_BUSY)
      {
        if((++ Mag_Timeout) >= MAG_TIMEOUT_THRESHOLD)
        {
          EvtBus_Publish(&(evt_publish_t){.ID = EVT_MAG_ERROR});
        }
      }
      else
      {
        EvtBus_Publish(&(evt_publish_t){.ID = EVT_MAG_ERROR});
      }
      Magcnt = 0;
    }

//读取气压计数据并入队
    if((++ Barcnt) >= BAR_READ_DIV)
    {
      if(BAR_Read() == LPS22HH_OK)
      {
        Bar_Timeout = 0;
        //Bar数据ready 发布至事件总线
        EvtBus_Publish(&(evt_publish_t){.ID = EVT_BAR_DATA_READY});
      }
      else
      {
        if((++ Bar_Timeout) >= BAR_TIMEOUT_THRESHOLD)
        {
          EvtBus_Publish(&(evt_publish_t){.ID = EVT_BAR_ERROR});
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

  rc_data_t  RcData  = {0};
  bar_data_t BarData = {0};
  state_t    CurrentState;

  float TargetAlt  = 0.0f;    //目标高度（相对）
  float CurrentAlt = 0.0f;    //当前高度（相对）

  TickType_t xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {
    vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(POS_ESTI_DT));
    if(xQueuePeek(xBAR_DataQ,&BarData,0) != pdTRUE) continue;

    CurrentAlt = BarData.Height - Result.HomeAlt;

    CurrentState = Get_CurrentState();
    if(CurrentState == STATE_FLYING)
    {
      if(xQueuePeek(xRC_DataQ,&RcData,0) == pdTRUE)
      {
        float Throttle = RcData.Throttle - 0.5f;
        //上升
        if((RcData.Throttle > PARAMS.Height_Control.Deadzone_High || RcData.Throttle < PARAMS.Height_Control.Deadzone_Low) && TargetAlt >= PARAMS.Height_Control.Min_Height && TargetAlt <= PARAMS.Height_Control.Max_Height)
        {
          TargetAlt += Throttle*PARAMS.Height_Control.Target_Rate;
        }
        else
        {
          TargetAlt = CurrentAlt;
        }
        TargetAlt = CLAMP(TargetAlt, PARAMS.Height_Control.Min_Height, PARAMS.Height_Control.Max_Height);
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

  TickType_t xCurrentTime;
  TickType_t xLoopStart;
  TickType_t xLastWakeTime = xTaskGetTickCount();     //上次任务唤醒时刻

  for(;;)
  {
    //固定1KHz调度
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    xCurrentTime = xTaskGetTickCount();
    xLoopStart   = xCurrentTime;

    if(Get_CurrentState() == STATE_FLYING)      //正常飞行中
    {
      if(MotorStopCmd == 1)                     //紧急停机指令有效，保持电机停止
      {
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,PWM_MIN);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,PWM_MIN);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,PWM_MIN);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,PWM_MIN);
      }
      else if(xQueueReceive(xIMU_DataQ,&ImuRaw,0) == pdTRUE)       //获取IMU数据
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
          MOTOR_Stop();

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
          EvtBus_Publish(&(evt_publish_t){.ID = EVT_IMU_ERROR});
          Imu_Timeout = 0;
        }
      }
    }
    else
    {
      if(EscCalibrating == 0)    //电调校准期间不干预PWM
      {
        MOTOR_Stop();
      }
    }
    //喂狗
    HAL_IWDG_Refresh(&hiwdg1);

    //1KHz周期监测：单次执行超过1ms则置位过载标志（供Sys_Observe监控）
    if((xTaskGetTickCount() - xLoopStart) > pdMS_TO_TICKS(1))
    {
      ExecOverrun = 1;
    }
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

      RC_Parse(RC_RxBuffer,RC_RxLength,&RcRaw);

      RcData.Throttle     = RcRaw.Left_Y/100.0f;
      RcData.TimeStamp    = xTaskGetTickCount();

      RcData.Yaw_Target   = RcRaw.Left_X *PARAMS.Attitude_Control.Yaw_Scale  *RAD;
      RcData.Roll_Target  = RcRaw.Right_X*PARAMS.Attitude_Control.Roll_Scale *RAD;
      RcData.Pitch_Target = RcRaw.Right_Y*PARAMS.Attitude_Control.Pitch_Scale*RAD;

      //外八
      if(RcRaw.Left_X > LOCK_X_THRESHOLD && RcRaw.Right_X < -LOCK_X_THRESHOLD && RcRaw.Left_Y/100.0f < LOCK_THRO_THRESHOLD)
      {
        EvtBus_Publish(&(evt_publish_t){.ID = EVT_ARM_GESTURE});
      }
      //内八
      if(RcRaw.Left_X < -LOCK_X_THRESHOLD && RcRaw.Right_X > LOCK_X_THRESHOLD && RcRaw.Left_Y/100.0f < LOCK_THRO_THRESHOLD)
      {
        EvtBus_Publish(&(evt_publish_t){.ID = EVT_DISARM_GESTURE});
      }

      xQueueOverwrite(xRC_DataQ,&RcData);
      RC_Init();
    }
    else
    {
      if((++ Rc_Timeout) > RC_TIMEOUT_THRESHOLD)
      {
        EvtBus_Publish(&(evt_publish_t){.ID = EVT_RC_LOST});
        Rc_Timeout = 0;
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

  log_data_t LogData = {0};
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {
    vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(LOG_WRITE_DT));

    if(xQueueReceive(xLOG_DataQ,&LogData,0) == pdTRUE)
    {
      Log_Save(&LogData);
    }
    else
    {
      //无新数据时同步缓冲区到SD卡
      Log_Sync();
    }
  }
  /* USER CODE END Log_Write */
}

/* USER CODE BEGIN Header_Sys_Observe */
/**
* @brief  系统健康监测函数
* @param  argument: Not used
* @retval None
*/
/* USER CODE END Header_Sys_Observe */
void Sys_Observe(void *argument)
{
  /* USER CODE BEGIN Sys_Observe */
  (void)argument;

  uint16_t InitTimeout = 0;    //传感器初始化超时计数
  uint16_t HealthTick  = 0;    //健康状态上报周期计数

  for(;;)
  {
    //传感器初始化超时监控：上电后若长时间未全部就绪，上报错误事件
    if(Get_CurrentState() == STATE_UNINIT)
    {
      if(HW_AllReady() == true)
      {
        InitTimeout = 0;
      }
      else if((++ InitTimeout) >= SENSORS_INIT_TIMEOUT)
      {
        EvtBus_Publish(&(evt_publish_t){.ID = EVT_SENSORS_ERROR});
        InitTimeout = 0;
      }
    }

    //系统健康度监测：控制任务过载告警，复位过载标志
    if(ExecOverrun == 1)
    {
      ExecOverrun = 0;
      //过载告警，可在此接入LED/蜂鸣器提示
    }

    if((++ HealthTick) >= SYS_OBS_DT)
    {
      HealthTick = 0;
      //周期性健康检查：可在此扩展电池电压/堆栈水位等监测项
    }

    osDelay(10);
  }
  /* USER CODE END Sys_Observe */
}

/* USER CODE BEGIN Header_EvtBus_Handler */
/**
* @brief  事件总线分发任务
* @param  argument: Not used
* @retval None
*/
/* USER CODE END Header_EvtBus_Handler */
void EvtBus_Handler(void *argument)
{
  /* USER CODE BEGIN EvtBus_Handler */
  (void)argument;

  evt_publish_t Event;

  for(;;)
  {
    //阻塞等待事件，取出后按事件ID分发至订阅回调
    if(xQueueReceive(xEvent_Q,&Event,portMAX_DELAY) == pdTRUE)
    {
      EvtBus_Dispatch(&Event);
    }
  }
  /* USER CODE END EvtBus_Handler */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

