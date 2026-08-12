#include "Config.h"
#include "EvtBus.h"
#include "HwState.h"

#define MAX_LEN 96

/*底层写入函数*/
static int IMU_WriteReg(uint8_t reg,const uint8_t *buf,uint32_t len)
{
	if(len > MAX_LEN) return -1;
	
	uint8_t tx_buf[MAX_LEN + 1];
	
	tx_buf[0] = reg & 0x7F;
	memcpy(&tx_buf[1],buf,len);
	
	HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1,tx_buf,len + 1,1000);
	
	return (status == HAL_OK) ? 0 : -1;
}

/*底层读取函数*/
static int IMU_ReadReg(uint8_t reg,uint8_t *buf,uint32_t len)
{
	if(len > MAX_LEN) return -1;
	
	uint8_t tx_buf[MAX_LEN + 1];
	uint8_t rx_buf[MAX_LEN + 1];
	
	tx_buf[0] = reg|0x80;
	memset(&tx_buf[1],0xFF,len);
	
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1,tx_buf,rx_buf,len + 1,1000);
	
	if(status == HAL_OK)
	{
		memcpy(buf,&rx_buf[1],len);
		return 0;
	}
	return -1;
}

/*底层休眠函数*/
static void IMU_Sleep(uint32_t us)
{
    if(us >= 500)
	{
        if(xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
		{
            //让出1ms,保证实时任务不被阻塞
            vTaskDelay(pdMS_TO_TICKS(1));
            return;
        }
    }
    volatile uint32_t count = us*(SystemCoreClock/1000000)/10;
    while(count--)
	{
       __asm volatile ("nop");
    }
}

inv_imu_device_t IMU = {0};
inv_imu_fifo_data_t FIFO_Data;		//IMU原始数据

extern SemaphoreHandle_t xIMU_DataReady;     //IMU数据就绪
extern QueueHandle_t     xIMU_DataQ;         //IMU数据队列

/*
	IMU初始化
*/
void IMU_Init(void)
{
	uint8_t ID = 0;
	int status = INV_IMU_OK;
	
	/*define drivers_BEGIN*/
	IMU.transport.write_reg	 = IMU_WriteReg;
	IMU.transport.read_reg	 = IMU_ReadReg;
	IMU.transport.sleep_us	 = IMU_Sleep;
	IMU.transport.serif_type = UI_SPI4;
	/*define drivers_END*/
	
	inv_imu_get_who_am_i(&IMU,&ID);
	if(ID != INV_IMU_WHOAMI)
	{
		HW_SetUnready(HW_IMU);
		EvtBus_Publish(&(evt_publish_t){.ID = EVT_IMU_ERROR});
		return ;
	}

	/*sensor mode Config_BEGIN*/
	status |= inv_imu_set_accel_mode(&IMU,PWR_MGMT0_ACCEL_MODE_LN);
	status |= inv_imu_set_gyro_mode(&IMU,PWR_MGMT0_GYRO_MODE_LN);
	/*sensor mode Config_END*/
	
	/*4g 500dps 3200Hz Config_BEGIN*/
	status |= inv_imu_set_accel_fsr(&IMU,ACCEL_CONFIG0_AP_ACCEL_FS_SEL_4_G);
	status |= inv_imu_set_accel_frequency(&IMU,ACCEL_CONFIG0_ACCEL_ODR_3200_HZ);
	status |= inv_imu_set_gyro_fsr(&IMU,GYRO_CONFIG0_AP_GYRO_FS_SEL_500_DPS);
	status |= inv_imu_set_gyro_frequency(&IMU,GYRO_CONFIG0_GYRO_ODR_3200_HZ);
	/*4g 500dps 3200Hz Config_END*/
	
	/*FIFO Config_BEGIN*/
	inv_imu_fifo_config_t FIFO_Config = 
	{
		.gyro_en	= INV_IMU_ENABLE,
		.accel_en	= INV_IMU_ENABLE,
		.hires_en	= INV_IMU_DISABLE,
		.fifo_wm_th = 48,								//48字节 = 3帧数据(16B/帧),3200Hz/3 ≈ 1066Hz中断
		.fifo_mode	= FIFO_CONFIG0_FIFO_MODE_STREAM,
		.fifo_depth = FIFO_CONFIG0_FIFO_DEPTH_MAX
	};
	
	status |= inv_imu_set_fifo_config(&IMU,&FIFO_Config);
	/*FIFO Config_END*/
	
	/*INT1 Config_BEGIN*/
	inv_imu_int_pin_config_t PIN1_Config =
	{
		.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH,
		.int_mode	  =	INTX_CONFIG2_INTX_MODE_PULSE,
		.int_drive	  = INTX_CONFIG2_INTX_DRIVE_PP
	};
	
	status |= inv_imu_set_pin_config_int(&IMU,INV_IMU_INT1,&PIN1_Config);
	
	inv_imu_int_state_t INT1_Config = {0};
	INT1_Config.INV_FIFO_THS = 1;
	
	status |= inv_imu_set_config_int(&IMU,INV_IMU_INT1,&INT1_Config);
	/*INT1 Config_END*/
	
	if(status == INV_IMU_OK)
	{
		HW_SetReady(HW_IMU);
		EvtBus_Publish(&(evt_publish_t){.ID = EVT_IMU_READY});
	}
	else
	{
		HW_SetUnready(HW_IMU);
		EvtBus_Publish(&(evt_publish_t){.ID = EVT_IMU_ERROR});
		return ;
	}
}