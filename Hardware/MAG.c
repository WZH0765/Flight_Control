#include "FreeRTOSConfig.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "Config.h"
#include "Error.h"
#include "Lock.h"
#include "MAG.h"
#include "i2c.h"

/*
*	值得注意的是，H7 硬件I2C需要配置GPIO为 pull up和very high
*/

mag_raw_t  MagRaw  = {0};
mag_data_t MagData = {0};

static int MAG_WriteReg(uint8_t reg,uint8_t val)
{
	if(HAL_I2C_Mem_Write(&hi2c1,0X0F<<1,reg,I2C_MEMADD_SIZE_8BIT,&val,1,100) != HAL_OK)
    {
        return MAG_ERROR;
    }
	return MAG_OK;
}

static int MAG_ReadReg(uint8_t reg,uint8_t *buf,uint16_t len)
{
	if(HAL_I2C_Mem_Read(&hi2c1,0X0F<<1,reg,I2C_MEMADD_SIZE_8BIT,buf,len,100) != HAL_OK)
    {
        return MAG_ERROR;
    }
	return MAG_OK;
}

void MAG_Init(void)
{
	uint8_t ID = 0;
    int status = MAG_OK;

    //读ID
	status |= MAG_ReadReg(MAG_ID_REG,&ID,1);
	if(ID != MAG_ID)
    {
        Error_Code.MAG_ReadID_Error = 1;
        return ;
    }

    //连续测量模式
	status |= MAG_WriteReg(MAG_CTRL_REG,MAG_CTRL_MODE);

    if(status == MAG_OK)
	{
		HW_LockState.MAG_Unlock = 1;
	}
	else
	{
		Error_Code.MAG_Config_Error = 1;
		return ;
	}
}

/**
*	@retval: 0 成功读取，1 数据未就绪，-1 I2C异常
 */
int MAG_Parse(void)
{
	uint8_t status  = 0;
	uint8_t data[6] = {0};

	if(MAG_ReadReg(MAG_STAT_REG,&status,1) != MAG_OK)
	{
		return MAG_ERROR;
	}
	if((status&0x01) == 0)
	{
		return MAG_BUSY;
	}
	if(MAG_ReadReg(MAG_DATA_L,data,6) != MAG_OK)
	{
		return MAG_ERROR;
	}

	MagRaw.Mag[0] = (int16_t)((uint16_t)data[1]<<8|data[0]);
	MagRaw.Mag[1] = (int16_t)((uint16_t)data[3]<<8|data[2]);
	MagRaw.Mag[2] = (int16_t)((uint16_t)data[5]<<8|data[4]);

	MagData.Mx = (float)MagRaw.Mag[0]*MAG_SCALE;
    MagData.My = (float)MagRaw.Mag[1]*MAG_SCALE;
    MagData.Mz = (float)MagRaw.Mag[2]*MAG_SCALE;
	
	xQueueOverwrite(xMAG_DataQ,&MagData);
	return MAG_OK;
}