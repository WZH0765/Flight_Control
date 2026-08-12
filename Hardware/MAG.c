#include "Config.h"
#include "EvtBus.h"
#include "HwState.h"

/*
*	值得注意的是，H7 硬件I2C需要配置GPIO为 pull up和very high
*/

mag_raw_t  MagRaw  = {0};
mag_data_t MagData = {0};

static int MAG_WriteReg(uint8_t reg,uint8_t val)
{
	if(HAL_I2C_Mem_Write(&hi2c1,0X0F<<1,reg,I2C_MEMADD_SIZE_8BIT,&val,1,100) != HAL_OK)
    {
        return RET_ERROR;
    }
	return RET_OK;
}

static int MAG_ReadReg(uint8_t reg,uint8_t *buf,uint16_t len)
{
	if(HAL_I2C_Mem_Read(&hi2c1,0X0F<<1,reg,I2C_MEMADD_SIZE_8BIT,buf,len,100) != HAL_OK)
    {
        return RET_ERROR;
    }
	return RET_OK;
}

void MAG_Init(void)
{
	uint8_t ID = 0;
    int status = RET_OK;

    //读ID
	status |= MAG_ReadReg(MAG_ID_REG,&ID,1);
	if(ID != MAG_ID)
    {
		HW_SetUnready(HW_MAG);
		EvtBus_Publish(&(evt_publish_t){.ID = EVT_MAG_ERROR});
        return ;
    }

    //连续测量模式
	status |= MAG_WriteReg(MAG_CTRL_REG,MAG_CTRL_MODE);

    if(status == RET_OK)
	{
		HW_SetReady(HW_MAG);
		EvtBus_Publish(&(evt_publish_t){.ID = EVT_MAG_READY});
	}
	else
	{
		HW_SetUnready(HW_MAG);
		EvtBus_Publish(&(evt_publish_t){.ID = EVT_MAG_ERROR});
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

	if(MAG_ReadReg(MAG_STAT_REG,&status,1) != RET_OK)
	{
		return RET_ERROR;
	}
	if((status&0x01) == 0)
	{
		return MAG_BUSY;
	}
	if(MAG_ReadReg(MAG_DATA_L,data,6) != RET_OK)
	{
		return RET_ERROR;
	}

	MagRaw.Mag[0] = (int16_t)((uint16_t)data[1]<<8|data[0]);
	MagRaw.Mag[1] = (int16_t)((uint16_t)data[3]<<8|data[2]);
	MagRaw.Mag[2] = (int16_t)((uint16_t)data[5]<<8|data[4]);

	MagData.Mx = (float)MagRaw.Mag[0]*MAG_SCALE;
    MagData.My = (float)MagRaw.Mag[1]*MAG_SCALE;
    MagData.Mz = (float)MagRaw.Mag[2]*MAG_SCALE;
	
	xQueueOverwrite(xMAG_DataQ,&MagData);
	return RET_OK;
}