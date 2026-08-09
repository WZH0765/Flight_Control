#include "cmsis_os2.h"
#include "Config.h"
#include "Error.h"
#include "MyI2C.h"
#include "Lock.h"
#include "MAG.h"
#include "i2c.h"

mag_raw_t MagRaw = {0};

static int MAG_WriteReg(uint8_t reg,uint8_t val)
{
	if(HAL_I2C_Mem_Write(&hi2c1,0X0F<<1,reg,I2C_MEMADD_SIZE_8BIT,&val,1,100) != HAL_OK)
    {
        return -1;
    }
	return 0;
}

static int MAG_ReadReg(uint8_t reg,uint8_t *buf,uint16_t len)
{
	if(HAL_I2C_Mem_Read(&hi2c1,0X0F<<1,reg,I2C_MEMADD_SIZE_8BIT,buf,len,100) != HAL_OK)
    {
        return -1;
    }
	return 0;
}

void MAG_Init(void)
{
	uint8_t ID = 0;
    int status = MAG_OK;

    //软复位
	status |= MAG_WriteReg(MAG_CTRL_REG,MAG_CTRL_SRST);
	osDelay(50);

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

void MAG_Read(void)
{
	uint8_t status  = 0;
	uint8_t data[6] = {0};

	if(MAG_ReadReg(MAG_STAT_REG,&status,1) != MAG_OK) return;
	if((status&0x01) == 0) return;

	if(MAG_ReadReg(MAG_DATA_L,data,6) != MAG_OK) return;

	MagRaw.Mag[0] = (int16_t)((uint16_t)data[1]<<8|data[0]);
	MagRaw.Mag[1] = (int16_t)((uint16_t)data[3]<<8|data[2]);
	MagRaw.Mag[2] = (int16_t)((uint16_t)data[5]<<8|data[4]);
}