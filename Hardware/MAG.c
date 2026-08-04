#include "cmsis_os2.h"
#include "Config.h"
#include "MAG.h"
#include "i2c.h"

mag_raw_t  MAG_RAW  = {0};
mag_data_t MAG_DATA = {0};

static int MAG_WriteReg(uint8_t reg, uint8_t val)
{
	if(HAL_I2C_Mem_Write(&hi2c1,MAG_ADDR<<1,reg,I2C_MEMADD_SIZE_8BIT,&val,1,100) != HAL_OK)
		return -1;
	return 0;
}

static int MAG_ReadReg(uint8_t reg,uint8_t *buf,uint16_t len)
{
	if(HAL_I2C_Mem_Read(&hi2c1,MAG_ADDR<<1,reg,I2C_MEMADD_SIZE_8BIT,buf,len,100) != HAL_OK)
		return -1;
	return 0;
}

int MAG_Init(void)
{
	uint8_t id = 0;

	if(MAG_WriteReg(MAG_CTRL_REG,MAG_CTRL_SRST) != 0)
		return -1;
	osDelay(50);

	if(MAG_ReadReg(MAG_ID_REG,&id,1) != 0)
		return -1;
	if(id != MAG_ID)
		return -1;

	if(MAG_WriteReg(MAG_CTRL_REG,MAG_CTRL_MODE) != 0)
		return -1;

	return 0;
}

int MAG_Read(void)
{
	uint8_t status;
	uint8_t data[6];

	if(MAG_ReadReg(MAG_STAT_REG,&status,1) != 0)
		return -1;
	if((status & 0x01) == 0)
		return -1;

	if(MAG_ReadReg(MAG_DATA_L,data,6) != 0)
		return -1;

	MAG_RAW.Mag[0] = (int16_t)((uint16_t)data[1] << 8 | data[0]);
	MAG_RAW.Mag[1] = (int16_t)((uint16_t)data[3] << 8 | data[2]);
	MAG_RAW.Mag[2] = (int16_t)((uint16_t)data[5] << 8 | data[4]);

	MAG_DATA.Mag[0] = (float)MAG_RAW.Mag[0] * MAG_SCALE;
	MAG_DATA.Mag[1] = (float)MAG_RAW.Mag[1] * MAG_SCALE;
	MAG_DATA.Mag[2] = (float)MAG_RAW.Mag[2] * MAG_SCALE;

	return 0;
}