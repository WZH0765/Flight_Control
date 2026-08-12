#include "Config.h"
#include "EvtBus.h"
#include "HwState.h"

/*底层写入函数*/
static int32_t BAR_WriteReg(uint16_t DevAddr,uint16_t Reg,uint8_t *pData,uint16_t Len)
{
    if(HAL_I2C_Mem_Write(&hi2c2,DevAddr,Reg,I2C_MEMADD_SIZE_8BIT,pData,Len,100) != HAL_OK)
    {
        return LPS22HH_ERROR;
    }
    return LPS22HH_OK;
}

/*底层读取函数*/
static int32_t BAR_ReadReg(uint16_t DevAddr,uint16_t Reg,uint8_t *pData,uint16_t Len)
{
    if(HAL_I2C_Mem_Read(&hi2c2,DevAddr,Reg,I2C_MEMADD_SIZE_8BIT,pData,Len,100) != HAL_OK)
    {
        return LPS22HH_ERROR;
    }
    return LPS22HH_OK;
}

/*底层休眠函数*/
static void BAR_Delay(uint32_t ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        vTaskDelay(pdMS_TO_TICKS(ms));
        return;
    }
    volatile uint32_t count = ms*(SystemCoreClock/1000)/10;
    while (count--)
    {
        __asm volatile ("nop");
    }
}

LPS22HH_Object_t BAR = {0};

void BAR_Init(void)
{
    uint8_t ID = 0;
    int status = LPS22HH_OK;

    /*define drivers_BEGIN*/
    LPS22HH_IO_t transport;
    transport.BusType  = LPS22HH_I2C_BUS;
    transport.WriteReg = BAR_WriteReg;
    transport.ReadReg  = BAR_ReadReg;
    transport.Delay    = BAR_Delay;
    /*define drivers_END*/

    status |= LPS22HH_RegisterBusIO(&BAR,&transport);
    status |= LPS22HH_Init(&BAR);

    LPS22HH_ReadID(&BAR,&ID);
    if(ID != LPS22HH_ID)
    {
        HW_SetUnready(HW_BAR);
        EvtBus_Publish(&(evt_publish_t){.ID = EVT_BAR_ERROR});
    }

    status |= lps22hh_lp_bandwidth_set(&BAR.Ctx,LPS22HH_LPF_ODR_DIV_9);
    status |= LPS22HH_PRESS_Enable(&BAR);
    status |= LPS22HH_TEMP_Enable(&BAR);

    if(status == LPS22HH_OK)
    {
        HW_SetReady(HW_BAR);
        EvtBus_Publish(&(evt_publish_t){.ID = EVT_BAR_READY});
    }
    else
    {
        HW_SetUnready(HW_BAR);
        EvtBus_Publish(&(evt_publish_t){.ID = EVT_BAR_ERROR});
        return;
    }
}

bar_raw_t  BarRaw  = {0};
bar_data_t BarData = {0};

int BAR_Read(void)
{
    float pressure,temperature;
    int status = LPS22HH_OK;

    status |= LPS22HH_PRESS_GetPressure(&BAR,&pressure);
    status |= LPS22HH_TEMP_GetTemperature(&BAR,&temperature);

    if(status == LPS22HH_OK)
    {
        BarRaw.Press = pressure;
        BarRaw.Temp  = temperature;

        BarData.TimeStamp = xTaskGetTickCount();
        BarData.Height    = (1.0f - powf(pressure/1013.25f,0.190263f))*44330.8f;

        xQueueOverwrite(xBAR_DataQ,&BarData);
        return LPS22HH_OK;
    }
    return LPS22HH_ERROR;
}