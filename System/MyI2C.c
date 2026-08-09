#include "MyI2C.h"

void I2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = MYI2C_SCL_PIN | MYI2C_SDA_PIN;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(MYI2C_PORT,&GPIO_InitStruct);
    HAL_GPIO_WritePin(MYI2C_PORT,MYI2C_SCL_PIN | MYI2C_SDA_PIN,GPIO_PIN_SET);
}

static void SDA_Input_Mode(void)
{
    GPIO_InitTypeDef GPIO = {0};
    GPIO.Pin   = MYI2C_SDA_PIN;
    GPIO.Pull  = GPIO_NOPULL;
    GPIO.Mode  = GPIO_MODE_INPUT;
    GPIO.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB,&GPIO);
}

static void SDA_Output_Mode(void)
{
    GPIO_InitTypeDef GPIO = {0};
    GPIO.Pin   = MYI2C_SDA_PIN;
    GPIO.Pull  = GPIO_NOPULL;
    GPIO.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB,&GPIO);
}

void I2C_Delay(void)
{
    for(volatile int i = 0;i < 100;i ++);
}

void I2C_Start(void)
{
    SDA_H(); SCL_H(); I2C_Delay();
    SDA_L(); I2C_Delay();
    SCL_L(); I2C_Delay();
}

void I2C_Stop(void)
{
    SCL_L(); I2C_Delay();
    SDA_L(); I2C_Delay();
    SCL_H(); I2C_Delay();
    SDA_H(); I2C_Delay();
}

uint8_t I2C_WriteByte(uint8_t byte)
{
    for(int i=0; i<8; i++)
    {
        if(byte & 0x80) SDA_H(); else SDA_L();
        byte <<= 1;
        SCL_H(); I2C_Delay();
        SCL_L(); I2C_Delay();
    }
    SDA_H();
    SDA_Input_Mode();
    SCL_H(); I2C_Delay();
    uint8_t ack = !SDA_R();
    SCL_L(); I2C_Delay();
    SDA_Output_Mode();
    return ack;
}

uint8_t I2C_ReadByte(uint8_t ack)
{
    uint8_t byte = 0;
    SDA_Input_Mode();
    SDA_H();
    for(int i=0; i<8; i++)
    {
        byte <<= 1;
        SCL_H(); I2C_Delay();
        if(SDA_R()) byte |= 1;
        SCL_L(); I2C_Delay();
    }
    SDA_Output_Mode();
    if(ack) SDA_L(); else SDA_H();
    SCL_H(); I2C_Delay();
    SCL_L(); I2C_Delay();
    SDA_H();
    return byte;
}
