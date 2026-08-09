#ifndef __MYI2C_H
#define __MYI2C_H

#include "main.h"
#include "stm32h7xx_hal.h"

#define MYI2C_PORT      GPIOB
#define MYI2C_SCL_PIN   GPIO_PIN_6
#define MYI2C_SDA_PIN   GPIO_PIN_7

#define SCL_H()   HAL_GPIO_WritePin(MYI2C_PORT,MYI2C_SCL_PIN,GPIO_PIN_SET)
#define SCL_L()   HAL_GPIO_WritePin(MYI2C_PORT,MYI2C_SCL_PIN,GPIO_PIN_RESET)
#define SDA_H()   HAL_GPIO_WritePin(MYI2C_PORT,MYI2C_SDA_PIN,GPIO_PIN_SET)
#define SDA_L()   HAL_GPIO_WritePin(MYI2C_PORT,MYI2C_SDA_PIN,GPIO_PIN_RESET)
#define SDA_R()   HAL_GPIO_ReadPin(MYI2C_PORT,MYI2C_SDA_PIN)

void I2C_Init(void);
void I2C_Stop(void);
void I2C_Start(void);
uint8_t I2C_ReadByte(uint8_t ack);
uint8_t I2C_WriteByte(uint8_t byte);

#endif