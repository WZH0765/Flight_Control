#ifndef __MYI2C_H
#define __MYI2C_H

#include "main.h"
#include "stm32h7xx_hal.h"

#define MYI2C_PORT      GPIOB
#define MYI2C_SCL_PIN   GPIO_PIN_6
#define MYI2C_SDA_PIN   GPIO_PIN_7

#define MYI2C_SCL_H()   HAL_GPIO_WritePin(MYI2C_PORT, MYI2C_SCL_PIN, GPIO_PIN_SET)
#define MYI2C_SCL_L()   HAL_GPIO_WritePin(MYI2C_PORT, MYI2C_SCL_PIN, GPIO_PIN_RESET)
#define MYI2C_SDA_H()   HAL_GPIO_WritePin(MYI2C_PORT, MYI2C_SDA_PIN, GPIO_PIN_SET)
#define MYI2C_SDA_L()   HAL_GPIO_WritePin(MYI2C_PORT, MYI2C_SDA_PIN, GPIO_PIN_RESET)
#define MYI2C_SDA_READ() HAL_GPIO_ReadPin(MYI2C_PORT, MYI2C_SDA_PIN)

void MyI2C_Init(void);
uint8_t MyI2C_ReadByte(uint8_t ack);
uint8_t MyI2C_WriteByte(uint8_t byte);

#endif /* __MYI2C_H */