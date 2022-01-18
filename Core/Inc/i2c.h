#ifndef I2C_ER_H_
#define I2C_ER_H_

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

#define SCL_PIN     GPIO_PIN_6
#define SCL_PORT    GPIOB

#define SDA_PIN     GPIO_PIN_7
#define SDA_PORT    GPIOB

#define I2C					hi2c1

extern UART_HandleTypeDef 	huart1;
#define DEBUG_UART_I2C 		huart1
#define INIT_DEBUG_I2C 		1

void I2C_Scan_Bus(I2C_HandleTypeDef *hi2c);
void I2C_Error(char *er, uint32_t status);
void I2C_Init(I2C_HandleTypeDef *hi2c);

#endif /* I2C_ER_H_ */

