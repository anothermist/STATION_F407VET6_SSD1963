#ifndef I2C_ER_H_
#define I2C_ER_H_

#include "stm32f4xx_hal.h"

#define SCL_PIN     GPIO_PIN_6
#define SCL_PORT    GPIOB

#define SDA_PIN     GPIO_PIN_7
#define SDA_PORT    GPIOB

void I2C_ClearBusyFlagErratum(I2C_HandleTypeDef *hi2c, uint32_t timeout);

#endif /* I2C_ER_H_ */
