#ifndef INC_W25QXX_H_
#define INC_W25QXX_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_rcc.h"

#include <stdio.h>

#define	W25QXX_SPI 			hspi1
#define	W25QXX_NSS_SOFT		1
#define	W25QXX_NSS_PORT 	GPIOB
#define	W25QXX_NSS_PIN 		GPIO_PIN_0

#define	W25QXX_ENABLE_RESET	0x66
#define	W25QXX_RESET		0x99
#define	W25QXX_READ			0x03
#define	W25QXX_GET_JEDEC_ID	0x9f

void W25QXX_Reset(void);
void W25QXX_Read_Data(uint32_t addr, uint8_t* data, uint32_t sz);
uint32_t W25QXX_Read_ID(void);
void W25QXX_Init(void);

#endif /* INC_W25QXX_H_ */
