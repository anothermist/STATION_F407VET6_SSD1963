#ifndef INC_FLASH25Q_H_
#define INC_FLASH25Q_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_rcc.h"

#include <stdio.h>

#define	FLASH25Q_SPI 		hspi1
#define	FLASH25Q_NSS_SOFT	1
#define	FLASH25Q_NSS_PORT 	GPIOB
#define	FLASH25Q_NSS_PIN 	GPIO_PIN_0

#define	W25_ENABLE_RESET	0x66
#define	W25_RESET			0x99
#define	W25_READ			0x03
#define	W25_GET_JEDEC_ID	0x9f

#define cs_set() HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET)
#define cs_reset() HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET)

typedef struct
{
  uint16_t  PageSize;
  uint32_t  PageCount;
  uint32_t  SectorSize;
  uint32_t  SectorCount;
  uint32_t  BlockSize;
  uint32_t  BlockCount;
  uint32_t  NumKB;
  uint8_t   SR1;
  uint8_t   SR2;
  uint8_t   SR3;
}w25_info_t;


void W25_Reset (void);
void W25_Read_Data(uint32_t addr, uint8_t* data, uint32_t sz);
uint32_t W25_Read_ID(void);
void W25_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_FLASH25Q_H_ */
