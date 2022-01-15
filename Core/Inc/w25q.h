#ifndef INC_W25Q_H_
#define INC_W25Q_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_rcc.h"

#include <stdio.h>
#include <string.h>

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
  uint8_t   high_cap;
  uint8_t   StatusRegister1;
  uint8_t   StatusRegister2;
  uint8_t   StatusRegister3;
}w25_info_t;

#define	W25Q_SPI 			hspi1
#define	W25Q_NSS_SOFT		1
#define	W25Q_NSS_PORT 		GPIOB
#define	W25Q_NSS_PIN 		GPIO_PIN_0

#define    W25Q_WRITE_DISABLE	0x04
#define    W25Q_WRITE_ENABLE	0x06
#define    W25Q_SECTOR_ERASE	0x20
#define    W25Q_BLOCK_ERASE		0xD8
#define    W25Q_CHIP_ERASE		0xC7
#define    W25Q_ENABLE_RESET	0x66
#define    W25Q_RESET			0x99
#define    W25Q_READ			0x03
#define    W25Q_PAGE_PROGRAMM	0x02
#define    W25Q_FAST_READ		0x0B
#define    W25Q_GET_JEDEC_ID	0x9F
#define    W25Q_READ_STATUS_1	0x05
#define    W25Q_READ_STATUS_2	0x35
#define    W25Q_READ_STATUS_3	0x15
#define    W25Q_WRITE_STATUS_1	0x01
#define    W25Q_WRITE_STATUS_2	0x31
#define    W25Q_WRITE_STATUS_3	0x11


void W25Q_Reset(void);
void W25Q_Read_Info(char* str_info);
void W25Q_Write_Enable(void);
void W25Q_Write_Disable(void);
void W25Q_Set_Block_Protect(uint8_t val);
void W25Q_Wait_Write_End(void);
void W25Q_Erase_Sector(uint32_t addr);
void W25Q_Erase_Block(uint32_t addr);
void W25Q_Erase_Chip(void);
void W25Q_Write_Data(uint32_t addr, uint8_t* data, uint32_t sz);
void W25Q_Write_Page(uint8_t* data, uint32_t page_addr, uint32_t offset, uint32_t sz);
void W25Q_Read_Data(uint32_t addr, uint8_t* data, uint32_t sz);
void W25Q_Read_Page(uint8_t* data, uint32_t page_addr, uint32_t offset, uint32_t sz);
uint32_t W25Q_Read_ID(void);
void W25Q_Init(void);

#endif /* INC_W25Q_H_ */
