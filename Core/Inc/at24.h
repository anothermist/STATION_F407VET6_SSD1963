#ifndef    INC_AT24_H
#define    INC_AT24_H

#include "stm32f4xx_hal.h"
#include <stdio.h>

#define AT24XX_I2C				hi2c1
#define AT24XX_ADDRESS			0x57 // 0x50 0x57
#define AT24XX_SIZE_KBIT		32

uint8_t AT24XX_IsConnected(void);

uint8_t AT24XX_Save(uint16_t address, void *data, size_t size_of_data);

uint8_t AT24XX_Load(uint16_t address, void *data, size_t size_of_data);

uint8_t AT24XX_Read(uint16_t address);

void AT24XX_Write(uint16_t address, uint8_t val);
void AT24XX_Update(uint16_t address, uint8_t val);
void I2C_Error(char *er, uint32_t status);

void AT24XX_String_Write(uint16_t addr, char *wstr, uint16_t len);
void AT24XX_String_Read(uint16_t addr, char *rstr, uint16_t len);
void AT24XX_mem_single_block(uint16_t addr, char *wstr, uint16_t len);

#endif /* INC_AT24_H_ */
