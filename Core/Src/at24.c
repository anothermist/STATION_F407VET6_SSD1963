#include "at24.h"

extern I2C_HandleTypeDef AT24XX_I2C;

uint8_t AT24XX_IsConnected(void) {
	if (HAL_I2C_IsDeviceReady(&AT24XX_I2C, AT24XX_ADDRESS << 1, 1, HAL_MAX_DELAY) == HAL_OK)
		return 1;
	else
		return 0;
}

uint8_t AT24XX_Save(uint16_t address, void *data, size_t size_of_data) {
#if ((AT24XX_SIZE_KBIT == 1) || (AT24XX_SIZE_KBIT == 2))
	if(size_of_data > 8)
		return 0;
#endif
#if ((AT24XX_SIZE_KBIT == 4) || (AT24XX_SIZE_KBIT == 8) || (AT24XX_SIZE_KBIT == 16))
	if(size_of_data > 16)
		return 0;
#endif
#if ((AT24XX_SIZE_KBIT == 32) || (AT24XX_SIZE_KBIT == 64) || (AT24XX_SIZE_KBIT == 128))
	if(size_of_data > 32)
		return 0;
#endif

#if ((AT24XX_SIZE_KBIT == 1) || (AT24XX_SIZE_KBIT == 2))
	if(HAL_I2C_Mem_Write(&EEPROM24XX_I2C, AT24XX_ADDRESS<<1, Address, I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, size_of_data, HAL_MAX_DELAY) == HAL_OK)
#else
		if (HAL_I2C_Mem_Write(&AT24XX_I2C, AT24XX_ADDRESS << 1, address, I2C_MEMADD_SIZE_16BIT, (uint8_t *) data, size_of_data, HAL_MAX_DELAY) == HAL_OK)
#endif
{

			HAL_Delay(5);
			return 1;
} else
	return 0;
}

uint8_t AT24XX_Load(uint16_t address, void *data, size_t size_of_data) {
#if ((AT24XX_SIZE_KBIT == 1) || (AT24XX_SIZE_KBIT == 2))
	if(HAL_I2C_Mem_Read(&EEPROM24XX_I2C, AT24XX_ADDRESS<<1, Address, I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, size_of_data, HAL_MAX_DELAY) == HAL_OK)
#else
		if (HAL_I2C_Mem_Read(&AT24XX_I2C, AT24XX_ADDRESS << 1, address, I2C_MEMADD_SIZE_16BIT, (uint8_t *) data, size_of_data, HAL_MAX_DELAY) == HAL_OK)
#endif
{
	return 1;
} else
	return 0;
}

uint8_t AT24XX_Read(uint16_t address) {
	uint16_t dt[1] = {0};
	AT24XX_Load(address, dt, 1);
	return dt[0];
}

void AT24XX_Write(uint16_t address, uint8_t val) {
	uint8_t save[] = {val};
	AT24XX_Save(address, save, 1);
}

void AT24XX_Update(uint16_t address, uint8_t val) {
	if (AT24XX_Read(address) != val) AT24XX_Write(address, val);
}





void AT24XX_String_Write(uint16_t addr, char *wstr, uint16_t len) {
	uint16_t cycle = len / AT24XX_SIZE_KBIT;
	uint32_t status = 0;

	for(uint16_t i = 0; i < cycle + 1; i++) {
		if(len > AT24XX_SIZE_KBIT) {
			status = HAL_I2C_Mem_Write(&hi2c1, AT24XX_ADDRESS, addr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)wstr, AT24XX_SIZE_KBIT, HAL_MAX_DELAY);
			wstr = wstr + AT24XX_SIZE_KBIT;
			len = len - AT24XX_SIZE_KBIT;
			addr = addr + AT24XX_SIZE_KBIT;
		} else {
			status = HAL_I2C_Mem_Write(&AT24XX_I2C, AT24XX_ADDRESS, addr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)wstr, len, HAL_MAX_DELAY);
		}

		if(status != HAL_OK) {
			char str[32] = {0,};
			snprintf(str, 32, "HAL_I2C_Mem_Write");
			I2C_Error(str, status);
		}

		for(uint8_t i = 0; i < 100; i++) {
			status = HAL_I2C_IsDeviceReady(&AT24XX_I2C, AT24XX_ADDRESS, 1, 100);

			if(status == HAL_OK) break;

			if(i > 99) {
				char str[32] = {0,};
				snprintf(str, 32, "HAL_I2C_IsDeviceReady");
				I2C_Error(str, status);
			}
		}
	}
}

void AT24XX_String_Read(uint16_t addr, char *rstr, uint16_t len) {
	uint32_t status = HAL_I2C_Mem_Read(&AT24XX_I2C, AT24XX_ADDRESS, addr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)rstr, len, HAL_MAX_DELAY);

	if(status != HAL_OK)
	{
		char str[32] = {0,};
		snprintf(str, 32, "HAL_I2C_Mem_Read");
		I2C_Error(str, status);
	}
}

void AT24XX_mem_single_block(uint16_t addr, char *wstr, uint16_t len) {
	uint16_t status = HAL_I2C_Mem_Write(&AT24XX_I2C, AT24XX_ADDRESS, addr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)wstr, len, HAL_MAX_DELAY);

	if(status != HAL_OK)
	{
		char str[32] = {0,};
		snprintf(str, 32, "HAL_I2C_Mem_Read");
		I2C_Error(str, status);
	}
}

