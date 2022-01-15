#include "w25q.h"

extern SPI_HandleTypeDef W25Q_SPI;

w25_info_t  w25_info;
char str1[30];
uint8_t buf[10];

void W25Q_Reset (void) {
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
	buf[0] = W25Q_ENABLE_RESET;
	buf[1] = W25Q_RESET;
	HAL_SPI_Transmit (&W25Q_SPI, buf, 2, 1000);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
}

void W25Q_Read_Info(char* str_info) {
	unsigned int id = W25Q_Read_ID();
	sprintf(str_info,"ID:0x%X\n",id);
	id &= 0x0000FFFF;
	w25_info.high_cap = 0;
	switch(id) {
	case 0x401A:
		w25_info.high_cap=1;
		w25_info.BlockCount=1024;
		strcat(str_info,"W25Q Chip: w25q512\n");
		break;
	case 0x4019:
		w25_info.high_cap=1;
		w25_info.BlockCount=512;
		strcat(str_info,"W25Q Chip: w25q256\n");
		break;
	case 0x4018:
		w25_info.BlockCount=256;
		strcat(str_info,"W25Q Chip: w25q128\n");
		break;
	case 0x4017:
		w25_info.BlockCount=128;
		strcat(str_info,"W25Q Chip: w25q64\n");
		break;
	case 0x4016:
		w25_info.BlockCount=64;
		strcat(str_info,"W25Q Chip: w25q32\n");
		break;
	case 0x4015:
		w25_info.BlockCount=32;
		strcat(str_info,"W25Q Chip: w25q16\n");
		break;
	case 0x4014:
		w25_info.BlockCount=16;
		strcat(str_info,"W25Q Chip: w25q80\n");
		break;
	case 0x4013:
		w25_info.BlockCount=8;
		strcat(str_info,"W25Q Chip: w25q40\n");
		break;
	case 0x4012:
		w25_info.BlockCount=4;
		strcat(str_info,"W25Q Chip: w25q20\n");
		break;
	case 0x4011:
		w25_info.BlockCount=2;
		strcat(str_info,"W25Q Chip: w25q10\n");
		break;
	default:
		strcat(str_info,"W25Q Unknown ID\n");
		return;
	}
	w25_info.PageSize=256;
	w25_info.SectorSize=0x1000;
	w25_info.SectorCount=w25_info.BlockCount*16;
	w25_info.PageCount=(w25_info.SectorCount*w25_info.SectorSize)/w25_info.PageSize;
	w25_info.BlockSize=w25_info.SectorSize*16;
	w25_info.NumKB=(w25_info.SectorCount*w25_info.SectorSize)/1024;
	sprintf(str1,"Page Size: %d Bytes\n",(unsigned int)w25_info.PageSize);
	strcat(str_info,str1);
	sprintf(str1,"Page Count: %u\n",(unsigned int)w25_info.PageCount);
	strcat(str_info,str1);
	sprintf(str1,"Sector Size: %u Bytes\n",(unsigned int)w25_info.SectorSize);
	strcat(str_info,str1);
	sprintf(str1,"Sector Count: %u\r\n",(unsigned int)w25_info.SectorCount);
	strcat(str_info,str1);
	sprintf(str1,"Block Size: %u Bytes\n",(unsigned int)w25_info.BlockSize);
	strcat(str_info,str1);
	sprintf(str1,"Block Count: %u\n",(unsigned int)w25_info.BlockCount);
	strcat(str_info,str1);
	sprintf(str1,"Capacity: %u KB\n",(unsigned int)w25_info.NumKB);
	strcat(str_info,str1);
}

void W25Q_Write_Enable(void) {
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
	buf[0] = W25Q_WRITE_ENABLE;
	HAL_SPI_Transmit (&W25Q_SPI, buf, 1, 1000);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
}

void W25Q_Write_Disable(void) {
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
	buf[0] = W25Q_WRITE_DISABLE;
	HAL_SPI_Transmit (&W25Q_SPI, buf, 1, 1000);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
}

void W25Q_Set_Block_Protect(uint8_t val) {
	buf[0] = 0x50;
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit (&W25Q_SPI, buf, 1, 1000);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
	buf[0] = W25Q_WRITE_STATUS_1;
	buf[1] = ((val & 0x0F) << 2);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit (&W25Q_SPI, buf, 2, 1000);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
}

void W25Q_Wait_Write_End(void) {
	HAL_Delay(1);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
	buf[0] = W25Q_READ_STATUS_1;
	HAL_SPI_Transmit (&W25Q_SPI, buf, 1, 1000);
	do {
		HAL_SPI_Receive (&W25Q_SPI, buf, 1, 1000);
		w25_info.StatusRegister1 = buf[0];
		HAL_Delay(1);
	}
	while((w25_info.StatusRegister1 & 0x01) == 0x01);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
}

void W25Q_Erase_Sector(uint32_t addr) {
	W25Q_Wait_Write_End();
	W25Q_Set_Block_Protect(0x00);
	addr = addr * w25_info.SectorSize;
	W25Q_Write_Enable();
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
	buf[0] = W25Q_SECTOR_ERASE;
	if(w25_info.high_cap)
	{
		buf[1] = (addr >> 24) & 0xFF;
		buf[2] = (addr >> 16) & 0xFF;
		buf[3] = (addr >> 8) & 0xFF;
		buf[4] = addr & 0xFF;
		HAL_SPI_Transmit (&W25Q_SPI, buf, 5, 1000);
	}
	else
	{
		buf[1] = (addr >> 16) & 0xFF;
		buf[2] = (addr >> 8) & 0xFF;
		buf[3] = addr & 0xFF;
		HAL_SPI_Transmit (&W25Q_SPI, buf, 4, 1000);
	}
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
	W25Q_Wait_Write_End();
	W25Q_Write_Disable();
	W25Q_Set_Block_Protect(0x0F);
}

void W25Q_Erase_Block(uint32_t addr) {
	W25Q_Wait_Write_End();
	W25Q_Set_Block_Protect(0x00);
	addr = addr * w25_info.BlockSize;
	W25Q_Write_Enable();
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
	buf[0] = W25Q_BLOCK_ERASE;
	if(w25_info.high_cap)
	{
		buf[1] = (addr >> 24) & 0xFF;
		buf[2] = (addr >> 16) & 0xFF;
		buf[3] = (addr >> 8) & 0xFF;
		buf[4] = addr & 0xFF;
		HAL_SPI_Transmit (&W25Q_SPI, buf, 5, 1000);
	}
	else
	{
		buf[1] = (addr >> 16) & 0xFF;
		buf[2] = (addr >> 8) & 0xFF;
		buf[3] = addr & 0xFF;
		HAL_SPI_Transmit (&W25Q_SPI, buf, 4, 1000);
	}
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
	W25Q_Wait_Write_End();
	W25Q_Write_Disable();
	W25Q_Set_Block_Protect(0x0F);
}

void W25Q_Erase_Chip(void) {
	W25Q_Wait_Write_End();
	W25Q_Set_Block_Protect(0x00);
	W25Q_Write_Enable();
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
	buf[0] = W25Q_CHIP_ERASE;
	HAL_SPI_Transmit (&W25Q_SPI, buf, 1, 1000);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
	W25Q_Wait_Write_End();
	W25Q_Write_Disable();
	W25Q_Set_Block_Protect(0x0F);
}

void W25Q_Write_Data(uint32_t addr, uint8_t* data, uint32_t sz) {
	W25Q_Wait_Write_End();
	W25Q_Set_Block_Protect(0x00);
	W25Q_Write_Enable();
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
	buf[0] = W25Q_PAGE_PROGRAMM;
	if(w25_info.high_cap)
	{
		buf[1] = (addr >> 24) & 0xFF;
		buf[2] = (addr >> 16) & 0xFF;
		buf[3] = (addr >> 8) & 0xFF;
		buf[4] = addr & 0xFF;
		HAL_SPI_Transmit (&W25Q_SPI, buf, 5, 1000);
	}
	else
	{
		buf[1] = (addr >> 16) & 0xFF;
		buf[2] = (addr >> 8) & 0xFF;
		buf[3] = addr & 0xFF;
		HAL_SPI_Transmit (&W25Q_SPI, buf, 4, 1000);
	}
	HAL_SPI_Transmit (&W25Q_SPI, data, sz, 1000);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
	W25Q_Wait_Write_End();
	W25Q_Write_Disable();
	W25Q_Set_Block_Protect(0x0F);
}

void W25Q_Write_Page(uint8_t* data, uint32_t page_addr, uint32_t offset, uint32_t sz) {
	if(sz > w25_info.PageSize)
		sz=w25_info.PageSize;
	if((offset+sz) > w25_info.PageSize)
		sz = w25_info.PageSize - offset;
	page_addr = page_addr * w25_info.PageSize + offset;

	W25Q_Wait_Write_End();
	W25Q_Set_Block_Protect(0x00);
	W25Q_Write_Enable();
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
	buf[0] = W25Q_PAGE_PROGRAMM;
	if(w25_info.high_cap)
	{
		buf[1] = (page_addr >> 24) & 0xFF;
		buf[2] = (page_addr >> 16) & 0xFF;
		buf[3] = (page_addr >> 8) & 0xFF;
		buf[4] = page_addr & 0xFF;
		HAL_SPI_Transmit (&W25Q_SPI, buf, 5, 1000);
	}
	else
	{
		buf[1] = (page_addr >> 16) & 0xFF;
		buf[2] = (page_addr >> 8) & 0xFF;
		buf[3] = page_addr & 0xFF;
		HAL_SPI_Transmit (&W25Q_SPI, buf, 4, 1000);
	}
	HAL_SPI_Transmit (&W25Q_SPI, data, sz, 1000);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
	W25Q_Wait_Write_End();
	W25Q_Write_Disable();
	W25Q_Set_Block_Protect(0x0F);
}

void W25Q_Read_Data(uint32_t addr, uint8_t* data, uint32_t sz) {
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
	buf[0] = W25Q_READ;
	buf[1] = (addr >> 16) & 0xFF;
	buf[2] = (addr >> 8) & 0xFF;
	buf[3] = addr & 0xFF;
	HAL_SPI_Transmit (&W25Q_SPI, buf, 4, 1000);
	HAL_SPI_Receive (&W25Q_SPI, data, sz, 1000);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
}

void W25Q_Read_Page(uint8_t* data, uint32_t page_addr, uint32_t offset, uint32_t sz) {
	if(sz > w25_info.PageSize)
		sz=w25_info.PageSize;
	if((offset+sz) > w25_info.PageSize)
		sz = w25_info.PageSize - offset;
	page_addr = page_addr * w25_info.PageSize + offset;
	buf[0] = W25Q_FAST_READ;
	if(w25_info.high_cap)
	{
		buf[1] = (page_addr >> 24) & 0xFF;
		buf[2] = (page_addr >> 16) & 0xFF;
		buf[3] = (page_addr >> 8) & 0xFF;
		buf[4] = page_addr & 0xFF;
		buf[5] = 0;
		HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
		HAL_SPI_Transmit (&W25Q_SPI, buf, 6, 1000);
	}
	else
	{
		buf[1] = (page_addr >> 16) & 0xFF;
		buf[2] = (page_addr >> 8) & 0xFF;
		buf[3] = page_addr & 0xFF;
		buf[4] = 0;
		HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
		HAL_SPI_Transmit (&W25Q_SPI, buf, 5, 1000);
	}
	HAL_SPI_Receive (&W25Q_SPI, data, sz, 1000);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
}

uint32_t W25Q_Read_ID(void) {
	uint8_t dt[4];
	buf[0] = W25Q_GET_JEDEC_ID;
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit (&W25Q_SPI, buf, 1, 1000);
	HAL_SPI_Receive (&W25Q_SPI, dt, 3, 1000);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
	return (dt[0] << 16) | (dt[1] << 8) | dt[2];
}

void W25Q_Init(void) {
	unsigned int id = W25Q_Read_ID();
	HAL_Delay(100);
	W25Q_Reset();
	HAL_Delay(100);
	id &= 0x0000ffff;
	w25_info.high_cap = 0;
	switch(id)
	{
	case 0x401A:
		w25_info.high_cap=1;
		w25_info.BlockCount=1024;
		break;
	case 0x4019:
		w25_info.high_cap=1;
		w25_info.BlockCount=512;
		break;
	case 0x4018:
		w25_info.BlockCount=256;
		break;
	case 0x4017:
		w25_info.BlockCount=128;
		break;
	case 0x4016:
		w25_info.BlockCount=64;
		break;
	case 0x4015:
		w25_info.BlockCount=32;
		break;
	case 0x4014:
		w25_info.BlockCount=16;
		break;
	case 0x4013:
		w25_info.BlockCount=8;
		break;
	case 0x4012:
		w25_info.BlockCount=4;
		break;
	case 0x4011:
		w25_info.BlockCount=2;
		break;
	default:
		return;
	}
	w25_info.PageSize=256;
	w25_info.SectorSize=0x1000;
	w25_info.SectorCount=w25_info.BlockCount*16;
	w25_info.PageCount=(w25_info.SectorCount*w25_info.SectorSize)/w25_info.PageSize;
	w25_info.BlockSize=w25_info.SectorSize*16;
	w25_info.NumKB=(w25_info.SectorCount*w25_info.SectorSize)/1024;
}
